//! Offline pipeline replay — run a captured raw-pose CSV through the *real*
//! production pipeline and write the filtered result.
//!
//! Because it calls [`build_default_pipeline`] + [`apply_config`] (the same
//! functions the engine run loop uses), what you tune here is exactly what
//! ships — there is no re-implementation to drift out of sync.
//!
//! Usage:
//! ```text
//! cargo run -p headtrack-core --example replay -- \
//!     --in /tmp/headtrack_raw_pose.csv \
//!     --out /tmp/headtrack_filtered.csv \
//!     [--profile ~/.config/headtrack-rs/active.toml]
//! ```
//!
//! Input CSV must have a header naming at least the columns
//! `yaw,pitch,roll,x,y,z`.  A `timestamp_us` column (and/or a `dt_s` column)
//! drives per-frame `dt`; absent both, a nominal 30 Hz step is assumed.
//! Output columns: `timestamp_us,dt_s,yaw,pitch,roll,x,y,z`.

use headtrack_core::config::ProfileConfig;
use headtrack_core::pipeline::{apply_config, build_default_pipeline, StandoffTracker};
use headtrack_core::pose::Pose;
use std::io::{BufRead, BufWriter, Write};

const NOMINAL_DT: f32 = 1.0 / 30.0;

fn main() -> std::io::Result<()> {
    let mut in_path = String::new();
    let mut out_path = String::new();
    let mut profile_path: Option<String> = None;
    let mut distance_scale = true;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--in" => in_path = args.next().unwrap_or_default(),
            "--out" => out_path = args.next().unwrap_or_default(),
            "--profile" => profile_path = args.next(),
            // Distance normalisation is part of the shipped pipeline, so replay
            // applies it by default or it would stop matching the engine — the
            // one thing this tool exists to guarantee. Off for A/B only.
            "--no-distance-scale" => distance_scale = false,
            "-h" | "--help" => {
                eprintln!(
                    "usage: replay --in <raw.csv> --out <filtered.csv> [--profile <active.toml>] \
[--no-distance-scale]"
                );
                return Ok(());
            }
            other => {
                eprintln!("warning: ignoring unknown argument '{other}'");
            }
        }
    }
    if in_path.is_empty() || out_path.is_empty() {
        eprintln!("error: --in and --out are required (see --help)");
        std::process::exit(2);
    }

    // Build the exact production pipeline, then mirror the engine's profile
    // branch precisely:
    //   * --profile given  → load + apply_config (engine "saved profile" path)
    //   * no --profile      → leave untouched     (engine "no saved profile" path)
    // These are NOT identical: the built-in default profile differs from the
    // stage constructor defaults (e.g. cross-axis z_to_y), so being explicit
    // about which baseline we replay against matters for fidelity.
    let mut pipeline = build_default_pipeline();
    // Mirrors the engine run loop: the same tracker type, fed the same raw
    // depth, pushing the same scale. Sharing the type is deliberate — a
    // reimplementation here would silently drift from what ships.
    let mut standoff = StandoffTracker::new();
    match profile_path.as_deref() {
        Some(p) => match ProfileConfig::load(std::path::Path::new(p)) {
            Ok(cfg) => {
                apply_config(&mut pipeline, &cfg);
                eprintln!("replay: applied profile '{}' from {p}", cfg.name);
            }
            Err(e) => eprintln!(
                "warning: could not load profile '{p}' ({e}); using built-in defaults"
            ),
        },
        None => eprintln!("replay: no --profile — using built-in default pipeline"),
    }

    let file = std::fs::File::open(&in_path)?;
    let mut reader = std::io::BufReader::new(file);

    // Parse the header → column-name → index map.
    let mut header = String::new();
    reader.read_line(&mut header)?;
    let cols: Vec<&str> = header.trim().split(',').collect();
    let idx = |name: &str| cols.iter().position(|c| *c == name);
    let (ci_yaw, ci_pitch, ci_roll, ci_x, ci_y, ci_z) = (
        idx("yaw").expect("missing 'yaw' column"),
        idx("pitch").expect("missing 'pitch' column"),
        idx("roll").expect("missing 'roll' column"),
        idx("x").expect("missing 'x' column"),
        idx("y").expect("missing 'y' column"),
        idx("z").expect("missing 'z' column"),
    );
    let ci_ts = idx("timestamp_us");
    let ci_dt = idx("dt_s");

    let out = std::fs::File::create(&out_path)?;
    let mut w = BufWriter::new(out);
    writeln!(w, "timestamp_us,dt_s,yaw,pitch,roll,x,y,z")?;

    let mut prev_ts: Option<u64> = None;
    let mut n_rows = 0u64;
    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();
        // `#` lines are event markers (recenter, param changes, tick marks)
        // written by the raw-pose recorder — metadata, not samples.
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let get = |i: usize| f.get(i).and_then(|s| s.trim().parse::<f32>().ok()).unwrap_or(0.0);

        let ts = ci_ts
            .and_then(|i| f.get(i))
            .and_then(|s| s.trim().parse::<u64>().ok())
            .unwrap_or_else(|| prev_ts.map(|p| p + 33_333).unwrap_or(0));

        // dt priority: explicit dt_s column → timestamp delta → nominal.
        let dt = ci_dt
            .map(get)
            .filter(|d| *d > 0.0)
            .or_else(|| prev_ts.map(|p| (ts.saturating_sub(p)) as f32 / 1_000_000.0))
            .filter(|d| *d > 0.0)
            .unwrap_or(NOMINAL_DT)
            .max(0.001);
        prev_ts = Some(ts);

        let pose = Pose {
            yaw: get(ci_yaw),
            pitch: get(ci_pitch),
            roll: get(ci_roll),
            x: get(ci_x),
            y: get(ci_y),
            z: get(ci_z),
            timestamp_us: ts,
        };
        if distance_scale {
            if let Some(scale) = standoff.observe(pose.z, dt) {
                pipeline.set_distance_scale(scale);
            }
        }
        let p = pipeline.process(pose, dt);
        writeln!(
            w,
            "{},{:.6},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
            ts, dt, p.yaw, p.pitch, p.roll, p.x, p.y, p.z
        )?;
        n_rows += 1;
    }
    w.flush()?;
    eprintln!("replay: {n_rows} frames → {out_path}");
    Ok(())
}
