//! headtrack-tester — live axis visualiser, recenter tester, and guided axis mapper
//!
//! Run while the daemon is running:
//!   cargo run -p headtrack-tester
//!
//! Keys:
//!   r = recenter (send to daemon)
//!   c = clear min/max stats
//!   f = freeze / unfreeze display
//!   g = start guided axis-identification test (4 movements, SPACE to begin each)
//!   q = quit

use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode},
};
use headtrack_core::Pose;
use std::{
    io::{self, Write},
    time::{Duration, Instant},
};
use tokio::sync::mpsc;

// ── Display parameters ────────────────────────────────────────────────────────

const ROT_MAX: f32 = 30.0;    // ±30° full-scale for rotation bars
const TRANS_MAX: f32 = 60.0;  // ±60mm full-scale for translation bars
const BAR_HALF: usize = 20;   // half-width of bar (total = 2*BAR_HALF+1)

const NAMES: [&str; 6]     = ["Yaw", "Pitch", "Roll", "X", "Y", "Z"];
const UNITS: [&str; 6]     = ["°", "°", "°", "mm", "mm", "mm"];
const MAXES: [f32; 6]      = [ROT_MAX, ROT_MAX, ROT_MAX, TRANS_MAX, TRANS_MAX, TRANS_MAX];
const HINTS: [&str; 6]     = [
    "turn right = +",
    "tilt up = +",
    "tilt right = +",
    "move right = +",
    "move up = +",
    "lean toward screen = +",
];
const POS_MOVE: [&str; 6] = [
    "turn RIGHT",
    "tilt UP (nose toward ceiling)",
    "tilt RIGHT (right ear down)",
    "slide RIGHT",
    "slide UP",
    "lean TOWARD screen",
];

// ── Stats ─────────────────────────────────────────────────────────────────────

struct Stats {
    pose: Option<Pose>,
    min: [f32; 6],
    max: [f32; 6],
    frames: u64,
    fps: f32,
    fps_frames: u64,
    fps_t: Instant,
}

impl Stats {
    fn new() -> Self {
        Self {
            pose: None,
            min: [f32::INFINITY; 6],
            max: [f32::NEG_INFINITY; 6],
            frames: 0,
            fps: 0.0,
            fps_frames: 0,
            fps_t: Instant::now(),
        }
    }

    fn update(&mut self, p: Pose) {
        let vals = pose_vals(&p);
        for (i, &v) in vals.iter().enumerate() {
            if v < self.min[i] { self.min[i] = v; }
            if v > self.max[i] { self.max[i] = v; }
        }
        self.frames += 1;
        self.fps_frames += 1;
        let elapsed = self.fps_t.elapsed();
        if elapsed >= Duration::from_millis(500) {
            self.fps = self.fps_frames as f32 / elapsed.as_secs_f32();
            self.fps_frames = 0;
            self.fps_t = Instant::now();
        }
        self.pose = Some(p);
    }

    fn clear_minmax(&mut self) {
        self.min = [f32::INFINITY; 6];
        self.max = [f32::NEG_INFINITY; 6];
    }
}

fn pose_vals(p: &Pose) -> [f32; 6] {
    [p.yaw, p.pitch, p.roll, p.x, p.y, p.z]
}

// ── Bar renderer ──────────────────────────────────────────────────────────────

fn bar(value: f32, max: f32) -> String {
    let total = BAR_HALF * 2 + 1;
    let mut buf = vec![b' '; total];
    buf[BAR_HALF] = b'|';

    let clamped = value.clamp(-max, max);
    let pos = ((clamped / max) * BAR_HALF as f32).round() as isize + BAR_HALF as isize;
    let pos = pos.clamp(0, (total - 1) as isize) as usize;

    if pos > BAR_HALF {
        for b in &mut buf[(BAR_HALF + 1)..=pos] { *b = b'-'; }
        buf[pos] = b'*';
    } else if pos < BAR_HALF {
        for b in &mut buf[pos..BAR_HALF] { *b = b'-'; }
        buf[pos] = b'*';
    } else {
        buf[BAR_HALF] = b'*';
    }
    format!("[{}]", std::str::from_utf8(&buf).unwrap())
}

fn dominant_idx(vals: &[f32; 6]) -> usize {
    (0..6)
        .max_by(|&a, &b| {
            (vals[a].abs() / MAXES[a])
                .partial_cmp(&(vals[b].abs() / MAXES[b]))
                .unwrap()
        })
        .unwrap()
}

// ── Guided test ───────────────────────────────────────────────────────────────

const GUIDE_STEPS: usize = 4;

/// Per-step: (instruction, expected_axis_idx, expected_sign_positive, axis_mask)
/// axis_mask: which axes to consider for dominance (true = include)
const GUIDE_INSTRUCTIONS: [(&str, usize, bool, [bool; 6]); GUIDE_STEPS] = [
    ("Turn head RIGHT (like saying 'no')",
        0, true,  [true,  true,  true,  false, false, false]),
    ("Tilt head UP (nose toward ceiling)",
        1, true,  [true,  true,  true,  false, false, false]),
    ("Tilt head RIGHT (right ear toward shoulder)",
        2, true,  [true,  true,  true,  false, false, false]),
    ("Lean toward screen (move head forward)",
        5, true,  [false, false, false, true,  true,  true]),
];

enum GuideState {
    Waiting(usize),
    Recording {
        step: usize,
        start: Instant,
        min: [f32; 6],
        max: [f32; 6],
    },
    Done,
}

struct GuideResult {
    step: usize,
    dominant: usize,
    positive: bool,
    /// Peak absolute values for all 6 axes during the recording window.
    peaks: [f32; 6],
}

impl GuideState {
    fn new() -> Self {
        GuideState::Waiting(0)
    }

    /// Feed a pose; returns `Some(result)` when a recording step completes.
    fn feed(&mut self, p: Pose) -> Option<GuideResult> {
        if let GuideState::Recording { start, min, max, step } = self {
            let vals = pose_vals(&p);
            for (i, &v) in vals.iter().enumerate() {
                if v < min[i] { min[i] = v; }
                if v > max[i] { max[i] = v; }
            }
            if start.elapsed() >= Duration::from_secs(3) {
                let step = *step;
                let (min, max) = (*min, *max);
                let (_, _, _, mask) = GUIDE_INSTRUCTIONS[step];
                let ranges: [f32; 6] = std::array::from_fn(|i| (max[i] - min[i]) / MAXES[i]);
                // Only consider axes in the mask for this step
                let dom = (0..6)
                    .filter(|&i| mask[i])
                    .max_by(|&a, &b| ranges[a].partial_cmp(&ranges[b]).unwrap())
                    .unwrap_or(0);
                let positive = max[dom].abs() >= min[dom].abs();
                // Peak absolute value for display
                let peaks: [f32; 6] = std::array::from_fn(|i| max[i].abs().max(min[i].abs()));
                *self = if step + 1 < GUIDE_STEPS {
                    GuideState::Waiting(step + 1)
                } else {
                    GuideState::Done
                };
                return Some(GuideResult { step, dominant: dom, positive, peaks });
            }
        }
        None
    }

    fn start_recording(&mut self) {
        if let GuideState::Waiting(step) = *self {
            *self = GuideState::Recording {
                step,
                start: Instant::now(),
                min: [f32::INFINITY; 6],
                max: [f32::NEG_INFINITY; 6],
            };
        }
    }
}

// ── Rendering ─────────────────────────────────────────────────────────────────

fn render_live(stats: &Stats, connected: bool, frozen: bool) {
    let mut out = String::with_capacity(2048);
    out.push_str("\x1B[H");

    let freeze_indicator = if frozen { "  \x1B[7m FROZEN \x1B[0m" } else { "" };
    out.push_str(&format!(
        "  headtrack-rs pose tester{}\x1B[K\n",
        freeze_indicator
    ));
    out.push_str("  keys:  r=recenter  c=clear  f=freeze  g=guided test  q=quit\x1B[K\n");
    out.push_str("  \x1B[K\n");

    // Spotlight
    if let Some(p) = stats.pose {
        let vals = pose_vals(&p);
        let dom = dominant_idx(&vals);
        let sign = if vals[dom] >= 0.0 { "+" } else { "-" };
        let move_str = if vals[dom] >= 0.0 {
            POS_MOVE[dom].to_string()
        } else {
            POS_MOVE[dom]
                .replace("RIGHT", "LEFT").replace("UP", "DOWN")
                .replace("right", "left")
                .replace("toward", "away from")
        };
        out.push_str(&format!(
            "  DOMINANT: \x1B[1m{} {:>+7.2}{}\x1B[0m  ({} = {})\x1B[K\n",
            NAMES[dom], vals[dom], UNITS[dom], sign, move_str
        ));
    } else {
        out.push_str("  DOMINANT: waiting for data…\x1B[K\n");
    }
    out.push_str("  \x1B[K\n");

    // Table header
    out.push_str(&format!(
        "  {:<6}  {:>8}  {:<width$}  {:>6} {:>6}   hint\x1B[K\n",
        "axis", "value", "  -max←0→+max  ",
        "min", "max",
        width = BAR_HALF * 2 + 3,
    ));
    out.push_str("  ──────────────────────────────────────────────────────────────────\x1B[K\n");

    if let Some(p) = stats.pose {
        let vals = pose_vals(&p);
        let dom = dominant_idx(&vals);
        for i in 0..6 {
            let b = bar(vals[i], MAXES[i]);
            let min_s = if stats.min[i].is_infinite() { "   ---".into() } else { format!("{:+6.1}", stats.min[i]) };
            let max_s = if stats.max[i].is_infinite() { "   ---".into() } else { format!("{:+6.1}", stats.max[i]) };
            let (bold, rst) = if i == dom { ("\x1B[1m", "\x1B[0m") } else { ("", "") };
            out.push_str(&format!(
                "  {}{:<6}  {:>+7.2}{:<2}  {}  {}  {}   {}{}\x1B[K\n",
                bold, NAMES[i], vals[i], UNITS[i], b, min_s, max_s, HINTS[i], rst
            ));
        }
    } else {
        for _ in 0..6 { out.push_str("  ---\x1B[K\n"); }
    }

    out.push_str("  \x1B[K\n");
    let status = if connected { "connected" } else { "DISCONNECTED — retrying…" };
    out.push_str(&format!(
        "  Status: {}   frames: {}   rate: {:.0} Hz\x1B[K\n",
        status, stats.frames, stats.fps
    ));
    out.push_str("\x1B[J");
    print_out(&out);
}

fn render_guided(guide: &GuideState, results: &[GuideResult]) {
    let mut out = String::with_capacity(2048);
    out.push_str("\x1B[H");
    out.push_str("  headtrack-rs GUIDED AXIS TEST\x1B[K\n");
    out.push_str("  ESC or q to cancel\x1B[K\n");
    out.push_str("  \x1B[K\n");

    // Past results
    for r in results {
        let (inst, expected_idx, expected_pos, _) = GUIDE_INSTRUCTIONS[r.step];
        let sign_str = if r.positive { "+" } else { "-" };
        let got = format!("{}{}", sign_str, NAMES[r.dominant]);
        let ok = r.dominant == expected_idx && r.positive == expected_pos;
        let mark = if ok { "\x1B[32m✓\x1B[0m" } else { "\x1B[31m✗\x1B[0m" };
        // Show peak values for all 6 axes
        let peaks_str: String = (0..6)
            .map(|i| format!("{}:{:+.1}{}", NAMES[i], r.peaks[i], UNITS[i]))
            .collect::<Vec<_>>()
            .join("  ");
        out.push_str(&format!(
            "  {} {:<45} → \x1B[1m{:<8}\x1B[0m\x1B[K\n",
            mark, inst, got
        ));
        out.push_str(&format!("      peaks: {}\x1B[K\n", peaks_str));
    }

    match guide {
        GuideState::Waiting(step) => {
            let (inst, _, _, _) = GUIDE_INSTRUCTIONS[*step];
            out.push_str("  \x1B[K\n");
            out.push_str(&format!(
                "  Step {}/{}: \x1B[1m{}\x1B[0m\x1B[K\n",
                step + 1, GUIDE_STEPS, inst
            ));
            out.push_str("  \x1B[K\n");
            out.push_str("  Press SPACE to begin 3-second recording, then do the movement.\x1B[K\n");
            out.push_str("  Return to neutral BEFORE pressing SPACE.\x1B[K\n");
        }
        GuideState::Recording { step, start, .. } => {
            let (inst, _, _, _) = GUIDE_INSTRUCTIONS[*step];
            let remaining = 3.0 - start.elapsed().as_secs_f32();
            let bar_len = ((remaining / 3.0) * 30.0) as usize;
            let bar_str: String = "█".repeat(bar_len) + &" ".repeat(30 - bar_len.min(30));
            out.push_str("  \x1B[K\n");
            out.push_str(&format!("  \x1B[1mNOW: {}\x1B[0m\x1B[K\n", inst));
            out.push_str(&format!("  [{}] {:.1}s\x1B[K\n", bar_str, remaining.max(0.0)));
        }
        GuideState::Done => {
            out.push_str("  \x1B[K\n");
            out.push_str("  \x1B[1mTest complete!\x1B[0m  Press SPACE or q to return to live view.\x1B[K\n");
        }
    }
    out.push_str("\x1B[J");
    print_out(&out);
}

fn print_out(s: &str) {
    let stdout = io::stdout();
    let mut lock = stdout.lock();
    lock.write_all(s.as_bytes()).ok();
    lock.flush().ok();
}

// ── Dump mode ─────────────────────────────────────────────────────────────────

/// `--dump <file>` — write raw CSV frames to file until Ctrl-C.
///
/// The scripted sequence to run while recording:
///   Phase 1 (still)    : hold neutral for 5 s
///   Phase 2 (yaw)      : turn head RIGHT and back, 5 slow reps
///   Phase 3 (still)    : hold neutral 3 s
///   Phase 4 (pitch)    : tilt head UP (nose toward ceiling) and back, 5 reps
///   Phase 5 (still)    : hold neutral 3 s
///   Phase 6 (roll)     : tilt RIGHT (right ear toward shoulder) and back, 5 reps
///   Phase 7 (still)    : hold neutral 3 s
///   Phase 8 (z)        : lean head toward monitor and back, 5 reps
///   Phase 9 (still)    : hold neutral 5 s, then Ctrl-C
async fn run_dump(path: &str) -> Result<()> {
    use std::fs::File;
    use std::io::BufWriter;
    use tokio::io::AsyncReadExt;
    use tokio::net::UnixListener;
    use tokio::signal;
    use tokio::sync::mpsc as tokio_mpsc;

    let socket_path = headtrack_ipc::socket_path();
    let out_file = File::create(path)?;
    let mut writer = BufWriter::new(out_file);

    // Param log listener — captures GUI parameter changes with timestamps.
    let (param_tx, mut param_rx) = tokio_mpsc::channel::<String>(64);
    let paramlog_path = headtrack_ipc::paramlog_socket_path();
    let _ = std::fs::remove_file(&paramlog_path);
    let paramlog_listener = UnixListener::bind(&paramlog_path)?;
    tokio::spawn(async move {
        loop {
            if let Ok((mut stream, _)) = paramlog_listener.accept().await {
                let mut buf = String::new();
                if stream.read_to_string(&mut buf).await.is_ok() {
                    let _ = param_tx.send(buf.trim().to_string()).await;
                }
            }
        }
    });

    // Print the script to stderr so the user can read it while the file is being written.
    eprintln!();
    eprintln!("=== headtrack-rs axis dump ===");
    eprintln!("Output: {}", path);
    eprintln!();
    eprintln!("Follow this sequence (start after 'Recording...' appears):");
    eprintln!("  0:00 - 0:05   HOLD STILL (neutral baseline)");
    eprintln!("  0:05 - 0:20   Turn head RIGHT and back, ~5 slow reps");
    eprintln!("  0:20 - 0:23   HOLD STILL");
    eprintln!("  0:23 - 0:38   Tilt head UP (nose toward ceiling) and back, ~5 reps");
    eprintln!("  0:38 - 0:41   HOLD STILL");
    eprintln!("  0:41 - 0:56   Tilt head RIGHT (right ear to shoulder) and back, ~5 reps");
    eprintln!("  0:56 - 0:59   HOLD STILL");
    eprintln!("  0:59 - 1:14   Lean toward monitor and back, ~5 reps");
    eprintln!("  1:14 - 1:19   HOLD STILL");
    eprintln!("  Then press Ctrl-C");
    eprintln!();
    eprintln!("Connecting to daemon...");

    let mut client = loop {
        match headtrack_ipc::client::IpcClient::connect(&socket_path).await {
            Ok(c) => break c,
            Err(_) => tokio::time::sleep(Duration::from_secs(1)).await,
        }
    };

    // CSV header
    writeln!(writer, "time_s,yaw,pitch,roll,x,y,z")?;

    let start = Instant::now();
    eprintln!("Recording... (Ctrl-C to stop)");

    loop {
        // Drain any pending param changes (non-blocking).
        while let Ok(cmd) = param_rx.try_recv() {
            let t = start.elapsed().as_secs_f64();
            writeln!(writer, "# {:.4} PARAM: {}", t, cmd)?;
            eprintln!("  [{:.1}s] PARAM: {}", t, cmd);
        }

        tokio::select! {
            result = client.recv() => {
                match result {
                    Ok(p) => {
                        let t = start.elapsed().as_secs_f64();
                        writeln!(writer, "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                            t, p.yaw, p.pitch, p.roll, p.x, p.y, p.z)?;
                    }
                    Err(_) => {
                        eprintln!("Daemon disconnected. Reconnecting...");
                        client = loop {
                            match headtrack_ipc::client::IpcClient::connect(&socket_path).await {
                                Ok(c) => break c,
                                Err(_) => tokio::time::sleep(Duration::from_secs(1)).await,
                            }
                        };
                    }
                }
            }
            _ = signal::ctrl_c() => {
                eprintln!("Stopped. {} written.", path);
                break;
            }
            _ = async {
                #[cfg(unix)]
                {
                    use tokio::signal::unix::{signal, SignalKind};
                    let mut s = signal(SignalKind::terminate()).unwrap();
                    s.recv().await;
                }
                #[cfg(not(unix))]
                std::future::pending::<()>().await;
            } => {
                eprintln!("Stopped (SIGTERM). {} written.", path);
                break;
            }
        }
    }

    writer.flush()?;
    let _ = std::fs::remove_file(&paramlog_path);
    Ok(())
}

// ── Main ──────────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    // Check for --dump mode before starting TUI.
    let args: Vec<String> = std::env::args().collect();
    if let Some(pos) = args.iter().position(|a| a == "--dump") {
        let path = args.get(pos + 1).map(|s| s.as_str()).unwrap_or("headtrack-dump.csv");
        return run_dump(path).await;
    }

    let socket_path = headtrack_ipc::socket_path();

    enable_raw_mode()?;
    print!("\x1B[?25l\x1B[2J");
    io::stdout().flush()?;

    let (key_tx, mut key_rx) = mpsc::channel::<KeyCode>(16);
    tokio::task::spawn_blocking(move || {
        loop {
            if event::poll(Duration::from_millis(50)).unwrap_or(false) {
                if let Ok(Event::Key(k)) = event::read() {
                    let _ = key_tx.blocking_send(k.code);
                    if matches!(k.code, KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL)) {
                        break;
                    }
                }
            }
        }
    });

    let mut stats = Stats::new();
    let mut connected = false;
    let mut frozen = false;
    let mut guide: Option<(GuideState, Vec<GuideResult>)> = None;
    let mut client: Option<headtrack_ipc::client::IpcClient> = None;
    let mut render_tick = tokio::time::interval(Duration::from_millis(50));

    loop {
        if client.is_none() {
            render_live(&stats, false, frozen);
            match headtrack_ipc::client::IpcClient::connect(&socket_path).await {
                Ok(c) => { client = Some(c); connected = true; }
                Err(_) => {
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    continue;
                }
            }
        }

        tokio::select! {
            result = client.as_mut().unwrap().recv() => {
                match result {
                    Ok(pose) => {
                        if !frozen { stats.update(pose); }
                        if let Some((ref mut gs, ref mut results)) = guide {
                            if let Some(result) = gs.feed(pose) {
                                results.push(result);
                            }
                        }
                    }
                    Err(_) => { client = None; connected = false; }
                }
            }

            _ = render_tick.tick() => {
                match &guide {
                    Some((gs, results)) => render_guided(gs, results),
                    None => render_live(&stats, connected, frozen),
                }
            }

            Some(key) = key_rx.recv() => {
                match key {
                    KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => {
                        if let Some((GuideState::Done, _)) = &guide {
                            guide = None;
                        } else if guide.is_some() {
                            guide = None;
                        } else {
                            break;
                        }
                    }
                    KeyCode::Char('r') | KeyCode::Char('R') => {
                        if guide.is_none() {
                            let _ = headtrack_ipc::send_cmd("recenter\n");
                        }
                    }
                    KeyCode::Char('c') | KeyCode::Char('C') => {
                        if guide.is_none() { stats.clear_minmax(); }
                    }
                    KeyCode::Char('f') | KeyCode::Char('F') => {
                        if guide.is_none() { frozen = !frozen; }
                    }
                    KeyCode::Char('g') | KeyCode::Char('G') => {
                        if guide.is_none() {
                            // Recenter before guided test
                            let _ = headtrack_ipc::send_cmd("recenter\n");
                            guide = Some((GuideState::new(), Vec::new()));
                        }
                    }
                    KeyCode::Char(' ') => {
                        if let Some((ref mut gs, _)) = guide {
                            match gs {
                                GuideState::Waiting(_) => {
                                    stats.clear_minmax();
                                    gs.start_recording();
                                }
                                GuideState::Done => {
                                    guide = None;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    disable_raw_mode()?;
    println!("\x1B[?25h");
    io::stdout().flush()?;
    Ok(())
}
