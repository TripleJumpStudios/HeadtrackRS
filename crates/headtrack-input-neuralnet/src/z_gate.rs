//! Physiological Z gate — tracker-side spike rejection + angle-conditioned
//! damping for the IPD-derived depth.

//!
//! Motivation (measured, 2026-08-01 extreme-angle session): raw Z steps
//! ~2 mm/frame near center but 6 mm/frame (p95 15 mm) at look-down ≥20° or
//! |yaw| ≥30°, with single-frame landmark misfires of 90–470 mm — physically
//! impossible head motion (0.5 m in 16 ms). Blinks produce the same signature
//! (eyelid closure shifts the eye keypoints → IPD → Z). The pipeline's
//! median-of-3 kills only 1-frame spikes; multi-frame excursions ramp through.
//!
//! Two mechanisms, both transparent in the normal envelope:
//!
//! 1. **Rate gate** — a head cannot change camera distance faster than
//!    ~0.7 m/s. Steps beyond that are rejected (output holds); if the new
//!    level *persists* ~170 ms it is accepted as a genuine move (re-detect
//!    after tracking loss, a genuinely fast slide). A blink dip (~100–150 ms)
//!    never persists long enough, so Z holds straight through it.
//! 2. **Angle-conditioned damping** — at poses where the IPD cue is
//!    ill-conditioned (far yaw: foreshortening ≈ cos(yaw) loses sensitivity;
//!    deep look-down: keypoint quality drops) the accepted innovation is
//!    scaled down (min ×0.25). Confidence is exactly 1.0 below 15° yaw /
//!    above −10° pitch — zero behavioral change where all tuning was done.
//!
//! The rate check runs on the RAW measurement derivative and the damping on a
//! separate output recursion — two states, deliberately (a single shared
//! state turns filter lag into false "impossible steps": the One-Euro D5
//! lesson).
//!
//! `HEADTRACK_Z_GATE=off` (or `0`) disables the whole gate.
//!
//! `HEADTRACK_Z_GATE_TRACE=<path>` writes one CSV row per sample recording the
//! gate's **input and output together**. This exists because the question "what
//! does the gate actually do to Z" could not be answered from the captures we
//! had: `HEADTRACK_DUMP_RAW` taps `RawPose`, which is produced *after* this
//! filter, so every "raw" capture in the project is already gated. An earlier
//! analysis simulated the gate over those dumps and concluded it amplified
//! excursions — it was applying the gate a second time to gated data. This
//! trace measures the transformation directly instead of inferring it, and it
//! still records (with `z_in == z_out`) when the gate is disabled, which makes
//! a gate-off run a usable control.

use std::io::Write;

/// Maximum plausible head-depth rate, mm/s (~0.7 m/s).
const MAX_RATE_MM_PER_S: f32 = 700.0;
/// A rejected level must persist this long before it is accepted as real.
/// Longer than a blink (~100–150 ms), short enough that a genuine fast move
/// or post-re-detect jump lands quickly.
const RESYNC_MS: f32 = 170.0;
/// Damping floor at maximally ill-conditioned poses.
const CONF_FLOOR: f32 = 0.25;

/// Innovation confidence for the IPD depth cue at a given head pose.
/// 1.0 inside the well-conditioned envelope; ramps down outside it.
fn confidence(yaw_deg: f32, pitch_deg: f32) -> f32 {
    let ya = yaw_deg.abs();
    // Full trust below 15° yaw, ramp to the floor by 40°.
    let yf = 1.0 - 0.65 * (((ya - 15.0) / 25.0).clamp(0.0, 1.0));
    // Look-down only: full trust above −10° pitch, ramp to the floor by −30°.
    let pf = 1.0 - 0.65 * (((-pitch_deg - 10.0) / 20.0).clamp(0.0, 1.0));
    (yf * pf).max(CONF_FLOOR)
}

pub(crate) struct ZGate {
    enabled: bool,
    /// `HEADTRACK_Z_GATE_TRACE` sink: per-sample `(z_in, z_out, held, conf)`.
    trace: Option<std::io::BufWriter<std::fs::File>>,
    trace_rows: u64,
    /// Last ACCEPTED raw measurement — the rate check's reference.
    last_raw: Option<f32>,
    /// Last emitted output — the damping recursion's reference.
    last_out: f32,
    last_ts_us: u64,
    /// Candidate new level while rejecting, and when it first appeared.
    suspect_z: f32,
    suspect_since_us: Option<u64>,
}

impl ZGate {
    pub(crate) fn new() -> Self {
        let enabled = !headtrack_core::env_flag_off("HEADTRACK_Z_GATE");
        let trace = std::env::var("HEADTRACK_Z_GATE_TRACE")
            .ok()
            .and_then(|path| {
                match std::fs::File::create(&path) {
                    Ok(f) => {
                        let mut w = std::io::BufWriter::new(f);
                        let _ = writeln!(w, "# z_gate={}", if enabled { "on" } else { "off" });
                        let _ = writeln!(w, "ts_us,z_in,z_out,held,conf,yaw,pitch");
                        // Flush the header now: a session ends with Ctrl-C, which
                        // skips Drop, and a run short enough to lose its buffer is
                        // exactly the kind you re-run wondering what went wrong.
                        let _ = w.flush();
                        tracing::info!(
                            "HEADTRACK_Z_GATE_TRACE — per-sample Z gate trace -> {path}"
                        );
                        Some(w)
                    }
                    Err(e) => {
                        tracing::warn!(
                            "HEADTRACK_Z_GATE_TRACE set but could not create {path}: {e}"
                        );
                        None
                    }
                }
            });
        Self {
            enabled,
            trace,
            trace_rows: 0,
            last_raw: None,
            last_out: 0.0,
            last_ts_us: 0,
            suspect_z: 0.0,
            suspect_since_us: None,
        }
    }

    /// Filter one Z sample (mm, canonical convention). Returns
    /// `(z_out, held)` — `held` is true when the sample was rejected and the
    /// previous output re-emitted (stats counter).
    pub(crate) fn filter(
        &mut self,
        z_meas: f32,
        yaw_deg: f32,
        pitch_deg: f32,
        ts_us: u64,
    ) -> (f32, bool) {
        let (z_out, held) = self.filter_inner(z_meas, yaw_deg, pitch_deg, ts_us);
        if let Some(w) = self.trace.as_mut() {
            let c = confidence(yaw_deg, pitch_deg);
            let _ = writeln!(
                w,
                "{ts_us},{z_meas:.3},{z_out:.3},{},{c:.4},{yaw_deg:.3},{pitch_deg:.3}",
                u8::from(held)
            );
            self.trace_rows += 1;
            // Flush every row. Ctrl-C skips Drop, so anything still buffered is
            // lost — and this is a diagnostic path that is off by default, so a
            // 60 Hz write of ~60 bytes is the right trade against losing the
            // tail of a session someone flew to collect.
            let _ = w.flush();
        }
        (z_out, held)
    }

    /// The gate proper. Split out so [`Self::filter`] can record every return
    /// path without threading the trace through four of them.
    fn filter_inner(
        &mut self,
        z_meas: f32,
        yaw_deg: f32,
        pitch_deg: f32,
        ts_us: u64,
    ) -> (f32, bool) {
        if !self.enabled || !z_meas.is_finite() {
            return (z_meas, false);
        }
        let Some(last_raw) = self.last_raw else {
            self.last_raw = Some(z_meas);
            self.last_out = z_meas;
            self.last_ts_us = ts_us;
            return (z_meas, false);
        };
        let dt_s = ((ts_us.saturating_sub(self.last_ts_us)) as f32 / 1e6).clamp(0.008, 0.05);
        self.last_ts_us = ts_us;
        let max_step = MAX_RATE_MM_PER_S * dt_s;

        if (z_meas - last_raw).abs() <= max_step {
            // Plausible motion: accept, with angle-conditioned damping.
            self.suspect_since_us = None;
            self.last_raw = Some(z_meas);
            let c = confidence(yaw_deg, pitch_deg);
            self.last_out += c * (z_meas - self.last_out);
            (self.last_out, false)
        } else {
            // Physically impossible step: hold the output. Track the new
            // level; accept it only if it persists (a real move, not a
            // misfire or blink).
            match self.suspect_since_us {
                Some(t0) if (z_meas - self.suspect_z).abs() <= max_step * 2.0 => {
                    self.suspect_z = z_meas;
                    if ts_us.saturating_sub(t0) as f32 / 1000.0 >= RESYNC_MS {
                        self.suspect_since_us = None;
                        self.last_raw = Some(z_meas);
                        self.last_out = z_meas;
                        return (z_meas, false);
                    }
                }
                _ => {
                    self.suspect_since_us = Some(ts_us);
                    self.suspect_z = z_meas;
                }
            }
            (self.last_out, true)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gate() -> ZGate {
        ZGate {
            enabled: true,
            trace: None,
            trace_rows: 0,
            last_raw: None,
            last_out: 0.0,
            last_ts_us: 0,
            suspect_z: 0.0,
            suspect_since_us: None,
        }
    }

    const FRAME_US: u64 = 16_667; // 60 fps

    /// Near center, plausible motion passes through exactly (confidence 1.0).
    #[test]
    fn transparent_in_normal_envelope() {
        let mut g = gate();
        let mut ts = 0;
        for (i, z) in [-600.0f32, -602.0, -599.5, -605.0, -601.0]
            .iter()
            .enumerate()
        {
            ts += FRAME_US;
            let (out, held) = g.filter(*z, 3.0, -5.0, ts);
            assert!(!held, "frame {i} held");
            assert!((out - z).abs() < 1e-4, "frame {i}: {out} != {z}");
        }
    }

    /// A single-frame teleport is rejected and the output holds.
    #[test]
    fn single_spike_held() {
        let mut g = gate();
        let (_, _) = g.filter(-600.0, 0.0, 0.0, FRAME_US);
        let (out, held) = g.filter(-130.0, 0.0, 0.0, 2 * FRAME_US); // +470mm
        assert!(held);
        assert!((out - (-600.0)).abs() < 1e-4);
        // Return to plausible → tracking resumes from the held value.
        let (out, held) = g.filter(-598.0, 0.0, 0.0, 3 * FRAME_US);
        assert!(!held);
        assert!((out - (-598.0)).abs() < 1e-4);
    }

    /// A blink-length dip (~7 frames) is held throughout — never leaks.
    #[test]
    fn blink_dip_held_throughout() {
        let mut g = gate();
        let mut ts = FRAME_US;
        g.filter(-600.0, 0.0, 0.0, ts);
        for _ in 0..7 {
            ts += FRAME_US;
            let (out, held) = g.filter(-520.0, 0.0, 0.0, ts); // 80mm blink shift
            assert!(held);
            assert!((out - (-600.0)).abs() < 1e-4);
        }
        ts += FRAME_US;
        let (out, held) = g.filter(-601.0, 0.0, 0.0, ts); // eyes open
        assert!(!held);
        assert!((out - (-601.0)).abs() < 1e-4);
    }

    /// A genuine new level (post-re-detect) is accepted after RESYNC_MS.
    #[test]
    fn persistent_level_resyncs() {
        let mut g = gate();
        let mut ts = FRAME_US;
        g.filter(-600.0, 0.0, 0.0, ts);
        let mut accepted = false;
        for i in 0..15 {
            ts += FRAME_US;
            let fed = -450.0 - i as f32 * 0.5;
            let (out, held) = g.filter(fed, 0.0, 0.0, ts);
            if !held {
                accepted = true;
                assert!(
                    (out - fed).abs() < 1e-4,
                    "resynced to the new level: {out} vs {fed}"
                );
                break;
            }
        }
        assert!(accepted, "never re-synced to a persistent level");
        // ~170ms at 60fps ≈ 11 frames.
    }

    /// At extreme pose the innovation is damped but motion still tracks.
    #[test]
    fn extreme_angle_damps() {
        let mut g = gate();
        g.filter(-600.0, 35.0, -25.0, FRAME_US);
        let (out, held) = g.filter(-590.0, 35.0, -25.0, 2 * FRAME_US); // +10mm, plausible
        assert!(!held);
        assert!(
            out < -590.0 && out > -600.0,
            "damped between old and new: {out}"
        );
        let c = confidence(35.0, -25.0);
        assert!((out - (-600.0 + c * 10.0)).abs() < 1e-3);
    }

    /// The trace exists to answer "what does the gate do to Z", so the two
    /// things it must never get wrong are: one row per sample (a dropped row
    /// silently changes the statistics), and `z_in` being the *measurement*
    /// while `z_out` is what the tracker actually emitted. A held sample is the
    /// case where those differ, which is exactly the case under investigation.
    #[test]
    fn the_trace_records_input_and_output_for_every_sample() {
        let path = std::env::temp_dir().join(format!("ht-zgate-trace-{}.csv", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let mut g = gate();
        g.trace = Some(std::io::BufWriter::new(
            std::fs::File::create(&path).expect("temp file"),
        ));

        g.filter(-600.0, 0.0, 0.0, FRAME_US);
        let (_, held) = g.filter(-130.0, 0.0, 0.0, 2 * FRAME_US); // +470mm: impossible
        assert!(held);
        drop(g);

        let body = std::fs::read_to_string(&path).expect("trace readable");
        let rows: Vec<&str> = body
            .lines()
            .filter(|l| !l.starts_with('#') && !l.is_empty())
            .collect();
        assert_eq!(rows.len(), 2, "one row per sample, held or not");

        let f: Vec<&str> = rows[1].split(',').collect();
        assert_eq!(f[3], "1", "held flag must be set");
        let z_in: f32 = f[1].parse().expect("z_in");
        let z_out: f32 = f[2].parse().expect("z_out");
        assert!(
            (z_in - -130.0).abs() < 0.01,
            "z_in is the raw measurement, got {z_in}"
        );
        assert!(
            (z_out - -600.0).abs() < 0.01,
            "z_out is the held output, got {z_out}"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn confidence_shape() {
        assert!((confidence(0.0, 0.0) - 1.0).abs() < 1e-6);
        assert!((confidence(14.9, -9.9) - 1.0).abs() < 1e-6);
        assert!(confidence(40.0, 0.0) < 0.4);
        assert!(confidence(0.0, -30.0) < 0.4);
        assert!(confidence(60.0, -40.0) >= CONF_FLOOR - 1e-6);
    }
}
