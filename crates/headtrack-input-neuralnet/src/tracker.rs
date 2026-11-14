use anyhow::{Context, Result};
use headtrack_core::{config::CameraConfig, RawPose};
use image::{GrayImage, ImageBuffer, Rgb};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{CameraIndex, CameraFormat, FrameFormat, RequestedFormat, RequestedFormatType, Resolution},
    Camera,
};
use std::{path::PathBuf, time::Instant};
use tracing::{debug, info, warn};
use crate::capture_errors::{is_device_gone, CaptureErrors, CaptureStep};

use crate::{
    image_proc::to_gray,
    landmark_position::position_from_eyes,
    optical_flow::track_landmarks,
    pnp::{pose_to_canonical, CameraIntrinsics, PnpSolver},
    pose_estimator_6drepnet::{HeadRotation, PoseEstimator6DRepNet, MODEL_FILENAME as MODEL_6DREPNET},
    scrfd_detector::{Detection, ScrfdDetector, MODEL_FILENAME as MODEL_SCRFD},
    v4l2_capture::V4l2Camera,
};

/// Return type for [`Tracker::step_with_frame`]: pose + optional RGB preview frame.
type StepFrame = (Option<RawPose>, Option<ImageBuffer<Rgb<u8>, Vec<u8>>>);

/// Periodic SCRFD re-detect interval for the optical-flow + PnP path. Bounds
/// landmark drift over time. ~5 s at 60 fps. Each periodic detect is
/// perceptually visible (~10 ms inference burst stretches that frame's cycle,
/// and the SCRFD-vs-LK landmark mismatch causes a tiny pose snap), so we
/// space them widely. The drift-based trigger below catches faster-moving
/// drift before this fallback fires.
const REDETECT_THRESHOLD: u32 = 300;

/// Read the re-detect interval, overridable with `HEADTRACK_REDETECT_FRAMES`.
///
/// This exists to test a specific hypothesis: **the Z excursions may be optical
/// flow drift, bounded by this interval rather than caused by the detector.**
/// Analysis of the 2026-08-21 MSFS gate traces found no Z excursion episode
/// longer than [`REDETECT_THRESHOLD`] samples in either detector (51 episodes,
/// maxima 262 and 288), and a median gap between jump episodes of exactly 301
/// samples. That is the signature of a process *terminated* by the re-detect.
///
/// It is confounded, though: the yaw protocol used 5 s holds and 300 frames at
/// ~62 fps is ~4.8 s, so movement cadence and re-detect cadence nearly coincide.
/// Varying this value breaks the tie — nothing else in the system has that
/// period. See `docs/DEPTH_CUE_PLAN.md`.
///
/// Note the trade this constant's doc comment above describes: a periodic
/// re-detect is perceptually visible because SCRFD and LK disagree slightly on
/// the landmarks. Spacing them widely was the right call **when drift was
/// assumed small**. If drift is really hundreds of millimetres, more frequent
/// re-detects mean more snaps but much smaller ones, which may be a net win.
/// That is a measurement, not an assumption — hence this knob.
///
/// Clamped to 10..=100_000: below ~10 the detector runs almost every frame and
/// the optical-flow path stops existing, which is a different program.
fn read_redetect_frames() -> u32 {
    let raw = std::env::var("HEADTRACK_REDETECT_FRAMES").ok();
    let chosen = redetect_frames_from(raw.as_deref());
    match raw.as_deref().map(str::trim) {
        None | Some("") => {}
        Some(s) if s.parse::<u32>().is_err() => warn!(
            "HEADTRACK_REDETECT_FRAMES={s:?} is not a number — using the default \
             {REDETECT_THRESHOLD}"
        ),
        Some(s) => {
            if s.parse::<u32>() != Ok(chosen) {
                warn!("HEADTRACK_REDETECT_FRAMES={s} out of range — clamped to {chosen}");
            }
            info!(
                "HEADTRACK_REDETECT_FRAMES={chosen} — detector re-seeds every {chosen} frames \
                 (default {REDETECT_THRESHOLD}); diagnostic knob, see docs/DEPTH_CUE_PLAN.md"
            );
        }
    }
    chosen
}

/// Pure half of [`read_redetect_frames`]: parse and clamp, no environment and
/// no logging, so the behaviour is testable without `set_var` racing other
/// tests. Anything unparseable falls back to the default rather than erroring —
/// a diagnostic knob must never be able to stop the tracker starting.
fn redetect_frames_from(raw: Option<&str>) -> u32 {
    raw.map(str::trim)
        .filter(|s| !s.is_empty())
        .and_then(|s| s.parse::<u32>().ok())
        .map(|v| v.clamp(REDETECT_FRAMES_MIN, REDETECT_FRAMES_MAX))
        .unwrap_or(REDETECT_THRESHOLD)
}

/// Below this the detector runs almost every frame and the optical-flow path
/// effectively stops existing, which is a different program rather than a
/// tuning of this one.
const REDETECT_FRAMES_MIN: u32 = 10;
/// Above this the periodic re-detect is functionally disabled; only the
/// drift-triggered path remains.
const REDETECT_FRAMES_MAX: u32 = 100_000;


/// Reprojection RMS threshold (pixels). If PnP's converged solution has RMS
/// above this for 3 consecutive frames, OF landmarks have drifted off real
/// face features and we trigger an immediate SCRFD re-detect even before
/// [`REDETECT_THRESHOLD`] expires. 6 px is well above per-frame measurement
/// noise (~1-2 px) but well below "completely lost".
const PNP_DRIFT_RMS_PX: f32 = 6.0;
/// Drift threshold when 6DRepNet drives rotation (the default). The tight 6px
/// [`PNP_DRIFT_RMS_PX`] was calibrated for the era when PnP drove *rotation* and
/// needed sub-pixel landmarks. With learned rotation, the drift re-detect only
/// guards the **translation** PnP — which tolerates much looser landmarks — so
/// the 6px trip just produced metronomic SCRFD re-anchor "ticks" in X/Y every
/// ~0.5s. Relaxed to 25 px so drift re-detects become rare (periodic +
/// genuine-failure re-detects still catch real track loss). The figure came
/// from the since-deleted dense path, whose everyday model error sat at ~14 px
/// while genuine tracking-loss spirals pushed RMS past 50 — 25 separates them.
const PNP_DRIFT_RMS_PX_6DREP: f32 = 25.0;
/// How many consecutive high-RMS frames trigger a drift-based re-detect.
const PNP_DRIFT_FRAMES: u32 = 3;
/// Minimum frames between *drift-triggered* (Layer-2) forced SCRFD re-detects.
///
/// The FPS-collapse failure mode: during a head turn, optical-flow drift keeps
/// PnP RMS above [`PNP_DRIFT_RMS_PX`], so the drift trip fires every few frames,
/// forcing a ~10 ms SCRFD inference each time and storming the CPU. This rate
/// limit caps drift-triggered re-detects to at most one per ~0.5 s (at 60 fps):
/// the tracker keeps riding optical flow in between, and a genuinely lost track
/// still gets re-anchored within half a second (and by the periodic
/// [`REDETECT_THRESHOLD`] regardless). Failure-path re-detects (OF/PnP failure)
/// are deliberately *not* rate-limited — those are real track loss that must
/// recover promptly, and they are self-limiting rather than motion-driven.
const MIN_FRAMES_BETWEEN_FORCED_REDETECT: u32 = 30;
/// How often to emit the pipeline-stat info line. 60 frames ≈ 1 s at 60 fps.
const STATS_LOG_INTERVAL: u32 = 60;

/// How many consecutive frames without a 6DRepNet result may be covered by
/// holding the previous rotation. ~165 ms at 30 Hz: long enough to ride out
/// the isolated inference failures seen under load, short enough that a real
/// outage falls back to PnP rather than freezing the head.
const ROT_HOLD_MAX_FRAMES: u32 = 5;
/// Minimum interval between 6DRepNet error warnings. Rate-limited, never
/// suppressed: a persistent failure must stay visible.
const ROT_ERROR_WARN_INTERVAL: std::time::Duration = std::time::Duration::from_secs(30);

/// Bridges brief 6DRepNet outages so a dropped frame doesn't swap the rotation
/// source mid-stream.
///
/// The ONNX session occasionally fails for a single frame under heavy system
/// load — observed twice on 2026-08-17, once building the input tensor and
/// once inside a `ReorderInput` node, both reporting `GetElementType is not
/// implemented`. The old behaviour reverted to the PnP rotation for that one
/// frame. The two sources routinely disagree by a few degrees, so that is a
/// visible rotation step, and because the landmark Z cue is yaw-corrected it
/// perturbs translation as well. Holding the last learned rotation for a few
/// frames costs nothing when the outage is one frame; a sustained outage still
/// falls through to PnP.
#[derive(Default)]
struct RotationHold {
    last: Option<HeadRotation>,
    held_frames: u32,
    errors: u64,
    last_warn: Option<Instant>,
}

impl RotationHold {
    /// Record an inference error, warning at most once per
    /// [`ROT_ERROR_WARN_INTERVAL`] with the running total.
    fn note_error(&mut self, e: &anyhow::Error) {
        self.errors += 1;
        let due = self.last_warn.is_none_or(|t| t.elapsed() >= ROT_ERROR_WARN_INTERVAL);
        if due {
            warn!(
                "6DRepNet inference error: {e:#} — holding the last rotation \
                 ({} total this session)",
                self.errors
            );
            self.last_warn = Some(Instant::now());
        } else {
            debug!("6DRepNet inference error: {e:#}");
        }
    }

    /// Resolve this frame's rotation. `fresh` is the estimator's result, or
    /// `None` when it produced nothing. Returns the rotation to apply, or
    /// `None` to leave the caller's PnP rotation in place.
    fn resolve(&mut self, fresh: Option<HeadRotation>) -> Option<HeadRotation> {
        match fresh {
            Some(r) => {
                self.last = Some(r);
                self.held_frames = 0;
                Some(r)
            }
            None => match self.last {
                Some(prev) if self.held_frames < ROT_HOLD_MAX_FRAMES => {
                    self.held_frames += 1;
                    Some(prev)
                }
                _ => {
                    if self.held_frames == ROT_HOLD_MAX_FRAMES {
                        self.held_frames += 1; // log the transition once
                        warn!(
                            "6DRepNet has produced nothing for {ROT_HOLD_MAX_FRAMES} frames — \
                             falling back to PnP rotation until it recovers"
                        );
                    }
                    None
                }
            },
        }
    }
}

/// Per-pipeline counters reset every [`STATS_LOG_INTERVAL`] frames. Used to
/// diagnose which stage is dominating cost / dropping frames; intended to be
/// turned off (or moved behind a flag) once the pipeline is stable.
#[derive(Default, Debug)]
struct PipelineStats {
    frames: u32,
    of_attempts: u32,
    of_success: u32,
    scrfd_attempts: u32,
    scrfd_success: u32,
    sixdrep_attempts: u32,
    sixdrep_success: u32,
    pnp_attempts: u32,
    pnp_success: u32,
    misses: u32,
    /// Forced (non-periodic) SCRFD re-detects this window — drift trips,
    /// optical-flow failures, PnP divergence. The storm metric: a healthy run
    /// shows ~0; a chronic re-detect storm (the FPS-collapse failure mode)
    /// shows this climbing with head motion. See `maybe_log_stats`.
    forced_redetects: u32,
    /// Drift trips that were *suppressed* by the forced-redetect rate limit
    /// (`MIN_FRAMES_BETWEEN_FORCED_REDETECT`). Non-zero means the rate limiter
    /// is actively holding the storm off.
    suppressed_redetects: u32,
    /// Sum / max / count of PnP reprojection RMS (px) over successful solves
    /// this window. Avg + peak RMS is the upstream cause of drift trips, so it
    /// rising during yaw is the direct signal behind the storm.
    rms_sum: f64,
    rms_max: f32,
    rms_count: u32,
    /// Frames where the physiological Z gate rejected an impossible depth
    /// step and held the previous value (spikes, blinks, landmark misfires).
    z_gate_holds: u32,
    inference_us_total: u64,
    inference_us_max: u64,
}

impl PipelineStats {
    fn reset(&mut self) {
        *self = Self::default();
    }
}


/// Selects the rotation source for the SCRFD-driven (PnP) pipeline.
///
/// Default `SixDRepNet`: rotation comes from the learned head-pose ONNX model
/// (`head-pose-mobilenetv3-large.onnx`), which regresses head orientation
/// holistically and so structurally ignores facial expression and is far
/// steadier than the 5-point PnP rotation. Translation still comes from PnP.
/// `Pnp` uses the geometric solver for both. Override the default with
/// `HEADTRACK_ROT_SOURCE=pnp` to force geometric rotation (e.g. if the model is
/// missing or the CPU budget is tight).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RotationSource {
    Pnp,
    SixDRepNet,
}

fn read_rotation_source() -> RotationSource {
    match std::env::var("HEADTRACK_ROT_SOURCE")
        .ok()
        .as_deref()
        .map(str::trim)
        .map(str::to_ascii_lowercase)
        .as_deref()
    {
        Some("pnp") | Some("geometric") => RotationSource::Pnp,
        // Empty / unset / "6drepnet" / anything else → 6DRepNet (the default).
        // Falls back to PnP automatically if the model fails to load.
        _ => RotationSource::SixDRepNet,
    }
}

/// Selects the source of translation (X/Y/Z) for the SCRFD (5-point) pipeline.
///
/// Default `Landmark`: X/Y from the eye centroid and Z from interpupillary
/// distance (`landmark_position`) — stable under expression and pitch, where the
/// rigid PnP `tvec` wobbles. `HEADTRACK_TRANS_SOURCE=pnp` reverts to the PnP
/// translation. The PnP solve still runs either way (rotation fallback + drift
/// guard); this only selects which translation feeds the pose.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TranslationSource {
    Landmark,
    Pnp,
}

/// Which landmark pairs derive depth. Defaults to [`DepthCue::Ipd`] — the
/// shipped behaviour — so an unset environment changes nothing.
///
/// The alternatives are ranked on 1731 real frames in [`DepthCue`]'s docs;
/// `eye-mouth` measured ~11.7× quieter than IPD at extreme yaw. This is a knob
/// for A/B in the cockpit, not a settled default: the absolute scale of any
/// multi-pair cue carries a per-person gain error until the face model is
/// calibrated, so compare *feel and noise*, not absolute lean distance.
/// `HEADTRACK_CAPTURE_RES=<W>x<H>` overrides the configured capture resolution
/// for one run.
///
/// This exists as an environment override rather than a config edit on purpose:
/// `camera.toml` also holds the user's controller bindings, and hand-editing it
/// to run an A/B is how those get lost.
///
/// Resolution matters more than it looks. The detectors letterbox the *whole
/// frame* to a fixed 640 (YuNet) or 320 (SCRFD), so a bigger frame buys nothing
/// at the detect step — but the detector only runs once per
/// [`REDETECT_THRESHOLD`] frames, and optical flow supplies the other 299,
/// tracking at full capture resolution. Depth error is `e·Z²/(f·B)` with `e` the
/// landmark error in pixels: flow's error is roughly constant in pixels, while
/// `f` scales with resolution, so doubling capture resolution should nearly
/// halve depth noise on flow-tracked frames. Unverified in-sim — that is what
/// the A/B is for.
fn read_capture_resolution(default: (u32, u32)) -> (u32, u32) {
    let Ok(raw) = std::env::var("HEADTRACK_CAPTURE_RES") else {
        return default;
    };
    let parsed = raw
        .trim()
        .split_once(['x', 'X'])
        .and_then(|(w, h)| Some((w.trim().parse().ok()?, h.trim().parse().ok()?)))
        .filter(|&(w, h): &(u32, u32)| w >= 160 && h >= 120 && w <= 7680 && h <= 4320);
    match parsed {
        Some(res) => {
            info!("HEADTRACK_CAPTURE_RES={raw:?} — requesting {}×{} instead of {}×{}",
                  res.0, res.1, default.0, default.1);
            res
        }
        None => {
            warn!("HEADTRACK_CAPTURE_RES={raw:?} unparseable — expected e.g. 1920x1080;                    using {}×{}", default.0, default.1);
            default
        }
    }
}

fn read_depth_cue() -> crate::landmark_position::DepthCue {
    use crate::landmark_position::DepthCue as C;
    let raw = std::env::var("HEADTRACK_DEPTH_CUE").ok();
    let cue = match raw.as_deref().map(str::trim).map(str::to_ascii_lowercase).as_deref() {
        None | Some("") | Some("diagonals") => C::Diagonals,
        Some("ipd") => C::Ipd,
        Some("nose-mouth") => C::NoseMouth,
        Some("eye-nose") => C::EyeNose,
        Some("eye-mouth") => C::EyeMouth,
        Some("verticals") => C::Verticals,
        Some("rigid") => C::Rigid,
        Some("median10") => C::Median10,
        Some(other) => {
            warn!(
                "HEADTRACK_DEPTH_CUE={other:?} unrecognised — using diagonals. Valid: \
                 ipd, nose-mouth, eye-nose, eye-mouth, verticals, diagonals, rigid, \
                 median10"
            );
            C::Diagonals
        }
    };
    if cue != C::Diagonals {
        info!("HEADTRACK_DEPTH_CUE={cue:?} — depth from {} landmark pair(s) instead of \
               the default diagonals", cue.pairs().len());
    }
    cue
}

fn read_translation_source() -> TranslationSource {
    let src = match std::env::var("HEADTRACK_TRANS_SOURCE")
        .ok()
        .as_deref()
        .map(str::trim)
        .map(str::to_ascii_lowercase)
        .as_deref()
    {
        Some("pnp") => TranslationSource::Pnp,
        // Empty / unset / "landmark" / anything else → Landmark (the default).
        _ => TranslationSource::Landmark,
    };
    info!("translation source for new pipeline: {src:?} (HEADTRACK_TRANS_SOURCE=pnp to force PnP)");
    src
}

/// Configuration passed into the background thread.
///
/// All fields are `Send + 'static` so they can be moved into `thread::spawn`.
pub struct TrackerConfig {
    /// Directory holding the ONNX models. The tracker resolves
    /// `scrfd-det-10g.onnx` and `head-pose-mobilenetv3-large.onnx` under it.
    pub model_dir: PathBuf,
    pub camera_index: u32,
    pub fov_diag_deg: f32,
    /// Requested camera resolution (width, height).  Higher resolution gives
    /// more precise face bounding boxes → less Z depth noise.  The ONNX models
    /// resize internally, so inference speed is barely affected.
    pub camera_resolution: (u32, u32),
}

impl TrackerConfig {
    pub fn from_model_dir(
        model_dir: &std::path::Path,
        camera_index: u32,
        fov_diag_deg: f32,
    ) -> Self {
        let cam_cfg = CameraConfig::load();
        Self {
            model_dir: model_dir.to_path_buf(),
            camera_index,
            fov_diag_deg,
            camera_resolution: cam_cfg.camera_resolution,
        }
    }

}

/// Abstraction over camera backends — nokhwa for most cameras, raw V4L2 for
/// cameras that crash nokhwa's format enumeration (fractional frame rates).
enum CameraBackend {
    Nokhwa(Camera),
    V4l2(V4l2Camera),
    /// Inference-only mode — camera owned by a separate capture thread.
    None,
}

impl CameraBackend {
    fn capture(&mut self) -> Result<ImageBuffer<Rgb<u8>, Vec<u8>>> {
        match self {
            CameraBackend::Nokhwa(cam) => {
                let buf = cam.frame().context("capturing frame")?;
                buf.decode_image::<RgbFormat>().context("decoding frame")
            }
            CameraBackend::V4l2(cam) => cam.frame(),
            CameraBackend::None => anyhow::bail!("no camera — use infer_gray() instead"),
        }
    }
}

/// Runs webcam capture + two-stage ONNX inference.
///
/// Created **inside** the background thread (nokhwa's `Camera` is not `Send`).
pub struct Tracker {
    camera: CameraBackend,
    /// Optional 6DRepNet-style rotation estimator. Used as the rotation
    /// source only when `HEADTRACK_ROT_SOURCE=6drepnet`; translation always
    /// comes from PnP in the SCRFD-driven path. When unset (the default after
    /// the PnP rebuild) this stays `None` and rotation comes from PnP too.
    pose_6drepnet: Option<PoseEstimator6DRepNet>,
    /// SCRFD face detector — supplies both the face bbox and the 5 keypoints.
    /// Mandatory since SL3 deleted the legacy localizer chain: if this model
    /// will not load there is no pipeline, so the constructor fails rather
    /// than degrading.
    scrfd: FaceDetector,
    /// Selected rotation source for the SCRFD path. Always `Pnp` if
    /// `pose_6drepnet` failed to load even when the user requested 6DRepNet.
    rot_source: RotationSource,
    /// Selected translation source for the SCRFD path (eye-landmark vs PnP).
    trans_source: TranslationSource,
    /// Stateful PnP solver (warm-started across frames).
    pnp: PnpSolver,
    /// Facial landmarks from the previous frame (full-resolution pixel
    /// coords). Optical flow uses them as the prior for the current frame.
    /// Length is whatever the upstream detector emitted (5 for SCRFD today,
    /// more if a denser landmark stage is ever added).
    last_landmarks: Option<Vec<(f32, f32)>>,
    /// Previous frame's grayscale buffer, paired with `last_landmarks`.
    last_gray: Option<GrayImage>,
    /// Frames elapsed since the last SCRFD re-detect. Caps drift by forcing
    /// a fresh detect every [`Self::redetect_frames`] frames.
    frames_since_redetect: u32,
    /// Which landmark pairs derive depth. [`DepthCue::Diagonals`] unless
    /// `HEADTRACK_DEPTH_CUE` overrides it — `ipd` restores the pre-0.2.0 cue.
    depth_cue: crate::landmark_position::DepthCue,
    /// The re-detect interval actually in force — [`REDETECT_THRESHOLD`] unless
    /// `HEADTRACK_REDETECT_FRAMES` overrides it. See [`read_redetect_frames`].
    redetect_frames: u32,
    /// Consecutive frames where PnP RMS exceeded [`PNP_DRIFT_RMS_PX`].
    /// Triggers an early re-detect when it crosses [`PNP_DRIFT_FRAMES`].
    pnp_high_rms_streak: u32,
    /// Frames since the last *drift-triggered* forced re-detect. Gates the
    /// Layer-2 trip against [`MIN_FRAMES_BETWEEN_FORCED_REDETECT`] so chronic
    /// high RMS during head motion can't storm the CPU with SCRFD inference.
    frames_since_forced_redetect: u32,
    /// Camera FOV used by the PnP intrinsics.
    fov_diag_deg: f32,
    /// Physiological Z spike/blink gate for the landmark depth cue.
    z_gate: crate::z_gate::ZGate,
    /// Covers isolated 6DRepNet inference failures — see [`RotationHold`].
    rot_hold: RotationHold,
    start: Instant,
    /// Most-recent face bounding box (preview overlay + 6DRepNet patch).
    tracked_box: Option<Detection>,
    /// Previous frame's bbox center, used by the tracking-loss detector
    /// (Layer 1) to spot physically-impossible jumps between
    /// frames. `None` until the first successful detection. Updated only
    /// on accepted bbox transitions; deliberately *not* touched when a
    /// tracking-loss trip suppresses the update.
    miss_count: u32,
    step_count: u32,
    /// Rolling per-stage counters for diagnostics; logged once per second.
    stats: PipelineStats,
    /// Developer frame capture for the detector bake-off. `None` unless
    /// `HEADTRACK_DUMP_FRAMES` is set — see [`crate::frame_dump`].
    frame_dump: Option<crate::frame_dump::FrameDumper>,
    /// Scratch for the facts `frame_dump` records. Kept on the struct so the
    /// point vectors keep their allocation across frames; untouched when the
    /// dump is off.
    frame_facts: crate::frame_dump::FrameFacts,
    /// The current run of capture errors (FINAL_REVIEW S2).
    capture_errors: CaptureErrors,
    /// What the last capture error asked the caller's loop to do: wait, or
    /// give up. Taken with [`Tracker::take_capture_step`].
    capture_step: Option<CaptureStep>,
}

/// Try to load the 6DRepNet ONNX from `model_dir`. Attempts the load whenever
/// 6DRepNet rotation is selected (the default — see [`read_rotation_source`]),
/// which only happens on the SCRFD pipeline. Logs and swallows failure so a
/// missing model file silently demotes back to PnP rotation.
fn try_load_6drepnet(
    model_dir: &std::path::Path,
    rot_source: RotationSource,
) -> Option<PoseEstimator6DRepNet> {
    if rot_source != RotationSource::SixDRepNet {
        return None;
    }
    let path = model_dir.join(MODEL_6DREPNET);
    match PoseEstimator6DRepNet::load(&path) {
        Ok(p) => {
            info!("6DRepNet rotation estimator enabled ({:?}) — default; HEADTRACK_ROT_SOURCE=pnp to force geometric", path);
            Some(p)
        }
        Err(e) => {
            warn!("6DRepNet model failed to load ({:?}): {e:#} — falling back to PnP rotation", path);
            None
        }
    }
}

/// A required ONNX model could not be loaded.
///
/// Typed rather than a bare `anyhow` string so callers can tell it apart from
/// a camera failure. The engine starts without a camera quite happily — you
/// can `SwitchCamera` later — but no camera index will fix a missing model,
/// so the two must not be reported the same way. Before SL3 they were: a
/// missing model surfaced as "no camera at index 0".
#[derive(Debug)]
pub struct MissingModel {
    pub path: PathBuf,
    pub filename: &'static str,
    /// Extra guidance for a model the bundle deliberately does not ship.
    /// `None` for the bundled models.
    pub note: Option<&'static str>,
}

/// Why `scrfd-det-10g.onnx` is not in the bundle, shown when someone opts into
/// it without supplying the file. P01 (2026-10-03) moved it to
/// `archive/models/`: its weights are licensed for non-commercial research
/// only, which a public MIT release cannot carry.
pub const SCRFD_OPT_IN_NOTE: &str =
    "SCRFD is not shipped: its weights are licensed for non-commercial research use only. \
     To use it anyway, obtain `scrfd-det-10g.onnx` yourself and place it in the model \
     directory, or unset HEADTRACK_DETECTOR to use the bundled YuNet detector (MIT).";

impl std::fmt::Display for MissingModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "required model `{}` could not be loaded from {}\n\
             Head tracking cannot run without it. Put `{}` in the model \
             directory, or point at another one with HEADTRACK_MODELS=/path/to/models.",
            self.filename,
            self.path.display(),
            self.filename,
        )?;
        if let Some(note) = self.note {
            write!(f, "\n{note}")?;
        }
        Ok(())
    }
}

impl std::error::Error for MissingModel {}

/// Load the SCRFD face detector from `model_dir`.
///
/// SCRFD supplies the face bbox and the 5 keypoints that every downstream
/// stage consumes, so this is a hard requirement. Before SL3 a failure here
/// silently demoted to a legacy localizer chain; that fallback masked model
/// path bugs (see `KNOWN_BUGS.md`, "Models not found unless launched from the
/// repo root"), so it now fails startup with the path it tried.
/// Which face detector supplies the bbox and the 5 keypoints.
///
/// Both produce the identical [`FaceDetection`] shape and the identical
/// keypoint order (right eye, left eye, nose tip, right mouth, left mouth), so
/// nothing downstream — PnP, `position_from_eyes`, the 6DRepNet crop — cares
/// which one is active.
///
/// **This switch exists because of licensing, not quality.** SCRFD's weights
/// are restricted to non-commercial academic research; YuNet is MIT. See
/// `docs/DETECTOR_BAKEOFF.md` for the measured comparison — YuNet matched
/// SCRFD's detection rate exactly out to 89° of yaw, at 2.5× the speed and
/// 1/74th the model size, with somewhat noisier keypoints at sustained angle.
pub enum FaceDetector {
    Scrfd(ScrfdDetector),
    YuNet(crate::yunet_detector::YuNetDetector),
}

impl FaceDetector {
    fn detect(
        &mut self,
        rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>,
    ) -> Result<Option<crate::scrfd_detector::FaceDetection>> {
        match self {
            Self::Scrfd(d) => d.detect(rgb),
            Self::YuNet(d) => d.detect(rgb),
        }
    }

    /// Name for logs and the tracker stats line.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Scrfd(_) => "SCRFD",
            Self::YuNet(_) => "YuNet",
        }
    }
}

/// Read `HEADTRACK_DETECTOR`. Defaults to YuNet (MIT) since P01, 2026-10-03;
/// SCRFD is opt-in with `HEADTRACK_DETECTOR=scrfd` and a user-supplied model.
pub(crate) fn read_detector_choice() -> &'static str {
    detector_choice_from(std::env::var("HEADTRACK_DETECTOR").ok().as_deref())
}

/// [`read_detector_choice`] without the environment, for tests.
fn detector_choice_from(raw: Option<&str>) -> &'static str {
    match raw.map(str::trim).map(str::to_ascii_lowercase).as_deref() {
        Some("scrfd") => "scrfd",
        Some("yunet") | None | Some("") => "yunet",
        Some(other) => {
            warn!("HEADTRACK_DETECTOR={other:?} is not a detector (yunet/scrfd) — using yunet");
            "yunet"
        }
    }
}

fn load_scrfd(model_dir: &std::path::Path) -> Result<FaceDetector> {
    load_detector(model_dir, read_detector_choice())
}

fn load_detector(model_dir: &std::path::Path, choice: &str) -> Result<FaceDetector> {
    if choice == "yunet" {
        let path = model_dir.join(crate::yunet_detector::MODEL_FILENAME);
        let detector = crate::yunet_detector::YuNetDetector::load(&path).map_err(|e| {
            anyhow::Error::new(MissingModel {
                path: path.clone(),
                filename: crate::yunet_detector::MODEL_FILENAME,
                note: None,
            })
            .context(e)
        })?;
        info!(
            "YuNet face detector loaded ({:?}) — MIT licensed, the default",
            path
        );
        return Ok(FaceDetector::YuNet(detector));
    }
    let path = model_dir.join(MODEL_SCRFD);
    let detector = ScrfdDetector::load(&path).map_err(|e| {
        anyhow::Error::new(MissingModel {
            path: path.clone(),
            filename: MODEL_SCRFD,
            note: Some(SCRFD_OPT_IN_NOTE),
        })
        .context(e)
    })?;
    info!(
        "SCRFD face detector loaded ({:?}) — opted in via HEADTRACK_DETECTOR=scrfd; \
         non-commercial research licence",
        path
    );
    Ok(FaceDetector::Scrfd(detector))
}

/// Default mouth-corner PnP weight for the 5-point SCRFD layout (indices 3, 4).
///
/// Eyes (0, 1) and nose (2) anchor rotation at weight 1.0. The mouth corners
/// are **reference points only**: they add vertical spread that stabilises
/// pitch, but they move with speech / smiles / jaw drop, so they must not
/// *drive* pose. Lowered from the historical 0.35, which still let expression
/// leak into yaw/pitch enough to make the system unusable while talking.
///
/// Kept non-zero on purpose — zeroing the mouth removes the only model points
/// below the eye line, leaving eyes + nose with poor vertical leverage and a
/// noisy/unstable pitch. A small weight is the balance between expression
/// rejection and pitch stability.
const DEFAULT_MOUTH_WEIGHT: f32 = 0.10;

/// Per-landmark PnP weights for the 5-point SCRFD layout. Mouth-corner weight
/// (indices 3, 4) defaults to [`DEFAULT_MOUTH_WEIGHT`] but can be overridden at
/// runtime with `HEADTRACK_MOUTH_WEIGHT` (clamped to `[0.0, 1.0]`) for live
/// tuning — set it, restart, and sweep without recompiling. Logged at startup.
fn pnp_landmark_weights() -> [f32; 5] {
    let mw = std::env::var("HEADTRACK_MOUTH_WEIGHT")
        .ok()
        .and_then(|v| v.parse::<f32>().ok())
        .map(|v| v.clamp(0.0, 1.0))
        .unwrap_or(DEFAULT_MOUTH_WEIGHT);
    info!("PnP mouth-corner weight = {mw:.3} (HEADTRACK_MOUTH_WEIGHT to override)");
    [1.0, 1.0, 1.0, mw, mw]
}

/// Build the `PnpSolver`: the 5-point SCRFD model with mouth corners
/// down-weighted via [`pnp_landmark_weights`], so expression doesn't read as pose.
fn make_pnp_solver() -> PnpSolver {
    PnpSolver::new().with_weights(pnp_landmark_weights().to_vec())
}

impl Tracker {
    /// Initialise models and open the webcam.  Must be called from the thread
    /// that will own the `Camera`.
    pub fn new(cfg: &TrackerConfig) -> Result<Self> {
        let model_dir = cfg.model_dir.as_path();
        let scrfd = load_scrfd(model_dir)?;
        let rot_source = read_rotation_source();
        let pose_6drepnet = try_load_6drepnet(model_dir, rot_source);
        let effective_rot_source = if pose_6drepnet.is_some() { rot_source } else { RotationSource::Pnp };
        info!("rotation source: {:?}", effective_rot_source);
        let pnp = make_pnp_solver();

        let (res_w, res_h) = read_capture_resolution(cfg.camera_resolution);
        info!("opening camera {} at {}×{}", cfg.camera_index, res_w, res_h);

        // Some cameras report fractional frame rates that crash nokhwa's format
        // enumeration, or have known nokhwa overhead that makes V4L2 faster.
        // Skip nokhwa entirely for those and go straight to the V4L2 path.
        let force_v4l2 = should_force_v4l2(cfg.camera_index);

        // Try nokhwa first (works for most cameras), unless the camera is known
        // to work better via raw V4L2 (e.g. Delan Cam 1).
        let nokhwa_result: Option<CameraBackend> = if force_v4l2 {
            info!("v4l2: skipping nokhwa for this camera (known V4L2-preferred device)");
            None
        } else {
            let req = RequestedFormat::new::<RgbFormat>(
                RequestedFormatType::Closest(CameraFormat::new(
                    Resolution::new(res_w, res_h),
                    FrameFormat::MJPEG,
                    60,
                )),
            );
            match Camera::new(CameraIndex::Index(cfg.camera_index), req) {
                Ok(mut cam) => {
                    cam.open_stream().context("starting webcam stream")?;
                    // Prevent auto-exposure from dropping fps in low light.
                    // Camera still auto-exposes via gain/ISO within the frame budget.
                    crate::v4l2_capture::disable_framerate_priority(cfg.camera_index);
                    let actual_res = cam.resolution();
                    info!("nokhwa: camera opened {}×{}", actual_res.width(), actual_res.height());
                    Some(CameraBackend::Nokhwa(cam))
                }
                Err(nokhwa_err) => {
                    info!("nokhwa failed ({nokhwa_err:#}), trying raw V4L2 fallback");
                    None
                }
            }
        };

        let camera = match nokhwa_result {
            Some(backend) => backend,
            None => {
                let v4l2 = V4l2Camera::open(cfg.camera_index, res_w, res_h, 60)
                    .context("opening webcam (v4l2 fallback)")?;
                let (w, h) = v4l2.resolution();
                info!("v4l2: camera opened {w}×{h}");
                CameraBackend::V4l2(v4l2)
            }
        };

        // Freeze AE/AGC/AWB after they settle, for cameras that want it (D8):
        // exposure steps shift landmark positions and manufacture pose pops.
        crate::v4l2_capture::maybe_lock_exposure(cfg.camera_index);

        Ok(Self {
            camera,
            pose_6drepnet,
            scrfd,
            rot_source: effective_rot_source,
            trans_source: read_translation_source(),
            pnp,
            last_landmarks: None,
            last_gray: None,
            frames_since_redetect: 0,
            redetect_frames: read_redetect_frames(),
            depth_cue: read_depth_cue(),
            pnp_high_rms_streak: 0,
            // Start "ready" so the first genuine drift trip fires immediately.
            frames_since_forced_redetect: MIN_FRAMES_BETWEEN_FORCED_REDETECT,
            fov_diag_deg: cfg.fov_diag_deg,
            z_gate: crate::z_gate::ZGate::new(),
            rot_hold: RotationHold::default(),
            start: Instant::now(),
            tracked_box: None,
            miss_count: 0,
            step_count: 0,
            stats: PipelineStats::default(),
            frame_dump: crate::frame_dump::FrameDumper::open_if_enabled(),
            frame_facts: crate::frame_dump::FrameFacts::default(),
            capture_errors: CaptureErrors::default(),
            capture_step: None,
        })
    }

    /// Initialise models only — no camera.
    ///
    /// Used by the decoupled YUYV path where a separate capture thread owns
    /// the camera and feeds grayscale frames to the inference thread.
    pub fn new_inference_only(cfg: &TrackerConfig) -> Result<Self> {
        let model_dir = cfg.model_dir.as_path();
        let scrfd = load_scrfd(model_dir)?;
        let rot_source = read_rotation_source();
        let pose_6drepnet = try_load_6drepnet(model_dir, rot_source);
        let effective_rot_source = if pose_6drepnet.is_some() { rot_source } else { RotationSource::Pnp };
        info!("rotation source: {:?}", effective_rot_source);
        let pnp = make_pnp_solver();

        // Dummy camera — will never be used.  The capture thread owns the real one.
        // We need a V4l2Camera to satisfy the enum, but we never call capture().
        // Use a placeholder; the CameraBackend is never accessed.
        Ok(Self {
            camera: CameraBackend::None,
            pose_6drepnet,
            scrfd,
            rot_source: effective_rot_source,
            trans_source: read_translation_source(),
            pnp,
            last_landmarks: None,
            last_gray: None,
            frames_since_redetect: 0,
            redetect_frames: read_redetect_frames(),
            depth_cue: read_depth_cue(),
            pnp_high_rms_streak: 0,
            // Start "ready" so the first genuine drift trip fires immediately.
            frames_since_forced_redetect: MIN_FRAMES_BETWEEN_FORCED_REDETECT,
            fov_diag_deg: cfg.fov_diag_deg,
            z_gate: crate::z_gate::ZGate::new(),
            rot_hold: RotationHold::default(),
            start: Instant::now(),
            tracked_box: None,
            miss_count: 0,
            step_count: 0,
            stats: PipelineStats::default(),
            frame_dump: crate::frame_dump::FrameDumper::open_if_enabled(),
            frame_facts: crate::frame_dump::FrameFacts::default(),
            capture_errors: CaptureErrors::default(),
            capture_step: None,
        })
    }

    /// Returns true if the inference path needs a colour frame on **every**
    /// frame, not just every 3rd preview frame — i.e. when the 6DRepNet
    /// rotation estimator is the active rotation source, since it reads RGB
    /// pixels each frame. The PnP-rotation path works from grayscale and
    /// returns `false`.
    pub fn needs_rgb_every_frame(&self) -> bool {
        self.pose_6drepnet.is_some() && self.rot_source == RotationSource::SixDRepNet
    }


    /// Capture one frame and run the inference pipeline.
    ///
    /// Returns a `RawPose` on success, `None` if no head is found or an
    /// error occurs (errors are logged, not propagated, so the loop continues).
    /// The RGB frame is dropped immediately after grayscale conversion unless
    /// the 6DRepNet rotation estimator is enabled, in which case it stays
    /// alive across inference (~1 MB extra working set).
    pub fn step(&mut self) -> Option<RawPose> {
        let t0 = Instant::now();
        let rgb_frame = match self.capture() {
            Ok(f) => {
                self.capture_ok();
                f
            }
            Err(e) => {
                self.capture_failed(&e);
                return None;
            }
        };

        let t_cap = t0.elapsed();
        if t_cap.as_millis() > 300 {
            warn!("webcam stall: capture took {:.0}ms (camera may have frozen)", t_cap.as_millis());
        }
        let gray = to_gray(&rgb_frame);
        let t_gray = t0.elapsed();

        let rgb_for_inference = if self.needs_rgb_every_frame() { Some(&rgb_frame) } else { None };
        let result = self.run_inference(gray, rgb_for_inference);
        // rgb_frame dropped here once inference is done.
        drop(rgb_frame);
        let t_total = t0.elapsed();

        self.step_count += 1;
        if self.step_count.is_multiple_of(30) {
            debug!(
                "step timing: cap={:.1}ms gray={:.1}ms infer={:.1}ms total={:.1}ms",
                t_cap.as_secs_f64() * 1000.0,
                (t_gray - t_cap).as_secs_f64() * 1000.0,
                (t_total - t_gray).as_secs_f64() * 1000.0,
                t_total.as_secs_f64() * 1000.0,
            );
        }
        result
    }

    /// Like [`step`] but also returns the raw RGB frame for preview.
    pub fn step_with_frame(&mut self) -> StepFrame {
        let rgb_frame = match self.capture() {
            Ok(f) => {
                self.capture_ok();
                f
            }
            Err(e) => {
                self.capture_failed(&e);
                return (None, None);
            }
        };

        let gray = to_gray(&rgb_frame);
        // Note: rgb_frame stays alive here for the preview return.
        // This is intentional — only called every 3rd frame.
        let pose = self.run_inference(gray, Some(&rgb_frame));
        (pose, Some(rgb_frame))
    }

    /// Run inference on a pre-captured grayscale frame.
    ///
    /// Used by the decoupled YUYV capture path where capture and inference
    /// run in separate threads.  When 6DRepNet is enabled the YUYV thread
    /// also passes an RGB copy of the same frame; otherwise `rgb` is `None`.
    pub fn infer_gray(
        &mut self,
        gray: image::GrayImage,
        rgb: Option<&ImageBuffer<Rgb<u8>, Vec<u8>>>,
    ) -> Option<RawPose> {
        self.run_inference(gray, rgb)
    }

    /// The most recent bounding box from the pose estimator.
    pub fn tracked_box(&self) -> Option<Detection> {
        self.tracked_box
    }

    /// Snapshot of the most recent landmark set for the preview overlay
    /// (full-resolution image coords). Returns an empty Vec before any
    /// successful detection. Cheap clone — N≤106.
    pub fn last_landmarks_for_overlay(&self) -> Vec<(f32, f32)> {
        self.last_landmarks.clone().unwrap_or_default()
    }

    /// The inference path: SCRFD + optical flow + PnP.
    ///
    /// Optical flow tracks the 5 SCRFD keypoints frame-to-frame on grayscale;
    /// SCRFD only re-runs every [`REDETECT_THRESHOLD`] frames or on OF/PnP
    /// drift. Cheaper per-frame than detecting every frame, at the cost of
    /// landmark drift between detects — which is what the drift trigger is for.
    ///
    /// Then: PnP solves the landmark set, optional 6DRepNet rotation override,
    /// cache state for next frame.
    ///
    /// `rgb` is the colour version of the same frame; it is only required on
    /// SCRFD re-detect frames (rare) and for the 6DRepNet rotation override,
    /// so it is `None` on most frames.
    fn run_inference(
        &mut self,
        gray: image::GrayImage,
        rgb: Option<&ImageBuffer<Rgb<u8>, Vec<u8>>>,
    ) -> Option<RawPose> {
        if self.frame_dump.is_none() {
            return self.run_inference_inner(gray, rgb);
        }
        // Dump-enabled path. Wrapping rather than writing at each of
        // `run_inference_inner`'s three exits means a miss frame — the most
        // interesting row in the file — is captured like any other.
        self.frame_facts.begin(self.last_landmarks.as_ref());
        let pose = self.run_inference_inner(gray, rgb);
        let timestamp_us = self.start.elapsed().as_micros() as u64;
        let Self { frame_dump, frame_facts, .. } = self;
        if let Some(d) = frame_dump.as_mut() {
            d.write(rgb, pose.as_ref(), frame_facts, timestamp_us);
        }
        pose
    }

    fn run_inference_inner(
        &mut self,
        gray: image::GrayImage,
        rgb: Option<&ImageBuffer<Rgb<u8>, Vec<u8>>>,
    ) -> Option<RawPose> {
        let t_inference_start = Instant::now();
        let (img_w, img_h) = gray.dimensions();

        // Landmark acquisition: optical flow carries the 5 SCRFD keypoints
        // between periodic re-detects, so SCRFD does not run every frame.
        let mut landmarks: Option<Vec<(f32, f32)>> = None;
        let mut bbox_from_detect: Option<Detection> = None;
        let mut tried_optical_flow = false;

        let need_redetect = self.last_landmarks.is_none()
            || self.last_gray.is_none()
            || self.frames_since_redetect >= self.redetect_frames;

        if !need_redetect {
            self.stats.of_attempts += 1;
            landmarks = self.try_optical_flow(&gray);
            tried_optical_flow = true;
            if landmarks.is_some() {
                self.stats.of_success += 1;
                self.frame_facts.source = crate::frame_dump::LandmarkSource::OpticalFlow;
            } else {
                self.frames_since_redetect = self.redetect_frames;
                self.stats.forced_redetects += 1;
            }
        }

        if landmarks.is_none() {
            if let Some(rgb_img) = rgb {
                self.stats.scrfd_attempts += 1;
                self.frame_facts.scrfd_ran = true;
                if let Some(face) = self.run_scrfd(rgb_img) {
                    self.stats.scrfd_success += 1;
                    self.frame_facts.source = crate::frame_dump::LandmarkSource::Detector;
                    self.frame_facts.bbox = Some(face.bbox);
                    bbox_from_detect = Some(face.bbox);
                    landmarks = Some(face.kps);
                }
            } else if !tried_optical_flow {
                self.stats.of_attempts += 1;
                landmarks = self.try_optical_flow(&gray);
                if landmarks.is_some() {
                    self.stats.of_success += 1;
                    self.frame_facts.source = crate::frame_dump::LandmarkSource::OpticalFlow;
                }
            }
        }

        let Some(landmarks) = landmarks else {
            self.stats.misses += 1;
            self.maybe_log_stats(t_inference_start);
            return self.miss();
        };

        if self.frame_dump.is_some() {
            self.frame_facts.kps.extend_from_slice(&landmarks);
        }

        // ── PnP solve on the 5 landmarks ────────────────────────────────────
        let intrinsics = CameraIntrinsics::from_diag_fov(img_w, img_h, self.fov_diag_deg);
        self.stats.pnp_attempts += 1;
        let pnp_result = match self.pnp.solve(&landmarks, &intrinsics) {
            Some(r) => {
                self.stats.pnp_success += 1;
                self.stats.rms_sum += r.rms_px as f64;
                self.stats.rms_max = self.stats.rms_max.max(r.rms_px);
                self.stats.rms_count += 1;
                r
            }
            None => {
                // Solver diverged — treat as a re-detect trigger.
                self.frames_since_redetect = self.redetect_frames;
                self.last_landmarks = None;
                self.last_gray = None;
                self.stats.forced_redetects += 1;
                self.stats.misses += 1;
                self.maybe_log_stats(t_inference_start);
                return self.miss();
            }
        };
        let (mut yaw, mut pitch, mut roll, x_mm, y_mm, z_mm) =
            pose_to_canonical(&pnp_result.r, &pnp_result.t);


        // Drift-based re-detect trigger (Layer 2 of the tracking-loss detector,
        // and the only drift trigger there is).
        //
        // PnP RMS rises when optical flow has carried the landmarks far enough
        // off real features to matter. The threshold depends on what the PnP
        // solve is still responsible for:
        //
        // - Rotation from PnP: tight (~6 px). The 5-point canonical's baseline
        //   RMS is sub-pixel, and a drifting solve corrupts rotation directly.
        // - Rotation from 6DRepNet (the default): loose (25 px). PnP only feeds
        //   translation then, which tolerates much looser landmarks, and the
        //   tight threshold produced metronomic re-anchor "ticks" in X/Y every
        //   ~0.5 s.
        let rms_threshold = if self.rot_source == RotationSource::SixDRepNet {
            // 6DRepNet drives rotation; drift re-detect only guards translation
            // PnP now, so loosen it to stop the metronomic re-anchor ticking.
            PNP_DRIFT_RMS_PX_6DREP
        } else {
            PNP_DRIFT_RMS_PX
        };
        let mut layer2_trip = false;
        if bbox_from_detect.is_none() {
            if pnp_result.rms_px > rms_threshold {
                self.pnp_high_rms_streak = self.pnp_high_rms_streak.saturating_add(1);
                if self.pnp_high_rms_streak >= PNP_DRIFT_FRAMES {
                    layer2_trip = true;
                    self.pnp_high_rms_streak = 0;
                }
            } else {
                self.pnp_high_rms_streak = 0;
            }
        } else {
            // Just re-detected — clear the streak.
            self.pnp_high_rms_streak = 0;
        }

        // ── Optional rotation override via 6DRepNet ─────────────────────────
        let bbox = bbox_from_detect.or_else(|| bbox_from_landmarks(&landmarks));
        if self.rot_source == RotationSource::SixDRepNet {
            if let (Some(est), Some(rgb_img), Some(b)) = (self.pose_6drepnet.as_mut(), rgb, bbox) {
                self.stats.sixdrep_attempts += 1;
                let fresh = match est.run(rgb_img, &b) {
                    Ok(Some(r)) => {
                        self.stats.sixdrep_success += 1;
                        Some(r)
                    }
                    Ok(None) => None, // patch fell outside the frame
                    Err(e) => {
                        self.rot_hold.note_error(&e);
                        None
                    }
                };
                // A dropped frame holds the last 6DRepNet rotation rather than
                // reverting to PnP for one frame: the two sources disagree by
                // degrees, so a single-frame swap is a visible rotation step —
                // and the landmark Z cue is yaw-corrected, so it drags
                // translation with it. Only a sustained outage falls back.
                if let Some(r) = self.rot_hold.resolve(fresh) {
                    yaw = r.yaw;
                    pitch = r.pitch;
                    roll = r.roll;
                }
            }
        }

        // Stable-landmark translation (default): X/Y from the eye centroid and Z
        // from interpupillary distance (yaw-corrected, using the final `yaw`) —
        // robust to expression and pitch where the PnP `tvec` wobbles.
        // Captured here while `landmarks` is still owned (it is moved below into
        // `last_landmarks`); applied at the RawPose build. `HEADTRACK_TRANS_SOURCE
        // =pnp` disables this and keeps the PnP tvec.
        let landmark_xyz = if self.trans_source == TranslationSource::Landmark
            && landmarks.len() >= 2
        {
            if self.depth_cue == crate::landmark_position::DepthCue::Ipd {
                Some(position_from_eyes(
                    landmarks[0],
                    landmarks[1],
                    (img_w, img_h),
                    self.fov_diag_deg,
                    yaw,
                ))
            } else {
                // All three translation axes come from one robust fit over all
                // five landmarks. X/Y were previously the eye centroid — the
                // *same two points* IPD uses — back-projected at the IPD depth,
                // so a single bad eye keypoint corrupted X, Y and Z together and
                // depth noise multiplied into the lateral axes. Falls back to the
                // eye-centroid path if too few pairs survive the guards.
                let rot = crate::landmark_position::rotation_from_canonical(yaw, pitch, roll);
                Some(crate::landmark_position::translation_from_landmarks(
                    &landmarks,
                    &rot,
                    (img_w, img_h),
                    (intrinsics.fx, intrinsics.fy),
                    self.depth_cue,
                )
                .map_or_else(
                    || {
                        position_from_eyes(
                            landmarks[0],
                            landmarks[1],
                            (img_w, img_h),
                            self.fov_diag_deg,
                            yaw,
                        )
                    },
                    |(x, y, z, _quality)| (x, y, z),
                ))
            }
        } else {
            None
        };


        // ── Update state ────────────────────────────────────────────────────
        // Four-branch precedence (top wins): SCRFD authoritative, then
        // tracking-loss recovery (Layers 1+2), then normal landmark-derived
        // update, then a no-bbox hold (only when the landmark set produced no
        // usable AABB). The collapsed-cloud case that the fourth branch guards
        // against came from the dense path's detector, which returned None on
        // collapse; with dense gone, the 5-keypoint set cannot degenerate the
        // same way, but the branch is kept as a cheap invariant.
        // The recovery branch deliberately keeps tracked_box and
        // last_bbox_center frozen at their last good values so the next
        // frame's crop is anchored to the previous known-good location
        // while SCRFD re-acquires.
        // Rate-limit drift-triggered (Layer-2) re-detects: chronic high RMS
        // during a head turn would otherwise trip every few frames and storm
        // SCRFD inference, collapsing the frame rate. Honour a drift trip only
        // once per MIN_FRAMES_BETWEEN_FORCED_REDETECT; in between, keep riding
        // optical flow.
        self.frames_since_forced_redetect =
            self.frames_since_forced_redetect.saturating_add(1);
        let layer2_allowed = layer2_trip
            && self.frames_since_forced_redetect >= MIN_FRAMES_BETWEEN_FORCED_REDETECT;
        if layer2_trip && !layer2_allowed {
            self.stats.suppressed_redetects += 1;
        }
        let force_redetect = layer2_allowed;

        if let Some(b) = bbox_from_detect {
            self.tracked_box = Some(b);
            self.frames_since_redetect = 0;
        } else if force_redetect {
            debug!("tracking-loss (layer2_rms drift) — forcing SCRFD next frame");
            self.frames_since_redetect = self.redetect_frames;
            self.last_landmarks = None;
            self.frames_since_forced_redetect = 0;
            self.stats.forced_redetects += 1;
            // tracked_box and last_bbox_center deliberately not touched.
        } else if let Some(b) = bbox {
            self.tracked_box = Some(b);
            self.frames_since_redetect = self.frames_since_redetect.saturating_add(1);
        } else {
            self.frames_since_redetect = self.frames_since_redetect.saturating_add(1);
        }
        self.miss_count = 0;
        // A forced re-detect clears last_landmarks above; otherwise update it.
        // A *suppressed* drift trip falls through to here and keeps tracking.
        if !force_redetect {
            self.last_landmarks = Some(landmarks);
        }
        self.last_gray = Some(gray);

        self.maybe_log_stats(t_inference_start);

        let timestamp_us = self.start.elapsed().as_micros() as u64;


        // Use the stable-landmark translation captured above when enabled,
        // else the PnP `tvec`.
        let (x, y, z) = landmark_xyz.unwrap_or((x_mm, y_mm, z_mm));

        // Physiological Z gate: reject impossible depth steps (landmark
        // misfires, blinks) and damp the cue at ill-conditioned poses.
        // Landmark path only — `HEADTRACK_TRANS_SOURCE=pnp` A/B stays pure.
        let z = if self.trans_source == TranslationSource::Landmark {
            let (z_out, held) = self.z_gate.filter(z, yaw, pitch, timestamp_us);
            if held {
                self.stats.z_gate_holds += 1;
            }
            z_out
        } else {
            z
        };

        Some(RawPose {
            yaw,
            pitch,
            roll,
            x,
            y,
            z,
            timestamp_us,
        })
    }


    /// Update `stats` with this frame's inference time and emit the rolling
    /// summary every [`STATS_LOG_INTERVAL`] frames. Cheap — just a `Duration`
    /// subtraction and a few u32 increments on the hot path.
    fn maybe_log_stats(&mut self, t_inference_start: Instant) {
        let us = t_inference_start.elapsed().as_micros() as u64;
        self.stats.inference_us_total += us;
        if us > self.stats.inference_us_max {
            self.stats.inference_us_max = us;
        }
        self.stats.frames += 1;

        if self.stats.frames < STATS_LOG_INTERVAL {
            return;
        }

        let s = &self.stats;
        let det = self.scrfd.name();
        let avg_us = if s.frames > 0 { s.inference_us_total / s.frames as u64 } else { 0 };
        let avg_rms = if s.rms_count > 0 { s.rms_sum / s.rms_count as f64 } else { 0.0 };
        // Once per second, every second, for the life of the process — this is
        // the per-frame diagnostic tier, not a lifecycle event.
        debug!(
            "pipeline {}f: OF {}/{} ({:.0}%), {} {}/{}, 6DRep {}/{}, PnP {}/{}, miss {}, redetect {}(+{} suppressed), rms avg {:.1}/max {:.1}px, z-hold {} | infer avg {:.1}ms peak {:.1}ms",
            s.frames,
            s.of_success, s.of_attempts,
            if s.of_attempts > 0 { 100.0 * s.of_success as f32 / s.of_attempts as f32 } else { 0.0 },
            det,
            s.scrfd_success, s.scrfd_attempts,
            s.sixdrep_success, s.sixdrep_attempts,
            s.pnp_success, s.pnp_attempts,
            s.misses,
            s.forced_redetects,
            s.suppressed_redetects,
            avg_rms,
            s.rms_max,
            s.z_gate_holds,
            avg_us as f32 / 1000.0,
            s.inference_us_max as f32 / 1000.0,
        );
        self.stats.reset();
    }


    /// Run SCRFD on `rgb`. Logs and returns `None` on any error or no-face
    /// outcome. Resets the PnP warm-start on success so the solver starts
    /// fresh from the new landmark set rather than dragging stale rotation
    /// state into a disjoint head pose.
    fn run_scrfd(&mut self, rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> Option<crate::scrfd_detector::FaceDetection> {
        let scrfd = &mut self.scrfd;
        match scrfd.detect(rgb) {
            Ok(Some(f)) => {
                self.pnp.reset();
                Some(f)
            }
            Ok(None) => None,
            Err(e) => {
                warn!("SCRFD inference error: {e:#}");
                None
            }
        }
    }

    /// Track the previous frame's landmarks into `gray`. Returns `None` if
    /// any landmark lost confidence (the caller should treat this as a
    /// tracking-lost event and schedule a re-detect). The returned `Vec`
    /// has the same length as `self.last_landmarks`.
    fn try_optical_flow(&self, gray: &GrayImage) -> Option<Vec<(f32, f32)>> {
        let prev_pts = self.last_landmarks.as_ref()?;
        let prev_gray = self.last_gray.as_ref()?;
        if prev_gray.dimensions() != gray.dimensions() {
            return None;
        }
        let tracked = track_landmarks(prev_gray, gray, prev_pts);
        if tracked.iter().any(|p| !p.tracked) {
            return None;
        }
        Some(tracked.into_iter().map(|t| t.point).collect())
    }

    /// Increment miss counter and return `None` (no pose this frame).
    fn miss(&mut self) -> Option<RawPose> {
        self.miss_count = self.miss_count.saturating_add(1);
        None
    }

    fn capture(&mut self) -> Result<ImageBuffer<Rgb<u8>, Vec<u8>>> {
        self.camera.capture()
    }

    /// A frame arrived: any run of capture errors is over.
    fn capture_ok(&mut self) {
        let n = self.capture_errors.ok();
        if n > 0 {
            debug!("webcam capture recovered after {n} error(s)");
        }
        self.capture_step = None;
    }

    /// Log a capture error once per run and record what the loop should do
    /// next (see [`crate::capture_errors`]).
    fn capture_failed(&mut self, e: &anyhow::Error) {
        let now = Instant::now();
        let step = self.capture_errors.error(now, is_device_gone(e));
        match step {
            CaptureStep::Retry { warn: true, .. } => {
                warn!("webcam capture error: {e:#} (retrying with backoff; further errors at DEBUG)");
            }
            CaptureStep::Retry { warn: false, .. } => debug!("webcam capture error: {e:#}"),
            CaptureStep::GiveUp => warn!(
                "webcam capture failed {} time(s) over {:.1}s ({e:#}); giving up — the camera \
                 is gone or unusable. Pick a camera again, or restart, once it is back",
                self.capture_errors.run_len(),
                self.capture_errors.run_age(now).as_secs_f32()
            ),
        }
        self.capture_step = Some(step);
    }

    /// What the last capture error asks of the capture loop, if anything:
    /// `Retry { wait }` to pause before the next step, `GiveUp` to end.
    pub(crate) fn take_capture_step(&mut self) -> Option<CaptureStep> {
        self.capture_step.take()
    }
}

/// Cameras known to perform better via raw V4L2 than nokhwa.
///
/// Reasons vary: nokhwa overhead, fractional framerate enumeration crashes,
/// or format negotiation mismatch.  Matched case-insensitively against the
/// sysfs device name.
const V4L2_PREFERRED: &[&str] = &[
    "delan",  // Delan Cam 1: duplicate YUYV intervals crash nokhwa; V4L2 is ~40% faster
];

/// Synthesise a head bounding box from facial landmarks, used when the
/// current frame was advanced by optical flow rather than a fresh detect
/// (we don't have a real bbox in that case but still want one for the
/// preview overlay and the optional 6DRepNet patch crop).
///
/// Uses the eye-mouth heuristic (indices 0/1/3/4 = eyes
/// + mouth corners) because the SCRFD landmarks don't span the jawline.
fn bbox_from_landmarks(kps: &[(f32, f32)]) -> Option<Detection> {
    if kps.len() < 5 {
        return None;
    }
    let (rex, rey) = kps[0];
    let (lex, ley) = kps[1];
    let (rmx, rmy) = kps[3];
    let (lmx, lmy) = kps[4];

    let eye_cx = (rex + lex) * 0.5;
    let eye_cy = (rey + ley) * 0.5;
    let mouth_cx = (rmx + lmx) * 0.5;
    let mouth_cy = (rmy + lmy) * 0.5;

    let eye_mouth = ((mouth_cx - eye_cx).powi(2) + (mouth_cy - eye_cy).powi(2)).sqrt();
    if !eye_mouth.is_finite() || eye_mouth < 1.0 {
        return None;
    }

    // Whole-head extent ≈ 2.4× eye-mouth distance (anatomical heuristic that
    // matches what SCRFD typically returns for a face crop).
    let side = eye_mouth * 2.4;
    let cx = (eye_cx + mouth_cx) * 0.5;
    let cy = (eye_cy + mouth_cy) * 0.5;

    Some(Detection {
        x: cx - side * 0.5,
        y: cy - side * 0.5,
        w: side,
        h: side,
        score: 1.0,
    })
}





/// Return true if this camera index should skip nokhwa and use raw V4L2.
fn should_force_v4l2(index: u32) -> bool {
    let name = std::fs::read_to_string(
        format!("/sys/class/video4linux/video{index}/name")
    )
    .unwrap_or_default();
    let lower = name.trim().to_lowercase();
    V4L2_PREFERRED.iter().any(|key| lower.contains(key))
}

#[cfg(test)]
mod model_requirement_tests {
    use super::*;

    /// SL3 made SCRFD mandatory. Before it, a missing model silently demoted to
    /// the legacy localizer chain, which is what let a wrong model *path* look
    /// like a broken *camera* for as long as it did (`KNOWN_BUGS.md`, "Models
    /// not found unless launched from the repo root").
    ///
    /// `load_scrfd` runs before any camera work in both constructors, so this
    /// exercises the real startup ordering without needing a webcam.
    /// The re-detect interval is the knob that discriminates optical-flow drift
    /// from a detector fault (`docs/DEPTH_CUE_PLAN.md` Phase 0). It is read from
    /// the environment, so the parse/clamp half is split out to be testable
    /// without `set_var` racing other tests.
    ///
    /// The invariant that matters most: **an absent or malformed value must
    /// yield exactly the shipped default**, so an unset environment always
    /// reproduces production behaviour.
    #[test]
    fn redetect_frames_override_parses_and_clamps() {
        // Absent, empty, whitespace, garbage, negative, overflow → the default.
        for raw in [None, Some(""), Some("   "), Some("abc"), Some("-1"), Some("1e9"),
                    Some("99999999999999999999")] {
            assert_eq!(
                redetect_frames_from(raw),
                REDETECT_THRESHOLD,
                "{raw:?} must fall back to the shipped default"
            );
        }
        // Valid values pass through, with surrounding whitespace tolerated.
        assert_eq!(redetect_frames_from(Some("60")), 60);
        assert_eq!(redetect_frames_from(Some(" 60 ")), 60);
        assert_eq!(redetect_frames_from(Some("300")), REDETECT_THRESHOLD);
        // Out of range clamps rather than erroring — this knob must never stop
        // the tracker starting.
        assert_eq!(redetect_frames_from(Some("0")), REDETECT_FRAMES_MIN);
        assert_eq!(redetect_frames_from(Some("5")), REDETECT_FRAMES_MIN);
        assert_eq!(redetect_frames_from(Some("4000000000")), REDETECT_FRAMES_MAX);
    }

    /// A temp model directory that holds nothing.
    fn empty_model_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("ht-p01-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    /// Unwrap a startup error into the typed MissingModel (Tracker and
    /// FaceDetector have no Debug impl, so no `unwrap_err`).
    fn missing_model_of<T>(r: Result<T>) -> anyhow::Error {
        match r {
            Ok(_) => panic!("a missing model must not start the tracker"),
            Err(e) => e,
        }
    }

    #[test]
    fn a_missing_default_model_fails_startup_with_a_typed_error() {
        let empty = empty_model_dir("default");
        let cfg = TrackerConfig {
            model_dir: empty.clone(),
            camera_index: 0,
            fov_diag_deg: 78.0,
            camera_resolution: headtrack_core::config::CameraConfig::default_resolution(),
        };
        // Only meaningful if the environment is not opting into SCRFD.
        if read_detector_choice() == "yunet" {
            let err = missing_model_of(Tracker::new_inference_only(&cfg));
            let missing = err
                .downcast_ref::<MissingModel>()
                .expect("must be a MissingModel so the engine can tell it from a camera failure");
            assert_eq!(missing.filename, crate::yunet_detector::MODEL_FILENAME);
            assert!(missing.note.is_none(), "a bundled model needs no opt-in note");
            let text = format!("{missing}");
            assert!(text.contains(crate::yunet_detector::MODEL_FILENAME), "names the model: {text}");
            assert!(text.contains("HEADTRACK_MODELS"), "names the override: {text}");
        }
        let _ = std::fs::remove_dir_all(&empty);
    }

    /// `HEADTRACK_DETECTOR=scrfd` without the user-supplied file: a typed error
    /// that names the file, where it goes, and why it is not shipped. No panic.
    #[test]
    fn scrfd_opt_in_without_the_file_explains_itself() {
        let empty = empty_model_dir("scrfd");
        let err = missing_model_of(load_detector(&empty, "scrfd"));
        let missing = err.downcast_ref::<MissingModel>().expect("typed MissingModel");
        assert_eq!(missing.filename, MODEL_SCRFD);
        let text = format!("{missing}");
        assert!(text.contains(MODEL_SCRFD), "names the file: {text}");
        assert!(text.contains(&empty.display().to_string()), "says where it looked: {text}");
        assert!(text.contains("model directory"), "says where to put it: {text}");
        assert!(text.contains("non-commercial"), "states the licence: {text}");
        let _ = std::fs::remove_dir_all(&empty);
    }

    #[test]
    fn detector_choice_defaults_to_yunet() {
        assert_eq!(detector_choice_from(None), "yunet");
        assert_eq!(detector_choice_from(Some("")), "yunet");
        assert_eq!(detector_choice_from(Some("yunet")), "yunet");
        assert_eq!(detector_choice_from(Some(" SCRFD ")), "scrfd");
        assert_eq!(detector_choice_from(Some("retinaface")), "yunet", "unknown falls back to yunet");
    }

    /// The shipped bundle (YuNet + 6DRepNet, no SCRFD) starts the default detector.
    #[test]
    fn shipped_bundle_loads_yunet() {
        let models = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/models");
        assert!(
            !models.join(MODEL_SCRFD).exists(),
            "SCRFD must not be in assets/models (P01: archive/models/)"
        );
        let det = match load_detector(&models, "yunet") {
            Ok(d) => d,
            Err(e) => panic!("bundled YuNet must load: {e:#}"),
        };
        assert_eq!(det.name(), "YuNet");
    }
}

#[cfg(test)]
mod capture_resolution_tests {
    use super::read_capture_resolution;

    const DEFAULT: (u32, u32) = (960, 540);

    fn with(val: Option<&str>) -> (u32, u32) {
        // Serialised by the caller; env is process-global.
        match val {
            Some(v) => std::env::set_var("HEADTRACK_CAPTURE_RES", v),
            None => std::env::remove_var("HEADTRACK_CAPTURE_RES"),
        }
        let got = read_capture_resolution(DEFAULT);
        std::env::remove_var("HEADTRACK_CAPTURE_RES");
        got
    }

    #[test]
    fn parses_and_rejects() {
        assert_eq!(with(None), DEFAULT, "unset keeps the configured value");
        assert_eq!(with(Some("1920x1080")), (1920, 1080));
        assert_eq!(with(Some(" 1280 X 720 ")), (1280, 720), "tolerant of spacing and case");

        // Anything unparseable or implausible must fall back rather than open the
        // camera at a size that will fail or crawl.
        for bad in ["", "1920", "1920*1080", "abcxdef", "0x0", "8000x5000", "-1x-1"] {
            assert_eq!(with(Some(bad)), DEFAULT, "{bad:?} should fall back");
        }
    }
}
