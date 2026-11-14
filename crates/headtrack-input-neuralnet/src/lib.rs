//! Webcam AI head tracking via ONNX Runtime.
//!
//! ## Pipeline
//!
//! A background thread owns the camera and the ONNX sessions.  On each webcam
//! frame it runs:
//!
//! 1. **YuNet** (`face_detection_yunet_2023mar.onnx`, MIT) — detects the face,
//!    returning a bounding box and 5 keypoints. It does *not* run every frame:
//!    optical flow carries the keypoints between detects, and the detector
//!    re-runs on a fixed interval or when drift is detected. SCRFD
//!    (`scrfd-det-10g.onnx`) is opt-in via `HEADTRACK_DETECTOR=scrfd` with a
//!    user-supplied file — its weights are non-commercial research only, so it
//!    is not bundled (P01, 2026-10-03; archived in `archive/models/`).
//!
//! 2. **PnP** — solves the 5-keypoint set against a canonical 3D face model for
//!    rotation and translation.
//!
//! 3. **6DRepNet** (`head-pose-mobilenetv3-large.onnx`) — regresses head
//!    orientation directly and overrides PnP's rotation. This is the default;
//!    `HEADTRACK_ROT_SOURCE=pnp` falls back to the geometric solve.
//!
//! 4. **Stable-landmark translation** — X/Y from the eye centroid, Z from
//!    yaw-corrected interpupillary distance, then a physiological rate gate
//!    (`z_gate`). `HEADTRACK_TRANS_SOURCE=pnp` falls back to the PnP `tvec`.
//!
//! `poll()` drains to the latest pose without blocking.
//!
//! ## ONNX Runtime
//!
//! `ort` requires ONNX Runtime at link time.  For development set
//! `ORT_DYLIB_PATH=/path/to/libonnxruntime.so.1.x.x` at build time, or
//! install via your distro (`dnf install onnxruntime`).
//!
//! The AppImage build will bundle the runtime; see `packaging/appimage/`.
//!
//! ## Model files
//!
//! Both are required; place them in `assets/models/`:
//! - `face_detection_yunet_2023mar.onnx` — face detector (OpenCV Zoo, MIT)
//! - `head-pose-mobilenetv3-large.onnx` — 6DRepNet rotation estimator
//!
//! SL3 deleted the legacy `head-localizer.onnx` + `head-pose-0.4-small-f32.onnx`
//! chain; a missing detector model is now a startup error rather than a silent
//! demotion to that path.

mod capture_errors;
mod frame_dump;
mod image_proc;
mod landmark_position;
mod z_gate;
mod optical_flow;
mod ort_session;
mod pnp;
mod pose_estimator_6drepnet;
// Exposed (hidden) so `examples/detect_frames.rs` can run a detector over saved
// frames without a camera — the offline half of the detector bake-off, and the
// only way to verify the Rust decode against OpenCV's reference implementation.
// Same role as headtrack-core's `examples/replay.rs`. Not a stable surface.
#[doc(hidden)]
pub mod scrfd_detector;
mod tracker;
mod v4l2_capture;
#[doc(hidden)]
pub mod yunet_detector;

use anyhow::Context;
use headtrack_core::{InputSource, RawPose};
use image::{ImageBuffer, Rgb};
use std::{
    path::Path,
    sync::{
        atomic::{AtomicBool, AtomicPtr, Ordering},
        mpsc::{self, Receiver},
        Arc, Condvar, Mutex,
    },
    thread,
    time::{Duration, Instant},
};
use tracing::{debug, info};
use scrfd_detector::Detection;
use capture_errors::{is_device_gone, send_pose, sleep_unless_shutdown, CaptureErrors, CaptureStep};
pub use tracker::MissingModel;

/// Name of the face detector this process will use — `"yunet"` (default) or `"scrfd"`.
///
/// Resolves `HEADTRACK_DETECTOR` through the same function the tracker itself
/// uses, so it reports what is *actually* running rather than what was asked
/// for: an unrecognised value falls back to `yunet` here exactly as it does
/// there.
///
/// Exists so captured data is self-describing. A diagnostic CSV that does not
/// record which detector produced it is worthless the moment a second detector
/// exists — you cannot tell an A from a B a week later, and the whole point of
/// the recordings is comparing them.
pub fn active_detector_name() -> &'static str {
    tracker::read_detector_choice()
}
use tracker::{Tracker, TrackerConfig};
use v4l2_capture::{V4l2YuyvCamera, yuyv_to_gray, yuyv_to_rgb};

/// Channel element type for frames passed to the preview encoder thread.
/// `(rgb, bbox, landmarks)` — landmarks may be empty when no fresh detection
/// happened this frame.
type PreviewFrame_ = (
    ImageBuffer<Rgb<u8>, Vec<u8>>,
    Option<Detection>,
    Vec<(f32, f32)>,
);

/// Lock-free single-producer single-consumer preview frame slot.
///
/// The encoder thread stores new JPEG frames, the preview-writer thread
/// takes the latest. Uses atomic pointer swap — no mutex, no poisoning.
pub struct PreviewSlot {
    ptr: AtomicPtr<Vec<u8>>,
}

impl Default for PreviewSlot {
    fn default() -> Self { Self::new() }
}

impl PreviewSlot {
    pub(crate) fn new() -> Self {
        Self { ptr: AtomicPtr::new(std::ptr::null_mut()) }
    }

    /// Store a new JPEG frame. Frees the previous frame if any.
    pub(crate) fn store(&self, jpeg: Vec<u8>) {
        let boxed = Box::into_raw(Box::new(jpeg));
        let old = self.ptr.swap(boxed, Ordering::AcqRel);
        if !old.is_null() {
            unsafe { drop(Box::from_raw(old)); }
        }
    }

    /// Take the latest frame, leaving the slot empty.
    /// Returns None if no new frame since last take.
    pub fn take(&self) -> Option<Vec<u8>> {
        let ptr = self.ptr.swap(std::ptr::null_mut(), Ordering::AcqRel);
        if ptr.is_null() {
            None
        } else {
            Some(unsafe { *Box::from_raw(ptr) })
        }
    }
}

impl Drop for PreviewSlot {
    fn drop(&mut self) {
        let ptr = *self.ptr.get_mut();
        if !ptr.is_null() {
            unsafe { drop(Box::from_raw(ptr)); }
        }
    }
}

// SAFETY: AtomicPtr provides atomic access. Each Box is exclusively
// owned between swap operations — no aliasing possible.
unsafe impl Send for PreviewSlot {}
unsafe impl Sync for PreviewSlot {}

/// Shared preview frame (JPEG bytes).  Updated by the encoder thread,
/// read by the daemon for the GUI camera preview.
pub type PreviewFrame = Arc<PreviewSlot>;

/// Shared latest-frame slot for decoupled capture/inference threads.
///
/// The capture thread stores the most recent YUYV buffer.  The inference
/// thread waits for a new frame via condvar, takes it, and processes it.
/// If the inference thread is slower than capture, intermediate frames are
/// silently dropped (always process the freshest data).
struct LatestFrame {
    data: Mutex<Option<Vec<u8>>>,
    ready: Condvar,
}

impl LatestFrame {
    fn new() -> Self {
        Self {
            data: Mutex::new(None),
            ready: Condvar::new(),
        }
    }

    /// Store a new frame, replacing any unconsumed previous frame.
    fn store(&self, frame: Vec<u8>) {
        let mut slot = self.data.lock().unwrap_or_else(|e| e.into_inner());
        *slot = Some(frame);
        self.ready.notify_one();
    }

    /// Block until a new frame is available or `shutdown` is set.
    fn take_or_shutdown(&self, shutdown: &AtomicBool) -> Option<Vec<u8>> {
        let mut slot = self.data.lock().unwrap_or_else(|e| e.into_inner());
        loop {
            if shutdown.load(Ordering::Acquire) {
                return None;
            }
            if let Some(frame) = slot.take() {
                return Some(frame);
            }
            // Wake periodically to check shutdown flag.
            let (guard, _) = self.ready.wait_timeout(slot, Duration::from_millis(100)).unwrap();
            slot = guard;
        }
    }
}

/// Cameras known to benefit from YUYV capture (no JPEG decode overhead).
///
/// YUYV is preferred when the camera produces huge MJPEG frames that take
/// longer to decode than the inference itself (e.g. DC1: 600KB JPEG, 88ms decode).
const YUYV_PREFERRED: &[&str] = &[
    "delan", // Delan Cam 1: 600KB MJPEG frames, 88ms decode → 10fps. YUYV: 0ms.
];

/// Return true if this camera should use YUYV capture with decoupled threads.
fn should_use_yuyv(camera_index: u32) -> bool {
    let name = std::fs::read_to_string(
        format!("/sys/class/video4linux/video{camera_index}/name"),
    )
    .unwrap_or_default();
    let lower = name.trim().to_lowercase();
    YUYV_PREFERRED.iter().any(|key| lower.contains(key))
}

/// Webcam AI head tracking — implements [`InputSource`].
pub struct NeuralNetInput {
    rx: Receiver<RawPose>,
    preview: PreviewFrame,
    overlay_enabled: Arc<AtomicBool>,
    /// Set to `true` on drop — signals all background threads to exit.
    shutdown: Arc<AtomicBool>,
    /// The tracker thread's channel closed and that has been logged; `poll`
    /// runs every 4 ms and must not repeat it (FINAL_REVIEW S2).
    thread_lost: bool,
}

impl Drop for NeuralNetInput {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
    }
}

impl NeuralNetInput {
    /// Initialise the tracker.
    ///
    /// - `model_dir` — directory containing both ONNX model files.
    ///   Typically `assets/models/`.
    /// - `camera_index` — V4L2 camera index (0 = first webcam).
    /// - `fov_diag_deg` — diagonal FOV of the webcam in degrees.
    ///   Use `78.0` as a reasonable default (matches Logitech C920).
    ///
    /// The ONNX sessions and webcam are opened **inside** the background
    /// thread (nokhwa's `Camera` is not `Send`).  Initialisation errors are
    /// propagated back to the caller via a rendezvous channel before `new`
    /// returns.
    pub fn new(
        model_dir: &Path,
        camera_index: u32,
        fov_diag_deg: f32,
    ) -> anyhow::Result<Self> {
        Self::with_overlay(model_dir, camera_index, fov_diag_deg, Arc::new(AtomicBool::new(false)))
    }

    /// Like [`new`] but accepts an external overlay toggle flag.
    /// The flag is shared with the encoder thread — setting it to `true`
    /// draws the head bounding box on preview frames.
    pub fn with_overlay(
        model_dir: &Path,
        camera_index: u32,
        fov_diag_deg: f32,
        overlay_enabled: Arc<AtomicBool>,
    ) -> anyhow::Result<Self> {
        if should_use_yuyv(camera_index) {
            Self::spawn_yuyv(model_dir, camera_index, fov_diag_deg, overlay_enabled)
        } else {
            Self::spawn_mjpeg(model_dir, camera_index, fov_diag_deg, overlay_enabled)
        }
    }

    /// MJPEG path — single tracker thread (capture + inference synchronous).
    /// Used for cameras where JPEG decode is cheap (small frames, e.g. C920).
    fn spawn_mjpeg(
        model_dir: &Path,
        camera_index: u32,
        fov_diag_deg: f32,
        overlay_enabled: Arc<AtomicBool>,
    ) -> anyhow::Result<Self> {
        let cfg = TrackerConfig::from_model_dir(model_dir, camera_index, fov_diag_deg);

        let (pose_tx, pose_rx) = mpsc::sync_channel::<RawPose>(64);
        let (init_tx, init_rx) = mpsc::sync_channel::<anyhow::Result<()>>(0);

        let preview: PreviewFrame = Arc::new(PreviewSlot::new());
        let preview_writer = Arc::clone(&preview);
        let overlay_flag = Arc::clone(&overlay_enabled);

        let (frame_tx, frame_rx) = mpsc::sync_channel::<PreviewFrame_>(1);

        // Encoder thread — JPEG-encodes preview frames off the hot tracking path.
        spawn_preview_encoder(frame_rx, preview_writer, overlay_flag)?;

        // The thread must see the same flag `Drop` sets. It used to get none,
        // so a camera switch left it running, camera and ORT sessions open,
        // until it happened to fail a send (FINAL_REVIEW S2).
        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_tracker = Arc::clone(&shutdown);

        thread::Builder::new()
            .name("neuralnet-tracker".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let tracker = match Tracker::new(&cfg) {
                        Ok(t) => {
                            let _ = init_tx.send(Ok(()));
                            t
                        }
                        Err(e) => {
                            let _ = init_tx.send(Err(e));
                            return;
                        }
                    };
                    run_mjpeg_tracker_loop(tracker, pose_tx, frame_tx, &shutdown_tracker);
                }));
                if let Err(e) = result {
                    let msg = e.downcast_ref::<&str>().copied()
                        .or_else(|| e.downcast_ref::<String>().map(|s| s.as_str()))
                        .unwrap_or("unknown panic payload");
                    tracing::error!("neuralnet-tracker thread panicked: {msg}");
                }
            })
            .context("spawning tracker thread")?;

        init_rx
            .recv()
            .context("tracker thread exited before signalling init")?
            .context("tracker initialisation failed")?;

        Ok(Self { rx: pose_rx, preview, overlay_enabled, shutdown, thread_lost: false })
    }

    /// YUYV path — decoupled capture and inference threads.
    ///
    /// Capture thread: grabs YUYV frames at full camera rate (30-60 fps),
    /// stores the latest in a shared slot.  Never blocked by inference.
    ///
    /// Inference thread: takes the freshest frame, extracts Y channel
    /// (= grayscale, zero computation), runs ONNX inference.  Always
    /// processes the most recent data — no stale-frame latency.
    ///
    /// Preview: every 3rd inference frame, also converts YUYV→RGB for
    /// the GUI camera preview.
    fn spawn_yuyv(
        model_dir: &Path,
        camera_index: u32,
        fov_diag_deg: f32,
        overlay_enabled: Arc<AtomicBool>,
    ) -> anyhow::Result<Self> {
        info!("using decoupled YUYV capture for camera {camera_index}");

        let cfg = TrackerConfig::from_model_dir(model_dir, camera_index, fov_diag_deg);
        let (res_w, res_h) = cfg.camera_resolution;

        let (pose_tx, pose_rx) = mpsc::sync_channel::<RawPose>(64);

        let shutdown = Arc::new(AtomicBool::new(false));

        let preview: PreviewFrame = Arc::new(PreviewSlot::new());
        let preview_writer = Arc::clone(&preview);
        let overlay_flag = Arc::clone(&overlay_enabled);

        let (frame_tx, frame_rx) = mpsc::sync_channel::<PreviewFrame_>(1);

        // Encoder thread — same as MJPEG path.
        spawn_preview_encoder(frame_rx, preview_writer, overlay_flag)?;

        // Shared latest-frame slot between capture and inference threads.
        let latest = Arc::new(LatestFrame::new());
        let latest_writer = Arc::clone(&latest);

        // Capture thread — grabs YUYV frames at full camera rate.
        let (cam_init_tx, cam_init_rx) = mpsc::sync_channel::<anyhow::Result<()>>(0);
        let shutdown_capture = Arc::clone(&shutdown);
        thread::Builder::new()
            .name("yuyv-capture".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let mut cam = match V4l2YuyvCamera::open(camera_index, res_w, res_h, 60) {
                        Ok(c) => {
                            let (w, h) = c.resolution();
                            info!("yuyv capture: camera opened {w}×{h}");
                            c
                        }
                        Err(e) => {
                            let _ = cam_init_tx.send(Err(e));
                            return;
                        }
                    };
                    let _ = cam_init_tx.send(Ok(()));

                    let mut errors = CaptureErrors::default();
                    loop {
                        if shutdown_capture.load(Ordering::Acquire) {
                            info!("yuyv capture: shutdown");
                            break;
                        }
                        match cam.frame_raw() {
                            Ok(yuyv) => {
                                let n = errors.ok();
                                if n > 0 {
                                    debug!("yuyv capture recovered after {n} error(s)");
                                }
                                latest_writer.store(yuyv);
                            }
                            Err(e) => {
                                let now = Instant::now();
                                match errors.error(now, is_device_gone(&e)) {
                                    CaptureStep::Retry { warn, wait } => {
                                        if warn {
                                            tracing::warn!(
                                                "yuyv capture error: {e:#} (retrying with \
                                                 backoff; further errors at DEBUG)"
                                            );
                                        } else {
                                            debug!("yuyv capture error: {e:#}");
                                        }
                                        if sleep_unless_shutdown(wait, &shutdown_capture) {
                                            info!("yuyv capture: shutdown");
                                            break;
                                        }
                                    }
                                    CaptureStep::GiveUp => {
                                        tracing::warn!(
                                            "yuyv capture failed {} time(s) over {:.1}s ({e:#}); \
                                             giving up — the camera is gone or unusable. Pick a \
                                             camera again, or restart, once it is back",
                                            errors.run_len(),
                                            errors.run_age(now).as_secs_f32()
                                        );
                                        // End the inference thread too, so the
                                        // pose channel closes and the engine
                                        // sees the source is gone.
                                        shutdown_capture.store(true, Ordering::Release);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }));
                if let Err(e) = result {
                    let msg = e.downcast_ref::<&str>().copied()
                        .or_else(|| e.downcast_ref::<String>().map(|s| s.as_str()))
                        .unwrap_or("unknown panic payload");
                    tracing::error!("yuyv-capture thread panicked: {msg}");
                }
            })
            .context("spawning yuyv capture thread")?;

        // Wait for camera init before spawning inference.
        cam_init_rx
            .recv()
            .context("capture thread exited before signalling init")?
            .context("yuyv camera open failed")?;

        // Inference thread init channel.
        let (model_init_tx, model_init_rx) = mpsc::sync_channel::<anyhow::Result<()>>(0);

        // Inference thread — takes latest YUYV frame, extracts Y, runs ONNX.
        let shutdown_infer = Arc::clone(&shutdown);
        thread::Builder::new()
            .name("neuralnet-inference".into())
            .spawn(move || {
                let mut tracker = match Tracker::new_inference_only(&cfg) {
                    Ok(t) => {
                        let _ = model_init_tx.send(Ok(()));
                        t
                    }
                    Err(e) => {
                        let _ = model_init_tx.send(Err(e));
                        return;
                    }
                };

                let start = Instant::now();
                let mut frame_counter: u32 = 0;
                let mut step_count: u32 = 0;
                let needs_rgb_every_frame = tracker.needs_rgb_every_frame();

                loop {
                    let yuyv = match latest.take_or_shutdown(&shutdown_infer) {
                        Some(y) => y,
                        None => {
                            info!("yuyv inference: shutdown");
                            break;
                        }
                    };

                    let t0 = Instant::now();
                    let gray = yuyv_to_gray(&yuyv, res_w, res_h);
                    let t_gray = t0.elapsed();

                    // RGB is needed every 3rd frame for the preview encoder, and
                    // every frame when 6DRepNet is enabled (it consumes color
                    // input). When neither applies, skip the YUYV→RGB conversion
                    // to save ~3 ms of work per frame.
                    frame_counter = frame_counter.wrapping_add(1);
                    let needs_rgb = needs_rgb_every_frame || frame_counter.is_multiple_of(3);
                    let rgb = if needs_rgb {
                        Some(yuyv_to_rgb(&yuyv, res_w, res_h))
                    } else {
                        None
                    };
                    drop(yuyv); // free 614KB YUYV buffer

                    let pose = tracker.infer_gray(gray, rgb.as_ref());
                    let t_total = t0.elapsed();

                    step_count += 1;
                    if step_count.is_multiple_of(30) {
                        debug!(
                            "yuyv step: gray={:.1}ms infer={:.1}ms total={:.1}ms",
                            t_gray.as_secs_f64() * 1000.0,
                            (t_total - t_gray).as_secs_f64() * 1000.0,
                            t_total.as_secs_f64() * 1000.0,
                        );
                    }

                    if let Some(p) = pose {
                        let raw = RawPose {
                            timestamp_us: start.elapsed().as_micros() as u64,
                            ..p
                        };
                        if !send_pose(&pose_tx, raw) {
                            break;
                        }
                        // Only forward to the preview encoder every 3rd frame,
                        // regardless of whether 6DRepNet also consumed the RGB.
                        if let Some(rgb) = rgb {
                            if frame_counter.is_multiple_of(3) {
                                let det = tracker.tracked_box();
                                let lmks = tracker.last_landmarks_for_overlay();
                                let _ = frame_tx.try_send((rgb, det, lmks));
                            }
                        }
                    }
                }
            })
            .context("spawning inference thread")?;

        model_init_rx
            .recv()
            .context("inference thread exited before signalling init")?
            .context("model loading failed")?;

        Ok(Self { rx: pose_rx, preview, overlay_enabled, shutdown, thread_lost: false })
    }

    /// Get a handle to the latest preview frame (JPEG bytes).
    pub fn preview_frame(&self) -> &PreviewFrame {
        &self.preview
    }

    /// Shared flag controlling bounding box overlay on preview frames.
    pub fn overlay_enabled(&self) -> &Arc<AtomicBool> {
        &self.overlay_enabled
    }
}

impl InputSource for NeuralNetInput {
    fn name(&self) -> &str {
        "neuralnet-webcam"
    }

    /// Returns the most recent pose frame, draining any backlog.
    ///
    /// Returns `None` when no new frame has arrived since the last call —
    /// normal behaviour when the daemon ticks faster than the webcam.
    fn poll(&mut self) -> Option<RawPose> {
        use std::sync::mpsc::TryRecvError;
        let mut latest = None;
        loop {
            match self.rx.try_recv() {
                Ok(pose) => latest = Some(pose),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    // Once per source, not once per 4 ms tick (FINAL_REVIEW S2).
                    if !std::mem::replace(&mut self.thread_lost, true) {
                        tracing::error!(
                            "tracker thread exited unexpectedly — camera disconnected or thread panicked"
                        );
                    }
                    break;
                }
            }
        }
        latest
    }

    /// Estimated webcam capture + ONNX inference latency.
    ///
    /// Typical: ~15 ms on a modern CPU without GPU acceleration.
    /// The prediction stage uses this to compensate in its lookahead.
    fn latency_hint_us(&self) -> u32 {
        15_000
    }
}

/// Inner loop for the MJPEG tracker thread.
///
/// Runs `step_with_frame` every 3rd iteration (to supply the preview encoder)
/// and the faster `step` on interleaved frames to keep pose rate high.
/// Returns when `shutdown` is set (the source was dropped), when the pose
/// channel is closed (consumer dropped), or when capture gives up
/// ([`capture_errors`]). A full channel drops the pose and carries on.
fn run_mjpeg_tracker_loop(
    mut tracker: Tracker,
    pose_tx: mpsc::SyncSender<RawPose>,
    frame_tx: mpsc::SyncSender<PreviewFrame_>,
    shutdown: &AtomicBool,
) {
    let mut frame_counter: u32 = 0;
    loop {
        if shutdown.load(Ordering::Acquire) {
            info!("neuralnet tracker: shutdown");
            break;
        }
        frame_counter = frame_counter.wrapping_add(1);

        if frame_counter.is_multiple_of(3) {
            let (pose, frame) = tracker.step_with_frame();
            if let Some(pose) = pose {
                if !send_pose(&pose_tx, pose) { break; }
            }
            if let Some(rgb) = frame {
                let det = tracker.tracked_box();
                let lmks = tracker.last_landmarks_for_overlay();
                let _ = frame_tx.try_send((rgb, det, lmks));
            }
        } else if let Some(pose) = tracker.step() {
            if !send_pose(&pose_tx, pose) { break; }
        }

        match tracker.take_capture_step() {
            None | Some(CaptureStep::Retry { wait: Duration::ZERO, .. }) => {}
            Some(CaptureStep::Retry { wait, .. }) => {
                if sleep_unless_shutdown(wait, shutdown) {
                    info!("neuralnet tracker: shutdown");
                    break;
                }
            }
            // Ending the thread closes the pose channel: the engine sees it.
            Some(CaptureStep::GiveUp) => break,
        }
    }
}

/// Spawn the JPEG preview encoder thread shared by both MJPEG and YUYV paths.
///
/// Receives `(rgb, detection)` frames, JPEG-encodes them off the hot tracking
/// path, and stores the result into `preview_writer`.
fn spawn_preview_encoder(
    frame_rx: mpsc::Receiver<PreviewFrame_>,
    preview_writer: PreviewFrame,
    overlay_flag: Arc<AtomicBool>,
) -> anyhow::Result<()> {
    thread::Builder::new()
        .name("preview-encoder".into())
        .spawn(move || {
            while let Ok((rgb, detection, landmarks)) = frame_rx.recv() {
                let (det, lmks) = if overlay_flag.load(Ordering::Relaxed) {
                    (detection.as_ref(), landmarks.as_slice())
                } else {
                    (None, &[][..])
                };
                if let Ok(jpeg) = encode_jpeg_preview(&rgb, det, lmks) {
                    preview_writer.store(jpeg);
                }
            }
        })
        .context("spawning preview encoder thread")?;
    Ok(())
}

/// Encode an RGB image to JPEG for preview display.
///
/// Downscales to at most 640 px wide (never upscaled), **preserving the source
/// aspect ratio**, with a Triangle filter at quality 80. It was 320 px,
/// nearest-neighbour, quality 60, which the GUI then stretched to a column
/// several times that width: blocky, with aliased edges (6e). The cost rose
/// from ~2.5-2.9 ms to ~11-16 ms per frame (960×540…1920×1080, release),
/// on this thread, not the tracking loop's.
///
/// The height was fixed at 240 for a long time, which silently anamorphosed
/// every non-4:3 camera: the Elgato Facecam's native 960×540 was squashed by
/// 1.33× horizontally, so faces in the preview were a third narrower than
/// reality. Tracking was unaffected (it reads the full-res
/// frame, never this thumbnail) — but the preview is what a user checks their
/// framing against.
///
/// If a `Detection` is provided, draws the head bounding box on the thumbnail.
/// If `landmarks` is non-empty, draws each landmark as a small dot — used by
/// the Phase A diagnostic to visually verify that the 2D106 model's points
/// stick to fixed anatomical features during head rotation. Without that
/// confirmation, no downstream PnP / bundle-adjustment fix can succeed.
fn encode_jpeg_preview(
    rgb: &image::ImageBuffer<image::Rgb<u8>, Vec<u8>>,
    detection: Option<&Detection>,
    landmarks: &[(f32, f32)],
) -> anyhow::Result<Vec<u8>> {
    use image::imageops::FilterType;
    use std::io::Cursor;

    const MAX_THUMB_W: u32 = 640;
    const QUALITY: u8 = 80;

    // Never wider than the source: upscaling adds bytes, not detail.
    let thumb_w = MAX_THUMB_W.min(rgb.width()).max(1);
    // Height follows the source, so the scale is uniform on both axes and the
    // preview shows the camera's real geometry.
    let thumb_h = ((thumb_w as f32 * rgb.height() as f32) / rgb.width().max(1) as f32)
        .round()
        .max(1.0) as u32;

    let mut thumb = image::imageops::resize(rgb, thumb_w, thumb_h, FilterType::Triangle);
    let scale_x = thumb_w as f32 / rgb.width() as f32;
    let scale_y = thumb_h as f32 / rgb.height() as f32;

    // Draw bounding box overlay if we have a detection.
    if let Some(det) = detection {
        draw_rect(
            &mut thumb,
            (det.x * scale_x) as i32,
            (det.y * scale_y) as i32,
            (det.w * scale_x) as u32,
            (det.h * scale_y) as u32,
            image::Rgb([0, 255, 0]),
        );
    }

    // Draw each landmark as a 3×3 magenta dot so they pop on faces of any
    // skin tone. Index 0 (chin) gets a different colour so the user can
    // visually anchor their orientation while sweeping the head.
    for (i, &(x, y)) in landmarks.iter().enumerate() {
        let tx = (x * scale_x) as i32;
        let ty = (y * scale_y) as i32;
        let colour = if i == 0 {
            image::Rgb([255, 255, 0]) // yellow chin
        } else if i == 86 {
            image::Rgb([0, 255, 255]) // cyan nose tip
        } else {
            image::Rgb([255, 0, 255]) // magenta everything else
        };
        draw_dot(&mut thumb, tx, ty, colour);
    }

    let mut buf = Cursor::new(Vec::new());
    let encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut buf, QUALITY);
    image::ImageEncoder::write_image(
        encoder,
        thumb.as_raw(),
        thumb_w,
        thumb_h,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(buf.into_inner())
}

/// Draw a 3×3 filled dot centred at `(x, y)`, clamped to image bounds.
fn draw_dot(img: &mut image::ImageBuffer<image::Rgb<u8>, Vec<u8>>, x: i32, y: i32, color: image::Rgb<u8>) {
    let (iw, ih) = (img.width() as i32, img.height() as i32);
    for dy in -1..=1 {
        for dx in -1..=1 {
            let px = x + dx;
            let py = y + dy;
            if px >= 0 && px < iw && py >= 0 && py < ih {
                img.put_pixel(px as u32, py as u32, color);
            }
        }
    }
}

/// Draw a single horizontal line segment at row `y` from column `x0` to `x1`, clamped.
fn draw_h_line(img: &mut image::ImageBuffer<image::Rgb<u8>, Vec<u8>>, y: i32, x0: i32, x1: i32, color: image::Rgb<u8>) {
    let (iw, ih) = (img.width() as i32, img.height() as i32);
    if y < 0 || y >= ih { return; }
    for x in x0..=x1 {
        if x >= 0 && x < iw {
            img.put_pixel(x as u32, y as u32, color);
        }
    }
}

/// Draw a single vertical line segment at column `x` from row `y0` to `y1`, clamped.
fn draw_v_line(img: &mut image::ImageBuffer<image::Rgb<u8>, Vec<u8>>, x: i32, y0: i32, y1: i32, color: image::Rgb<u8>) {
    let (iw, ih) = (img.width() as i32, img.height() as i32);
    if x < 0 || x >= iw { return; }
    for y in y0..=y1 {
        if y >= 0 && y < ih {
            img.put_pixel(x as u32, y as u32, color);
        }
    }
}

/// Draw a 2px rectangle outline on an RGB image (clamped to bounds).
fn draw_rect(
    img: &mut image::ImageBuffer<image::Rgb<u8>, Vec<u8>>,
    x: i32,
    y: i32,
    w: u32,
    h: u32,
    color: image::Rgb<u8>,
) {
    let x1 = x + w as i32 - 1;
    let y1 = y + h as i32 - 1;
    for t in 0..2i32 {
        draw_h_line(img, y + t,  x, x1, color); // top edge
        draw_h_line(img, y1 - t, x, x1, color); // bottom edge
        draw_v_line(img, x + t,  y, y1, color); // left edge
        draw_v_line(img, x1 - t, y, y1, color); // right edge
    }
}

#[cfg(test)]
mod lost_thread_tests {
    use super::*;

    /// S2: a dead tracker thread is noted once, not on every 4 ms engine tick,
    /// and dropping the source sets the flag its threads watch.
    #[test]
    fn a_lost_tracker_thread_is_noted_once_and_drop_signals_shutdown() {
        let (tx, rx) = mpsc::sync_channel::<RawPose>(1);
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut input = NeuralNetInput {
            rx,
            preview: Arc::new(PreviewSlot::new()),
            overlay_enabled: Arc::new(AtomicBool::new(false)),
            shutdown: Arc::clone(&shutdown),
            thread_lost: false,
        };
        assert!(input.poll().is_none());
        assert!(!input.thread_lost, "an empty channel is not a lost thread");
        drop(tx);
        assert!(input.poll().is_none());
        assert!(input.thread_lost);
        assert!(input.poll().is_none(), "later ticks stay quiet");
        drop(input);
        assert!(shutdown.load(Ordering::Acquire));
    }
}
