//! Webcam AI head tracking via ONNX Runtime.
//!
//! ## Pipeline
//!
//! A background thread owns the camera and both ONNX sessions.  On each
//! webcam frame it runs:
//!
//! 1. **Localizer** (`head-localizer.onnx`) — detects the head and returns a
//!    bounding box.  Once a box is found, subsequent frames use the pose
//!    estimator's refined box so the localizer only reruns after N consecutive
//!    failures.
//!
//! 2. **Pose estimator** (`head-pose-0.2-big.onnx`) — crops the head region,
//!    runs ONNX inference, returns a quaternion + 3D position.
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
//! Download from the latest opentrack release and place in `assets/models/`:
//! - `head-localizer.onnx`
//! - `head-pose-0.2-big.onnx`

mod image_proc;
mod localizer;
mod math;
mod pose_estimator;
mod tracker;
mod v4l2_capture;

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
use tracing::info;
use localizer::Detection;
use tracker::{Tracker, TrackerConfig};
use v4l2_capture::{V4l2YuyvCamera, yuyv_to_gray, yuyv_to_rgb};

/// Lock-free single-producer single-consumer preview frame slot.
///
/// The encoder thread stores new JPEG frames, the preview-writer thread
/// takes the latest. Uses atomic pointer swap — no mutex, no poisoning.
pub struct PreviewSlot {
    ptr: AtomicPtr<Vec<u8>>,
}

impl PreviewSlot {
    pub fn new() -> Self {
        Self { ptr: AtomicPtr::new(std::ptr::null_mut()) }
    }

    /// Store a new JPEG frame. Frees the previous frame if any.
    pub fn store(&self, jpeg: Vec<u8>) {
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
        let mut slot = self.data.lock().unwrap();
        *slot = Some(frame);
        self.ready.notify_one();
    }

    /// Block until a new frame is available or `shutdown` is set.
    fn take_or_shutdown(&self, shutdown: &AtomicBool) -> Option<Vec<u8>> {
        let mut slot = self.data.lock().unwrap();
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

        let (pose_tx, pose_rx) = mpsc::channel::<RawPose>();
        let (init_tx, init_rx) = mpsc::sync_channel::<anyhow::Result<()>>(0);

        let preview: PreviewFrame = Arc::new(PreviewSlot::new());
        let preview_writer = Arc::clone(&preview);
        let overlay_flag = Arc::clone(&overlay_enabled);

        let (frame_tx, frame_rx) =
            mpsc::sync_channel::<(ImageBuffer<Rgb<u8>, Vec<u8>>, Option<Detection>)>(1);

        // Encoder thread — JPEG-encodes preview frames off the hot tracking path.
        thread::Builder::new()
            .name("preview-encoder".into())
            .spawn(move || {
                while let Ok((rgb, detection)) = frame_rx.recv() {
                    let det = if overlay_flag.load(Ordering::Relaxed) {
                        detection.as_ref()
                    } else {
                        None
                    };
                    if let Ok(jpeg) = encode_jpeg_preview(&rgb, det) {
                        preview_writer.store(jpeg);
                    }
                }
            })
            .context("spawning preview encoder thread")?;

        thread::Builder::new()
            .name("neuralnet-tracker".into())
            .spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let mut tracker = match Tracker::new(&cfg) {
                        Ok(t) => {
                            let _ = init_tx.send(Ok(()));
                            t
                        }
                        Err(e) => {
                            let _ = init_tx.send(Err(e));
                            return;
                        }
                    };

                    let mut frame_counter: u32 = 0;
                    loop {
                        frame_counter = frame_counter.wrapping_add(1);
                        let want_frame = frame_counter % 3 == 0;

                        if want_frame {
                            let (pose, frame) = tracker.step_with_frame();
                            if let Some(pose) = pose {
                                if pose_tx.send(pose).is_err() {
                                    break;
                                }
                            }
                            if let Some(rgb) = frame {
                                let det = tracker.tracked_box();
                                let _ = frame_tx.try_send((rgb, det));
                            }
                        } else {
                            if let Some(pose) = tracker.step() {
                                if pose_tx.send(pose).is_err() {
                                    break;
                                }
                            }
                        }
                    }
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

        Ok(Self { rx: pose_rx, preview, overlay_enabled, shutdown: Arc::new(AtomicBool::new(false)) })
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

        let (pose_tx, pose_rx) = mpsc::channel::<RawPose>();

        let shutdown = Arc::new(AtomicBool::new(false));

        let preview: PreviewFrame = Arc::new(PreviewSlot::new());
        let preview_writer = Arc::clone(&preview);
        let overlay_flag = Arc::clone(&overlay_enabled);

        let (frame_tx, frame_rx) =
            mpsc::sync_channel::<(ImageBuffer<Rgb<u8>, Vec<u8>>, Option<Detection>)>(1);

        // Encoder thread — same as MJPEG path.
        thread::Builder::new()
            .name("preview-encoder".into())
            .spawn(move || {
                while let Ok((rgb, detection)) = frame_rx.recv() {
                    let det = if overlay_flag.load(Ordering::Relaxed) {
                        detection.as_ref()
                    } else {
                        None
                    };
                    if let Ok(jpeg) = encode_jpeg_preview(&rgb, det) {
                        preview_writer.store(jpeg);
                    }
                }
            })
            .context("spawning preview encoder thread")?;

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

                    loop {
                        if shutdown_capture.load(Ordering::Acquire) {
                            info!("yuyv capture: shutdown");
                            break;
                        }
                        match cam.frame_raw() {
                            Ok(yuyv) => latest_writer.store(yuyv),
                            Err(e) => {
                                tracing::warn!("yuyv capture error: {e:#}");
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

                    // Every 3rd frame, also produce RGB for preview.
                    frame_counter = frame_counter.wrapping_add(1);
                    let want_preview = frame_counter % 3 == 0;

                    if want_preview {
                        let rgb = yuyv_to_rgb(&yuyv, res_w, res_h);
                        drop(yuyv); // free 614KB YUYV buffer

                        let pose = tracker.infer_gray(gray);
                        let t_total = t0.elapsed();

                        step_count += 1;
                        if step_count % 30 == 0 {
                            info!(
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
                            if pose_tx.send(raw).is_err() {
                                break;
                            }
                            // Send RGB to preview encoder.
                            let det = tracker.tracked_box();
                            let _ = frame_tx.try_send((rgb, det));
                        }
                    } else {
                        drop(yuyv);

                        let pose = tracker.infer_gray(gray);
                        let t_total = t0.elapsed();

                        step_count += 1;
                        if step_count % 30 == 0 {
                            info!(
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
                            if pose_tx.send(raw).is_err() {
                                break;
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

        Ok(Self { rx: pose_rx, preview, overlay_enabled, shutdown })
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
                    tracing::error!("tracker thread exited unexpectedly — camera disconnected or thread panicked");
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

/// Encode an RGB image to JPEG at low quality for preview display.
/// Downscales to 320x240 to keep the JPEG small (~5-15 KB).
/// If a `Detection` is provided, draws the head bounding box on the thumbnail.
fn encode_jpeg_preview(
    rgb: &image::ImageBuffer<image::Rgb<u8>, Vec<u8>>,
    detection: Option<&Detection>,
) -> anyhow::Result<Vec<u8>> {
    use image::imageops::FilterType;
    use std::io::Cursor;

    const THUMB_W: u32 = 320;
    const THUMB_H: u32 = 240;

    // Downscale to 320x240 for the preview thumbnail.
    let mut thumb = image::imageops::resize(rgb, THUMB_W, THUMB_H, FilterType::Nearest);

    // Draw bounding box overlay if we have a detection.
    if let Some(det) = detection {
        let scale_x = THUMB_W as f32 / rgb.width() as f32;
        let scale_y = THUMB_H as f32 / rgb.height() as f32;
        draw_rect(
            &mut thumb,
            (det.x * scale_x) as i32,
            (det.y * scale_y) as i32,
            (det.w * scale_x) as u32,
            (det.h * scale_y) as u32,
            image::Rgb([0, 255, 0]),
        );
    }

    let mut buf = Cursor::new(Vec::new());
    let encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut buf, 60);
    image::ImageEncoder::write_image(
        encoder,
        thumb.as_raw(),
        THUMB_W,
        THUMB_H,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(buf.into_inner())
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
    let (iw, ih) = (img.width() as i32, img.height() as i32);

    // Draw horizontal lines (top + bottom edges).
    for thickness in 0..2i32 {
        let top = y + thickness;
        let bot = y + h as i32 - 1 - thickness;
        for px in x..=(x + w as i32 - 1) {
            if px >= 0 && px < iw {
                if top >= 0 && top < ih {
                    img.put_pixel(px as u32, top as u32, color);
                }
                if bot >= 0 && bot < ih {
                    img.put_pixel(px as u32, bot as u32, color);
                }
            }
        }
    }

    // Draw vertical lines (left + right edges).
    for thickness in 0..2i32 {
        let left = x + thickness;
        let right = x + w as i32 - 1 - thickness;
        for py in y..=(y + h as i32 - 1) {
            if py >= 0 && py < ih {
                if left >= 0 && left < iw {
                    img.put_pixel(left as u32, py as u32, color);
                }
                if right >= 0 && right < iw {
                    img.put_pixel(right as u32, py as u32, color);
                }
            }
        }
    }
}
