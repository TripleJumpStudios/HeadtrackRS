use anyhow::{Context, Result};
use headtrack_core::{config::CameraConfig, RawPose};
use image::{ImageBuffer, Rgb};
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{CameraIndex, CameraFormat, FrameFormat, RequestedFormat, RequestedFormatType, Resolution},
    Camera,
};
use std::{path::PathBuf, time::Instant};
use tracing::{debug, info, warn};

use crate::{
    image_proc::to_gray,
    localizer::{Detection, Localizer},
    pose_estimator::PoseEstimator,
    v4l2_capture::V4l2Camera,
};

/// Return type for [`Tracker::step_with_frame`]: pose + optional RGB preview frame.
type StepFrame = (Option<RawPose>, Option<ImageBuffer<Rgb<u8>, Vec<u8>>>);

/// Frames since last good pose before the localizer is re-run from scratch.
const LOCALIZER_RERUN_AFTER_MISSES: u32 = 10;

/// File names of the two ONNX models within the model directory.
pub const LOCALIZER_FILENAME: &str = "head-localizer.onnx";
/// Default pose model. small variant is 3x faster on CPU with similar quality.
/// Switch to head-pose-0.4-big-f32.onnx if a GPU/CUDA backend is available.
pub const POSE_FILENAME: &str = "head-pose-0.4-small-f32.onnx";

/// Configuration passed into the background thread.
///
/// All fields are `Send + 'static` so they can be moved into `thread::spawn`.
pub struct TrackerConfig {
    pub localizer_path: PathBuf,
    pub pose_path: PathBuf,
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
            localizer_path: model_dir.join(LOCALIZER_FILENAME),
            pose_path: model_dir.join(POSE_FILENAME),
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
    localizer: Localizer,
    pose: PoseEstimator,
    start: Instant,
    /// Most-recent bounding box from the pose estimator (tracking mode).
    tracked_box: Option<Detection>,
    miss_count: u32,
    step_count: u32,
}

impl Tracker {
    /// Initialise models and open the webcam.  Must be called from the thread
    /// that will own the `Camera`.
    pub fn new(cfg: &TrackerConfig) -> Result<Self> {
        info!("loading localizer: {:?}", cfg.localizer_path);
        let localizer = Localizer::load(&cfg.localizer_path)?;

        info!("loading pose estimator: {:?}", cfg.pose_path);
        let pose = PoseEstimator::load(&cfg.pose_path, cfg.fov_diag_deg)?;

        let (res_w, res_h) = cfg.camera_resolution;
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

        Ok(Self {
            camera,
            localizer,
            pose,
            start: Instant::now(),
            tracked_box: None,
            miss_count: 0,
            step_count: 0,
        })
    }

    /// Initialise models only — no camera.
    ///
    /// Used by the decoupled YUYV path where a separate capture thread owns
    /// the camera and feeds grayscale frames to the inference thread.
    pub fn new_inference_only(cfg: &TrackerConfig) -> Result<Self> {
        info!("loading localizer: {:?}", cfg.localizer_path);
        let localizer = Localizer::load(&cfg.localizer_path)?;

        info!("loading pose estimator: {:?}", cfg.pose_path);
        let pose = PoseEstimator::load(&cfg.pose_path, cfg.fov_diag_deg)?;

        // Dummy camera — will never be used.  The capture thread owns the real one.
        // We need a V4l2Camera to satisfy the enum, but we never call capture().
        // Use a placeholder; the CameraBackend is never accessed.
        Ok(Self {
            camera: CameraBackend::None,
            localizer,
            pose,
            start: Instant::now(),
            tracked_box: None,
            miss_count: 0,
            step_count: 0,
        })
    }

    /// Capture one frame and run the inference pipeline.
    ///
    /// Returns a `RawPose` on success, `None` if no head is found or an
    /// error occurs (errors are logged, not propagated, so the loop continues).
    /// The RGB frame is dropped immediately after grayscale conversion.
    pub fn step(&mut self) -> Option<RawPose> {
        let t0 = Instant::now();
        let rgb_frame = match self.capture() {
            Ok(f) => f,
            Err(e) => {
                warn!("webcam capture error: {e:#}");
                return None;
            }
        };

        let t_cap = t0.elapsed();
        if t_cap.as_millis() > 300 {
            warn!("webcam stall: capture took {:.0}ms (camera may have frozen)", t_cap.as_millis());
        }
        let gray = to_gray(&rgb_frame);
        drop(rgb_frame); // free 921KB before inference
        let t_gray = t0.elapsed();

        let result = self.run_inference(gray);
        let t_total = t0.elapsed();

        self.step_count += 1;
        if self.step_count.is_multiple_of(30) {
            info!(
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
            Ok(f) => f,
            Err(e) => {
                warn!("webcam capture error: {e:#}");
                return (None, None);
            }
        };

        let gray = to_gray(&rgb_frame);
        // Note: rgb_frame stays alive here for the preview return.
        // This is intentional — only called every 3rd frame.
        let pose = self.run_inference(gray);
        (pose, Some(rgb_frame))
    }

    /// Run inference on a pre-captured grayscale frame.
    ///
    /// Used by the decoupled YUYV capture path where capture and inference
    /// run in separate threads.
    pub fn infer_gray(&mut self, gray: image::GrayImage) -> Option<RawPose> {
        self.run_inference(gray)
    }

    /// Common inference logic shared by `step` and `step_with_frame`.
    fn run_inference(&mut self, gray: image::GrayImage) -> Option<RawPose> {
        if self.miss_count >= LOCALIZER_RERUN_AFTER_MISSES {
            debug!("tracking lost — running localizer");
            self.tracked_box = None;
        }

        let detection = match self.tracked_box {
            Some(existing) => existing,
            None => match self.localizer.run(&gray) {
                Ok(Some(det)) => {
                    debug!("localizer: head found (score={:.2})", det.score);
                    det
                }
                Ok(None) => {
                    debug!("localizer: no head");
                    self.miss_count += 1;
                    return None;
                }
                Err(e) => {
                    warn!("localizer error: {e:#}");
                    return None;
                }
            },
        };

        match self.pose.run(&gray, &detection) {
            Ok(Some(head_pose)) => {
                self.tracked_box = Some(head_pose.updated_box);
                self.miss_count = 0;
                Some(RawPose {
                    yaw: head_pose.yaw,
                    pitch: head_pose.pitch,
                    roll: head_pose.roll,
                    x: head_pose.x,
                    y: head_pose.y,
                    z: head_pose.z,
                    timestamp_us: self.start.elapsed().as_micros() as u64,
                })
            }
            Ok(None) => {
                self.miss_count += 1;
                None
            }
            Err(e) => {
                warn!("pose estimator error: {e:#}");
                self.miss_count += 1;
                None
            }
        }
    }

    /// The most recent bounding box from the pose estimator.
    pub fn tracked_box(&self) -> Option<Detection> {
        self.tracked_box
    }

    fn capture(&mut self) -> Result<ImageBuffer<Rgb<u8>, Vec<u8>>> {
        self.camera.capture()
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

/// Return true if this camera index should skip nokhwa and use raw V4L2.
fn should_force_v4l2(index: u32) -> bool {
    let name = std::fs::read_to_string(
        format!("/sys/class/video4linux/video{index}/name")
    )
    .unwrap_or_default();
    let lower = name.trim().to_lowercase();
    V4L2_PREFERRED.iter().any(|key| lower.contains(key))
}
