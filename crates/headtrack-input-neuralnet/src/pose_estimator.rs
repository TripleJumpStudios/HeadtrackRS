use anyhow::{Context, Result};
use image::GrayImage;
use ort::{
    session::Session,
    value::{Tensor, ValueType},
};
use std::path::Path;
use tracing::debug;

use crate::image_proc::{extract_square_patch, normalize_brightness, resize_gray};
use crate::localizer::Detection;
use crate::math::{estimate_position_mm, quat_to_euler_deg};

/// Estimated head pose ready to hand to the pipeline.
#[derive(Debug, Clone, Copy)]
pub struct HeadPose {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    /// mm, canonical (x=right, y=up, z=forward).
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Refined bounding box for the next frame's tracking pass.
    pub updated_box: Detection,
}

/// Wraps the `head-pose-0.2-big.onnx` ONNX session.
///
/// Input size is queried from ONNX metadata at load time.
pub struct PoseEstimator {
    session: Session,
    input_w: usize,
    input_h: usize,
    fov_diag_deg: f32,
}

impl PoseEstimator {
    pub fn load(model_path: &Path, fov_diag_deg: f32) -> Result<Self> {
        let session = Session::builder()
            .context("creating ONNX session builder")?
            .commit_from_file(model_path)
            .context("loading head-pose model")?;

        // Query input tensor shape — expected [batch=1, ch=1, H, W].
        // session.inputs() returns &[Outlet]; each Outlet has .dtype() -> &ValueType.
        // ValueType::Tensor { shape: Shape, .. }; Shape derefs to [i64].
        let (input_h, input_w) = match session
            .inputs()
            .first()
            .context("pose model has no inputs")?
            .dtype()
        {
            ValueType::Tensor { shape, .. } if shape.len() == 4 => {
                (shape[2].max(1) as usize, shape[3].max(1) as usize)
            }
            _ => anyhow::bail!("expected 4D input tensor [B,C,H,W] for pose model"),
        };

        debug!("pose model input: {input_w}×{input_h}");

        Ok(Self {
            session,
            input_w,
            input_h,
            fov_diag_deg,
        })
    }

    /// Run the pose estimator on `frame` given a head `detection`.
    ///
    /// # Preprocessing (matching opentrack)
    /// 1. Square-crop around detection centre with 5% padding.
    /// 2. Resize to model input size.
    /// 3. 90th-percentile brightness normalisation → [−0.5, ~0.5].
    ///
    /// Returns `None` if the crop falls outside the frame.
    pub fn run(&mut self, frame: &GrayImage, detection: &Detection) -> Result<Option<HeadPose>> {
        let (img_w, img_h) = frame.dimensions();
        let patch_size = detection.patch_size();
        let centre = detection.center();

        let patch = match extract_square_patch(frame, centre, patch_size) {
            Some(p) => p,
            None => return Ok(None),
        };

        let resized = resize_gray(&patch, self.input_w as u32, self.input_h as u32);
        let pixels = normalize_brightness(&resized);

        let shape = [1usize, 1, self.input_h, self.input_w];
        let tensor =
            Tensor::<f32>::from_array((shape, pixels)).context("building pose tensor")?;

        // Pose model input may be unnamed — use positional (index 0).
        let outputs = self
            .session
            .run(ort::inputs![tensor])
            .context("pose estimator inference")?;

        // ── Quaternion [x, y, z, w] ──────────────────────────────────────────
        let (_, q) = outputs["quat"]
            .try_extract_tensor::<f32>()
            .context("extracting quat")?;
        let (yaw, pitch, roll) = quat_to_euler_deg(q[0], q[1], q[2], q[3]);

        // ── Position/size [x_norm, y_norm, size_norm] ────────────────────────
        // Normalised within the patch in [-1, 1].
        let (_, p) = outputs["pos_size"]
            .try_extract_tensor::<f32>()
            .context("extracting pos_size")?;
        let half = patch_size as f32 * 0.5;
        let face_cx = centre.0 + half * p[0];
        let face_cy = centre.1 + half * p[1];
        let face_size_px = half * p[2];

        let (x_mm, y_mm, z_mm) = estimate_position_mm(
            (face_cx, face_cy),
            face_size_px,
            (img_w, img_h),
            self.fov_diag_deg,
        );

        // ── Updated bounding box [x0, y0, x1, y1] ───────────────────────────
        let (_, b) = outputs["box"]
            .try_extract_tensor::<f32>()
            .context("extracting box")?;
        let updated_box = Detection {
            x: centre.0 + half * b[0],
            y: centre.1 + half * b[1],
            w: (half * (b[2] - b[0])).max(1.0),
            h: (half * (b[3] - b[1])).max(1.0),
            score: detection.score,
        };

        Ok(Some(HeadPose {
            yaw,
            pitch,
            roll,
            x: x_mm,
            y: y_mm,
            z: z_mm,
            updated_box,
        }))
    }
}
