//! Experimental 6DRepNet-style head-pose estimator.
//!
//! Loads `head-pose-mobilenetv3-large.onnx` (yakhyo/head-pose-estimation, MIT,
//! MobileNetV3-Large backbone trained with 6D rotation representation à la
//! 6DRepNet). The model takes a 224×224 RGB face crop and returns a 3×3
//! rotation matrix; we decode that to (yaw, pitch, roll) in degrees.
//!
//! Translation (x/y/z) is **not** produced here — it comes from the stable
//! landmarks (or the PnP solve). This is the **default** rotation source;
//! `HEADTRACK_ROT_SOURCE=pnp` selects the geometric solver instead.

use anyhow::{Context, Result};
use image::{ImageBuffer, Rgb};
use ort::{session::Session, value::Tensor};
use std::path::Path;
use tracing::debug;

use crate::image_proc::{extract_square_patch_rgb, imagenet_normalise_chw, resize_rgb};
use crate::scrfd_detector::Detection;

/// File name of the 6DRepNet-style ONNX model under the model directory.
pub const MODEL_FILENAME: &str = "head-pose-mobilenetv3-large.onnx";

/// Just rotation — translation comes from the legacy estimator.
#[derive(Debug, Clone, Copy)]
pub struct HeadRotation {
    pub yaw:   f32,
    pub pitch: f32,
    pub roll:  f32,
}

/// Wraps the MobileNetV3-Large 6DRepNet-style ONNX session.
pub struct PoseEstimator6DRepNet {
    session: Session,
    input_w: u32,
    input_h: u32,
}

impl PoseEstimator6DRepNet {
    pub fn load(model_path: &Path) -> Result<Self> {
        let session = crate::ort_session::builder("6DRepNet")?
            .commit_from_file(model_path)
            .context("loading 6DRepNet pose model")?;

        // Input shape [1, 3, 224, 224] per the yakhyo ONNX export. We hardcode
        // the channel/batch and only read H/W from the model in case a future
        // export uses a different size.
        use ort::value::ValueType;
        let (input_h, input_w) = match session
            .inputs()
            .first()
            .context("6DRepNet model has no inputs")?
            .dtype()
        {
            ValueType::Tensor { shape, .. } if shape.len() == 4 => {
                (shape[2].max(1) as u32, shape[3].max(1) as u32)
            }
            _ => anyhow::bail!("expected 4D input tensor [B,C,H,W] for 6DRepNet model"),
        };

        debug!("6DRepNet model input: {input_w}×{input_h}");

        Ok(Self { session, input_w, input_h })
    }

    /// Run the rotation estimator on the same square crop the legacy estimator
    /// uses. Returns `None` if the patch falls outside the frame.
    pub fn run(
        &mut self,
        rgb_frame: &ImageBuffer<Rgb<u8>, Vec<u8>>,
        detection: &Detection,
    ) -> Result<Option<HeadRotation>> {
        let patch_size = detection.patch_size();
        let centre = detection.center();

        let patch = match extract_square_patch_rgb(rgb_frame, centre, patch_size) {
            Some(p) => p,
            None => return Ok(None),
        };

        let resized = resize_rgb(&patch, self.input_w, self.input_h);
        let pixels = imagenet_normalise_chw(&resized);

        let shape = [1usize, 3, self.input_h as usize, self.input_w as usize];
        let tensor =
            Tensor::<f32>::from_array((shape, pixels)).context("building 6DRepNet tensor")?;

        let outputs = self
            .session
            .run(ort::inputs![tensor])
            .context("6DRepNet inference")?;

        // Output is named `rotation_matrix`, shape [1, 3, 3], row-major.
        let (_, r) = outputs["rotation_matrix"]
            .try_extract_tensor::<f32>()
            .context("extracting rotation_matrix")?;

        if r.len() < 9 {
            anyhow::bail!("rotation_matrix returned {} values, expected 9", r.len());
        }

        Ok(Some(rotation_matrix_to_yaw_pitch_roll(r)))
    }
}

/// Decode a row-major flat 3×3 rotation matrix to (yaw, pitch, roll) in
/// degrees, headtrack-rs canonical convention:
/// - yaw   = turn head right is +
/// - pitch = tilt head up is +
/// - roll  = tilt head right (right ear toward shoulder) is +
///
/// **Sign convention is provisional** — the 6DRepNet/yakhyo training data is
/// 300W-LP/AFLW2000 which uses (pitch around X, yaw around Y, roll around Z)
/// in a camera-facing frame. The signs below were chosen to match our existing
/// NeuralNet output and need empirical validation in-cockpit. Tweak the three
/// `*_SIGN` constants (and only those) if any axis ends up reversed.
fn rotation_matrix_to_yaw_pitch_roll(r: &[f32]) -> HeadRotation {
    // Standard XYZ Tait-Bryan decomposition (yakhyo's `rotation_matrix_to_euler`):
    //   sy    = sqrt(R[0,0]^2 + R[1,0]^2)
    //   pitch = atan2( R[2,1], R[2,2])   if !singular else atan2(-R[1,2], R[1,1])
    //   yaw   = atan2(-R[2,0], sy)
    //   roll  = atan2( R[1,0], R[0,0])   if !singular else 0
    let r00 = r[0]; let r01 = r[1]; let r02 = r[2];
    let r10 = r[3]; let r11 = r[4]; let r12 = r[5];
    let r20 = r[6]; let r21 = r[7]; let r22 = r[8];

    let sy = (r00 * r00 + r10 * r10).sqrt();
    let singular = sy < 1e-6;

    let (pitch_rad, roll_rad) = if singular {
        ((-r12).atan2(r11), 0.0)
    } else {
        (r21.atan2(r22), r10.atan2(r00))
    };
    let yaw_rad = (-r20).atan2(sy);

    let _ = (r01, r02); // unused but kept named for readability above.

    let to_deg = 180.0 / std::f32::consts::PI;

    // Sign mapping — verified empirically in-cockpit 2026-05-07 (Cirrus-class
    // aircraft) against the headtrack-rs canonical convention:
    //   pitch: +ve = nose up                                        ✓ pass-through
    //   yaw:   +ve = head turned right                              ✓ pass-through
    //   roll:  +ve = head tilted right (right ear toward shoulder)  → flip
    //
    // The yakhyo MobileNetV3-Large output produces +roll for left-tilt; we
    // negate to match our canonical (right-tilt = +).
    const PITCH_SIGN: f32 =  1.0;
    const YAW_SIGN:   f32 =  1.0;
    const ROLL_SIGN:  f32 = -1.0;

    HeadRotation {
        yaw:   YAW_SIGN   * yaw_rad   * to_deg,
        pitch: PITCH_SIGN * pitch_rad * to_deg,
        roll:  ROLL_SIGN  * roll_rad  * to_deg,
    }
}
