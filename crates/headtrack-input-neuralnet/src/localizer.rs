use anyhow::{Context, Result};
use image::GrayImage;
use ort::{session::Session, value::Tensor};
use std::path::Path;
use tracing::debug;

use crate::image_proc::{normalize_linear, resize_gray, unnorm};
use crate::math::sigmoid;

/// Input dimensions for `head-localizer.onnx` — fixed by the model.
pub const LOCALIZER_W: u32 = 288;
pub const LOCALIZER_H: u32 = 224;

const SCORE_THRESHOLD: f32 = 0.5;

/// Detected head bounding box in pixel coordinates of the original frame.
#[derive(Debug, Clone, Copy)]
pub struct Detection {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
    pub score: f32,
}

impl Detection {
    pub fn center(&self) -> (f32, f32) {
        (self.x + self.w * 0.5, self.y + self.h * 0.5)
    }

    /// Side length of the square patch enclosing this detection (5% padding).
    pub fn patch_size(&self) -> u32 {
        (self.w.max(self.h) * 1.05) as u32
    }
}

pub struct Localizer {
    session: Session,
}

impl Localizer {
    pub fn load(model_path: &Path) -> Result<Self> {
        let session = Session::builder()
            .context("creating ONNX session builder")?
            .commit_from_file(model_path)
            .context("loading head-localizer.onnx")?;
        Ok(Self { session })
    }

    /// Run the localizer on a full grayscale frame.
    ///
    /// Preprocessing matches opentrack:
    ///   resize to 288×224, normalise to [−0.5, 0.5] (÷255 − 0.5).
    ///
    /// Returns `None` when no head is found above the confidence threshold.
    pub fn run(&mut self, frame: &GrayImage) -> Result<Option<Detection>> {
        let (img_w, img_h) = frame.dimensions();

        let scaled = resize_gray(frame, LOCALIZER_W, LOCALIZER_H);
        let pixels = normalize_linear(&scaled);

        // Build tensor from (shape, Vec<f32>) — no ndarray needed.
        let shape = [1usize, 1, LOCALIZER_H as usize, LOCALIZER_W as usize];
        let tensor =
            Tensor::<f32>::from_array((shape, pixels)).context("building localizer tensor")?;

        // inputs! does not return Result in ort 2.0.0-rc — no ? here.
        let outputs = self
            .session
            .run(ort::inputs!["x" => tensor])
            .context("localizer inference")?;

        // "logit_box": shape [1, 5] = [score_logit, x0, y0, x1, y1]
        // Coordinates are in normalised [-1, 1] space wrt. the model input.
        let (_, r) = outputs["logit_box"]
            .try_extract_tensor::<f32>()
            .context("extracting logit_box")?;

        let score = sigmoid(r[0]);
        debug!("localizer score: {score:.3}");

        if score < SCORE_THRESHOLD {
            return Ok(None);
        }

        // Unnormalise to original-frame pixel coordinates.
        let x0 = unnorm(r[1], img_w);
        let y0 = unnorm(r[2], img_h);
        let x1 = unnorm(r[3], img_w);
        let y1 = unnorm(r[4], img_h);

        Ok(Some(Detection {
            x: x0,
            y: y0,
            w: (x1 - x0).max(1.0),
            h: (y1 - y0).max(1.0),
            score,
        }))
    }
}
