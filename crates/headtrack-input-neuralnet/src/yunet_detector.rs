//! YuNet face detector — bbox + the same 5 facial landmarks as SCRFD.
//!
//! The permissively-licensed alternative to SCRFD. **MIT, © 2020 Shiqi Yu**,
//! from OpenCV Zoo (`models/face_detection_yunet/`) — commercial use explicitly
//! permitted, unlike InsightFace's weights, which are restricted to
//! non-commercial academic research. That licence difference is the entire
//! reason this module exists; see `docs/DETECTOR_BAKEOFF.md`.
//!
//! # Why it is a drop-in
//!
//! YuNet emits its 5 keypoints in **exactly** SCRFD's order — right eye, left
//! eye, nose tip, right mouth corner, left mouth corner — so
//! [`crate::landmark_position::position_from_eyes`] and the PnP solve need no
//! remapping. It returns the same [`FaceDetection`] type.
//!
//! # Why the input size is not a tuning knob
//!
//! The bake-off measured YuNet's Z noise (which comes from interpupillary
//! distance, so it *is* eye-keypoint precision) at two input sizes:
//!
//! | input | Z std on a still clip | vs SCRFD |
//! |---|---|---|
//! | 320×320 | 18.9 mm | **4.1× worse** |
//! | 640×640 | 5.4 mm | 1.17× — acceptable |
//!
//! At 320 this model is not good enough for our depth cue; at 640 it matches
//! SCRFD closely. YuNet is a 228 KB network and needs the pixels that SCRFD's
//! far larger capacity can do without. **Do not "optimise" this to a smaller
//! input** — it would silently restore a 4× depth-noise regression that is
//! invisible in a detection-rate metric.
//!
//! The ONNX export helpfully makes that hard: its input shape is a fixed
//! `[1, 3, 640, 640]`, so a smaller size cannot be fed without reshaping the
//! graph.
//!
//! # Known residual
//!
//! At sustained yaw beyond ~40°, YuNet's keypoints are ~2.5× noisier than
//! SCRFD's. For depth that is already covered — [`crate::z_gate`]'s
//! `confidence()` ramps the IPD cue down to a 0.35 floor by 40° yaw for
//! unrelated prior reasons. For X/Y it is not damped, and shows up as a larger
//! re-anchor step at each periodic re-detect. Recall itself is unaffected:
//! YuNet matched SCRFD at 100 % detection out to 89° of yaw.

use anyhow::{Context, Result};
use image::{ImageBuffer, Rgb};
use ort::{session::Session, value::Tensor};
use tracing::debug;

use crate::scrfd_detector::{Detection, FaceDetection};

/// File name of the YuNet ONNX under the model directory.
pub const MODEL_FILENAME: &str = "face_detection_yunet_2023mar.onnx";

/// Square input size. **Fixed by the ONNX graph and by the depth-noise
/// measurement above — not a knob.** See the module docs.
const INPUT_SIZE: u32 = 640;

/// Feature-pyramid strides, matching the `cls_8` / `cls_16` / `cls_32` outputs.
const STRIDES: [u32; 3] = [8, 16, 32];

/// Score threshold. YuNet's score is `sqrt(cls * obj)`, so it is already a
/// product of two calibrated probabilities and sits lower than SCRFD's for an
/// equivalent detection; 0.6 is OpenCV's own default and was what the bake-off
/// measured.
const CONF_THRESHOLD: f32 = 0.6;

/// IoU threshold for non-maximum suppression (OpenCV's default for this model).
const NMS_THRESHOLD: f32 = 0.3;

const KPS: usize = 5;

/// `(score, [x0, y0, x1, y1], 5 keypoints)` in letterboxed model space.
type RawDetection = (f32, [f32; 4], [(f32, f32); KPS]);

pub struct YuNetDetector {
    session: Session,
}

impl YuNetDetector {
    pub fn load(model_path: &std::path::Path) -> Result<Self> {
        let mut builder = crate::ort_session::builder("YuNet")?;
        let session = builder
            .commit_from_file(model_path)
            .context("loading YuNet model")?;
        debug!("YuNet: loaded {model_path:?}");
        Ok(Self { session })
    }

    /// Detect the highest-scoring face. Coordinates are in the original
    /// image's pixel space, not the letterboxed model space.
    pub fn detect(
        &mut self,
        rgb_frame: &ImageBuffer<Rgb<u8>, Vec<u8>>,
    ) -> Result<Option<FaceDetection>> {
        let (img_w, img_h) = rgb_frame.dimensions();

        // ── Letterbox: isotropic scale into the top-left of a square canvas.
        // Uniform scale keeps face geometry undistorted, which matters because
        // interpupillary distance becomes depth downstream.
        let scale = (INPUT_SIZE as f32 / img_w as f32).min(INPUT_SIZE as f32 / img_h as f32);
        let new_w = (img_w as f32 * scale).round().max(1.0) as u32;
        let new_h = (img_h as f32 * scale).round().max(1.0) as u32;

        let resized = image::imageops::resize(
            rgb_frame,
            new_w,
            new_h,
            image::imageops::FilterType::Triangle,
        );

        // YuNet takes raw pixel values — OpenCV builds its blob with
        // scalefactor 1.0 and no mean subtraction — in **BGR** channel order,
        // because it feeds cv::Mat directly without swapRB. Our frames are RGB,
        // so the channel write order below is deliberately reversed. Padding
        // stays 0.0 (black), matching the zeroed canvas OpenCV is given.
        let n = (INPUT_SIZE * INPUT_SIZE) as usize;
        let mut input = vec![0.0f32; 3 * n];
        for y in 0..new_h {
            for x in 0..new_w {
                let px = resized.get_pixel(x, y);
                let i = (y * INPUT_SIZE + x) as usize;
                input[i] = px[2] as f32; // B
                input[i + n] = px[1] as f32; // G
                input[i + 2 * n] = px[0] as f32; // R
            }
        }

        let shape = [1usize, 3, INPUT_SIZE as usize, INPUT_SIZE as usize];
        let tensor = Tensor::<f32>::from_array((shape, input)).context("building YuNet tensor")?;
        let outputs = self
            .session
            .run(ort::inputs![tensor])
            .context("YuNet inference")?;

        let mut all: Vec<RawDetection> = Vec::with_capacity(64);

        for &stride in STRIDES.iter() {
            let cols = (INPUT_SIZE / stride) as usize;

            let (_, cls) = outputs[format!("cls_{stride}").as_str()]
                .try_extract_tensor::<f32>()
                .context("extracting cls")?;
            let (_, obj) = outputs[format!("obj_{stride}").as_str()]
                .try_extract_tensor::<f32>()
                .context("extracting obj")?;
            let (_, bbox) = outputs[format!("bbox_{stride}").as_str()]
                .try_extract_tensor::<f32>()
                .context("extracting bbox")?;
            let (_, kps) = outputs[format!("kps_{stride}").as_str()]
                .try_extract_tensor::<f32>()
                .context("extracting kps")?;

            for i in 0..cls.len() {
                // Score is the geometric mean of classification and objectness
                // — this is how OpenCV's own decoder combines the two heads.
                let score = (cls[i].clamp(0.0, 1.0) * obj[i].clamp(0.0, 1.0)).sqrt();
                if score < CONF_THRESHOLD {
                    continue;
                }
                let col = (i % cols) as f32;
                let row = (i / cols) as f32;
                let s = stride as f32;

                // Centre is an offset from the anchor cell; extent is
                // log-encoded. One anchor per cell, unlike SCRFD's two.
                let bx = i * 4;
                let cx = (col + bbox[bx]) * s;
                let cy = (row + bbox[bx + 1]) * s;
                let w = bbox[bx + 2].exp() * s;
                let h = bbox[bx + 3].exp() * s;
                let bb = [cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5];

                let kx = i * 10;
                let mut pts = [(0.0f32, 0.0f32); KPS];
                for (k, p) in pts.iter_mut().enumerate() {
                    *p = ((col + kps[kx + k * 2]) * s, (row + kps[kx + k * 2 + 1]) * s);
                }

                all.push((score, bb, pts));
            }
        }

        if all.is_empty() {
            return Ok(None);
        }

        all.sort_unstable_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let kept = nms(&all, NMS_THRESHOLD);
        let &(score, bbox, kps) = match kept.first() {
            Some(idx) => &all[*idx],
            None => return Ok(None),
        };

        // Un-letterbox back to original image coordinates.
        let inv = 1.0 / scale;
        let (x0, y0) = (bbox[0] * inv, bbox[1] * inv);
        let (x1, y1) = (bbox[2] * inv, bbox[3] * inv);
        let kps_orig: Vec<(f32, f32)> = kps.iter().map(|&(x, y)| (x * inv, y * inv)).collect();

        let det = Detection {
            x: x0.clamp(0.0, img_w as f32 - 1.0),
            y: y0.clamp(0.0, img_h as f32 - 1.0),
            w: (x1 - x0).max(1.0),
            h: (y1 - y0).max(1.0),
            score,
        };

        debug!("YuNet: face found (score={:.3})", det.score);
        Ok(Some(FaceDetection {
            bbox: det,
            kps: kps_orig,
        }))
    }
}

/// Non-maximum suppression over detections sorted descending by score.
///
/// Note this uses plain areas, without SCRFD's `+1.0` pixel convention —
/// OpenCV's NMS for this model does the same, and the bake-off's reference
/// numbers came from that implementation.
fn nms(dets: &[RawDetection], iou_thres: f32) -> Vec<usize> {
    let n = dets.len();
    let mut suppressed = vec![false; n];
    let mut keep = Vec::with_capacity(n);

    for i in 0..n {
        if suppressed[i] {
            continue;
        }
        keep.push(i);
        let a = dets[i].1;
        let area_a = (a[2] - a[0]) * (a[3] - a[1]);
        for j in (i + 1)..n {
            if suppressed[j] {
                continue;
            }
            let b = dets[j].1;
            let iw = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
            let ih = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
            let inter = iw * ih;
            let area_b = (b[2] - b[0]) * (b[3] - b[1]);
            let denom = area_a + area_b - inter;
            if denom > 0.0 && inter / denom > iou_thres {
                suppressed[j] = true;
            }
        }
    }
    keep
}
