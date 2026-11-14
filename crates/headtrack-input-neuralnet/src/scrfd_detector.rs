//! SCRFD face detector — bbox + 5 facial landmarks per detection.
//!
//! Loads `scrfd-det-10g.onnx` (yakhyo/head-pose-estimation, MIT licensed).
//! The model is a multi-scale anchor-based detector trained on WIDER-FACE; it
//! emits scores, bbox distances, and 5-point keypoint distances at three
//! feature pyramid strides (8, 16, 32). We decode all three scales, run NMS,
//! and return the highest-scoring detection.
//!
//! This module is paired with [`landmark_position`] to derive translation
//! from facial landmarks (interpupillary distance for Z, eye centroid for
//! X/Y) — far more rotation-stable than the legacy face-area Z heuristic.

use anyhow::{Context, Result};
use image::{ImageBuffer, Rgb};
use ort::{logging::LogLevel, session::Session, value::Tensor};
use std::path::Path;
use tracing::debug;

/// Detected head bounding box in pixel coordinates of the original frame.
///
/// Lived in `localizer.rs` until SL3 deleted the legacy chain; SCRFD is now its
/// only producer.
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

/// File name of the SCRFD ONNX under the model directory.
pub const MODEL_FILENAME: &str = "scrfd-det-10g.onnx";

/// Square input size used by the SCRFD model.
///
/// The model is dynamic-shape `[1, 3, ?, ?]` and was trained at 640×640, but
/// for our use case (a single face filling ~30% of a webcam frame at moderate
/// distance) 320×320 is plenty — SCRFD's smallest detectable face is ~16 px
/// so a face that is 100+ px even after letterboxing still detects reliably.
/// 320 gives a 4× reduction in detection work vs 640, which is the difference
/// between ~10 fps and ~30+ fps end-to-end. If face detection ever misses at
/// edge angles, bump this to 480 (1.5× cost, slightly better recall).
const INPUT_SIZE: u32 = 320;

/// Feature-pyramid strides in the SCRFD det_10g architecture.
const STRIDES: [u32; 3] = [8, 16, 32];
/// Anchors per spatial location at each stride.
const NUM_ANCHORS: usize = 2;

/// Confidence threshold below which detections are discarded.
const CONF_THRESHOLD: f32 = 0.5;
/// IoU threshold for non-maximum suppression.
const IOU_THRESHOLD: f32 = 0.4;

/// Pixel mean / std applied during preprocessing (matches yakhyo).
const PIX_MEAN: f32 = 127.5;
const PIX_STD:  f32 = 128.0;

/// Internal NMS candidate: `(score, bbox=[x0, y0, x1, y1], 5 keypoints)`.
type RawDetection = (f32, [f32; 4], [(f32, f32); 5]);

/// One face detection in the *original* image coordinate frame, with N
/// keypoints. SCRFD itself emits 5 (right eye, left eye, nose tip, right
/// mouth, left mouth); a denser landmark stage would
/// produces 100+ here. The detection score is on `bbox.score`.
#[derive(Debug, Clone)]
pub struct FaceDetection {
    pub bbox: Detection,
    pub kps: Vec<(f32, f32)>,
}

/// SCRFD ONNX session + cached anchor centers (one set per stride).
pub struct ScrfdDetector {
    session:        Session,
    anchor_centers: [Vec<(f32, f32)>; 3],
}

impl ScrfdDetector {
    pub fn load(model_path: &Path) -> Result<Self> {
        // SCRFD's ONNX export hard-codes 640×640 output-tensor shapes in its
        // metadata, but we deliberately feed it 320×320 (4× faster, same recall
        // for our face-fills-30%-of-frame use case). At session run time ORT
        // logs a "Expected shape ... does not match actual shape" WARN per
        // mismatched output tensor — 9 lines per inference, ~1× per second on
        // re-detect frames. Functionally harmless. Pin the per-session log
        // threshold to Error so the noise stops at source instead of forcing
        // users to thread `ort=error` through their RUST_LOG filter.
        let mut session_builder = crate::ort_session::builder("SCRFD")?
            .with_log_level(LogLevel::Error)
            .map_err(|e| anyhow::anyhow!("setting SCRFD log level: {e}"))?;
        let session = session_builder
            .commit_from_file(model_path)
            .context("loading SCRFD model")?;

        debug!("SCRFD: loaded {model_path:?}");

        // Anchor centers are deterministic given input size + stride; precompute
        // once so the per-frame hot path is just inference + decode.
        let anchor_centers = [
            build_anchor_centers(INPUT_SIZE, STRIDES[0]),
            build_anchor_centers(INPUT_SIZE, STRIDES[1]),
            build_anchor_centers(INPUT_SIZE, STRIDES[2]),
        ];

        Ok(Self { session, anchor_centers })
    }

    /// Detect the highest-scoring face in `rgb_frame`. Returns coordinates in
    /// the original image's pixel space, *not* the letterboxed model space.
    pub fn detect(&mut self, rgb_frame: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> Result<Option<FaceDetection>> {
        let (img_w, img_h) = rgb_frame.dimensions();

        // ── Letterbox into a 640×640 canvas, preserving aspect ratio. ───────
        let im_ratio    = img_h as f32 / img_w as f32;
        let model_ratio = 1.0; // 640/640
        let (new_w, new_h) = if im_ratio > model_ratio {
            // Image taller than model → match height, scale width.
            let h = INPUT_SIZE;
            let w = (h as f32 / im_ratio).round() as u32;
            (w, h)
        } else {
            let w = INPUT_SIZE;
            let h = (w as f32 * im_ratio).round() as u32;
            (w, h)
        };
        let det_scale = new_h as f32 / img_h as f32;

        let resized = image::imageops::resize(
            rgb_frame, new_w, new_h, image::imageops::FilterType::Triangle,
        );

        // ── Preprocess into NCHW float32 with mean/std normalisation. ──────
        // The model was trained with `cv2.dnn.blobFromImage(swapRB=True)`,
        // which reads BGR (cv2 default) and emits an **RGB** blob. The model
        // therefore expects RGB. Our frames are already RGB, so we pass
        // channels through unchanged. (Earlier revisions did an RGB→BGR
        // swap here based on inverted reasoning; that caused SCRFD to score
        // false-positive regions on backgrounds higher than the real face.)
        let n = (INPUT_SIZE * INPUT_SIZE) as usize;
        let mut input = vec![0.0f32; 3 * n];
        for y in 0..new_h {
            for x in 0..new_w {
                let px = resized.get_pixel(x, y);
                let i = (y * INPUT_SIZE + x) as usize;
                input[i]         = (px[0] as f32 - PIX_MEAN) / PIX_STD; // R
                input[i + n]     = (px[1] as f32 - PIX_MEAN) / PIX_STD; // G
                input[i + 2 * n] = (px[2] as f32 - PIX_MEAN) / PIX_STD; // B
            }
        }

        let shape  = [1usize, 3, INPUT_SIZE as usize, INPUT_SIZE as usize];
        let tensor = Tensor::<f32>::from_array((shape, input)).context("building SCRFD tensor")?;
        let outputs = self.session.run(ort::inputs![tensor]).context("SCRFD inference")?;

        // Outputs are anonymously named ('448', '471', '494', ...) — order is
        // [scores×3, bboxes×3, kps×3] across the three strides 8/16/32.
        let names: Vec<String> = outputs.iter().map(|(n, _)| n.to_string()).collect();
        if names.len() != 9 {
            anyhow::bail!("SCRFD: expected 9 output tensors, got {}", names.len());
        }

        let mut all: Vec<RawDetection> = Vec::with_capacity(64);

        for (idx, &stride) in STRIDES.iter().enumerate() {
            let centers = &self.anchor_centers[idx];

            // SCORES: shape [N, 1]
            let (_, scores) = outputs[&names[idx][..]]
                .try_extract_tensor::<f32>()
                .context("extracting scores")?;
            // BBOX:   shape [N, 4]  (distances l,t,r,b in *grid units*; multiply by stride)
            let (_, bbox_d) = outputs[&names[idx + 3][..]]
                .try_extract_tensor::<f32>()
                .context("extracting bbox preds")?;
            // KPS:    shape [N, 10] (5 keypoints × 2 coords; same unit as bbox)
            let (_, kps_d) = outputs[&names[idx + 6][..]]
                .try_extract_tensor::<f32>()
                .context("extracting kps preds")?;

            for (i, &(cx, cy)) in centers.iter().enumerate() {
                let score = scores[i];
                if score < CONF_THRESHOLD { continue; }

                // bbox: anchor − distance for top-left, anchor + distance for bottom-right.
                let bx = i * 4;
                let l = bbox_d[bx]     * stride as f32;
                let t = bbox_d[bx + 1] * stride as f32;
                let r = bbox_d[bx + 2] * stride as f32;
                let b = bbox_d[bx + 3] * stride as f32;
                let bbox = [cx - l, cy - t, cx + r, cy + b];

                // 5 keypoints: anchor + distance, alternating (x, y).
                let kx = i * 10;
                let mut kps = [(0.0f32, 0.0f32); 5];
                for k in 0..5 {
                    kps[k] = (
                        cx + kps_d[kx + k * 2]     * stride as f32,
                        cy + kps_d[kx + k * 2 + 1] * stride as f32,
                    );
                }

                all.push((score, bbox, kps));
            }
        }

        if all.is_empty() {
            return Ok(None);
        }

        // Sort descending by score, NMS, take top.
        all.sort_unstable_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let kept = nms(&all, IOU_THRESHOLD);

        let &(score, bbox, kps) = match kept.first() {
            Some(idx) => &all[*idx],
            None => return Ok(None),
        };

        // Un-letterbox: divide by det_scale to go back to original image coords.
        let inv = 1.0 / det_scale;
        let to_orig = |x: f32, y: f32| (x * inv, y * inv);
        let (x0, y0) = to_orig(bbox[0], bbox[1]);
        let (x1, y1) = to_orig(bbox[2], bbox[3]);
        // SCRFD emits exactly 5 keypoints; `FaceDetection.kps` is a Vec so a
        // denser landmark stage could carry more without changing this type.
        let kps_orig: Vec<(f32, f32)> = kps
            .iter()
            .map(|&(kx, ky)| to_orig(kx, ky))
            .collect();

        // Map to the existing Detection struct (top-left origin + width/height).
        let det = Detection {
            x: x0.max(0.0).min(img_w as f32 - 1.0),
            y: y0.max(0.0).min(img_h as f32 - 1.0),
            w: (x1 - x0).max(1.0),
            h: (y1 - y0).max(1.0),
            score,
        };

        debug!("SCRFD: face found (score={:.3})", det.score);
        Ok(Some(FaceDetection { bbox: det, kps: kps_orig }))
    }
}

/// Build the (cx, cy) anchor centre list for one feature-pyramid stride at
/// the given square input size. Centres are repeated `NUM_ANCHORS` times per
/// spatial location to match the SCRFD anchor-prior layout.
fn build_anchor_centers(input_size: u32, stride: u32) -> Vec<(f32, f32)> {
    let h = (input_size / stride) as usize;
    let w = (input_size / stride) as usize;
    let mut centres = Vec::with_capacity(h * w * NUM_ANCHORS);
    for y in 0..h {
        for x in 0..w {
            let cx = (x as u32 * stride) as f32;
            let cy = (y as u32 * stride) as f32;
            // Repeat NUM_ANCHORS times to align with the anchor-major
            // flattening order used in the model output tensors.
            for _ in 0..NUM_ANCHORS {
                centres.push((cx, cy));
            }
        }
    }
    centres
}

/// Non-maximum suppression over `(score, bbox, kps)` tuples sorted descending
/// by score. Returns indices into `dets` to keep.
fn nms(dets: &[RawDetection], iou_thres: f32) -> Vec<usize> {
    let n = dets.len();
    let mut suppressed = vec![false; n];
    let mut keep = Vec::with_capacity(n);

    for i in 0..n {
        if suppressed[i] { continue; }
        keep.push(i);
        let a = dets[i].1;
        let area_a = (a[2] - a[0] + 1.0) * (a[3] - a[1] + 1.0);
        for j in (i + 1)..n {
            if suppressed[j] { continue; }
            let b = dets[j].1;
            let inter_x0 = a[0].max(b[0]);
            let inter_y0 = a[1].max(b[1]);
            let inter_x1 = a[2].min(b[2]);
            let inter_y1 = a[3].min(b[3]);
            let iw = (inter_x1 - inter_x0 + 1.0).max(0.0);
            let ih = (inter_y1 - inter_y0 + 1.0).max(0.0);
            let inter = iw * ih;
            let area_b = (b[2] - b[0] + 1.0) * (b[3] - b[1] + 1.0);
            let iou = inter / (area_a + area_b - inter);
            if iou > iou_thres {
                suppressed[j] = true;
            }
        }
    }

    keep
}
