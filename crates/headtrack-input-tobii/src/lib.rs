//! Tobii IR head-pose estimator for HeadtrackRS (ESTIMATOR half only).
//!
//! PORTED from TobiiLab `crates/tobii-head` (a local, unpublished prototype)
//! on 2026-10-03 (task P04): `Estimator`, `fit_rotation`, `neutral`, `geometry_data`.
//! TobiiLab's `service.rs` (the live worker) was not ported; P06 replaced it with
//! [`source::TobiiInput`], the engine-facing `InputSource` (raw rotation, no filter).
//! Provenance and design: TobiiLab `docs/HEAD_ESTIMATOR.md`, `docs/PROVENANCE.md`.
//! Deviations from the source are listed in docs/P04_TOBII_ESTIMATOR.md.
//!
//! Experimental image-coordinate rotation. Not calibrated canonical head pose.
//! Models and geometry: Google MediaPipe, Apache-2.0; see assets/models/tobii.
mod geometry_data;
pub mod neutral;
#[cfg(test)]
mod parity;
pub mod source;
pub use source::TobiiInput;
use anyhow::ensure;
use headtrack_tobii_device::image::GrayImage;
use nalgebra::{Matrix3, Rotation3, Vector3};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::sync::Arc;
use std::{path::Path, time::Instant};
use thiserror::Error;
use tract_tflite::prelude::*;

pub const ESTIMATOR: &str = "google-facemesh-v2-tract-weak3d-v1";
#[derive(Clone, Debug, Serialize)]
pub struct Estimate {
    /// Euler decomposition Rz * Ry * Rx, in image basis (right, down, away).
    /// Array is [about X, about Y, about Z]. Physical head signs are unmeasured.
    pub image_angles_deg: [f32; 3],
    pub detector_score: f32,
    pub presence_score: f32,
    pub fit_relative_rms: f32,
    pub inference_ms: f32,
    pub landmarks: Vec<[f32; 3]>,
    pub crop: Crop,
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Crop {
    pub cx: f32,
    pub cy: f32,
    pub size: f32,
    pub angle: f32,
}
#[derive(Clone)]
struct Detection {
    score: f32,
    values: [f32; 16],
}
#[derive(Clone, Debug, Serialize)]
pub struct Evaluation {
    pub estimate: Option<Estimate>,
    pub reason: String,
    pub elapsed_ms: f32,
}
pub struct Estimator {
    detector: Arc<TypedRunnableModel>,
    mesh: Arc<TypedRunnableModel>,
}
/// Why a model file was refused. The integrity variants are produced BEFORE any
/// byte of the file is handed to the TFLite parser.
#[derive(Debug, Error)]
pub enum ModelError {
    #[error("cannot read model file {path}: {source}")]
    Io {
        path: std::path::PathBuf,
        source: std::io::Error,
    },
    #[error(
        "invalid model file size for {path}: not a regular file or over {MAX_MODEL_BYTES} bytes"
    )]
    BadFileSize { path: std::path::PathBuf },
    #[error("model checksum mismatch: {path} (expected sha256 {expected}, found {actual})")]
    ChecksumMismatch {
        path: std::path::PathBuf,
        expected: String,
        actual: String,
    },
    /// Hash verified, but the model failed parsing, shape checks or optimisation.
    #[error("model {path} rejected: {reason}")]
    Invalid {
        path: std::path::PathBuf,
        reason: String,
    },
}
const MAX_MODEL_BYTES: u64 = 4 * 1024 * 1024;
/// Read a model file and verify its size and sha256. No parsing happens here.
fn verified_bytes(path: &Path, digest: &str) -> Result<Vec<u8>, ModelError> {
    let io = |source| ModelError::Io {
        path: path.to_path_buf(),
        source,
    };
    let meta = std::fs::metadata(path).map_err(io)?;
    if !(meta.is_file() && meta.len() <= MAX_MODEL_BYTES) {
        return Err(ModelError::BadFileSize {
            path: path.to_path_buf(),
        });
    }
    let bytes = std::fs::read(path).map_err(io)?;
    let actual = format!("{:x}", Sha256::digest(&bytes));
    if actual != digest {
        return Err(ModelError::ChecksumMismatch {
            path: path.to_path_buf(),
            expected: digest.to_owned(),
            actual,
        });
    }
    Ok(bytes)
}
fn parse_model(
    bytes: &[u8],
    input: &[usize],
    outputs: &[usize],
) -> TractResult<Arc<TypedRunnableModel>> {
    let model = tract_tflite::tflite().model_for_read(&mut &*bytes)?;
    ensure!(
        model.input_outlets()?.len() == 1
            && model.input_fact(0)?.shape.as_concrete() == Some(input),
        "Unexpected model input"
    );
    ensure!(
        model.output_outlets()?.len() == outputs.len(),
        "Unexpected model outputs"
    );
    for (i, n) in outputs.iter().enumerate() {
        let f = model.output_fact(i)?;
        ensure!(
            f.datum_type == DatumType::F32
                && f.shape
                    .as_concrete()
                    .is_some_and(|s| s.iter().product::<usize>() == *n),
            "Unexpected output shape"
        );
    }
    model.into_optimized()?.into_runnable()
}
fn checked_model(
    path: &Path,
    digest: &str,
    input: &[usize],
    outputs: &[usize],
) -> Result<Arc<TypedRunnableModel>, ModelError> {
    let bytes = verified_bytes(path, digest)?;
    parse_model(&bytes, input, outputs).map_err(|e| ModelError::Invalid {
        path: path.to_path_buf(),
        reason: format!("{e:#}"),
    })
}
/// Pinned sha256 of the shipped detector model (also listed in assets/models/tobii/MANIFEST.json).
pub const DETECTOR_SHA256: &str =
    "b4578f35940bf5a1a655214a1cce5cab13eba73c1297cd78e1a04c2380b0152f";
/// Pinned sha256 of the shipped face-landmark (mesh) model.
pub const MESH_SHA256: &str = "c7d54204ce0448474c7f3fa9af494787c0965cbdd6f20fc72867e43046bd43d5";
pub const DETECTOR_FILE: &str = "face_detector.tflite";
pub const MESH_FILE: &str = "face_landmarks_detector.tflite";
impl Estimator {
    /// `root` is the directory holding the two `.tflite` files (assets/models/tobii).
    /// Both files are sha256-verified before they are parsed.
    pub fn load(root: &Path) -> Result<Self, ModelError> {
        Ok(Self {
            detector: checked_model(
                &root.join(DETECTOR_FILE),
                DETECTOR_SHA256,
                &[1, 128, 128, 3],
                &[896 * 16, 896],
            )?,
            mesh: checked_model(
                &root.join(MESH_FILE),
                MESH_SHA256,
                &[1, 256, 256, 3],
                &[1434, 1, 1],
            )?,
        })
    }
    /// Optional audit directory receives exact model inputs/outputs for offline parity.
    /// The caller must explicitly create/select it; ordinary inference writes nothing.
    pub fn evaluate(&self, image: &GrayImage, audit: Option<&Path>) -> TractResult<Evaluation> {
        let start = Instant::now();
        let invalid = |reason: &str| Evaluation {
            estimate: None,
            reason: reason.into(),
            elapsed_ms: start.elapsed().as_secs_f32() * 1000.,
        };
        ensure!(
            image.width == 280 && image.height == 280 && image.pixels.len() == 280 * 280,
            "Only measured 280x280 IR mode supported"
        );
        let full = Crop {
            cx: 140.,
            cy: 140.,
            size: 280.,
            angle: 0.,
        };
        let input = resample(image, full, 128, true);
        let outputs = run(&self.detector, &input, 128, audit, "detector")?;
        let detections = decode_detections(&outputs[0], &outputs[1])?;
        if detections.is_empty() {
            return Ok(invalid("No face detected"));
        }
        if detections.len() != 1 {
            return Ok(invalid("Multiple faces; no identity selected"));
        }
        let d = &detections[0];
        let crop = detection_crop(d);
        if !crop.size.is_finite() || !(35.0..=420.).contains(&crop.size) || !crop.angle.is_finite()
        {
            return Ok(invalid("Unusable face crop"));
        }
        let input = resample(image, crop, 256, false);
        let outputs = run(&self.mesh, &input, 256, audit, "mesh")?;
        let presence = sigmoid(outputs[1][0]);
        if presence < 0.5 {
            return Ok(invalid("Face landmarks lost"));
        }
        let landmarks = project_landmarks(&outputs[0], crop);
        let Some((angles, rms)) = fit_rotation(&landmarks) else {
            return Ok(invalid("Degenerate or nonfinite geometry"));
        };
        if rms > 0.25 {
            return Ok(invalid("Face geometry fit rejected"));
        }
        let ms = start.elapsed().as_secs_f32() * 1000.;
        Ok(Evaluation {
            reason: "Image-axis fit; head mapping requires captured neutral".into(),
            elapsed_ms: ms,
            estimate: Some(Estimate {
                image_angles_deg: angles,
                detector_score: d.score,
                presence_score: presence,
                fit_relative_rms: rms,
                inference_ms: ms,
                landmarks,
                crop,
            }),
        })
    }
}
/// Face crop in 280x280 image pixels from a decoded detection (extracted from `evaluate`).
fn detection_crop(d: &Detection) -> Crop {
    let v = d.values;
    Crop {
        cx: v[0] * 280.,
        cy: v[1] * 280.,
        size: v[2].max(v[3]) * 280. * 1.5,
        angle: (v[7] - v[5]).atan2(v[6] - v[4]),
    }
}
/// Mesh-model output (256-px crop space) back into image pixels (extracted from `evaluate`).
fn project_landmarks(raw: &[f32], crop: Crop) -> Vec<[f32; 3]> {
    let (sn, cs) = crop.angle.sin_cos();
    raw.as_chunks::<3>()
        .0
        .iter()
        .map(|p| {
            let x = (p[0] / 256. - 0.5) * crop.size;
            let y = (p[1] / 256. - 0.5) * crop.size;
            [
                crop.cx + cs * x - sn * y,
                crop.cy + sn * x + cs * y,
                p[2] / 256. * crop.size,
            ]
        })
        .collect()
}
fn run(
    model: &Arc<TypedRunnableModel>,
    input: &[f32],
    size: usize,
    audit: Option<&Path>,
    name: &str,
) -> TractResult<Vec<Vec<f32>>> {
    let out = model.run(tvec!(Tensor::from_shape(&[1, size, size, 3], input)?.into()))?;
    let values: Vec<Vec<f32>> = out
        .iter()
        .map(|t| Ok(t.to_plain_array_view::<f32>()?.iter().copied().collect()))
        .collect::<TractResult<_>>()?;
    ensure!(
        values.iter().flatten().all(|v| v.is_finite()),
        "Nonfinite inference result"
    );
    if let Some(dir) = audit {
        fn save(path: std::path::PathBuf, values: &[f32]) -> std::io::Result<()> {
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)?;
            for v in values {
                f.write_all(&v.to_le_bytes())?;
            }
            Ok(())
        }
        save(dir.join(format!("{name}-input.f32")), input)?;
        for (i, v) in values.iter().enumerate() {
            save(dir.join(format!("{name}-output-{i}.f32")), v)?;
        }
    }
    Ok(values)
}
fn sigmoid(v: f32) -> f32 {
    1. / (1. + (-v.clamp(-100., 100.)).exp())
}
/// Pixel-center bilinear sampling, zero padding, replicated gray RGB. No temporal smoothing.
fn resample(image: &GrayImage, crop: Crop, size: usize, detector: bool) -> Vec<f32> {
    let (sn, cs) = crop.angle.sin_cos();
    let mut out = Vec::with_capacity(size * size * 3);
    let sample = |x: i32, y: i32| -> f32 {
        if x < 0 || y < 0 || x >= image.width as i32 || y >= image.height as i32 {
            0.
        } else {
            image.pixels[y as usize * image.width + x as usize] as f32
        }
    };
    for y in 0..size {
        for x in 0..size {
            let u = ((x as f32 + 0.5) / size as f32 - 0.5) * crop.size;
            let v = ((y as f32 + 0.5) / size as f32 - 0.5) * crop.size;
            let px = crop.cx + cs * u - sn * v - 0.5;
            let py = crop.cy + sn * u + cs * v - 0.5;
            let ix = px.floor() as i32;
            let iy = py.floor() as i32;
            let fx = px - ix as f32;
            let fy = py - iy as f32;
            let gray = (sample(ix, iy) * (1. - fx) + sample(ix + 1, iy) * fx) * (1. - fy)
                + (sample(ix, iy + 1) * (1. - fx) + sample(ix + 1, iy + 1) * fx) * fy;
            let value = if detector {
                gray / 127.5 - 1.
            } else {
                gray / 255.
            };
            out.extend([value; 3]);
        }
    }
    out
}
fn iou(a: &[f32; 16], b: &[f32; 16]) -> f32 {
    let w = ((a[0] + a[2] / 2.).min(b[0] + b[2] / 2.) - (a[0] - a[2] / 2.).max(b[0] - b[2] / 2.))
        .max(0.);
    let h = ((a[1] + a[3] / 2.).min(b[1] + b[3] / 2.) - (a[1] - a[3] / 2.).max(b[1] - b[3] / 2.))
        .max(0.);
    w * h / (a[2] * a[3] + b[2] * b[3] - w * h).max(1e-9)
}
fn decode_detections(boxes: &[f32], scores: &[f32]) -> TractResult<Vec<Detection>> {
    ensure!(
        boxes.len() == 896 * 16 && scores.len() == 896,
        "Wrong detector tensor length"
    );
    ensure!(
        boxes.iter().chain(scores).all(|v| v.is_finite()),
        "Nonfinite detection"
    );
    let mut all = Vec::new();
    let mut i = 0;
    // Google short-range anchor layout: stride 8 repeated twice, stride 16 six times.
    for (grid, repeats) in [(16, 2), (8, 6)] {
        for y in 0..grid {
            for x in 0..grid {
                for _ in 0..repeats {
                    let score = sigmoid(scores[i]);
                    if score >= 0.6 {
                        let mut v = [0.; 16];
                        v.copy_from_slice(&boxes[i * 16..(i + 1) * 16]);
                        for n in 0..8 {
                            v[n * 2] = v[n * 2] / 128. + (x as f32 + 0.5) / grid as f32;
                            v[n * 2 + 1] = v[n * 2 + 1] / 128. + (y as f32 + 0.5) / grid as f32;
                        }
                        v[2] = boxes[i * 16 + 2] / 128.;
                        v[3] = boxes[i * 16 + 3] / 128.;
                        if v[2] > 0. && v[3] > 0. {
                            all.push(Detection { score, values: v });
                        }
                    }
                    i += 1;
                }
            }
        }
    }
    all.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept = Vec::new();
    while !all.is_empty() {
        let top = all.remove(0);
        let mut sum = top.score;
        let mut values = top.values.map(|v| v * top.score);
        all.retain(|d| {
            if iou(&top.values, &d.values) > 0.3 {
                sum += d.score;
                for (v, x) in values.iter_mut().zip(d.values) {
                    *v += x * d.score;
                }
                false
            } else {
                true
            }
        });
        kept.push(Detection {
            score: top.score,
            values: values.map(|v| v / sum),
        });
        if kept.len() > 1 {
            break;
        }
    }
    Ok(kept)
}
/// Weighted similarity fit to learned screen-space 3D, not metric camera geometry.
/// Basis conversion is mathematical; it does not establish physical head-axis signs.
pub fn fit_rotation(landmarks: &[[f32; 3]]) -> Option<([f32; 3], f32)> {
    if landmarks.len() != 478 || landmarks.iter().flatten().any(|v| !v.is_finite()) {
        return None;
    }
    let mut source = Vec::new();
    let mut target = Vec::new();
    let mut weights = Vec::new();
    for &(id, p, w) in geometry_data::BASIS {
        source.push(Vector3::from(p));
        target.push(Vector3::from(landmarks[id].map(f64::from)));
        weights.push(w);
    }
    fit(&source, &target, &weights)
}
fn fit(
    source: &[Vector3<f64>],
    target: &[Vector3<f64>],
    weights: &[f64],
) -> Option<([f32; 3], f32)> {
    let total: f64 = weights.iter().sum();
    let mean = |points: &[Vector3<f64>]| {
        points
            .iter()
            .zip(weights)
            .fold(Vector3::zeros(), |a, (p, w)| a + p * *w)
            / total
    };
    let a = mean(source);
    let b = mean(target);
    let mut h = Matrix3::zeros();
    let mut variance = 0.;
    let mut target_var = 0.;
    for ((p, q), w) in source.iter().zip(target).zip(weights) {
        h += (p - a) * (q - b).transpose() * *w;
        variance += (p - a).norm_squared() * w;
        target_var += (q - b).norm_squared() * w;
    }
    if variance < 1e-9 || target_var < 1e-9 {
        return None;
    }
    let svd = h.svd(true, true);
    if svd.singular_values[1] < svd.singular_values[0] * 1e-5 {
        return None;
    }
    let u = svd.u?;
    let v = svd.v_t?.transpose();
    let mut correction = Matrix3::identity();
    correction[(2, 2)] = (v * u.transpose()).determinant().signum();
    let rotation = v * correction * u.transpose();
    let scale = (svd.singular_values[0]
        + svd.singular_values[1]
        + correction[(2, 2)] * svd.singular_values[2])
        / variance;
    let residual = source
        .iter()
        .zip(target)
        .zip(weights)
        .map(|((p, q), w)| ((rotation * (p - a) * scale + b) - q).norm_squared() * w)
        .sum::<f64>();
    let (x, y, z) = Rotation3::from_matrix_unchecked(rotation).euler_angles();
    let angles = [
        x.to_degrees() as f32,
        y.to_degrees() as f32,
        z.to_degrees() as f32,
    ];
    let rms = (residual / target_var).sqrt() as f32;
    (angles.iter().all(|v| v.is_finite()) && rms.is_finite()).then_some((angles, rms))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn similarity_fit_recovers_rotation_despite_translation_and_scale() {
        let a: Vec<_> = geometry_data::BASIS
            .iter()
            .map(|(_, p, _)| Vector3::from(*p))
            .collect();
        let weights: Vec<_> = geometry_data::BASIS.iter().map(|(_, _, w)| *w).collect();
        for (x, y, z) in [(0., 0., 0.), (0.3, -0.5, 0.2), (-0.4, 0.6, -0.3)] {
            let r = Rotation3::from_euler_angles(x, y, z);
            let b: Vec<_> = a
                .iter()
                .map(|p| r * p * 15. + Vector3::new(150., 110., -20.))
                .collect();
            let (angles, rms) = fit(&a, &b, &weights).unwrap();
            for (v, e) in angles.into_iter().zip([x, y, z]) {
                assert!((v as f64 - e.to_degrees()).abs() < 1e-4);
            }
            assert!(rms < 1e-6);
        }
    }
    #[test]
    fn invalid_geometry_rejected() {
        assert!(fit_rotation(&vec![[0.; 3]; 478]).is_none());
        assert!(fit_rotation(&vec![[f32::NAN; 3]; 478]).is_none());
        assert!(fit_rotation(&vec![[0.; 3]; 477]).is_none());
    }
    #[test]
    fn detector_rejects_bad_shapes_and_nonfinite() {
        assert!(decode_detections(&[], &[]).is_err());
        assert!(decode_detections(&vec![0.; 896 * 16], &vec![f32::NAN; 896]).is_err());
        assert!(decode_detections(&vec![0.; 896 * 16], &vec![-100.; 896])
            .unwrap()
            .is_empty());
    }
    #[test]
    fn sampling_preserves_pixel_centers() {
        let img = GrayImage::new(2, 2, 2, &[0, 64, 128, 255]).unwrap();
        let a = resample(
            &img,
            Crop {
                cx: 1.,
                cy: 1.,
                size: 2.,
                angle: 0.,
            },
            2,
            false,
        );
        for (i, p) in img.pixels.iter().enumerate() {
            assert!((a[i * 3] - *p as f32 / 255.).abs() < 1e-6);
        }
    }
}
