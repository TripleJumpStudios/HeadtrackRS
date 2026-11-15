//! Sparse pyramidal Lucas-Kanade optical flow tracker.
//!
//! Tracks an arbitrary set of facial landmarks (5 SCRFD keypoints in the
//! shipping pipeline) from the previous grayscale frame to the current one. ~0.5 ms total for 5 points on a modern CPU,
//! scales linearly with point count — far cheaper than re-running a face
//! model every frame.
//!
//! Pure-Rust implementation; no opencv. The 2×2 linear system per iteration
//! is solved directly via the closed-form determinant inverse rather than
//! through `nalgebra`, keeping the inner loop branch- and allocation-free.
//! Pyramid levels are built once per call (one downsample step per level).
//!
//! ## Algorithm sketch
//!
//! For each landmark we want to find the displacement `(dx, dy)` such that
//! the patch around `(x + dx, y + dy)` in the current frame matches the patch
//! around `(x, y)` in the previous frame. Lucas-Kanade linearises the image
//! around `(x, y)` and solves
//!
//! ```text
//!   G * d = b           (2×2 linear system per iteration)
//!   G   = Σ [ Ix Ix    Ix Iy ]   over the window around (x, y)
//!         Σ [ Ix Iy    Iy Iy ]
//!   b   = Σ [ Ix It ; Iy It ]    where It = I_curr(x+dx, y+dy) - I_prev(x, y)
//! ```
//!
//! `G` only depends on the previous frame and is computed once per level;
//! the inner iteration recomputes `b` each step until `d` converges.
//!
//! Pyramid coarse-to-fine: solve at 1/4 resolution first to absorb large
//! motions, then refine at 1/2 and full resolution. Doubles the achievable
//! per-frame head displacement before tracking breaks.
//!
//! ## Confidence
//!
//! The minimum eigenvalue of `G` (Shi-Tomasi corner score) is the standard
//! trackability metric — high = textured neighbourhood, low = ambiguous
//! (uniform region or 1D edge). We normalise by window area so the threshold
//! is window-size-independent.

use image::GrayImage;

/// Half-width of the integration window. Full window is `(2W+1)²` pixels
/// (e.g. W=5 → 11×11). 11×11 is the OpenCV default for sparse LK and gives
/// a good cost/accuracy balance for face landmarks at 720p.
const WIN_HALF: i32 = 5;
const WIN_SIDE: i32 = WIN_HALF * 2 + 1;
const WIN_AREA: f32 = (WIN_SIDE * WIN_SIDE) as f32;

/// Number of refinement iterations at each pyramid level.
const MAX_ITERS: usize = 10;

/// Convergence threshold for `(δdx, δdy)` magnitude in pixels (per-level).
/// Once the residual is sub-tenth-pixel further iteration is wasted work.
const CONVERGE_EPS: f32 = 0.03;

/// Coarse-to-fine pyramid depth. 3 levels = full / half / quarter.
/// 1280×720 quarter = 320×180 — covers ~80 px head displacements per frame
/// before tracking breaks, which is far more than a person can move at 60 fps.
pub const PYRAMID_LEVELS: usize = 3;

/// Minimum-eigenvalue floor (after normalisation by window area). Set very
/// low so we only reject *pure singular* structure tensors (genuinely
/// uniform regions like a wall behind the face). SCRFD's landmark
/// placements (eye centres, mouth corners) routinely land on patches with
/// modest local gradient — a stricter floor caused 0% LK success in
/// practice (see logs from 2026-05-10). Geometric validation now happens
/// downstream in PnP via reprojection RMS, not here.
const MIN_EIGEN_THRESHOLD: f32 = 0.05;

/// Per-pixel intensity-error ceiling. Real faces have specular highlights,
/// expression changes, and camera noise that push residuals well above the
/// 25-RMS originally used. 100 is permissive enough to let normal frame-
/// to-frame appearance shift through; PnP catches truly mistracked points
/// via reprojection error.
const MAX_RESIDUAL_PER_PIXEL: f32 = 100.0;

/// Per-point tracking result. `tracked == false` means the point should be
/// considered lost and the caller should fall back to a full re-detect.
#[derive(Debug, Clone, Copy)]
pub struct TrackedPoint {
    pub point: (f32, f32),
    /// Min eigenvalue of the structure tensor (Shi-Tomasi). Higher = better.
    /// Read by tests; production code looks at `tracked` directly.
    #[allow(dead_code)]
    pub confidence: f32,
    pub tracked: bool,
}

/// Track N points from `prev` to `curr` using pyramidal Lucas-Kanade.
///
/// `prev_pts` are pixel coordinates in the *full-resolution* previous frame.
/// Returned points are in the same coordinate space (full-resolution current
/// frame). Both frames must have identical dimensions. The returned `Vec` has
/// the same length as `prev_pts`.
pub fn track_landmarks(
    prev: &GrayImage,
    curr: &GrayImage,
    prev_pts: &[(f32, f32)],
) -> Vec<TrackedPoint> {
    debug_assert_eq!(prev.dimensions(), curr.dimensions());

    let prev_pyr = build_pyramid(prev, PYRAMID_LEVELS);
    let curr_pyr = build_pyramid(curr, PYRAMID_LEVELS);

    prev_pts
        .iter()
        .map(|&p| track_one(&prev_pyr, &curr_pyr, p))
        .collect()
}

/// One pyramid level — pixel data plus dimensions. Full-resolution level is
/// borrowed; coarser levels are owned downsampled buffers.
struct PyramidLevel {
    data: Vec<u8>,
    w: i32,
    h: i32,
}

impl PyramidLevel {
    fn from_image(img: &GrayImage) -> Self {
        Self {
            data: img.as_raw().to_vec(),
            w: img.width() as i32,
            h: img.height() as i32,
        }
    }

    /// 2× decimation with a simple 2×2 box filter. Aliasing is acceptable
    /// here because we only sample at integer-ish coords from the coarser
    /// level to seed the next iteration; sub-pixel detail is reintroduced
    /// at the finer level anyway.
    fn downsample(&self) -> Self {
        let new_w = self.w / 2;
        let new_h = self.h / 2;
        let mut data = vec![0u8; (new_w * new_h) as usize];
        for y in 0..new_h {
            for x in 0..new_w {
                let sx = x * 2;
                let sy = y * 2;
                let p00 = self.data[(sy * self.w + sx) as usize] as u16;
                let p10 = self.data[(sy * self.w + sx + 1) as usize] as u16;
                let p01 = self.data[((sy + 1) * self.w + sx) as usize] as u16;
                let p11 = self.data[((sy + 1) * self.w + sx + 1) as usize] as u16;
                data[(y * new_w + x) as usize] = ((p00 + p10 + p01 + p11 + 2) >> 2) as u8;
            }
        }
        Self {
            data,
            w: new_w,
            h: new_h,
        }
    }

    /// Bilinear sample at sub-pixel `(x, y)` clamped to image bounds.
    fn sample(&self, x: f32, y: f32) -> f32 {
        let x = x.clamp(0.0, (self.w - 1) as f32);
        let y = y.clamp(0.0, (self.h - 1) as f32);
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = (x0 + 1).min(self.w - 1);
        let y1 = (y0 + 1).min(self.h - 1);
        let fx = x - x0 as f32;
        let fy = y - y0 as f32;

        let p00 = self.data[(y0 * self.w + x0) as usize] as f32;
        let p10 = self.data[(y0 * self.w + x1) as usize] as f32;
        let p01 = self.data[(y1 * self.w + x0) as usize] as f32;
        let p11 = self.data[(y1 * self.w + x1) as usize] as f32;

        let a = p00 * (1.0 - fx) + p10 * fx;
        let b = p01 * (1.0 - fx) + p11 * fx;
        a * (1.0 - fy) + b * fy
    }
}

fn build_pyramid(img: &GrayImage, levels: usize) -> Vec<PyramidLevel> {
    let mut pyr = Vec::with_capacity(levels);
    pyr.push(PyramidLevel::from_image(img));
    for _ in 1..levels {
        let next = pyr.last().expect("at least one level pushed").downsample();
        pyr.push(next);
    }
    pyr
}

/// Track a single point through the pyramid coarse → fine.
fn track_one(
    prev_pyr: &[PyramidLevel],
    curr_pyr: &[PyramidLevel],
    full_pt: (f32, f32),
) -> TrackedPoint {
    // Initial displacement guess: zero. Refine coarse-to-fine.
    let mut dx = 0.0f32;
    let mut dy = 0.0f32;

    // Track confidence is the min-eigenvalue at the finest level.
    let mut confidence = 0.0f32;
    let mut residual = f32::INFINITY;

    for level in (0..prev_pyr.len()).rev() {
        let scale = (1 << level) as f32;
        let inv_scale = 1.0 / scale;
        let prev = &prev_pyr[level];
        let curr = &curr_pyr[level];

        // Point coords at this level.
        let px = full_pt.0 * inv_scale;
        let py = full_pt.1 * inv_scale;

        // Reject if the integration window can't fit at this level (point
        // is too close to the border). For the coarser levels this almost
        // never happens because we're well inside a typical face crop.
        if px < WIN_HALF as f32
            || py < WIN_HALF as f32
            || px > (prev.w - WIN_HALF - 1) as f32
            || py > (prev.h - WIN_HALF - 1) as f32
        {
            return TrackedPoint {
                point: full_pt,
                confidence: 0.0,
                tracked: false,
            };
        }

        // Precompute structure tensor G and gradient samples on the prev frame.
        // Spatial gradients via centred differences.
        let mut sum_ix2 = 0.0f32;
        let mut sum_iy2 = 0.0f32;
        let mut sum_ixy = 0.0f32;
        let mut grads: [[f32; (WIN_SIDE * WIN_SIDE) as usize]; 2] =
            [[0.0; (WIN_SIDE * WIN_SIDE) as usize]; 2];
        let mut prev_samples = [0.0f32; (WIN_SIDE * WIN_SIDE) as usize];

        let mut idx = 0;
        for dy_w in -WIN_HALF..=WIN_HALF {
            for dx_w in -WIN_HALF..=WIN_HALF {
                let sx = px + dx_w as f32;
                let sy = py + dy_w as f32;
                let ix = 0.5 * (prev.sample(sx + 1.0, sy) - prev.sample(sx - 1.0, sy));
                let iy = 0.5 * (prev.sample(sx, sy + 1.0) - prev.sample(sx, sy - 1.0));
                grads[0][idx] = ix;
                grads[1][idx] = iy;
                prev_samples[idx] = prev.sample(sx, sy);
                sum_ix2 += ix * ix;
                sum_iy2 += iy * iy;
                sum_ixy += ix * iy;
                idx += 1;
            }
        }

        let det = sum_ix2 * sum_iy2 - sum_ixy * sum_ixy;
        if det.abs() < 1e-6 {
            // Singular structure tensor → uniform region; can't track.
            return TrackedPoint {
                point: full_pt,
                confidence: 0.0,
                tracked: false,
            };
        }
        let inv_det = 1.0 / det;

        // Iteratively refine the displacement at this level.
        let mut sum_residual_sq = 0.0f32;
        for _ in 0..MAX_ITERS {
            let mut sum_ixt = 0.0f32;
            let mut sum_iyt = 0.0f32;
            sum_residual_sq = 0.0;
            let mut k = 0;
            for dy_w in -WIN_HALF..=WIN_HALF {
                for dx_w in -WIN_HALF..=WIN_HALF {
                    let curr_x = px + dx + dx_w as f32;
                    let curr_y = py + dy + dy_w as f32;
                    let it = curr.sample(curr_x, curr_y) - prev_samples[k];
                    sum_ixt += grads[0][k] * it;
                    sum_iyt += grads[1][k] * it;
                    sum_residual_sq += it * it;
                    k += 1;
                }
            }

            // Solve G * Δ = -b   →   Δ = -G⁻¹ b   (closed-form 2×2 inverse).
            let ddx = -inv_det * (sum_iy2 * sum_ixt - sum_ixy * sum_iyt);
            let ddy = -inv_det * (sum_ix2 * sum_iyt - sum_ixy * sum_ixt);

            dx += ddx;
            dy += ddy;

            if (ddx * ddx + ddy * ddy).sqrt() < CONVERGE_EPS {
                break;
            }
        }
        residual = (sum_residual_sq / WIN_AREA).sqrt();

        // Min eigenvalue of [[a, c], [c, b]] = ((a+b) − sqrt((a−b)² + 4c²)) / 2.
        let trace = sum_ix2 + sum_iy2;
        let diff = sum_ix2 - sum_iy2;
        let min_eig = 0.5 * (trace - (diff * diff + 4.0 * sum_ixy * sum_ixy).sqrt());
        confidence = min_eig / WIN_AREA;

        // Promote the displacement to the next finer level.
        if level > 0 {
            dx *= 2.0;
            dy *= 2.0;
        }
    }

    let new_pt = (full_pt.0 + dx, full_pt.1 + dy);
    let tracked = confidence >= MIN_EIGEN_THRESHOLD && residual <= MAX_RESIDUAL_PER_PIXEL;

    TrackedPoint {
        point: new_pt,
        confidence,
        tracked,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_textured(w: u32, h: u32, dx: i32, dy: i32) -> GrayImage {
        // A repeated diagonal-stripe pattern is high-contrast and full-rank:
        // both Ix and Iy are non-zero, so the structure tensor is invertible
        // and Lucas-Kanade can solve for displacement unambiguously.
        let mut img = GrayImage::new(w, h);
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let v = ((((x + dx) ^ (y + dy)) * 7) & 0xFF) as u8;
                img.put_pixel(x as u32, y as u32, image::Luma([v]));
            }
        }
        img
    }

    #[test]
    fn pure_translation_recovered() {
        let prev = synthetic_textured(160, 120, 0, 0);
        let curr = synthetic_textured(160, 120, -3, -2); // image content moved by (+3, +2)
        let pts = [(80.0, 60.0); 5];
        let result = track_landmarks(&prev, &curr, &pts);
        for r in result.iter() {
            assert!(r.tracked, "expected to track synthetic pattern");
            assert!((r.point.0 - 83.0).abs() < 0.5, "x off: got {}", r.point.0);
            assert!((r.point.1 - 62.0).abs() < 0.5, "y off: got {}", r.point.1);
        }
    }

    #[test]
    fn uniform_region_not_tracked() {
        // Solid grey patch — gradient matrix is singular, should report not tracked.
        let prev = GrayImage::from_pixel(160, 120, image::Luma([128]));
        let curr = GrayImage::from_pixel(160, 120, image::Luma([128]));
        let pts = [(80.0, 60.0); 5];
        let result = track_landmarks(&prev, &curr, &pts);
        for r in result.iter() {
            assert!(!r.tracked, "should not track on uniform region");
        }
    }
}
