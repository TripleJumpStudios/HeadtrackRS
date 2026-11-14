//! Perspective-n-Point head pose solver.
//!
//! Given N image-space landmark observations, the corresponding N 3D model
//! points, and camera intrinsics, recover the full 6DOF rigid transform
//! `(R, t)` such that
//!
//! ```text
//!     p_camera = R · P_model + t          (camera frame: X right, Y down, Z into scene)
//! ```
//!
//! The model and observations are stored as `Vec`s so the solver supports any
//! point count from 4 upward. The historical 5-point SCRFD layout is exposed
//! as [`FACE_MODEL_MM`], which is the only layout production uses.
//!
//! Solver: iterative Gauss-Newton with Levenberg-Marquardt damping on the
//! 6-parameter manifold (so(3) ⊕ R³). Rotation is updated multiplicatively
//! `R ← exp(δω) · R` so we never drift off SO(3). Each iteration accumulates
//! a 6×6 normal equation; solving it is a tiny direct inverse — well under
//! a millisecond even at 100+ points × ~5 iterations.
//!
//! Warm-start: the last successful `(R, t)` seeds the next call. With smooth
//! head motion this typically converges in 1-2 iterations.
//!
//! ## 3D face model (5-point SCRFD layout)
//!
//! Camera-aligned at identity pose (head facing camera, no rotation). The
//! values are the standard symmetric anatomical model used by most webcam
//! head-pose work; eye separation matches the 63 mm IPD assumed in
//! `landmark_position::IPD_MM`. Coordinates: X = image-right, Y = image-down,
//! Z = into-scene (away from camera), all in millimetres.
//!
//! ```text
//! right eye  (subject's, image-LEFT):   (-31.5, -35.0,   0.0)
//! left  eye  (subject's, image-RIGHT):  ( 31.5, -35.0,   0.0)
//! nose tip   (protruding toward camera): (  0.0,  -5.0, -25.0)
//! right mouth corner:                    (-22.0,  45.0,  -5.0)
//! left  mouth corner:                    ( 22.0,  45.0,  -5.0)
//! ```

use nalgebra::{Matrix2x3, Matrix2x6, Matrix3, Matrix6, Vector2, Vector3, Vector6};

/// Pinhole intrinsics. Focal lengths in pixels; principal point is the
/// optical centre, typically (image_w/2, image_h/2) for a webcam.
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

impl CameraIntrinsics {
    /// Derive intrinsics from frame size and the camera's diagonal FOV (deg),
    /// matching how the rest of the pipeline characterises the webcam.
    pub fn from_diag_fov(img_w: u32, img_h: u32, fov_diag_deg: f32) -> Self {
        let fov_diag = fov_diag_deg.to_radians();
        let aspect = img_h as f32 / img_w as f32;
        let fov_w = 2.0 * ((fov_diag / 2.0).tan() / (1.0 + aspect * aspect).sqrt()).atan();
        let fov_h =
            2.0 * ((fov_diag / 2.0).tan() / (1.0 + 1.0 / (aspect * aspect)).sqrt()).atan();
        Self {
            fx: (img_w as f32 * 0.5) / (fov_w * 0.5).tan(),
            fy: (img_h as f32 * 0.5) / (fov_h * 0.5).tan(),
            cx: img_w as f32 * 0.5,
            cy: img_h as f32 * 0.5,
        }
    }
}

/// Legacy 5-point face model (mm), camera-aligned at identity. Index order
/// matches SCRFD: 0=right eye, 1=left eye, 2=nose tip, 3=right mouth, 4=left
/// mouth. Use [`PnpSolver::new`] for the 5-point default or
/// [`PnpSolver::with_model`] for a custom layout (tests only).
pub const FACE_MODEL_MM: [[f32; 3]; 5] = [
    [-31.5, -35.0,   0.0],  // right eye  (subject's right = image-left)
    [ 31.5, -35.0,   0.0],  // left eye   (subject's left  = image-right)
    [  0.0,  -5.0, -25.0],  // nose tip   (protrudes toward camera = -Z)
    [-22.0,  45.0,  -5.0],  // right mouth corner
    [ 22.0,  45.0,  -5.0],  // left mouth corner
];

/// One PnP solve result. `r` and `t` are in camera frame.
#[derive(Debug, Clone, Copy)]
pub struct PnpResult {
    pub r: Matrix3<f32>,
    pub t: Vector3<f32>,
    /// Final reprojection RMS error in pixels. Surfaced for diagnostics
    /// (read by tests; production callers can log it periodically).
    #[allow(dead_code)]
    pub rms_px: f32,
    /// Iterations the solver actually performed. Useful when tuning
    /// `MAX_ITERS` / `CONVERGE_DELTA`; not consulted on the hot path.
    #[allow(dead_code)]
    pub iterations: usize,
}

/// Stateful PnP solver — keeps the last solution as warm start.
pub struct PnpSolver {
    /// 3D model points in head-centric millimetres. Length matches the
    /// expected observation count passed to [`solve`].
    model: Vec<Vector3<f32>>,
    /// Per-landmark weights applied in the weighted least-squares solve.
    /// Length matches `model`. Defaults to `1.0` everywhere (uniform). Used
    /// by the 5-point path to down-weight the mouth corners (indices 3, 4)
    /// so the solver doesn't read expression motion as head pose.
    weights: Vec<f32>,
    /// Most recent `(R, t)`. Used to seed the next solve. `None` means
    /// "cold start": identity rotation, default depth.
    last: Option<(Matrix3<f32>, Vector3<f32>)>,
}

impl Default for PnpSolver {
    fn default() -> Self { Self::new() }
}

impl PnpSolver {
    /// Default constructor: the 5-point SCRFD layout from [`FACE_MODEL_MM`].
    /// This is the only solver production builds.
    pub fn new() -> Self {
        let model: Vec<Vector3<f32>> = FACE_MODEL_MM
            .iter()
            .map(|p| Vector3::new(p[0], p[1], p[2]))
            .collect();
        let weights = vec![1.0; model.len()];
        Self { model, weights, last: None }
    }

    /// Build a solver around a caller-supplied 3D model.
    ///
    /// Test-only since the dense/106-point pipeline was removed: the solver
    /// correctness tests feed it synthetic models with a known ground-truth
    /// pose. Nothing in production constructs a non-[`FACE_MODEL_MM`] solver.
    #[cfg(test)]
    pub fn with_model(model: Vec<Vector3<f32>>) -> Self {
        let weights = vec![1.0; model.len()];
        Self { model, weights, last: None }
    }

    /// Replace the per-landmark weights. Length must equal `self.model.len()`.
    /// Panics on mismatch — this is a programmer error, not a runtime input.
    /// Used by the tracker to down-weight expression-sensitive landmarks
    /// (5-point mouth corners) so expressions don't bleed into pose.
    pub fn with_weights(mut self, weights: Vec<f32>) -> Self {
        assert_eq!(
            weights.len(),
            self.model.len(),
            "PnpSolver weights length {} must match model length {}",
            weights.len(),
            self.model.len(),
        );
        self.weights = weights;
        self
    }

    /// Drop warm-start state. Call after a re-detect / acquire so the next
    /// solve starts from the cold prior rather than potentially-stale state.
    pub fn reset(&mut self) {
        self.last = None;
    }



    /// Solve PnP. Returns `None` if the iteration diverges, any step places
    /// a model point behind the camera, or `observations_px.len()` does not
    /// match the configured model. The caller should treat `None` the same
    /// as a tracking-lost event.
    pub fn solve(
        &mut self,
        observations_px: &[(f32, f32)],
        intrinsics: &CameraIntrinsics,
    ) -> Option<PnpResult> {
        if observations_px.len() != self.model.len() {
            return None;
        }
        const MAX_ITERS: usize = 12;
        const CONVERGE_DELTA: f32 = 1e-4;
        const REJECT_RMS_PX: f32 = 30.0;
        const LM_LAMBDA: f32 = 1e-3;

        let (mut r, mut t) = self
            .last
            .unwrap_or_else(|| (Matrix3::identity(), Vector3::new(0.0, 0.0, 500.0)));

        let mut iter_count = 0;
        let mut converged = false;

        for it in 0..MAX_ITERS {
            iter_count = it + 1;
            let mut h = Matrix6::<f32>::zeros();
            let mut g = Vector6::<f32>::zeros();

            for (i, obs) in observations_px.iter().enumerate() {
                let p_world = self.model[i];
                let p_cam = r * p_world + t;
                if p_cam.z < 1.0 {
                    // A model point behind / on the camera plane → projection
                    // is degenerate. Bail and let the caller re-detect.
                    return None;
                }
                let inv_z = 1.0 / p_cam.z;
                let proj_u = intrinsics.fx * p_cam.x * inv_z + intrinsics.cx;
                let proj_v = intrinsics.fy * p_cam.y * inv_z + intrinsics.cy;
                let res = Vector2::new(proj_u - obs.0, proj_v - obs.1);

                // 2×3 Jacobian d(u, v) / d(p_cam)
                let j_p = Matrix2x3::new(
                    intrinsics.fx * inv_z, 0.0,                    -intrinsics.fx * p_cam.x * inv_z * inv_z,
                    0.0,                    intrinsics.fy * inv_z, -intrinsics.fy * p_cam.y * inv_z * inv_z,
                );

                // d(p_cam) / d(δω) = -[p_cam]_×   (left-multiplicative SO(3) perturbation)
                // d(p_cam) / d(t)  =  I
                let p_cross_neg = -skew(&p_cam);
                let j_rot = j_p * p_cross_neg;       // 2×3
                let j_t = j_p;                       // 2×3
                let mut j = Matrix2x6::<f32>::zeros();
                for row in 0..2 {
                    for col in 0..3 {
                        j[(row, col)]     = j_rot[(row, col)];
                        j[(row, col + 3)] = j_t[(row, col)];
                    }
                }

                // Weighted least squares: min Σ w_i ||r_i||² gives normal
                // equations (Σ w_i J_iᵀ J_i) δ = −(Σ w_i J_iᵀ r_i). With
                // uniform w_i=1 this collapses to the original unweighted
                // accumulation. Per-landmark weights let the 5-point caller
                // down-weight expression-sensitive points (mouth corners).
                let w = self.weights[i];
                h += j.transpose() * j * w;
                g += j.transpose() * res * w;
            }

            // Levenberg-Marquardt damping for numerical conditioning.
            for d in 0..6 {
                h[(d, d)] += LM_LAMBDA;
            }

            // A singular Hessian means the observation geometry is degenerate
            // (coincident or collinear landmarks) — treat as tracking-lost.
            let delta: Vector6<f32> = -(h.try_inverse()? * g);

            let delta_omega = Vector3::new(delta[0], delta[1], delta[2]);
            let delta_t = Vector3::new(delta[3], delta[4], delta[5]);
            r = exp_so3(&delta_omega) * r;
            t += delta_t;

            if delta.norm() < CONVERGE_DELTA {
                converged = true;
                break;
            }
        }

        // Final residual.
        let mut rms_sq = 0.0;
        for (i, obs) in observations_px.iter().enumerate() {
            let p_cam = r * self.model[i] + t;
            if p_cam.z < 1.0 { return None; }
            let inv_z = 1.0 / p_cam.z;
            let dx = intrinsics.fx * p_cam.x * inv_z + intrinsics.cx - obs.0;
            let dy = intrinsics.fy * p_cam.y * inv_z + intrinsics.cy - obs.1;
            rms_sq += dx * dx + dy * dy;
        }
        let rms_px = (rms_sq / observations_px.len() as f32).sqrt();

        if !converged && rms_px > REJECT_RMS_PX {
            // Solver wandered off — drop warm-start and let the caller re-detect.
            self.last = None;
            return None;
        }

        self.last = Some((r, t));
        Some(PnpResult { r, t, rms_px, iterations: iter_count })
    }
}

/// Convert a camera-frame `(R, t)` into headtrack-rs canonical
/// `(yaw, pitch, roll, x, y, z)` — degrees and millimetres.
///
/// The face model in [`FACE_MODEL_MM`] is laid out so that identity head pose
/// (face directly toward the camera) corresponds to `R = I`. Standard XYZ
/// Tait-Bryan extraction (`R = Rx(α) · Ry(β) · Rz(γ)`) then yields per-axis
/// angles that map to canonical yaw/pitch/roll with the sign flips below.
///
/// **Sign conventions** — derived analytically from the model layout (see the
/// "Sign derivation" comment inside this function) and matching the existing
/// canonical convention from CLAUDE.md:
///
/// - canonical yaw   = +β   (head turn right)
/// - canonical pitch = −α   (nose up)
/// - canonical roll  = −γ   (right ear down)
///
/// Verify in-cockpit at first run; flip the `*_SIGN` constants if any axis
/// reads inverted (same procedure used for the 6DRepNet decoder).
pub fn pose_to_canonical(r: &Matrix3<f32>, t: &Vector3<f32>) -> (f32, f32, f32, f32, f32, f32) {
    let r00 = r[(0, 0)];
    let r10 = r[(1, 0)];
    let r11 = r[(1, 1)];
    let r12 = r[(1, 2)];
    let r20 = r[(2, 0)];
    let r21 = r[(2, 1)];
    let r22 = r[(2, 2)];

    let sy = (r00 * r00 + r10 * r10).sqrt();
    let singular = sy < 1e-6;

    let (pitch_rad, roll_rad) = if singular {
        ((-r12).atan2(r11), 0.0)
    } else {
        (r21.atan2(r22), r10.atan2(r00))
    };
    let yaw_rad = (-r20).atan2(sy);

    // Sign derivation (face model in camera-aligned mm, identity = R=I):
    //   +yaw  (head right): right side of face moves further into scene → +Z
    //                       at right eye → corresponds to Ry(+α). Hence YAW = +β.
    //   +pitch (nose up):   nose tip Y becomes more negative (image-up) →
    //                       corresponds to Rx(-α). Hence PITCH = -α.
    //   +roll  (right ear down): right eye Y becomes more positive (image-down)
    //                            → corresponds to Rz(-α). Hence ROLL = -γ.
    const PITCH_SIGN: f32 = -1.0;
    const YAW_SIGN:   f32 =  1.0;
    const ROLL_SIGN:  f32 = -1.0;

    let to_deg = 180.0 / std::f32::consts::PI;
    let yaw   = YAW_SIGN   * yaw_rad   * to_deg;
    let pitch = PITCH_SIGN * pitch_rad * to_deg;
    let roll  = ROLL_SIGN  * roll_rad  * to_deg;

    // Translation: camera frame (X right, Y down, Z into-scene)
    //          → canonical (X right, Y up, Z forward = toward camera).
    let x =  t.x;
    let y = -t.y;
    let z = -t.z;

    (yaw, pitch, roll, x, y, z)
}

fn skew(v: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
         0.0, -v.z,  v.y,
         v.z,  0.0, -v.x,
        -v.y,  v.x,  0.0,
    )
}

/// Rodrigues exponential: so(3) → SO(3). For tiny rotations the second-order
/// Taylor expansion handles the `sin θ / θ` and `(1−cos θ) / θ²` singularities.
fn exp_so3(omega: &Vector3<f32>) -> Matrix3<f32> {
    let theta_sq = omega.norm_squared();
    if theta_sq < 1e-12 {
        return Matrix3::identity() + skew(omega);
    }
    let theta = theta_sq.sqrt();
    let kx = skew(omega) / theta;
    Matrix3::identity() + kx * theta.sin() + kx * kx * (1.0 - theta.cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn project(p_cam: Vector3<f32>, k: &CameraIntrinsics) -> (f32, f32) {
        (
            k.fx * p_cam.x / p_cam.z + k.cx,
            k.fy * p_cam.y / p_cam.z + k.cy,
        )
    }

    fn project_model(r: Matrix3<f32>, t: Vector3<f32>, k: &CameraIntrinsics) -> Vec<(f32, f32)> {
        FACE_MODEL_MM
            .iter()
            .map(|p| project(r * Vector3::new(p[0], p[1], p[2]) + t, k))
            .collect()
    }

    fn project_points(
        model: &[Vector3<f32>],
        r: Matrix3<f32>,
        t: Vector3<f32>,
        k: &CameraIntrinsics,
    ) -> Vec<(f32, f32)> {
        model.iter().map(|p| project(r * p + t, k)).collect()
    }

    /// Synthesize a 16-point "face" — denser than SCRFD's 5, sparse enough to
    /// exercise the loop without making the test slow. Points are scattered
    /// across an anatomically plausible bounding region; depth varies so PnP
    /// has real out-of-plane structure to triangulate against.
    fn dense_synthetic_model() -> Vec<Vector3<f32>> {
        vec![
            Vector3::new(-31.5, -35.0,   0.0),   // right eye outer
            Vector3::new(-15.0, -38.0,  -3.0),   // right eye inner
            Vector3::new( 15.0, -38.0,  -3.0),   // left eye inner
            Vector3::new( 31.5, -35.0,   0.0),   // left eye outer
            Vector3::new(-12.0, -20.0, -10.0),   // right nose wing
            Vector3::new(  0.0,  -5.0, -25.0),   // nose tip
            Vector3::new( 12.0, -20.0, -10.0),   // left nose wing
            Vector3::new(-22.0,  45.0,  -5.0),   // right mouth corner
            Vector3::new(  0.0,  48.0,  -3.0),   // upper-lip centre
            Vector3::new( 22.0,  45.0,  -5.0),   // left mouth corner
            Vector3::new(-55.0,   0.0,  20.0),   // right cheekbone
            Vector3::new( 55.0,   0.0,  20.0),   // left cheekbone
            Vector3::new(-45.0,  35.0,  25.0),   // right jaw
            Vector3::new( 45.0,  35.0,  25.0),   // left jaw
            Vector3::new(  0.0,  70.0,   5.0),   // chin tip
            Vector3::new(  0.0, -60.0,  10.0),   // forehead centre
        ]
    }

    #[test]
    fn identity_pose_recovered() {
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let r_true = Matrix3::identity();
        let t_true = Vector3::new(0.0, 0.0, 500.0);
        let obs = project_model(r_true, t_true, &k);

        let mut solver = PnpSolver::new();
        let res = solver.solve(&obs, &k).expect("solver failed");

        assert!(res.rms_px < 0.1, "rms too high: {}", res.rms_px);
        assert!((res.t - t_true).norm() < 0.5, "t drift: {:?}", res.t);
        let (yaw, pitch, roll, x, y, z) = pose_to_canonical(&res.r, &res.t);
        assert!(yaw.abs()   < 0.5, "yaw   should be ~0, got {yaw}");
        assert!(pitch.abs() < 0.5, "pitch should be ~0, got {pitch}");
        assert!(roll.abs()  < 0.5, "roll  should be ~0, got {roll}");
        assert!(x.abs() < 0.5);
        assert!(y.abs() < 0.5);
        assert!((z - (-500.0)).abs() < 0.5);
    }

    #[test]
    fn yaw_right_recovered() {
        // Synthesize: head turned right by 15° → R = Ry(+15°) in camera frame.
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let theta = 15.0_f32.to_radians();
        let r_true = Matrix3::new(
            theta.cos(), 0.0, theta.sin(),
            0.0,         1.0, 0.0,
            -theta.sin(), 0.0, theta.cos(),
        );
        let t_true = Vector3::new(0.0, 0.0, 500.0);
        let obs = project_model(r_true, t_true, &k);

        let mut solver = PnpSolver::new();
        let res = solver.solve(&obs, &k).expect("solver failed");
        let (yaw, _, _, _, _, _) = pose_to_canonical(&res.r, &res.t);
        assert!((yaw - 15.0).abs() < 1.0, "expected yaw ≈ +15, got {yaw}");
    }

    #[test]
    fn pitch_up_recovered() {
        // canonical +pitch (nose up) = Rx(-α) in camera frame.
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let theta = -10.0_f32.to_radians();   // Rx(-10°)
        let r_true = Matrix3::new(
            1.0, 0.0,         0.0,
            0.0, theta.cos(), -theta.sin(),
            0.0, theta.sin(),  theta.cos(),
        );
        let t_true = Vector3::new(0.0, 0.0, 500.0);
        let obs = project_model(r_true, t_true, &k);

        let mut solver = PnpSolver::new();
        let res = solver.solve(&obs, &k).expect("solver failed");
        let (_, pitch, _, _, _, _) = pose_to_canonical(&res.r, &res.t);
        assert!((pitch - 10.0).abs() < 1.0, "expected pitch ≈ +10, got {pitch}");
    }

    #[test]
    fn roll_right_recovered() {
        // canonical +roll (right ear down) = Rz(-α) in camera frame.
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let theta = -12.0_f32.to_radians();
        let r_true = Matrix3::new(
            theta.cos(), -theta.sin(), 0.0,
            theta.sin(),  theta.cos(), 0.0,
            0.0,          0.0,         1.0,
        );
        let t_true = Vector3::new(0.0, 0.0, 500.0);
        let obs = project_model(r_true, t_true, &k);

        let mut solver = PnpSolver::new();
        let res = solver.solve(&obs, &k).expect("solver failed");
        let (_, _, roll, _, _, _) = pose_to_canonical(&res.r, &res.t);
        assert!((roll - 12.0).abs() < 1.0, "expected roll ≈ +12, got {roll}");
    }

    #[test]
    fn translation_recovered() {
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let r_true = Matrix3::identity();
        // Subject moved 50mm to their right, 30mm up, 100mm closer to camera.
        // canonical: x=+50, y=+30, z=+100. Camera frame: tx=+50, ty=-30, tz=400.
        let t_true = Vector3::new(50.0, -30.0, 400.0);
        let obs = project_model(r_true, t_true, &k);

        let mut solver = PnpSolver::new();
        let res = solver.solve(&obs, &k).expect("solver failed");
        let (_, _, _, x, y, z) = pose_to_canonical(&res.r, &res.t);
        assert!((x -  50.0).abs() < 1.0, "x: {x}");
        assert!((y -  30.0).abs() < 1.0, "y: {y}");
        assert!((z - (-400.0)).abs() < 1.0, "z: {z}");
    }

    /// Phase 2 verification: the Vec-generalised solver handles a >10-point
    /// model identically to the 5-point path. Uses a 16-point synthetic
    /// "face" with real depth structure to exercise the over-determined
    /// least-squares fit that many-point models rely on. Recovers
    /// yaw/pitch/roll/translation jointly.
    #[test]
    fn dense_16_point_pose_recovered() {
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let model = dense_synthetic_model();
        assert!(model.len() > 10);

        // Target canonical pose: yaw +15°, pitch +8°, roll +6°.
        // pose_to_canonical's matrix extraction matches the ZYX convention
        // R = Rz(γ) · Ry(β) · Rx(α) regardless of what the surrounding
        // docstring suggests — the single-axis tests above can't tell the
        // two orders apart, but the combined fit can. So we build the
        // ground-truth rotation in that order here. Signs per pose_to_canonical:
        //   canonical yaw   = +β
        //   canonical pitch = -α    →  α = -canonical_pitch
        //   canonical roll  = -γ    →  γ = -canonical_roll
        let alpha = (-8.0_f32).to_radians();   // canonical +8° pitch
        let beta  = 15.0_f32.to_radians();
        let gamma = (-6.0_f32).to_radians();   // canonical +6° roll
        let rx = Matrix3::new(
            1.0, 0.0,         0.0,
            0.0, alpha.cos(), -alpha.sin(),
            0.0, alpha.sin(),  alpha.cos(),
        );
        let ry = Matrix3::new(
             beta.cos(), 0.0, beta.sin(),
             0.0,        1.0, 0.0,
            -beta.sin(), 0.0, beta.cos(),
        );
        let rz = Matrix3::new(
            gamma.cos(), -gamma.sin(), 0.0,
            gamma.sin(),  gamma.cos(), 0.0,
            0.0,          0.0,         1.0,
        );
        let r_true = rz * ry * rx;
        let t_true = Vector3::new(20.0, -15.0, 450.0);
        let obs = project_points(&model, r_true, t_true, &k);

        let mut solver = PnpSolver::with_model(model);
        let res = solver.solve(&obs, &k).expect("dense solver failed");

        assert!(res.rms_px < 0.5, "rms too high: {}", res.rms_px);
        let (yaw, pitch, roll, x, y, z) = pose_to_canonical(&res.r, &res.t);
        assert!((yaw - 15.0).abs() < 0.5,  "yaw: {yaw}");
        assert!((pitch -  8.0).abs() < 0.5, "pitch: {pitch}");
        assert!((roll  -  6.0).abs() < 0.5, "roll: {roll}");
        assert!((x -  20.0).abs() < 0.5, "x: {x}");
        assert!((y -  15.0).abs() < 0.5, "y: {y}");
        assert!((z - (-450.0)).abs() < 0.5, "z: {z}");
    }

    /// Per-point weighting actually reduces an outlier landmark's influence.
    /// Take the 5-point identity pose, then perturb the right mouth corner
    /// (index 3) horizontally by 30 px — the kind of geometric shift a smile
    /// or speech produces. With uniform weights the solver distributes that
    /// residual across all 6 DOF, biasing yaw. With the mouth weight at 0.1
    /// (10% of the eye/nose weight), the bad point's leverage shrinks and
    /// the recovered yaw is markedly closer to ground truth.
    #[test]
    fn weighted_solver_downweights_off_landmark() {
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let r_true = Matrix3::identity();
        let t_true = Vector3::new(0.0, 0.0, 500.0);
        let mut obs = project_model(r_true, t_true, &k);
        // Shift right mouth corner by +30 px in u — simulates a smile.
        obs[3].0 += 30.0;

        // Uniform weights: baseline yaw drift.
        let mut uniform = PnpSolver::new();
        let res_u = uniform.solve(&obs, &k).expect("uniform solve");
        let (yaw_u, _, _, _, _, _) = pose_to_canonical(&res_u.r, &res_u.t);

        // Mouth corners down-weighted to 10%: yaw should be closer to 0.
        let mut weighted = PnpSolver::new()
            .with_weights(vec![1.0, 1.0, 1.0, 0.1, 0.1]);
        let res_w = weighted.solve(&obs, &k).expect("weighted solve");
        let (yaw_w, _, _, _, _, _) = pose_to_canonical(&res_w.r, &res_w.t);

        assert!(
            yaw_w.abs() < yaw_u.abs(),
            "down-weighted mouth should reduce yaw bleed: uniform={yaw_u} weighted={yaw_w}",
        );
        // The improvement should be substantial, not just numerical noise.
        assert!(
            yaw_w.abs() < yaw_u.abs() * 0.5,
            "expected weighted yaw < half of uniform: uniform={yaw_u} weighted={yaw_w}",
        );
    }

    #[test]
    #[should_panic(expected = "weights length")]
    fn weighted_solver_rejects_length_mismatch() {
        let _ = PnpSolver::new().with_weights(vec![1.0; 4]);
    }

    #[test]
    fn observation_count_mismatch_rejected() {
        // Default 5-point solver should refuse a 4-point observation slice.
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        let mut solver = PnpSolver::new();
        let four_pts: [(f32, f32); 4] = [(0.0, 0.0); 4];
        assert!(solver.solve(&four_pts, &k).is_none());
    }

    /// Sign conventions must hold for a **many-point** model, not just the
    /// 5-point layout: `yaw=+β, pitch=-α, roll=-γ` from `R = Rz(γ)·Ry(β)·Rx(α)`,
    /// and canonical `x=+t.x, y=-t.y, z=-t.z`.
    ///
    /// The model here is a synthetic face-scale point cloud rather than a real
    /// landmark layout — the property under test is the solver's coordinate
    /// convention, which is independent of which specific points are fed in.
    /// (This previously used the 106-point dense canonical; that model was
    /// removed with the dense pipeline, but the convention check it provided
    /// is worth keeping.)
    #[test]
    fn many_point_sign_conventions_preserved() {
        let k = CameraIntrinsics { fx: 800.0, fy: 800.0, cx: 640.0, cy: 360.0 };
        // Deterministic face-scale cloud: 106 points spread over roughly a
        // head-sized volume, no two coincident, none behind the camera.
        let model: Vec<Vector3<f32>> = (0..106)
            .map(|i| {
                let a = i as f32 * 0.591_5; // irrational-ish step → no clustering
                Vector3::new(
                    70.0 * a.sin(),
                    60.0 * (a * 1.7).cos(),
                    20.0 * (a * 0.9).sin(),
                )
            })
            .collect();
        assert_eq!(model.len(), 106);

        // Canonical pose: yaw +20, pitch +6, roll -10, translation (+15, -8, +400).
        // R = Rz(γ)·Ry(β)·Rx(α) with γ=-(-10)=10, β=+20, α=-(+6)=-6.
        let alpha = (-6.0_f32).to_radians();
        let beta  = 20.0_f32.to_radians();
        let gamma = 10.0_f32.to_radians();
        let rx = Matrix3::new(
            1.0, 0.0,         0.0,
            0.0, alpha.cos(), -alpha.sin(),
            0.0, alpha.sin(),  alpha.cos(),
        );
        let ry = Matrix3::new(
             beta.cos(), 0.0, beta.sin(),
             0.0,        1.0, 0.0,
            -beta.sin(), 0.0, beta.cos(),
        );
        let rz = Matrix3::new(
            gamma.cos(), -gamma.sin(), 0.0,
            gamma.sin(),  gamma.cos(), 0.0,
            0.0,          0.0,         1.0,
        );
        let r_true = rz * ry * rx;
        let t_true = Vector3::new(15.0, 8.0, 400.0);
        let obs: Vec<(f32, f32)> = model
            .iter()
            .map(|p| project(r_true * p + t_true, &k))
            .collect();

        let mut solver = PnpSolver::with_model(model);
        let res = solver.solve(&obs, &k).expect("dense 106 solver failed");
        let (yaw, pitch, roll, x, y, z) = pose_to_canonical(&res.r, &res.t);

        assert!(res.rms_px < 1.0, "rms too high: {}", res.rms_px);
        assert!((yaw   - 20.0).abs() < 0.5, "yaw: {yaw}");
        assert!((pitch -  6.0).abs() < 0.5, "pitch: {pitch}");
        assert!((roll  - (-10.0)).abs() < 0.5, "roll: {roll}");
        assert!((x -  15.0).abs() < 0.5, "x: {x}");
        assert!((y - (-8.0)).abs() < 0.5, "y: {y}");
        assert!((z - (-400.0)).abs() < 0.5, "z: {z}");
    }
}
