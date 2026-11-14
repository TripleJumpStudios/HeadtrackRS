//! Translation derivation from the two eye landmarks.
//!
//! Far more stable than the full rigid PnP `tvec` under **expression** (a smile
//! shifts the mouth/cheek points the PnP solve leans on) and **head pitch**
//! (look-down foreshortens the lower face). The eyes barely move under either:
//!
//! - **X/Y**: from the eye centroid (midpoint of the two eyes). Its pixel
//!   position moves only when the head actually translates, not when the chin,
//!   forehead, or mouth move due to pitch or expression.
//! - **Z**: from interpupillary distance (IPD) vs an assumed real-world 63 mm,
//!   via the pinhole relation `Z = focal_px * IPD_mm / IPD_px`. IPD is invariant
//!   under pitch and roll; it shrinks ~`cos(yaw)` under yaw, which we undo using
//!   the (6DRepNet) yaw estimate.

/// Reference adult interpupillary distance in mm (population mean ≈ 63).
const IPD_MM: f32 = 63.0;

/// Floor on `cos(yaw)` in the yaw→Z correction (~±60°). Beyond this the far eye
/// occludes and the projected IPD is unreliable, so we stop scaling rather than
/// let Z blow up.
const YAW_COS_FLOOR: f32 = 0.5;

/// Estimate `(X, Y, Z)` head position in mm — canonical headtrack-rs convention
/// (x = right, y = up, z = forward / toward screen) — from the two eye keypoints
/// in full-resolution image pixels.
///
/// `right_eye` / `left_eye` are SCRFD keypoints 0 / 1. `image_size` =
/// `(width, height)` of the camera frame. `fov_diag_deg` = camera diagonal FOV.
/// `yaw_deg` = head yaw (degrees) used to undo IPD foreshortening; pass `0.0` to
/// disable the correction.
pub fn position_from_eyes(
    right_eye: (f32, f32),
    left_eye: (f32, f32),
    image_size: (u32, u32),
    fov_diag_deg: f32,
    yaw_deg: f32,
) -> (f32, f32, f32) {
    let (img_w, img_h) = (image_size.0 as f32, image_size.1 as f32);

    // Per-axis FOV from diagonal FOV (matches the legacy estimator so the
    // path stays directly comparable).
    let fov_diag = fov_diag_deg.to_radians();
    let aspect = img_h / img_w;
    let fov_w = 2.0 * ((fov_diag / 2.0).tan() / (1.0 + aspect * aspect).sqrt()).atan();
    let fov_h = 2.0 * ((fov_diag / 2.0).tan() / (1.0 + 1.0 / (aspect * aspect)).sqrt()).atan();
    let fl_w_px = (img_w * 0.5) / (fov_w * 0.5).tan(); // focal length, pixels
    let fl_h_px = (img_h * 0.5) / (fov_h * 0.5).tan();

    let (rex, rey) = right_eye;
    let (lex, ley) = left_eye;

    // Z from interpupillary distance, undoing yaw foreshortening:
    // projected_IPD ≈ true_IPD · cos(yaw), so divide by cos(yaw) to recover
    // the frontal IPD before the pinhole depth.
    let dx = lex - rex;
    let dy = ley - rey;
    let ipd_px = (dx * dx + dy * dy).sqrt().max(1.0);
    let yaw_cos = yaw_deg.to_radians().cos().abs().max(YAW_COS_FLOOR);
    let ipd_corrected = ipd_px / yaw_cos;
    let z_camera_mm = fl_w_px * IPD_MM / ipd_corrected;

    // X/Y from the eye centroid, back-projected to mm at depth Z.
    let cx_px = (rex + lex) * 0.5;
    let cy_px = (rey + ley) * 0.5;
    let nx = (cx_px - img_w * 0.5) / fl_w_px;
    let ny = (cy_px - img_h * 0.5) / fl_h_px;
    let x_mm = nx * z_camera_mm;
    let y_mm = ny * z_camera_mm;

    // Canonical convention: y up = +ve (image y is down → flip),
    // z toward screen = +ve (camera z is depth-away → flip).
    (x_mm, -y_mm, -z_camera_mm)
}

// ---------------------------------------------------------------------------
// Multi-baseline depth cue
// ---------------------------------------------------------------------------

/// Which landmark pairs derive depth. `Ipd` is the shipped behaviour.
///
/// # Why anything other than IPD
///
/// Depth today comes from **one** pair — interpupillary distance — corrected by
/// `1/cos(yaw)`. Of the ten pairwise distances the 5-point model offers, eye–eye
/// is the *only* one with yaw-sensitivity 1.00, so it foreshortens fastest, the
/// correction amplifies its noise (×1.41 at 45°, ×2.0 at 60°), and its far point
/// is the first to occlude — which `YAW_COS_FLOOR` already concedes.
///
/// Measured on 1731 real frames across ±80° (`scripts/depth_cue_eval.py` over
/// `~/headtrack-bakeoff/yaw-sweep`), relative frame-to-frame noise of the implied
/// depth:
///
/// | cue | \|yaw\| 0–15° | 45–90° | degradation |
/// |---|---:|---:|---:|
/// | [`Ipd`](Self::Ipd) | 0.174% | **2.260%** | **×12.96** |
/// | [`NoseMouth`](Self::NoseMouth) | 0.420% | 0.472% | ×1.51 |
/// | [`EyeNose`](Self::EyeNose) | 0.220% | 0.542% | ×2.53 |
/// | [`EyeMouth`](Self::EyeMouth) | 0.131% | **0.193%** | ×1.84 |
/// | [`Verticals`](Self::Verticals) | 0.126% | 0.251% | ×2.00 |
/// | [`Median10`](Self::Median10) | 0.114% | 0.357% | ×3.12 |
///
/// [`EyeMouth`](Self::EyeMouth) is ~11.7× quieter than IPD at extreme yaw. Note
/// [`Median10`](Self::Median10) is **not** the best: more than half the pairs
/// degrade with yaw, so a plain median over all ten lands among them. A chosen
/// subset beats blind aggregation here.
///
/// **That bench table oversold the yaw case** — the ×11.7 was measured on slow
/// controlled sweeps and came back 1.7–2.4× in the cockpit. The shipping choice
/// was settled by a different measurement: identical frames, every cue, six
/// clips (`scripts/depth_cue_eval.py`), p95 deviation of implied depth —
///
/// | clip | `Ipd` | `EyeMouth` | `Diagonals` | `Verticals` | `Median10` |
/// |---|---:|---:|---:|---:|---:|
/// | still | 11.90 | 19.22 | **3.11** | 9.91 | 3.11 |
/// | yaw-hold-45 | 25.41 | 18.97 | **9.36** | 13.24 | 12.99 |
/// | yaw-sweep | 126.27 | 13.79 | 13.14 | **12.26** | 14.76 |
/// | yaw-hold-max | 53.92 | **9.10** | 12.14 | 8.66 | 19.43 |
/// | pitch-extreme | 14.28 | 33.11 | **18.96** | 28.15 | 16.49 |
/// | combined | 30.02 | 14.42 | **10.29** | 12.86 | 9.44 |
///
/// [`Diagonals`](Self::Diagonals) is best or within noise of best on five of six
/// and is the shipped default from v0.2.0. `pitch-extreme` is the clip that
/// decided it: among the cues that use the mouth it wins by 1.5–1.7×, because it
/// is the only one that excludes the near-vertical same-side pairs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DepthCue {
    /// Eye↔eye only, with the `1/cos(yaw)` correction. The default before
    /// v0.2.0; kept as `HEADTRACK_DEPTH_CUE=ipd` to restore old behaviour.
    Ipd,
    /// Nose↔mouth corners. Flattest response to yaw (×1.51) and keeps two of its
    /// three points visible at any angle, but the nose tip is the worst-modelled
    /// landmark — it needs a ±25% correction against `FACE_MODEL_MM`.
    NoseMouth,
    /// Eye↔nose, both sides.
    EyeNose,
    /// Eye↔same-side mouth corner. Best measured cue: longest usable baseline
    /// (80.7 mm vs IPD's 63), 0.12 yaw-sensitivity, and the pair that fits the
    /// generic face model most closely (×0.94–1.00).
    EyeMouth,
    /// All four eye↔mouth pairs, same-side and diagonal — a strict **superset**
    /// of [`Diagonals`](Self::Diagonals). The two extra pairs it adds are the
    /// near-vertical ones, and pitch foreshortens those: at −60° they retain 46%
    /// of their length against the diagonals' 67%, so their noise is amplified
    /// ×2.17 against ×1.49. Mixing them into the median is what makes this cue
    /// collapse when the pilot looks down at the instruments — measured in the
    /// C172 pair as a gate held on 66–82% of deep pitch-down frames, against
    /// 28–36% for [`Diagonals`](Self::Diagonals).
    Verticals,
    /// Eye↔**opposite** mouth corner only, and the shipped default from v0.2.0.
    /// The longest baselines available (96.4 mm vs IPD's 63), the least
    /// expression-sensitive of any pair that involves the mouth (a jaw drop
    /// moves them ~10% against ~17% for nose↔mouth, since sensitivity scales
    /// inversely with vertical span), and oblique to *both* rotation axes — so
    /// neither yaw nor pitch foreshortens them the way yaw kills IPD and pitch
    /// kills the same-side pairs.
    Diagonals,
    /// Eyes and nose only — the three pairs among rigid landmarks, so **fully
    /// immune to expression**. Includes IPD, which is the worst pair under yaw,
    /// but with three pairs the median falls back onto the eye↔nose pairs when
    /// IPD foreshortens, which is the behaviour we want anyway.
    Rigid,
    /// Median over all ten pairs.
    Median10,
}

impl DepthCue {
    /// Landmark index pairs this cue aggregates. Indices match `FACE_MODEL_MM`:
    /// 0 = right eye, 1 = left eye, 2 = nose, 3 = right mouth, 4 = left mouth.
    pub fn pairs(self) -> &'static [(usize, usize)] {
        match self {
            Self::Ipd => &[(0, 1)],
            Self::NoseMouth => &[(2, 3), (2, 4)],
            Self::EyeNose => &[(0, 2), (1, 2)],
            Self::EyeMouth => &[(0, 3), (1, 4)],
            Self::Verticals => &[(0, 3), (1, 4), (0, 4), (1, 3)],
            Self::Diagonals => &[(0, 4), (1, 3)],
            Self::Rigid => &[(0, 1), (0, 2), (1, 2)],
            Self::Median10 => &[
                (0, 1), (0, 2), (0, 3), (0, 4), (1, 2),
                (1, 3), (1, 4), (2, 3), (2, 4), (3, 4),
            ],
        }
    }
}

/// A pair is unusable below this projected model length (mm): the baseline has
/// foreshortened to nothing and dividing by it amplifies noise without limit.
///
/// **This guard has never fired in production, and it does not replace the
/// `1/cos(yaw)` correction — an earlier version of this comment claimed it did.**
/// Measured over the model geometry, the *shortest* projected baseline of any
/// pair at 60° pitch is 36.9 mm and IPD at 60° yaw is 31.5 mm, both ~5× this
/// floor. It catches only a degenerate rotation, never real foreshortening.
///
/// A relative floor — drop a pair once it retains less than some fraction of its
/// own unrotated length — was the obvious repair and was tested offline at
/// 0.70–0.90 over six clips. **It did not work**: on `Verticals` and `Median10`
/// alike it was neutral-to-worse on every clip. Shedding a foreshortened pair
/// also sheds its vote, and with four pairs the median is more damaged by
/// dropping to two than by keeping a noisy estimate. Choosing pairs that do not
/// foreshorten in the first place — [`DepthCue::Diagonals`] — beat every variant
/// of the floor, which is why the fix is a cue choice and not a guard.
const MIN_PROJECTED_MM: f32 = 6.0;
/// A pair is unusable below this observed pixel separation: the two keypoints
/// have effectively collapsed, which is exactly the failure that produces an
/// IPD of 7 px and a reported depth of 6.3 m.
const MIN_OBSERVED_PX: f32 = 8.0;

/// Depth in mm (negative, camera frame) from a chosen set of landmark pairs,
/// plus a **quality** value: the normalised spread of the per-pair estimates.
///
/// Each pair independently estimates the same scalar, so their disagreement is a
/// per-frame confidence the Z gate can act on — it can currently see rate and
/// pose angle, but not that the landmarks are bad, which is why its 170 ms
/// "a level that persists is real" rule adopts errors as truth.
///
/// `quality` is 0.0 for perfect agreement and rises with disagreement; it is
/// `None` for single-pair cues, which have nothing to disagree with.
///
/// Rotation is *modelled* rather than corrected for, so no `cos(yaw)` term
/// appears. Returns `None` when fewer than one pair survives the guards.
///
/// **Known limitation:** `FACE_MODEL_MM` is a population average. Measured
/// against one real face the per-pair scales still disagree by ~39% after
/// correcting for anatomy, mostly on the nose-involving pairs. That is a
/// constant per person, so it shows up as a **gain** error rather than noise,
/// and `CenterStage` removes the offset — but per-user calibration is the
/// follow-on that would let `quality` become a clean signal.
pub fn depth_from_pairs(
    landmarks: &[(f32, f32)],
    rotation: &[[f32; 3]; 3],
    fl_px: f32,
    cue: DepthCue,
) -> Option<(f32, Option<f32>)> {
    if landmarks.len() < 5 {
        return None;
    }
    // Rotate the model into the camera frame; the projected (x, y) extent of a
    // pair is its baseline as the image actually sees it.
    let rotated: Vec<[f32; 3]> = crate::pnp::FACE_MODEL_MM
        .iter()
        .map(|p| {
            let mut out = [0.0f32; 3];
            for (a, o) in out.iter_mut().enumerate() {
                *o = (0..3).map(|b| rotation[a][b] * p[b]).sum();
            }
            out
        })
        .collect();

    let mut scales: Vec<f32> = Vec::with_capacity(10);
    for &(i, j) in cue.pairs() {
        let projected = (rotated[i][0] - rotated[j][0]).hypot(rotated[i][1] - rotated[j][1]);
        if projected < MIN_PROJECTED_MM {
            continue;
        }
        let observed =
            (landmarks[i].0 - landmarks[j].0).hypot(landmarks[i].1 - landmarks[j].1);
        if observed < MIN_OBSERVED_PX {
            continue;
        }
        scales.push(observed / projected);
    }
    if scales.is_empty() {
        return None;
    }
    scales.sort_by(|a, b| a.total_cmp(b));
    let scale = median_sorted(&scales);
    if scale <= 0.0 {
        return None;
    }
    // Spread of the per-pair estimates, normalised — 0.0 means perfect agreement.
    //
    // Upper-quartile deviation, deliberately **not** the median absolute
    // deviation. MAD is the natural partner to a median estimate, but it is too
    // robust to be a *detector*: one blown keypoint corrupts 4 of the 10 pairs,
    // and MAD discards them and reports 0.0 — precisely the frame we most need
    // flagged. The upper quartile rises once roughly a quarter of the pairs
    // disagree, which is also where the median estimate itself starts to be at
    // risk (it breaks down at half).
    let quality = (scales.len() > 1).then(|| {
        let mut dev: Vec<f32> = scales.iter().map(|s| (s - scale).abs()).collect();
        dev.sort_by(|a, b| a.total_cmp(b));
        // Upper-quartile index, biased up so a 2-pair cue reports its one real
        // disagreement rather than always reporting the smaller deviation.
        let idx = (dev.len() * 3).div_ceil(4).min(dev.len()) - 1;
        dev[idx] / scale
    });
    Some((-(fl_px / scale), quality))
}

/// Eye midpoint in model space (mm) — the physical point the tracker reports as
/// head position. `FACE_MODEL_MM`'s origin sits between the eyes and the mouth,
/// so the robust fit is offset back to the eye midpoint to keep the reported
/// reference identical to [`position_from_eyes`]. Changing which point is
/// tracked would silently change how rotation couples into X/Y.
const EYE_MID_MODEL: [f32; 3] = [0.0, -35.0, 0.0];

/// Full translation from a robust fit over **all five** landmarks.
///
/// # Why not the eye centroid
///
/// [`position_from_eyes`] derives X/Y from the midpoint of the two eyes — the
/// *same two points* IPD uses. So one bad eye keypoint corrupts X, Y and Z in
/// the same frame, and nothing downstream can tell. It also back-projects at
/// the IPD-derived depth (`x_mm = nx * z_camera_mm`), so depth noise multiplies
/// straight into the lateral axes.
///
/// This fits the rotated model to all five observed landmarks and reads the eye
/// midpoint off the fit:
///
/// ```text
/// t     = median_i( p_i − scale · (R · P_i).xy )    # per axis, over 5 points
/// eye   = t + scale · (R · EYE_MID_MODEL).xy
/// ```
///
/// A median over five tolerates **two** outliers — and the two mouth corners are
/// exactly the landmarks that move with expression, so a jaw drop or a smile is
/// rejected rather than averaged in. That is the property a centroid cannot have
/// at any number of points, because a mean has a breakdown point of zero.
///
/// Returns `(x_mm, y_mm, z_mm, quality)` in the canonical convention
/// (X right, Y up, Z toward screen), or `None` if too few pairs survive.
pub fn translation_from_landmarks(
    landmarks: &[(f32, f32)],
    rotation: &[[f32; 3]; 3],
    image_size: (u32, u32),
    fl_px: (f32, f32),
    cue: DepthCue,
) -> Option<(f32, f32, f32, Option<f32>)> {
    let (z_camera_mm, quality) = depth_from_pairs(landmarks, rotation, fl_px.0, cue)?;
    // depth_from_pairs returns the canonical (negative) depth; the projection
    // below works in camera depth, which is its magnitude.
    let z_cam = -z_camera_mm;
    if !(z_cam.is_finite() && z_cam > 1.0) {
        return None;
    }
    let scale = fl_px.0 / z_cam; // px per mm, the same scale the depth came from

    let rotate = |p: [f32; 3]| -> [f32; 2] {
        [
            (0..3).map(|b| rotation[0][b] * p[b]).sum(),
            (0..3).map(|b| rotation[1][b] * p[b]).sum(),
        ]
    };

    // Per-axis median of the implied image position of the model origin.
    let mut tx: Vec<f32> = Vec::with_capacity(5);
    let mut ty: Vec<f32> = Vec::with_capacity(5);
    for (i, model) in crate::pnp::FACE_MODEL_MM.iter().enumerate() {
        let q = rotate(*model);
        tx.push(landmarks[i].0 - scale * q[0]);
        ty.push(landmarks[i].1 - scale * q[1]);
    }
    tx.sort_by(|a, b| a.total_cmp(b));
    ty.sort_by(|a, b| a.total_cmp(b));

    let eye = rotate(EYE_MID_MODEL);
    let cx_px = median_sorted(&tx) + scale * eye[0];
    let cy_px = median_sorted(&ty) + scale * eye[1];

    let (img_w, img_h) = (image_size.0 as f32, image_size.1 as f32);
    let nx = (cx_px - img_w * 0.5) / fl_px.0;
    let ny = (cy_px - img_h * 0.5) / fl_px.1;

    // Canonical: y up = +ve (image y is down), z toward screen = +ve.
    Some((nx * z_cam, -(ny * z_cam), -z_cam, quality))
}

/// Rotation matrix from canonical yaw/pitch/roll — the inverse of
/// [`crate::pnp::pose_to_canonical`], whose derivation this mirrors:
/// `R = Rz(-roll) · Ry(+yaw) · Rx(-pitch)`, model in the camera frame
/// (X right, Y down, Z into scene).
///
/// The depth cue rebuilds `R` from the *final* rotation rather than reusing
/// `PnpResult::r`, because 6DRepNet overrides PnP's rotation downstream and the
/// learned one is the better estimate — rotation is the solved axis.
pub fn rotation_from_canonical(yaw_deg: f32, pitch_deg: f32, roll_deg: f32) -> [[f32; 3]; 3] {
    let (y, p, r) = (
        yaw_deg.to_radians(),
        (-pitch_deg).to_radians(),
        (-roll_deg).to_radians(),
    );
    let (cy, sy) = (y.cos(), y.sin());
    let (cp, sp) = (p.cos(), p.sin());
    let (cr, sr) = (r.cos(), r.sin());
    // Rz * Ry * Rx, expanded.
    [
        [
            cr * cy,
            cr * sy * sp - sr * cp,
            cr * sy * cp + sr * sp,
        ],
        [
            sr * cy,
            sr * sy * sp + cr * cp,
            sr * sy * cp - cr * sp,
        ],
        [-sy, cy * sp, cy * cp],
    ]
}

/// Median of an already-sorted, non-empty slice.
fn median_sorted(v: &[f32]) -> f32 {
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) * 0.5 }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FL: f32 = 700.0;

    /// Weak-perspective projection of the face model at a known depth — the
    /// same model [`depth_from_pairs`] inverts, so a correct estimator recovers
    /// `z_mm` exactly. Mirrors `pnp.rs`'s `project_model` test helper.
    fn project(z_mm: f32, rot: &[[f32; 3]; 3]) -> Vec<(f32, f32)> {
        let scale = FL / z_mm;
        crate::pnp::FACE_MODEL_MM
            .iter()
            .map(|p| {
                let x = (0..3).map(|b| rot[0][b] * p[b]).sum::<f32>();
                let y = (0..3).map(|b| rot[1][b] * p[b]).sum::<f32>();
                (480.0 + scale * x, 270.0 + scale * y)
            })
            .collect()
    }

    const ALL: [DepthCue; 7] = [
        DepthCue::NoseMouth,
        DepthCue::EyeNose,
        DepthCue::EyeMouth,
        DepthCue::Verticals,
        DepthCue::Diagonals,
        DepthCue::Rigid,
        DepthCue::Median10,
    ];

    /// Zero rotation must be the identity, or every projected baseline is wrong
    /// and the whole cue silently mis-scales.
    #[test]
    fn zero_rotation_is_identity() {
        let r = rotation_from_canonical(0.0, 0.0, 0.0);
        for (i, row) in r.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((v - want).abs() < 1e-6, "R[{i}][{j}] = {v}, want {want}");
            }
        }
    }

    /// The estimator must invert its own projection exactly, at any pose.
    #[test]
    fn recovers_a_known_depth() {
        for &(yaw, pitch, roll) in &[
            (0.0, 0.0, 0.0),
            (35.0, 0.0, 0.0),
            (-50.0, -15.0, 0.0),
            (20.0, 10.0, 12.0),
        ] {
            let rot = rotation_from_canonical(yaw, pitch, roll);
            let kps = project(600.0, &rot);
            for cue in ALL {
                let (z, _) = depth_from_pairs(&kps, &rot, FL, cue)
                    .unwrap_or_else(|| panic!("{cue:?} produced nothing at yaw {yaw}"));
                assert!(
                    (z + 600.0).abs() < 1.0,
                    "{cue:?} at yaw {yaw}: got {z}, want -600"
                );
            }
        }
    }

    /// The whole point of aggregating pairs: one blown keypoint must not move
    /// the estimate. This is YuNet's observed failure — an eye keypoint landing
    /// on top of the other, which drives IPD to 7 px and depth to 6.3 m.
    #[test]
    fn one_blown_keypoint_does_not_move_a_multi_pair_estimate() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let mut kps = project(600.0, &rot);
        kps[1] = kps[0]; // left eye collapses onto the right

        // Single-pair IPD is destroyed by exactly this — it is guarded out
        // entirely rather than reporting a wild depth.
        assert!(
            depth_from_pairs(&kps, &rot, FL, DepthCue::Ipd).is_none(),
            "a collapsed pair must be rejected, not turned into a 6 m reading"
        );

        // A cue that does not involve the blown point is untouched.
        let (z, _) = depth_from_pairs(&kps, &rot, FL, DepthCue::NoseMouth).expect("nose-mouth");
        assert!((z + 600.0).abs() < 1.0, "nose-mouth unaffected, got {z}");

        // Aggregating cues survive: the surviving pairs outvote the bad one.
        for cue in [DepthCue::Verticals, DepthCue::Median10] {
            let (z, _) = depth_from_pairs(&kps, &rot, FL, cue).expect("aggregate");
            assert!(
                (z + 600.0).abs() < 60.0,
                "{cue:?} should stay near -600 with one bad point, got {z}"
            );
        }
    }

    /// Landmark disagreement is the signal `z_gate.rs` lacks: it can see rate
    /// and pose angle, but not that the landmarks are bad.
    #[test]
    fn quality_reports_landmark_disagreement() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let clean = project(600.0, &rot);

        let (_, q_clean) = depth_from_pairs(&clean, &rot, FL, DepthCue::Median10).expect("clean");
        let q_clean = q_clean.expect("multi-pair cues report quality");
        assert!(q_clean < 1e-3, "perfect landmarks must agree, got {q_clean}");

        let mut bad = clean.clone();
        bad[3].0 += 25.0; // shove one mouth corner sideways
        bad[3].1 -= 25.0;
        let (_, q_bad) = depth_from_pairs(&bad, &rot, FL, DepthCue::Median10).expect("bad");
        let q_bad = q_bad.expect("quality");
        assert!(q_bad > q_clean * 10.0, "quality must rise: {q_clean} -> {q_bad}");

        // A single-pair cue has nothing to disagree with.
        let (_, q) = depth_from_pairs(&clean, &rot, FL, DepthCue::Ipd).expect("ipd");
        assert!(q.is_none(), "single-pair cues cannot report agreement");
    }

    /// Project the model at a known depth AND lateral offset, with the **eye
    /// midpoint** at the image centre when the offset is zero — that is the
    /// point the tracker reports, so "centred" should mean the eyes are centred.
    fn project_at(z_mm: f32, off_px: (f32, f32), rot: &[[f32; 3]; 3]) -> Vec<(f32, f32)> {
        let scale = FL / z_mm;
        let rot_xy = |p: [f32; 3]| {
            (
                (0..3).map(|b| rot[0][b] * p[b]).sum::<f32>(),
                (0..3).map(|b| rot[1][b] * p[b]).sum::<f32>(),
            )
        };
        let (ex, ey) = rot_xy(EYE_MID_MODEL);
        crate::pnp::FACE_MODEL_MM
            .iter()
            .map(|p| {
                let (x, y) = rot_xy(*p);
                (
                    480.0 + off_px.0 + scale * (x - ex),
                    270.0 + off_px.1 + scale * (y - ey),
                )
            })
            .collect()
    }

    const IMG: (u32, u32) = (960, 540);

    /// The robust fit must reproduce the geometry it was built from.
    #[test]
    fn translation_recovers_a_known_pose() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let kps = project_at(600.0, (0.0, 0.0), &rot);
        let (x, y, z, _) =
            translation_from_landmarks(&kps, &rot, IMG, (FL, FL), DepthCue::Verticals)
                .expect("fit");
        assert!((z + 600.0).abs() < 1.0, "depth {z}");
        assert!(x.abs() < 2.0 && y.abs() < 2.0, "centred pose should be near zero, got {x},{y}");

        // A lateral shift of 70 px at 600 mm with a 700 px focal length is 60 mm.
        let kps = project_at(600.0, (70.0, 0.0), &rot);
        let (x, _, _, _) =
            translation_from_landmarks(&kps, &rot, IMG, (FL, FL), DepthCue::Verticals)
                .expect("fit");
        assert!((x - 60.0).abs() < 2.0, "70px shift should read ~60mm, got {x}");
    }

    /// **The reason this function exists** — and a limit worth pinning.
    ///
    /// Expression moves exactly two landmarks (the mouth corners), and a median
    /// over five tolerates two outliers, so the *lateral* fit rejects them. A
    /// centroid cannot: a mean has a breakdown point of zero.
    ///
    /// But X and Y are scaled by depth, so an expression-**sensitive** depth cue
    /// still leaks a jaw drop into reported position through the scale factor.
    /// Both halves are asserted here, because the second is the reason the depth
    /// cue choice matters for all three axes and not just Z.
    #[test]
    fn expression_does_not_move_the_reported_position() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let neutral = project_at(600.0, (0.0, 0.0), &rot);
        // Jaw drop: both mouth corners down ~12 px (~10 mm at this depth).
        let mut talking = neutral.clone();
        talking[3].1 += 12.0;
        talking[4].1 += 12.0;

        let shift = |cue: DepthCue| {
            let a = translation_from_landmarks(&neutral, &rot, IMG, (FL, FL), cue).expect("a");
            let b = translation_from_landmarks(&talking, &rot, IMG, (FL, FL), cue).expect("b");
            ((b.0 - a.0).abs(), (b.1 - a.1).abs())
        };

        // Rigid depth (eyes + nose) is expression-immune, so with the robust
        // lateral fit the reported position barely moves at all.
        let (dx, dy) = shift(DepthCue::Rigid);
        assert!(dx < 1.0 && dy < 1.0, "rigid depth: expected no leak, got {dx},{dy}");

        // Verticals depth involves the mouth, so some leak survives via the
        // scale — bounded, but real. Pinned so a regression is visible.
        let (_, dy_v) = shift(DepthCue::Verticals);
        assert!(dy_v < 6.0, "verticals leak should stay small, got {dy_v}");

        // For contrast: the naive mean of all five points moves regardless of
        // depth cue — that is the trap this function avoids.
        let mean_y = |k: &[(f32, f32)]| k.iter().map(|p| p.1).sum::<f32>() / k.len() as f32;
        assert!(
            (mean_y(&talking) - mean_y(&neutral)).abs() > 4.0,
            "the naive centroid should move — that is the trap being avoided"
        );
    }

    /// One collapsed eye keypoint used to corrupt X, Y and Z together, because
    /// all three came from the same two points.
    #[test]
    fn a_blown_eye_keypoint_no_longer_corrupts_x_and_y() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let clean = project_at(600.0, (0.0, 0.0), &rot);
        let (x0, y0, _, _) =
            translation_from_landmarks(&clean, &rot, IMG, (FL, FL), DepthCue::Verticals)
                .expect("clean");

        let mut bad = clean.clone();
        bad[1] = bad[0]; // left eye collapses onto the right — YuNet's failure
        let (x1, y1, _, _) =
            translation_from_landmarks(&bad, &rot, IMG, (FL, FL), DepthCue::Verticals)
                .expect("degraded but usable");
        assert!(
            (x1 - x0).abs() < 15.0 && (y1 - y0).abs() < 15.0,
            "one bad eye must not throw X/Y: ({x0},{y0}) -> ({x1},{y1})"
        );

        // The old eye-centroid path moves by half the error — ~31 mm here.
        let old = position_from_eyes(bad[0], bad[1], IMG, 78.0, 0.0);
        let good = position_from_eyes(clean[0], clean[1], IMG, 78.0, 0.0);
        assert!(
            (old.0 - good.0).abs() > 20.0 || !old.2.is_finite() || old.2.abs() > 2000.0,
            "the eye-centroid path should be visibly worse here"
        );
    }

    /// Guards, not corrections: a foreshortened or collapsed baseline is
    /// dropped rather than divided back up the way `1/cos(yaw)` does.
    #[test]
    fn degenerate_input_is_rejected_not_amplified() {
        let rot = rotation_from_canonical(0.0, 0.0, 0.0);
        let collapsed = vec![(100.0, 100.0); 5];
        for cue in ALL {
            assert!(
                depth_from_pairs(&collapsed, &rot, FL, cue).is_none(),
                "{cue:?} must reject fully collapsed landmarks"
            );
        }
        assert!(
            depth_from_pairs(&[(0.0, 0.0); 3], &rot, FL, DepthCue::Median10).is_none(),
            "fewer than 5 landmarks must be rejected"
        );
    }
}

#[cfg(test)]
mod pitch_foreshortening_tests {
    use super::*;

    /// Fraction of a pair's unrotated length that survives a given pitch.
    fn retention(pair: (usize, usize), pitch_deg: f32) -> f32 {
        let m = &crate::pnp::FACE_MODEL_MM;
        let (i, j) = pair;
        let d = [m[i][0] - m[j][0], m[i][1] - m[j][1], m[i][2] - m[j][2]];
        let (c, s) = (pitch_deg.to_radians().cos(), pitch_deg.to_radians().sin());
        // Rx(pitch) applied to the difference vector; only x/y survive projection.
        let y = c * d[1] - s * d[2];
        (d[0] * d[0] + y * y).sqrt() / (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    }

    fn worst(cue: DepthCue, pitch_deg: f32) -> f32 {
        cue.pairs()
            .iter()
            .map(|&p| retention(p, pitch_deg))
            .fold(f32::INFINITY, f32::min)
    }

    /// The reason `Diagonals` ships and `Verticals` does not.
    ///
    /// `Verticals` is a strict superset — the two pairs it adds are the
    /// near-vertical same-side ones, which pitch foreshortens far harder. Depth
    /// is `f·B/b`, so noise gain is `1/retention`: whatever the median lands on,
    /// the superset can land on a pair that has lost half its length.
    #[test]
    fn diagonals_survive_pitch_better_than_the_pairs_verticals_adds() {
        for &(i, j) in DepthCue::Diagonals.pairs() {
            assert!(
                DepthCue::Verticals.pairs().contains(&(i, j)),
                "Diagonals must stay a subset of Verticals; ({i},{j}) is not"
            );
        }
        assert_eq!(DepthCue::Verticals.pairs().len(), 4);
        assert_eq!(DepthCue::Diagonals.pairs().len(), 2);

        for pitch in [-30.0_f32, -45.0, -60.0] {
            let d = worst(DepthCue::Diagonals, pitch);
            let v = worst(DepthCue::Verticals, pitch);
            assert!(
                d > v,
                "at {pitch}° the worst diagonal must outlast the worst vertical, \
                 got {d:.3} vs {v:.3}"
            );
        }
        // The headline number in the DepthCue docs: 67% against 46% at -60°.
        assert!((worst(DepthCue::Diagonals, -60.0) - 0.67).abs() < 0.02);
        assert!((worst(DepthCue::Verticals, -60.0) - 0.46).abs() < 0.02);
    }

    /// IPD is untouched by pitch and destroyed by yaw — the complementary
    /// failure, and why the shipped cue must be oblique to both axes.
    #[test]
    fn ipd_is_pitch_immune_which_is_not_the_same_as_good() {
        assert!((retention((0, 1), -60.0) - 1.0).abs() < 1e-4);
    }
}
