/// Quaternion → (yaw, pitch, roll) in degrees.
///
/// Quaternion convention: `[x, y, z, w]` as output by the pose model.
///
/// Model coordinate frame (image/camera space):
///   X = right, Y = down, Z = forward (into screen, away from user).
///
/// Canonical headtrack-rs output convention:
///   yaw   positive = turn right
///   pitch positive = tilt up (nose toward ceiling)
///   roll  positive = clockwise tilt (right ear toward right shoulder)
///
/// ## Derivation
///
/// Build the rotation matrix column-by-column from the quaternion.
/// mx/my/mz are where the unit axes end up after the head rotation.
///
/// In image space (Y-down):
///
/// - **Yaw** (rotation around world-up = -Y_image):
///   `yaw = atan2(mx[2], mx[0])` gives positive for right-turn. ✓
///
/// - **Pitch** (rotation around X_image = right):
///   `pitch = atan2(mz[1], mz[2])` — how far the forward axis tilts into Y.
///   Y-down means `mz[1] > 0` is "nose down"; negate for canonical up = +.
///   After sign analysis: `pitch = atan2(-mz[1], mz[2])` ... but the sign
///   actually works out correctly without extra negation given the `mz`
///   orientation when nose moves toward -Y (up in world).
///   Empirically verified: `-RAD2DEG * roll_old` = correct canonical pitch.
///
/// - **Roll** (rotation around Z_image = forward into screen):
///   Similarly: `-RAD2DEG * pitch_old` = correct canonical roll.
///
/// The original code had pitch and roll swapped (pitch formula captured roll
/// motion; roll formula captured pitch motion).  Fixed by exchanging them
/// and adjusting signs.
pub fn quat_to_euler_deg(x: f32, y: f32, z: f32, w: f32) -> (f32, f32, f32) {
    // Rotation matrix, column-major.
    // mx = R * [1, 0, 0]  (where "right" ends up after rotation)
    // my = R * [0, 1, 0]  (where "down" ends up after rotation)
    // mz = R * [0, 0, 1]  (where "forward/into-screen" ends up after rotation)
    let mx = [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ];
    let my = [
        2.0 * (x * y - w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + w * x),
    ];
    let mz = [
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ];

    // Empirically derived from guided axis test (2026-03-15):
    //
    //   Formula variable  | Responds to      | Sign for positive canonical motion
    //   A = mx[2].atan2(mx[0])                 physical pitch   negative (needs flip)
    //   B = -((-mx[1]).atan2(xz_len))          physical roll    negative (needs flip)
    //   C = (-mz[1]).atan2(my[1])              physical yaw     negative (needs flip)
    //
    // Correct canonical output: Yaw=-A, Pitch=-C, Roll=-B
    let xz_len = (mx[2] * mx[2] + mx[0] * mx[0]).sqrt();

    let a = mx[2].atan2(mx[0]);
    let b = -((-mx[1]).atan2(xz_len));
    let c = (-mz[1]).atan2(my[1]);

    const RAD2DEG: f32 = 180.0 / std::f32::consts::PI;
    (-RAD2DEG * a, -RAD2DEG * c, -RAD2DEG * b)
}

/// Estimate 3D head position in mm (canonical space: x=right, y=up, z=forward).
///
/// Uses a pinhole camera model.  `face_center` is in pixel coordinates,
/// `face_size_px` is the detected head diameter in pixels,
/// `image_size` is `(width, height)`.
///
/// `fov_diag_deg` is the camera's diagonal field of view — default to 78°
/// (Logitech C920) if unknown.
pub fn estimate_position_mm(
    face_center: (f32, f32),
    face_size_px: f32,
    image_size: (u32, u32),
    fov_diag_deg: f32,
) -> (f32, f32, f32) {
    const HEAD_SIZE_MM: f32 = 200.0; // reference head diameter

    let (img_w, img_h) = (image_size.0 as f32, image_size.1 as f32);

    // Derive per-axis focal lengths from diagonal FOV.
    let fov_diag = fov_diag_deg.to_radians();
    let aspect = img_h / img_w;
    let fov_w = 2.0 * ((fov_diag / 2.0).tan() / (1.0 + aspect * aspect).sqrt()).atan();
    let fov_h = 2.0 * ((fov_diag / 2.0).tan() / (1.0 + 1.0 / (aspect * aspect)).sqrt()).atan();
    let fl_w = 1.0 / (fov_w / 2.0).tan(); // focal length in normalised clip units
    let fl_h = 1.0 / (fov_h / 2.0).tan();

    // Normalised face centre: [-0.5, 0.5] from image centre.
    let nx = (face_center.0 / img_w) - 0.5;
    let ny = (face_center.1 / img_h) - 0.5;

    // Depth from head size: Z = focal_length_w * HEAD_SIZE_MM / face_size_px
    // face_size_px is in pixels, convert to fraction of image width for focal length in clip units.
    let face_size_norm = face_size_px / img_w;
    let z_mm = fl_w * HEAD_SIZE_MM / face_size_norm;

    // Back-project to 3D.  Image y is down; canonical y is up.
    // Negate z: z_mm here is depth-from-camera (always +), but canonical z = toward screen = +
    // so leaning toward camera should give +z, meaning we invert depth.
    let x_mm = (nx / fl_w) * z_mm;
    let y_mm = -(ny / fl_h) * z_mm; // flip sign: image down → canonical up
    (x_mm, y_mm, -z_mm)
}

/// Sigmoid: 1 / (1 + e^-x).
pub fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_quaternion_gives_zero_angles() {
        let (yaw, pitch, roll) = quat_to_euler_deg(0.0, 0.0, 0.0, 1.0);
        assert!(yaw.abs() < 1e-4, "yaw {yaw}");
        assert!(pitch.abs() < 1e-4, "pitch {pitch}");
        assert!(roll.abs() < 1e-4, "roll {roll}");
    }
}
