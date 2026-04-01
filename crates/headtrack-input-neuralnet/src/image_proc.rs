use image::{GrayImage, ImageBuffer, Luma, Rgb};

/// Convert an RGB frame to grayscale.
pub fn to_gray(rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> GrayImage {
    let (w, h) = rgb.dimensions();
    let src = rgb.as_raw();
    let len = (w * h) as usize;
    let mut out = Vec::with_capacity(len);

    // BT.601 fixed-point: 77/256 ≈ 0.301, 150/256 ≈ 0.586, 29/256 ≈ 0.113
    let chunks = src.chunks_exact(12); // 4 pixels × 3 bytes
    let remainder = chunks.remainder();

    for chunk in chunks {
        out.push(((77 * chunk[0] as u32 + 150 * chunk[1] as u32 + 29 * chunk[2] as u32) >> 8) as u8);
        out.push(((77 * chunk[3] as u32 + 150 * chunk[4] as u32 + 29 * chunk[5] as u32) >> 8) as u8);
        out.push(((77 * chunk[6] as u32 + 150 * chunk[7] as u32 + 29 * chunk[8] as u32) >> 8) as u8);
        out.push(((77 * chunk[9] as u32 + 150 * chunk[10] as u32 + 29 * chunk[11] as u32) >> 8) as u8);
    }

    for px in remainder.chunks_exact(3) {
        out.push(((77 * px[0] as u32 + 150 * px[1] as u32 + 29 * px[2] as u32) >> 8) as u8);
    }

    GrayImage::from_vec(w, h, out).unwrap()
}

/// Resize a grayscale image to `(width, height)` using bilinear interpolation.
pub fn resize_gray(img: &GrayImage, width: u32, height: u32) -> GrayImage {
    image::imageops::resize(img, width, height, image::imageops::FilterType::Triangle)
}

/// Normalise a grayscale image to f32 in `[-0.5, 0.5]` by
/// dividing by 255 and subtracting 0.5.
///
/// Used for the **localizer** model (simple linear normalisation).
pub fn normalize_linear(img: &GrayImage) -> Vec<f32> {
    img.pixels()
        .map(|Luma([v])| *v as f32 / 255.0 - 0.5)
        .collect()
}

/// Normalise a grayscale image to f32 in `[-0.5, ~0.5]` using the
/// **90th-percentile brightness** method from opentrack.
///
/// Scales so that the 90th percentile intensity maps to 0.5, then shifts
/// by −0.5.  This is robust to varying lighting conditions.
///
/// Used for the **pose estimator** model.
pub fn normalize_brightness(img: &GrayImage) -> Vec<f32> {
    // Compute histogram.
    let mut hist = [0u32; 256];
    for Luma([v]) in img.pixels() {
        hist[*v as usize] += 1;
    }

    // Find 90th-percentile intensity.
    let threshold = (img.width() * img.height()) as f32 * 0.9;
    let mut accum = 0u32;
    let mut brightness = 127u8;
    for (i, &count) in hist.iter().enumerate() {
        accum += count;
        if accum as f32 > threshold {
            brightness = i as u8;
            break;
        }
    }

    // Match opentrack: scale so pct-brightness maps to 0.5, else 1/255.
    let alpha: f32 = if brightness < 127 {
        0.9 * 0.5 / (brightness.max(5) as f32)
    } else {
        1.0 / 255.0
    };

    img.pixels()
        .map(|Luma([v])| alpha * *v as f32 - 0.5)
        .collect()
}

/// Crop a grayscale frame to a square patch centred on `centre`,
/// with side length `patch_size`, clamped to image bounds.
///
/// Returns `None` if the patch centre falls outside the image.
pub fn extract_square_patch(
    img: &GrayImage,
    centre: (f32, f32),
    patch_size: u32,
) -> Option<GrayImage> {
    let (cx, cy) = centre;
    let half = patch_size as f32 * 0.5;

    let x0 = (cx - half).round() as i64;
    let y0 = (cy - half).round() as i64;

    if x0 < 0
        || y0 < 0
        || x0 as u32 + patch_size > img.width()
        || y0 as u32 + patch_size > img.height()
    {
        // Patch extends outside the frame — skip this frame rather than pad.
        return None;
    }

    let cropped = image::imageops::crop_imm(img, x0 as u32, y0 as u32, patch_size, patch_size)
        .to_image();
    Some(cropped)
}

/// Unnormalise a bounding-box coordinate from `[-1, 1]` to pixel space.
///
/// `axis_size` is the width (for x) or height (for y) of the image.
pub fn unnorm(v: f32, axis_size: u32) -> f32 {
    0.5 * (v + 1.0) * axis_size as f32
}
