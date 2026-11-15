use image::{GrayImage, ImageBuffer, Rgb};

/// Convert an RGB frame to grayscale.
pub fn to_gray(rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> GrayImage {
    let (w, h) = rgb.dimensions();
    let src = rgb.as_raw();
    let len = (w * h) as usize;
    let mut out = Vec::with_capacity(len);

    // BT.601 fixed-point: 77/256 ≈ 0.301, 150/256 ≈ 0.586, 29/256 ≈ 0.113
    let (chunks, remainder) = src.as_chunks::<12>(); // 4 pixels × 3 bytes

    for chunk in chunks {
        out.push(
            ((77 * chunk[0] as u32 + 150 * chunk[1] as u32 + 29 * chunk[2] as u32) >> 8) as u8,
        );
        out.push(
            ((77 * chunk[3] as u32 + 150 * chunk[4] as u32 + 29 * chunk[5] as u32) >> 8) as u8,
        );
        out.push(
            ((77 * chunk[6] as u32 + 150 * chunk[7] as u32 + 29 * chunk[8] as u32) >> 8) as u8,
        );
        out.push(
            ((77 * chunk[9] as u32 + 150 * chunk[10] as u32 + 29 * chunk[11] as u32) >> 8) as u8,
        );
    }

    for px in remainder.as_chunks::<3>().0 {
        out.push(((77 * px[0] as u32 + 150 * px[1] as u32 + 29 * px[2] as u32) >> 8) as u8);
    }

    GrayImage::from_vec(w, h, out).unwrap()
}

// ── RGB helpers (used by the 6DRepNet-style head pose model) ─────────────────

/// Crop a colour frame to a square patch centred on `centre`, with side length
/// `patch_size`, clamped to image bounds. Returns `None` if the patch extends
/// outside the frame.
pub fn extract_square_patch_rgb(
    img: &ImageBuffer<Rgb<u8>, Vec<u8>>,
    centre: (f32, f32),
    patch_size: u32,
) -> Option<ImageBuffer<Rgb<u8>, Vec<u8>>> {
    let (cx, cy) = centre;
    let half = patch_size as f32 * 0.5;
    let x0 = (cx - half).round() as i64;
    let y0 = (cy - half).round() as i64;
    if x0 < 0
        || y0 < 0
        || x0 as u32 + patch_size > img.width()
        || y0 as u32 + patch_size > img.height()
    {
        return None;
    }
    let cropped =
        image::imageops::crop_imm(img, x0 as u32, y0 as u32, patch_size, patch_size).to_image();
    Some(cropped)
}

/// Resize a colour image to `(width, height)` using bilinear interpolation.
pub fn resize_rgb(
    img: &ImageBuffer<Rgb<u8>, Vec<u8>>,
    width: u32,
    height: u32,
) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    image::imageops::resize(img, width, height, image::imageops::FilterType::Triangle)
}

/// Convert a HWC RGB image to a CHW float buffer normalised with ImageNet
/// statistics (mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]).
///
/// Used by the 6DRepNet-style head pose model. Output is `3 * width * height`
/// f32 values in `R | G | B` channel-plane order.
pub fn imagenet_normalise_chw(img: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> Vec<f32> {
    const MEAN: [f32; 3] = [0.485, 0.456, 0.406];
    const STD: [f32; 3] = [0.229, 0.224, 0.225];
    let (w, h) = img.dimensions();
    let n = (w * h) as usize;
    let mut out = vec![0.0f32; 3 * n];
    let src = img.as_raw();
    for (i, px) in src.as_chunks::<3>().0.iter().enumerate() {
        out[i] = (px[0] as f32 / 255.0 - MEAN[0]) / STD[0];
        out[i + n] = (px[1] as f32 / 255.0 - MEAN[1]) / STD[1];
        out[i + 2 * n] = (px[2] as f32 / 255.0 - MEAN[2]) / STD[2];
    }
    out
}
