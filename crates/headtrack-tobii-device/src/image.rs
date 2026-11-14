use std::sync::Arc;
use thiserror::Error;
pub const MAX_SIDE: usize = 2048;
pub const MAX_PIXELS: usize = MAX_SIDE * MAX_SIDE;
#[derive(Clone, Debug)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Arc<[u8]>,
}
#[derive(Debug, Error)]
#[error("invalid grayscale dimensions, stride, format, or incomplete pixels")]
pub struct ImageError;
impl GrayImage {
    pub fn new(
        width: usize,
        height: usize,
        stride: usize,
        pixels: &[u8],
    ) -> Result<Self, ImageError> {
        let size = stride.checked_mul(height).ok_or(ImageError)?;
        if width == 0
            || height == 0
            || width > MAX_SIDE
            || height > MAX_SIDE
            || stride < width
            || stride > MAX_SIDE
            || size > MAX_PIXELS
            || pixels.len() != size
        {
            return Err(ImageError);
        }
        let mut packed = Vec::with_capacity(width * height);
        for row in pixels.chunks_exact(stride) {
            packed.extend_from_slice(&row[..width]);
        }
        Ok(Self {
            width,
            height,
            pixels: packed.into(),
        })
    }
    /// Strict P5/8-bit PGM reader. Header scan and raster are bounded. Whitespace
    /// pixel values are preserved: only the required final delimiter is consumed.
    pub fn pgm(data: &[u8]) -> Result<Self, ImageError> {
        if data.len() > MAX_PIXELS + 4096 {
            return Err(ImageError);
        }
        let mut p = 0;
        let mut token = || -> Result<&[u8], ImageError> {
            loop {
                while p < data.len() && data[p].is_ascii_whitespace() {
                    p += 1;
                }
                if data.get(p) == Some(&b'#') {
                    while p < data.len() && data[p] != b'\n' {
                        p += 1;
                    }
                } else {
                    break;
                }
                if p > 4096 {
                    return Err(ImageError);
                }
            }
            let start = p;
            while p < data.len() && !data[p].is_ascii_whitespace() {
                p += 1;
            }
            if p > 4096 || p == start {
                return Err(ImageError);
            }
            Ok(&data[start..p])
        };
        if token()? != b"P5" {
            return Err(ImageError);
        }
        fn number(b: &[u8]) -> Result<usize, ImageError> {
            std::str::from_utf8(b)
                .map_err(|_| ImageError)?
                .parse()
                .map_err(|_| ImageError)
        }
        let w = number(token()?)?;
        let h = number(token()?)?;
        if number(token()?)? != 255 {
            return Err(ImageError);
        }
        if !data.get(p).is_some_and(u8::is_ascii_whitespace) {
            return Err(ImageError);
        }
        p += 1;
        if data.get(p - 1) == Some(&b'\r') && data.get(p) == Some(&b'\n') {
            p += 1;
        }
        Self::new(w, h, w, &data[p..])
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn overflow_and_truncation_rejected() {
        for side in [0, 65536, usize::MAX] {
            assert!(GrayImage::new(side, side, side, &[]).is_err());
        }
        assert!(GrayImage::new(2, 2, 2, &[0; 3]).is_err());
        assert!(GrayImage::new(2, 2, 1, &[0; 2]).is_err());
    }
    #[test]
    fn preserves_whitespace_pixels_and_padding() {
        let img = GrayImage::pgm(b"P5\n# test\n2 1\n255\n\n ").unwrap();
        assert_eq!(&*img.pixels, b"\n ");
        let img = GrayImage::new(1, 2, 2, &[1, 99, 2, 99]).unwrap();
        assert_eq!(&*img.pixels, &[1, 2]);
    }
    #[test]
    fn rejects_every_truncated_raster() {
        let data = b"P5\n2 2\n255\n1234";
        for end in 0..data.len() {
            assert!(GrayImage::pgm(&data[..end]).is_err());
        }
    }
}
