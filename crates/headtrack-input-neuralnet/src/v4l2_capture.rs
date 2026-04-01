//! Raw V4L2 camera capture — fallback for cameras that crash nokhwa's format
//! enumeration (e.g. Delan Cam 1 reports fractional frame rates like 15/2).
//!
//! Supports two pixel formats:
//! - **MJPEG** — for cameras where JPEG decode is cheap (small frames, e.g. C920)
//! - **YUYV** — for cameras with huge MJPEG frames where decode dominates (DC1).
//!   The Y channel is the grayscale value, so `yuyv_to_gray` is a trivial copy
//!   of every other byte — zero computation.

use anyhow::{Context, Result};
use image::{GrayImage, ImageBuffer, Rgb};
use std::io::Cursor;
use std::mem::ManuallyDrop;
use tracing::{info, warn};
use v4l::buffer::Type;
use v4l::control::{Control, Value};
use v4l::io::mmap::Stream;
use v4l::io::traits::CaptureStream;
use v4l::prelude::*;
use v4l::video::Capture;
use v4l::{FourCC, Fraction};

// V4L2 camera control IDs (from linux/v4l2-controls.h).
// V4L2_CID_EXPOSURE_AUTO: menu — 0=auto, 1=manual, 2=shutter-priority, 3=aperture-priority
const V4L2_CID_EXPOSURE_AUTO: u32 = 0x009a_0901;
// V4L2_CID_EXPOSURE_ABSOLUTE: integer in units of 100 µs (156 → 15.6 ms, Delan Cam min)
const V4L2_CID_EXPOSURE_ABSOLUTE: u32 = 0x009a_0902;
// V4L2_CID_EXPOSURE_AUTO_PRIORITY: bool — when 1, driver may drop fps to accommodate exposure.
// Must be 0 to honour the requested frame rate in manual exposure mode.
const V4L2_CID_EXPOSURE_AUTO_PRIORITY: u32 = 0x009a_0903;

/// Per-camera V4L2 control overrides applied after format negotiation.
///
/// Matched case-insensitively against the sysfs device name substring.
/// `exposure_100us = None` means "leave auto-exposure on" (most cameras).
struct CameraQuirks {
    /// sysfs name substring to match (lowercase)
    name_contains: &'static str,
    /// If `Some`, disable auto-exposure and set this absolute exposure value
    /// (in V4L2 units of 100 µs).  150 = 15 ms — bright-room sweet spot for
    /// the Delan Cam 1 at 60 fps.
    exposure_100us: Option<i64>,
}

const CAMERA_QUIRKS: &[CameraQuirks] = &[
    CameraQuirks { name_contains: "delan", exposure_100us: Some(156) },
];

/// Raw V4L2 camera that bypasses nokhwa entirely.
pub struct V4l2Camera {
    /// Stream must be dropped before the device. `ManuallyDrop` lets us
    /// control drop order in our `Drop` impl.
    stream: ManuallyDrop<Stream<'static>>,
    /// The leaked `Device` — reclaimed and dropped after `stream` in `Drop`.
    dev_ptr: *mut Device,
    width: u32,
    height: u32,
    frame_count: u32,
}

// SAFETY: `Device` holds a file descriptor and is not thread-local.
// The raw pointer is only accessed through our own Drop impl.
unsafe impl Send for V4l2Camera {}

impl Drop for V4l2Camera {
    fn drop(&mut self) {
        // Drop stream first (stops mmap DMA, returns buffers to driver).
        unsafe { ManuallyDrop::drop(&mut self.stream); }
        // Now safe to close the device fd.
        unsafe { drop(Box::from_raw(self.dev_ptr)); }
    }
}

impl V4l2Camera {
    /// Open `/dev/videoN` with MJPEG at the requested resolution.
    pub fn open(index: u32, width: u32, height: u32, fps: u32) -> Result<Self> {
        let path = format!("/dev/video{index}");
        info!("v4l2 fallback: opening {path} at {width}×{height} MJPEG @{fps}fps");

        let dev = Device::with_path(&path)
            .with_context(|| format!("opening {path}"))?;

        // Set pixel format to MJPEG at requested resolution.
        let mut fmt = dev.format().context("reading current format")?;
        fmt.width = width;
        fmt.height = height;
        fmt.fourcc = FourCC::new(b"MJPG");
        let fmt = dev.set_format(&fmt).context("setting MJPEG format")?;
        info!(
            "v4l2 fallback: format set to {}×{} {:?}",
            fmt.width, fmt.height, fmt.fourcc
        );

        // Apply per-camera V4L2 control overrides BEFORE requesting fps.
        // The driver calculates the allowed framerate based on current exposure
        // mode — if auto-exposure is still active when we call set_params, the
        // driver may lock to a lower fps and not recalculate after we change mode.
        apply_camera_quirks(&dev, index);

        // Request the target frame rate (after exposure controls are configured).
        let mut params = dev.params().context("reading stream params")?;
        params.interval = Fraction::new(1, fps);
        let params = dev.set_params(&params).context("setting frame rate")?;
        let actual_fps = if params.interval.numerator > 0 {
            params.interval.denominator as f32 / params.interval.numerator as f32
        } else {
            0.0
        };
        info!("v4l2 fallback: frame rate {actual_fps:.1} fps");

        // Start streaming with memory-mapped buffers.
        // SAFETY: `dev_ptr` is stored in the struct and reclaimed in Drop,
        // always after `stream` is dropped (ManuallyDrop + explicit drop order).
        let dev_ptr = Box::into_raw(Box::new(dev));
        let stream = Stream::with_buffers(unsafe { &mut *dev_ptr }, Type::VideoCapture, 4)
            .context("starting mmap stream")?;

        Ok(Self {
            stream: ManuallyDrop::new(stream),
            dev_ptr,
            width: fmt.width,
            height: fmt.height,
            frame_count: 0,
        })
    }

    /// Capture one frame and decode MJPEG → RGB.
    pub fn frame(&mut self) -> Result<ImageBuffer<Rgb<u8>, Vec<u8>>> {
        let t0 = std::time::Instant::now();
        let (buf, _meta) = self.stream.next().context("capturing frame")?;
        let t_cap = t0.elapsed();

        let jpeg_len = buf.len();
        let img = image::load(Cursor::new(buf), image::ImageFormat::Jpeg)
            .context("decoding MJPEG frame")?
            .into_rgb8();
        let t_dec = t0.elapsed() - t_cap;

        if self.frame_count % 30 == 0 {
            info!(
                "v4l2 frame: cap={:.1}ms decode={:.1}ms jpeg={}KB",
                t_cap.as_secs_f64() * 1000.0,
                t_dec.as_secs_f64() * 1000.0,
                jpeg_len / 1024,
            );
        }
        self.frame_count += 1;

        Ok(img)
    }

    pub fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }
}

/// V4L2 camera capturing YUYV (raw uncompressed) — zero decode overhead.
///
/// YUYV stores two pixels per 4 bytes: [Y0, U, Y1, V].  The Y channel is
/// the luminance (≈ grayscale), so extracting gray is a trivial byte copy
/// instead of the 88 ms MJPEG decode that was bottlenecking the DC1.
pub struct V4l2YuyvCamera {
    /// Stream must be dropped before the device. `ManuallyDrop` lets us
    /// control drop order in our `Drop` impl.
    stream: ManuallyDrop<Stream<'static>>,
    /// The leaked `Device` — reclaimed and dropped after `stream` in `Drop`.
    dev_ptr: *mut Device,
    width: u32,
    height: u32,
    frame_count: u32,
}

// SAFETY: same as V4l2Camera — raw ptr only touched in Drop.
unsafe impl Send for V4l2YuyvCamera {}

impl Drop for V4l2YuyvCamera {
    fn drop(&mut self) {
        unsafe { ManuallyDrop::drop(&mut self.stream); }
        unsafe { drop(Box::from_raw(self.dev_ptr)); }
    }
}

impl V4l2YuyvCamera {
    /// Open `/dev/videoN` with YUYV at the requested resolution and fps.
    pub fn open(index: u32, width: u32, height: u32, fps: u32) -> Result<Self> {
        let path = format!("/dev/video{index}");
        info!("v4l2 yuyv: opening {path} at {width}×{height} YUYV @{fps}fps");

        let dev = Device::with_path(&path)
            .with_context(|| format!("opening {path}"))?;

        let mut fmt = dev.format().context("reading current format")?;
        fmt.width = width;
        fmt.height = height;
        fmt.fourcc = FourCC::new(b"YUYV");
        let fmt = dev.set_format(&fmt).context("setting YUYV format")?;
        info!(
            "v4l2 yuyv: format set to {}×{} {:?}",
            fmt.width, fmt.height, fmt.fourcc
        );

        // Apply per-camera V4L2 control overrides BEFORE requesting fps.
        apply_camera_quirks(&dev, index);

        let mut params = dev.params().context("reading stream params")?;
        params.interval = Fraction::new(1, fps);
        let params = dev.set_params(&params).context("setting frame rate")?;
        let actual_fps = if params.interval.numerator > 0 {
            params.interval.denominator as f32 / params.interval.numerator as f32
        } else {
            0.0
        };
        info!("v4l2 yuyv: frame rate {actual_fps:.1} fps");

        let dev_ptr = Box::into_raw(Box::new(dev));
        let stream = Stream::with_buffers(unsafe { &mut *dev_ptr }, Type::VideoCapture, 4)
            .context("starting mmap stream")?;

        Ok(Self {
            stream: ManuallyDrop::new(stream),
            dev_ptr,
            width: fmt.width,
            height: fmt.height,
            frame_count: 0,
        })
    }

    /// Capture one YUYV frame, returning the raw bytes (owned copy).
    ///
    /// Length = width × height × 2.
    pub fn frame_raw(&mut self) -> Result<Vec<u8>> {
        let t0 = std::time::Instant::now();
        let (buf, _meta) = self.stream.next().context("capturing yuyv frame")?;
        let t_cap = t0.elapsed();

        // buf is borrowed from the mmap region — we must copy before returning
        // the buffer to the driver.
        let owned = buf.to_vec();

        if t_cap.as_millis() > 300 {
            warn!("yuyv capture stall: {:.0}ms (camera may have frozen)", t_cap.as_millis());
        }
        if self.frame_count % 30 == 0 {
            info!(
                "v4l2 yuyv frame: cap={:.1}ms size={}KB",
                t_cap.as_secs_f64() * 1000.0,
                owned.len() / 1024,
            );
        }
        self.frame_count += 1;
        Ok(owned)
    }

    pub fn resolution(&self) -> (u32, u32) {
        (self.width, self.height)
    }
}

/// Extract the Y (luminance) channel from a YUYV buffer — this IS grayscale.
///
/// YUYV layout: [Y0, U, Y1, V, Y2, U, Y3, V, ...].  Every other byte
/// starting at offset 0 is a Y sample.  Zero computation — just a byte copy.
pub fn yuyv_to_gray(yuyv: &[u8], width: u32, height: u32) -> GrayImage {
    let npx = (width * height) as usize;
    let mut gray = Vec::with_capacity(npx);

    // Process 4 bytes at a time (2 pixels): [Y0, U, Y1, V]
    let chunks = yuyv.chunks_exact(4);
    for chunk in chunks {
        gray.push(chunk[0]); // Y0
        gray.push(chunk[2]); // Y1
    }

    gray.truncate(npx); // safety: ensure exact size
    GrayImage::from_vec(width, height, gray).expect("yuyv_to_gray: dimension mismatch")
}

/// Convert a YUYV buffer to RGB for preview display.
///
/// Uses the BT.601 conversion:
///   R = Y + 1.402 * (V - 128)
///   G = Y - 0.344 * (U - 128) - 0.714 * (V - 128)
///   B = Y + 1.772 * (U - 128)
///
/// Fixed-point to avoid per-pixel float ops.
pub fn yuyv_to_rgb(yuyv: &[u8], width: u32, height: u32) -> ImageBuffer<Rgb<u8>, Vec<u8>> {
    let npx = (width * height) as usize;
    let mut rgb = Vec::with_capacity(npx * 3);

    // Process 4 bytes at a time (2 pixels sharing U/V): [Y0, U, Y1, V]
    for chunk in yuyv.chunks_exact(4) {
        let y0 = chunk[0] as i32;
        let u = chunk[1] as i32 - 128;
        let v = chunk[3] as i32 - 128;
        let y1 = chunk[2] as i32;

        // Fixed-point: multiply by 256, shift right 8
        // 1.402 * 256 = 359, 0.344 * 256 = 88, 0.714 * 256 = 183, 1.772 * 256 = 454
        let r0 = (y0 + ((359 * v) >> 8)).clamp(0, 255) as u8;
        let g0 = (y0 - ((88 * u + 183 * v) >> 8)).clamp(0, 255) as u8;
        let b0 = (y0 + ((454 * u) >> 8)).clamp(0, 255) as u8;

        let r1 = (y1 + ((359 * v) >> 8)).clamp(0, 255) as u8;
        let g1 = (y1 - ((88 * u + 183 * v) >> 8)).clamp(0, 255) as u8;
        let b1 = (y1 + ((454 * u) >> 8)).clamp(0, 255) as u8;

        rgb.extend_from_slice(&[r0, g0, b0, r1, g1, b1]);
    }

    ImageBuffer::from_vec(width, height, rgb).expect("yuyv_to_rgb: dimension mismatch")
}

/// Disable dynamic framerate reduction on a camera opened via nokhwa.
///
/// Opens a second V4L2 fd on the same device (Linux allows this while nokhwa
/// is streaming) solely to clear `EXPOSURE_AUTO_PRIORITY`.  This prevents the
/// camera from dropping fps to accommodate a longer shutter in low light —
/// the camera still auto-exposes via gain/ISO, it just stays at the requested
/// frame rate.  Silently ignored if the camera doesn't support the control.
pub fn disable_framerate_priority(camera_index: u32) {
    let path = format!("/dev/video{camera_index}");
    let Ok(dev) = Device::with_path(&path) else { return };
    match dev.set_control(Control { id: V4L2_CID_EXPOSURE_AUTO_PRIORITY, value: Value::Boolean(false) }) {
        Ok(()) => info!("v4l2: exposure_dynamic_framerate disabled for camera {camera_index}"),
        Err(e) => warn!("v4l2: couldn't disable framerate priority for camera {camera_index}: {e}"),
    }
}

/// Read the sysfs device name for `/dev/videoN` and apply any matching quirks.
fn apply_camera_quirks(dev: &Device, index: u32) {
    let sysfs_name = std::fs::read_to_string(
        format!("/sys/class/video4linux/video{index}/name")
    )
    .unwrap_or_default();
    let sysfs_lower = sysfs_name.trim().to_lowercase();

    let quirks = CAMERA_QUIRKS
        .iter()
        .find(|q| sysfs_lower.contains(q.name_contains));

    let Some(q) = quirks else { return };

    info!("v4l2: applying quirks for '{}' (matched '{}')", sysfs_name.trim(), q.name_contains);

    if let Some(exp) = q.exposure_100us {
        // Step 1: disable dynamic framerate — without this the driver drops fps
        // to accommodate exposure even in manual mode (Delan Cam default=1).
        match dev.set_control(Control { id: V4L2_CID_EXPOSURE_AUTO_PRIORITY, value: Value::Boolean(false) }) {
            Ok(()) => info!("v4l2: exposure_dynamic_framerate disabled"),
            Err(e) => warn!("v4l2: couldn't clear exposure_dynamic_framerate: {e}"),
        }
        // Step 2: switch to manual exposure mode.
        match dev.set_control(Control { id: V4L2_CID_EXPOSURE_AUTO, value: Value::Integer(1) }) {
            Ok(()) => info!("v4l2: auto-exposure disabled (manual mode)"),
            Err(e) => warn!("v4l2: couldn't set exposure_auto: {e}"),
        }
        // Step 3: set absolute exposure (in 100 µs units; camera min=156).
        match dev.set_control(Control { id: V4L2_CID_EXPOSURE_ABSOLUTE, value: Value::Integer(exp) }) {
            Ok(()) => info!("v4l2: exposure_absolute set to {exp} (~{}ms)", exp / 10),
            Err(e) => warn!("v4l2: couldn't set exposure_absolute: {e}"),
        }
    }
}
