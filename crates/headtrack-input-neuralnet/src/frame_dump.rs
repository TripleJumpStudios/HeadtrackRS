//! Developer frame capture for the detector bake-off (`HEADTRACK_DUMP_FRAMES`).
//!
//! Writes the full-resolution camera frame plus the per-frame tracker labels, so
//! two face detectors can be compared **offline on identical frames**. Without
//! this there is nothing to compare against: the product deliberately writes no
//! image data, and `HEADTRACK_DUMP_CROP` was deleted in Snow Leopard 3.
//!
//! # This is not a product feature
//!
//! **Compiled in only with the non-default `frame-dump` Cargo feature** (Q-1,
//! 2026-10-18). A release build — the AppImage included — has no code that can
//! write a camera frame. Without the feature, [`FrameDumper::open_if_enabled`]
//! is a stub that warns once if `HEADTRACK_DUMP_FRAMES` is set and records
//! nothing. `scripts/capture-frames.sh` builds with the feature.
//!
//! Even when compiled in, it is off unless `HEADTRACK_DUMP_FRAMES` is set to a
//! truthy value, exactly like
//! [`headtrack_core::env_flag`]'s other opt-in diagnostics. It is the **only**
//! path in the codebase that writes camera imagery to disk. The user-facing
//! diagnostic recorder still writes pose numbers and nothing else — see
//! `docs/CONTRIBUTING_DATA.md`, whose privacy claim covers that recorder and is
//! unaffected by this flag.
//!
//! ```bash
//! HEADTRACK_DUMP_FRAMES=1 HEADTRACK_DUMP_FRAMES_DIR=~/bakeoff/yaw-sweep \
//!     cargo run -p headtrack-gui --features headtrack-engine/frame-dump
//! ```
//!
//! # Cost
//!
//! A full-resolution JPEG encode plus a synchronous write per frame, on the
//! tracking thread. That **will** cost frame rate at 60 fps — accepted
//! deliberately, because every label in `frames.csv` is derived from the frame
//! it sits beside, so a slower capture still yields correctly paired data. Do
//! not leave it on for a tuning session.
//!
//! # Output
//!
//! ```text
//! <dir>/frame_000000.jpg   full-resolution, quality 92
//! <dir>/frames.csv         one row per frame, joined by frame_idx
//! ```
//!
//! ## Why `prev_kps` is in the schema
//!
//! Criterion 5 of the bake-off is the **re-anchor jump**: how far the landmark
//! set moves when a periodic re-detect replaces optical-flow-carried points with
//! fresh detector output. That jump is what a "tick" is made of.
//!
//! Measuring it exactly would need both optical flow *and* the detector run on
//! the same frame, but [`REDETECT_THRESHOLD`](crate::tracker) makes them
//! mutually exclusive — a re-detect frame skips optical flow entirely. So we log
//! the **previous** frame's landmarks instead, which is off by one frame of head
//! motion.
//!
//! That approximation is sound for this comparison specifically: the metric
//! compares candidate-vs-SCRFD jump, both measured against the same `prev_kps`,
//! so the one-frame error is common-mode and cancels. It would *not* be sound as
//! an absolute measure of re-anchor distance.

#[cfg(feature = "frame-dump")]
use std::fs::File;
#[cfg(feature = "frame-dump")]
use std::io::{BufWriter, Write};
#[cfg(feature = "frame-dump")]
use std::path::PathBuf;

use image::{ImageBuffer, Rgb};
#[cfg(feature = "frame-dump")]
use tracing::info;
use tracing::warn;

use crate::scrfd_detector::Detection;

/// Which source produced the landmarks used on a given frame.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum LandmarkSource {
    /// Optical flow carried the previous frame's points forward.
    OpticalFlow,
    /// The detector ran and produced a fresh set.
    Detector,
    /// Neither succeeded — this frame is a miss.
    #[default]
    None,
}

impl LandmarkSource {
    #[cfg_attr(not(feature = "frame-dump"), allow(dead_code))]
    fn as_str(self) -> &'static str {
        match self {
            Self::OpticalFlow => "of",
            Self::Detector => "scrfd",
            Self::None => "none",
        }
    }
}

/// Per-frame facts gathered during inference, written out once the frame is done.
///
/// Filled in by `run_inference` as each fact becomes known. Only touched when
/// dumping is enabled, so the normal hot path never allocates for it.
#[derive(Default)]
pub(crate) struct FrameFacts {
    /// True if the detector was invoked this frame (whether or not it found a face).
    pub scrfd_ran: bool,
    /// The detector's bounding box, when it ran and succeeded.
    pub bbox: Option<Detection>,
    /// Landmarks actually used this frame, from whichever source.
    pub kps: Vec<(f32, f32)>,
    /// Previous frame's landmarks — the re-anchor jump reference. See module docs.
    pub prev_kps: Vec<(f32, f32)>,
    /// Where `kps` came from.
    pub source: LandmarkSource,
}

impl FrameFacts {
    /// Reset for a new frame, preserving the allocation of both point vectors.
    pub fn begin(&mut self, prev: Option<&Vec<(f32, f32)>>) {
        self.scrfd_ran = false;
        self.bbox = None;
        self.kps.clear();
        self.prev_kps.clear();
        if let Some(p) = prev {
            self.prev_kps.extend_from_slice(p);
        }
        self.source = LandmarkSource::None;
    }
}

/// Number of keypoints written per point-set. SCRFD emits 5; the CSV schema is
/// fixed-width so the offline harness can parse it without a variable schema.
#[cfg(feature = "frame-dump")]
const KPS: usize = 5;

/// Release-build stand-in: there is no way to construct one, so `write` is
/// unreachable and no image-writing code is compiled.
#[cfg(not(feature = "frame-dump"))]
pub(crate) enum FrameDumper {}

#[cfg(not(feature = "frame-dump"))]
impl FrameDumper {
    /// Always `None`. Warns once if `HEADTRACK_DUMP_FRAMES` is set, so a
    /// developer capture run against the wrong build fails loudly, not silently.
    pub fn open_if_enabled() -> Option<Self> {
        if headtrack_core::env_flag("HEADTRACK_DUMP_FRAMES") {
            warn!(
                "HEADTRACK_DUMP_FRAMES is set, but this build has no frame capture \
                 (build with --features headtrack-engine/frame-dump). Recording nothing."
            );
        }
        None
    }

    pub fn write(
        &mut self,
        _rgb: Option<&ImageBuffer<Rgb<u8>, Vec<u8>>>,
        _pose: Option<&headtrack_core::RawPose>,
        _facts: &FrameFacts,
        _timestamp_us: u64,
    ) {
        match *self {}
    }
}

#[cfg(feature = "frame-dump")]
pub(crate) struct FrameDumper {
    dir: PathBuf,
    csv: BufWriter<File>,
    frame_idx: u64,
    line: String,
    /// Encode buffer, reused across frames.
    jpeg: Vec<u8>,
    /// Warn once rather than per frame if writes start failing (disk full).
    warned: bool,
}

#[cfg(feature = "frame-dump")]
impl FrameDumper {
    /// Returns a dumper only when `HEADTRACK_DUMP_FRAMES` is truthy.
    ///
    /// A failure to create the directory or the CSV warns and disables the dump
    /// rather than taking the tracker down — a diagnostic must never be the
    /// reason tracking stops.
    pub fn open_if_enabled() -> Option<Self> {
        if !headtrack_core::env_flag("HEADTRACK_DUMP_FRAMES") {
            return None;
        }
        let dir = PathBuf::from(
            std::env::var("HEADTRACK_DUMP_FRAMES_DIR")
                .unwrap_or_else(|_| "/tmp/headtrack-frames".to_string()),
        );
        if let Err(e) = std::fs::create_dir_all(&dir) {
            warn!("HEADTRACK_DUMP_FRAMES set but {dir:?} could not be created: {e}");
            return None;
        }
        let csv_path = dir.join("frames.csv");
        let file = match File::create(&csv_path) {
            Ok(f) => f,
            Err(e) => {
                warn!("HEADTRACK_DUMP_FRAMES set but {csv_path:?} could not be opened: {e}");
                return None;
            }
        };
        let mut csv = BufWriter::new(file);

        let mut header = String::from(
            "frame_idx,timestamp_us,yaw,pitch,roll,source,scrfd_ran,\
             bbox_x,bbox_y,bbox_w,bbox_h,bbox_score",
        );
        for i in 0..KPS {
            header.push_str(&format!(",kp{i}_x,kp{i}_y"));
        }
        for i in 0..KPS {
            header.push_str(&format!(",prev_kp{i}_x,prev_kp{i}_y"));
        }
        header.push('\n');
        if let Err(e) = csv.write_all(header.as_bytes()) {
            warn!("HEADTRACK_DUMP_FRAMES: writing {csv_path:?} header failed: {e}");
            return None;
        }

        info!(
            "HEADTRACK_DUMP_FRAMES=1 — full-resolution frame capture -> {}",
            dir.display()
        );
        info!("  this is a developer capture: it writes camera imagery and costs frame rate");
        Some(Self {
            dir,
            csv,
            frame_idx: 0,
            line: String::with_capacity(320),
            jpeg: Vec::with_capacity(128 * 1024),
            warned: false,
        })
    }

    /// Write one frame's image and label row.
    ///
    /// `pose` is `None` on a miss — the row is still written, with the pose
    /// columns empty, because a frame the current detector *failed* on is
    /// precisely the most interesting row in the file.
    pub fn write(
        &mut self,
        rgb: Option<&ImageBuffer<Rgb<u8>, Vec<u8>>>,
        pose: Option<&headtrack_core::RawPose>,
        facts: &FrameFacts,
        timestamp_us: u64,
    ) {
        let idx = self.frame_idx;
        self.frame_idx += 1;

        if let Some(img) = rgb {
            if let Err(e) = self.write_jpeg(idx, img) {
                if !self.warned {
                    warn!(
                        "HEADTRACK_DUMP_FRAMES: frame write failed, further errors suppressed: {e}"
                    );
                    self.warned = true;
                }
            }
        }

        use std::fmt::Write as _;
        self.line.clear();
        let _ = write!(self.line, "{idx},{timestamp_us},");
        match pose {
            Some(p) => {
                let _ = write!(self.line, "{:.5},{:.5},{:.5},", p.yaw, p.pitch, p.roll);
            }
            // Miss: no pose this frame. Left empty rather than zero — zero is a
            // valid pose and would silently become a data point.
            None => self.line.push_str(",,,"),
        }
        let _ = write!(
            self.line,
            "{},{},",
            facts.source.as_str(),
            facts.scrfd_ran as u8
        );
        match facts.bbox {
            Some(b) => {
                let _ = write!(
                    self.line,
                    "{:.2},{:.2},{:.2},{:.2},{:.4}",
                    b.x, b.y, b.w, b.h, b.score
                );
            }
            None => self.line.push_str(",,,,"),
        }
        write_points(&mut self.line, &facts.kps);
        write_points(&mut self.line, &facts.prev_kps);
        self.line.push('\n');

        let _ = self.csv.write_all(self.line.as_bytes());
        // Flush per row: a capture usually ends by closing the window, which
        // runs no shutdown hook — same reasoning as `RawPoseRecorder`.
        let _ = self.csv.flush();
    }

    fn write_jpeg(&mut self, idx: u64, rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>) -> anyhow::Result<()> {
        use image::codecs::jpeg::JpegEncoder;
        use image::ExtendedColorType;

        self.jpeg.clear();
        // Quality 92: high enough that JPEG artifacts cannot plausibly be
        // mistaken for a detector difference, which is the whole point of the
        // capture. Deliberately not the preview encoder — that downscales to
        // 320 px wide and would destroy the keypoint precision under test.
        let mut enc = JpegEncoder::new_with_quality(&mut self.jpeg, 92);
        enc.encode(
            rgb.as_raw(),
            rgb.width(),
            rgb.height(),
            ExtendedColorType::Rgb8,
        )?;

        let path = self.dir.join(format!("frame_{idx:06}.jpg"));
        std::fs::write(path, &self.jpeg)?;
        Ok(())
    }
}

/// Append a fixed-width point set: `KPS` pairs, empty fields when absent, so
/// every row has the same column count regardless of what the tracker managed.
#[cfg(feature = "frame-dump")]
fn write_points(line: &mut String, pts: &[(f32, f32)]) {
    use std::fmt::Write as _;
    for i in 0..KPS {
        match pts.get(i) {
            Some(&(x, y)) => {
                let _ = write!(line, ",{x:.3},{y:.3}");
            }
            None => line.push_str(",,"),
        }
    }
}

#[cfg(all(test, not(feature = "frame-dump")))]
mod release_build_tests {
    use super::FrameDumper;

    /// Q-1: a build without `frame-dump` cannot capture frames, even when asked to.
    #[test]
    fn a_release_build_never_opens_a_frame_dumper() {
        std::env::set_var("HEADTRACK_DUMP_FRAMES", "1");
        let dumper = FrameDumper::open_if_enabled();
        std::env::remove_var("HEADTRACK_DUMP_FRAMES");
        assert!(dumper.is_none());
    }
}
