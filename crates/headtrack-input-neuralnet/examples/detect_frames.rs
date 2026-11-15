//! Run a detector over a directory of saved frames and print CSV.
//!
//! The offline half of the detector bake-off. Its purpose is verification: the
//! Rust YuNet decode is hand-written from the ONNX output layout, and this lets
//! it be checked against OpenCV's own `FaceDetectorYN` on the *same* frames —
//! `scripts/detector_bakeoff.py` already wrote those reference results.
//!
//! A decoder that is subtly wrong (an off-by-one in the anchor grid, a
//! transposed keypoint pair) still detects faces and still looks plausible in a
//! preview. It shows up as degraded tracking that gets blamed on the model.
//!
//! ```bash
//! cargo run -p headtrack-input-neuralnet --example detect_frames -- \
//!     yunet ~/headtrack-bakeoff/yaw-sweep > /tmp/rust-yunet.csv
//! ```
use std::path::PathBuf;

use headtrack_input_neuralnet::{scrfd_detector::ScrfdDetector, yunet_detector::YuNetDetector};

enum Det {
    Scrfd(Box<ScrfdDetector>),
    YuNet(Box<YuNetDetector>),
}

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let which = args.next().unwrap_or_else(|| "yunet".into());
    let dir: PathBuf = args.next().map(PathBuf::from).ok_or_else(|| {
        anyhow::anyhow!("usage: detect_frames <scrfd|yunet> <frame-dir> [models-dir]")
    })?;
    let models: PathBuf = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("assets/models"));

    let mut det = match which.as_str() {
        "scrfd" => Det::Scrfd(Box::new(ScrfdDetector::load(
            &models.join("scrfd-det-10g.onnx"),
        )?)),
        "yunet" => Det::YuNet(Box::new(YuNetDetector::load(
            &models.join(headtrack_input_neuralnet::yunet_detector::MODEL_FILENAME),
        )?)),
        other => anyhow::bail!("unknown detector {other}"),
    };

    // Frame order must match the capture's numbering, since the comparison
    // joins on frame_idx.
    let mut frames: Vec<PathBuf> = std::fs::read_dir(&dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "jpg"))
        .collect();
    frames.sort();

    println!("frame_idx,found,score,bx,by,bw,bh,kp0_x,kp0_y,kp1_x,kp1_y,kp2_x,kp2_y,kp3_x,kp3_y,kp4_x,kp4_y");
    for path in frames {
        let idx: u64 = path
            .file_stem()
            .and_then(|s| s.to_str())
            .and_then(|s| s.rsplit('_').next())
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);
        let img = image::open(&path)?.to_rgb8();
        let res = match &mut det {
            Det::Scrfd(d) => d.detect(&img)?,
            Det::YuNet(d) => d.detect(&img)?,
        };
        match res {
            Some(f) => {
                print!(
                    "{idx},1,{:.4},{:.2},{:.2},{:.2},{:.2}",
                    f.bbox.score, f.bbox.x, f.bbox.y, f.bbox.w, f.bbox.h
                );
                for (x, y) in f.kps.iter().take(5) {
                    print!(",{x:.3},{y:.3}");
                }
                println!();
            }
            None => println!("{idx},0,,,,,,,,,,,,,,,"),
        }
    }
    Ok(())
}
