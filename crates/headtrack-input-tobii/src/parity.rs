//! Asset-integrity and numerical-parity tests (new in HeadtrackRS; not in TobiiLab's
//! Rust tests, whose parity check was `scripts/check-litert-parity.py`).
//!
//! The fixture `parity-initial-20260907/` is TobiiLab's saved audit of one
//! 280x280 IR frame: exact model inputs and the outputs TobiiLab's native tract run
//! produced, which TobiiLab compared against ai-edge-litert 2.2.0 at absolute
//! tolerances 0.02 (geometry/regressors) and 0.002 (logits/flags). We feed the same
//! inputs through the ported estimator and require the same tolerances against those
//! saved outputs.
//!
//! **Privacy (hardline, 2026-10-03):** the two `*-input.f32` files are a real person's
//! IR face stored as floats, so the fixture is NOT in the repository. It lives outside
//! the published set (P06 moved it to
//! `~/headtrack-dataset/private-fixtures/parity-initial-20260907/`). The two tests that
//! need it read `HEADTRACK_PRIVATE_FIXTURES` (the directory *containing*
//! `parity-initial-20260907/`) and skip with a printed reason when it is unset or absent:
//!
//! ```text
//! HEADTRACK_PRIVATE_FIXTURES=~/headtrack-dataset/private-fixtures \
//!   cargo test -p headtrack-input-tobii -- --nocapture parity
//! ```
use super::*;
use sha2::{Digest, Sha256};
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}
fn model_dir() -> PathBuf {
    repo_root().join("assets/models/tobii")
}
const PRIVATE_FIXTURE: &str = "parity-initial-20260907";

/// Where the private parity fixture lives, or `None` (with the reason printed) to skip.
fn private_fixture_dir() -> Option<PathBuf> {
    let Some(root) = std::env::var_os("HEADTRACK_PRIVATE_FIXTURES") else {
        eprintln!(
            "SKIPPED: HEADTRACK_PRIVATE_FIXTURES is unset; the parity fixture is a private \
             real-face capture kept outside the repository (see parity.rs)"
        );
        return None;
    };
    let dir = PathBuf::from(root).join(PRIVATE_FIXTURE);
    if !dir.is_dir() {
        eprintln!(
            "SKIPPED: {} does not exist; the parity fixture is private and not in the repository",
            dir.display()
        );
        return None;
    }
    Some(dir)
}
fn fixture(dir: &Path, name: &str) -> Vec<f32> {
    let p = dir.join(name);
    let b = std::fs::read(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    assert_eq!(b.len() % 4, 0, "{name}");
    b.as_chunks::<4>()
        .0
        .iter()
        .map(|c| f32::from_le_bytes(*c))
        .collect()
}
fn sha256_file(p: &Path) -> String {
    format!(
        "{:x}",
        Sha256::digest(std::fs::read(p).unwrap_or_else(|e| panic!("{}: {e}", p.display())))
    )
}
fn max_abs(a: &[f32], b: &[f32]) -> f32 {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0., f32::max)
}

#[test]
fn shipped_assets_match_manifest_and_pins() {
    let dir = model_dir();
    let manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(dir.join("MANIFEST.json")).unwrap()).unwrap();
    let files = manifest["files"].as_object().unwrap();
    assert_eq!(files.len(), 7);
    for (name, digest) in files {
        let path = dir.join(name);
        // The two model-card PDFs embed photographs, so they are local-only and
        // never published (P16). MANIFEST.json still records them as provenance;
        // when present they must match, and a clean clone simply lacks them.
        if name.ends_with(".pdf") && !path.exists() {
            continue;
        }
        assert_eq!(sha256_file(&path), digest.as_str().unwrap(), "{name}");
    }
    assert_eq!(files[DETECTOR_FILE], DETECTOR_SHA256);
    assert_eq!(files[MESH_FILE], MESH_SHA256);
    // Every file in the directory is either listed or one of the two unhashed docs.
    for e in std::fs::read_dir(&dir).unwrap() {
        let n = e.unwrap().file_name().into_string().unwrap();
        assert!(
            files.contains_key(&n) || n == "MANIFEST.json" || n == "NOTICE.md",
            "unlisted file in assets/models/tobii: {n}"
        );
    }
}

#[test]
fn tract_snapshot_hashes_verify() {
    let root = repo_root().join("vendor/tract-tflite");
    let snap: serde_json::Value =
        serde_json::from_slice(&std::fs::read(root.join("TRACT_SNAPSHOT.json")).unwrap()).unwrap();
    let modified = snap["modified_file"].as_str().unwrap();
    let mut checked = 0;
    for (name, digest) in snap["original_sha256"].as_object().unwrap() {
        let expected = if name == modified {
            snap["modified_sha256"].as_str().unwrap()
        } else {
            digest.as_str().unwrap()
        };
        assert_eq!(sha256_file(&root.join(name)), expected, "{name}");
        checked += 1;
    }
    for (name, digest) in snap["added_files"].as_object().unwrap() {
        assert_eq!(
            sha256_file(&root.join(name)),
            digest.as_str().unwrap(),
            "{name}"
        );
        checked += 1;
    }
    assert_eq!(checked, 22); // 20 original files + 2 added licence files
}

#[test]
fn shipped_models_load() {
    Estimator::load(&model_dir()).expect("shipped models verify and load");
}

fn staged_models() -> tempfile::TempDir {
    let t = tempfile::tempdir().unwrap();
    for f in [DETECTOR_FILE, MESH_FILE] {
        std::fs::copy(model_dir().join(f), t.path().join(f)).unwrap();
    }
    t
}

#[test]
fn unmodified_temp_copy_loads() {
    let t = staged_models();
    Estimator::load(t.path()).expect("pristine copy loads");
}

#[test]
fn flipped_byte_in_either_model_is_a_typed_checksum_error() {
    for file in [DETECTOR_FILE, MESH_FILE] {
        let t = staged_models();
        let p = t.path().join(file);
        let mut bytes = std::fs::read(&p).unwrap();
        let mid = bytes.len() / 2;
        bytes[mid] ^= 0x01;
        std::fs::write(&p, &bytes).unwrap();
        match Estimator::load(t.path()) {
            Err(ModelError::ChecksumMismatch {
                path,
                expected,
                actual,
            }) => {
                assert!(path.ends_with(file));
                assert_ne!(expected, actual);
                assert_eq!(
                    expected,
                    if file == DETECTOR_FILE {
                        DETECTOR_SHA256
                    } else {
                        MESH_SHA256
                    }
                );
            }
            Err(e) => panic!("{file}: wrong error variant: {e:?}"),
            Ok(_) => panic!("{file}: tampered model was accepted"),
        }
    }
}

#[test]
fn checksum_is_checked_before_parsing() {
    // Not a TFLite file at all: if the parser ran first this would be `Invalid`.
    let t = staged_models();
    std::fs::write(t.path().join(DETECTOR_FILE), b"definitely not a flatbuffer").unwrap();
    assert!(matches!(
        Estimator::load(t.path()),
        Err(ModelError::ChecksumMismatch { .. })
    ));
}

#[test]
fn missing_and_oversize_models_are_typed() {
    let t = tempfile::tempdir().unwrap();
    assert!(matches!(
        Estimator::load(t.path()),
        Err(ModelError::Io { .. })
    ));
    std::fs::write(
        t.path().join(DETECTOR_FILE),
        vec![0u8; MAX_MODEL_BYTES as usize + 1],
    )
    .unwrap();
    assert!(matches!(
        Estimator::load(t.path()),
        Err(ModelError::BadFileSize { .. })
    ));
}

#[test]
fn native_tensors_match_saved_tobiilab_audit_at_tobiilab_tolerance() {
    let Some(dir) = private_fixture_dir() else {
        return;
    };
    let fixture = |name: &str| fixture(&dir, name);
    let est = Estimator::load(&model_dir()).unwrap();
    let det = run(
        &est.detector,
        &fixture("detector-input.f32"),
        128,
        None,
        "detector",
    )
    .unwrap();
    let mesh = run(&est.mesh, &fixture("mesh-input.f32"), 256, None, "mesh").unwrap();
    assert_eq!(det.len(), 2);
    assert_eq!(mesh.len(), 3);
    // (got, saved, TobiiLab tolerance, label)
    let cases: [(&Vec<f32>, &str, f32, &str); 5] = [
        (
            &det[0],
            "detector-output-0.f32",
            0.02,
            "detector regressors",
        ),
        (
            &det[1],
            "detector-output-1.f32",
            0.002,
            "detector classificators",
        ),
        (&mesh[0], "mesh-output-0.f32", 0.02, "mesh landmarks"),
        (&mesh[1], "mesh-output-1.f32", 0.002, "mesh presence logit"),
        (&mesh[2], "mesh-output-2.f32", 0.002, "mesh Identity_2"),
    ];
    for (got, file, tol, label) in cases {
        let m = max_abs(got, &fixture(file));
        eprintln!("parity {label}: max_abs={m:e} (tolerance {tol})");
        assert!(m <= tol, "{label}: max_abs {m} > {tol}");
    }
}

#[test]
fn saved_tensors_run_through_decode_crop_and_fit() {
    let Some(dir) = private_fixture_dir() else {
        return;
    };
    let fixture = |name: &str| fixture(&dir, name);
    let boxes = fixture("detector-output-0.f32");
    let scores = fixture("detector-output-1.f32");
    let dets = decode_detections(&boxes, &scores).unwrap();
    assert_eq!(dets.len(), 1, "fixture frame holds exactly one face");
    assert!(dets[0].score >= 0.6);
    let crop = detection_crop(&dets[0]);
    assert!(crop.size.is_finite() && (35.0..=420.).contains(&crop.size));
    let presence = sigmoid(fixture("mesh-output-1.f32")[0]);
    assert!(presence >= 0.5, "presence {presence}");
    let landmarks = project_landmarks(&fixture("mesh-output-0.f32"), crop);
    assert_eq!(landmarks.len(), 478);
    let (angles, rms) = fit_rotation(&landmarks).expect("fit");
    eprintln!(
        "chain: score={} presence={presence} crop={crop:?} angles={angles:?} rms={rms}",
        dets[0].score
    );
    assert!(rms <= 0.25, "fit rms {rms}");
    assert!(angles.iter().all(|a| a.is_finite() && a.abs() < 90.));
}
