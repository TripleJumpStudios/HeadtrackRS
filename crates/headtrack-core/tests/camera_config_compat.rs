//! Golden `camera.toml` fixtures for the P05 head-source contract
//! (`docs/P05_SOURCE_CONTRACT.md` §8).
//!
//! Old files must load unchanged and open webcam 0, exactly as before P05.
//! New files must load to exactly what they say. A source kind this build does
//! not know must not cost the user their controller bindings.

use headtrack_core::config::{CameraConfig, HeadSource, ProfileConfig, WebcamSelection};
use headtrack_core::input::SourceKind;
use headtrack_core::pipeline::{DistanceScale, DistanceScalePolicy};
use headtrack_core::{TobiiState, TobiiStatus};

fn parse(fixture: &str) -> CameraConfig {
    toml::from_str(fixture).expect("fixture must parse")
}

const V011_MINIMAL: &str = include_str!("fixtures/camera-v0.1.1-minimal.toml");
const PRE_P05_FULL: &str = include_str!("fixtures/camera-pre-p05-full.toml");
const P05_WEBCAM: &str = include_str!("fixtures/camera-p05-webcam.toml");
const P05_TOBII: &str = include_str!("fixtures/camera-p05-tobii.toml");
const P05_TOBII_NO_SERIAL: &str = include_str!("fixtures/camera-p05-tobii-no-serial.toml");
const FUTURE_SOURCE: &str = include_str!("fixtures/camera-future-source.toml");

#[test]
fn v011_minimal_loads_as_webcam_zero_with_defaults() {
    let cfg = parse(V011_MINIMAL);
    let def = CameraConfig::default();
    assert_eq!(cfg.fov_diag_deg, 60.0);
    assert_eq!(
        cfg.head_source,
        HeadSource::Webcam {
            index: 0,
            name: None
        }
    );
    assert_eq!(cfg.last_webcam, None);
    assert_eq!(cfg.source_kind(), SourceKind::Webcam);
    assert_eq!(cfg.webcam_selection(), WebcamSelection::default());
    // No resolution line: this user ran the old 640×480 default and keeps it,
    // although a fresh install now gets 1280×720 (Q-UI6b).
    assert_eq!(cfg.camera_resolution, (640, 480));
    assert_eq!(cfg.camera_resolution, CameraConfig::legacy_resolution());
    assert_eq!(cfg.minimize_to_tray, def.minimize_to_tray);
    assert_eq!(cfg.msfs_camera_smoothing, def.msfs_camera_smoothing);
    assert_eq!(cfg.sc_output_smoothing, def.sc_output_smoothing);
    assert!(!cfg.show_advanced);
    assert!(cfg.controller_bindings.is_empty());
}

/// Q-UI6b: a fresh install (no file) captures at 1280×720; a saved choice of
/// either resolution survives a load and a save.
#[test]
fn a_fresh_install_gets_720p_and_saved_choices_stay() {
    assert_eq!(CameraConfig::default().camera_resolution, (1280, 720));
    assert_eq!(CameraConfig::default_resolution(), (1280, 720));
    for saved in [(640, 480), (320, 240), (1280, 720)] {
        let cfg = CameraConfig {
            camera_resolution: saved,
            ..CameraConfig::default()
        };
        let back = parse(&toml::to_string_pretty(&cfg).expect("serialize"));
        assert_eq!(back.camera_resolution, saved);
    }
}

#[test]
fn pre_p05_full_keeps_every_value() {
    let cfg = parse(PRE_P05_FULL);
    assert_eq!(cfg.fov_diag_deg, 84.0);
    assert_eq!(cfg.camera_resolution, (1280, 720));
    assert!(!cfg.minimize_to_tray);
    assert_eq!(cfg.diag_directory, "/home/pilot/htrs-diag");
    assert_eq!(cfg.hotkey_recenter.as_deref(), Some("F9"));
    assert_eq!(cfg.hotkey_toggle.as_deref(), Some("F10"));
    assert_eq!(cfg.msfs_camera_smoothing, 12.5);
    assert_eq!(cfg.sc_output_smoothing, 0.0);
    assert!(cfg.show_advanced);
    assert_eq!(cfg.window_size, Some((1200.0, 800.0)));
    assert_eq!(cfg.fov_presets.len(), 2);
    assert_eq!(cfg.fov_presets[1].name, "C920");
    assert_eq!(cfg.controller_bindings.len(), 2);
    assert_eq!(cfg.controller_bindings[1].action, "msfs-seat-fwd");
    assert_eq!(cfg.controller_bindings[1].code, 722);
    // The P05 fields default to exactly the pre-P05 behaviour.
    assert_eq!(cfg.head_source, HeadSource::default());
    assert_eq!(cfg.last_webcam, None);
}

#[test]
fn pre_p05_full_survives_a_save_cycle() {
    // load → save → load: what P07 does on the first source switch.
    let cfg = parse(PRE_P05_FULL);
    let back = parse(&toml::to_string_pretty(&cfg).expect("serialize"));
    assert_eq!(back.controller_bindings.len(), 2);
    assert_eq!(back.fov_presets.len(), 2);
    assert_eq!(back.window_size, cfg.window_size);
    assert_eq!(back.head_source, HeadSource::default());
}

/// The golden new-format file: built in code, serialized, compared byte for
/// byte. A change to the on-disk shape fails here first.
#[test]
fn p05_webcam_is_byte_identical_to_its_serialization() {
    let mut cfg = CameraConfig {
        fov_diag_deg: 84.0,
        camera_resolution: (1280, 720),
        diag_directory: "/home/pilot".into(),
        ..CameraConfig::default()
    };
    cfg.select_webcam(2, Some("Elgato Facecam MK.2".into()));
    let text = toml::to_string_pretty(&cfg).expect("serialize");
    assert_eq!(text, P05_WEBCAM, "on-disk shape changed:\n{text}");

    let back = parse(P05_WEBCAM);
    assert_eq!(
        back.head_source,
        HeadSource::Webcam {
            index: 2,
            name: Some("Elgato Facecam MK.2".into())
        }
    );
    assert_eq!(back.webcam_selection().index, 2);
}

#[test]
fn p05_tobii_remembers_the_webcam() {
    let cfg = parse(P05_TOBII);
    assert_eq!(cfg.source_kind(), SourceKind::Tobii);
    assert_eq!(
        cfg.head_source,
        HeadSource::Tobii {
            serial: Some("IS5FF-100203345678".into())
        }
    );
    let w = cfg.webcam_selection();
    assert_eq!(w.index, 2);
    assert_eq!(w.name.as_deref(), Some("Elgato Facecam MK.2"));
}

#[test]
fn p05_tobii_without_serial_or_webcam() {
    let cfg = parse(P05_TOBII_NO_SERIAL);
    assert_eq!(cfg.head_source, HeadSource::Tobii { serial: None });
    assert_eq!(cfg.webcam_selection(), WebcamSelection::default());
}

#[test]
fn unknown_source_kind_falls_back_and_keeps_bindings() {
    let cfg = parse(FUTURE_SOURCE);
    assert_eq!(cfg.head_source, HeadSource::default());
    assert_eq!(cfg.fov_diag_deg, 84.0);
    assert_eq!(
        cfg.controller_bindings.len(),
        1,
        "bindings must survive an unknown source"
    );
}

#[test]
fn malformed_file_is_still_an_error() {
    // Leniency is scoped to [head_source]; a duplicate key elsewhere still fails.
    let bad = "fov_diag_deg = 84.0\nfov_diag_deg = 78.0\n[head_source]\nkind = \"webcam\"\n";
    assert!(toml::from_str::<CameraConfig>(bad).is_err());
}

#[test]
fn webcam_tobii_webcam_round_trip_keeps_the_webcam() {
    let mut cfg = CameraConfig::default();
    cfg.select_webcam(3, Some("Logitech C920".into()));
    cfg.select_tobii(None);
    assert_eq!(cfg.source_kind(), SourceKind::Tobii);
    // Persisted through a save in between, as a real restart would.
    let mut cfg = parse(&toml::to_string_pretty(&cfg).expect("serialize"));
    let w = cfg.webcam_selection();
    cfg.select_webcam(w.index, w.name);
    assert_eq!(
        cfg.head_source,
        HeadSource::Webcam {
            index: 3,
            name: Some("Logitech C920".into())
        }
    );
}

#[test]
fn tobii_to_tobii_does_not_clobber_the_remembered_webcam() {
    let mut cfg = CameraConfig::default();
    cfg.select_webcam(1, None);
    cfg.select_tobii(None);
    cfg.select_tobii(Some("SERIAL-B".into()));
    assert_eq!(cfg.webcam_selection().index, 1);
}

#[test]
fn per_source_policy() {
    assert_eq!(
        SourceKind::Webcam.distance_scale_policy(),
        DistanceScalePolicy::FromStandoff
    );
    assert_eq!(
        SourceKind::Tobii.distance_scale_policy(),
        DistanceScalePolicy::None
    );
    assert!(SourceKind::Webcam.observes_standoff());
    assert!(!SourceKind::Tobii.observes_standoff());
    assert!(SourceKind::Webcam.provides_translation());
    assert!(!SourceKind::Tobii.provides_translation());

    // Tobii is never scaled, whatever standoff is fed in.
    assert_eq!(
        DistanceScalePolicy::None.scale_for(1000.0),
        DistanceScale::NONE
    );
    assert_eq!(
        DistanceScalePolicy::FromStandoff.scale_for(1000.0),
        DistanceScale::from_standoff_mm(1000.0)
    );
}

#[test]
fn per_source_active_profile_paths() {
    let webcam = ProfileConfig::active_config_path_for(SourceKind::Webcam);
    let tobii = ProfileConfig::active_config_path_for(SourceKind::Tobii);
    assert_eq!(
        webcam,
        ProfileConfig::active_config_path(),
        "existing callers must not move"
    );
    assert!(webcam.ends_with("headtrack-rs/active.toml"));
    assert!(tobii.ends_with("headtrack-rs/active-tobii.toml"));
}

#[test]
fn tobii_status_defaults_to_no_device() {
    let s = TobiiStatus::default();
    assert_eq!(s.state, TobiiState::NoDevice);
    assert_eq!(s.gaze, None);
    assert_eq!(s.last_sample_age_ms, None);
}

// ── P08: per-source active profile ───────────────────────────────────────────

mod per_source_profile {
    use headtrack_core::config::ProfileConfig;
    use headtrack_core::SourceKind;

    fn scratch(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("htrs-p08-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn absent_files_resolve_to_each_sources_builtin() {
        let d = scratch("absent");
        let w = ProfileConfig::load_active_in(&d, SourceKind::Webcam).unwrap();
        let t = ProfileConfig::load_active_in(&d, SourceKind::Tobii).unwrap();
        assert_eq!(
            w.to_toml().unwrap(),
            ProfileConfig::default().to_toml().unwrap()
        );
        assert_eq!(
            t.to_toml().unwrap(),
            ProfileConfig::tobii_default().to_toml().unwrap()
        );
        let _ = std::fs::remove_dir_all(&d);
    }

    /// webcam → Tobii → webcam: each source reads its own file, and neither
    /// file changes by being loaded or by the other source being used.
    #[test]
    fn the_two_active_profiles_are_independent_and_round_trip() {
        let d = scratch("roundtrip");
        let mut webcam = ProfileConfig {
            name: "My webcam tuning".into(),
            ..Default::default()
        };
        webcam.deadzone[0] = 0.33;
        let mut tobii = ProfileConfig::tobii_default();
        tobii.name = "My Tobii tuning".into();
        tobii.curve[1].sensitivity = 0.7; // P18-style pitch gain cut
        let wpath = d.join("active.toml");
        let tpath = d.join("active-tobii.toml");
        webcam.save(&wpath).unwrap();
        tobii.save(&tpath).unwrap();
        let (wbytes, tbytes) = (
            std::fs::read(&wpath).unwrap(),
            std::fs::read(&tpath).unwrap(),
        );

        for kind in [SourceKind::Webcam, SourceKind::Tobii, SourceKind::Webcam] {
            let got = ProfileConfig::load_active_in(&d, kind).unwrap();
            let want = if kind == SourceKind::Webcam {
                &webcam
            } else {
                &tobii
            };
            assert_eq!(got.to_toml().unwrap(), want.to_toml().unwrap(), "{kind:?}");
        }
        assert_eq!(std::fs::read(&wpath).unwrap(), wbytes);
        assert_eq!(std::fs::read(&tpath).unwrap(), tbytes);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn an_unreadable_profile_is_an_error_not_a_silent_builtin() {
        let d = scratch("bad");
        std::fs::write(d.join("active-tobii.toml"), "name = [unterminated").unwrap();
        assert!(ProfileConfig::load_active_in(&d, SourceKind::Tobii).is_err());
        let _ = std::fs::remove_dir_all(&d);
    }
}
