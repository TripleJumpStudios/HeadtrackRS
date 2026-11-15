//! Headless render tests: every tab, at four window sizes, in the states that
//! change what it draws.
//!
//! Ported from HangarRS's `hangar-app/src/app.rs` (`no_tab_overflows_its_column`
//! and `no_prose_run_is_longer_than_a_sentence`). `egui::Context::run` drives
//! a real layout pass with no window and no GPU, so this is the equivalent of
//! clicking through every tab without opening anything on anyone's desktop.
//!
//! **Nothing here starts an engine or touches the user's machine.** The app is
//! built around a hand-made `EngineState` and a command sink that drops
//! everything (`CmdSink::Null`); `HeadtrackApp::draw` is called directly, so
//! window-size persistence and hotkeys never run; the camera tab is told its
//! device is already queried, so `/dev/video*` is never opened; and `HOME` and
//! `XDG_CONFIG_HOME` point at a scratch directory for the whole process, so the
//! per-frame `CameraConfig::load()` on the Camera tab can never read — or
//! quarantine — the live `camera.toml`.

use super::*;
use headtrack_core::input::{TobiiState, TobiiStatus};
use headtrack_engine::EngineState;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

/// The window sizes every tab is drawn at: the minimum the window allows, the
/// default first-launch size, a common laptop size, and full HD.
const SIZES: [(f32, f32); 4] = [
    CameraConfig::MIN_WINDOW,
    CameraConfig::DEFAULT_WINDOW,
    (1280.0, 800.0),
    (1920.0, 1080.0),
];

/// A window tall enough that every page is drawn whole. Text checks use it:
/// egui skips painting a widget outside the visible rect.
const TALL: (f32, f32) = (1280.0, 6000.0);

const ALL_TABS: [Tab; 8] = [
    Tab::Dashboard,
    Tab::Tuning,
    Tab::Profiles,
    Tab::Camera,
    Tab::Tobii,
    Tab::Keybinds,
    Tab::Settings,
    Tab::Advanced,
];

/// Serialises the tests that switch `HOME` between the empty and the
/// populated fixture homes.
static ENV_LOCK: Mutex<()> = Mutex::new(());

/// The scratch root, set once per test process.
static SCRATCH: OnceLock<PathBuf> = OnceLock::new();

/// Removes the scratch root when the test process exits. The harness has no
/// teardown hook and statics are never dropped, so every run used to leave a
/// `/tmp/ht-ui-render-<pid>` behind. `exit()` runs `atexit` handlers on the
/// normal path and on the harness's `process::exit(101)` after a failure.
extern "C" fn remove_scratch() {
    if let Some(root) = SCRATCH.get() {
        let _ = std::fs::remove_dir_all(root);
    }
}

extern "C" {
    fn atexit(cb: extern "C" fn()) -> std::ffi::c_int;
}

/// The scratch root, created once per test process and removed at exit.
fn scratch() -> &'static Path {
    SCRATCH.get_or_init(|| {
        // SAFETY: registers a plain `extern "C" fn` with no captured state.
        unsafe { atexit(remove_scratch) };
        let root = std::env::temp_dir().join(format!("ht-ui-render-{}", std::process::id()));
        for home in ["empty", "populated"] {
            std::fs::create_dir_all(root.join(home).join(".config")).expect("scratch home");
        }
        // The populated home has what auto-detection looks for: an MSFS 2024
        // Proton prefix, an X-Plane 12 folder with the plugin, and a Star
        // Citizen prefix with the bridge DLL.
        let pop = root.join("populated");
        std::fs::create_dir_all(pop.join(
            ".steam/steam/steamapps/compatdata/2537590/pfx/drive_c/users/steamuser/AppData/\
             Roaming/Microsoft Flight Simulator 2024",
        ))
        .expect("msfs prefix");
        let plugin = pop.join("X-Plane 12/Resources/plugins/headtrack/64");
        std::fs::create_dir_all(&plugin).expect("xplane plugin dir");
        std::fs::write(plugin.join("lin.xpl"), b"").expect("lin.xpl");
        let bin64 = sc_bin64_path(&pop.join("Games/star-citizen").display().to_string());
        std::fs::create_dir_all(&bin64).expect("bin64");
        std::fs::write(bin64.join("NPClient64.dll"), b"").expect("dll");
        root
    })
}

/// Point `HOME` and `XDG_CONFIG_HOME` at one of the scratch homes. Hold
/// `ENV_LOCK` across the call and the renders that depend on it.
fn use_home(which: &str) -> PathBuf {
    let home = scratch().join(which);
    std::env::set_var("HOME", &home);
    std::env::set_var("XDG_CONFIG_HOME", home.join(".config"));
    home
}

fn engine_state() -> Arc<EngineState> {
    Arc::new(EngineState {
        latest_pose: std::sync::Mutex::new(headtrack_core::Pose::zero(0)),
        fps: std::sync::Mutex::new(0.0),
        camera_connected: std::sync::atomic::AtomicBool::new(false),
        camera_index: std::sync::atomic::AtomicU32::new(0),
        head_source: std::sync::Mutex::new(Default::default()),
        tobii: Arc::new(std::sync::Mutex::new(TobiiStatus::default())),
        current_fov: std::sync::atomic::AtomicU32::new(78.0_f32.to_bits()),
        preview: Arc::new(std::sync::Mutex::new((0, None))),
        overlay_enabled: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        recording_status: std::sync::atomic::AtomicU8::new(0),
        recording_bytes: std::sync::atomic::AtomicU64::new(0),
        recording_error: std::sync::Mutex::new(None),
        msfs_seat: std::array::from_fn(|_| std::sync::atomic::AtomicU32::new(0)),
        msfs_title: std::sync::Mutex::new(String::new()),
        msfs_enabled: std::sync::atomic::AtomicBool::new(true),
        msfs_connected: std::sync::atomic::AtomicBool::new(false),
        global_shortcuts: std::sync::Mutex::new(None),
        controller_bindings: std::sync::Mutex::new(Vec::new()),
        controller_capture: std::sync::Mutex::new(None),
    })
}

/// The app with nothing found and nothing set up — a fresh install on a
/// machine with no sims, no controllers and no camera.
fn empty_app() -> HeadtrackApp {
    let profile = ProfileConfig::default();
    HeadtrackApp {
        engine: None,
        state: engine_state(),
        cmd: CmdSink::Null,
        active_tab: Tab::Dashboard,
        source_kind: SourceKind::Webcam,
        fake_tobii: None,
        shot_path: None,
        shot_frames: 0,
        tobii_setup: tobii_setup::TobiiSetup::default(),
        tuning_group: TuningGroup::Look,
        dev_selected_axis: 0,
        profile_name: profile.name.clone(),
        profile,
        axis_active: [true; 6],
        landmark_overlay: false,
        tracking_enabled: true,
        minimize_to_tray: true,
        show_advanced: true,
        dash_feed_h: 240.0,
        last_window_size: (0.0, 0.0),
        window_save_due: None,
        msfs_smoothing: 15.0,
        sc_smoothing: 0.0,
        recenter_flash: None,
        page_overflow: 0.0,
        rail_overlap: 0.0,
        hotkey_recenter: None,
        hotkey_toggle: None,
        hotkey_capture: None,
        hotkey_capture_started: std::time::Instant::now(),
        rebind_clicked_at: None,
        hotkey_capture_initial: std::collections::HashSet::new(),
        cameras: Vec::new(),
        selected_camera: 0,
        camera_fov: 78.0,
        last_engine_fov: 78.0_f32.to_bits(),
        camera_res_idx: CAMERA_RES_DEFAULT_IDX,
        fov_preset_name: String::new(),
        v4l2_device: None,
        v4l2_controls: Vec::new(),
        // Equal to `selected_camera`: the Camera tab believes the device was
        // already queried and never opens `/dev/video0`.
        v4l2_last_camera: 0,
        v4l2_optimize_status: None,
        profiles_list: Vec::new(),
        profile_status: String::new(),
        profile_dirty: false,
        profile_pending_delete: None,
        preview_texture: None,
        preview_last_seq: 0,
        logo_texture: None,
        github_texture: None,
        xplane_path: String::new(),
        xplane_install_status: None,
        sc_prefix_path: String::new(),
        sc_install_status: None,
        msfs_setup_status: None,
        sc_install_pending: None,
        ipc_socket_label: "/run/user/1000/headtrack.sock".into(),
        diag_size_limit_mb: 50,
        diag_recording_start: None,
        diag_recording_path: PathBuf::new(),
        diag_directory: String::new(),
    }
}

/// Everything found, everything set up, every status line showing, and every
/// value as long as it plausibly gets.
fn populated_app(home: &Path) -> HeadtrackApp {
    let mut app = empty_app();
    let s = &app.state;
    *s.fps.lock().unwrap() = 30.0;
    s.camera_connected.store(true, Ordering::Relaxed);
    s.msfs_connected.store(true, Ordering::Relaxed);
    *s.msfs_title.lock().unwrap() =
        "Asobo Cessna 172 Skyhawk G1000 Asobo Livery With A Very Long Registration".into();
    s.msfs_seat[0].store(0.04_f32.to_bits(), Ordering::Relaxed);
    s.msfs_seat[2].store((-0.12_f32).to_bits(), Ordering::Relaxed);
    let stick = "VKB-Sim (C) Alex Oz 2023 S-TECS MODERN THROTTLE MAX STEM";
    *s.controller_bindings.lock().unwrap() = vec![
        (
            "recenter".into(),
            "Recenter".into(),
            Some(format!("{stick} \u{00b7} BTN_TRIGGER_HAPPY18")),
        ),
        ("toggle".into(), "Toggle tracking".into(), None),
        (
            "msfs-seat-forward".into(),
            "MSFS seat forward".into(),
            Some(format!("{stick} \u{00b7} BTN_BASE4")),
        ),
        (
            "msfs-enabled".into(),
            "Release the MSFS camera".into(),
            None,
        ),
    ];
    *s.controller_capture.lock().unwrap() = Some("toggle".into());
    *s.global_shortcuts.lock().unwrap() = Some(vec![
        ("Recenter head tracking".into(), "Ctrl+Alt+Shift+F12".into()),
        ("Toggle tracking".into(), String::new()),
    ]);
    s.recording_status.store(1, Ordering::Relaxed);
    // A synthetic 4:3 frame — flat grey, never a picture of anyone — so the
    // preview's texture and aspect path is drawn, not only its placeholder.
    *s.preview.lock().unwrap() = (1, Some(Arc::new(grey_jpeg(64, 48))));
    s.recording_bytes.store(12 * 1024 * 1024, Ordering::Relaxed);

    app.cameras = vec![CameraDevice {
        index: 0,
        name: "Logitech BRIO 4K Stream Edition with an unusually long V4L2 card name".into(),
    }];
    app.tracking_enabled = false;
    app.recenter_flash = Some(std::time::Instant::now());
    app.profile_name =
        "MSFS TRANSLATION v7 (v6 + pitch_to_yaw 0.25 -> 0.05) for the long evenings".into();
    app.profile_dirty = true;
    app.profile_status = format!(
        "Saved to {}",
        home.join(".config/headtrack-rs/profiles/MSFS_TRANSLATION_v7.toml")
            .display()
    );
    app.profiles_list = vec![
        (
            app.profile_name.clone(),
            home.join(".config/headtrack-rs/profiles/a.toml"),
        ),
        (
            "X-Plane 12 evening".into(),
            home.join(".config/headtrack-rs/profiles/b.toml"),
        ),
        (
            "Star Citizen dogfight, slew-limited pitch".into(),
            home.join(".config/headtrack-rs/profiles/c.toml"),
        ),
    ];
    app.profile_pending_delete = Some(home.join(".config/headtrack-rs/profiles/b.toml"));
    app.hotkey_recenter = Some(egui::Key::F9);
    app.hotkey_capture = Some(HotkeyAction::Toggle);
    app.fov_preset_name = "Brio wide".into();
    app.v4l2_optimize_status = Some((tabs::camera::optimise_message(2), std::time::Instant::now()));
    app.xplane_path = home.join("X-Plane 12").display().to_string();
    app.xplane_install_status = Some((
        "Installed lin.xpl in the X-Plane 12 folder, under Resources/plugins/headtrack/64.".into(),
        true,
    ));
    app.sc_prefix_path = home.join("Games/star-citizen").display().to_string();
    app.sc_install_status = Some((
        "NPClient64.dll installed and registry key set. Enable TrackIR in the Star Citizen \
         settings."
            .into(),
        true,
    ));
    let (_tx, rx) = std::sync::mpsc::channel();
    app.sc_install_pending = Some(rx);
    app.msfs_setup_status = Some((
        "Added port 5557. Original saved as SimConnect.xml.headtrack-backup-1792000000. \
         Restart MSFS."
            .into(),
        true,
    ));
    app.diag_directory = home
        .join("Documents/HeadtrackRS diagnostics/a folder with a long name")
        .display()
        .to_string();
    app.diag_recording_start = Some(std::time::Instant::now());
    app.diag_recording_path =
        PathBuf::from(&app.diag_directory).join("headtrack_record_1792000000.csv");

    // FOV presets come from `camera.toml` every frame — the scratch one.
    let cfg = CameraConfig {
        fov_presets: vec![
            headtrack_core::config::FovPreset {
                name: "Brio wide".into(),
                fov: 90.0,
            },
            headtrack_core::config::FovPreset {
                name: "C920".into(),
                fov: 78.0,
            },
            headtrack_core::config::FovPreset {
                name: "Kiyo Pro narrow".into(),
                fov: 65.0,
            },
        ],
        ..CameraConfig::default()
    };
    cfg.save().expect("scratch camera.toml");
    app
}

/// A flat grey JPEG of `w` × `h`.
fn grey_jpeg(w: u32, h: u32) -> Vec<u8> {
    let img = image::RgbImage::from_pixel(w, h, image::Rgb([60, 60, 60]));
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Jpeg)
        .expect("encode jpeg");
    out.into_inner()
}

/// The populated app as the Tobii sees it, in `state`.
fn tobii_app(home: &Path, state: TobiiState) -> HeadtrackApp {
    tobii_app_with(home, tabs::tobii::fake_status(state))
}

/// The populated app with the Tobii as the source, showing `status`.
fn tobii_app_with(home: &Path, status: TobiiStatus) -> HeadtrackApp {
    let mut app = populated_app(home);
    app.fake_tobii = Some(status);
    app.source_kind = SourceKind::Tobii;
    app
}

/// Draw `frames` frames of the app at `size` and return the last one.
///
/// Several frames because the Dashboard converges its feed height against the
/// previous frame's row, so the first frame is not the steady state.
fn render(app: &mut HeadtrackApp, size: (f32, f32)) -> egui::FullOutput {
    let ctx = egui::Context::default();
    ui::theme::install(&ctx);
    let input = || egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(size.0, size.1),
        )),
        ..Default::default()
    };
    for _ in 0..3 {
        let _ = ctx.run(input(), |ctx| app.draw(ctx));
    }
    ctx.run(input(), |ctx| app.draw(ctx))
}

/// Every page each fixture can show: each tab, both Tuning groups, a rotation
/// and a translation axis in the Advanced editor, and every recorder state.
fn each_page(app: &mut HeadtrackApp, mut check: impl FnMut(&mut HeadtrackApp, &str)) {
    for tab in ALL_TABS {
        app.active_tab = tab;
        match tab {
            Tab::Tuning => {
                for group in [TuningGroup::Look, TuningGroup::Move] {
                    app.tuning_group = group;
                    check(
                        app,
                        if group == TuningGroup::Look {
                            "Tuning/Look"
                        } else {
                            "Tuning/Move"
                        },
                    );
                }
            }
            Tab::Advanced => {
                let recording = app.state.recording_status.load(Ordering::Relaxed);
                for (axis, status) in [(0usize, recording), (5, 2), (2, 3), (4, 0)] {
                    app.dev_selected_axis = axis;
                    app.state.recording_status.store(status, Ordering::Relaxed);
                    if status == 3 {
                        *app.state.recording_error.lock().unwrap() = Some(
                            "Permission denied (os error 13) opening /home/user/Documents/x.csv"
                                .into(),
                        );
                    }
                    app.profile.center.recenter_range =
                        (axis % 2 == 0).then_some([12.0, 12.0, 4.0, 10.0, 10.0, 60.0]);
                    check(app, &format!("Advanced/axis{axis}/rec{status}"));
                }
                app.state
                    .recording_status
                    .store(recording, Ordering::Relaxed);
            }
            _ => check(app, tab.label()),
        }
    }
}

/// No tab may render wider than the column it is given.
///
/// A tab can render without panicking and still run off the right edge,
/// because egui clips the overflow and a headless render cannot see what was
/// clipped. `ui::page` measures the content, the app records it in
/// `page_overflow`, and this asserts it is zero — every tab, at four sizes,
/// empty and populated, and the Tobii tab in every device state.
#[test]
fn no_tab_overflows_its_column() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut failures = Vec::new();

    let mut fixtures: Vec<(String, HeadtrackApp)> = Vec::new();
    use_home("empty");
    fixtures.push(("empty".into(), empty_app()));
    let home = use_home("populated");
    fixtures.push(("populated".into(), populated_app(&home)));
    for state in [
        TobiiState::NoDevice,
        TobiiState::PermissionDenied,
        TobiiState::Busy,
        TobiiState::Opening,
        TobiiState::Streaming,
        TobiiState::NoFace,
        TobiiState::Lost,
    ] {
        fixtures.push((format!("tobii {state:?}"), tobii_app(&home, state)));
    }
    fixtures.push((
        "tobii neutral".into(),
        tobii_app_with(&home, tabs::tobii::fake_neutral_status(0.4)),
    ));

    for (name, mut app) in fixtures {
        // The empty fixture must see the empty home: no MSFS prefix.
        use_home(if name == "empty" {
            "empty"
        } else {
            "populated"
        });
        for size in SIZES {
            each_page(&mut app, |app, page| {
                render(app, size);
                if app.page_overflow > 0.0 {
                    failures.push(format!(
                        "{name}: {page} overflows its column by {:.1} px at {}x{}",
                        app.page_overflow, size.0, size.1
                    ));
                }
            });
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The rail footer is laid out bottom-up and the nav list top-down; nothing
/// but this test keeps them apart. At the minimum height, with Advanced shown,
/// the footer used to be drawn over the `Advanced` row (1a).
#[test]
fn the_rail_footer_never_overlaps_the_nav_list() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut failures = Vec::new();
    for (name, mut app) in [("empty", empty_app()), ("populated", populated_app(&home))] {
        assert!(
            app.show_advanced,
            "the fixture must draw the longest nav list"
        );
        // Every 10 px from the minimum up past the compact-brand threshold,
        // then the standard sizes.
        let sweep = (0..=16).map(|i| {
            (
                CameraConfig::MIN_WINDOW.0,
                CameraConfig::MIN_WINDOW.1 + 10.0 * i as f32,
            )
        });
        for size in sweep.chain(SIZES) {
            render(&mut app, size);
            if app.rail_overlap > 0.0 {
                failures.push(format!(
                    "{name}: footer overlaps the nav list by {:.1} px at {}x{}",
                    app.rail_overlap, size.0, size.1
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The status bar and the Dashboard take their dot from `source_health`; for
/// each Tobii state it must be the tone the Tobii page header uses. They used
/// to be red for the amber states (7d).
#[test]
fn the_status_bar_tone_matches_the_tobii_page_for_every_state() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    for state in [
        TobiiState::NoDevice,
        TobiiState::PermissionDenied,
        TobiiState::Busy,
        TobiiState::Opening,
        TobiiState::Streaming,
        TobiiState::NoFace,
        TobiiState::Lost,
    ] {
        let app = tobii_app(&home, state);
        match app.source_health(true) {
            Ok(()) => assert_eq!(state, TobiiState::Streaming, "{state:?} reported healthy"),
            Err((_, tone)) => assert_eq!(tone, tabs::tobii::state_tone(state), "{state:?}"),
        }
    }
    // Streaming but still taking the neutral is not tracking yet: amber
    // "hold still" on the status bar, as on the page header.
    let app = tobii_app_with(&home, tabs::tobii::fake_neutral_status(0.4));
    let status = app.tobii_status();
    assert_eq!(
        app.source_health(true),
        Err((
            "Tobii: hold still".into(),
            tabs::tobii::status_label(&status).1
        ))
    );
}

/// Q-UI6b: the preset a fresh install lands on is the core default, 1280×720,
/// and it is the only one called recommended.
#[test]
fn the_recommended_resolution_is_the_default_one() {
    assert_eq!(
        (
            CAMERA_RESOLUTIONS[CAMERA_RES_DEFAULT_IDX].0,
            CAMERA_RESOLUTIONS[CAMERA_RES_DEFAULT_IDX].1
        ),
        CameraConfig::default_resolution()
    );
    assert_eq!(CameraConfig::default_resolution(), (1280, 720));
    let recommended: Vec<_> = CAMERA_RESOLUTIONS
        .iter()
        .filter(|(_, _, l)| l.contains("(recommended)"))
        .collect();
    assert_eq!(recommended.len(), 1);
    assert_eq!((recommended[0].0, recommended[0].1), (1280, 720));
}

/// D1/D2: until the Tobii tracks, the Dashboard feed and the Tobii tab's head
/// box say what to do, in words big enough to see: starting, then hold still
/// with a countdown, or face the tracker. Once it tracks, the prompt is gone.
#[test]
fn the_tobii_feed_tells_the_user_what_to_do_until_it_tracks() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut no_face = tabs::tobii::fake_status(TobiiState::NoFace);
    no_face.neutral_progress = Some(0.0);
    let cases: [(&str, TobiiStatus, Option<&str>, Option<&str>); 4] = [
        (
            "opening",
            tabs::tobii::fake_status(TobiiState::Opening),
            Some("Starting the tracker"),
            None,
        ),
        (
            "neutral",
            tabs::tobii::fake_neutral_status(0.25),
            Some("Hold still"),
            Some("1.5"),
        ),
        ("no face", no_face, Some("Face the tracker"), None),
        (
            "tracking",
            tabs::tobii::fake_status(TobiiState::Streaming),
            None,
            None,
        ),
    ];
    for (name, status, headline, countdown) in cases {
        let mut app = tobii_app_with(&home, status);
        for tab in [Tab::Dashboard, Tab::Tobii] {
            app.active_tab = tab;
            for size in [CameraConfig::MIN_WINDOW, CameraConfig::DEFAULT_WINDOW] {
                let texts = all_texts(&render(&mut app, size));
                let where_ = format!("{name} {} {size:?}", tab.label());
                let prompts = ["Starting the tracker", "Hold still", "Face the tracker"];
                for p in prompts {
                    let shown = texts.iter().any(|t| t == p);
                    assert_eq!(
                        shown,
                        headline == Some(p),
                        "{where_}: {p:?} shown = {shown}"
                    );
                }
                if let Some(c) = countdown {
                    assert!(
                        texts.iter().any(|t| t == c),
                        "{where_}: countdown {c} missing"
                    );
                }
                if headline.is_some() && tab == Tab::Tobii {
                    assert!(
                        !texts.iter().any(|t| t == "No head pose"),
                        "{where_}: placeholder under the prompt"
                    );
                }
            }
        }
    }
}

/// Left x of the first painted text run equal to `want`, if any.
fn text_x(out: &egui::FullOutput, want: &str) -> Option<f32> {
    fn walk(shape: &egui::Shape, want: &str) -> Option<f32> {
        match shape {
            egui::Shape::Text(t) if t.galley.job.text == want => Some(t.pos.x),
            egui::Shape::Vec(v) => v.iter().find_map(|s| walk(s, want)),
            _ => None,
        }
    }
    out.shapes.iter().find_map(|cs| walk(&cs.shape, want))
}

/// The three binding grids on Keybinds share one label column, so a
/// controller binding, an in-app hotkey and a keyboard shortcut all start at
/// the same x (8b). The keyboard value is a bare label, not a button, so it
/// sits one button padding to the left. Since L6 (2026-11-02) the desktop
/// shortcuts sit in a disclosure under the in-app hotkeys, indented, and their
/// label column is narrowed by the indent to keep this true.
#[test]
fn the_keybind_grids_share_their_label_column() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut app = populated_app(&home);
    app.active_tab = Tab::Keybinds;
    for size in [CameraConfig::MIN_WINDOW, CameraConfig::DEFAULT_WINDOW, TALL] {
        let out = render(&mut app, size);
        let pad = {
            let ctx = egui::Context::default();
            ui::theme::install(&ctx);
            ctx.style().spacing.button_padding.x
        };
        let ctrl = text_x(&out, "BTN_TRIGGER_HAPPY18").expect("controller binding drawn");
        let Some(hotkey) = text_x(&out, "F9") else {
            continue;
        }; // below the fold
        assert!(
            (ctrl - hotkey).abs() < 1.0,
            "{size:?}: controller {ctrl:.1} vs in-app {hotkey:.1}"
        );
        if let Some(kbd) = text_x(&out, "Ctrl+Alt+Shift+F12") {
            assert!(
                (ctrl - pad - kbd).abs() < 1.0,
                "{size:?}: controller {ctrl:.1} vs keyboard {kbd:.1}"
            );
        }
    }
}

/// Gaze is hidden for v0.2.0 (L4, deferred): the Tobii tab draws no Gaze
/// card at any state, and the Head card is still there, so the row did not
/// just lose its right half.
#[test]
fn the_tobii_tab_has_no_gaze_card_for_now() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    for state in [
        TobiiState::Opening,
        TobiiState::Streaming,
        TobiiState::NoFace,
    ] {
        let mut app = tobii_app(&home, state);
        app.active_tab = Tab::Tobii;
        let texts = all_texts(&render(&mut app, TALL));
        assert!(texts.iter().any(|t| t == "Head"), "{state:?}: no Head card");
        for gone in ["Gaze", "No gaze"] {
            assert!(
                !texts.iter().any(|t| t == gone),
                "{state:?}: {gone:?} is drawn"
            );
        }
    }
}

/// Every painted text run, in paint order.
fn all_texts(out: &egui::FullOutput) -> Vec<String> {
    fn walk(shape: &egui::Shape, out: &mut Vec<String>) {
        match shape {
            egui::Shape::Text(t) => out.push(t.galley.job.text.clone()),
            egui::Shape::Vec(v) => v.iter().for_each(|s| walk(s, out)),
            _ => {}
        }
    }
    let mut all = Vec::new();
    for cs in &out.shapes {
        walk(&cs.shape, &mut all);
    }
    all
}

/// The rail shows the two-tone wordmark on one line at every height, and no
/// tagline (maintainer, 2026-11-02: "ditch the subtext below the icon"). The
/// compact block once wrapped its tagline to "6 DOF · LI / NUX".
#[test]
fn the_rail_shows_the_wordmark_on_one_line_and_no_tagline() {
    fn walk(shape: &egui::Shape, marks: &mut Vec<(usize, Vec<egui::Color32>)>, tagline: &mut bool) {
        match shape {
            egui::Shape::Text(t) => {
                let text = &t.galley.job.text;
                if text == "HeadtrackRS" {
                    let tones = t
                        .galley
                        .job
                        .sections
                        .iter()
                        .map(|s| s.format.color)
                        .collect();
                    marks.push((t.galley.rows.len(), tones));
                }
                // `track_out` letter-spaces with U+2009.
                if text
                    .chars()
                    .filter(|c| !c.is_whitespace())
                    .collect::<String>()
                    == "6DOF·LINUX"
                {
                    *tagline = true;
                }
            }
            egui::Shape::Vec(v) => v.iter().for_each(|s| walk(s, marks, tagline)),
            _ => {}
        }
    }
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = empty_app();
    for h in (620..=780).step_by(20) {
        let out = render(&mut app, (CameraConfig::MIN_WINDOW.0, h as f32));
        let (mut marks, mut tagline) = (Vec::new(), false);
        for cs in &out.shapes {
            walk(&cs.shape, &mut marks, &mut tagline);
        }
        assert!(!tagline, "height {h}: the tagline is back");
        assert_eq!(marks.len(), 1, "height {h}: wordmarks drawn {marks:?}");
        let (rows, tones) = &marks[0];
        assert_eq!(*rows, 1, "height {h}: the wordmark wrapped");
        assert_eq!(
            tones,
            &[colors::TEXT_HI, colors::ORANGE],
            "height {h}: Headtrack + RS in the accent"
        );
    }
}

/// The Live pose gauges are visible, not just painted: each label's clip rect
/// must contain it. A column clipped to a zero-height slot painted every
/// gauge into a clip of nothing, and text-only checks still found them.
#[test]
fn the_live_pose_gauges_are_not_clipped_away() {
    fn visible(out: &egui::FullOutput, want: &str) -> bool {
        fn walk(shape: &egui::Shape, clip: egui::Rect, want: &str) -> bool {
            match shape {
                egui::Shape::Text(t) if t.galley.job.text == want => {
                    clip.intersects(t.visual_bounding_rect())
                }
                egui::Shape::Vec(v) => v.iter().any(|s| walk(s, clip, want)),
                _ => false,
            }
        }
        out.shapes
            .iter()
            .any(|cs| walk(&cs.shape, cs.clip_rect, want))
    }
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    for mut app in [
        populated_app(&home),
        tobii_app(&home, TobiiState::Streaming),
    ] {
        app.active_tab = Tab::Dashboard;
        let out = render(&mut app, TALL);
        for want in ["Rotation", "Translation", AXIS_LABELS[0], AXIS_LABELS[5]] {
            assert!(
                visible(&out, want),
                "{want:?} is clipped away on the Dashboard"
            );
        }
    }
}

/// While the Tobii streams, the Dashboard feed box and the Tobii page both
/// show the head marker, captioned with the live rotation; in any other state
/// neither shows a pose, because a held one would look like a working
/// tracker (3c).
#[test]
fn the_tobii_head_marker_shows_the_live_pose_only_while_streaming() {
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let caption = "Yaw +12°  Pitch -5°  Roll +3°";
    for state in [TobiiState::Streaming, TobiiState::NoFace, TobiiState::Lost] {
        let mut app = tobii_app(&home, state);
        {
            let mut pose = app.state.latest_pose.lock().unwrap();
            (pose.yaw, pose.pitch, pose.roll) = (12.0, -5.0, 3.0);
        }
        for tab in [Tab::Dashboard, Tab::Tobii] {
            app.active_tab = tab;
            for size in [CameraConfig::MIN_WINDOW, CameraConfig::DEFAULT_WINDOW] {
                let texts = all_texts(&render(&mut app, size));
                let shown = texts.iter().any(|t| t == caption);
                assert_eq!(
                    shown,
                    state == TobiiState::Streaming,
                    "{state:?} {} {size:?}: marker shown = {shown}",
                    tab.label()
                );
                assert!(
                    !texts
                        .iter()
                        .any(|t| t.starts_with("Yaw ") && t.contains("Z ")),
                    "{}: Tobii has no Z",
                    tab.label()
                );
                if tab == Tab::Tobii && state != TobiiState::Streaming {
                    assert!(
                        texts.iter().any(|t| t == "No head pose"),
                        "{state:?} {size:?}"
                    );
                }
            }
        }
    }
}

/// An unfocused path field too narrow for its path shows the path's end,
/// like every read-only path row, not its start (9a).
#[test]
fn a_long_diagnostics_folder_shows_its_end() {
    fn texts(shape: &egui::Shape, out: &mut Vec<String>) {
        match shape {
            egui::Shape::Text(t) => out.push(t.galley.job.text.clone()),
            egui::Shape::Vec(v) => v.iter().for_each(|s| texts(s, out)),
            _ => {}
        }
    }
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut app = populated_app(&home);
    app.active_tab = Tab::Settings;
    let out = render(&mut app, CameraConfig::MIN_WINDOW);
    let mut all = Vec::new();
    for cs in &out.shapes {
        texts(&cs.shape, &mut all);
    }
    assert!(
        all.iter()
            .any(|t| t.starts_with('\u{2026}') && t.ends_with("a folder with a long name")),
        "no left-elided diagnostics folder among {:?}",
        all.iter()
            .filter(|t| t.contains("diagnostics") || t.contains("long name"))
            .collect::<Vec<_>>()
    );
}

/// A page body may hold a label, a value, or one sentence of consequence;
/// anything longer is prose and belongs in documentation.
///
/// Scoped to the proportional family, because data — paths, commands, device
/// messages — is monospace. That exempts every path row without an allowlist
/// anyone has to maintain.
#[test]
fn no_prose_run_is_longer_than_a_sentence() {
    /// One sentence. The limit the page body is held to.
    const MAX_CHARS: usize = 160;

    fn collect(shape: &egui::Shape, out: &mut Vec<String>) {
        match shape {
            egui::Shape::Text(t) => {
                let job = &t.galley.job;
                for section in &job.sections {
                    if section.format.font_id.family != egui::FontFamily::Proportional {
                        continue;
                    }
                    let run = &job.text[section.byte_range.clone()];
                    if run.chars().count() > MAX_CHARS {
                        out.push(run.to_owned());
                    }
                }
            }
            egui::Shape::Vec(v) => v.iter().for_each(|s| collect(s, out)),
            _ => {}
        }
    }

    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut offenders: Vec<(String, String)> = Vec::new();

    let mut fixtures: Vec<(&str, HeadtrackApp)> = Vec::new();
    use_home("empty");
    fixtures.push(("empty", empty_app()));
    let home = use_home("populated");
    fixtures.push(("populated", populated_app(&home)));
    for state in [
        TobiiState::PermissionDenied,
        TobiiState::Busy,
        TobiiState::Lost,
    ] {
        fixtures.push(("tobii", tobii_app(&home, state)));
    }

    for (name, mut app) in fixtures {
        use_home(if name == "empty" {
            "empty"
        } else {
            "populated"
        });
        // Tall, so the whole page is on screen: egui does not paint a label
        // below the fold, and an unpainted label has no text to check.
        each_page(&mut app, |app, page| {
            let out = render(app, TALL);
            let mut long = Vec::new();
            for cs in &out.shapes {
                collect(&cs.shape, &mut long);
            }
            for run in long {
                if !offenders.iter().any(|(_, seen)| *seen == run) {
                    offenders.push((format!("{name}: {page}"), run));
                }
            }
        });
    }

    assert!(
        offenders.is_empty(),
        "prose runs over {MAX_CHARS} characters:\n{}",
        offenders
            .iter()
            .map(|(page, run)| format!("  [{page}] {run}"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// The fixtures really do reach the states they claim to: a render test that
/// only ever drew the empty branch would pass while the populated layout broke.
#[test]
fn the_populated_fixture_draws_its_populated_branches() {
    fn text_of(out: &egui::FullOutput) -> String {
        fn collect(shape: &egui::Shape, text: &mut String) {
            match shape {
                egui::Shape::Text(t) => {
                    text.push_str(&t.galley.job.text);
                    text.push('\n');
                }
                egui::Shape::Vec(v) => v.iter().for_each(|s| collect(s, text)),
                _ => {}
            }
        }
        let mut text = String::new();
        for cs in &out.shapes {
            collect(&cs.shape, &mut text);
        }
        text
    }

    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut app = populated_app(&home);

    app.active_tab = Tab::Settings;
    let settings = text_of(&render(&mut app, TALL));
    for want in [
        "Installed",
        "Prefix folder",
        "Bin64",
        "Configuring the registry",
    ] {
        assert!(settings.contains(want), "Settings is missing {want:?}");
    }
    assert!(
        settings.contains("Port 5557 set") || settings.contains("Needs port 5557"),
        "Settings did not find the scratch MSFS prefix"
    );

    app.active_tab = Tab::Profiles;
    let profiles = text_of(&render(&mut app, TALL));
    for want in [
        "Delete this profile?",
        "Active",
        "Unsaved changes",
        "Planned",
    ] {
        assert!(profiles.contains(want), "Profiles is missing {want:?}");
    }
    assert!(
        !profiles.contains("Phase 4"),
        "Profiles shows the internal roadmap label"
    );

    app.active_tab = Tab::Keybinds;
    let keybinds = text_of(&render(&mut app, TALL));
    for want in [
        "headtrack-rs/seat_reset",
        "msfs-seat reset",
        "BTN_TRIGGER_HAPPY18",
    ] {
        assert!(keybinds.contains(want), "Keybinds is missing {want:?}");
    }

    app.active_tab = Tab::Camera;
    let camera = text_of(&render(&mut app, TALL));
    assert!(
        camera.contains("Kiyo Pro narrow"),
        "the scratch camera.toml presets were not drawn"
    );

    use_home("empty");
    let mut empty = empty_app();
    empty.active_tab = Tab::Settings;
    let settings = text_of(&render(&mut empty, TALL));
    assert!(
        settings.contains("MSFS 2024 not found"),
        "the empty home found an MSFS prefix"
    );
}

/// The source-level half of the rules: no raw buttons or checkboxes in the
/// tabs, no size that is not one of the five, no uppercased text.
#[test]
fn the_tabs_use_the_design_system_and_the_type_scale() {
    for (file, src) in [
        ("advanced.rs", include_str!("tabs/advanced.rs")),
        ("camera.rs", include_str!("tabs/camera.rs")),
        ("dashboard.rs", include_str!("tabs/dashboard.rs")),
        ("keybinds.rs", include_str!("tabs/keybinds.rs")),
        ("profiles.rs", include_str!("tabs/profiles.rs")),
        ("settings.rs", include_str!("tabs/settings.rs")),
        ("tobii.rs", include_str!("tabs/tobii.rs")),
        ("tuning.rs", include_str!("tabs/tuning.rs")),
    ] {
        for (n, line) in src.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("");
            for banned in [
                "ui.button(",
                "egui::Button::new",
                "small_button(",
                ".checkbox(",
                "to_uppercase(",
                "track_out(",
                "egui::Frame::new()",
            ] {
                assert!(
                    !code.contains(banned),
                    "{file}:{}: `{banned}` in {line:?}",
                    n + 1
                );
            }
            for literal in [
                ".size(1",
                ".size(2",
                "FontId::new(1",
                "FontId::new(2",
                "proportional(1",
                "monospace(1",
            ] {
                assert!(
                    !code.contains(literal),
                    "{file}:{}: a numeric text size — use a theme::TEXT_* token: {line:?}",
                    n + 1
                );
            }
        }
    }
}

// ── Offline PNGs for review by eye ──────────────────────────────────────────

/// Draws `app` at `size` and rasterises the frame on the CPU at
/// `pixels_per_point`: egui's own tessellation, every textured triangle filled
/// with nearest-texel sampling and premultiplied "over" blending. No window,
/// no GPU, and only fixture state, so no camera frame can ever appear.
/// Edges stay smooth because egui feathers them with alpha vertices.
fn rasterize(app: &mut HeadtrackApp, size: (f32, f32), pixels_per_point: f32) -> image::RgbaImage {
    use egui::epaint::{ImageData, Primitive};
    use std::collections::HashMap;

    let ctx = egui::Context::default();
    ui::theme::install(&ctx);
    ctx.set_pixels_per_point(pixels_per_point);
    let input = || egui::RawInput {
        screen_rect: Some(egui::Rect::from_min_size(
            egui::Pos2::ZERO,
            egui::vec2(size.0, size.1),
        )),
        ..Default::default()
    };
    // Texture id → (width, height, premultiplied texels), kept across frames
    // because the font atlas arrives whole once and then in patches.
    let mut textures: HashMap<egui::TextureId, (usize, usize, Vec<egui::Color32>)> = HashMap::new();
    let mut last = None;
    for _ in 0..4 {
        let out = ctx.run(input(), |ctx| app.draw(ctx));
        for (id, delta) in &out.textures_delta.set {
            let (w, h, px): (usize, usize, Vec<egui::Color32>) = match &delta.image {
                ImageData::Color(img) => (img.size[0], img.size[1], img.pixels.clone()),
                ImageData::Font(f) => (f.size[0], f.size[1], f.srgba_pixels(None).collect()),
            };
            match delta.pos {
                None => {
                    textures.insert(*id, (w, h, px));
                }
                Some([x0, y0]) => {
                    let t = textures.get_mut(id).expect("patch for a known texture");
                    for y in 0..h {
                        for x in 0..w {
                            t.2[(y0 + y) * t.0 + x0 + x] = px[y * w + x];
                        }
                    }
                }
            }
        }
        last = Some(out);
    }
    let out = last.expect("one frame");
    let ppp = out.pixels_per_point;
    let prims = ctx.tessellate(out.shapes, ppp);
    let (w, h) = (
        (size.0 * ppp).round() as usize,
        (size.1 * ppp).round() as usize,
    );
    let mut buf = vec![[0.0_f32; 4]; w * h];
    let rgba = |c: egui::Color32| c.to_array().map(|v| v as f32 / 255.0);

    for prim in prims {
        let Primitive::Mesh(mesh) = prim.primitive else {
            continue;
        };
        let clip = prim.clip_rect;
        let (cx0, cy0) = (
            (clip.min.x * ppp).max(0.0) as usize,
            (clip.min.y * ppp).max(0.0) as usize,
        );
        let (cx1, cy1) = (
            ((clip.max.x * ppp).ceil() as usize).min(w),
            ((clip.max.y * ppp).ceil() as usize).min(h),
        );
        let tex = textures.get(&mesh.texture_id);
        for tri in mesh.indices.as_chunks::<3>().0 {
            let v = [0, 1, 2].map(|i| &mesh.vertices[tri[i] as usize]);
            let p = v.map(|v| (v.pos.x * ppp, v.pos.y * ppp));
            let area =
                (p[1].0 - p[0].0) * (p[2].1 - p[0].1) - (p[1].1 - p[0].1) * (p[2].0 - p[0].0);
            if area.abs() < 1e-6 {
                continue;
            }
            let xs = [p[0].0, p[1].0, p[2].0];
            let ys = [p[0].1, p[1].1, p[2].1];
            let bx0 =
                (xs.iter().cloned().fold(f32::MAX, f32::min).floor().max(0.0) as usize).max(cx0);
            let by0 =
                (ys.iter().cloned().fold(f32::MAX, f32::min).floor().max(0.0) as usize).max(cy0);
            let bx1 =
                (xs.iter().cloned().fold(f32::MIN, f32::max).ceil().max(0.0) as usize).min(cx1);
            let by1 =
                (ys.iter().cloned().fold(f32::MIN, f32::max).ceil().max(0.0) as usize).min(cy1);
            let cols = v.map(|v| rgba(v.color));
            for y in by0..by1 {
                for x in bx0..bx1 {
                    let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                    let e = |a: (f32, f32), b: (f32, f32)| {
                        (b.0 - a.0) * (py - a.1) - (b.1 - a.1) * (px - a.0)
                    };
                    let l = [
                        e(p[1], p[2]) / area,
                        e(p[2], p[0]) / area,
                        e(p[0], p[1]) / area,
                    ];
                    if l.iter().any(|&b| b < 0.0) {
                        continue;
                    }
                    let u = l[0] * v[0].uv.x + l[1] * v[1].uv.x + l[2] * v[2].uv.x;
                    let t = l[0] * v[0].uv.y + l[1] * v[1].uv.y + l[2] * v[2].uv.y;
                    let texel = tex.map_or([1.0; 4], |(tw, th, px)| {
                        let tx = ((u * *tw as f32) as usize).min(tw - 1);
                        let ty = ((t * *th as f32) as usize).min(th - 1);
                        rgba(px[ty * tw + tx])
                    });
                    let src: [f32; 4] = std::array::from_fn(|i| {
                        (l[0] * cols[0][i] + l[1] * cols[1][i] + l[2] * cols[2][i]) * texel[i]
                    });
                    let dst = &mut buf[y * w + x];
                    for i in 0..4 {
                        dst[i] = src[i] + dst[i] * (1.0 - src[3]);
                    }
                }
            }
        }
    }
    image::RgbaImage::from_fn(w as u32, h as u32, |x, y| {
        let c = buf[y as usize * w + x as usize];
        image::Rgba(
            [0, 1, 2]
                .map(|i| (c[i].clamp(0.0, 1.0) * 255.0).round() as u8)
                .into_iter()
                .chain([255])
                .collect::<Vec<_>>()
                .try_into()
                .unwrap(),
        )
    })
}

/// Writes PNGs of the Tobii hold-still prompt states (D1/D2), plus the Keybinds tab, for review, into
/// `$HEADTRACK_UI_PNG_DIR`. Ignored: it is a tool, not a check. Run with
/// `HEADTRACK_UI_PNG_DIR=/tmp/x cargo test -p headtrack-gui -- --ignored render_review_pngs`.
#[test]
#[ignore = "writes review PNGs; run by hand with HEADTRACK_UI_PNG_DIR"]
fn render_review_pngs() {
    let dir =
        PathBuf::from(std::env::var("HEADTRACK_UI_PNG_DIR").expect("set HEADTRACK_UI_PNG_DIR"));
    std::fs::create_dir_all(&dir).expect("png dir");
    let _env = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let home = use_home("populated");
    let mut no_face = tabs::tobii::fake_status(TobiiState::NoFace);
    no_face.neutral_progress = Some(0.0);
    let shots: [(&str, TobiiStatus, Tab, (f32, f32)); 9] = [
        (
            "1-dashboard-starting",
            tabs::tobii::fake_status(TobiiState::Opening),
            Tab::Dashboard,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "2-dashboard-hold-still-40",
            tabs::tobii::fake_neutral_status(0.4),
            Tab::Dashboard,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "3-dashboard-hold-still-85",
            tabs::tobii::fake_neutral_status(0.85),
            Tab::Dashboard,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "4-dashboard-face-the-tracker",
            no_face.clone(),
            Tab::Dashboard,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "5-dashboard-tracking",
            tabs::tobii::fake_status(TobiiState::Streaming),
            Tab::Dashboard,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "6-dashboard-hold-still-min-size",
            tabs::tobii::fake_neutral_status(0.4),
            Tab::Dashboard,
            CameraConfig::MIN_WINDOW,
        ),
        (
            "7-tobii-tab-hold-still",
            tabs::tobii::fake_neutral_status(0.4),
            Tab::Tobii,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "8-tobii-tab-starting",
            tabs::tobii::fake_status(TobiiState::Opening),
            Tab::Tobii,
            CameraConfig::DEFAULT_WINDOW,
        ),
        (
            "9-keybinds",
            tabs::tobii::fake_status(TobiiState::Streaming),
            Tab::Keybinds,
            CameraConfig::DEFAULT_WINDOW,
        ),
    ];
    for (name, status, tab, size) in shots {
        let mut app = tobii_app_with(&home, status);
        app.active_tab = tab;
        rasterize(&mut app, size, 1.5)
            .save(dir.join(format!("{name}.png")))
            .expect("save png");
    }
}
