use eframe::egui;
use std::sync::atomic::AtomicBool;
use headtrack_core::config::{
    AxisCurve, AxisFilter, CameraConfig, CenterConfig, CrossAxisComp, FovPreset, Prediction,
    ProfileConfig,
};
use headtrack_engine::{Engine, EngineCmd, EngineState, RecordingStatus};
use std::sync::{atomic::Ordering, Arc};

// ── Camera enumeration ──────────────────────────────────────────────────────

/// A detected V4L2 camera device.
#[derive(Clone)]
struct CameraDevice {
    index: u32,
    name: String,
}

impl CameraDevice {
    fn label(&self) -> String {
        format!("{} (/dev/video{})", self.name, self.index)
    }
}

/// Scan `/sys/class/video4linux/` for capture-capable camera devices.
fn enumerate_cameras() -> Vec<CameraDevice> {
    let mut devices = Vec::new();
    let Ok(entries) = std::fs::read_dir("/sys/class/video4linux") else {
        return devices;
    };
    for entry in entries.flatten() {
        let fname = entry.file_name();
        let fname = fname.to_string_lossy();
        let Some(idx_str) = fname.strip_prefix("video") else { continue };
        let Ok(index) = idx_str.parse::<u32>() else { continue };

        let sysfs_index = std::fs::read_to_string(entry.path().join("index"))
            .ok()
            .and_then(|s| s.trim().parse::<u32>().ok())
            .unwrap_or(u32::MAX);
        if sysfs_index != 0 { continue; }

        let name = std::fs::read_to_string(entry.path().join("name"))
            .unwrap_or_else(|_| format!("Camera {index}"))
            .trim()
            .to_string();

        devices.push(CameraDevice { index, name });
    }
    devices.sort_by_key(|d| d.index);
    devices
}

// ── Star Citizen / Wine bridge detection ─────────────────────────────────────

/// Scan common locations for the Star Citizen Wine prefix.
/// Looks for a prefix that already contains the SC Bin64 directory.
fn detect_sc_wine_prefix() -> String {
    let home = std::env::var("HOME").unwrap_or_default();
    let candidates = [
        format!("{home}/Games/star-citizen"),
        format!("{home}/Games/StarCitizen"),
        format!("{home}/Games/star-citizen-live"),
        // Lutris default wine prefix location
        format!("{home}/.local/share/lutris/runners/wine/star-citizen"),
        // Bottles
        format!("{home}/.var/app/com.usebottles.bottles/data/bottles/bottles/StarCitizen"),
        format!("{home}/.wine"),
    ];
    // Prefer a prefix that actually has the SC install inside it.
    for prefix in &candidates {
        let bin64 = sc_bin64_path(prefix);
        if bin64.is_dir() {
            return prefix.clone();
        }
    }
    // Fall back to first prefix that exists.
    candidates
        .into_iter()
        .find(|p| std::path::Path::new(p).is_dir())
        .unwrap_or_default()
}

/// Returns the expected Bin64 path for a given Wine prefix.
fn sc_bin64_path(prefix: &str) -> std::path::PathBuf {
    std::path::Path::new(prefix)
        .join("drive_c/Program Files/Roberts Space Industries/StarCitizen/LIVE/Bin64")
}

/// Find `NPClient64.dll` — in AppImage bridge/ subdir, next to exe, or build output.
fn find_npclient_dll() -> Option<std::path::PathBuf> {
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            // AppImage layout: bridge/ subdir next to exe
            let p = dir.join("bridge/NPClient64.dll");
            if p.exists() {
                return Some(p);
            }
            // Flat layout
            let p = dir.join("NPClient64.dll");
            if p.exists() {
                return Some(p);
            }
        }
    }
    // Cargo cross-build output (development)
    for rel in &[
        "target/x86_64-pc-windows-gnu/release/NPClient64.dll",
        "../target/x86_64-pc-windows-gnu/release/NPClient64.dll",
        "bridge/NPClient64.dll",
    ] {
        let p = std::path::Path::new(rel);
        if p.exists() {
            return Some(p.canonicalize().unwrap_or_else(|_| p.to_path_buf()));
        }
    }
    None
}

// ── X-Plane 12 detection ─────────────────────────────────────────────────────

/// Parse Steam's libraryfolders.vdf and return all library root paths.
fn steam_library_paths() -> Vec<String> {
    let home = std::env::var("HOME").unwrap_or_default();
    let vdf_locations = [
        format!("{home}/.steam/steam/steamapps/libraryfolders.vdf"),
        format!("{home}/.local/share/Steam/steamapps/libraryfolders.vdf"),
        // Flatpak Steam
        format!("{home}/.var/app/com.valvesoftware.Steam/.local/share/Steam/steamapps/libraryfolders.vdf"),
    ];
    let mut paths = Vec::new();
    for vdf in &vdf_locations {
        let Ok(contents) = std::fs::read_to_string(vdf) else { continue };
        for line in contents.lines() {
            // Lines look like:  "path"    "/some/steam/library"
            let parts: Vec<&str> = line.trim().splitn(4, '"').collect();
            if parts.len() >= 4 && parts[1] == "path" && !parts[3].is_empty() {
                paths.push(parts[3].to_string());
            }
        }
    }
    paths
}

/// Scan common X-Plane 12 install locations, including all Steam libraries.
fn detect_xplane12_path() -> String {
    let home = std::env::var("HOME").unwrap_or_default();

    // Steam library installs (highest priority — most common on Linux)
    for lib in steam_library_paths() {
        let p = format!("{lib}/steamapps/common/X-Plane 12");
        if std::path::Path::new(&p).is_dir() {
            return p;
        }
    }

    let candidates = [
        format!("{home}/X-Plane 12"),
        format!("{home}/X-Plane-12"),
        format!("{home}/X-Plane_12"),
        format!("{home}/Games/X-Plane 12"),
        format!("{home}/Games/X-Plane-12"),
        "/opt/xplane12".to_string(),
    ];
    candidates
        .into_iter()
        .find(|p| std::path::Path::new(p).is_dir())
        .unwrap_or_default()
}

/// Find `libheadtrack_xplane.so` — next to the running exe (AppImage), then
/// in the Cargo release build output (development).
fn find_xplane_plugin_so() -> Option<std::path::PathBuf> {
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let p = dir.join("libheadtrack_xplane.so");
            if p.exists() {
                return Some(p);
            }
        }
    }
    for rel in &[
        "target/release/libheadtrack_xplane.so",
        "../target/release/libheadtrack_xplane.so",
    ] {
        let p = std::path::Path::new(rel);
        if p.exists() {
            return Some(p.canonicalize().unwrap_or_else(|_| p.to_path_buf()));
        }
    }
    None
}

// ── System tray ──────────────────────────────────────────────────────────────

struct HeadtrackTray {
    show_flag: Arc<AtomicBool>,
    ctx_slot:  Arc<std::sync::OnceLock<egui::Context>>,
}

impl ksni::Tray for HeadtrackTray {
    fn id(&self) -> String { "headtrack-rs".into() }
    fn title(&self) -> String { "Headtrack RS".into() }
    fn icon_name(&self) -> String { String::new() }

    fn icon_pixmap(&self) -> Vec<ksni::Icon> {
        let icon = eframe::icon_data::from_png_bytes(
            include_bytes!("../../../assets/icons/headtrack-rs.png"),
        ).expect("tray icon PNG");
        // StatusNotifierItem requires ARGB32 big-endian: bytes are [A, R, G, B] per pixel.
        let argb: Vec<u8> = icon.rgba.chunks(4)
            .flat_map(|p| [p[3], p[0], p[1], p[2]])
            .collect();
        vec![ksni::Icon { width: icon.width as i32, height: icon.height as i32, data: argb }]
    }

    fn menu(&self) -> Vec<ksni::MenuItem<Self>> {
        use ksni::menu::*;
        vec![
            MenuItem::Standard(StandardItem {
                label: "Show Headtrack RS".into(),
                activate: Box::new(|this: &mut Self| {
                    this.show_flag.store(true, Ordering::Relaxed);
                    // Wake the event loop so update() runs even if window is minimized.
                    if let Some(ctx) = this.ctx_slot.get() {
                        ctx.request_repaint();
                    }
                }),
                ..Default::default()
            }),
            MenuItem::Separator,
            MenuItem::Standard(StandardItem {
                label: "Quit".into(),
                activate: Box::new(|_this: &mut Self| {
                    // Exit directly — eframe's update() loop doesn't run while minimized,
                    // so ViewportCommand::Close would never be delivered.
                    std::process::exit(0);
                }),
                ..Default::default()
            }),
        ]
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() -> eframe::Result<()> {
    // Engine::start() blocks ~2-3s for ONNX model load + camera init.
    // Spawn it in a background thread so the window appears immediately.
    let (engine_tx, engine_rx) = std::sync::mpsc::channel::<Engine>();
    std::thread::spawn(move || {
        match Engine::start() {
            Ok(engine) => { let _ = engine_tx.send(engine); }
            Err(e)     => eprintln!("headtrack-rs: engine failed to start: {e}"),
        }
    });

    // System tray — spawns a background thread with its own async runtime.
    // assume_sni_available(true) routes SNI-watcher-not-found errors to
    // Tray::watcher_offline() instead of failing spawn; tray silently absent
    // on desktops without StatusNotifierItem support (GNOME w/o extension).
    let tray_show = Arc::new(AtomicBool::new(false));
    let tray_ctx: Arc<std::sync::OnceLock<egui::Context>> = Arc::new(std::sync::OnceLock::new());
    use ksni::blocking::TrayMethods;
    let _tray = HeadtrackTray {
        show_flag: Arc::clone(&tray_show),
        ctx_slot:  Arc::clone(&tray_ctx),
    }
    .assume_sni_available(true)
    .spawn()
    .ok();

    // Background thread: while show_flag is pending, keep nudging the event loop
    // so update() runs even if the window is minimized and eframe has stopped repainting.
    {
        let show_flag = Arc::clone(&tray_show);
        let ctx_slot  = Arc::clone(&tray_ctx);
        std::thread::spawn(move || loop {
            std::thread::sleep(std::time::Duration::from_millis(50));
            if show_flag.load(Ordering::Relaxed) {
                if let Some(ctx) = ctx_slot.get() {
                    ctx.request_repaint();
                }
            }
        });
    }

    let icon = eframe::icon_data::from_png_bytes(
        include_bytes!("../../../assets/icons/headtrack-rs.png"),
    )
    .expect("invalid window icon");

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([820.0, 600.0])
            .with_resizable(false)
            .with_title("Headtrack RS")
            .with_app_id("headtrack-rs")
            .with_icon(icon),
        ..Default::default()
    };

    eframe::run_native(
        "Headtrack RS",
        options,
        Box::new(move |cc| {
            let mut style = (*cc.egui_ctx.style()).clone();

            use egui::TextStyle;
            style.text_styles.insert(TextStyle::Body,      egui::FontId::proportional(15.0));
            style.text_styles.insert(TextStyle::Heading,   egui::FontId::proportional(22.0));
            style.text_styles.insert(TextStyle::Small,     egui::FontId::proportional(12.0));
            style.text_styles.insert(TextStyle::Button,    egui::FontId::proportional(15.0));
            style.text_styles.insert(TextStyle::Monospace, egui::FontId::monospace(14.0));

            style.visuals.widgets.inactive.fg_stroke =
                egui::Stroke::new(1.0, egui::Color32::from_gray(200));
            style.visuals.widgets.hovered.fg_stroke =
                egui::Stroke::new(1.0, egui::Color32::WHITE);
            style.visuals.widgets.active.fg_stroke =
                egui::Stroke::new(2.0, egui::Color32::WHITE);
            style.visuals.widgets.noninteractive.fg_stroke =
                egui::Stroke::new(1.0, egui::Color32::from_gray(210));

            style.visuals.widgets.hovered.weak_bg_fill = egui::Color32::from_gray(55);
            style.visuals.widgets.hovered.bg_fill      = egui::Color32::from_gray(60);

            style.spacing.slider_rail_height = 5.0;
            style.spacing.slider_width       = 200.0;

            cc.egui_ctx.set_style(style);

            Ok(Box::new(AppWrapper::new(engine_rx, tray_show, tray_ctx)))
        }),
    )
}

// ── Constants ─────────────────────────────────────────────────────────────────

const AXIS_NAMES:  [&str; 6] = ["yaw", "pitch", "roll", "x", "y", "z"];
const AXIS_LABELS: [&str; 6] = ["Yaw", "Pitch", "Roll", "X", "Y", "Z"];
const AXIS_UNITS:  [&str; 6] = ["\u{00b0}", "\u{00b0}", "\u{00b0}", "mm", "mm", "mm"];
const ROT_MAX:   f32 = 30.0;
const TRANS_MAX: f32 = 60.0;

/// Ordered list of capture resolution presets (width, height, display label).
/// Lower resolution = higher FPS / lower latency; higher = better localizer accuracy at distance.
const CAMERA_RESOLUTIONS: [(u32, u32, &str); 3] = [
    (320, 240, "Performance  320×240  (max FPS)"),
    (640, 480, "Balanced     640×480  (recommended)"),
    (1280, 720, "Detail       1280×720 (far from camera)"),
];


fn axis_max(i: usize) -> f32 {
    if i < 3 { ROT_MAX } else { TRANS_MAX }
}

// ── Tabs ──────────────────────────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Dashboard,
    Tuning,
    Profiles,
    Settings,
    Camera,
    Dev,
}

impl Tab {
    fn label(&self) -> &'static str {
        match self {
            Tab::Dashboard => "Dashboard",
            Tab::Tuning    => "Tuning",
            Tab::Profiles  => "Profiles",
            Tab::Settings  => "Settings",
            Tab::Camera    => "Camera",
            Tab::Dev       => "Dev Tools",
        }
    }

    fn icon(&self) -> &'static str {
        match self {
            Tab::Dashboard => "\u{25c9}",
            Tab::Tuning    => "\u{2699}",
            Tab::Profiles  => "\u{2630}",
            Tab::Settings  => "\u{2638}",
            Tab::Camera    => "\u{1F4F7}",
            Tab::Dev       => "\u{2692}",
        }
    }
}

const TABS: [Tab; 6] = [
    Tab::Dashboard,
    Tab::Tuning,
    Tab::Profiles,
    Tab::Settings,
    Tab::Camera,
    Tab::Dev,
];

const AXIS_COLORS: [egui::Color32; 6] = [
    egui::Color32::from_rgb(100, 160, 255),
    egui::Color32::from_rgb(100, 220, 130),
    egui::Color32::from_rgb(200, 130, 255),
    egui::Color32::from_rgb(255, 160,  60),
    egui::Color32::from_rgb(220, 220,  60),
    egui::Color32::from_rgb(255, 100, 100),
];

#[derive(PartialEq, Clone, Copy, Default)]
enum TuningGroup {
    #[default]
    Look,
    Movement,
}

// ── App state ─────────────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct AxisFilterState {
    min_cutoff: f32,
    beta: f32,
}

#[derive(Clone, Copy)]
struct AxisCurveState {
    sensitivity: f32,
    exponent: f32,
}

struct HeadtrackApp {
    // ── Engine (owns the tracking infrastructure; kept alive for app lifetime) ──
    #[allow(dead_code)]
    engine: Engine,
    state:  Arc<EngineState>,
    cmd:    headtrack_engine::EngineCmdSender,

    active_tab:       Tab,
    tuning_group:     TuningGroup,
    dev_selected_axis: usize,

    // Per-axis filter + curve
    axis_filter: [AxisFilterState; 6],
    axis_curve:  [AxisCurveState; 6],
    drift_rate:  f32,
    z_drift_mult: f32,
    axis_active: [bool; 6],

    // Cross-axis compensation
    yaw_to_pitch: f32,
    yaw_to_x:     f32,
    yaw_to_y:     f32,
    yaw_to_roll:  f32,
    pitch_to_y:   f32,
    z_to_y:       f32,
    z_to_pitch:   f32,

    // Prediction / Kalman
    predict_ms:            f32,
    rot_process_noise:     f32,
    rot_measurement_noise: f32,
    pos_process_noise:     f32,
    pos_measurement_noise: f32,

    // Per-axis deadzone
    deadzone: [f32; 6],

    // Settings state
    landmark_overlay:    bool,
    tracking_enabled:    bool,
    minimize_to_tray:    bool,
    recenter_flash:      Option<std::time::Instant>,

    // Camera selection
    cameras:         Vec<CameraDevice>,
    selected_camera: u32,
    camera_fov:      f32,
    /// Last FOV value read from EngineState — used to detect engine-side changes.
    last_engine_fov: u32,
    /// Active capture resolution — index into CAMERA_RESOLUTIONS.
    camera_res_idx:  usize,
    /// Text field for naming a new FOV preset before saving.
    fov_preset_name: String,

    // V4L2 Direct API cache
    v4l2_device:      Option<v4l::Device>,
    v4l2_controls:    Vec<v4l::control::Description>,
    v4l2_last_camera: u32,
    v4l2_optimize_status: Option<(String, std::time::Instant)>,

    // Profiles
    profile_name:   String,
    profiles_list:  Vec<(String, std::path::PathBuf)>,
    profile_status: String,

    // Camera preview
    preview_texture:  Option<egui::TextureHandle>,
    preview_last_seq: u64,

    // Sidebar logo
    logo_texture: Option<egui::TextureHandle>,

    // X-Plane installer
    xplane_path:           String,
    xplane_install_status: Option<(String, bool)>, // (message, success)

    // Star Citizen / Wine bridge installer
    sc_prefix_path:    String,
    sc_install_status: Option<(String, bool)>,

    // Cached display string
    ipc_socket_label: String,

    // Diagnostics recorder
    diag_size_limit_mb:    u32,
    diag_recording_start:  Option<std::time::Instant>,
    diag_recording_path:   std::path::PathBuf,
}

impl HeadtrackApp {
    fn new(engine: Engine) -> Self {
        let state  = engine.state();
        let cmd    = engine.cmd_sender();
        let cfg    = ProfileConfig::load(&ProfileConfig::active_config_path()).unwrap_or_default();
        let camera_cfg = CameraConfig::load();

        // Read initial camera index from engine (already set during Engine::start).
        let initial_camera = state.camera_index.load(Ordering::Relaxed);

        Self {
            ipc_socket_label: headtrack_engine::ipc_socket_path().display().to_string(),
            state,
            cmd,
            engine,
            active_tab:       Tab::Dashboard,
            tuning_group:     TuningGroup::default(),
            dev_selected_axis: 0,
            axis_filter: std::array::from_fn(|i| AxisFilterState {
                min_cutoff: cfg.filter[i].min_cutoff,
                beta:       cfg.filter[i].beta,
            }),
            axis_curve: std::array::from_fn(|i| AxisCurveState {
                sensitivity: cfg.curve[i].sensitivity,
                exponent:    cfg.curve[i].exponent,
            }),
            drift_rate:   cfg.center.drift_rate,
            z_drift_mult: cfg.center.z_drift_mult,
            axis_active: [true; 6],
            yaw_to_pitch: cfg.cross_axis.yaw_to_pitch,
            yaw_to_x:     cfg.cross_axis.yaw_to_x,
            yaw_to_y:     cfg.cross_axis.yaw_to_y,
            yaw_to_roll:  cfg.cross_axis.yaw_to_roll,
            pitch_to_y:   cfg.cross_axis.pitch_to_y,
            z_to_y:       cfg.cross_axis.z_to_y,
            z_to_pitch:   cfg.cross_axis.z_to_pitch,
            predict_ms:            cfg.prediction.predict_ms,
            rot_process_noise:     cfg.prediction.rot_process_noise,
            rot_measurement_noise: cfg.prediction.rot_measurement_noise,
            pos_process_noise:     cfg.prediction.pos_process_noise,
            pos_measurement_noise: cfg.prediction.pos_measurement_noise,
            deadzone:          cfg.deadzone,
            landmark_overlay:  false,
            tracking_enabled:  true,
            minimize_to_tray:  camera_cfg.minimize_to_tray,
            recenter_flash:    None,
            cameras:         enumerate_cameras(),
            selected_camera: initial_camera,
            camera_fov:      camera_cfg.fov_diag_deg,
            last_engine_fov: camera_cfg.fov_diag_deg.to_bits(),
            camera_res_idx:  CAMERA_RESOLUTIONS
                .iter()
                .position(|(w, h, _)| (*w, *h) == camera_cfg.camera_resolution)
                .unwrap_or(1), // default to Balanced (640×480)
            fov_preset_name: String::new(),
            v4l2_device:     None,
            v4l2_controls:   Vec::new(),
            v4l2_last_camera: u32::MAX,
            v4l2_optimize_status: None,
            profile_name:    cfg.name,
            profiles_list:   ProfileConfig::list_profiles(),
            profile_status:  String::new(),
            preview_texture:  None,
            preview_last_seq: 0,
            logo_texture:     None,
            xplane_path:           detect_xplane12_path(),
            xplane_install_status: None,
            sc_prefix_path:    detect_sc_wine_prefix(),
            sc_install_status: None,
            diag_size_limit_mb:   50,
            diag_recording_start: None,
            diag_recording_path:  std::path::PathBuf::new(),
        }
    }

    /// Send a pipeline parameter update to the engine.
    fn send_set(&self, stage: &str, param: &str, value: f32) {
        self.cmd.send(EngineCmd::SetParam {
            stage: stage.into(),
            param: param.into(),
            value,
        });
    }

    /// Snapshot current GUI state into a [`ProfileConfig`].
    fn to_profile(&self) -> ProfileConfig {
        ProfileConfig {
            name: self.profile_name.clone(),
            filter: std::array::from_fn(|i| AxisFilter {
                min_cutoff: self.axis_filter[i].min_cutoff,
                beta:       self.axis_filter[i].beta,
            }),
            curve: std::array::from_fn(|i| AxisCurve {
                sensitivity: self.axis_curve[i].sensitivity,
                exponent:    self.axis_curve[i].exponent,
            }),
            cross_axis: CrossAxisComp {
                yaw_to_pitch: self.yaw_to_pitch,
                yaw_to_x:     self.yaw_to_x,
                yaw_to_y:     self.yaw_to_y,
                yaw_to_roll:  self.yaw_to_roll,
                pitch_to_y:   self.pitch_to_y,
                z_to_y:       self.z_to_y,
                z_to_pitch:   self.z_to_pitch,
            },
            prediction: Prediction {
                predict_ms:             self.predict_ms,
                rot_process_noise:      self.rot_process_noise,
                rot_measurement_noise:  self.rot_measurement_noise,
                pos_process_noise:      self.pos_process_noise,
                pos_measurement_noise:  self.pos_measurement_noise,
            },
            deadzone: self.deadzone,
            center: CenterConfig {
                drift_rate:   self.drift_rate,
                z_drift_mult: self.z_drift_mult,
            },
            ..ProfileConfig::default()
        }
    }

    /// Apply a [`ProfileConfig`] to GUI state and push all params to the engine.
    fn apply_profile(&mut self, cfg: &ProfileConfig) {
        self.profile_name = cfg.name.clone();

        for i in 0..6 {
            self.axis_filter[i].min_cutoff = cfg.filter[i].min_cutoff;
            self.axis_filter[i].beta       = cfg.filter[i].beta;
            self.send_set("one-euro-filter", &format!("{}.min_cutoff", AXIS_NAMES[i]), cfg.filter[i].min_cutoff);
            self.send_set("one-euro-filter", &format!("{}.beta",       AXIS_NAMES[i]), cfg.filter[i].beta);

            self.axis_curve[i].sensitivity = cfg.curve[i].sensitivity;
            self.axis_curve[i].exponent    = cfg.curve[i].exponent;
            self.send_set("response-curve", &format!("{}.sensitivity", AXIS_NAMES[i]), cfg.curve[i].sensitivity);
            self.send_set("response-curve", &format!("{}.exponent",    AXIS_NAMES[i]), cfg.curve[i].exponent);

            self.deadzone[i] = cfg.deadzone[i];
            self.send_set("deadzone",    AXIS_NAMES[i], cfg.deadzone[i]);
            self.send_set("slew-limit",  AXIS_NAMES[i], cfg.slew_limit[i]);
        }

        self.yaw_to_pitch = cfg.cross_axis.yaw_to_pitch;
        self.yaw_to_x     = cfg.cross_axis.yaw_to_x;
        self.yaw_to_y     = cfg.cross_axis.yaw_to_y;
        self.yaw_to_roll  = cfg.cross_axis.yaw_to_roll;
        self.pitch_to_y   = cfg.cross_axis.pitch_to_y;
        self.z_to_y       = cfg.cross_axis.z_to_y;
        self.z_to_pitch   = cfg.cross_axis.z_to_pitch;
        self.send_set("cross-axis", "yaw_to_pitch", cfg.cross_axis.yaw_to_pitch);
        self.send_set("cross-axis", "yaw_to_x",     cfg.cross_axis.yaw_to_x);
        self.send_set("cross-axis", "yaw_to_y",     cfg.cross_axis.yaw_to_y);
        self.send_set("cross-axis", "yaw_to_roll",  cfg.cross_axis.yaw_to_roll);
        self.send_set("cross-axis", "pitch_to_y",   cfg.cross_axis.pitch_to_y);
        self.send_set("cross-axis", "z_to_y",       cfg.cross_axis.z_to_y);
        self.send_set("cross-axis", "z_to_pitch",   cfg.cross_axis.z_to_pitch);

        self.predict_ms            = cfg.prediction.predict_ms;
        self.rot_process_noise     = cfg.prediction.rot_process_noise;
        self.rot_measurement_noise = cfg.prediction.rot_measurement_noise;
        self.pos_process_noise     = cfg.prediction.pos_process_noise;
        self.pos_measurement_noise = cfg.prediction.pos_measurement_noise;
        self.send_set("prediction", "predict_ms",            cfg.prediction.predict_ms);
        self.send_set("prediction", "rot_process_noise",     cfg.prediction.rot_process_noise);
        self.send_set("prediction", "rot_measurement_noise", cfg.prediction.rot_measurement_noise);
        self.send_set("prediction", "pos_process_noise",     cfg.prediction.pos_process_noise);
        self.send_set("prediction", "pos_measurement_noise", cfg.prediction.pos_measurement_noise);

        self.drift_rate   = cfg.center.drift_rate;
        self.z_drift_mult = cfg.center.z_drift_mult;
        self.send_set("center", "drift_rate",   cfg.center.drift_rate);
        self.send_set("center", "z.drift_mult", cfg.center.z_drift_mult);
    }
}

// ── Drawing helpers ──────────────────────────────────────────────────────────

fn draw_bar(ui: &mut egui::Ui, value: f32, max_val: f32, active: bool, width: f32) {
    let bar_height = 16.0;
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(width, bar_height),
        egui::Sense::hover(),
    );

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, if active {
        egui::Color32::from_gray(40)
    } else {
        egui::Color32::from_gray(25)
    });

    let center_x = rect.center().x;
    painter.line_segment(
        [egui::pos2(center_x, rect.top()), egui::pos2(center_x, rect.bottom())],
        egui::Stroke::new(1.0, egui::Color32::from_gray(80)),
    );

    let normalized = (value / max_val).clamp(-1.0, 1.0);
    let bar_color = if !active {
        egui::Color32::from_gray(60)
    } else if normalized.abs() > 0.85 {
        egui::Color32::from_rgb(220, 80, 80)
    } else if normalized.abs() > 0.6 {
        egui::Color32::from_rgb(220, 180, 50)
    } else {
        egui::Color32::from_rgb(80, 180, 220)
    };

    let bar_end_x = center_x + normalized * (width / 2.0);
    let bar_rect = if normalized >= 0.0 {
        egui::Rect::from_min_max(
            egui::pos2(center_x, rect.top() + 2.0),
            egui::pos2(bar_end_x, rect.bottom() - 2.0),
        )
    } else {
        egui::Rect::from_min_max(
            egui::pos2(bar_end_x, rect.top() + 2.0),
            egui::pos2(center_x, rect.bottom() - 2.0),
        )
    };
    painter.rect_filled(bar_rect, 1.0, bar_color);
}

fn placeholder_box(ui: &mut egui::Ui, width: f32, height: f32, label: &str) {
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(width, height),
        egui::Sense::hover(),
    );
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(25, 25, 30));
    painter.rect_stroke(rect, 4.0, egui::Stroke::new(1.0, egui::Color32::from_gray(60)), egui::StrokeKind::Outside);
    painter.text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        label,
        egui::FontId::proportional(14.0),
        egui::Color32::from_gray(130),
    );
}

fn param_cell(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    decimals: usize,
) -> bool {
    let mut changed = false;
    ui.vertical(|ui| {
        ui.label(egui::RichText::new(label).strong().small());
        ui.horizontal(|ui| {
            changed |= ui
                .add(egui::Slider::new(value, range.clone()).show_value(false))
                .changed();
            changed |= ui
                .add(
                    egui::DragValue::new(value)
                        .fixed_decimals(decimals)
                        .range(range)
                        .speed(0.001),
                )
                .changed();
        });
    });
    changed
}

fn section(ui: &mut egui::Ui, label: &str) {
    ui.add_space(12.0);
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_at_least(egui::vec2(3.0, 18.0), egui::Sense::hover());
        ui.painter().rect_filled(rect, 2.0, colors::ORANGE);
        ui.add_space(4.0);
        ui.heading(egui::RichText::new(label).strong().size(16.0));
    });
    ui.add_space(4.0);
}

fn nice_ceil(v: f32) -> f32 {
    if !v.is_finite() || v <= 0.0 { return 10.0; }
    let mag = 10_f32.powf(v.log10().floor());
    ((v / mag).ceil() * mag).max(1.0)
}

fn draw_curve_editor(
    ui: &mut egui::Ui,
    curves: &[(f32, f32, f32, egui::Color32)],
    is_rotation: bool,
    width: f32,
    height: f32,
) {
    let max_x: f32 = if is_rotation { 45.0 } else { 80.0 };
    let unit = if is_rotation { "°" } else { "mm" };

    let lpad = 32.0;
    let rpad = 16.0;
    let bpad = 20.0;
    let tpad =  8.0;

    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(width, height + bpad + tpad),
        egui::Sense::hover(),
    );
    let painter = ui.painter_at(rect);

    let chart = egui::Rect::from_min_max(
        egui::pos2(rect.left() + lpad, rect.top() + tpad),
        egui::pos2(rect.right() - rpad, rect.top() + tpad + height),
    );

    painter.rect_filled(chart, 4.0, egui::Color32::from_rgb(18, 18, 24));
    painter.rect_stroke(
        chart,
        4.0,
        egui::Stroke::new(1.0, egui::Color32::from_gray(50)),
        egui::StrokeKind::Outside,
    );

    let raw_max_y = curves
        .iter()
        .map(|(s, e, _, _)| s * max_x.powf(*e))
        .fold(max_x * 0.2, f32::max);
    let max_y = nice_ceil(raw_max_y);

    let to_sx = |x: f32| chart.left() + (x / max_x).clamp(0.0, 1.0) * chart.width();
    let to_sy = |y: f32| chart.bottom() - (y / max_y).clamp(0.0, 1.2) * chart.height();

    let grid_col = egui::Color32::from_gray(38);
    let tick_col = egui::Color32::from_gray(120);

    for i in 1..=4_u32 {
        let x = max_x * i as f32 / 4.0;
        let sx = to_sx(x);
        painter.line_segment(
            [egui::pos2(sx, chart.top()), egui::pos2(sx, chart.bottom())],
            egui::Stroke::new(1.0, grid_col),
        );
        painter.text(
            egui::pos2(sx, chart.bottom() + 3.0),
            egui::Align2::CENTER_TOP,
            format!("{:.0}{unit}", x),
            egui::FontId::proportional(10.0),
            tick_col,
        );
    }

    for i in 1..=3_u32 {
        let y = max_y * i as f32 / 3.0;
        let sy = to_sy(y);
        painter.line_segment(
            [egui::pos2(chart.left(), sy), egui::pos2(chart.right(), sy)],
            egui::Stroke::new(1.0, grid_col),
        );
        painter.text(
            egui::pos2(chart.left() - 4.0, sy),
            egui::Align2::RIGHT_CENTER,
            format!("{:.0}{unit}", y),
            egui::FontId::proportional(10.0),
            tick_col,
        );
    }

    {
        let pts: Vec<egui::Pos2> = (0..=60).map(|i| {
            let x = max_x * i as f32 / 60.0;
            egui::pos2(to_sx(x), to_sy(x))
        }).collect();
        for chunk in pts.chunks(3) {
            if let [a, _, c] = chunk {
                painter.line_segment(
                    [*a, *c],
                    egui::Stroke::new(1.0, egui::Color32::from_gray(65)),
                );
            }
        }
    }

    const N: usize = 120;
    for &(sens, exp, live_input, color) in curves {
        let pts: Vec<egui::Pos2> = (0..=N)
            .map(|j| {
                let x = max_x * j as f32 / N as f32;
                let y = sens * x.powf(exp);
                egui::pos2(to_sx(x), to_sy(y))
            })
            .collect();
        painter.add(egui::Shape::line(pts, egui::Stroke::new(2.0, color)));

        let lx = live_input.abs().clamp(0.0, max_x);
        let ly = sens * lx.powf(exp);
        let sx = to_sx(lx);
        let sy = to_sy(ly).clamp(chart.top(), chart.bottom());
        painter.circle_filled(egui::pos2(sx, sy), 5.0, color);
        painter.circle_stroke(egui::pos2(sx, sy), 5.0, egui::Stroke::new(1.5, egui::Color32::WHITE));
    }

}

fn draw_pose_bubble(
    ui: &mut egui::Ui,
    yaw: f32, pitch: f32, roll: f32, z: f32,
    width: f32, height: f32,
) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let painter = ui.painter_at(rect);

    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(16, 16, 20));
    painter.rect_stroke(
        rect, 4.0,
        egui::Stroke::new(1.0, egui::Color32::from_gray(45)),
        egui::StrokeKind::Outside,
    );

    let cx = rect.center().x;
    let cy = rect.center().y;

    let cross_col = egui::Color32::from_gray(38);
    painter.line_segment(
        [egui::pos2(rect.left(), cy), egui::pos2(rect.right(), cy)],
        egui::Stroke::new(1.0, cross_col),
    );
    painter.line_segment(
        [egui::pos2(cx, rect.top()), egui::pos2(cx, rect.bottom())],
        egui::Stroke::new(1.0, cross_col),
    );

    let travel_x = width  * 0.30;
    let travel_y = height * 0.27;
    let bx = cx + (yaw   / 90.0).clamp(-1.0, 1.0) * travel_x;
    let by = cy - (pitch / 90.0).clamp(-1.0, 1.0) * travel_y;

    let base_r = (width.min(height) * 0.13).max(16.0);
    let radius = base_r * (1.0 + (z / 150.0).clamp(-0.35, 0.55));

    let bubble_color = egui::Color32::from_rgba_unmultiplied(255, 140, 0, 160);
    painter.circle_stroke(egui::pos2(bx, by), radius, egui::Stroke::new(1.5, bubble_color));
    painter.circle_filled(egui::pos2(bx, by), 2.5, colors::ORANGE);

    {
        let roll_rad = roll.to_radians();
        let arc_half = 18.0_f32.to_radians();
        // Roll arc uses the roll axis color — purple/violet
        let roll_color = egui::Color32::from_rgb(200, 130, 255);
        const N: usize = 24;
        let arc_pts: Vec<egui::Pos2> = (0..=N)
            .map(|k| {
                let frac = k as f32 / N as f32;
                let angle =
                    roll_rad - arc_half + frac * arc_half * 2.0 - std::f32::consts::FRAC_PI_2;
                egui::pos2(bx + radius * angle.cos(), by + radius * angle.sin())
            })
            .collect();
        painter.add(egui::Shape::line(arc_pts, egui::Stroke::new(3.5, roll_color)));
    }

    painter.text(
        egui::pos2(cx, rect.bottom() - 5.0),
        egui::Align2::CENTER_BOTTOM,
        format!("Y {:+.0}°  P {:+.0}°  R {:+.0}°  Z {:+.0}mm", yaw, pitch, roll, z),
        egui::FontId::proportional(10.0),
        egui::Color32::from_gray(140),
    );
}

// ── Palette ───────────────────────────────────────────────────────────────────

mod colors {
    use eframe::egui::Color32;
    pub const ORANGE:      Color32 = Color32::from_rgb(255, 140,   0);
    pub const ORANGE_DIM:  Color32 = Color32::from_rgb(130,  72,   0);
    pub const BG_DARK:     Color32 = Color32::from_rgb( 18,  18,  20);
    pub const TEXT_DIM:    Color32 = Color32::from_gray(110);
    pub const GREEN_READY: Color32 = Color32::from_rgb( 80, 200, 100);
}

// Legacy names — used throughout the main UI; will migrate to colors:: over time.
const COLOR_CONNECTED:    egui::Color32 = egui::Color32::from_rgb(80, 200, 120);
const COLOR_DISCONNECTED: egui::Color32 = egui::Color32::from_rgb(200, 80, 80);
const COLOR_SIDEBAR_BG:   egui::Color32 = egui::Color32::from_rgb(30, 30, 35);

// ── Splash screen ─────────────────────────────────────────────────────────────

struct SplashScreen {
    start_time: std::time::Instant,
    last_frame: std::time::Instant,
    dot_offset: egui::Vec2, // current lerped dot offset from center
    ring_angle: f32,        // current rotation of the dashed inner ring
}

impl SplashScreen {
    fn new() -> Self {
        let now = std::time::Instant::now();
        Self { start_time: now, last_frame: now, dot_offset: egui::Vec2::ZERO, ring_angle: 0.0 }
    }

    fn elapsed(&self) -> f32 {
        self.start_time.elapsed().as_secs_f32()
    }

    fn show(&mut self, ctx: &egui::Context, ready: bool) {
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(colors::BG_DARK))
            .show(ctx, |ui| self.draw(ui, ready));
        ctx.request_repaint(); // drive animation at display refresh rate
    }

    fn draw(&mut self, ui: &mut egui::Ui, ready: bool) {
        let now     = std::time::Instant::now();
        let dt      = now.duration_since(self.last_frame).as_secs_f32().min(0.05);
        self.last_frame = now;
        let elapsed = now.duration_since(self.start_time).as_secs_f32();

        let painter = ui.painter();
        let rect    = ui.max_rect();
        let center  = rect.center();

        // Geometry
        let outer_r  = 120.0_f32;
        let inner_r  =  96.0_f32;
        let travel_r =  72.0_f32;

        // Outer ring — very faint
        painter.circle_stroke(
            center, outer_r,
            egui::Stroke::new(1.0, egui::Color32::from_white_alpha(20)),
        );

        // Cardinal ticks — orange, flush inside the outer ring
        let tick_stroke = egui::Stroke::new(2.0, colors::ORANGE);
        let tick_outer  = outer_r - 2.0;
        let tick_inner  = tick_outer - 16.0;
        for &deg in &[0.0_f32, 90.0, 180.0, 270.0] {
            let rad = deg.to_radians();
            let dir = egui::Vec2::new(rad.cos(), rad.sin());
            painter.line_segment(
                [center + dir * tick_inner, center + dir * tick_outer],
                tick_stroke,
            );
        }

        // Rotating dashed inner ring
        self.ring_angle += 1.8 * dt;
        let dash_count = 36_usize;
        for i in 0..dash_count {
            if i % 2 == 0 {
                let a1 = self.ring_angle + i as f32 * std::f32::consts::TAU / dash_count as f32;
                let a2 = self.ring_angle + (i as f32 + 0.8) * std::f32::consts::TAU / dash_count as f32;
                let p1 = center + egui::Vec2::new(a1.cos(), a1.sin()) * inner_r;
                let p2 = center + egui::Vec2::new(a2.cos(), a2.sin()) * inner_r;
                painter.line_segment(
                    [p1, p2],
                    egui::Stroke::new(1.5, egui::Color32::from_white_alpha(40)),
                );
            }
        }

        // Cardinal sweep dot — smooth lerp toward each cardinal position
        // 1.2-second looping cycle: center → left → right → up → down → center
        let cycle  = elapsed % 1.2;
        let target = if cycle < 0.2 {
            egui::Vec2::ZERO
        } else if cycle < 0.4 {
            egui::Vec2::new(-travel_r, 0.0)
        } else if cycle < 0.6 {
            egui::Vec2::new(travel_r, 0.0)
        } else if cycle < 0.8 {
            egui::Vec2::new(0.0, -travel_r)
        } else if cycle < 1.0 {
            egui::Vec2::new(0.0, travel_r)
        } else {
            egui::Vec2::ZERO
        };

        let k = (20.0 * dt).min(1.0); // lerp factor — crisp transitions
        self.dot_offset += (target - self.dot_offset) * k;
        let dot_pos = center + self.dot_offset;
        painter.circle_filled(dot_pos, 6.0, colors::ORANGE);

        // Telemetry — top left
        let mono_sm = egui::FontId::monospace(11.0);
        let tl = rect.left_top() + egui::Vec2::new(20.0, 20.0);
        painter.text(tl, egui::Align2::LEFT_TOP,
            format!("SYS_UPTIME: {:.3}s", elapsed + 10.0),
            mono_sm.clone(), colors::TEXT_DIM);
        painter.text(tl + egui::Vec2::new(0.0, 14.0), egui::Align2::LEFT_TOP,
            format!("DOT_POS:   [{:.0}, {:.0}]", dot_pos.x, dot_pos.y),
            mono_sm.clone(), colors::TEXT_DIM);

        // Telemetry — top right (label row + value row)
        let tr = rect.right_top() + egui::Vec2::new(-210.0, 20.0);
        painter.text(tr, egui::Align2::LEFT_TOP,
            "STATUS   SENSOR    LAT",
            mono_sm.clone(), colors::TEXT_DIM);
        let (telem_val, telem_color) = if ready {
            ("READY    ONLINE    <3ms", colors::GREEN_READY)
        } else {
            ("INIT     PENDING   ---",  colors::ORANGE)
        };
        painter.text(tr + egui::Vec2::new(0.0, 14.0), egui::Align2::LEFT_TOP,
            telem_val, mono_sm.clone(), telem_color);

        // Brand
        painter.text(
            center + egui::Vec2::new(0.0, outer_r + 38.0),
            egui::Align2::CENTER_CENTER,
            "Headtrack RS",
            egui::FontId::proportional(22.0),
            colors::ORANGE,
        );

        // Status message — blinking while loading, steady green when ready
        if ready {
            painter.text(
                center + egui::Vec2::new(0.0, outer_r + 62.0),
                egui::Align2::CENTER_CENTER,
                "TRACKING ONLINE",
                egui::FontId::monospace(13.0),
                colors::GREEN_READY,
            );
        } else {
            let status_msg = if elapsed < 1.0 {
                "INITIALIZING..."
            } else if elapsed < 2.8 {
                "LOADING MODEL..."
            } else {
                "OPENING CAMERA..."
            };
            if (elapsed * 3.0) as u32 % 2 == 0 {
                painter.text(
                    center + egui::Vec2::new(0.0, outer_r + 62.0),
                    egui::Align2::CENTER_CENTER,
                    status_msg,
                    egui::FontId::monospace(13.0),
                    colors::ORANGE,
                );
            }
        }
    }
}

// ── App lifecycle state machine ───────────────────────────────────────────────

/// Minimum time the splash is shown regardless of how fast the engine starts.
const MIN_SPLASH_SECS: f32 = 0.5;

enum AppState {
    /// Engine loading in background — show splash animation.
    Splash {
        rx:     std::sync::mpsc::Receiver<Engine>,
        engine: Option<Engine>, // held here once ready, until MIN_SPLASH_SECS elapses
        splash: SplashScreen,
    },
    /// Engine ready — fade from black into the main dashboard.
    FadingIn {
        app:        Box<HeadtrackApp>,
        fade_start: std::time::Instant,
    },
    /// Normal operation.
    Running {
        app: Box<HeadtrackApp>,
    },
}

struct AppWrapper {
    state:     Option<AppState>, // Option so we can take() during update
    tray_show: Arc<AtomicBool>,
    tray_ctx:  Arc<std::sync::OnceLock<egui::Context>>,
}

impl AppWrapper {
    fn new(
        rx: std::sync::mpsc::Receiver<Engine>,
        tray_show: Arc<AtomicBool>,
        tray_ctx:  Arc<std::sync::OnceLock<egui::Context>>,
    ) -> Self {
        Self {
            state: Some(AppState::Splash { rx, engine: None, splash: SplashScreen::new() }),
            tray_show,
            tray_ctx,
        }
    }
}

impl eframe::App for AppWrapper {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        // Give the tray a handle to wake the event loop (needed for quit while minimized).
        let _ = self.tray_ctx.set(ctx.clone());

        // Minimize to tray when the user clicks the window close button (if enabled).
        if ctx.input(|i| i.viewport().close_requested()) {
            if CameraConfig::load().minimize_to_tray {
                ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
                ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(true));
            }
            // If minimize_to_tray is false, let the close proceed normally.
        }
        // Tray "Show" clicked — restore from minimized and focus.
        if self.tray_show.swap(false, Ordering::Relaxed) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
        }

        let Some(state) = self.state.take() else {
            debug_assert!(false, "AppWrapper state missing");
            return;
        };

        self.state = Some(match state {
            // ── Splash: draw HUD, collect engine, respect minimum duration ──
            AppState::Splash { rx, mut engine, mut splash } => {
                // Collect the engine as soon as it's ready (non-blocking).
                if engine.is_none() {
                    if let Ok(e) = rx.try_recv() { engine = Some(e); }
                }
                let ready   = engine.is_some();
                let elapsed = splash.elapsed();
                splash.show(ctx, ready);

                if ready && elapsed >= MIN_SPLASH_SECS {
                    AppState::FadingIn {
                        app:        Box::new(HeadtrackApp::new(engine.unwrap())),
                        fade_start: std::time::Instant::now(),
                    }
                } else {
                    AppState::Splash { rx, engine, splash }
                }
            }

            // ── Fade-in: render main app then overlay dimming rect ────────
            AppState::FadingIn { mut app, fade_start } => {
                app.update(ctx, frame);

                const FADE_SECS: f32 = 0.30;
                let alpha = 1.0 - (fade_start.elapsed().as_secs_f32() / FADE_SECS).clamp(0.0, 1.0);

                if alpha > 0.004 {
                    let a = (alpha * 255.0) as u8;
                    ctx.layer_painter(egui::LayerId::new(
                        egui::Order::Foreground,
                        egui::Id::new("fade_overlay"),
                    ))
                    .rect_filled(ctx.screen_rect(), 0.0, egui::Color32::from_black_alpha(a));
                    ctx.request_repaint();
                    AppState::FadingIn { app, fade_start }
                } else {
                    AppState::Running { app }
                }
            }

            // ── Running: pure delegation ──────────────────────────────────
            AppState::Running { mut app } => {
                app.update(ctx, frame);
                AppState::Running { app }
            }
        });
    }
}

// ── App implementation ────────────────────────────────────────────────────────

impl eframe::App for HeadtrackApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(std::time::Duration::from_millis(33));

        // Sync FOV from engine in case a camera switch triggered auto-detection.
        let engine_fov_bits = self.state.current_fov.load(Ordering::Relaxed);
        if engine_fov_bits != self.last_engine_fov {
            self.last_engine_fov = engine_fov_bits;
            self.camera_fov = f32::from_bits(engine_fov_bits);
        }

        // Read live state directly from the engine — zero socket/IPC overhead.
        let pose      = *self.state.latest_pose.lock().unwrap_or_else(|e| e.into_inner());
        let fps       = *self.state.fps.lock().unwrap_or_else(|e| e.into_inner());
        let connected = self.state.camera_connected.load(Ordering::Relaxed);
        let pose_vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];

        // ── Status bar (bottom) ──────────────────────────────────────────
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let (status_text, status_color) = if connected {
                    ("Tracking", COLOR_CONNECTED)
                } else {
                    ("No Camera", COLOR_DISCONNECTED)
                };
                ui.label(egui::RichText::new("\u{25cf}").color(status_color));
                ui.label(status_text);
                ui.separator();
                ui.label(egui::RichText::new(format!("{:.0} Hz", fps)).color(colors::ORANGE));

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(egui::RichText::new("Headtrack RS v0.1").weak().small());
                });
            });
        });

        // ── Sidebar ──────────────────────────────────────────────────────
        egui::SidePanel::left("sidebar")
            .exact_width(130.0)
            .resizable(false)
            .frame(
                egui::Frame::NONE
                    .fill(COLOR_SIDEBAR_BG)
                    .inner_margin(egui::Margin::same(8)),
            )
            .show(ctx, |ui| {
                ui.add_space(8.0);

                // ── Logo ─────────────────────────────────────────────────
                let logo = self.logo_texture.get_or_insert_with(|| {
                    let bytes = include_bytes!("../../../assets/icons/headtrack-rs.png");
                    let icon  = eframe::icon_data::from_png_bytes(bytes).expect("logo PNG");
                    let image = egui::ColorImage::from_rgba_unmultiplied(
                        [icon.width as usize, icon.height as usize],
                        &icon.rgba,
                    );
                    ctx.load_texture("app-logo", image, egui::TextureOptions::LINEAR)
                });
                ui.vertical_centered(|ui| {
                    let sized = egui::load::SizedTexture::new(logo.id(), egui::vec2(96.0, 96.0));
                    ui.add(egui::Image::new(sized));
                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("Headtrack RS").strong().size(13.0).color(colors::ORANGE));
                });

                ui.add_space(14.0);

                for tab in &TABS {
                    let is_active = self.active_tab == *tab;
                    let text  = format!("{} {}", tab.icon(), tab.label());
                    let label = egui::RichText::new(text)
                        .size(14.0)
                        .color(if is_active { egui::Color32::WHITE } else { egui::Color32::from_gray(170) });

                    let btn = egui::Button::new(label)
                        .fill(if is_active {
                            egui::Color32::from_rgb(50, 60, 80)
                        } else {
                            egui::Color32::TRANSPARENT
                        })
                        .corner_radius(4.0)
                        .min_size(egui::vec2(114.0, 34.0));

                    let response = ui.add(btn);

                    if is_active {
                        let r = response.rect;
                        let accent = egui::Rect::from_min_max(
                            egui::pos2(r.left(), r.top() + 4.0),
                            egui::pos2(r.left() + 3.0, r.bottom() - 4.0),
                        );
                        ui.painter().rect_filled(accent, 1.5, colors::ORANGE);
                    }

                    if response.clicked() {
                        self.active_tab = *tab;
                    }
                    ui.add_space(2.0);
                }
            });

        // ── Main content area ────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    ui.set_min_width(480.0);
                    match self.active_tab {
                        Tab::Dashboard => self.ui_dashboard(ui, &pose_vals),
                        Tab::Tuning    => self.ui_tuning(ui, &pose_vals),
                        Tab::Profiles  => self.ui_profiles(ui),
                        Tab::Settings  => self.ui_settings(ui),
                        Tab::Camera    => self.ui_camera(ui),
                        Tab::Dev       => self.ui_dev(ui, &pose_vals),
                    }
                });
        });
    }
}

// ── Tab implementations ──────────────────────────────────────────────────────

impl HeadtrackApp {
    /// Decode JPEG preview and show in an image widget at `size`, or placeholder.
    fn draw_camera_preview(&mut self, ui: &mut egui::Ui, size: egui::Vec2) {
        let preview_size = size;

        // Read new JPEG from the engine's shared preview slot.
        let mut new_image = None;
        if let Ok(lock) = self.state.preview.lock() {
            let (seq, ref jpeg) = *lock;
            if seq != self.preview_last_seq {
                if let Some(ref data) = jpeg {
                    self.preview_last_seq = seq;
                    new_image = Some(data.clone());
                }
            }
        }

        if let Some(jpeg) = new_image {
            if let Ok(img) = image::load_from_memory_with_format(&jpeg, image::ImageFormat::Jpeg) {
                let rgba = img.to_rgba8();
                let size = [rgba.width() as usize, rgba.height() as usize];
                let pixels = rgba.as_flat_samples();
                let color_image = egui::ColorImage::from_rgba_unmultiplied(size, pixels.as_slice());
                let tex = ui.ctx().load_texture(
                    "camera_preview",
                    color_image,
                    egui::TextureOptions::LINEAR,
                );
                self.preview_texture = Some(tex);
            }
        }

        if let Some(ref tex) = self.preview_texture {
            ui.image(egui::load::SizedTexture::new(tex.id(), preview_size));
        } else {
            placeholder_box(ui, preview_size.x, preview_size.y, "Camera Feed\n(initialising...)");
        }
    }

    // ── Dashboard ────────────────────────────────────────────────────────
    fn ui_dashboard(
        &mut self,
        ui: &mut egui::Ui,
        pose_vals: &[f32; 6],
    ) {
        let fps       = *self.state.fps.lock().unwrap_or_else(|e| e.into_inner());
        let connected = self.state.camera_connected.load(Ordering::Relaxed);

        ui.heading("Dashboard");
        ui.add_space(8.0);

        // ── Action buttons ───────────────────────────────────────────────
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 6.0;

            let recenter_lit = self.recenter_flash
                .map(|t| t.elapsed().as_secs_f32() < 0.4)
                .unwrap_or(false);
            let recenter_color = if recenter_lit { colors::ORANGE } else { egui::Color32::from_gray(200) };
            if ui
                .add(
                    egui::Button::new(egui::RichText::new("Recenter").color(recenter_color))
                        .min_size(egui::vec2(90.0, 30.0)),
                )
                .clicked()
            {
                self.cmd.send(EngineCmd::Recenter);
                self.recenter_flash = Some(std::time::Instant::now());
            }

            let (track_label, track_color, track_fill) = if self.tracking_enabled {
                ("Pause Tracking", egui::Color32::from_gray(200), egui::Color32::from_gray(50))
            } else {
                ("Resume Tracking", colors::ORANGE, colors::ORANGE_DIM)
            };
            if ui
                .add(
                    egui::Button::new(egui::RichText::new(track_label).color(track_color))
                        .fill(track_fill)
                        .min_size(egui::vec2(130.0, 30.0)),
                )
                .clicked()
            {
                self.tracking_enabled = !self.tracking_enabled;
                for i in 0..6 {
                    let v = if self.tracking_enabled && self.axis_active[i] { 1.0 } else { 0.0 };
                    self.send_set("axis-mask", AXIS_NAMES[i], v);
                }
            }

            let overlay_color = if self.landmark_overlay { colors::ORANGE } else { egui::Color32::from_gray(170) };
            if ui
                .add(egui::Button::new(egui::RichText::new("Overlay").color(overlay_color))
                    .min_size(egui::vec2(75.0, 30.0)))
                .clicked()
            {
                self.landmark_overlay = !self.landmark_overlay;
                self.cmd.send(EngineCmd::Overlay { enabled: self.landmark_overlay });
            }

            // Recenter confirmation flash
            if let Some(t) = self.recenter_flash {
                if t.elapsed().as_secs_f32() < 1.5 {
                    ui.add_space(6.0);
                    ui.label(egui::RichText::new("\u{2713} Recentered").color(COLOR_CONNECTED).small());
                } else {
                    self.recenter_flash = None;
                }
            }
        });

        ui.add_space(10.0);

        // ── Two-column: preview left, info + pose right ──────────────────
        ui.horizontal_top(|ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_rgb(16, 16, 20))
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(45)))
                .corner_radius(4.0)
                .inner_margin(egui::Margin::same(0))
                .show(ui, |ui| {
                    self.draw_camera_preview(ui, egui::vec2(320.0, 240.0));
                });

            ui.add_space(14.0);

            ui.vertical(|ui| {
                // Status info
                egui::Grid::new("dash_status")
                    .spacing([16.0, 5.0])
                    .show(ui, |ui| {
                        let (status_text, status_color) = if connected {
                            ("Tracking", COLOR_CONNECTED)
                        } else {
                            ("No Camera", COLOR_DISCONNECTED)
                        };
                        ui.label(egui::RichText::new("Status").weak().small());
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("\u{25cf}").color(status_color).small());
                            ui.label(egui::RichText::new(status_text).small());
                        });
                        ui.end_row();

                        let cam_name = self.cameras.iter()
                            .find(|c| c.index == self.selected_camera)
                            .map(|c| c.name.clone())
                            .unwrap_or_else(|| format!("/dev/video{}", self.selected_camera));
                        ui.label(egui::RichText::new("Camera").weak().small());
                        ui.label(egui::RichText::new(cam_name).small());
                        ui.end_row();

                        ui.label(egui::RichText::new("Profile").weak().small());
                        ui.label(egui::RichText::new(&self.profile_name).small());
                        ui.end_row();

                        ui.label(egui::RichText::new("FPS").weak().small());
                        ui.label(egui::RichText::new(format!("{:.0}", fps)).color(colors::ORANGE).small());
                        ui.end_row();
                    });

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(6.0);

                // Live pose — compact text only, gives recenter feedback
                ui.label(egui::RichText::new("Live Pose").weak().small());
                ui.add_space(3.0);
                egui::Grid::new("dash_pose")
                    .spacing([10.0, 3.0])
                    .show(ui, |ui| {
                        for i in 0..6 {
                            ui.label(
                                egui::RichText::new(AXIS_LABELS[i])
                                    .color(AXIS_COLORS[i])
                                    .monospace()
                                    .small(),
                            );
                            ui.label(
                                egui::RichText::new(format!("{:+6.1}{}", pose_vals[i], AXIS_UNITS[i]))
                                    .monospace()
                                    .small(),
                            );
                            ui.end_row();
                        }
                    });
            });
        });
    }

    // ── Tuning ───────────────────────────────────────────────────────────
    fn ui_tuning(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        ui.heading("Tuning");
        ui.label(
            egui::RichText::new(
                "Adjust how head tracking feels. Use Dev Tools for per-axis fine control.",
            )
            .weak(),
        );
        ui.add_space(8.0);

        egui::Frame::new()
            .fill(egui::Color32::from_gray(28))
            .stroke(egui::Stroke::new(1.0, egui::Color32::from_gray(55)))
            .corner_radius(6.0)
            .inner_margin(egui::Margin::same(3))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 2.0;
                    for (group, label) in [
                        (TuningGroup::Look,     "Look Controls (Yaw / Pitch)"),
                        (TuningGroup::Movement, "Movement Controls (X / Y / Z / Roll)"),
                    ] {
                        let active = self.tuning_group == group;
                        let btn = egui::Button::new(
                            egui::RichText::new(label).size(13.0).color(
                                if active { egui::Color32::WHITE } else { egui::Color32::from_gray(140) }
                            )
                        )
                        .fill(if active { colors::ORANGE } else { egui::Color32::TRANSPARENT })
                        .corner_radius(4.0);
                        if ui.add(btn).clicked() {
                            self.tuning_group = group;
                        }
                    }
                });
            });

        ui.add_space(12.0);

        match self.tuning_group {
            TuningGroup::Look     => self.ui_tuning_group(ui, pose_vals, true),
            TuningGroup::Movement => self.ui_tuning_group(ui, pose_vals, false),
        }
    }

    fn ui_tuning_group(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6], is_look: bool) {
        let rep_idx = if is_look { 0 } else { 3 };
        let group_axes: &[usize] = if is_look { &[0, 1] } else { &[2, 3, 4, 5] };

        let sens = self.axis_curve[rep_idx].sensitivity;
        let exp  = self.axis_curve[rep_idx].exponent;

        egui::Grid::new(if is_look { "look_telem" } else { "move_telem" })
            .spacing([20.0, 3.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("").size(11.0));
                for &idx in group_axes {
                    ui.label(egui::RichText::new(AXIS_LABELS[idx]).color(AXIS_COLORS[idx]).size(11.0));
                }
                ui.end_row();

                ui.label(egui::RichText::new("HEAD").weak().size(11.0));
                for &idx in group_axes {
                    ui.label(
                        egui::RichText::new(format!("{:.1}{}", pose_vals[idx].abs(), AXIS_UNITS[idx]))
                            .strong().size(15.0),
                    );
                }
                ui.end_row();

                ui.label(egui::RichText::new("CAMERA").weak().size(11.0));
                for &idx in group_axes {
                    let head = pose_vals[idx].abs();
                    let cam = self.axis_curve[idx].sensitivity * head.powf(self.axis_curve[idx].exponent);
                    ui.label(
                        egui::RichText::new(format!("{:.1}{}", cam, AXIS_UNITS[idx]))
                            .strong().size(15.0),
                    );
                }
                ui.end_row();
            });

        ui.add_space(8.0);

        let avail = ui.available_width().min(600.0);
        draw_pose_bubble(ui, pose_vals[0], pose_vals[1], pose_vals[2], pose_vals[5], avail, 220.0);

        ui.add_space(8.0);
        ui.separator();
        ui.add_space(6.0);

        ui.horizontal(|ui| {
            let mut gain = sens;
            if param_cell(ui, "Responsiveness (Gain)", &mut gain, 0.1..=3.0, 2) {
                for &i in group_axes {
                    self.axis_curve[i].sensitivity = gain;
                    self.send_set("response-curve", &format!("{}.sensitivity", AXIS_NAMES[i]), gain);
                }
            }
            ui.add_space(8.0);
            let mut exp_val = exp;
            if param_cell(ui, "Center Stability (Exp)", &mut exp_val, 1.0..=3.0, 2) {
                for &i in group_axes {
                    self.axis_curve[i].exponent = exp_val;
                    self.send_set("response-curve", &format!("{}.exponent", AXIS_NAMES[i]), exp_val);
                }
            }
        });

        ui.add_space(8.0);

        ui.horizontal(|ui| {
            if ui
                .add(
                    egui::Button::new(egui::RichText::new("Reset Curve").size(13.0))
                        .min_size(egui::vec2(110.0, 28.0)),
                )
                .clicked()
            {
                let defaults = ProfileConfig::default();
                for &i in group_axes {
                    self.axis_curve[i].sensitivity = defaults.curve[i].sensitivity;
                    self.axis_curve[i].exponent    = defaults.curve[i].exponent;
                    self.send_set("response-curve", &format!("{}.sensitivity", AXIS_NAMES[i]), defaults.curve[i].sensitivity);
                    self.send_set("response-curve", &format!("{}.exponent",    AXIS_NAMES[i]), defaults.curve[i].exponent);
                }
            }
            if ui
                .add(
                    egui::Button::new(egui::RichText::new("Recenter").size(13.0))
                        .min_size(egui::vec2(90.0, 28.0)),
                )
                .clicked()
            {
                self.cmd.send(EngineCmd::Recenter);
            }
        });
    }

    // ── Profiles ─────────────────────────────────────────────────────────
    fn ui_profiles(&mut self, ui: &mut egui::Ui) {
        ui.heading("Profiles");
        ui.label(egui::RichText::new("Save and load tracking configurations.").weak());
        ui.add_space(8.0);

        // ── Active Profile ────────────────────────────────────────────────
        section(ui, "Active Profile");

        ui.horizontal(|ui| {
            ui.label("Name:");
            ui.add(egui::TextEdit::singleline(&mut self.profile_name).hint_text("Profile name…"));
        });
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui.button("Save Profile").clicked() {
                let cfg = self.to_profile();
                let filename = cfg.name.replace(|c: char| !c.is_alphanumeric() && c != '-' && c != '_', "_");
                let path = ProfileConfig::profiles_dir().join(format!("{filename}.toml"));
                match cfg.save(&path) {
                    Ok(()) => {
                        let _ = cfg.save(&ProfileConfig::active_config_path());
                        self.profile_status = format!("Saved to {}", path.display());
                        self.profiles_list  = ProfileConfig::list_profiles();
                    }
                    Err(e) => self.profile_status = format!("Save failed: {e}"),
                }
            }
            if ui.button("Reset to Defaults").clicked() {
                let defaults = ProfileConfig::default();
                self.apply_profile(&defaults);
                self.profile_status = "Reset to defaults".into();
            }
        });
        if !self.profile_status.is_empty() {
            ui.add_space(4.0);
            ui.label(egui::RichText::new(&self.profile_status).weak().small());
        }

        // ── Saved Profiles ────────────────────────────────────────────────
        section(ui, "Saved Profiles");

        ui.horizontal(|ui| {
            if ui.small_button("↺ Refresh").clicked() {
                self.profiles_list = ProfileConfig::list_profiles();
            }
        });
        ui.add_space(4.0);

        if self.profiles_list.is_empty() {
            ui.label(
                egui::RichText::new("No saved profiles. Save your current settings above.")
                    .weak().italics(),
            );
        } else {
            let mut load_path:   Option<std::path::PathBuf> = None;
            let mut delete_path: Option<std::path::PathBuf> = None;

            egui::Grid::new("saved_profiles")
                .striped(true)
                .spacing([12.0, 4.0])
                .show(ui, |ui| {
                    for (name, path) in &self.profiles_list {
                        let is_active = *name == self.profile_name;
                        let label_text = egui::RichText::new(name)
                            .color(if is_active { colors::ORANGE } else { egui::Color32::from_gray(200) });
                        let label_text = if is_active { label_text.strong() } else { label_text };
                        ui.label(label_text);
                        if ui.small_button("Load").clicked()   { load_path   = Some(path.clone()); }
                        if ui.small_button("Delete").clicked() { delete_path = Some(path.clone()); }
                        ui.end_row();
                    }
                });

            if let Some(path) = load_path {
                match ProfileConfig::load(&path) {
                    Ok(cfg) => {
                        self.apply_profile(&cfg);
                        let _ = cfg.save(&ProfileConfig::active_config_path());
                        self.profile_status = format!("Loaded: {}", cfg.name);
                    }
                    Err(e) => self.profile_status = format!("Load failed: {e}"),
                }
            }
            if let Some(path) = delete_path {
                let _ = std::fs::remove_file(&path);
                self.profiles_list = ProfileConfig::list_profiles();
                self.profile_status = "Profile deleted".into();
            }
        }

        // ── Process Auto-Switch (future) ──────────────────────────────────
        section(ui, "Process Auto-Switch");
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("[Phase 4]").small().color(colors::ORANGE_DIM));
            ui.label(
                egui::RichText::new("Automatically load a profile when a game is detected — planned for a future release.")
                    .weak().small().italics(),
            );
        });
    }

    // ── Camera controls ────────────────────────────────────────────────────────
    fn ui_camera(&mut self, ui: &mut egui::Ui) {
        ui.heading("Camera Controls");
        ui.label(egui::RichText::new("Hardware settings apply to the live tracking capture — not just the preview.").weak());
        ui.add_space(8.0);

        // ── Hardware ──────────────────────────────────────────────────────
        section(ui, "Hardware");

        egui::Grid::new("camera_hw_grid")
            .spacing([8.0, 8.0])
            .min_col_width(130.0)
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Device").strong());
                ui.horizontal(|ui| {
                    let selected_label = self.cameras
                        .iter()
                        .find(|c| c.index == self.selected_camera)
                        .map(|c| c.label())
                        .unwrap_or_else(|| format!("/dev/video{}", self.selected_camera));

                    egui::ComboBox::from_id_salt("camera_select")
                        .width(300.0)
                        .selected_text(&selected_label)
                        .show_ui(ui, |ui| {
                            for cam in &self.cameras {
                                if ui.selectable_label(
                                    cam.index == self.selected_camera,
                                    cam.label(),
                                ).clicked() {
                                    self.selected_camera = cam.index;
                                    self.v4l2_last_camera = u32::MAX;
                                    self.cmd.send(EngineCmd::SwitchCamera { index: cam.index });
                                }
                            }
                        });
                    if ui.small_button("↺ Refresh").clicked() {
                        self.cameras = enumerate_cameras();
                    }
                });
                ui.end_row();

                ui.label(egui::RichText::new("FOV (diagonal)").strong());
                ui.horizontal(|ui| {
                    if ui.small_button("−").clicked() {
                        self.camera_fov = (self.camera_fov - 1.0).max(20.0);
                    }
                    ui.add(
                        egui::DragValue::new(&mut self.camera_fov)
                            .range(20.0..=150.0)
                            .speed(0.0)
                            .suffix("°")
                            .fixed_decimals(0),
                    );
                    if ui.small_button("+").clicked() {
                        self.camera_fov = (self.camera_fov + 1.0).min(150.0);
                    }
                    ui.add_space(4.0);
                    if ui.button("Apply").clicked() {
                        self.cmd.send(EngineCmd::SetFov { degrees: self.camera_fov });
                    }
                    ui.label(egui::RichText::new("Affects depth (Z) estimation accuracy").weak().small().italics());
                });
                ui.end_row();

                ui.label(egui::RichText::new("FOV Presets").strong());
                ui.horizontal(|ui| {
                    let mut cam_cfg = CameraConfig::load();
                    let preset_label = if cam_cfg.fov_presets.is_empty() {
                        "— no presets —".to_owned()
                    } else {
                        format!("{} preset(s)", cam_cfg.fov_presets.len())
                    };
                    egui::ComboBox::from_id_salt("fov_presets")
                        .width(180.0)
                        .selected_text(preset_label)
                        .show_ui(ui, |ui| {
                            let mut apply = None;
                            let mut delete = None;
                            for (i, p) in cam_cfg.fov_presets.iter().enumerate() {
                                ui.horizontal(|ui| {
                                    if ui.selectable_label(false, format!("{} ({}°)", p.name, p.fov as i32)).clicked() {
                                        apply = Some(i);
                                    }
                                    if ui.small_button("✕").clicked() {
                                        delete = Some(i);
                                    }
                                });
                            }
                            if let Some(i) = apply {
                                self.camera_fov = cam_cfg.fov_presets[i].fov;
                                self.cmd.send(EngineCmd::SetFov { degrees: self.camera_fov });
                            }
                            if let Some(i) = delete {
                                cam_cfg.fov_presets.remove(i);
                                let _ = cam_cfg.save();
                            }
                        });
                    ui.add_space(4.0);
                    ui.add(
                        egui::TextEdit::singleline(&mut self.fov_preset_name)
                            .hint_text("name…")
                            .desired_width(100.0),
                    );
                    if ui.button("Save").clicked() {
                        let name = self.fov_preset_name.trim().to_owned();
                        if !name.is_empty() {
                            let mut cfg = CameraConfig::load();
                            if let Some(p) = cfg.fov_presets.iter_mut().find(|p| p.name == name) {
                                p.fov = self.camera_fov;
                            } else {
                                cfg.fov_presets.push(FovPreset { name, fov: self.camera_fov });
                            }
                            let _ = cfg.save();
                            self.fov_preset_name.clear();
                        }
                    }
                });
                ui.end_row();

                ui.label(egui::RichText::new("Resolution").strong());
                ui.horizontal(|ui| {
                    let current_label = CAMERA_RESOLUTIONS[self.camera_res_idx].2;
                    egui::ComboBox::from_id_salt("camera_resolution")
                        .width(280.0)
                        .selected_text(current_label)
                        .show_ui(ui, |ui| {
                            for (i, (w, h, label)) in CAMERA_RESOLUTIONS.iter().enumerate() {
                                if ui.selectable_label(self.camera_res_idx == i, *label).clicked() {
                                    self.camera_res_idx = i;
                                    self.cmd.send(EngineCmd::SetResolution { width: *w, height: *h });
                                }
                            }
                        });
                    ui.label(egui::RichText::new("Restarts camera — takes ~1 s").weak().small().italics());
                });
                ui.end_row();
            });

        // ── V4L2 Hardware Controls ─────────────────────────────────────────
        section(ui, "V4L2 Hardware Controls");

        // Re-open control fd when camera changes
        if self.v4l2_last_camera != self.selected_camera {
            self.v4l2_last_camera = self.selected_camera;
            self.v4l2_optimize_status = None;
            if let Ok(dev) = v4l::Device::with_path(format!("/dev/video{}", self.selected_camera)) {
                if let Ok(controls) = dev.query_controls() {
                    self.v4l2_controls = controls;
                }
                self.v4l2_device = Some(dev);
            } else {
                self.v4l2_device = None;
                self.v4l2_controls.clear();
            }
        }

        // Optimize button
        ui.horizontal(|ui| {
            let btn = ui.add(
                egui::Button::new(
                    egui::RichText::new("⚡  Optimize for Tracking").color(colors::ORANGE)
                )
            );
            if btn.clicked() {
                if let Some(dev) = &self.v4l2_device {
                    let mut applied = 0u32;
                    for desc in &self.v4l2_controls {
                        let n = desc.name.to_lowercase();
                        if n.contains("exposure") && n.contains("priority") {
                            if dev.set_control(v4l::control::Control {
                                id: desc.id,
                                value: v4l::control::Value::Boolean(false),
                            }).is_ok() { applied += 1; }
                        }
                        if n == "auto exposure" || n == "exposure, auto" {
                            if dev.set_control(v4l::control::Control {
                                id: desc.id,
                                value: v4l::control::Value::Integer(1),
                            }).is_ok() { applied += 1; }
                        }
                    }
                    if let Ok(controls) = dev.query_controls() {
                        self.v4l2_controls = controls;
                    }
                    self.v4l2_optimize_status = Some((
                        if applied > 0 {
                            format!("✔  Applied ({applied} controls set) — manual exposure active. Raise Gain if dark.")
                        } else {
                            "⚠  No exposure controls found on this camera.".into()
                        },
                        std::time::Instant::now(),
                    ));
                }
            }
            ui.label(
                egui::RichText::new("Disables auto-exposure priority to prevent FPS drops in low light.")
                    .weak().small().italics()
            );
        });

        // Fading status
        if let Some((msg, t)) = &self.v4l2_optimize_status {
            let age = t.elapsed().as_secs_f32();
            if age < 8.0 {
                let alpha = ((1.0 - age / 8.0) * 255.0) as u8;
                let col = if msg.starts_with('✔') {
                    egui::Color32::from_rgba_unmultiplied(80, 220, 100, alpha)
                } else {
                    egui::Color32::from_rgba_unmultiplied(220, 180, 50, alpha)
                };
                ui.label(egui::RichText::new(msg).color(col).small());
                ui.ctx().request_repaint();
            } else {
                self.v4l2_optimize_status = None;
            }
        }

        ui.add_space(6.0);

        if self.v4l2_controls.is_empty() {
            ui.label(egui::RichText::new("No V4L2 controls exposed by this device driver.").weak());
            return;
        }

        // Pre-collect controls into sections so we can render each section
        // as its own two-column Grid — guarantees all sliders start at the
        // same x regardless of label length.
        let sections: Vec<(Option<String>, Vec<usize>)> = {
            let mut out: Vec<(Option<String>, Vec<usize>)> = vec![(None, vec![])];
            for (i, desc) in self.v4l2_controls.iter().enumerate() {
                if desc.typ == v4l::control::Type::CtrlClass {
                    out.push((Some(desc.name.clone()), vec![]));
                } else {
                    out.last_mut().unwrap().1.push(i);
                }
            }
            out
        };

        let Some(dev) = &self.v4l2_device else { return };

        for (section_name, indices) in &sections {
            if indices.is_empty() { continue; }

            if let Some(name) = section_name {
                section(ui, name);
            }

            egui::Grid::new(section_name.as_deref().unwrap_or("v4l2_top"))
                .spacing([12.0, 5.0])
                .min_col_width(170.0)
                .max_col_width(200.0)
                .show(ui, |ui| {
                    for &i in indices {
                        let desc = &self.v4l2_controls[i];

                        // Label column — fixed width, right-aligned text
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| { ui.label(&desc.name); },
                        );

                        // Control column
                        if let Ok(val) = dev.control(desc.id) {
                            match desc.typ {
                                v4l::control::Type::Integer => {
                                    if let v4l::control::Value::Integer(mut current) = val.value {
                                        ui.horizontal(|ui| {
                                            let changed_s = ui.add(
                                                egui::Slider::new(&mut current, desc.minimum..=desc.maximum)
                                                    .show_value(false)
                                            ).changed();
                                            let changed_d = ui.add(
                                                egui::DragValue::new(&mut current)
                                                    .range(desc.minimum..=desc.maximum)
                                                    .speed(1)
                                            ).changed();
                                            if changed_s || changed_d {
                                                let _ = dev.set_control(v4l::control::Control {
                                                    id: desc.id,
                                                    value: v4l::control::Value::Integer(current),
                                                });
                                            }
                                        });
                                    }
                                }
                                v4l::control::Type::Boolean => {
                                    if let v4l::control::Value::Boolean(mut current) = val.value {
                                        if ui.checkbox(&mut current, "").changed() {
                                            let _ = dev.set_control(v4l::control::Control {
                                                id: desc.id,
                                                value: v4l::control::Value::Boolean(current),
                                            });
                                        }
                                    }
                                }
                                v4l::control::Type::Menu => {
                                    if let v4l::control::Value::Integer(mut current) = val.value {
                                        if let Some(items) = &desc.items {
                                            let label = items.iter()
                                                .find(|(idx, _)| *idx as i64 == current)
                                                .map(|(_, item)| match item {
                                                    v4l::control::MenuItem::Name(s) => s.clone(),
                                                    v4l::control::MenuItem::Value(v) => v.to_string(),
                                                })
                                                .unwrap_or_else(|| format!("{current}"));
                                            egui::ComboBox::from_id_salt(desc.id)
                                                .width(160.0)
                                                .selected_text(label)
                                                .show_ui(ui, |ui| {
                                                    for (idx, item) in items {
                                                        let name = match item {
                                                            v4l::control::MenuItem::Name(s) => s.clone(),
                                                            v4l::control::MenuItem::Value(v) => v.to_string(),
                                                        };
                                                        if ui.selectable_value(&mut current, *idx as i64, &name).changed() {
                                                            let _ = dev.set_control(v4l::control::Control {
                                                                id: desc.id,
                                                                value: v4l::control::Value::Integer(current),
                                                            });
                                                        }
                                                    }
                                                });
                                        }
                                    }
                                }
                                _ => { ui.label(""); }
                            }
                        } else {
                            ui.label(egui::RichText::new("—").weak());
                        }

                        ui.end_row();
                    }
                });

            ui.add_space(4.0);
        }
    }

    // ── Settings ─────────────────────────────────────────────────────────
    fn ui_settings(&mut self, ui: &mut egui::Ui) {
        ui.heading("Settings");
        ui.add_space(8.0);

        // ── Application ───────────────────────────────────────────────────
        section(ui, "Application");

        egui::Grid::new("app_settings_grid")
            .spacing([8.0, 6.0])
            .min_col_width(100.0)
            .show(ui, |ui| {
                ui.label("Window close:");
                if ui.checkbox(&mut self.minimize_to_tray, "Minimize to tray").changed() {
                    let mut cfg = CameraConfig::load();
                    cfg.minimize_to_tray = self.minimize_to_tray;
                    let _ = cfg.save();
                }
                ui.end_row();
            });

        // ── Pose Model ────────────────────────────────────────────────────
        section(ui, "Pose Model");

        egui::Grid::new("model_grid")
            .spacing([8.0, 6.0])
            .min_col_width(100.0)
            .show(ui, |ui| {
                ui.label("Model:");
                ui.horizontal(|ui| {
                    ui.label("head-pose-0.4-small (13MB)");
                    ui.label(egui::RichText::new("Additional models — Phase 4").weak().small().italics());
                });
                ui.end_row();
                ui.label("Localizer:");
                ui.label("head-localizer.onnx (273K)");
                ui.end_row();
            });

        // ── Output Plugins ────────────────────────────────────────────────
        section(ui, "X-Plane 12 Plugin");

        let so_path = find_xplane_plugin_so();
        egui::Grid::new("xplane_install_grid")
            .spacing([8.0, 5.0])
            .min_col_width(100.0)
            .show(ui, |ui| {
                ui.label("X-Plane 12 folder:");
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.xplane_path)
                            .desired_width(240.0)
                            .hint_text("e.g. /home/user/X-Plane 12"),
                    );
                    if ui.small_button("Browse…").clicked() {
                        let start = if std::path::Path::new(&self.xplane_path).is_dir() {
                            self.xplane_path.clone()
                        } else {
                            std::env::var("HOME").unwrap_or_default()
                        };
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title("Select X-Plane 12 folder")
                            .set_directory(start)
                            .pick_folder()
                        {
                            self.xplane_path = folder.display().to_string();
                        }
                    }
                });
                ui.end_row();

                ui.label("Plugin binary:");
                match &so_path {
                    Some(p) => { ui.label(egui::RichText::new(p.display().to_string()).small().weak()); }
                    None => {
                        ui.label(egui::RichText::new(
                            "Not found — run: cargo build -p headtrack-xplane --release"
                        ).small().color(egui::Color32::from_rgb(210, 110, 60)));
                    }
                }
                ui.end_row();
            });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let can_install = so_path.is_some() && !self.xplane_path.trim().is_empty();
            if ui.add_enabled(can_install, egui::Button::new("Install X-Plane 12 Plugin").min_size(egui::vec2(200.0, 28.0))).clicked() {
                let xp = std::path::Path::new(self.xplane_path.trim());
                let plugin_dir = xp.join("Resources/plugins/headtrack/64");
                let result = std::fs::create_dir_all(&plugin_dir).and_then(|_| {
                    std::fs::copy(so_path.as_ref().unwrap(), plugin_dir.join("lin.xpl")).map(|_| ())
                });
                self.xplane_install_status = Some(match result {
                    Ok(()) => (format!("Installed → {}/Resources/plugins/headtrack/64/lin.xpl", self.xplane_path.trim()), true),
                    Err(e) => (format!("Error: {e}"), false),
                });
            }
            if let Some((msg, ok)) = &self.xplane_install_status {
                ui.label(egui::RichText::new(msg).small().color(if *ok { COLOR_CONNECTED } else { COLOR_DISCONNECTED }));
            }
        });

        section(ui, "Star Citizen / Wine Bridge");

        let dll_path = find_npclient_dll();
        let bin64 = sc_bin64_path(self.sc_prefix_path.trim());
        let bin64_ok = bin64.is_dir();

        egui::Grid::new("sc_install_grid")
            .spacing([8.0, 5.0])
            .min_col_width(100.0)
            .show(ui, |ui| {
                ui.label("Wine prefix:");
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.sc_prefix_path)
                            .desired_width(240.0)
                            .hint_text("e.g. /home/user/Games/star-citizen"),
                    );
                    if ui.small_button("Browse…").clicked() {
                        let start = if std::path::Path::new(&self.sc_prefix_path).is_dir() {
                            self.sc_prefix_path.clone()
                        } else {
                            std::env::var("HOME").unwrap_or_default()
                        };
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title("Select Star Citizen Wine prefix folder")
                            .set_directory(start)
                            .pick_folder()
                        {
                            self.sc_prefix_path = folder.display().to_string();
                        }
                    }
                });
                ui.end_row();

                ui.label("SC Bin64:");
                if bin64_ok {
                    ui.label(egui::RichText::new(bin64.display().to_string()).small().weak());
                } else {
                    ui.label(egui::RichText::new("Not found — check Wine prefix path above").small().color(egui::Color32::from_rgb(210, 110, 60)));
                }
                ui.end_row();

                ui.label("NPClient DLL:");
                match &dll_path {
                    Some(p) => { ui.label(egui::RichText::new(p.display().to_string()).small().weak()); }
                    None => {
                        ui.label(egui::RichText::new(
                            "Not found — run: cargo build -p headtrack-npclient --target x86_64-pc-windows-gnu --release"
                        ).small().color(egui::Color32::from_rgb(210, 110, 60)));
                    }
                }
                ui.end_row();
            });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let can_setup = dll_path.is_some() && bin64_ok && !self.sc_prefix_path.trim().is_empty();
            if ui.add_enabled(can_setup, egui::Button::new("Setup Wine Bridge").min_size(egui::vec2(200.0, 28.0))).clicked() {
                let prefix = self.sc_prefix_path.trim().to_string();
                let dll_src = dll_path.as_ref().unwrap();
                let copy_result = std::fs::copy(dll_src, bin64.join("NPClient64.dll"));
                let result = match copy_result {
                    Err(e) => Err(format!("DLL copy failed: {e}")),
                    Ok(_) => {
                        let reg_key = r"HKCU\Software\NaturalPoint\NATURALPOINT\NPClient Location";
                        let reg_val = r"C:\Program Files\Roberts Space Industries\StarCitizen\LIVE\Bin64\";
                        std::process::Command::new("wine")
                            .env("WINEPREFIX", &prefix)
                            .args(["reg", "add", reg_key, "/v", "Path", "/t", "REG_SZ", "/d", reg_val, "/f"])
                            .output()
                            .map_err(|e| format!("wine not found: {e}"))
                            .and_then(|out| {
                                if out.status.success() { Ok(()) }
                                else { Err(format!("wine reg add failed: {}", String::from_utf8_lossy(&out.stderr).trim())) }
                            })
                    }
                };
                self.sc_install_status = Some(match result {
                    Ok(()) => ("NPClient64.dll installed + registry key set. Enable TrackIR in SC settings.".to_string(), true),
                    Err(e) => (e, false),
                });
            }
            if let Some((msg, ok)) = &self.sc_install_status {
                ui.label(egui::RichText::new(msg).small().color(if *ok { COLOR_CONNECTED } else { COLOR_DISCONNECTED }));
            }
        });

        // ── IPC ───────────────────────────────────────────────────────────
        section(ui, "IPC");

        ui.horizontal(|ui| {
            ui.label("Pose socket:");
            ui.label(egui::RichText::new(&self.ipc_socket_label).monospace().small());
            if ui.small_button("Copy").clicked() {
                ui.ctx().copy_text(self.ipc_socket_label.clone());
            }
        });
    }
    // ── Dev Tools ────────────────────────────────────────────────────────
    fn ui_dev(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        ui.heading("Dev Tools");
        ui.label(
            egui::RichText::new("Per-axis pipeline controls. Extreme values may cause motion sickness.")
                .weak(),
        );
        ui.add_space(4.0);

        // ── Recenter at top ──────────────────────────────────────────────
        if ui
            .add(
                egui::Button::new(egui::RichText::new("Recenter").size(14.0))
                    .min_size(egui::vec2(120.0, 32.0)),
            )
            .clicked()
        {
            self.cmd.send(EngineCmd::Recenter);
        }

        // ── Live Axes — unified selector + display ───────────────────────
        section(ui, "Live Axes");
        ui.label(
            egui::RichText::new("Click a row to select — each axis has independent curve, filter, and deadzone.")
                .weak().small(),
        );
        ui.add_space(2.0);

        for i in 0..6 {
            let is_selected = self.dev_selected_axis == i;
            let active      = self.axis_active[i];
            let value       = pose_vals[i];

            let row_resp = ui.horizontal(|ui| {
                // Fixed-width name column so all bars start at the same x
                let name_btn = egui::Button::new(
                    egui::RichText::new(AXIS_LABELS[i])
                        .color(if active { AXIS_COLORS[i] } else { egui::Color32::from_gray(100) })
                        .monospace(),
                )
                .fill(egui::Color32::TRANSPARENT)
                .frame(false);
                if ui.add_sized([46.0, 20.0], name_btn).clicked() {
                    self.dev_selected_axis = i;
                }

                // Fixed-width value column ("−41.8mm" is the widest case)
                ui.add_sized(
                    [72.0, 20.0],
                    egui::Label::new(
                        egui::RichText::new(format!("{value:+6.1}{}", AXIS_UNITS[i])).monospace(),
                    ),
                );
                draw_bar(ui, value, axis_max(i), active, 160.0);

                // Enable/disable toggle
                if ui
                    .add(
                        egui::Button::new(
                            egui::RichText::new(if active { "◉" } else { "○" })
                                .color(if active { AXIS_COLORS[i] } else { egui::Color32::from_gray(60) })
                                .small(),
                        )
                        .fill(egui::Color32::TRANSPARENT)
                        .frame(false),
                    )
                    .clicked()
                {
                    self.axis_active[i] = !self.axis_active[i];
                    let val = if self.axis_active[i] { 1.0 } else { 0.0 };
                    self.send_set("axis-mask", AXIS_NAMES[i], val);
                }
            });

            // Orange left accent on the selected row
            if is_selected {
                let r = row_resp.response.rect;
                let accent = egui::Rect::from_min_max(
                    egui::pos2(r.left(), r.top() + 2.0),
                    egui::pos2(r.left() + 3.0, r.bottom() - 2.0),
                );
                ui.painter().rect_filled(accent, 1.5, colors::ORANGE);
            }
            ui.add_space(1.0);
        }

        // ── Axis Editor ──────────────────────────────────────────────────
        let i = self.dev_selected_axis;
        let is_rotation = i < 3;

        section(ui, &format!("Axis Editor — {}", AXIS_LABELS[i]));

        let avail = ui.available_width().min(560.0);
        draw_curve_editor(
            ui,
            &[(self.axis_curve[i].sensitivity, self.axis_curve[i].exponent, pose_vals[i], AXIS_COLORS[i])],
            is_rotation,
            avail,
            150.0,
        );

        ui.add_space(6.0);

        ui.horizontal(|ui| {
            if param_cell(ui, "Responsiveness (Gain)", &mut self.axis_curve[i].sensitivity, 0.1..=5.0, 2) {
                self.send_set("response-curve", &format!("{}.sensitivity", AXIS_NAMES[i]), self.axis_curve[i].sensitivity);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Center Stability (Exp)", &mut self.axis_curve[i].exponent, 0.3..=3.0, 2) {
                self.send_set("response-curve", &format!("{}.exponent", AXIS_NAMES[i]), self.axis_curve[i].exponent);
            }
        });

        ui.add_space(4.0);

        ui.horizontal(|ui| {
            if param_cell(ui, "Still (min_cutoff)", &mut self.axis_filter[i].min_cutoff, 0.001..=5.0, 3) {
                self.send_set("one-euro-filter", &format!("{}.min_cutoff", AXIS_NAMES[i]), self.axis_filter[i].min_cutoff);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Speed (beta)", &mut self.axis_filter[i].beta, 0.001..=5.0, 3) {
                self.send_set("one-euro-filter", &format!("{}.beta", AXIS_NAMES[i]), self.axis_filter[i].beta);
            }
        });

        ui.add_space(4.0);

        let dz_max = if is_rotation { 5.0_f32 } else { 20.0 };
        if param_cell(
            ui,
            if is_rotation { "Deadzone (deg)" } else { "Deadzone (mm)" },
            &mut self.deadzone[i],
            0.0..=dz_max,
            1,
        ) {
            self.send_set("deadzone", AXIS_NAMES[i], self.deadzone[i]);
        }

        // ── Global Pipeline Parameters ────────────────────────────────────
        section(ui, "Cross-Axis Compensation");
        ui.label(
            egui::RichText::new("Cancels bleed between axes (e.g. yaw causing phantom pitch drift)")
                .weak().small(),
        );

        ui.horizontal(|ui| {
            if param_cell(ui, "Yaw -> Pitch", &mut self.yaw_to_pitch, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "yaw_to_pitch", self.yaw_to_pitch);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Yaw -> X", &mut self.yaw_to_x, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "yaw_to_x", self.yaw_to_x);
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if param_cell(ui, "Yaw -> Y", &mut self.yaw_to_y, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "yaw_to_y", self.yaw_to_y);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Yaw -> Roll", &mut self.yaw_to_roll, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "yaw_to_roll", self.yaw_to_roll);
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if param_cell(ui, "Pitch -> Y", &mut self.pitch_to_y, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "pitch_to_y", self.pitch_to_y);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Z -> Y", &mut self.z_to_y, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "z_to_y", self.z_to_y);
            }
        });
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if param_cell(ui, "Z -> Pitch", &mut self.z_to_pitch, -1.0..=1.0, 2) {
                self.send_set("cross-axis", "z_to_pitch", self.z_to_pitch);
            }
            // intentional spacer — 7 params don't fill 4 pairs evenly
        });

        section(ui, "Prediction");
        ui.label(
            egui::RichText::new("Kalman filter + forward prediction to compensate system latency")
                .weak().small(),
        );

        if param_cell(ui, "Lookahead (ms)", &mut self.predict_ms, 0.0..=100.0, 0) {
            self.send_set("prediction", "predict_ms", self.predict_ms);
        }

        ui.add_space(4.0);
        ui.label(egui::RichText::new("Rotation (yaw/pitch/roll)").weak().small());
        ui.horizontal(|ui| {
            if param_cell(ui, "Process Q", &mut self.rot_process_noise, 0.1..=50.0, 1) {
                self.send_set("prediction", "rot_process_noise", self.rot_process_noise);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Measure R", &mut self.rot_measurement_noise, 0.1..=50.0, 1) {
                self.send_set("prediction", "rot_measurement_noise", self.rot_measurement_noise);
            }
        });

        ui.add_space(4.0);
        ui.label(egui::RichText::new("Translation (x/y/z)").weak().small());
        ui.horizontal(|ui| {
            if param_cell(ui, "Process Q", &mut self.pos_process_noise, 0.1..=100.0, 1) {
                self.send_set("prediction", "pos_process_noise", self.pos_process_noise);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Measure R", &mut self.pos_measurement_noise, 0.1..=100.0, 1) {
                self.send_set("prediction", "pos_measurement_noise", self.pos_measurement_noise);
            }
        });

        section(ui, "Center Stage");
        ui.label(
            egui::RichText::new("Slowly drifts the neutral point back to physical center while you hold still — prevents pose from creeping over a long session.")
                .weak().small(),
        );

        ui.horizontal(|ui| {
            if param_cell(ui, "Drift Correction (/s)", &mut self.drift_rate, 0.0..=0.2, 3) {
                self.send_set("center", "drift_rate", self.drift_rate);
            }
            ui.add_space(8.0);
            if param_cell(ui, "Z Drift Multiplier", &mut self.z_drift_mult, 1.0..=10.0, 1) {
                self.send_set("center", "z.drift_mult", self.z_drift_mult);
            }
        });

        section(ui, "Diagnostics");
        ui.label(
            egui::RichText::new(
                "Record pose + parameter changes to a CSV file for tuning analysis or bug reports."
            )
            .weak()
            .small(),
        );
        ui.add_space(4.0);

        let status = RecordingStatus::from_u8(
            self.state.recording_status.load(Ordering::Relaxed)
        );

        match status {
            RecordingStatus::Idle => {
                ui.horizontal(|ui| {
                    ui.label("Size limit:");
                    ui.add(
                        egui::DragValue::new(&mut self.diag_size_limit_mb)
                            .range(1..=500)
                            .suffix(" MB"),
                    );
                });
                ui.add_space(4.0);
                if ui
                    .add(
                        egui::Button::new(
                            egui::RichText::new("\u{25cf}  Record").color(egui::Color32::from_gray(220)),
                        )
                        .min_size(egui::vec2(90.0, 28.0)),
                    )
                    .clicked()
                {
                    let path = diag_default_path();
                    self.diag_recording_path = path.clone();
                    self.diag_recording_start = Some(std::time::Instant::now());
                    self.cmd.send(EngineCmd::StartRecording {
                        path,
                        size_limit_mb: self.diag_size_limit_mb,
                    });
                }
            }

            RecordingStatus::Active => {
                let elapsed = self.diag_recording_start
                    .map(|s| s.elapsed().as_secs())
                    .unwrap_or(0);
                let bytes = self.state.recording_bytes.load(Ordering::Relaxed);
                let limit_bytes = self.diag_size_limit_mb as u64 * 1024 * 1024;
                let frac = (bytes as f32 / limit_bytes as f32).min(1.0);

                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("\u{25a0}  Stop")
                                    .color(colors::ORANGE),
                            )
                            .min_size(egui::vec2(90.0, 28.0)),
                        )
                        .clicked()
                    {
                        self.diag_recording_start = None;
                        self.cmd.send(EngineCmd::StopRecording);
                    }
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new(
                        format!("{:02}:{:02}", elapsed / 60, elapsed % 60)
                    ).monospace());
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new(
                        format!("{:.1} / {} MB", bytes as f64 / (1024.0 * 1024.0), self.diag_size_limit_mb)
                    ).small().weak());
                });

                ui.add_space(2.0);
                ui.add(
                    egui::ProgressBar::new(frac)
                        .desired_width(320.0)
                        .fill(colors::ORANGE_DIM),
                );
                ui.add_space(2.0);

                let path_str = self.diag_recording_path.display().to_string();
                ui.label(egui::RichText::new(&path_str).small().weak().monospace());
            }

            RecordingStatus::LimitReached => {
                self.diag_recording_start = None;
                ui.horizontal(|ui| {
                    ui.label(
                        egui::RichText::new("\u{26a0}  Recording stopped — size limit reached")
                            .color(colors::ORANGE)
                            .small(),
                    );
                });
                let path_str = self.diag_recording_path.display().to_string();
                ui.label(egui::RichText::new(format!("Saved: {path_str}")).small().weak().monospace());
                ui.add_space(4.0);
                if ui.small_button("Dismiss").clicked() {
                    self.cmd.send(EngineCmd::StopRecording);
                }
            }
        }
    }
}

fn diag_default_path() -> std::path::PathBuf {
    let home = std::env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    std::path::PathBuf::from(home).join(format!("headtrack-diag-{ts}.csv"))
}
