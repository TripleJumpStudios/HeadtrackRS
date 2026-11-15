use eframe::egui;
use headtrack_core::config::{CameraConfig, ProfileConfig};
use headtrack_core::input::{SourceKind, TobiiStatus};
use headtrack_core::pipeline::stage_names;
use headtrack_engine::{Engine, EngineCmd, EngineState};
use std::sync::atomic::AtomicBool;
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
        let Some(idx_str) = fname.strip_prefix("video") else {
            continue;
        };
        let Ok(index) = idx_str.parse::<u32>() else {
            continue;
        };

        let sysfs_index = std::fs::read_to_string(entry.path().join("index"))
            .ok()
            .and_then(|s| s.trim().parse::<u32>().ok())
            .unwrap_or(u32::MAX);
        if sysfs_index != 0 {
            continue;
        }

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
    // AppImage sets $APPDIR to the mount root; bridge/ lives at AppDir root
    if let Ok(appdir) = std::env::var("APPDIR") {
        let p = std::path::Path::new(&appdir).join("bridge/NPClient64.dll");
        if p.exists() {
            return Some(p);
        }
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            // Flat layout (next to exe)
            let p = dir.join("NPClient64.dll");
            if p.exists() {
                return Some(p);
            }
            // Walk up to find bridge/ (handles any exe depth inside AppDir)
            let mut d = dir.to_path_buf();
            for _ in 0..4 {
                let p = d.join("bridge/NPClient64.dll");
                if p.exists() {
                    return Some(p);
                }
                match d.parent() {
                    Some(parent) => d = parent.to_path_buf(),
                    None => break,
                }
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
        let Ok(contents) = std::fs::read_to_string(vdf) else {
            continue;
        };
        for line in contents.lines() {
            if let Some(p) = parse_vdf_path_line(line) {
                if !paths.iter().any(|existing| existing == &p) {
                    paths.push(p);
                }
            }
        }
    }
    paths
}

/// Extract the value from a Steam `libraryfolders.vdf` `"path"` line.
///
/// Lines look like: `\t\t"path"\t\t"/mnt/steam/SteamLibrary"`
///
/// Splitting on `"` yields `["\t\t", "path", "\t\t", "<value>", ""]`. Note this
/// must **not** use a bounded `splitn`: the bounded form puts the entire
/// remainder in the final field, so the value comes back with its closing quote
/// still attached and every derived path silently fails to exist. That was the
/// behaviour until 0.2.0, which is why Steam auto-detection never found a game
/// in a secondary library.
fn parse_vdf_path_line(line: &str) -> Option<String> {
    let parts: Vec<&str> = line.trim().split('"').collect();
    if parts.len() >= 4 && parts[1] == "path" && !parts[3].is_empty() {
        Some(parts[3].to_string())
    } else {
        None
    }
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

// ── MSFS 2024 SimConnect.xml setup ───────────────────────────────────────────

/// Steam app id for Microsoft Flight Simulator 2024.
const MSFS_2024_APPID: &str = "2537590";

/// The port HeadtrackRS expects MSFS to listen on.
const MSFS_SIMCONNECT_PORT: &str = "5557";

/// The `SimConnect.Comm` block MSFS needs in order to accept our TCP connection.
const SIMCONNECT_COMM_BLOCK: &str = r#"  <SimConnect.Comm>
    <Disabled>False</Disabled>
    <Protocol>IPv4</Protocol>
    <Scope>local</Scope>
    <Address>127.0.0.1</Address>
    <Port>5557</Port>
    <MaxClients>64</MaxClients>
    <MaxRecvSize>41088</MaxRecvSize>
    <DisableNagle>1</DisableNagle>
  </SimConnect.Comm>
"#;

/// A whole SimConnect.xml, for when the sim has not written one yet.
const SIMCONNECT_XML_FULL: &str = r#"<?xml version="1.0" encoding="Windows-1252"?>
<SimBase.Document Type="SimConnect" version="1,0">
<SimConnect.Comm>
    <Disabled>False</Disabled>
    <Protocol>IPv4</Protocol>
    <Scope>local</Scope>
    <Address>127.0.0.1</Address>
    <Port>5557</Port>
    <MaxClients>64</MaxClients>
    <MaxRecvSize>41088</MaxRecvSize>
    <DisableNagle>1</DisableNagle>
  </SimConnect.Comm>
</SimBase.Document>
"#;

/// Locate the MSFS 2024 Proton prefix directory that holds `SimConnect.xml`.
///
/// Steam installs the prefix under `compatdata/<appid>` in whichever library the
/// game lives in, so every library root is checked before the default locations.
fn detect_msfs_simconnect_dir() -> Option<std::path::PathBuf> {
    const SUFFIX: &str =
        "pfx/drive_c/users/steamuser/AppData/Roaming/Microsoft Flight Simulator 2024";
    let home = std::env::var("HOME").unwrap_or_default();

    let mut roots: Vec<String> = steam_library_paths();
    roots.push(format!("{home}/.steam/steam"));
    roots.push(format!("{home}/.local/share/Steam"));
    roots.push(format!(
        "{home}/.var/app/com.valvesoftware.Steam/.local/share/Steam"
    ));

    for root in roots {
        let p = std::path::Path::new(&root)
            .join("steamapps/compatdata")
            .join(MSFS_2024_APPID)
            .join(SUFFIX);
        if p.is_dir() {
            return Some(p);
        }
    }
    None
}

/// Outcome of a SimConnect.xml setup attempt, so the UI can say what happened
/// rather than just "ok".
#[derive(Debug)]
pub enum SimConnectSetup {
    /// The file already had a port-5557 entry; nothing was written.
    AlreadyConfigured,
    /// A new SimConnect.xml was created.
    Created(std::path::PathBuf),
    /// An existing file was amended; the original was copied to `backup`.
    Amended {
        path: std::path::PathBuf,
        backup: std::path::PathBuf,
    },
}

/// Ensure MSFS 2024's `SimConnect.xml` contains a port-5557 listener.
///
/// Safe to run repeatedly: it is a no-op once configured. MSFS can rewrite this
/// file during a major update, which silently breaks tracking, so re-running it
/// is the intended recovery — that is why it never replaces a file wholesale.
/// An existing document is *amended* (the new block is inserted before the
/// closing tag) and the original is kept alongside it, because a user may have
/// their own SimConnect entries for other tools.
fn install_simconnect_xml(dir: &std::path::Path) -> std::io::Result<SimConnectSetup> {
    let path = dir.join("SimConnect.xml");

    let existing = match std::fs::read_to_string(&path) {
        Ok(c) => c,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            std::fs::create_dir_all(dir)?;
            std::fs::write(&path, SIMCONNECT_XML_FULL)?;
            return Ok(SimConnectSetup::Created(path));
        }
        Err(e) => return Err(e),
    };

    // Already listening on our port? Leave it completely alone.
    if existing.contains(&format!("<Port>{MSFS_SIMCONNECT_PORT}</Port>")) {
        return Ok(SimConnectSetup::AlreadyConfigured);
    }

    let Some(close_at) = existing.rfind("</SimBase.Document>") else {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "SimConnect.xml exists but has no </SimBase.Document> closing tag — it may be corrupt. Move it aside and run this again to write a fresh one.",
        ));
    };

    let backup = dir.join(format!(
        "SimConnect.xml.headtrack-backup-{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0)
    ));
    std::fs::copy(&path, &backup)?;

    let mut amended = String::with_capacity(existing.len() + SIMCONNECT_COMM_BLOCK.len() + 1);
    amended.push_str(&existing[..close_at]);
    if !amended.ends_with('\n') {
        amended.push('\n');
    }
    amended.push_str(SIMCONNECT_COMM_BLOCK);
    amended.push_str(&existing[close_at..]);
    std::fs::write(&path, amended)?;

    Ok(SimConnectSetup::Amended { path, backup })
}

// ── System tray ──────────────────────────────────────────────────────────────

struct HeadtrackTray {
    show_flag: Arc<AtomicBool>,
    ctx_slot: Arc<std::sync::OnceLock<egui::Context>>,
}

impl ksni::Tray for HeadtrackTray {
    fn id(&self) -> String {
        "headtrack-rs".into()
    }
    fn title(&self) -> String {
        "HeadtrackRS".into()
    }
    fn icon_name(&self) -> String {
        String::new()
    }

    fn icon_pixmap(&self) -> Vec<ksni::Icon> {
        let icon = eframe::icon_data::from_png_bytes(include_bytes!(
            "../../../assets/icons/headtrack-rs.png"
        ))
        .expect("tray icon PNG");
        // StatusNotifierItem requires ARGB32 big-endian: bytes are [A, R, G, B] per pixel.
        let argb: Vec<u8> = icon
            .rgba
            .chunks(4)
            .flat_map(|p| [p[3], p[0], p[1], p[2]])
            .collect();
        vec![ksni::Icon {
            width: icon.width as i32,
            height: icon.height as i32,
            data: argb,
        }]
    }

    fn menu(&self) -> Vec<ksni::MenuItem<Self>> {
        use ksni::menu::{MenuItem, StandardItem};
        vec![
            MenuItem::Standard(StandardItem {
                label: "Show HeadtrackRS".into(),
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
    tracing_subscriber::fmt()
        .with_env_filter(headtrack_engine::log_filter("warn"))
        .init();

    // Engine::start() blocks ~2-3s for ONNX model load + camera init.
    // Spawn it in a background thread so the window appears immediately.
    let (engine_tx, engine_rx) = std::sync::mpsc::channel::<Engine>();
    std::thread::Builder::new()
        .name("engine-init".into())
        .spawn(move || match Engine::start() {
            Ok(engine) => {
                let _ = engine_tx.send(engine);
            }
            Err(e) => eprintln!("headtrack-rs: engine failed to start: {e}"),
        })
        .expect("failed to spawn engine-init thread");

    // System tray — spawns a background thread with its own async runtime.
    // assume_sni_available(true) routes SNI-watcher-not-found errors to
    // Tray::watcher_offline() instead of failing spawn; tray silently absent
    // on desktops without StatusNotifierItem support (GNOME w/o extension).
    let tray_show = Arc::new(AtomicBool::new(false));
    let tray_ctx: Arc<std::sync::OnceLock<egui::Context>> = Arc::new(std::sync::OnceLock::new());
    use ksni::blocking::TrayMethods;
    let _tray = HeadtrackTray {
        show_flag: Arc::clone(&tray_show),
        ctx_slot: Arc::clone(&tray_ctx),
    }
    .assume_sni_available(true)
    .spawn()
    .ok();

    // Background thread: while show_flag is pending, keep nudging the event loop
    // so update() runs even if the window is minimized and eframe has stopped repainting.
    {
        let show_flag = Arc::clone(&tray_show);
        let ctx_slot = Arc::clone(&tray_ctx);
        std::thread::Builder::new()
            .name("tray-repaint".into())
            .spawn(move || loop {
                std::thread::sleep(std::time::Duration::from_millis(50));
                if show_flag.load(Ordering::Relaxed) {
                    if let Some(ctx) = ctx_slot.get() {
                        ctx.request_repaint();
                    }
                }
            })
            .ok();
    }

    let icon =
        eframe::icon_data::from_png_bytes(include_bytes!("../../../assets/icons/headtrack-rs.png"))
            .expect("invalid window icon");

    // Restore the last window size. Validated and clamped by
    // `restored_window_size`, so a corrupt or stale value falls back to the
    // default rather than opening a window that cannot be used.
    let (win_w, win_h) = CameraConfig::load()
        .restored_window_size()
        .unwrap_or(CameraConfig::DEFAULT_WINDOW);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([win_w, win_h])
            .with_min_inner_size([CameraConfig::MIN_WINDOW.0, CameraConfig::MIN_WINDOW.1])
            .with_resizable(true)
            .with_title("HeadtrackRS")
            .with_app_id("headtrack-rs")
            .with_icon(icon),
        ..Default::default()
    };

    eframe::run_native(
        "HeadtrackRS",
        options,
        Box::new(move |cc| {
            ui::theme::install(&cc.egui_ctx);
            Ok(Box::new(AppWrapper::new(engine_rx, tray_show, tray_ctx)))
        }),
    )
}

// ── Constants ─────────────────────────────────────────────────────────────────

const AXIS_NAMES: [&str; 6] = ["yaw", "pitch", "roll", "x", "y", "z"];
const AXIS_LABELS: [&str; 6] = ["Yaw", "Pitch", "Roll", "X", "Y", "Z"];
const AXIS_UNITS: [&str; 6] = ["\u{00b0}", "\u{00b0}", "\u{00b0}", "mm", "mm", "mm"];
const ROT_MAX: f32 = 30.0;
const TRANS_MAX: f32 = 60.0;

/// Ordered list of capture resolution presets (width, height, display label).
/// Lower resolution = higher FPS / lower latency; higher = better localizer accuracy at distance.
/// `CAMERA_RESOLUTIONS[CAMERA_RES_DEFAULT_IDX]` is `CameraConfig::default_resolution()`
/// (1280×720, Q-UI6b) — the fallback when the saved resolution is not a preset.
const CAMERA_RES_DEFAULT_IDX: usize = 2;
const CAMERA_RESOLUTIONS: [(u32, u32, &str); 3] = [
    (320, 240, "Performance  320×240  (max FPS)"),
    (640, 480, "Balanced     640×480"),
    (1280, 720, "Detail       1280×720 (recommended)"),
];

fn axis_max(i: usize) -> f32 {
    if i < 3 {
        ROT_MAX
    } else {
        TRANS_MAX
    }
}

// ── Tabs ──────────────────────────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Dashboard,
    Tuning,
    Profiles,
    Camera,
    Tobii,
    Keybinds,
    Settings,
    Advanced,
}

impl Tab {
    fn label(&self) -> &'static str {
        match self {
            Tab::Dashboard => "Dashboard",
            Tab::Tuning => "Tuning",
            Tab::Profiles => "Profiles",
            Tab::Camera => "Camera",
            Tab::Tobii => "Tobii",
            Tab::Keybinds => "Keybinds",
            Tab::Settings => "Settings",
            Tab::Advanced => "Advanced",
        }
    }

    fn icon(&self) -> &'static str {
        use egui_phosphor::regular as ph;
        match self {
            Tab::Dashboard => ph::GAUGE,
            Tab::Tuning => ph::SLIDERS_HORIZONTAL,
            Tab::Profiles => ph::STACK,
            Tab::Camera => ph::CAMERA,
            Tab::Tobii => ph::EYE,
            Tab::Keybinds => ph::KEYBOARD,
            Tab::Settings => ph::GEAR_SIX,
            Tab::Advanced => ph::WRENCH,
        }
    }
}

/// Tabs above the sidebar separator — the everyday surface.
const PRIMARY_TABS: [Tab; 3] = [Tab::Dashboard, Tab::Tuning, Tab::Profiles];
/// Tabs below it — setup and, when revealed, the advanced surface.
const SECONDARY_TABS: [Tab; 4] = [Tab::Camera, Tab::Tobii, Tab::Keybinds, Tab::Settings];

/// Actions bindable to an in-app hotkey (fired only while the window is focused).
#[derive(PartialEq, Clone, Copy)]
enum HotkeyAction {
    Recenter,
    Toggle,
}

/// Tuning axis groups.
///
/// Grouped by **unit**, not by intuition: `Look` is the three rotations
/// (degrees) and `Move` the three translations (millimetres). The old split put
/// roll in with X/Y/Z, which was harmless while the only grouped controls were
/// dimensionless gain and exponent — but a grouped deadzone slider spanning
/// degrees and millimetres has no coherent range.
#[derive(PartialEq, Clone, Copy, Default)]
enum TuningGroup {
    #[default]
    Look,
    Move,
}

impl TuningGroup {
    fn axes(self) -> &'static [usize] {
        match self {
            TuningGroup::Look => &[0, 1, 2],
            TuningGroup::Move => &[3, 4, 5],
        }
    }

    /// Axis whose values seed the grouped sliders.
    fn representative(self) -> usize {
        match self {
            TuningGroup::Look => 0,
            TuningGroup::Move => 3,
        }
    }

    fn is_rotation(self) -> bool {
        self == TuningGroup::Look
    }
}

// ── App state ─────────────────────────────────────────────────────────────────

/// Where the GUI's commands go.
///
/// The running engine's sender, always, in the application. The headless
/// render tests draw every tab without an engine — `Engine::start` would bind
/// the live pose socket and open a camera — so they get a sink that drops
/// commands. Nothing a tab draws can tell the two apart.
#[derive(Clone)]
pub(crate) enum CmdSink {
    Engine(headtrack_engine::EngineCmdSender),
    #[cfg(test)]
    Null,
}

impl CmdSink {
    /// Send a command to the engine. Non-blocking; see `EngineCmdSender::send`.
    pub(crate) fn send(&self, cmd: EngineCmd) {
        match self {
            CmdSink::Engine(sender) => sender.send(cmd),
            #[cfg(test)]
            CmdSink::Null => drop(cmd),
        }
    }
}

struct HeadtrackApp {
    // ── Engine (owns the tracking infrastructure; kept alive for app lifetime) ──
    /// Never read — held purely so `Engine::drop` runs when the app exits.
    /// Dropping this field would shut the engine down at construction.
    /// `None` only in the headless render tests, which must never start an
    /// engine: `Engine::start` binds the live pose socket and opens a camera.
    #[allow(dead_code)]
    engine: Option<Engine>,
    state: Arc<EngineState>,
    cmd: CmdSink,

    active_tab: Tab,
    /// The head source kind the GUI is mirroring; see `sync_source_kind`.
    source_kind: SourceKind,
    /// `HEADTRACK_GUI_FAKE_TOBII` dev fixture (tabs/tobii.rs), read once.
    fake_tobii: Option<TobiiStatus>,
    /// `HEADTRACK_GUI_SCREENSHOT=<file.png>` dev fixture: after the first
    /// ~3 s, save this window's own pixels there and close. Captures nothing
    /// but this viewport, so it cannot catch another app or the desktop.
    shot_path: Option<std::path::PathBuf>,
    shot_frames: u32,
    /// P11's one-click udev rule install (Tobii tab, PermissionDenied).
    tobii_setup: tobii_setup::TobiiSetup,
    tuning_group: TuningGroup,
    dev_selected_axis: usize,

    // Per-axis filter + curve
    /// **The single mirror of the tuning state.**
    ///
    /// Controls edit this in place; `apply_profile` overwrites it wholesale;
    /// `to_profile` hands it straight back for saving. There is exactly one
    /// assignment path, so a field cannot be live in one direction and stale
    /// in the other — the failure mode that silently reset `slew_limit` on
    /// every save and dropped `cross_axis.yaw_to_z` on every load.
    profile: ProfileConfig,

    axis_active: [bool; 6],

    // Settings state
    landmark_overlay: bool,
    tracking_enabled: bool,
    minimize_to_tray: bool,
    /// Reveals the Advanced tab. Mirrors `CameraConfig::show_advanced`.
    show_advanced: bool,
    /// Dashboard camera-feed panel height, converged each frame from the
    /// leftover space so the tab fills the window exactly. See `ui_dashboard`.
    dash_feed_h: f32,
    /// Window size as of the last frame, and when to write it to disk.
    ///
    /// Resizing fires every frame of the drag, so the write is debounced rather
    /// than done per frame — `camera.toml` is a whole load-modify-save.
    last_window_size: (f32, f32),
    window_save_due: Option<std::time::Instant>,
    msfs_smoothing: f32,
    sc_smoothing: f32,
    recenter_flash: Option<std::time::Instant>,
    /// How far the last drawn page ran past its column (0 when it fits).
    /// Recorded every frame; asserted on by `render_tests`.
    #[cfg_attr(not(test), allow(dead_code))]
    page_overflow: f32,
    /// How far the rail footer was drawn over the last nav row (0 when they
    /// clear). Recorded every frame; asserted on by `render_tests`.
    #[cfg_attr(not(test), allow(dead_code))]
    rail_overlap: f32,

    // In-app hotkeys (fire only while this window is focused)
    hotkey_recenter: Option<egui::Key>,
    hotkey_toggle: Option<egui::Key>,
    hotkey_capture: Option<HotkeyAction>,
    /// When capture began — a short grace period ignores input queued at the
    /// moment the bind button was clicked.
    hotkey_capture_started: std::time::Instant,
    /// When "Change bindings…" was last clicked: the button rests and says
    /// what is happening for a few seconds, because System Settings is slow to
    /// appear and clicks made meanwhile each opened it again (D3).
    rebind_clicked_at: Option<std::time::Instant>,
    /// Keys already held when capture began (HOTAS maintained switches park on
    /// a key permanently) — never accepted as a binding; only fresh presses.
    hotkey_capture_initial: std::collections::HashSet<egui::Key>,

    // Camera selection
    cameras: Vec<CameraDevice>,
    selected_camera: u32,
    camera_fov: f32,
    /// Last FOV value read from EngineState — used to detect engine-side changes.
    last_engine_fov: u32,
    /// Active capture resolution — index into CAMERA_RESOLUTIONS.
    camera_res_idx: usize,
    /// Text field for naming a new FOV preset before saving.
    fov_preset_name: String,

    // V4L2 Direct API cache
    v4l2_device: Option<v4l::Device>,
    v4l2_controls: Vec<v4l::control::Description>,
    v4l2_last_camera: u32,
    v4l2_optimize_status: Option<(String, std::time::Instant)>,

    // Profiles
    profile_name: String,
    profiles_list: Vec<(String, std::path::PathBuf)>,
    profile_status: String,
    /// True once any control has edited `profile` since the last save/load —
    /// drives the "unsaved changes" marker in the Profiles tab.
    profile_dirty: bool,
    /// Profile awaiting delete confirmation (the row shows Cancel / Delete).
    profile_pending_delete: Option<std::path::PathBuf>,

    // Camera preview
    preview_texture: Option<egui::TextureHandle>,
    preview_last_seq: u64,

    // Sidebar logo + GitHub icon
    logo_texture: Option<egui::TextureHandle>,
    github_texture: Option<egui::TextureHandle>,

    // X-Plane installer
    xplane_path: String,
    xplane_install_status: Option<(String, bool)>, // (message, success)

    // Star Citizen / Wine bridge installer
    sc_prefix_path: String,
    sc_install_status: Option<(String, bool)>,
    /// Result of the last MSFS SimConnect.xml setup attempt: (message, success).
    msfs_setup_status: Option<(String, bool)>,
    sc_install_pending: Option<std::sync::mpsc::Receiver<(String, bool)>>,

    // Cached display string
    ipc_socket_label: String,

    // Diagnostics recorder
    diag_size_limit_mb: u32,
    diag_recording_start: Option<std::time::Instant>,
    diag_recording_path: std::path::PathBuf,
    diag_directory: String,
}

impl HeadtrackApp {
    fn new(engine: Engine) -> Self {
        let state = engine.state();
        let cmd = engine.cmd_sender();
        // Mirror the engine: load the active source's profile (active.toml for
        // the webcam, active-tobii.toml for the Tobii) so the GUI's displayed
        // profile name and Tuning sliders match the running pipeline. Falls
        // back to that source's built-in default when there is no file yet.
        #[cfg(feature = "dev-fixtures")]
        let fake_tobii = std::env::var(tabs::tobii::FAKE_TOBII_ENV)
            .ok()
            .and_then(|v| tabs::tobii::fake_tobii_status(&v));
        #[cfg(not(feature = "dev-fixtures"))]
        let fake_tobii: Option<TobiiStatus> = None;
        let source_kind = if fake_tobii.is_some() {
            SourceKind::Tobii
        } else {
            state
                .head_source
                .lock()
                .map(|s| s.kind())
                .unwrap_or(SourceKind::Webcam)
        };
        let cfg = tabs::tobii::active_profile_for(source_kind);
        #[cfg(feature = "dev-fixtures")]
        let page = std::env::var(tabs::tobii::PAGE_ENV)
            .ok()
            .and_then(|v| tabs::tobii::fixture_page(&v));
        #[cfg(not(feature = "dev-fixtures"))]
        let page: Option<(Tab, TuningGroup)> = None;
        let camera_cfg = CameraConfig::load();

        // Read initial camera index from engine (already set during Engine::start).
        let initial_camera = state.camera_index.load(Ordering::Relaxed);

        Self {
            ipc_socket_label: headtrack_engine::ipc_socket_path().display().to_string(),
            state,
            cmd: CmdSink::Engine(cmd),
            engine: Some(engine),
            active_tab: page.map_or(Tab::Dashboard, |p| p.0),
            source_kind,
            fake_tobii,
            #[cfg(feature = "dev-fixtures")]
            shot_path: std::env::var_os("HEADTRACK_GUI_SCREENSHOT").map(Into::into),
            #[cfg(not(feature = "dev-fixtures"))]
            shot_path: None,
            shot_frames: 0,
            tobii_setup: tobii_setup::TobiiSetup::default(),
            tuning_group: page.map_or(TuningGroup::default(), |p| p.1),
            dev_selected_axis: 0,
            axis_active: [true; 6],
            landmark_overlay: false,
            tracking_enabled: true,
            minimize_to_tray: camera_cfg.minimize_to_tray,
            show_advanced: camera_cfg.show_advanced,
            dash_feed_h: 240.0,
            last_window_size: (0.0, 0.0),
            window_save_due: None,
            msfs_smoothing: camera_cfg.msfs_camera_smoothing,
            sc_smoothing: camera_cfg.sc_output_smoothing,
            recenter_flash: None,
            page_overflow: 0.0,
            rail_overlap: 0.0,
            hotkey_recenter: camera_cfg
                .hotkey_recenter
                .as_deref()
                .and_then(egui::Key::from_name),
            hotkey_toggle: camera_cfg
                .hotkey_toggle
                .as_deref()
                .and_then(egui::Key::from_name),
            hotkey_capture: None,
            hotkey_capture_started: std::time::Instant::now(),
            rebind_clicked_at: None,
            hotkey_capture_initial: std::collections::HashSet::new(),
            cameras: enumerate_cameras(),
            selected_camera: initial_camera,
            camera_fov: camera_cfg.fov_diag_deg,
            last_engine_fov: camera_cfg.fov_diag_deg.to_bits(),
            camera_res_idx: CAMERA_RESOLUTIONS
                .iter()
                .position(|(w, h, _)| (*w, *h) == camera_cfg.camera_resolution)
                .unwrap_or(CAMERA_RES_DEFAULT_IDX),
            fov_preset_name: String::new(),
            v4l2_device: None,
            v4l2_controls: Vec::new(),
            v4l2_last_camera: u32::MAX,
            v4l2_optimize_status: None,
            profile_name: cfg.name.clone(),
            profile: cfg,
            profiles_list: ProfileConfig::list_profiles(),
            profile_status: String::new(),
            profile_dirty: false,
            profile_pending_delete: None,
            preview_texture: None,
            preview_last_seq: 0,
            logo_texture: None,
            github_texture: None,
            xplane_path: detect_xplane12_path(),
            xplane_install_status: None,
            sc_prefix_path: detect_sc_wine_prefix(),
            sc_install_status: None,
            msfs_setup_status: None,
            sc_install_pending: None,
            diag_size_limit_mb: 50,
            diag_recording_start: None,
            diag_recording_path: std::path::PathBuf::new(),
            diag_directory: camera_cfg.diag_directory,
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

    /// Recenter the pipeline baseline (shared by the Dashboard button and hotkey).
    fn trigger_recenter(&mut self) {
        self.cmd.send(EngineCmd::Recenter);
        self.recenter_flash = Some(std::time::Instant::now());
    }

    /// Toggle tracking on/off (shared by the Dashboard button and hotkey).
    fn toggle_tracking(&mut self) {
        self.tracking_enabled = !self.tracking_enabled;
        for (i, &axis) in AXIS_NAMES.iter().enumerate() {
            let v = if self.tracking_enabled && self.axis_active[i] {
                1.0
            } else {
                0.0
            };
            self.send_set(stage_names::AXIS_MASK, axis, v);
        }
        self.cmd
            .send(EngineCmd::SimConnectEnabled(self.tracking_enabled));
    }

    /// Persist the current hotkey bindings to `camera.toml`.
    fn save_hotkeys(&self) {
        let mut cfg = CameraConfig::load();
        cfg.hotkey_recenter = self.hotkey_recenter.map(|k| k.name().to_string());
        cfg.hotkey_toggle = self.hotkey_toggle.map(|k| k.name().to_string());
        let _ = cfg.save();
    }

    /// Dispatch in-app hotkeys. While capturing a new binding, grabs the next
    /// key press; otherwise fires bound actions only when the window is focused.
    /// Persist the window size so the next launch opens where the user left it.
    ///
    /// Debounced: a resize drag produces a size change on every frame, and each
    /// save is a whole `camera.toml` load-modify-write. The write happens once
    /// the size has been still for `SETTLE`, which also means a drag that ends
    /// back where it started costs nothing.
    fn track_window_size(&mut self, ctx: &egui::Context) {
        const SETTLE: std::time::Duration = std::time::Duration::from_millis(600);

        // `screen_rect`, not `viewport().inner_rect`: on Wayland the compositor
        // owns window geometry and winit reports both `inner_rect` and
        // `outer_rect` as `None`, so reading those persisted nothing at all on
        // this project's primary platform. `screen_rect` is the egui canvas,
        // which for a CentralPanel app is the window's inner size, and it is
        // populated on every backend.
        let rect = ctx.screen_rect();
        let size = (rect.width(), rect.height());
        if !size.0.is_finite() || !size.1.is_finite() || size.0 <= 0.0 || size.1 <= 0.0 {
            return;
        }

        // First frame establishes the baseline without scheduling a write —
        // otherwise every launch rewrites the file with the size it just read.
        if self.last_window_size == (0.0, 0.0) {
            self.last_window_size = size;
            return;
        }

        if (size.0 - self.last_window_size.0).abs() > 1.0
            || (size.1 - self.last_window_size.1).abs() > 1.0
        {
            self.last_window_size = size;
            self.window_save_due = Some(std::time::Instant::now() + SETTLE);
        }

        if self
            .window_save_due
            .is_some_and(|t| std::time::Instant::now() >= t)
        {
            self.window_save_due = None;
            let mut cfg = CameraConfig::load();
            cfg.window_size = Some(self.last_window_size);
            let _ = cfg.save();
        }
    }

    fn handle_hotkeys(&mut self, ctx: &egui::Context) {
        if let Some(action) = self.hotkey_capture {
            // Only FRESH presses count (`repeat: false`): a held key streaming
            // OS auto-repeat — a HOTAS maintained switch parked on a key —
            // must not be captured.
            let pressed = ctx.input(|i| {
                i.events.iter().find_map(|e| match e {
                    egui::Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        ..
                    } => Some(*key),
                    _ => None,
                })
            });
            if let Some(key) = pressed {
                if key == egui::Key::Escape {
                    self.hotkey_capture = None; // cancel without binding
                    return;
                }
                // Hardening: ignore keys that were already down when capture
                // began, and anything arriving in the first 250 ms (input
                // queued behind the bind-button click).
                let fresh = !self.hotkey_capture_initial.contains(&key)
                    && self.hotkey_capture_started.elapsed().as_millis() >= 250;
                if fresh {
                    match action {
                        HotkeyAction::Recenter => self.hotkey_recenter = Some(key),
                        HotkeyAction::Toggle => self.hotkey_toggle = Some(key),
                    }
                    self.save_hotkeys();
                    self.hotkey_capture = None;
                }
            }
            return; // don't fire actions while capturing
        }

        if !ctx.input(|i| i.focused) {
            return; // only fire when HeadtrackRS has keyboard focus
        }
        if let Some(k) = self.hotkey_recenter {
            if ctx.input(|i| i.key_pressed(k)) {
                self.trigger_recenter();
            }
        }
        if let Some(k) = self.hotkey_toggle {
            if ctx.input(|i| i.key_pressed(k)) {
                self.toggle_tracking();
            }
        }
    }

    /// Snapshot current GUI state into a [`ProfileConfig`].
    ///
    /// Trivial because the GUI edits a `ProfileConfig` directly. This used to
    /// rebuild the struct field by field and ended in `..ProfileConfig::default()`,
    /// which quietly refilled anything the GUI had forgotten to mirror —
    /// `slew_limit` was one, so a hand-tuned rate (the v7 profile's pitch =
    /// 80 °/s) was reset to stock on every save.
    fn to_profile(&self) -> ProfileConfig {
        let mut cfg = self.profile.clone();
        cfg.name = self.profile_name.clone();
        cfg
    }

    /// Apply a [`ProfileConfig`] to GUI state and push every param to the engine.
    ///
    /// Both halves are derived, not hand-written: the mirror is a whole-struct
    /// assignment, and the engine pushes come from `profile_params` — the same
    /// list the engine's own `apply_config` iterates. Previously this was ~60
    /// lines of hand-copied `send_set` calls maintained in parallel with the
    /// core applier, and the two had drifted apart (`cross_axis.yaw_to_z` was
    /// pushed here but dropped there).
    /// See `shot_path`. Waits ~90 frames so the splash, fonts and the
    /// Dashboard's converging feed height have settled.
    /// Compiled only with the `dev-fixtures` feature: a release build has no
    /// code that writes this window's pixels anywhere.
    #[cfg(not(feature = "dev-fixtures"))]
    fn fixture_screenshot(&mut self, _ctx: &egui::Context) {
        let _ = (&self.shot_path, self.shot_frames);
    }

    #[cfg(feature = "dev-fixtures")]
    fn fixture_screenshot(&mut self, ctx: &egui::Context) {
        let Some(path) = self.shot_path.clone() else {
            return;
        };
        self.shot_frames += 1;
        if self.shot_frames == 90 {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
        }
        let shot = ctx.input(|i| {
            i.raw.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            let [w, h] = img.size;
            let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
            match image::RgbaImage::from_raw(w as u32, h as u32, bytes) {
                Some(buf) => match buf.save_with_format(&path, image::ImageFormat::Png) {
                    Ok(()) => eprintln!("fixture screenshot: {}", path.display()),
                    Err(e) => eprintln!("fixture screenshot failed: {e}"),
                },
                None => eprintln!("fixture screenshot: bad buffer {w}x{h}"),
            }
            // Exit outright: with minimize-to-tray on (the default), a Close
            // would only minimise. The Wine SHM is never unlinked on a normal
            // exit either, so nothing is skipped that a clean shutdown does.
            std::process::exit(0);
        }
    }

    fn apply_profile(&mut self, cfg: &ProfileConfig) {
        self.profile = cfg.clone();
        self.profile_name = cfg.name.clone();
        // The mirror now matches what was loaded, so nothing is unsaved.
        self.profile_dirty = false;
        for (stage, param, value) in headtrack_core::pipeline::profile_params(cfg) {
            self.send_set(stage, &param, value);
        }
    }
}

// ── Shared visual language ───────────────────────────────────────────────────
//
// Every drawing helper and palette constant now lives in `ui`. Tab modules do
// `use crate::ui;` and call `ui::card(…)`, `ui::AXIS_COLORS`, and so on; only
// the palette keeps a short alias, because it is referenced constantly.

pub(crate) use ui::theme as colors;

/// Version string shown in the status bar and the About card.
pub(crate) const VERSION: &str = env!("CARGO_PKG_VERSION");

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
        Self {
            start_time: now,
            last_frame: now,
            dot_offset: egui::Vec2::ZERO,
            ring_angle: 0.0,
        }
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

    /// Draw the outer ring, cardinal ticks, and rotating dashed inner ring.
    fn draw_rings_and_ticks(
        painter: &egui::Painter,
        center: egui::Pos2,
        outer_r: f32,
        inner_r: f32,
        ring_angle: &mut f32,
        dt: f32,
    ) {
        painter.circle_stroke(
            center,
            outer_r,
            egui::Stroke::new(1.0_f32, egui::Color32::from_white_alpha(20)),
        );

        let tick_stroke = egui::Stroke::new(2.0_f32, colors::ORANGE);
        let tick_outer = outer_r - 2.0;
        let tick_inner = tick_outer - 16.0;
        for &deg in &[0.0_f32, 90.0, 180.0, 270.0] {
            let rad = deg.to_radians();
            let dir = egui::Vec2::new(rad.cos(), rad.sin());
            painter.line_segment(
                [center + dir * tick_inner, center + dir * tick_outer],
                tick_stroke,
            );
        }

        *ring_angle += 1.8 * dt;
        let dash_count = 36_usize;
        for i in 0..dash_count {
            if i % 2 == 0 {
                let a1 = *ring_angle + i as f32 * std::f32::consts::TAU / dash_count as f32;
                let a2 = *ring_angle + (i as f32 + 0.8) * std::f32::consts::TAU / dash_count as f32;
                let p1 = center + egui::Vec2::new(a1.cos(), a1.sin()) * inner_r;
                let p2 = center + egui::Vec2::new(a2.cos(), a2.sin()) * inner_r;
                painter.line_segment(
                    [p1, p2],
                    egui::Stroke::new(1.5_f32, egui::Color32::from_white_alpha(40)),
                );
            }
        }
    }

    /// Advance the sweep dot toward its next cardinal target, return new offset.
    ///
    /// 1.2-second looping cycle: center → left → right → up → down → center.
    fn advance_dot(
        dot_offset: &mut egui::Vec2,
        elapsed: f32,
        dt: f32,
        travel_r: f32,
    ) -> egui::Vec2 {
        let cycle = elapsed % 1.2;
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
        *dot_offset += (target - *dot_offset) * (20.0 * dt).min(1.0);
        *dot_offset
    }

    fn draw(&mut self, ui: &mut egui::Ui, ready: bool) {
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_frame).as_secs_f32().min(0.05);
        self.last_frame = now;
        let elapsed = now.duration_since(self.start_time).as_secs_f32();

        let painter = ui.painter();
        let rect = ui.max_rect();
        let center = rect.center();
        let outer_r = 120.0_f32;
        let inner_r = 96.0_f32;
        let travel_r = 72.0_f32;

        Self::draw_rings_and_ticks(painter, center, outer_r, inner_r, &mut self.ring_angle, dt);

        let dot_offset = Self::advance_dot(&mut self.dot_offset, elapsed, dt, travel_r);
        let dot_pos = center + dot_offset;
        painter.circle_filled(dot_pos, 6.0, colors::ORANGE);

        // Telemetry — top left
        let mono_sm = egui::FontId::monospace(12.0);
        let tl = rect.left_top() + egui::Vec2::new(22.0, 22.0);
        painter.text(
            tl,
            egui::Align2::LEFT_TOP,
            format!("SYS_UPTIME: {:.3}s", elapsed + 10.0),
            mono_sm.clone(),
            colors::DIMMER,
        );
        painter.text(
            tl + egui::Vec2::new(0.0, 15.0),
            egui::Align2::LEFT_TOP,
            format!("DOT_POS:   [{:.0}, {:.0}]", dot_pos.x, dot_pos.y),
            mono_sm.clone(),
            colors::DIMMER,
        );

        // Telemetry — top right (label row + value row)
        let tr = rect.right_top() + egui::Vec2::new(-210.0, 22.0);
        painter.text(
            tr,
            egui::Align2::LEFT_TOP,
            "STATUS   SENSOR    LAT",
            mono_sm.clone(),
            colors::DIMMER,
        );
        let (telem_val, telem_color) = if ready {
            ("READY    ONLINE    <3ms", colors::OK)
        } else {
            ("INIT     PENDING   ---", colors::ORANGE)
        };
        painter.text(
            tr + egui::Vec2::new(0.0, 15.0),
            egui::Align2::LEFT_TOP,
            telem_val,
            egui::FontId::new(11.0, colors::mono_bold()),
            telem_color,
        );

        // Brand: the same two-tone wordmark as the nav rail, no tagline.
        let brand = ui.fonts(|f| f.layout_job(ui::shell::wordmark(22.0)));
        painter.galley(
            center + egui::Vec2::new(0.0, outer_r + 46.0) - brand.size() / 2.0,
            brand,
            colors::TEXT_HI,
        );

        // Status message — blinking while loading, steady green when ready
        if ready {
            painter.text(
                center + egui::Vec2::new(0.0, outer_r + 86.0),
                egui::Align2::CENTER_CENTER,
                colors::track_out("TRACKING ONLINE"),
                egui::FontId::new(12.0, colors::mono_bold()),
                colors::OK,
            );
        } else {
            let status_msg = if elapsed < 1.0 {
                "INITIALIZING"
            } else if elapsed < 2.8 {
                "LOADING MODEL"
            } else {
                "OPENING CAMERA"
            };
            if ((elapsed * 3.0) as u32).is_multiple_of(2) {
                painter.text(
                    center + egui::Vec2::new(0.0, outer_r + 86.0),
                    egui::Align2::CENTER_CENTER,
                    colors::track_out(status_msg),
                    egui::FontId::new(12.0, colors::mono_bold()),
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
        rx: std::sync::mpsc::Receiver<Engine>,
        engine: Option<Engine>, // held here once ready, until MIN_SPLASH_SECS elapses
        splash: SplashScreen,
    },
    /// Engine ready — fade from black into the main dashboard.
    FadingIn {
        app: Box<HeadtrackApp>,
        fade_start: std::time::Instant,
    },
    /// Normal operation.
    Running { app: Box<HeadtrackApp> },
}

struct AppWrapper {
    state: Option<AppState>, // Option so we can take() during update
    tray_show: Arc<AtomicBool>,
    tray_ctx: Arc<std::sync::OnceLock<egui::Context>>,
}

impl AppWrapper {
    fn new(
        rx: std::sync::mpsc::Receiver<Engine>,
        tray_show: Arc<AtomicBool>,
        tray_ctx: Arc<std::sync::OnceLock<egui::Context>>,
    ) -> Self {
        Self {
            state: Some(AppState::Splash {
                rx,
                engine: None,
                splash: SplashScreen::new(),
            }),
            tray_show,
            tray_ctx,
        }
    }
}

impl AppWrapper {
    /// Handle minimize-to-tray close intercept and tray "Show" restore.
    fn handle_tray_events(&self, ctx: &egui::Context) {
        if ctx.input(|i| i.viewport().close_requested()) && CameraConfig::load().minimize_to_tray {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(true));
        }
        // If minimize_to_tray is false, let the close proceed normally.
        // Tray "Show" clicked — restore from minimized and focus.
        if self.tray_show.swap(false, Ordering::Relaxed) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
        }
    }
}

impl eframe::App for AppWrapper {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        // Give the tray a handle to wake the event loop (needed for quit while minimized).
        let _ = self.tray_ctx.set(ctx.clone());
        self.handle_tray_events(ctx);

        let Some(state) = self.state.take() else {
            debug_assert!(false, "AppWrapper state missing");
            return;
        };

        self.state = Some(match state {
            // ── Splash: draw HUD, collect engine, respect minimum duration ──
            AppState::Splash {
                rx,
                mut engine,
                mut splash,
            } => {
                // Collect the engine as soon as it's ready (non-blocking).
                if engine.is_none() {
                    if let Ok(e) = rx.try_recv() {
                        engine = Some(e);
                    }
                }
                let ready = engine.is_some();
                let elapsed = splash.elapsed();
                splash.show(ctx, ready);

                if ready && elapsed >= MIN_SPLASH_SECS {
                    AppState::FadingIn {
                        app: Box::new(HeadtrackApp::new(engine.unwrap())),
                        fade_start: std::time::Instant::now(),
                    }
                } else {
                    AppState::Splash { rx, engine, splash }
                }
            }

            // ── Fade-in: render main app then overlay dimming rect ────────
            AppState::FadingIn {
                mut app,
                fade_start,
            } => {
                app.update(ctx, frame);

                const FADE_SECS: f32 = 0.30;
                let alpha = 1.0 - (fade_start.elapsed().as_secs_f32() / FADE_SECS).clamp(0.0, 1.0);

                if alpha > 0.004 {
                    let a = (alpha * 255.0) as u8;
                    ctx.layer_painter(egui::LayerId::new(
                        egui::Order::Foreground,
                        egui::Id::new("fade_overlay"),
                    ))
                    .rect_filled(
                        ctx.screen_rect(),
                        0.0,
                        egui::Color32::from_black_alpha(a),
                    );
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

        self.track_window_size(ctx);

        // In-app hotkeys (recenter / toggle) — focus-gated inside.
        self.handle_hotkeys(ctx);

        // Poll background wine-bridge setup.
        if let Some(result) = self
            .sc_install_pending
            .as_ref()
            .and_then(|rx| rx.try_recv().ok())
        {
            self.sc_install_status = Some(result);
            self.sc_install_pending = None;
        }

        // Sync FOV from engine in case a camera switch triggered auto-detection.
        let engine_fov_bits = self.state.current_fov.load(Ordering::Relaxed);
        if engine_fov_bits != self.last_engine_fov {
            self.last_engine_fov = engine_fov_bits;
            self.camera_fov = f32::from_bits(engine_fov_bits);
        }

        self.sync_source_kind();
        self.fixture_screenshot(ctx);

        self.draw(ctx);
    }
}

impl HeadtrackApp {
    /// Draw one frame: status bar, nav rail and the active tab's page.
    ///
    /// Everything with a side effect outside this window — window-size
    /// persistence, hotkeys, the source-kind sync — happens in `update` before
    /// this is called, so the headless render tests can call `draw` alone.
    fn draw(&mut self, ctx: &egui::Context) {
        // Read live state directly from the engine — zero socket/IPC overhead.
        let pose = *self
            .state
            .latest_pose
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let fps = *self.state.fps.lock().unwrap_or_else(|e| e.into_inner());
        let connected = self.state.camera_connected.load(Ordering::Relaxed);
        let pose_vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];

        self.draw_status_bar(ctx, connected, fps);
        self.draw_sidebar(ctx);

        // ── Main content area ────────────────────────────────────────────
        egui::CentralPanel::default()
            .frame(
                egui::Frame::NONE
                    .fill(colors::BG)
                    .inner_margin(egui::Margin::same(colors::PAGE_INSET)),
            )
            .show(ctx, |ui| {
                // `ui::page` caps the content column — the single lever that
                // keeps right-aligned values next to their labels — adds the
                // scroll fade, and measures what the tab actually drew.
                let tab = self.active_tab;
                let metrics = ui
                    .push_id(tab.label(), |ui| {
                        ui::page(ui, |ui| match tab {
                            Tab::Dashboard => self.ui_dashboard(ui, &pose_vals),
                            Tab::Tuning => self.ui_tuning(ui, &pose_vals),
                            Tab::Profiles => self.ui_profiles(ui),
                            Tab::Camera => self.ui_camera(ui),
                            Tab::Tobii => self.ui_tobii(ui),
                            Tab::Keybinds => self.ui_keybinds(ui),
                            Tab::Settings => self.ui_settings(ui),
                            Tab::Advanced => self.ui_advanced(ui, &pose_vals),
                        })
                    })
                    .inner;
                self.page_overflow = metrics.overflow();
            });
    }
}

impl HeadtrackApp {
    /// Whether the SimConnect TCP session to MSFS is up.
    ///
    /// The engine mirrors this into `EngineState` every run-loop tick, and has
    /// done since the seat-offset work — it was simply never displayed.
    pub(crate) fn msfs_link(&self) -> bool {
        self.state.msfs_connected.load(Ordering::Relaxed)
    }

    fn draw_status_bar(&self, ctx: &egui::Context, connected: bool, fps: f32) {
        use ui::shell::{status_dot, status_mono, status_sep};
        egui::TopBottomPanel::bottom("status_bar")
            .exact_height(ui::shell::STATUS_HEIGHT)
            .frame(
                egui::Frame::NONE
                    .fill(colors::SIDEBAR)
                    .inner_margin(egui::Margin::symmetric(16, 0)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;

                    // The health reason *is* the one-line "why it's waiting".
                    match self.source_health(connected) {
                        Ok(()) => status_dot(ui, "Tracking", ui::Tone::Ok),
                        Err((why, tone)) => status_dot(ui, &why, tone),
                    }

                    status_sep(ui);
                    status_mono(ui, &format!("{fps:.0} Hz"), colors::ORANGE);

                    status_sep(ui);
                    if self.msfs_link() {
                        status_dot(ui, "MSFS linked", ui::Tone::Ok);
                    } else {
                        status_dot(ui, "MSFS waiting", ui::Tone::Idle);
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        status_mono(ui, &format!("HeadtrackRS v{VERSION}"), colors::DIMMER);
                    });
                });
            });
    }

    fn draw_sidebar(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("sidebar")
            .exact_width(ui::shell::NAV_WIDTH)
            .resizable(false)
            .frame(
                egui::Frame::NONE
                    .fill(colors::SIDEBAR)
                    .inner_margin(egui::Margin::symmetric(12, 16)),
            )
            .show(ctx, |ui| {
                let logo = self.logo_texture.get_or_insert_with(|| {
                    let bytes = include_bytes!("../../../assets/icons/headtrack-rs.png");
                    let icon = eframe::icon_data::from_png_bytes(bytes).expect("logo PNG");
                    let image = egui::ColorImage::from_rgba_unmultiplied(
                        [icon.width as usize, icon.height as usize],
                        &icon.rgba,
                    );
                    ctx.load_texture("app-logo", image, egui::TextureOptions::LINEAR)
                });
                let logo = egui::load::SizedTexture::new(logo.id(), egui::vec2(64.0, 64.0));
                if ui.available_height() >= ui::shell::RAIL_FULL_H {
                    ui::shell::brand_block(ui, logo);
                    ui.add_space(18.0);
                } else {
                    // A short window: the footer is bottom-anchored and the
                    // nav list top-anchored, so the brand block gives way.
                    ui::shell::brand_block_compact(ui, logo);
                    ui.add_space(14.0);
                }

                for tab in PRIMARY_TABS {
                    self.nav_row(ui, tab);
                }
                ui::shell::nav_separator(ui);
                for tab in SECONDARY_TABS {
                    self.nav_row(ui, tab);
                }
                if self.show_advanced {
                    self.nav_row(ui, Tab::Advanced);
                }
                // The cursor sits one item gap below the last row drawn.
                let nav_bottom = ui.cursor().top() - ui.spacing().item_spacing.y;

                let footer_top = self.draw_sidebar_footer(ui);
                self.rail_overlap = (nav_bottom - footer_top).max(0.0);
            });
    }

    fn nav_row(&mut self, ui: &mut egui::Ui, tab: Tab) {
        if ui::shell::nav_item(ui, tab.icon(), tab.label(), self.active_tab == tab).clicked() {
            self.active_tab = tab;
        }
    }

    /// Bottom-anchored live output summary: the pose socket external tools
    /// connect to, and the SimConnect link.
    fn draw_sidebar_footer(&self, ui: &mut egui::Ui) -> f32 {
        ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
            ui.add_space(2.0);
            let socket = std::path::Path::new(&self.ipc_socket_label)
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| self.ipc_socket_label.clone());
            ui::shell::status_mono(ui, &socket, colors::DIM);
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 8.0;
                if self.msfs_link() {
                    ui::shell::status_dot(ui, "SimConnect linked", ui::Tone::Ok);
                } else {
                    ui::shell::status_dot(ui, "SimConnect waiting", ui::Tone::Idle);
                }
            });
        })
        .response
        .rect
        .top()
    }
}

#[cfg(test)]
mod param_names;
#[cfg(test)]
mod render_tests;
mod tabs;
mod tobii_setup;
mod ui;

// ── Tab implementations ──────────────────────────────────────────────────────
#[cfg(test)]
mod simconnect_setup_tests {
    use super::*;

    fn tmpdir(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!(
            "ht-simconnect-{tag}-{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn creates_the_file_when_the_sim_has_not_written_one() {
        let d = tmpdir("create");
        let r = install_simconnect_xml(&d).expect("create");
        assert!(matches!(r, SimConnectSetup::Created(_)));
        let written = std::fs::read_to_string(d.join("SimConnect.xml")).unwrap();
        assert!(written.contains("<Port>5557</Port>"));
        assert!(written.contains("</SimBase.Document>"));
        let _ = std::fs::remove_dir_all(&d);
    }

    /// Re-running must be a no-op. MSFS rewrites this file on major updates, so
    /// "click it again" is the documented recovery — it must not stack blocks.
    #[test]
    fn is_idempotent() {
        let d = tmpdir("idem");
        install_simconnect_xml(&d).expect("first");
        let after_first = std::fs::read_to_string(d.join("SimConnect.xml")).unwrap();

        let r = install_simconnect_xml(&d).expect("second");
        assert!(matches!(r, SimConnectSetup::AlreadyConfigured));
        let after_second = std::fs::read_to_string(d.join("SimConnect.xml")).unwrap();
        assert_eq!(
            after_first, after_second,
            "second run must not touch the file"
        );
        assert_eq!(after_second.matches("<SimConnect.Comm>").count(), 1);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The case that matters: a user with their own SimConnect entries for other
    /// tools must keep them. We amend, never replace.
    #[test]
    fn preserves_existing_entries_when_amending() {
        let d = tmpdir("amend");
        let path = d.join("SimConnect.xml");
        let theirs = r#"<?xml version="1.0" encoding="Windows-1252"?>
<SimBase.Document Type="SimConnect" version="1,0">
  <SimConnect.Comm>
    <Descr>SomeOtherTool</Descr>
    <Protocol>IPv4</Protocol>
    <Port>500</Port>
  </SimConnect.Comm>
</SimBase.Document>
"#;
        std::fs::write(&path, theirs).unwrap();

        let r = install_simconnect_xml(&d).expect("amend");
        let SimConnectSetup::Amended { backup, .. } = r else {
            panic!("expected Amended")
        };

        let now = std::fs::read_to_string(&path).unwrap();
        assert!(now.contains("SomeOtherTool"), "their entry must survive");
        assert!(now.contains("<Port>500</Port>"), "their port must survive");
        assert!(now.contains("<Port>5557</Port>"), "ours must be added");
        assert_eq!(
            now.matches("</SimBase.Document>").count(),
            1,
            "still one document"
        );
        assert_eq!(
            now.matches("<SimConnect.Comm>").count(),
            2,
            "two comm blocks"
        );

        // the untouched original is recoverable
        assert_eq!(std::fs::read_to_string(&backup).unwrap(), theirs);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A corrupt file must be refused, not overwritten — it may be the only copy
    /// of something the user cares about.
    #[test]
    fn refuses_a_file_with_no_closing_tag() {
        let d = tmpdir("corrupt");
        let path = d.join("SimConnect.xml");
        let junk = "this is not xml at all";
        std::fs::write(&path, junk).unwrap();

        let err = install_simconnect_xml(&d).expect_err("must refuse");
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            junk,
            "a refused file must be left exactly as found"
        );
        let _ = std::fs::remove_dir_all(&d);
    }
}

#[cfg(test)]
mod steam_vdf_tests {
    use super::parse_vdf_path_line;

    /// The exact shape Steam writes, tabs and all.
    #[test]
    fn extracts_the_path_without_its_closing_quote() {
        let line = "\t\t\"path\"\t\t\"/mnt/steam/SteamLibrary\"";
        assert_eq!(
            parse_vdf_path_line(line).as_deref(),
            Some("/mnt/steam/SteamLibrary")
        );
    }

    /// The regression this guards: a bounded split leaves the closing quote on
    /// the value, so every path built from it points at a directory that cannot
    /// exist and auto-detection silently finds nothing.
    #[test]
    fn result_is_usable_as_a_path() {
        let line = "\t\t\"path\"\t\t\"/home/user/.local/share/Steam\"";
        let p = parse_vdf_path_line(line).expect("parsed");
        assert!(!p.contains('"'), "no stray quote: {p:?}");
        assert!(!p.ends_with('/'), "no trailing separator: {p:?}");
        assert_eq!(
            std::path::Path::new(&p)
                .join("steamapps")
                .file_name()
                .unwrap(),
            "steamapps"
        );
    }

    #[test]
    fn ignores_lines_that_are_not_paths() {
        for line in [
            "\t\"libraryfolders\"",
            "\t\t\"label\"\t\t\"\"",
            "\t\t\"contentid\"\t\t\"12345\"",
            "{",
            "",
        ] {
            assert_eq!(parse_vdf_path_line(line), None, "should ignore: {line:?}");
        }
    }

    #[test]
    fn ignores_an_empty_path_value() {
        assert_eq!(parse_vdf_path_line("\t\t\"path\"\t\t\"\""), None);
    }
}
