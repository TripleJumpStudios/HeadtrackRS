//! headtrack-xplane — X-Plane 12 plugin
//!
//! Install as:
//!   X-Plane 12/Resources/plugins/headtrack/64/lin.xpl
//!
//! ## Coordinate transform
//!
//! headtrack-rs canonical → X-Plane DataRef convention:
//!
//! | headtrack | DataRef                        | Notes              |
//! |-----------|--------------------------------|--------------------|
//! | yaw (°)   | pilots_head_psi   positive=right | same sign          |
//! | pitch (°) | pilots_head_the   positive=up   | same sign          |
//! | roll (°)  | pilots_head_phi   positive=right tilt | same sign    |
//! | x (mm)    | pilots_head_x     right, metres | ÷ 1000             |
//! | y (mm)    | pilots_head_y     up, metres    | ÷ 1000             |
//! | z (mm)    | pilots_head_z     fwd, metres   | ÷ 1000             |

use std::ffi::{c_void, CString};
use std::path::PathBuf;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex, OnceLock,
};

mod ffi;
mod ipc;

use ffi::{
    debug, XPLMAppendMenuItem, XPLMAppendMenuSeparator, XPLMCheckMenuItem, XPLMCommandRef,
    XPLMCreateCommand, XPLMCreateMenu, XPLMCreateWindowEx, XPLMCreateWindow_t, XPLMDataRef,
    XPLMDestroyWindow, XPLMDrawString, XPLMFindDataRef, XPLMFindPluginsMenu, XPLMGetDatab,
    XPLMGetDataf, XPLMGetDatai, XPLMGetScreenBoundsGlobal, XPLMGetWindowGeometry,
    XPLMGetWindowIsVisible, XPLMMenuID, XPLMRegisterCommandHandler, XPLMRegisterFlightLoopCallback,
    XPLMSetDataf, XPLMSetMenuItemName, XPLMSetWindowIsVisible, XPLMSetWindowPositioningMode,
    XPLMSetWindowTitle, XPLMUnregisterCommandHandler, XPLMUnregisterFlightLoopCallback,
    XPLMWindowID, XPLM_CURSOR_DEFAULT, XPLM_FONT_PROPORTIONAL, XPLM_MOUSE_DOWN, XPLM_MOUSE_UP,
    XPLM_WINDOW_DECORATION_ROUND_RECTANGLE, XPLM_WINDOW_LAYER_FLOATING, XPLM_WINDOW_POSITION_FREE,
};
use headtrack_core::Pose;

// ── Global plugin state ───────────────────────────────────────────────────────
// All X-Plane callbacks fire on the sim's main thread, so DataRef handles
// and viewport_ref are safe to access without a lock.  The pose is shared
// with the IPC thread via Arc<Mutex<>>.

static DATAREFS: OnceLock<DataRefs> = OnceLock::new();
static LATEST_POSE: OnceLock<Arc<Mutex<Option<Pose>>>> = OnceLock::new();
static ENABLED: AtomicBool = AtomicBool::new(true);
/// Saved submenu handle for checkmark updates from menu handler.
static SUBMENU: OnceLock<XPLMMenuID> = OnceLock::new();

/// Per-frame interpolated pose — smooths the 20 Hz input across 60+ Hz render.
/// Only accessed from the sim main thread (flight loop callback).
static mut SMOOTH_POSE: [f32; 6] = [0.0; 6];

/// Last known connection state — used to detect transitions and update menu.
/// Only accessed from the sim main thread (flight loop callback).
static mut LAST_CONNECTED: bool = false;

/// Interpolation speed: fraction of the gap closed per second.
/// 15.0 ≈ reaches ~95% in ~200 ms, feels snappy but no staircase.
const LERP_SPEED: f32 = 15.0;

/// Captured acf_pe baseline — refreshed on every PLANE_LOADED message.
/// Only accessed from the sim main thread (flight loop + message callback).
static mut VIEWPORT_REF: [f32; 3] = [0.0; 3];
/// Set true by PLANE_LOADED to trigger a re-read of acf_pe on the next frame.
static MUST_RESET_VIEWPORT: AtomicBool = AtomicBool::new(true);

/// User-adjustable seat/base offset in aircraft coordinates (metres):
/// `[x = right, y = up, z = aft]`.  Persisted per-aircraft and added on top of
/// `acf_pe`, so head tracking rides on whatever seat position the user chooses.
/// Only accessed from the sim main thread (flight loop, menu, command handlers).
static mut SEAT_OFFSET: [f32; 3] = [0.0; 3];

/// Stored seat command handles, kept so we can unregister on shutdown.
/// Each entry is `(command, action_code)` — see [`apply_seat_action`].
static SEAT_COMMANDS: OnceLock<Vec<(XPLMCommandRef, usize)>> = OnceLock::new();

/// Bindable recenter command handle, kept for unregistration.
static RECENTER_COMMAND: OnceLock<XPLMCommandRef> = OnceLock::new();

/// The on-screen seat-adjust panel window.
static SEAT_WINDOW: OnceLock<XPLMWindowID> = OnceLock::new();

/// Action code of the panel arrow currently held down with the mouse, if any —
/// nudged every flight-loop frame for hold-to-slide. Sim main thread only.
static mut SEAT_HELD: Option<usize> = None;

/// Bindable "toggle seat panel" command handle, kept for unregistration.
static SEAT_PANEL_COMMAND: OnceLock<XPLMCommandRef> = OnceLock::new();

/// Menu item indices (within our submenu).
/// Separators count as items in X-Plane's menu system.
const MENU_ENABLE: i32 = 0;
const MENU_RECENTER: i32 = 1;
// index 2 = separator
const MENU_SEAT_PANEL: i32 = 3;
const MENU_SEAT_FWD: i32 = 4;
const MENU_SEAT_BACK: i32 = 5;
const MENU_SEAT_UP: i32 = 6;
const MENU_SEAT_DOWN: i32 = 7;
const MENU_SEAT_LEFT: i32 = 8;
const MENU_SEAT_RIGHT: i32 = 9;
const MENU_SEAT_RESET: i32 = 10;
// index 11 = separator
const MENU_STATUS: i32 = 12;

// Axis sign corrections — flip any that feel backwards in-sim.
// +1.0 = pass through, -1.0 = invert.
const SIGN_YAW: f32 = 1.0;
const SIGN_PITCH: f32 = 1.0;
const SIGN_ROLL: f32 = 1.0;
const SIGN_X: f32 = -1.0;
const SIGN_Y: f32 = 1.0;
// SIGN_Z is inverted *and* half-scaled, the only magnitude scale in any output
// adapter (SimConnect passes position through at /1000.0).
//
// The original justification — "depth-from-face-size, tamed by the slew limiter
// plus a heavy One-Euro filter" — is now two-thirds historical. The slew limiter
// is still live (core/pipeline.rs, SlewLimitStage) and this plugin still LERPs
// at LERP_SPEED, but One-Euro was deleted in Snow Leopard 2, and Z is no longer
// face-area depth: it is yaw-corrected IPD behind the z_gate rate limiter.
//
// Flown 2026-08-17 and again 2026-08-18 on the current tracker and Accela, all
// six axes clean. So -0.5 is **verified good**, not merely inherited. Whether
// -1.0 would give preferable Z travel is an open tuning question, not a defect,
// and is deliberately not being changed on a guess — the standing rule is bug
// fixes only while flying (docs/FLIGHT_TEST_LOG.md §4). Provenance for all four
// adapters: docs/PLUGINS.md.
const SIGN_Z: f32 = -0.5;

/// Metres the seat moves per command tick (per key-press and per held frame).
const SEAT_STEP: f32 = 0.01;
/// Clamp the seat offset so the camera can't fly out of the cockpit.
const SEAT_LIMIT: f32 = 0.6;

/// X-Plane interior view type constant.
const VIEW_TYPE_3D_COCKPIT: i32 = 1026;

struct DataRefs {
    view_type: XPLMDataRef,
    yaw: XPLMDataRef,
    pitch: XPLMDataRef,
    roll: XPLMDataRef,
    x: XPLMDataRef,
    y: XPLMDataRef,
    z: XPLMDataRef,
    acf_pe_x: XPLMDataRef,
    acf_pe_y: XPLMDataRef,
    acf_pe_z: XPLMDataRef,
    acf_relative_path: XPLMDataRef,
    acf_icao: XPLMDataRef,
}

// DataRefs are opaque handles valid for the sim lifetime — safe to share.
unsafe impl Send for DataRefs {}
unsafe impl Sync for DataRefs {}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn find_dataref(name: &str) -> XPLMDataRef {
    let c = CString::new(name).unwrap();
    unsafe { XPLMFindDataRef(c.as_ptr()) }
}

fn cstr(s: &str) -> CString {
    CString::new(s).unwrap_or_default()
}

/// Read acf_pe into VIEWPORT_REF.  Called from the flight loop (not from
/// the message handler, because DataRefs may not be populated yet at
/// message time).
unsafe fn capture_viewport_ref(dr: &DataRefs) {
    let x = XPLMGetDataf(dr.acf_pe_x);
    let y = XPLMGetDataf(dr.acf_pe_y);
    let z = XPLMGetDataf(dr.acf_pe_z);
    VIEWPORT_REF = [x, y, z];
    debug(&format!(
        "headtrack-rs: viewport_ref = [{x:.4}, {y:.4}, {z:.4}]\n"
    ));
}

// ── Seat / base offset ──────────────────────────────────────────────────────
//
// A persistent, user-controlled translation in aircraft coordinates, added on
// top of `acf_pe` so head tracking rides on whatever seat position the user has
// dialled in.  Stored per-aircraft in a small TSV file and reloaded on
// PLANE_LOADED.  All access is from the sim main thread.

/// Read a string DataRef (e.g. the aircraft path) into a `String`.
unsafe fn read_string_dataref(dr: XPLMDataRef) -> String {
    if dr.is_null() {
        return String::new();
    }
    let mut buf = [0u8; 512];
    let n = XPLMGetDatab(dr, buf.as_mut_ptr() as *mut c_void, 0, buf.len() as i32);
    if n <= 0 {
        return String::new();
    }
    let n = (n as usize).min(buf.len());
    let end = buf[..n].iter().position(|&b| b == 0).unwrap_or(n);
    String::from_utf8_lossy(&buf[..end]).into_owned()
}

/// Per-aircraft key used to store/recall the seat offset.
unsafe fn acf_key(dr: &DataRefs) -> String {
    let path = read_string_dataref(dr.acf_relative_path);
    if !path.is_empty() {
        return path;
    }
    let icao = read_string_dataref(dr.acf_icao);
    if !icao.is_empty() {
        return icao;
    }
    "default".to_string()
}

/// `$XDG_CONFIG_HOME/headtrack-rs/xplane-seats.tsv` (falls back to `~/.config`).
fn seat_config_path() -> PathBuf {
    let base = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("{}/.config", std::env::var("HOME").unwrap_or_default()));
    PathBuf::from(base).join("headtrack-rs/xplane-seats.tsv")
}

/// Read the saved seat map: one `key\tx\ty\tz` line per aircraft.
fn read_seat_map() -> Vec<(String, [f32; 3])> {
    let mut out = Vec::new();
    let Ok(contents) = std::fs::read_to_string(seat_config_path()) else {
        return out;
    };
    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut f = line.split('\t');
        if let (Some(k), Some(x), Some(y), Some(z)) = (f.next(), f.next(), f.next(), f.next()) {
            if let (Ok(x), Ok(y), Ok(z)) = (x.parse(), y.parse(), z.parse()) {
                out.push((k.to_string(), [x, y, z]));
            }
        }
    }
    out
}

/// Load this aircraft's saved seat offset into `SEAT_OFFSET` (0 if none saved).
unsafe fn load_seat(dr: &DataRefs) {
    let key = acf_key(dr);
    let off = read_seat_map()
        .into_iter()
        .find(|(k, _)| *k == key)
        .map(|(_, o)| o)
        .unwrap_or([0.0; 3]);
    SEAT_OFFSET = off;
    debug(&format!(
        "headtrack-rs: seat offset for '{key}' = [{:.3}, {:.3}, {:.3}]\n",
        off[0], off[1], off[2]
    ));
}

/// Persist the current `SEAT_OFFSET` for the active aircraft, preserving others.
unsafe fn save_seat() {
    let Some(dr) = DATAREFS.get() else { return };
    let key = acf_key(dr);
    let off = SEAT_OFFSET;
    let mut map = read_seat_map();
    if let Some(entry) = map.iter_mut().find(|(k, _)| *k == key) {
        entry.1 = off;
    } else {
        map.push((key.clone(), off));
    }
    let mut text =
        String::from("# headtrack-rs X-Plane seat offsets — key<TAB>x<TAB>y<TAB>z (metres)\n");
    for (k, o) in &map {
        text.push_str(&format!("{}\t{:.4}\t{:.4}\t{:.4}\n", k, o[0], o[1], o[2]));
    }
    let path = seat_config_path();
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let _ = std::fs::write(&path, text);
}

/// Nudge one seat axis by `delta` metres, clamped to the cockpit envelope.
unsafe fn nudge_seat(axis: usize, delta: f32) {
    SEAT_OFFSET[axis] = (SEAT_OFFSET[axis] + delta).clamp(-SEAT_LIMIT, SEAT_LIMIT);
}

/// Apply a seat action by code. Aircraft axes: x=right, y=up, z=aft.
/// 1=fwd 2=back 3=up 4=down 5=left 6=right 7=reset.
unsafe fn apply_seat_action(code: usize) {
    match code {
        1 => nudge_seat(2, -SEAT_STEP), // forward = -Z
        2 => nudge_seat(2, SEAT_STEP),  // back/aft = +Z
        3 => nudge_seat(1, SEAT_STEP),  // up      = +Y
        4 => nudge_seat(1, -SEAT_STEP), // down    = -Y
        5 => nudge_seat(0, -SEAT_STEP), // left    = -X
        6 => nudge_seat(0, SEAT_STEP),  // right   = +X
        7 => SEAT_OFFSET = [0.0; 3],    // reset
        _ => {}
    }
}

/// Bindable seat command handler. Nudges on begin/continue (so holding a key
/// slides the seat), then persists on release.
unsafe extern "C" fn seat_command(_cmd: XPLMCommandRef, phase: i32, refcon: *mut c_void) -> i32 {
    let code = refcon as usize;
    match phase {
        0 => apply_seat_action(code),              // begin
        1 if code != 7 => apply_seat_action(code), // continue (reset is one-shot)
        2 => save_seat(),                          // end → persist
        _ => {}
    }
    1
}

/// Apply a seat action from a menu click and persist immediately.
unsafe fn seat_menu_action(code: usize) {
    apply_seat_action(code);
    save_seat();
}

/// Trigger a recenter: clear the latest pose and tell the engine to reset its
/// baseline. Shared by the menu item and the bindable command.
unsafe fn do_recenter() {
    if let Some(arc) = LATEST_POSE.get() {
        if let Ok(mut g) = arc.lock() {
            *g = None;
        }
    }
    match headtrack_ipc::send_cmd("recenter") {
        Ok(()) => debug("headtrack-rs: recentered\n"),
        Err(e) => debug(&format!("headtrack-rs: recenter failed: {e}\n")),
    }
}

/// Bindable recenter command handler — fires once on key-down.
unsafe extern "C" fn recenter_command(
    _cmd: XPLMCommandRef,
    phase: i32,
    _refcon: *mut c_void,
) -> i32 {
    if phase == 0 {
        do_recenter();
    }
    1
}

/// Create and register the bindable commands (seat adjust + recenter).
/// They appear in X-Plane → Settings → Keyboard / Joystick under "headtrack-rs".
unsafe fn register_commands() {
    let specs: [(&str, &str, usize); 7] = [
        (
            "headtrack-rs/seat_forward",
            "HeadtrackRS: move pilot seat forward",
            1,
        ),
        (
            "headtrack-rs/seat_back",
            "HeadtrackRS: move pilot seat aft",
            2,
        ),
        ("headtrack-rs/seat_up", "HeadtrackRS: move pilot seat up", 3),
        (
            "headtrack-rs/seat_down",
            "HeadtrackRS: move pilot seat down",
            4,
        ),
        (
            "headtrack-rs/seat_left",
            "HeadtrackRS: move pilot seat left",
            5,
        ),
        (
            "headtrack-rs/seat_right",
            "HeadtrackRS: move pilot seat right",
            6,
        ),
        (
            "headtrack-rs/seat_reset",
            "HeadtrackRS: reset pilot seat offset",
            7,
        ),
    ];
    let mut stored = Vec::with_capacity(specs.len());
    for (name, desc, code) in specs {
        let cmd = XPLMCreateCommand(cstr(name).as_ptr(), cstr(desc).as_ptr());
        XPLMRegisterCommandHandler(cmd, seat_command, 1, code as *mut c_void);
        stored.push((cmd, code));
    }
    SEAT_COMMANDS.set(stored).ok();

    let recenter = XPLMCreateCommand(
        cstr("headtrack-rs/recenter").as_ptr(),
        cstr("HeadtrackRS: recenter head tracking").as_ptr(),
    );
    XPLMRegisterCommandHandler(recenter, recenter_command, 1, std::ptr::null_mut());
    RECENTER_COMMAND.set(recenter).ok();

    let panel = XPLMCreateCommand(
        cstr("headtrack-rs/toggle_seat_panel").as_ptr(),
        cstr("HeadtrackRS: show/hide the seat adjust panel").as_ptr(),
    );
    XPLMRegisterCommandHandler(panel, seat_panel_command, 1, std::ptr::null_mut());
    SEAT_PANEL_COMMAND.set(panel).ok();
}

// ── Seat adjust panel (on-screen window) ─────────────────────────────────────
//
// A small floating window with a D-pad of arrows. Click-and-hold an arrow to
// slide the seat (the held action is nudged every flight-loop frame); release
// to persist. Toggled via the menu item or the bindable command.

struct SeatButton {
    code: usize,
    label: &'static str,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
}

/// Compute button hit-boxes from the window geometry (global boxel coords,
/// y up). Shared by the draw and click callbacks so they always agree.
fn seat_buttons(l: i32, t: i32, r: i32, _b: i32) -> [SeatButton; 7] {
    let cx = (l + r) / 2;
    let pad_y = t - 44; // top row, below the title bar
    let row = 30;
    let colx = 46;
    let hw = 28;
    let hh = 11;
    let mk = |code, label, ccx: i32, ccy: i32| SeatButton {
        code,
        label,
        x0: ccx - hw,
        y0: ccy - hh,
        x1: ccx + hw,
        y1: ccy + hh,
    };
    [
        mk(1, "FWD", cx, pad_y),                // forward (-Z)
        mk(5, "LEFT", cx - colx, pad_y - row),  // left  (-X)
        mk(7, "CTR", cx, pad_y - row),          // reset
        mk(6, "RIGHT", cx + colx, pad_y - row), // right (+X)
        mk(2, "BACK", cx, pad_y - 2 * row),     // back  (+Z)
        mk(3, "UP", cx - 34, pad_y - 3 * row),  // height up   (+Y)
        mk(4, "DN", cx + 34, pad_y - 3 * row),  // height down (-Y)
    ]
}

unsafe fn panel_text(x: i32, y: i32, s: &str, mut rgb: [f32; 3]) {
    let c = CString::new(s).unwrap_or_default();
    XPLMDrawString(
        rgb.as_mut_ptr(),
        x,
        y,
        c.as_ptr() as *mut i8,
        std::ptr::null_mut(),
        XPLM_FONT_PROPORTIONAL,
    );
}

unsafe extern "C" fn seat_draw(window: XPLMWindowID, _refcon: *mut c_void) {
    let (mut l, mut t, mut r, mut b) = (0, 0, 0, 0);
    XPLMGetWindowGeometry(window, &mut l, &mut t, &mut r, &mut b);
    let held = SEAT_HELD;
    for btn in &seat_buttons(l, t, r, b) {
        let ccx = (btn.x0 + btn.x1) / 2;
        let ccy = (btn.y0 + btn.y1) / 2;
        let est_w = btn.label.len() as i32 * 5; // ~5 px/char, proportional font
        let color = if held == Some(btn.code) {
            [1.0, 0.55, 0.0] // orange while held
        } else {
            [0.85, 0.85, 0.85]
        };
        panel_text(ccx - est_w / 2, ccy - 5, btn.label, color);
    }
    let off = SEAT_OFFSET;
    let readout = format!("X{:+.2} Y{:+.2} Z{:+.2} m", off[0], off[1], off[2]);
    panel_text(l + 8, b + 12, &readout, [0.55, 0.8, 1.0]);
}

unsafe extern "C" fn seat_click(
    window: XPLMWindowID,
    x: i32,
    y: i32,
    mouse: i32,
    _refcon: *mut c_void,
) -> i32 {
    match mouse {
        XPLM_MOUSE_DOWN => {
            let (mut l, mut t, mut r, mut b) = (0, 0, 0, 0);
            XPLMGetWindowGeometry(window, &mut l, &mut t, &mut r, &mut b);
            let hit = seat_buttons(l, t, r, b)
                .iter()
                .find(|btn| x >= btn.x0 && x <= btn.x1 && y >= btn.y0 && y <= btn.y1)
                .map(|btn| btn.code);
            if let Some(code) = hit {
                apply_seat_action(code); // immediate nudge / reset on press
                if code == 7 {
                    save_seat(); // reset is one-shot
                } else {
                    SEAT_HELD = Some(code); // keep sliding while held
                }
            }
        }
        XPLM_MOUSE_UP => {
            let held = SEAT_HELD;
            if held.is_some() {
                SEAT_HELD = None;
                save_seat();
            }
        }
        _ => {}
    }
    1
}

unsafe extern "C" fn seat_key(
    _w: XPLMWindowID,
    _key: i8,
    _flags: i32,
    _vkey: i8,
    _rc: *mut c_void,
    _losing_focus: i32,
) {
}

unsafe extern "C" fn seat_cursor(_w: XPLMWindowID, _x: i32, _y: i32, _rc: *mut c_void) -> i32 {
    XPLM_CURSOR_DEFAULT
}

unsafe extern "C" fn seat_wheel(
    _w: XPLMWindowID,
    _x: i32,
    _y: i32,
    _wheel: i32,
    _clicks: i32,
    _rc: *mut c_void,
) -> i32 {
    0
}

unsafe extern "C" fn seat_rightclick(
    _w: XPLMWindowID,
    _x: i32,
    _y: i32,
    _m: i32,
    _rc: *mut c_void,
) -> i32 {
    1
}

/// Create the (initially hidden) seat-adjust window.
unsafe fn create_seat_window() {
    let (mut sl, mut st, mut _sr, mut _sb) = (0, 0, 0, 0);
    XPLMGetScreenBoundsGlobal(&mut sl, &mut st, &mut _sr, &mut _sb);
    let (w, h) = (170, 200);
    let left = sl + 60;
    let top = st - 140;
    let mut params = XPLMCreateWindow_t {
        structSize: std::mem::size_of::<XPLMCreateWindow_t>() as i32,
        left,
        top,
        right: left + w,
        bottom: top - h,
        visible: 0, // start hidden; toggled via menu / command
        drawWindowFunc: Some(seat_draw),
        handleMouseClickFunc: Some(seat_click),
        handleKeyFunc: Some(seat_key),
        handleCursorFunc: Some(seat_cursor),
        handleMouseWheelFunc: Some(seat_wheel),
        refcon: std::ptr::null_mut(),
        decorateAsFloatingWindow: XPLM_WINDOW_DECORATION_ROUND_RECTANGLE,
        layer: XPLM_WINDOW_LAYER_FLOATING,
        handleRightClickFunc: Some(seat_rightclick),
    };
    let win = XPLMCreateWindowEx(&mut params);
    XPLMSetWindowPositioningMode(win, XPLM_WINDOW_POSITION_FREE, -1);
    XPLMSetWindowTitle(win, cstr("HeadtrackRS Seat").as_ptr());
    SEAT_WINDOW.set(win).ok();
}

unsafe fn toggle_seat_window() {
    if let Some(&win) = SEAT_WINDOW.get() {
        let vis = XPLMGetWindowIsVisible(win);
        XPLMSetWindowIsVisible(win, if vis != 0 { 0 } else { 1 });
    }
}

unsafe extern "C" fn seat_panel_command(
    _cmd: XPLMCommandRef,
    phase: i32,
    _refcon: *mut c_void,
) -> i32 {
    if phase == 0 {
        toggle_seat_window();
    }
    1
}

// ── Flight loop ───────────────────────────────────────────────────────────────

/// Called every flight model frame.  Reads the latest pose, interpolates
/// smoothly toward it, and writes DataRefs.
unsafe extern "C" fn flight_loop(
    elapsed: f32,
    _elapsed_sim: f32,
    _counter: i32,
    _refcon: *mut c_void,
) -> f32 {
    // Hold-to-slide: while a panel arrow is held with the mouse, nudge the seat
    // every frame. Runs regardless of tracking state or current view.
    let held = SEAT_HELD;
    if let Some(code) = held {
        apply_seat_action(code);
    }

    // Update status menu item on connection state change.
    let connected = ipc::CONNECTED.load(Ordering::Relaxed);
    if connected != LAST_CONNECTED {
        LAST_CONNECTED = connected;
        if let Some(&submenu) = SUBMENU.get() {
            let label = if connected {
                cstr("Status: Connected")
            } else {
                cstr("Status: Disconnected")
            };
            XPLMSetMenuItemName(submenu, MENU_STATUS, label.as_ptr(), 0);
        }
    }

    if !ENABLED.load(Ordering::Relaxed) {
        return -1.0;
    }

    let Some(dr) = DATAREFS.get() else {
        return -1.0;
    };

    // Only write when in 3D cockpit view.
    let view_type = XPLMGetDatai(dr.view_type);
    if view_type != VIEW_TYPE_3D_COCKPIT {
        return -1.0;
    }

    // Re-read acf_pe when flagged by PLANE_LOADED (deferred to flight loop
    // because DataRefs aren't populated yet at message-handler time).
    if MUST_RESET_VIEWPORT.load(Ordering::Relaxed) {
        capture_viewport_ref(dr);
        load_seat(dr);
        MUST_RESET_VIEWPORT.store(false, Ordering::Relaxed);
    }

    let Some(pose_arc) = LATEST_POSE.get() else {
        return -1.0;
    };
    let Ok(guard) = pose_arc.try_lock() else {
        return -1.0;
    };
    let Some(pose) = *guard else { return -1.0 };

    // Target values with axis signs applied.
    let target = [
        SIGN_YAW * pose.yaw,
        SIGN_PITCH * pose.pitch,
        SIGN_ROLL * pose.roll,
        SIGN_X * pose.x / 1000.0,
        SIGN_Y * pose.y / 1000.0,
        SIGN_Z * pose.z / 1000.0,
    ];

    // Exponential lerp: close `alpha` fraction of the gap each frame.
    // At 60 fps, alpha ≈ 0.22 per frame → smooth motion, no staircase.
    let alpha = (LERP_SPEED * elapsed.max(0.001)).min(1.0);
    for i in 0..6 {
        SMOOTH_POSE[i] += alpha * (target[i] - SMOOTH_POSE[i]);
    }

    let vr = VIEWPORT_REF;
    let seat = SEAT_OFFSET;

    XPLMSetDataf(dr.yaw, SMOOTH_POSE[0]);
    XPLMSetDataf(dr.pitch, SMOOTH_POSE[1]);
    XPLMSetDataf(dr.roll, SMOOTH_POSE[2]);
    XPLMSetDataf(dr.x, vr[0] + seat[0] + SMOOTH_POSE[3]);
    XPLMSetDataf(dr.y, vr[1] + seat[1] + SMOOTH_POSE[4]);
    XPLMSetDataf(dr.z, vr[2] + seat[2] + SMOOTH_POSE[5]);

    -1.0 // call every frame
}

// ── Menu ──────────────────────────────────────────────────────────────────────

/// Handle the Enable/Disable toggle menu item.
///
/// Flips the `ENABLED` atomic, updates the menu item label and check mark,
/// and — when disabling — zeroes the interpolation state and restores the
/// default pilot head position via DataRefs.
unsafe fn handle_enable_toggle() {
    let was_enabled = ENABLED.fetch_xor(true, Ordering::Relaxed);
    let now_enabled = !was_enabled;

    if let Some(&submenu) = SUBMENU.get() {
        XPLMCheckMenuItem(submenu, MENU_ENABLE, if now_enabled { 1 } else { 0 });
        let label = if now_enabled {
            cstr("Enable Tracking")
        } else {
            cstr("Tracking Disabled")
        };
        XPLMSetMenuItemName(submenu, MENU_ENABLE, label.as_ptr(), 0);
    }

    if !now_enabled {
        if let Some(dr) = DATAREFS.get() {
            SMOOTH_POSE = [0.0; 6];
            let vr = VIEWPORT_REF;
            let seat = SEAT_OFFSET;
            XPLMSetDataf(dr.yaw, 0.0);
            XPLMSetDataf(dr.pitch, 0.0);
            XPLMSetDataf(dr.roll, 0.0);
            XPLMSetDataf(dr.x, vr[0] + seat[0]);
            XPLMSetDataf(dr.y, vr[1] + seat[1]);
            XPLMSetDataf(dr.z, vr[2] + seat[2]);
        }
    }

    debug(&format!(
        "headtrack-rs: tracking {}\n",
        if now_enabled { "enabled" } else { "disabled" }
    ));
}

unsafe extern "C" fn menu_handler(_menu_ref: *mut c_void, item_ref: *mut c_void) {
    match item_ref as i32 {
        MENU_ENABLE => handle_enable_toggle(),
        MENU_RECENTER => do_recenter(),
        MENU_SEAT_PANEL => toggle_seat_window(),
        MENU_SEAT_FWD => seat_menu_action(1),
        MENU_SEAT_BACK => seat_menu_action(2),
        MENU_SEAT_UP => seat_menu_action(3),
        MENU_SEAT_DOWN => seat_menu_action(4),
        MENU_SEAT_LEFT => seat_menu_action(5),
        MENU_SEAT_RIGHT => seat_menu_action(6),
        MENU_SEAT_RESET => seat_menu_action(7),
        _ => {}
    }
}

// ── Plugin entry points ───────────────────────────────────────────────────────

/// # Safety
/// Called by X-Plane; buffers are valid for at least 256 bytes each.
#[no_mangle]
pub unsafe extern "C" fn XPluginStart(
    out_name: *mut i8,
    out_sig: *mut i8,
    out_desc: *mut i8,
) -> i32 {
    write_cstr(out_name, "headtrack-rs");
    write_cstr(out_sig, "com.headtrack-rs.plugin");
    write_cstr(
        out_desc,
        "6DoF head tracking via webcam — connects to headtrack-rs daemon",
    );

    let dr = DataRefs {
        view_type: find_dataref("sim/graphics/view/view_type"),
        yaw: find_dataref("sim/graphics/view/pilots_head_psi"),
        pitch: find_dataref("sim/graphics/view/pilots_head_the"),
        roll: find_dataref("sim/graphics/view/pilots_head_phi"),
        x: find_dataref("sim/graphics/view/pilots_head_x"),
        y: find_dataref("sim/graphics/view/pilots_head_y"),
        z: find_dataref("sim/graphics/view/pilots_head_z"),
        acf_pe_x: find_dataref("sim/aircraft/view/acf_peX"),
        acf_pe_y: find_dataref("sim/aircraft/view/acf_peY"),
        acf_pe_z: find_dataref("sim/aircraft/view/acf_peZ"),
        acf_relative_path: find_dataref("sim/aircraft/view/acf_relative_path"),
        acf_icao: find_dataref("sim/aircraft/view/acf_ICAO"),
    };
    DATAREFS.set(dr).ok();

    // Start IPC receiver.
    let socket_path = headtrack_ipc::socket_path();
    let pose_arc = ipc::start(socket_path);
    LATEST_POSE.set(pose_arc).ok();

    // Register per-frame flight loop.
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0, std::ptr::null_mut());

    // Build Plugins menu.
    let plugins_menu = XPLMFindPluginsMenu();
    let item = XPLMAppendMenuItem(
        plugins_menu,
        cstr("headtrack-rs").as_ptr(),
        std::ptr::null_mut(),
        0,
    );
    let submenu = XPLMCreateMenu(
        cstr("headtrack-rs").as_ptr(),
        plugins_menu,
        item,
        menu_handler,
        std::ptr::null_mut(),
    );
    SUBMENU.set(submenu).ok();

    // Enable — checkbox, starts checked
    XPLMAppendMenuItem(
        submenu,
        cstr("Enable Tracking").as_ptr(),
        MENU_ENABLE as *mut c_void,
        0,
    );
    XPLMCheckMenuItem(submenu, MENU_ENABLE, 1); // checked on start

    // Recenter
    XPLMAppendMenuItem(
        submenu,
        cstr("Recenter").as_ptr(),
        MENU_RECENTER as *mut c_void,
        0,
    );

    // Seat position adjust (also exposed as bindable commands below)
    XPLMAppendMenuSeparator(submenu);
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Adjuster").as_ptr(),
        MENU_SEAT_PANEL as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Forward").as_ptr(),
        MENU_SEAT_FWD as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Back").as_ptr(),
        MENU_SEAT_BACK as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Up").as_ptr(),
        MENU_SEAT_UP as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Down").as_ptr(),
        MENU_SEAT_DOWN as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Left").as_ptr(),
        MENU_SEAT_LEFT as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Seat Right").as_ptr(),
        MENU_SEAT_RIGHT as *mut c_void,
        0,
    );
    XPLMAppendMenuItem(
        submenu,
        cstr("Reset Seat").as_ptr(),
        MENU_SEAT_RESET as *mut c_void,
        0,
    );

    // Separator + status line
    XPLMAppendMenuSeparator(submenu);
    XPLMAppendMenuItem(
        submenu,
        cstr("Status: Disconnected").as_ptr(),
        MENU_STATUS as *mut c_void,
        0,
    );

    // Bindable commands (X-Plane → Settings → Keyboard / Joystick → "headtrack-rs").
    register_commands();

    // On-screen seat-adjust panel (hidden until toggled).
    create_seat_window();

    debug("headtrack-rs: plugin started\n");
    1
}

/// # Safety
/// Called by X-Plane on plugin shutdown; all X-Plane APIs are valid at this point.
#[no_mangle]
pub unsafe extern "C" fn XPluginStop() {
    XPLMUnregisterFlightLoopCallback(flight_loop, std::ptr::null_mut());
    if let Some(cmds) = SEAT_COMMANDS.get() {
        for &(cmd, code) in cmds {
            XPLMUnregisterCommandHandler(cmd, seat_command, 1, code as *mut c_void);
        }
    }
    if let Some(&recenter) = RECENTER_COMMAND.get() {
        XPLMUnregisterCommandHandler(recenter, recenter_command, 1, std::ptr::null_mut());
    }
    if let Some(&panel) = SEAT_PANEL_COMMAND.get() {
        XPLMUnregisterCommandHandler(panel, seat_panel_command, 1, std::ptr::null_mut());
    }
    if let Some(&win) = SEAT_WINDOW.get() {
        XPLMDestroyWindow(win);
    }
    debug("headtrack-rs: plugin stopped\n");
}

/// # Safety
/// Called by X-Plane when the plugin is enabled; X-Plane APIs are valid.
#[no_mangle]
pub unsafe extern "C" fn XPluginEnable() -> i32 {
    ENABLED.store(true, Ordering::Relaxed);
    1
}

/// # Safety
/// Called by X-Plane when the plugin is disabled; X-Plane APIs are valid.
#[no_mangle]
pub unsafe extern "C" fn XPluginDisable() {
    ENABLED.store(false, Ordering::Relaxed);
}

/// X-Plane message IDs.
const XPLM_MSG_PLANE_LOADED: i32 = 102;

/// # Safety
/// Called by X-Plane with a valid message ID; `_param` lifetime is managed by X-Plane.
#[no_mangle]
pub unsafe extern "C" fn XPluginReceiveMessage(_from: i32, msg: i32, _param: *mut c_void) {
    if msg == XPLM_MSG_PLANE_LOADED {
        // Flag for the flight loop to re-read acf_pe on the next frame.
        // We don't read here because DataRefs may not be populated yet.
        MUST_RESET_VIEWPORT.store(true, Ordering::Relaxed);
    }
}

// ── Utilities ─────────────────────────────────────────────────────────────────

unsafe fn write_cstr(dst: *mut i8, src: &str) {
    assert!(
        src.len() < 256,
        "X-Plane plugin string buffers are 256 bytes"
    );
    let bytes = src.as_bytes();
    std::ptr::copy_nonoverlapping(bytes.as_ptr() as *const i8, dst, bytes.len());
    *dst.add(bytes.len()) = 0;
}
