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
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex, OnceLock,
};

mod ffi;
mod ipc;

use ffi::*;
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

/// Menu item indices (within our submenu).
/// Separators count as items in X-Plane's menu system.
const MENU_ENABLE: i32 = 0;
const MENU_RECENTER: i32 = 1;
#[allow(dead_code)]
const MENU_SEPARATOR: i32 = 2; // separator occupies an index
const MENU_STATUS: i32 = 3;

// Axis sign corrections — flip any that feel backwards in-sim.
// +1.0 = pass through, -1.0 = invert.
const SIGN_YAW:   f32 =  1.0;
const SIGN_PITCH: f32 =  1.0;
const SIGN_ROLL:  f32 =  1.0;
const SIGN_X:     f32 = -1.0;
const SIGN_Y:     f32 =  1.0;
const SIGN_Z:     f32 = -0.5; // depth-from-face-size — inverted + scaled (slew limiter + heavy One Euro tame noise)

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
    debug(&format!("headtrack-rs: viewport_ref = [{x:.4}, {y:.4}, {z:.4}]\n"));
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

    let Some(dr) = DATAREFS.get() else { return -1.0 };

    // Only write when in 3D cockpit view.
    let view_type = XPLMGetDatai(dr.view_type);
    if view_type != VIEW_TYPE_3D_COCKPIT {
        return -1.0;
    }

    // Re-read acf_pe when flagged by PLANE_LOADED (deferred to flight loop
    // because DataRefs aren't populated yet at message-handler time).
    if MUST_RESET_VIEWPORT.load(Ordering::Relaxed) {
        capture_viewport_ref(dr);
        MUST_RESET_VIEWPORT.store(false, Ordering::Relaxed);
    }

    let Some(pose_arc) = LATEST_POSE.get() else { return -1.0 };
    let Ok(guard) = pose_arc.try_lock() else { return -1.0 };
    let Some(pose) = *guard else { return -1.0 };

    // Target values with axis signs applied.
    let target = [
        SIGN_YAW   * pose.yaw,
        SIGN_PITCH * pose.pitch,
        SIGN_ROLL  * pose.roll,
        SIGN_X     * pose.x / 1000.0,
        SIGN_Y     * pose.y / 1000.0,
        SIGN_Z     * pose.z / 1000.0,
    ];

    // Exponential lerp: close `alpha` fraction of the gap each frame.
    // At 60 fps, alpha ≈ 0.22 per frame → smooth motion, no staircase.
    let alpha = (LERP_SPEED * elapsed.max(0.001)).min(1.0);
    for i in 0..6 {
        SMOOTH_POSE[i] += alpha * (target[i] - SMOOTH_POSE[i]);
    }

    let vr = VIEWPORT_REF;

    XPLMSetDataf(dr.yaw,   SMOOTH_POSE[0]);
    XPLMSetDataf(dr.pitch, SMOOTH_POSE[1]);
    XPLMSetDataf(dr.roll,  SMOOTH_POSE[2]);
    XPLMSetDataf(dr.x,     vr[0] + SMOOTH_POSE[3]);
    XPLMSetDataf(dr.y,     vr[1] + SMOOTH_POSE[4]);
    XPLMSetDataf(dr.z,     vr[2] + SMOOTH_POSE[5]);

    -1.0 // call every frame
}

// ── Menu ──────────────────────────────────────────────────────────────────────

unsafe extern "C" fn menu_handler(_menu_ref: *mut c_void, item_ref: *mut c_void) {
    let item = item_ref as i32;
    match item {
        MENU_ENABLE => {
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

            // When disabling, return head to default pilot position.
            if !now_enabled {
                if let Some(dr) = DATAREFS.get() {
                    // Zero out interpolation state.
                    SMOOTH_POSE = [0.0; 6];
                    let vr = VIEWPORT_REF;
                    XPLMSetDataf(dr.yaw,   0.0);
                    XPLMSetDataf(dr.pitch, 0.0);
                    XPLMSetDataf(dr.roll,  0.0);
                    XPLMSetDataf(dr.x,     vr[0]);
                    XPLMSetDataf(dr.y,     vr[1]);
                    XPLMSetDataf(dr.z,     vr[2]);
                }
            }

            debug(&format!(
                "headtrack-rs: tracking {}\n",
                if now_enabled { "enabled" } else { "disabled" }
            ));
        }
        MENU_RECENTER => {
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
    write_cstr(out_sig,  "com.headtrack-rs.plugin");
    write_cstr(out_desc, "6DoF head tracking via webcam — connects to headtrack-rs daemon");

    let dr = DataRefs {
        view_type: find_dataref("sim/graphics/view/view_type"),
        yaw:       find_dataref("sim/graphics/view/pilots_head_psi"),
        pitch:     find_dataref("sim/graphics/view/pilots_head_the"),
        roll:      find_dataref("sim/graphics/view/pilots_head_phi"),
        x:         find_dataref("sim/graphics/view/pilots_head_x"),
        y:         find_dataref("sim/graphics/view/pilots_head_y"),
        z:         find_dataref("sim/graphics/view/pilots_head_z"),
        acf_pe_x:  find_dataref("sim/aircraft/view/acf_peX"),
        acf_pe_y:  find_dataref("sim/aircraft/view/acf_peY"),
        acf_pe_z:  find_dataref("sim/aircraft/view/acf_peZ"),
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
    XPLMAppendMenuItem(submenu, cstr("Enable Tracking").as_ptr(), MENU_ENABLE as *mut c_void, 0);
    XPLMCheckMenuItem(submenu, MENU_ENABLE, 1); // checked on start

    // Recenter
    XPLMAppendMenuItem(submenu, cstr("Recenter").as_ptr(), MENU_RECENTER as *mut c_void, 0);

    // Separator + status line
    XPLMAppendMenuSeparator(submenu);
    XPLMAppendMenuItem(submenu, cstr("Status: Disconnected").as_ptr(), MENU_STATUS as *mut c_void, 0);

    debug("headtrack-rs: plugin started\n");
    1
}

#[no_mangle]
pub unsafe extern "C" fn XPluginStop() {
    XPLMUnregisterFlightLoopCallback(flight_loop, std::ptr::null_mut());
    debug("headtrack-rs: plugin stopped\n");
}

#[no_mangle]
pub unsafe extern "C" fn XPluginEnable() -> i32 {
    ENABLED.store(true, Ordering::Relaxed);
    1
}

#[no_mangle]
pub unsafe extern "C" fn XPluginDisable() {
    ENABLED.store(false, Ordering::Relaxed);
}

/// X-Plane message IDs.
const XPLM_MSG_PLANE_LOADED: i32 = 102;

#[no_mangle]
pub unsafe extern "C" fn XPluginReceiveMessage(
    _from: i32,
    msg: i32,
    _param: *mut c_void,
) {
    if msg == XPLM_MSG_PLANE_LOADED {
        // Flag for the flight loop to re-read acf_pe on the next frame.
        // We don't read here because DataRefs may not be populated yet.
        MUST_RESET_VIEWPORT.store(true, Ordering::Relaxed);
    }
}

// ── Utilities ─────────────────────────────────────────────────────────────────

unsafe fn write_cstr(dst: *mut i8, src: &str) {
    debug_assert!(src.len() < 256, "X-Plane plugin string buffers are 256 bytes");
    let bytes = src.as_bytes();
    std::ptr::copy_nonoverlapping(bytes.as_ptr() as *const i8, dst, bytes.len());
    *dst.add(bytes.len()) = 0;
}
