//! Minimal hand-written FFI declarations for the XPLM functions we use.
//!
//! Only binds what the plugin actually calls — no generated bloat.
//! All symbols are resolved by X-Plane at plugin load time; the build
//! uses `--allow-shlib-undefined` so they don't need a link-time stub.

use std::ffi::c_void;

// ── Opaque handles ────────────────────────────────────────────────────────────

/// Opaque handle to an X-Plane DataRef.  Valid for the lifetime of the sim.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct XPLMDataRef(*mut c_void);

// Safe: X-Plane DataRef handles are stable pointers valid across threads.
unsafe impl Send for XPLMDataRef {}
unsafe impl Sync for XPLMDataRef {}

/// Opaque handle to an X-Plane menu.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct XPLMMenuID(pub *mut c_void);

unsafe impl Send for XPLMMenuID {}
unsafe impl Sync for XPLMMenuID {}

// ── Callback types ────────────────────────────────────────────────────────────

#[allow(non_camel_case_types)]
/// Flight loop callback signature.
/// Return value: seconds until next call (negative = fractions of a frame).
pub type XPLMFlightLoop_f = unsafe extern "C" fn(
    elapsed_since_last_call: f32,
    elapsed_since_last_loop: f32,
    counter: i32,
    refcon: *mut c_void,
) -> f32;

#[allow(non_camel_case_types)]
/// Plugins menu handler signature.
pub type XPLMMenuHandler_f =
    unsafe extern "C" fn(menu_ref: *mut c_void, item_ref: *mut c_void);

// ── XPLM API declarations ─────────────────────────────────────────────────────

#[allow(dead_code)]
extern "C" {
    // DataAccess
    pub fn XPLMFindDataRef(name: *const i8) -> XPLMDataRef;
    pub fn XPLMGetDatai(data_ref: XPLMDataRef) -> i32;
    pub fn XPLMGetDataf(data_ref: XPLMDataRef) -> f32;
    pub fn XPLMSetDataf(data_ref: XPLMDataRef, value: f32);

    // Processing (flight loop)
    pub fn XPLMRegisterFlightLoopCallback(
        callback: XPLMFlightLoop_f,
        interval: f32,
        refcon: *mut c_void,
    );
    pub fn XPLMUnregisterFlightLoopCallback(
        callback: XPLMFlightLoop_f,
        refcon: *mut c_void,
    );

    // Menus
    pub fn XPLMFindPluginsMenu() -> XPLMMenuID;
    pub fn XPLMCreateMenu(
        name: *const i8,
        parent_menu: XPLMMenuID,
        parent_item: i32,
        handler: XPLMMenuHandler_f,
        menu_ref: *mut c_void,
    ) -> XPLMMenuID;
    pub fn XPLMAppendMenuItem(
        menu: XPLMMenuID,
        item_name: *const i8,
        item_ref: *mut c_void,
        deprecated: i32,
    ) -> i32;
    pub fn XPLMAppendMenuSeparator(menu: XPLMMenuID);
    pub fn XPLMCheckMenuItem(menu: XPLMMenuID, index: i32, checked: i32);
    pub fn XPLMSetMenuItemName(menu: XPLMMenuID, index: i32, name: *const i8, deprecated: i32);
    pub fn XPLMEnableMenuItem(menu: XPLMMenuID, index: i32, enabled: i32);

    // Utilities
    pub fn XPLMDebugString(string: *const i8);
}

/// Convenience: log a Rust string to X-Plane's log.txt.
pub fn debug(s: &str) {
    let c = std::ffi::CString::new(s).unwrap_or_default();
    unsafe { XPLMDebugString(c.as_ptr()) };
}
