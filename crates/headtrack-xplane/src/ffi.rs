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

impl XPLMDataRef {
    /// True if the DataRef was not found (XPLMFindDataRef returned null).
    pub fn is_null(&self) -> bool {
        self.0.is_null()
    }
}

/// Opaque handle to an X-Plane command.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct XPLMCommandRef(*mut c_void);

unsafe impl Send for XPLMCommandRef {}
unsafe impl Sync for XPLMCommandRef {}

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

#[allow(non_camel_case_types)]
/// Command handler signature.
/// `phase`: 0 = begin, 1 = continue, 2 = end.
/// Return 1 to let other handlers/X-Plane also process the command, 0 to consume it.
pub type XPLMCommandCallback_f =
    unsafe extern "C" fn(command: XPLMCommandRef, phase: i32, refcon: *mut c_void) -> i32;

// ── Window API (modern XPLMDisplay) ────────────────────────────────────────────

/// Opaque handle to an X-Plane window.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct XPLMWindowID(*mut c_void);
unsafe impl Send for XPLMWindowID {}
unsafe impl Sync for XPLMWindowID {}

// XPLMMouseStatus — the full enum is transcribed so the values stay
// self-documenting next to each other; only DOWN and UP are dispatched on.
pub const XPLM_MOUSE_DOWN: i32 = 1;
#[allow(dead_code)]
pub const XPLM_MOUSE_DRAG: i32 = 2;
pub const XPLM_MOUSE_UP: i32 = 3;
// XPLMWindowDecoration
pub const XPLM_WINDOW_DECORATION_ROUND_RECTANGLE: i32 = 1;
// XPLMWindowLayer
pub const XPLM_WINDOW_LAYER_FLOATING: i32 = 1;
// XPLMWindowPositioningMode
pub const XPLM_WINDOW_POSITION_FREE: i32 = 0;
// XPLMCursorStatus
pub const XPLM_CURSOR_DEFAULT: i32 = 0;
// XPLMFontID
pub const XPLM_FONT_PROPORTIONAL: i32 = 18;

#[allow(non_camel_case_types)]
pub type XPLMDrawWindow_f = unsafe extern "C" fn(window: XPLMWindowID, refcon: *mut c_void);
#[allow(non_camel_case_types)]
pub type XPLMHandleMouseClick_f =
    unsafe extern "C" fn(window: XPLMWindowID, x: i32, y: i32, mouse: i32, refcon: *mut c_void) -> i32;
#[allow(non_camel_case_types)]
pub type XPLMHandleKey_f = unsafe extern "C" fn(
    window: XPLMWindowID,
    key: i8,
    flags: i32,
    virtual_key: i8,
    refcon: *mut c_void,
    losing_focus: i32,
);
#[allow(non_camel_case_types)]
pub type XPLMHandleCursor_f =
    unsafe extern "C" fn(window: XPLMWindowID, x: i32, y: i32, refcon: *mut c_void) -> i32;
#[allow(non_camel_case_types)]
pub type XPLMHandleMouseWheel_f =
    unsafe extern "C" fn(window: XPLMWindowID, x: i32, y: i32, wheel: i32, clicks: i32, refcon: *mut c_void) -> i32;

/// Mirrors `XPLMCreateWindow_t` from the current X-Plane SDK (XPLM400).
/// Field order and `structSize` must match the SDK exactly or the sim crashes.
#[repr(C)]
#[allow(non_snake_case)]
pub struct XPLMCreateWindow_t {
    pub structSize: i32,
    pub left: i32,
    pub top: i32,
    pub right: i32,
    pub bottom: i32,
    pub visible: i32,
    pub drawWindowFunc: Option<XPLMDrawWindow_f>,
    pub handleMouseClickFunc: Option<XPLMHandleMouseClick_f>,
    pub handleKeyFunc: Option<XPLMHandleKey_f>,
    pub handleCursorFunc: Option<XPLMHandleCursor_f>,
    pub handleMouseWheelFunc: Option<XPLMHandleMouseWheel_f>,
    pub refcon: *mut c_void,
    pub decorateAsFloatingWindow: i32,
    pub layer: i32,
    pub handleRightClickFunc: Option<XPLMHandleMouseClick_f>,
}

// ── XPLM API declarations ─────────────────────────────────────────────────────

// Hand-written XPLM bindings. The block carries a blanket `dead_code` allow on
// purpose: these are *foreign* declarations transcribed from the X-Plane SDK
// headers, and their value is being a faithful, complete transcription. A
// per-item allow on each currently-unused entry would be noise that has to be
// churned every time the plugin starts or stops calling one.
#[allow(dead_code)]
extern "C" {
    // DataAccess
    pub fn XPLMFindDataRef(name: *const i8) -> XPLMDataRef;
    pub fn XPLMGetDatai(data_ref: XPLMDataRef) -> i32;
    pub fn XPLMGetDataf(data_ref: XPLMDataRef) -> f32;
    pub fn XPLMSetDataf(data_ref: XPLMDataRef, value: f32);
    pub fn XPLMGetDatab(
        data_ref: XPLMDataRef,
        out_value: *mut c_void,
        offset: i32,
        max_bytes: i32,
    ) -> i32;

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

    // Commands (bindable to keyboard/joystick in X-Plane settings)
    pub fn XPLMCreateCommand(name: *const i8, description: *const i8) -> XPLMCommandRef;
    pub fn XPLMRegisterCommandHandler(
        command: XPLMCommandRef,
        handler: XPLMCommandCallback_f,
        before: i32,
        refcon: *mut c_void,
    );
    pub fn XPLMUnregisterCommandHandler(
        command: XPLMCommandRef,
        handler: XPLMCommandCallback_f,
        before: i32,
        refcon: *mut c_void,
    );

    // Windows (modern XPLMDisplay)
    pub fn XPLMCreateWindowEx(params: *mut XPLMCreateWindow_t) -> XPLMWindowID;
    pub fn XPLMDestroyWindow(window: XPLMWindowID);
    pub fn XPLMGetWindowGeometry(
        window: XPLMWindowID,
        left: *mut i32,
        top: *mut i32,
        right: *mut i32,
        bottom: *mut i32,
    );
    pub fn XPLMGetWindowIsVisible(window: XPLMWindowID) -> i32;
    pub fn XPLMSetWindowIsVisible(window: XPLMWindowID, visible: i32);
    pub fn XPLMSetWindowPositioningMode(window: XPLMWindowID, mode: i32, monitor_index: i32);
    pub fn XPLMSetWindowTitle(window: XPLMWindowID, title: *const i8);
    pub fn XPLMGetScreenBoundsGlobal(
        left: *mut i32,
        top: *mut i32,
        right: *mut i32,
        bottom: *mut i32,
    );

    // Drawing (XPLMGraphics)
    pub fn XPLMDrawString(
        color_rgb: *mut f32,
        x: i32,
        y: i32,
        text: *mut i8,
        word_wrap_width: *mut i32,
        font: i32,
    );
    pub fn XPLMDrawTranslucentDarkBox(left: i32, top: i32, right: i32, bottom: i32);

    // Utilities
    pub fn XPLMDebugString(string: *const i8);
}

/// Convenience: log a Rust string to X-Plane's log.txt.
pub fn debug(s: &str) {
    let c = std::ffi::CString::new(s).unwrap_or_default();
    unsafe { XPLMDebugString(c.as_ptr()) };
}
