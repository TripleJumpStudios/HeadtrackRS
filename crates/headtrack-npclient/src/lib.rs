//! NPClient64.dll — fake NaturalPoint TrackIR client for headtrack-rs.
//!
//! Star Citizen loads this DLL from its Bin64/ directory when TrackIR is
//! selected as the head tracking source.  Implements the full 21-export
//! NaturalPoint NPClient API and feeds data from the headtrack-rs engine's
//! POSIX shared memory via Wine's Z: drive mapping.
//!
//! # Build
//!
//! ```
//! cargo build --manifest-path crates/headtrack-npclient/Cargo.toml \
//!             --target x86_64-pc-windows-gnu --release
//! ```
//! Requires `rust-std-static-x86_64-pc-windows-gnu` (Fedora) and `mingw64-gcc`.
//!
//! # Install
//!
//! ```
//! cp target/x86_64-pc-windows-gnu/release/NPClient64.dll \
//!    "$HOME/Games/star-citizen/drive_c/Program Files/Roberts Space Industries/StarCitizen/LIVE/Bin64/"
//! ```
//!
//! # Data format
//!
//! TrackIR headspace units: ±16383 = ±180° rotation, ±16383 = ±500mm position.
//! A checksum is computed over the data struct (required by TIR5 protocol).
//! Encryption is applied for games that require it (Star Citizen game ID 3450).
//!
//! # Safety model
//!
//! All NP_* exports are called from SC's single game thread.  The notification
//! thread only reads `RUNNING` (AtomicBool) and `G_HWND` (AtomicUsize).
//! All other mutable statics are game-thread-only and accessed without locks.
//! `catch_unwind` wraps every export so a Rust panic cannot unwind into SC.

#![allow(non_snake_case, clippy::missing_safety_doc)]

use std::panic::catch_unwind;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use winapi::shared::minwindef::{BOOL, DWORD, HINSTANCE, LPVOID, TRUE, USHORT};
use winapi::um::fileapi::{
    CreateFileA, SetFilePointer, WriteFile, OPEN_ALWAYS, OPEN_EXISTING,
};
use winapi::um::handleapi::{CloseHandle, INVALID_HANDLE_VALUE};
use winapi::um::libloaderapi::DisableThreadLibraryCalls;
use winapi::um::memoryapi::{MapViewOfFile, FILE_MAP_READ};
use winapi::um::winbase::CreateFileMappingA;
use winapi::um::processthreadsapi::{CreateThread, CreateProcessA, PROCESS_INFORMATION, STARTUPINFOA};
use winapi::um::synchapi::{Sleep, WaitForSingleObject};
use winapi::um::winnt::{
    FILE_ATTRIBUTE_NORMAL, FILE_SHARE_READ, FILE_SHARE_WRITE, GENERIC_READ,
    GENERIC_WRITE, PAGE_READONLY,
};
use winapi::um::winuser::{IsWindow, PostMessageA, WM_APP};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DLL_PROCESS_ATTACH: DWORD = 1;
const FILE_END: DWORD = 2;
const NP_AXIS_MAX: f64 = 16383.0;

const SRC_PATH: &[u8] = b"Z:\\dev\\shm\\headtrack-pose\0";
const LOG_PATH: &[u8] = b"C:\\headtrack\\npclient.log\0";

// Game ID and encryption key (from OpenTrack's facetracknoir supported games.csv).
const GAME_ID_SC:      u16    = 3450;  // Star Citizen
const ENCRYPT_KEY_SC: [u8; 8] = [0x34, 0x4E, 0xCE, 0xF4, 0xA2, 0xB4, 0xC6, 0x3E];

// ---------------------------------------------------------------------------
// Shared memory layout — must exactly match WineShmPose in
// headtrack-output-wine/src/lib.rs.
// ---------------------------------------------------------------------------

#[repr(C, packed)]
struct WineShmPose {
    sequence: u64,
    yaw:      f64,
    pitch:    f64,
    roll:     f64,
    x:        f64,
    y:        f64,
    z:        f64,
}

// ---------------------------------------------------------------------------
// NaturalPoint TrackIR data struct — matches opentrack's tir_data_t.
// ---------------------------------------------------------------------------

#[repr(C, packed)]
pub struct TirData {
    status:  i16,       // bitmask: bit0=active, bit3=tracking (0x0009 = active+tracking)
    frame:   i16,       // increments each new frame
    cksum:   u32,       // checksum over entire struct (with cksum=0)
    roll:    f32,
    pitch:   f32,
    yaw:     f32,
    tx:      f32,
    ty:      f32,
    tz:      f32,
    padding: [f32; 9],  // raw/delta/smooth — zeroed
}

#[repr(C)]
pub struct TirSignature {
    dll_signature: [u8; 200],
    app_signature: [u8; 200],
}

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

// File/map handles stored as usize (HANDLE = *mut c_void; usize::MAX = INVALID).
static mut G_HFILE: usize = usize::MAX;
static mut G_HMAP:  usize = 0;
// Pointer to the mapped WineShmPose — null until shm is open.
static mut G_POSE: *const WineShmPose = core::ptr::null();

// Game-thread-only counters.
static mut G_FRAME: i16 = 0;
static mut G_GETDATA_COUNT: i32 = 0;

// Shared between game thread (writer) and notify thread (reader).
static G_HWND:    AtomicUsize = AtomicUsize::new(0);
static G_THREAD:  AtomicUsize = AtomicUsize::new(0);
static G_RUNNING: AtomicBool  = AtomicBool::new(false);

// Encryption state (game-thread-only).
static mut G_ENCRYPT:       bool   = false;
static mut G_ENCRYPT_TABLE: [u8; 8] = [0; 8];


// ---------------------------------------------------------------------------
// Diagnostics: append a line to /tmp/headtrack-npclient.log on the Linux host.
// ---------------------------------------------------------------------------

unsafe fn diag(msg: &[u8]) {
    let h = CreateFileA(
        LOG_PATH.as_ptr() as *const i8,
        GENERIC_WRITE,
        FILE_SHARE_READ,
        core::ptr::null_mut(),
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        core::ptr::null_mut(),
    );
    if h == INVALID_HANDLE_VALUE {
        return;
    }
    SetFilePointer(h, 0, core::ptr::null_mut(), FILE_END);
    let mut written: DWORD = 0;
    WriteFile(h, msg.as_ptr() as *const _, msg.len() as DWORD, &mut written, core::ptr::null_mut());
    let crlf = b"\r\n";
    WriteFile(h, crlf.as_ptr() as *const _, 2, &mut written, core::ptr::null_mut());
    CloseHandle(h);
}

// ---------------------------------------------------------------------------
// Shared memory: open the POSIX shm written by the headtrack-rs engine.
// Wine maps /dev/shm/headtrack-pose as Z:\dev\shm\headtrack-pose.
// No-op if already open.
// ---------------------------------------------------------------------------

unsafe fn try_open_shm() {
    if !G_POSE.is_null() {
        return;
    }

    let hfile = CreateFileA(
        SRC_PATH.as_ptr() as *const i8,
        GENERIC_READ,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        core::ptr::null_mut(),
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        core::ptr::null_mut(),
    ) as usize;

    if hfile == usize::MAX {
        diag(b"shm open FAILED (engine not running?)");
        return;
    }

    let hmap = CreateFileMappingA(
        hfile as winapi::um::winnt::HANDLE,
        core::ptr::null_mut(),
        PAGE_READONLY,
        0,
        core::mem::size_of::<WineShmPose>() as DWORD,
        core::ptr::null(),
    ) as usize;

    if hmap == 0 {
        diag(b"CreateFileMapping FAILED");
        CloseHandle(hfile as winapi::um::winnt::HANDLE);
        return;
    }

    let ptr = MapViewOfFile(
        hmap as winapi::um::winnt::HANDLE,
        FILE_MAP_READ,
        0,
        0,
        core::mem::size_of::<WineShmPose>(),
    );

    if ptr.is_null() {
        diag(b"MapViewOfFile FAILED");
        CloseHandle(hmap as winapi::um::winnt::HANDLE);
        CloseHandle(hfile as winapi::um::winnt::HANDLE);
        return;
    }

    G_HFILE = hfile;
    G_HMAP  = hmap;
    G_POSE  = ptr as *const WineShmPose;
    diag(b"shm opened OK");
}


// ---------------------------------------------------------------------------
// TIR5 checksum — from opentrack, required by games.
// Ported directly from the C implementation in bridge/npclient.c.
// ---------------------------------------------------------------------------

fn tir_cksum(buf: &[u8]) -> u32 {
    let size = buf.len();
    if size == 0 {
        return 0;
    }

    let mut c = size as i32;
    let rounds = size >> 2;
    let rem = size & 3;
    let mut off = 0usize;

    for _ in 0..rounds {
        let a0 = i16::from_le_bytes([buf[off], buf[off + 1]]) as i32;
        let mut a2 = i16::from_le_bytes([buf[off + 2], buf[off + 3]]) as i32;
        off += 4;
        c += a0;
        a2 ^= c << 5;
        a2 <<= 11;
        c ^= a2;
        c += c >> 11;
    }

    let a2 = match rem {
        3 => {
            let a0 = i16::from_le_bytes([buf[off], buf[off + 1]]) as i32;
            let a2 = buf[off + 2] as i8 as i32;
            c += a0;
            let a2 = (a2 << 2) ^ c;
            c ^= a2 << 16;
            c >> 11
        }
        2 => {
            let a2 = i16::from_le_bytes([buf[off], buf[off + 1]]) as i32;
            c += a2;
            c ^= c << 11;
            c >> 17
        }
        1 => {
            let a2 = buf[off] as i8 as i32;
            c += a2;
            c ^= c << 10;
            c >> 1
        }
        _ => 0,
    };

    if rem != 0 {
        c += a2;
    }

    c ^= c << 3;
    c += c >> 5;
    c ^= c << 4;
    c += c >> 17;
    c ^= c << 25;
    c += c >> 6;

    c as u32
}

// ---------------------------------------------------------------------------
// TIR5 encryption — XOR cipher, required by games that use NP_GetSignature.
// Ported directly from the C implementation in bridge/npclient.c.
// ---------------------------------------------------------------------------

fn enhance(buf: &mut [u8], key: &[u8]) {
    if buf.is_empty() || key.is_empty() {
        return;
    }
    let mut key_ptr = 0usize;
    let mut var: u8 = 0x88;
    let mut size = buf.len();
    loop {
        size -= 1;
        let tmp = buf[size];
        buf[size] = tmp ^ key[key_ptr] ^ var;
        var = var.wrapping_add(size as u8).wrapping_add(tmp);
        key_ptr += 1;
        if key_ptr >= key.len() {
            key_ptr -= key.len();
        }
        if size == 0 {
            break;
        }
    }
}

#[inline]
fn clamp_axis(x: f64) -> f32 {
    x.clamp(-NP_AXIS_MAX, NP_AXIS_MAX) as f32
}

/// Write an i32 as decimal ASCII into buf at offset, return new offset.
fn write_i32(buf: &mut [u8], mut pos: usize, mut val: i32) -> usize {
    if val < 0 {
        buf[pos] = b'-';
        pos += 1;
        val = -val;
    }
    if val == 0 {
        buf[pos] = b'0';
        return pos + 1;
    }
    let start = pos;
    let mut v = val;
    while v > 0 {
        buf[pos] = b'0' + (v % 10) as u8;
        pos += 1;
        v /= 10;
    }
    buf[start..pos].reverse();
    pos
}

// ---------------------------------------------------------------------------
// Notification thread — posts WM_APP+3 to the game window at ~60 Hz.
// ---------------------------------------------------------------------------

unsafe extern "system" fn notify_thread(_: LPVOID) -> DWORD {
    diag(b"notify_thread started");
    let mut first_tick = true;
    while G_RUNNING.load(Ordering::Relaxed) {
        let hwnd = G_HWND.load(Ordering::Relaxed) as winapi::shared::windef::HWND;
        let is_valid = !hwnd.is_null() && IsWindow(hwnd) != 0;
        if first_tick {
            if is_valid {
                diag(b"notify_thread: IsWindow=true, posting WM_APP+3");
            } else if hwnd.is_null() {
                diag(b"notify_thread: HWND is null, NOT posting");
            } else {
                diag(b"notify_thread: IsWindow=false, NOT posting");
            }
            first_tick = false;
        }
        if is_valid {
            PostMessageA(hwnd, WM_APP + 3, 0, 0);
        }
        Sleep(16); // ~60 Hz
    }
    diag(b"notify_thread stopped");
    0
}

// ---------------------------------------------------------------------------
// DllMain
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "system" fn DllMain(
    hinst: HINSTANCE,
    reason: DWORD,
    _reserved: LPVOID,
) -> BOOL {
    if reason == DLL_PROCESS_ATTACH {
        // Phase 0 smoke test: minimal DLL load verification
        diag(b"DLL loaded successfully");
        DisableThreadLibraryCalls(hinst);
        // try_open_shm() — disabled for Phase 0 to isolate DLL load issues
    }
    TRUE
}

// ---------------------------------------------------------------------------
// NP_GetData — hot path, called at the game's render rate (~100 Hz in SC).
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "system" fn NP_GetData(data: *mut TirData) -> i32 {
    let result = catch_unwind(|| unsafe { np_get_data_inner(data) });
    result.unwrap_or(0)
}

unsafe fn np_get_data_inner(data: *mut TirData) -> i32 {
    if G_POSE.is_null() {
        try_open_shm();
    }

    // Log first 5 calls, then every 500th call for ongoing monitoring.
    if G_GETDATA_COUNT < 5 || G_GETDATA_COUNT % 500 == 0 {
        if G_POSE.is_null() {
            diag(b"NP_GetData: SHM, no pose yet");
        } else {
            diag(b"NP_GetData: SHM pose ok");
        }
    }
    G_GETDATA_COUNT += 1;

    G_FRAME = G_FRAME.wrapping_add(1);
    core::ptr::write_bytes(data, 0, 1);

    let has_data = !G_POSE.is_null() && (*G_POSE).sequence != 0;

    if !has_data {
        // status=1 and return 1 = NPCLIENT_STATUS_DISABLED (no tracker data yet).
        // Frame must still increment so MSFS doesn't think the DLL is frozen.
        (*data).status = 1;
        (*data).frame  = G_FRAME;
        let buf = core::slice::from_raw_parts(data as *const u8, core::mem::size_of::<TirData>());
        (*data).cksum = tir_cksum(buf);
        if G_ENCRYPT {
            let buf = core::slice::from_raw_parts_mut(data as *mut u8, core::mem::size_of::<TirData>());
            let key = G_ENCRYPT_TABLE;
            enhance(buf, &key);
        }
        return 1;
    }

    let pose = &*G_POSE;

    (*data).status = 0; // NPCLIENT_STATUS_OK — tracker active, data valid
    (*data).frame  = G_FRAME;
    (*data).cksum  = 0;

    // Scale degrees → headspace units (±16383 = ±180°).
    // All axes negated: headtrack-rs canonical signs are opposite TrackIR convention.
    (*data).yaw   = clamp_axis(-pose.yaw   * NP_AXIS_MAX / 180.0);
    (*data).pitch = clamp_axis(-pose.pitch * NP_AXIS_MAX / 180.0);
    (*data).roll  = clamp_axis(-pose.roll  * NP_AXIS_MAX / 180.0);

    // Scale mm → headspace units (±16383 = ±500mm).
    (*data).tx = clamp_axis(-pose.x * NP_AXIS_MAX / 500.0);
    (*data).ty = clamp_axis(-pose.y * NP_AXIS_MAX / 500.0);
    (*data).tz = clamp_axis(-pose.z * NP_AXIS_MAX / 500.0);

    // Compute checksum with cksum field = 0.
    let buf = core::slice::from_raw_parts(data as *const u8, core::mem::size_of::<TirData>());
    (*data).cksum = tir_cksum(buf);

    // Apply TIR5 encryption if the game requires it.
    if G_ENCRYPT {
        let buf = core::slice::from_raw_parts_mut(data as *mut u8, core::mem::size_of::<TirData>());
        let key = G_ENCRYPT_TABLE;
        enhance(buf, &key);
    }

    0 // NPCLIENT_STATUS_OK — data valid, tracker active
}

// ---------------------------------------------------------------------------
// NP_GetSignature — DLL/App signature (XOR-encoded, from opentrack).
// Required by Star Citizen and other games that use the Enhanced protocol.
// ---------------------------------------------------------------------------

// Signature data bytes — XOR of part1 and part2 produces the NaturalPoint
// signature strings that games use to validate the DLL.
// In C these are [200] arrays with implicit zero-fill; here we store only
// the non-zero prefix and zero-extend in NP_GetSignature.
#[rustfmt::skip]
static PART1_1: &[u8] = &[
    0x1d, 0x79, 0xce, 0x35, 0x1d, 0x95, 0x79, 0xdf, 0x4c, 0x8d, 0x55, 0xeb, 0x20, 0x17,
    0x9f, 0x26, 0x3e, 0xf0, 0x88, 0x8e, 0x7a, 0x08, 0x11, 0x52, 0xfc, 0xd8, 0x3f, 0xb9,
    0xd2, 0x5c, 0x61, 0x03, 0x56, 0xfd, 0xbc, 0xb4, 0x0a, 0xf1, 0x13, 0x5d, 0x90, 0x0a,
    0x0e, 0xee, 0x09, 0x19, 0x45, 0x5a, 0xeb, 0xe3, 0xf0, 0x58, 0x5f, 0xac, 0x23, 0x84,
    0x1f, 0xc5, 0xe3, 0xa6, 0x18, 0x5d, 0xb8, 0x47, 0xdc, 0xe6, 0xf2, 0x0b, 0x03, 0x55,
    0x61, 0xab, 0xe3, 0x57, 0xe3, 0x67, 0xcc, 0x16, 0x38, 0x3c, 0x11, 0x25, 0x88, 0x8a,
    0x24, 0x7f, 0xf7, 0xeb, 0xf2, 0x5d, 0x82, 0x89, 0x05, 0x53, 0x32, 0x6b, 0x28, 0x54,
    0x13, 0xf6, 0xe7, 0x21, 0x1a, 0xc6, 0xe3, 0xe1, 0xff,
];

#[rustfmt::skip]
static PART1_2: &[u8] = &[
    0x6d, 0x0b, 0xab, 0x56, 0x74, 0xe6, 0x1c, 0xff, 0x24, 0xe8, 0x34, 0x8f, 0x00, 0x63,
    0xed, 0x47, 0x5d, 0x9b, 0xe1, 0xe0, 0x1d, 0x02, 0x31, 0x22, 0x89, 0xac, 0x1f, 0xc0,
    0xbd, 0x29, 0x13, 0x23, 0x3e, 0x98, 0xdd, 0xd0, 0x2a, 0x98, 0x7d, 0x29, 0xff, 0x2a,
    0x7a, 0x86, 0x6c, 0x39, 0x22, 0x3b, 0x86, 0x86, 0xfa, 0x78, 0x31, 0xc3, 0x54, 0xa4,
    0x78, 0xaa, 0xc3, 0xca, 0x77, 0x32, 0xd3, 0x67, 0xbd, 0x94, 0x9d, 0x7e, 0x6d, 0x31,
    0x6b, 0xa1, 0xc3, 0x14, 0x8c, 0x17, 0xb5, 0x64, 0x51, 0x5b, 0x79, 0x51, 0xa8, 0xcf,
    0x5d, 0x1a, 0xb4, 0x84, 0x9c, 0x29, 0xf0, 0xe6, 0x69, 0x73, 0x66, 0x0e, 0x4b, 0x3c,
    0x7d, 0x99, 0x8b, 0x4e, 0x7d, 0xaf, 0x86, 0x92, 0xff,
];

#[rustfmt::skip]
static PART2_1: &[u8] = &[
    0x8b, 0x84, 0xfc, 0x8c, 0x71, 0xb5, 0xd9, 0xaa, 0xda, 0x32, 0xc7, 0xe9, 0x0c, 0x20,
    0x40, 0xd4, 0x4b, 0x02, 0x89, 0xca, 0xde, 0x61, 0x9d, 0xfb, 0xb3, 0x8c, 0x97, 0x8a,
    0x13, 0x6a, 0x0f, 0xf8, 0xf8, 0x0d, 0x65, 0x1b, 0xe3, 0x05, 0x1e, 0xb6, 0xf6, 0xd9,
    0x13, 0xad, 0xeb, 0x38, 0xdd, 0x86, 0xfc, 0x59, 0x2e, 0xf6, 0x2e, 0xf4, 0xb0, 0xb0,
    0xfd, 0xb0, 0x70, 0x23, 0xfb, 0xc9, 0x1a, 0x50, 0x89, 0x92, 0xf0, 0x01, 0x09, 0xa1,
    0xfd, 0x5b, 0x19, 0x29, 0x73, 0x59, 0x2b, 0x81, 0x83, 0x9e, 0x11, 0xf3, 0xa2, 0x1f,
    0xc8, 0x24, 0x53, 0x60, 0x0a, 0x42, 0x78, 0x7a, 0x39, 0xea, 0xc1, 0x59, 0xad, 0xc5,
    0x00,
];

#[rustfmt::skip]
static PART2_2: &[u8] = &[
    0xe3, 0xe5, 0x8e, 0xe8, 0x06, 0xd4, 0xab, 0xcf, 0xfa, 0x51, 0xa6, 0x84, 0x69, 0x52,
    0x21, 0xde, 0x6b, 0x71, 0xe6, 0xac, 0xaa, 0x16, 0xfc, 0x89, 0xd6, 0xac, 0xe7, 0xf8,
    0x7c, 0x09, 0x6a, 0x8b, 0x8b, 0x64, 0x0b, 0x7c, 0xc3, 0x61, 0x7f, 0xc2, 0x97, 0xd3,
    0x33, 0xd9, 0x99, 0x59, 0xbe, 0xed, 0xdc, 0x2c, 0x5d, 0x93, 0x5c, 0xd4, 0xdd, 0xdf,
    0x8b, 0xd5, 0x1d, 0x46, 0x95, 0xbd, 0x10, 0x5a, 0xa9, 0xd1, 0x9f, 0x71, 0x70, 0xd3,
    0x94, 0x3c, 0x71, 0x5d, 0x53, 0x1c, 0x52, 0xe4, 0xc0, 0xf1, 0x7f, 0x87, 0xd0, 0x70,
    0xa4, 0x04, 0x07, 0x05, 0x69, 0x2a, 0x16, 0x15, 0x55, 0x85, 0xa6, 0x30, 0xc8, 0xb6,
    0x00,
];

/// Get byte at index, returning 0 for out-of-range (matches C's implicit zero-fill).
#[inline]
fn sig_byte(data: &[u8], i: usize) -> u8 {
    if i < data.len() { data[i] } else { 0 }
}

#[no_mangle]
pub unsafe extern "system" fn NP_GetSignature(sig: *mut TirSignature) -> i32 {
    diag(b"NP_GetSignature");
    for i in 0..200 {
        (*sig).dll_signature[i] = sig_byte(PART1_2, i) ^ sig_byte(PART1_1, i);
        (*sig).app_signature[i] = sig_byte(PART2_1, i) ^ sig_byte(PART2_2, i);
    }
    0
}

// ---------------------------------------------------------------------------
// Remaining NP_* functions
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "system" fn NP_QueryVersion(ver: *mut USHORT) -> i32 {
    diag(b"NP_QueryVersion");
    if !ver.is_null() {
        *ver = 0x0500; // TrackIR 5
    }
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_ReCenter() -> i32 {
    diag(b"NP_ReCenter");
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_RegisterProgramProfileID(id: USHORT) -> i32 {
    match id {
        GAME_ID_SC => {
            G_ENCRYPT_TABLE = ENCRYPT_KEY_SC;
            G_ENCRYPT = true;
            diag(b"NP_RegisterProgramProfileID: SC (3450), encryption ON");
        }
        _ => {
            G_ENCRYPT_TABLE = [0; 8];
            G_ENCRYPT = false;
            diag(b"NP_RegisterProgramProfileID: unknown game, encryption OFF");
        }
    }
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_RegisterWindowHandle(hwnd: *mut core::ffi::c_void) -> i32 {
    let hwnd_val = hwnd as usize;
    if hwnd_val == 0 {
        diag(b"NP_RegisterWindowHandle: HWND is NULL");
    } else {
        let mut msg = [0u8; 60];
        let prefix = b"NP_RegisterWindowHandle: hwnd=";
        msg[..prefix.len()].copy_from_slice(prefix);
        let pos = write_i32(&mut msg, prefix.len(), hwnd_val as i32);
        diag(&msg[..pos]);
    }
    G_HWND.store(hwnd_val, Ordering::Relaxed);
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_RequestData(_flags: USHORT) -> i32 { 0 }

#[no_mangle]
pub unsafe extern "system" fn NP_StartCursor() -> i32 { 0 }

#[no_mangle]
pub unsafe extern "system" fn NP_StopCursor() -> i32 { 0 }

#[no_mangle]
pub unsafe extern "system" fn NP_GetParameter(_a: i32, _b: i32) -> i32 { 0 }

#[no_mangle]
pub unsafe extern "system" fn NP_SetParameter(_a: i32, _b: i32) -> i32 { 0 }

#[no_mangle]
pub unsafe extern "system" fn NP_StartDataTransmission() -> i32 {
    diag(b"NP_StartDataTransmission");
    G_RUNNING.store(true, Ordering::Relaxed);

    // Launch TrackIR.exe stub so MSFS's process scan finds it.
    // MSFS checks CreateToolhelp32Snapshot for "TrackIR.exe" before activating
    // NPClient polling. Launching from inside the DLL ensures it runs in the
    // same Wine prefix and is visible to MSFS's snapshot.
    {
        let exe = b"C:\\headtrack\\TrackIR.exe\0";
        let mut si: STARTUPINFOA = core::mem::zeroed();
        si.cb = core::mem::size_of::<STARTUPINFOA>() as DWORD;
        let mut pi: PROCESS_INFORMATION = core::mem::zeroed();
        let ok = CreateProcessA(
            exe.as_ptr() as *const i8,
            core::ptr::null_mut(),
            core::ptr::null_mut(),
            core::ptr::null_mut(),
            0,
            0,
            core::ptr::null_mut(),
            core::ptr::null_mut(),
            &mut si,
            &mut pi,
        );
        if ok != 0 {
            diag(b"TrackIR.exe launched");
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
        } else {
            diag(b"TrackIR.exe launch FAILED");
        }
    }

    let thread = CreateThread(
        core::ptr::null_mut(),
        0,
        Some(notify_thread),
        core::ptr::null_mut(),
        0,
        core::ptr::null_mut(),
    );
    G_THREAD.store(thread as usize, Ordering::Relaxed);
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_StopDataTransmission() -> i32 {
    diag(b"NP_StopDataTransmission");
    G_RUNNING.store(false, Ordering::Relaxed);
    let thread = G_THREAD.load(Ordering::Relaxed) as winapi::um::winnt::HANDLE;
    if !thread.is_null() {
        WaitForSingleObject(thread, 1000);
        CloseHandle(thread);
        G_THREAD.store(0, Ordering::Relaxed);
    }
    0
}

#[no_mangle]
pub unsafe extern "system" fn NP_UnregisterWindowHandle() -> i32 {
    diag(b"NP_UnregisterWindowHandle");
    G_HWND.store(0, Ordering::Relaxed);
    0
}

// ---------------------------------------------------------------------------
// NPPriv_* stubs (ordinals 1-7 in the real DLL).
// ---------------------------------------------------------------------------

#[no_mangle] pub unsafe extern "system" fn NPPriv_ClientNotify() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_GetLastError() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_SetData() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_SetLastError() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_SetParameter() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_SetSignature() -> i32 { 0 }
#[no_mangle] pub unsafe extern "system" fn NPPriv_SetVersion() -> i32 { 0 }
