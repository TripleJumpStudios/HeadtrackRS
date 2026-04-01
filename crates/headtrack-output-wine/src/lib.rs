//! Wine shared memory bridge for Star Citizen (NPClient64.dll via POSIX SHM).
//!
//! # Architecture
//!
//! Creates `/dev/shm/headtrack-pose` via POSIX `shm_open` and writes processed
//! head pose to it each frame.  The `NPClient64.dll` running inside Star
//! Citizen's Wine instance reads it via Wine's `Z:\dev\shm` path.
//!
//! # Axis sign conventions
//!
//! The SHM layout matches FT_SharedMem / opentrack's coordinate system:
//! - yaw:   degrees, right = positive   ← matches headtrack-rs canonical ✓
//! - pitch: degrees, up = positive      ← matches headtrack-rs canonical ✓
//! - roll:  degrees, right = positive   ← matches headtrack-rs canonical ✓
//! - y:     mm, up = positive           ← matches headtrack-rs canonical ✓
//! - z:     mm, toward screen = positive← matches headtrack-rs canonical ✓
//! - x:     mm, right = positive        ← SIGN_X = -1.0 (same empirical flip
//!          as X-Plane; headtrack-rs canonical X is mirrored vs world-space.
//!          Change SIGN_X to 1.0 if lateral movement is inverted in-game.)

use headtrack_core::Pose;
use libc::{
    MAP_FAILED, MAP_SHARED, O_CREAT, O_RDWR, PROT_READ, PROT_WRITE,
    S_IRUSR, S_IWUSR, c_int,
};
use std::ptr;
use tracing::{info, warn};

/// Name of the POSIX shared memory segment.
///
/// Accessible on the Linux filesystem at `/dev/shm/headtrack-pose`.
/// The Wine DLL reads it via `Z:\dev\shm\headtrack-pose`.
/// `/dev/shm` is used (not `$XDG_RUNTIME_DIR`) because it is reliably
/// bind-mounted inside Proton's pressure-vessel bubblewrap container.
const SHM_NAME: &[u8] = b"/headtrack-pose\0";

/// X-axis sign correction: headtrack-rs canonical X is mirrored vs world-space.
/// -1.0 matches the X-Plane empirical finding.  Change to 1.0 if lateral
/// movement appears inverted in-game.
const SIGN_X: f64 = -1.0;

/// Layout of the POSIX shared memory written by the daemon.
///
/// Must exactly match the `WineShmPose` struct in `bridge/main.c`.
/// `repr(C)` + `repr(packed)` ensures no padding surprises across languages.
#[repr(C, packed)]
struct WineShmPose {
    /// Monotonic write counter.  Zero means no data yet.  The companion
    /// polls this and only copies to FT_SharedMem when it changes.
    sequence: u64,
    yaw:   f64,  // degrees, right = positive
    pitch: f64,  // degrees, up = positive
    roll:  f64,  // degrees, right ear down = positive
    x:     f64,  // mm (SIGN_X applied)
    y:     f64,  // mm, up = positive
    z:     f64,  // mm, toward screen = positive
}

const SHM_SIZE: usize = std::mem::size_of::<WineShmPose>();

/// Writes processed pose into the POSIX shared memory that the Wine companion reads.
pub struct WineShmWriter {
    ptr: *mut WineShmPose,
    fd: c_int,
    sequence: u64,
}

// SAFETY: the mmap'd pointer is only accessed from one thread (the run_loop).
unsafe impl Send for WineShmWriter {}

impl WineShmWriter {
    /// Create (or re-create) the shared memory segment and map it.
    pub fn open() -> anyhow::Result<Self> {
        unsafe {
            let fd = libc::shm_open(
                SHM_NAME.as_ptr() as *const libc::c_char,
                O_CREAT | O_RDWR,
                (S_IRUSR | S_IWUSR) as libc::mode_t,
            );
            if fd < 0 {
                return Err(anyhow::anyhow!(
                    "shm_open failed: {}",
                    std::io::Error::last_os_error()
                ));
            }

            if libc::ftruncate(fd, SHM_SIZE as libc::off_t) < 0 {
                libc::close(fd);
                return Err(anyhow::anyhow!(
                    "ftruncate failed: {}",
                    std::io::Error::last_os_error()
                ));
            }

            let ptr = libc::mmap(
                ptr::null_mut(),
                SHM_SIZE,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                0,
            );

            if ptr == MAP_FAILED {
                libc::close(fd);
                return Err(anyhow::anyhow!(
                    "mmap failed: {}",
                    std::io::Error::last_os_error()
                ));
            }

            // Zero out — sequence=0 signals "no data yet" to companion.
            ptr::write_bytes(ptr, 0, SHM_SIZE);

            info!(
                "wine shm ready: /dev/shm/headtrack-pose ({SHM_SIZE} bytes)"
            );

            Ok(Self {
                ptr: ptr as *mut WineShmPose,
                fd,
                sequence: 0,
            })
        }
    }

    /// Write a processed pose into shared memory.
    ///
    /// Uses a monotonic sequence counter so the companion can detect new frames
    /// without comparing floating-point values.
    #[inline]
    pub fn write(&mut self, pose: &Pose) {
        self.sequence = self.sequence.wrapping_add(1);
        let p = WineShmPose {
            sequence: self.sequence,
            yaw:   pose.yaw   as f64,
            pitch: pose.pitch as f64,
            roll:  pose.roll  as f64,
            x:     SIGN_X * pose.x as f64,
            y:     pose.y as f64,
            z:     pose.z as f64,
        };
        // SAFETY: ptr is valid for the lifetime of WineShmWriter, and we are
        // the sole writer.  The companion is a read-only consumer.
        unsafe { ptr::write_volatile(self.ptr, p) };
    }
}

impl Drop for WineShmWriter {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, SHM_SIZE);
            libc::close(self.fd);
            libc::shm_unlink(SHM_NAME.as_ptr() as *const libc::c_char);
        }
        warn!("wine shm closed and unlinked");
    }
}

