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
//!   as X-Plane; headtrack-rs canonical X is mirrored vs world-space.
//!   Change SIGN_X to 1.0 if lateral movement is inverted in-game.)

use headtrack_core::Pose;
use libc::{
    MAP_FAILED, MAP_SHARED, O_CREAT, O_RDWR, PROT_READ, PROT_WRITE,
    S_IRUSR, S_IWUSR, c_int,
};
use std::ffi::CStr;
use std::ptr;
use tracing::{debug, info};

/// Name of the POSIX shared memory segment.
///
/// Accessible on the Linux filesystem at `/dev/shm/headtrack-pose`.
/// The Wine DLL reads it via `Z:\dev\shm\headtrack-pose`.
/// `/dev/shm` is used (not `$XDG_RUNTIME_DIR`) because it is reliably
/// bind-mounted inside Proton's pressure-vessel bubblewrap container.
const SHM_NAME: &CStr = c"/headtrack-pose";

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
/// Default output smoothing speed, matching X-Plane's `LERP_SPEED`.
///
/// Star Citizen was the only target with no output smoothing at all: X-Plane
/// interpolates on the sim's own frame clock and MSFS gained an adjustable
/// low-pass, but the SHM path handed the game raw pipeline output. The pipeline
/// is shared, so the difference was purely structural — SC simply had one fewer
/// stage between the tracker and the view, and read rougher for it.
pub const DEFAULT_SMOOTHING: f32 = 15.0;

pub struct WineShmWriter {
    ptr: *mut WineShmPose,
    fd: c_int,
    sequence: u64,
    /// Exponential smoothing speed. Higher = snappier, lower = smoother.
    smoothing: f32,
    /// Last emitted pose, the accumulator the smoother converges toward the
    /// target from. `None` until the first write, which is emitted as-is so a
    /// fresh start (or a recenter) does not glide in from zero.
    last: Option<(f64, f64, f64, f64, f64, f64)>,
    /// Wall-clock reference for the smoothing `dt`. Unlike MSFS there is no
    /// frame event to key off — the game pulls from SHM on its own schedule —
    /// so the interval between our own writes is the only clock available.
    last_write: Option<std::time::Instant>,
}

// SAFETY: the mmap'd pointer is only accessed from one thread (the run_loop).
unsafe impl Send for WineShmWriter {}

impl WineShmWriter {
    /// Create (or re-create) the shared memory segment and map it.
    pub fn open() -> anyhow::Result<Self> {
        Self::open_named(SHM_NAME)
    }

    /// [`open`](Self::open) on a segment of the caller's choosing.
    ///
    /// Private on purpose: production always uses `SHM_NAME`, the name the DLL
    /// reads. The tests use it to get a segment of their own, so `cargo test`
    /// never writes the live `/dev/shm/headtrack-pose` a running game may have
    /// mapped, and parallel tests cannot race each other (FINAL_REVIEW S6).
    fn open_named(name: &CStr) -> anyhow::Result<Self> {
        unsafe {
            let fd = libc::shm_open(
                name.as_ptr(),
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

            // Do NOT zero the segment.
            //
            // A newly created one is already zero-filled by the kernel. A
            // pre-existing one means a previous run left it behind on purpose
            // (see `Drop`), and a game may still be mapped to it — zeroing sets
            // `sequence` back to 0, which the DLL reads as "no data yet", so
            // tracking would blink out at the exact moment we reattach.
            //
            // Instead continue the previous sequence. The DLL only tests
            // `sequence != 0`, so the value need not be globally monotonic; this
            // simply avoids ever passing back through 0.
            let existing = ptr::read_volatile(ptr as *const WineShmPose);
            let sequence = existing.sequence;

            info!(
                "wine shm ready: /dev/shm{} ({SHM_SIZE} bytes)",
                name.to_string_lossy()
            );

            Ok(Self {
                ptr: ptr as *mut WineShmPose,
                fd,
                sequence,
                smoothing: DEFAULT_SMOOTHING,
                last: None,
                last_write: None,
            })
        }
    }

    /// Set the output smoothing speed. Higher = snappier, lower = smoother.
    ///
    /// `0.0` disables smoothing entirely and restores the pre-0.2.0 behaviour of
    /// writing raw pipeline output.
    pub fn set_smoothing(&mut self, speed: f32) {
        self.smoothing = speed.max(0.0);
        debug!("wine shm output smoothing = {:.1}", self.smoothing);
    }

    /// Exponential smoothing toward `target`, framed so that a long `dt` cannot
    /// overshoot: `alpha = 1 - e^(-speed·dt)` approaches 1 but never exceeds it.
    /// A naive `speed * dt` reaches 1.0 at dt = 1/speed and keeps going, which
    /// is what produced the stepping this replaces.
    #[inline]
    fn smooth_step(&mut self, target: (f64, f64, f64, f64, f64, f64)) -> (f64, f64, f64, f64, f64, f64) {
        let now = std::time::Instant::now();
        let dt = self
            .last_write
            .map(|t| now.duration_since(t).as_secs_f32())
            .unwrap_or(0.0)
            // A stall (camera hiccup, scheduler hitch) must not teleport the
            // accumulator; clamp to a sane frame interval.
            .clamp(0.0, 0.1);
        self.last_write = Some(now);

        let Some(mut cur) = self.last else {
            self.last = Some(target);
            return target;
        };
        if self.smoothing <= 0.0 {
            self.last = Some(target);
            return target;
        }

        let alpha = (1.0 - (-self.smoothing * dt).exp()) as f64;
        cur.0 += alpha * (target.0 - cur.0);
        cur.1 += alpha * (target.1 - cur.1);
        cur.2 += alpha * (target.2 - cur.2);
        cur.3 += alpha * (target.3 - cur.3);
        cur.4 += alpha * (target.4 - cur.4);
        cur.5 += alpha * (target.5 - cur.5);
        self.last = Some(cur);
        cur
    }

    /// Drop the smoothing accumulator so the next write is emitted as-is.
    ///
    /// Called on recenter: gliding to a freshly re-zeroed baseline would show the
    /// user their old offset draining away instead of an instant snap to centre.
    pub fn reset_smoothing(&mut self) {
        self.last = None;
        self.last_write = None;
    }

    /// Write a processed pose into shared memory.
    ///
    /// Uses a monotonic sequence counter so the companion can detect new frames
    /// without comparing floating-point values.
    #[inline]
    pub fn write(&mut self, pose: &Pose) {
        self.sequence = self.sequence.wrapping_add(1);
        let (yaw, pitch, roll, x, y, z) = self.smooth_step((
            pose.yaw   as f64,
            pose.pitch as f64,
            pose.roll  as f64,
            SIGN_X * pose.x as f64,
            pose.y as f64,
            pose.z as f64,
        ));
        let p = WineShmPose {
            sequence: self.sequence,
            yaw,
            pitch,
            roll,
            x,
            y,
            z,
        };
        // SAFETY: ptr is valid for the lifetime of WineShmWriter, and we are
        // the sole writer.  The companion is a read-only consumer.
        unsafe { ptr::write_volatile(self.ptr, p) };
    }
}

impl Drop for WineShmWriter {
    /// Unmap and close, but deliberately **do not** `shm_unlink`.
    ///
    /// This looks like a leak and is not. Unlinking removes the *name* while any
    /// existing mapping keeps the inode alive — so a game that already mapped the
    /// segment goes on reading an orphaned inode, and the next engine start
    /// creates a *different* inode under the same name. The game then never sees
    /// another byte for the rest of its session. Observed directly:
    ///
    /// ```text
    /// live segment:  inode 6062
    /// StarCitizen.exe: /dev/shm/headtrack-pose (deleted)   inode 5848
    /// ```
    ///
    /// That is the whole of the long-standing "SC does not reconnect after
    /// restarting HeadtrackRS" bug, and unlinking here was the cause rather than
    /// the cleanup it appears to be. Leaving the name in place lets the next run
    /// reopen the *same* inode, so an already-mapped game picks straight back up.
    ///
    /// The segment is 56 bytes and is reused by the next run, so nothing
    /// accumulates. `/dev/shm` is tmpfs — it clears on reboot.
    fn drop(&mut self) {
        if self.ptr.is_null() {
            return;
        }
        unsafe {
            // Zero the sequence counter before unmapping.
            //
            // The segment outlives us by design (see above), and the DLL's only
            // freshness test is `sequence != 0`. Left as-is, a game started
            // while HeadtrackRS is *not* running would map the segment, read a
            // non-zero sequence, and apply whatever head offset happened to be
            // in it — a frozen, permanently-off-centre view with no tracker
            // running to explain it.
            //
            // Zeroing keeps the inode (so a reattach still works) while
            // correctly telling the DLL there is no live tracker. The next run
            // writes 1, 2, 3… and the game picks up from there.
            ptr::write_volatile(ptr::addr_of_mut!((*self.ptr).sequence), 0);
            libc::munmap(self.ptr as *mut libc::c_void, SHM_SIZE);
            libc::close(self.fd);
        }
        debug!("wine shm unmapped (segment left for reattach, sequence cleared)");
    }
}


#[cfg(test)]
mod smoothing_tests {
    use super::*;

    /// Exercise the smoother without touching /dev/shm.
    fn writer(speed: f32) -> WineShmWriter {
        WineShmWriter {
            ptr: std::ptr::null_mut(),
            fd: -1,
            sequence: 0,
            smoothing: speed,
            last: None,
            last_write: None,
        }
    }

    const T: (f64, f64, f64, f64, f64, f64) = (10.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /// The first sample must pass through untouched — a fresh start or a
    /// recenter should snap, not glide in from zero.
    #[test]
    fn first_write_is_not_smoothed() {
        let mut w = writer(15.0);
        assert_eq!(w.smooth_step(T), T);
    }

    /// Speed 0 restores the pre-0.2.0 behaviour exactly.
    #[test]
    fn zero_speed_is_pass_through() {
        let mut w = writer(0.0);
        w.smooth_step(T);
        let (yaw, ..) = w.smooth_step((42.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assert_eq!(yaw, 42.0);
    }

    /// The point of the exercise: successive writes approach the target rather
    /// than jumping to it.
    #[test]
    fn converges_toward_the_target_without_overshooting() {
        let mut w = writer(15.0);
        w.smooth_step((0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        let mut prev = 0.0_f64;
        for _ in 0..200 {
            std::thread::sleep(std::time::Duration::from_millis(2));
            let (yaw, ..) = w.smooth_step(T);
            assert!(yaw >= prev - 1e-9, "must not move away from target");
            assert!(yaw <= T.0 + 1e-9, "must never overshoot: {yaw}");
            prev = yaw;
        }
        assert!(prev > T.0 * 0.9, "should have substantially converged, got {prev}");
    }

    /// A stall must not teleport the accumulator. Without the dt clamp a long
    /// gap drives alpha to 1.0 and the smoothing silently stops existing at
    /// exactly the moment — a hitch — when it matters most.
    #[test]
    fn a_long_stall_does_not_defeat_the_smoothing() {
        let mut w = writer(15.0);
        w.smooth_step((0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        // simulate a 5-second stall
        w.last_write = Some(std::time::Instant::now() - std::time::Duration::from_secs(5));
        let (yaw, ..) = w.smooth_step(T);
        assert!(yaw < T.0, "a stall must not snap straight to target, got {yaw}");
    }

    /// Recenter drops the accumulator so the next frame snaps to the new zero.
    #[test]
    fn reset_makes_the_next_write_pass_through() {
        let mut w = writer(15.0);
        w.smooth_step((0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        std::thread::sleep(std::time::Duration::from_millis(5));
        w.smooth_step(T);
        w.reset_smoothing();
        assert_eq!(w.smooth_step(T), T, "after reset the target is emitted as-is");
    }

    /// Every axis is smoothed, not just the one the tests poke.
    #[test]
    fn all_six_axes_are_smoothed() {
        let mut w = writer(15.0);
        w.smooth_step((0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        std::thread::sleep(std::time::Duration::from_millis(5));
        let out = w.smooth_step((10.0, 10.0, 10.0, 10.0, 10.0, 10.0));
        for (i, v) in [out.0, out.1, out.2, out.3, out.4, out.5].iter().enumerate() {
            assert!(*v > 0.0 && *v < 10.0, "axis {i} not smoothed: {v}");
        }
    }
}

#[cfg(test)]
mod reattach_tests {
    use super::*;
    use std::ffi::CString;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A segment of this test's own, never the live `/dev/shm/headtrack-pose`
    /// (FINAL_REVIEW S6: the shared name made the tests race each other and
    /// could inject a pose into a running game). Unlinked when dropped.
    struct TestSegment {
        name: CString,
        path: PathBuf,
    }

    impl TestSegment {
        fn new(test: &str) -> Self {
            static N: AtomicU32 = AtomicU32::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            let base = format!("headtrack-pose-test-{}-{n}-{test}", std::process::id());
            Self {
                name: CString::new(format!("/{base}")).expect("no NUL in the name"),
                path: PathBuf::from("/dev/shm").join(base),
            }
        }

        fn open(&self) -> anyhow::Result<WineShmWriter> {
            WineShmWriter::open_named(&self.name)
        }

        fn inode(&self) -> Option<u64> {
            std::fs::metadata(&self.path)
                .ok()
                .map(|m| std::os::unix::fs::MetadataExt::ino(&m))
        }
    }

    impl Drop for TestSegment {
        fn drop(&mut self) {
            // SAFETY: a valid NUL-terminated name; a missing segment is fine.
            unsafe {
                libc::shm_unlink(self.name.as_ptr());
            }
        }
    }

    /// The test segments really are separate from each other and from the
    /// production name, which is what makes the tests safe to run in parallel
    /// while a game is mapped to the live segment.
    #[test]
    fn test_segments_never_use_the_production_name() {
        let a = TestSegment::new("a");
        let b = TestSegment::new("a");
        assert_ne!(a.name, b.name);
        assert_ne!(a.name.as_c_str(), SHM_NAME);
        assert_ne!(a.path, PathBuf::from("/dev/shm/headtrack-pose"));
    }

    /// The regression that matters: a second writer must land on the **same
    /// inode** as the first.
    ///
    /// Until 0.2.0 `Drop` called `shm_unlink`, so the second run created a fresh
    /// inode while an already-mapped game kept reading the orphaned one — and
    /// tracking never came back for the rest of that game session. Nothing
    /// detected it, because from our side everything looked healthy: the segment
    /// existed, writes succeeded, the sequence advanced.
    #[test]
    fn a_restart_reuses_the_same_segment() {
        let seg = TestSegment::new("restart");
        let inode_of = || seg.inode();

        let Ok(mut first) = seg.open() else {
            eprintln!("skipping: /dev/shm unavailable in this environment");
            return;
        };
        first.write(&Pose { yaw: 1.0, pitch: 2.0, roll: 3.0, x: 4.0, y: 5.0, z: 6.0, timestamp_us: 0 });
        let before = inode_of().expect("segment exists while open");
        drop(first);

        // The name must survive the drop — that is the entire fix.
        let after_drop = inode_of();
        assert_eq!(
            after_drop,
            Some(before),
            "Drop must not unlink: an already-mapped game would be stranded on a deleted inode"
        );

        let second = seg.open().expect("reopen");
        let after = inode_of().expect("segment exists");
        assert_eq!(after, before, "a restart must reattach to the same inode, not create a new one");

        drop(second);
    }

    /// A game started while HeadtrackRS is **not** running must see "no tracker",
    /// not a frozen stale pose.
    ///
    /// The segment deliberately outlives the process so a reattach works, and
    /// the DLL's only freshness test is `sequence != 0`. Without clearing it on
    /// shutdown, the leftover segment reads as live data and the game applies
    /// whatever offset was last written — an off-centre view with nothing
    /// running to explain it.
    #[test]
    fn shutdown_leaves_the_segment_marked_as_having_no_tracker() {
        let seg = TestSegment::new("shutdown");
        let Ok(mut w) = seg.open() else {
            eprintln!("skipping: /dev/shm unavailable in this environment");
            return;
        };
        w.write(&Pose { yaw: 30.0, pitch: 0.0, roll: 0.0, x: 0.0, y: 0.0, z: 0.0, timestamp_us: 0 });
        assert!(w.sequence > 0, "precondition: we wrote something");
        drop(w);

        // Read the raw segment the way the DLL would.
        let raw = std::fs::read(&seg.path).expect("segment still exists");
        assert!(raw.len() >= 8, "segment retains its size");
        let seq = u64::from_ne_bytes(raw[..8].try_into().unwrap());
        assert_eq!(seq, 0, "sequence must read as 'no tracker' after a clean shutdown");
    }

    /// The three start orders that matter, as a single statement of intent:
    /// whichever of app and game starts first, and however many times the app
    /// restarts, the game must end up reading live data from the same segment.
    #[test]
    fn reattach_survives_repeated_restarts() {
        let seg = TestSegment::new("repeated");
        let Ok(w) = seg.open() else {
            eprintln!("skipping: /dev/shm unavailable in this environment");
            return;
        };
        let first = seg.inode();
        drop(w);
        for _ in 0..3 {
            let mut w = seg.open().expect("reopen");
            w.write(&Pose::zero(0));
            assert!(w.sequence > 0, "each run publishes live data");
            assert_eq!(seg.inode(), first, "every restart must reuse the same inode");
            drop(w);
        }
    }
}
