//! What a capture thread does when the camera stops giving it frames.
//!
//! FINAL_REVIEW S2: on a webcam unplug, V4L2's poll/DQBUF fail at once, and the
//! capture loops retried with no pause, logging a WARN per failure: one core
//! pegged, thousands of lines a second, and a thread that never ended. This is
//! the policy both capture paths (MJPEG tracker thread, YUYV capture thread)
//! now share:
//!
//! - one WARN per run of consecutive errors, DEBUG for the rest of the run
//!   (CLAUDE.md log contract: a repeating condition announces once);
//! - the first error of a run retries at once (a single corrupt MJPEG frame is
//!   not worth a pause), then a backoff of 100 ms doubling to 1 s;
//! - give up after [`GIVE_UP_AFTER`] of nothing but errors, or at once when the
//!   device is gone (`ENODEV`), so the thread ends and the engine sees its
//!   channel close instead of a source that is silently dead.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{SyncSender, TrySendError},
    },
    time::{Duration, Instant},
};

/// Pause before the second retry of a run; doubles from there.
pub(crate) const FIRST_BACKOFF: Duration = Duration::from_millis(100);
/// The backoff never grows past this.
pub(crate) const MAX_BACKOFF: Duration = Duration::from_secs(1);
/// A run of errors this long means the camera is not coming back.
pub(crate) const GIVE_UP_AFTER: Duration = Duration::from_secs(5);

/// `ENODEV` on Linux: the device was unplugged. V4L2 is Linux-only, and this
/// crate does not otherwise depend on `libc`.
const ENODEV: i32 = 19;

/// What to do after a capture error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CaptureStep {
    /// Retry after `wait`. `warn`: this is the first error of the run, so log
    /// it at WARN; later ones go to DEBUG.
    Retry { warn: bool, wait: Duration },
    /// Stop capturing: the device is gone or has failed for [`GIVE_UP_AFTER`].
    GiveUp,
}

/// Consecutive capture errors, reset by the next good frame.
#[derive(Debug, Default)]
pub(crate) struct CaptureErrors {
    /// Errors in the current run.
    run: u32,
    /// When the current run started.
    since: Option<Instant>,
}

impl CaptureErrors {
    /// A frame arrived. Returns how many errors the run it ends held (0 = none).
    pub(crate) fn ok(&mut self) -> u32 {
        std::mem::take(self).run
    }

    /// A capture failed at `now`; `device_gone` is [`is_device_gone`].
    pub(crate) fn error(&mut self, now: Instant, device_gone: bool) -> CaptureStep {
        let since = *self.since.get_or_insert(now);
        self.run = self.run.saturating_add(1);
        if device_gone || now.saturating_duration_since(since) >= GIVE_UP_AFTER {
            return CaptureStep::GiveUp;
        }
        let wait = match self.run {
            1 => Duration::ZERO,
            n => FIRST_BACKOFF
                .saturating_mul(1u32 << (n - 2).min(4))
                .min(MAX_BACKOFF),
        };
        CaptureStep::Retry {
            warn: self.run == 1,
            wait,
        }
    }

    /// Errors in the current run.
    pub(crate) fn run_len(&self) -> u32 {
        self.run
    }

    /// How long the current run has lasted at `now`.
    pub(crate) fn run_age(&self, now: Instant) -> Duration {
        self.since
            .map_or(Duration::ZERO, |s| now.saturating_duration_since(s))
    }
}

/// Whether a capture error says the device itself has gone (unplugged).
pub(crate) fn is_device_gone(e: &anyhow::Error) -> bool {
    e.chain().any(|c| {
        c.downcast_ref::<std::io::Error>()
            .is_some_and(|io| io.raw_os_error() == Some(ENODEV))
    })
}

/// Sleep for `d`, waking early when `shutdown` is set. Returns `true` if it was.
///
/// Slices of at most 50 ms, so a camera switch (which gives the old source
/// 500 ms to let go of the device) never waits out a full backoff.
pub(crate) fn sleep_unless_shutdown(d: Duration, shutdown: &AtomicBool) -> bool {
    let end = Instant::now() + d;
    loop {
        if shutdown.load(Ordering::Acquire) {
            return true;
        }
        let left = end.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return false;
        }
        std::thread::sleep(left.min(Duration::from_millis(50)));
    }
}

/// Hand a pose to the engine. Returns `false` only when the engine has gone
/// (`Disconnected`), which is the thread's cue to end. A full channel means the
/// engine is behind; the pose is dropped and tracking carries on (S2: a `Full`
/// used to end the thread too).
pub(crate) fn send_pose<T>(tx: &SyncSender<T>, pose: T) -> bool {
    match tx.try_send(pose) {
        Ok(()) | Err(TrySendError::Full(_)) => true,
        Err(TrySendError::Disconnected(_)) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    /// Simulate an unplug: capture fails at once every time, and the loop
    /// waits whatever the policy says. One WARN, bounded waits, and an end.
    #[test]
    fn an_unplug_warns_once_backs_off_and_gives_up() {
        let t0 = Instant::now();
        let mut errs = CaptureErrors::default();
        let mut now = t0;
        let (mut warns, mut retries) = (0, 0);
        let mut waits = Vec::new();
        // Ends on `GiveUp`, the only other step.
        while let CaptureStep::Retry { warn, wait } = errs.error(now, false) {
            warns += usize::from(warn);
            retries += 1;
            assert!(wait <= MAX_BACKOFF, "backoff is bounded: {wait:?}");
            waits.push(wait);
            now += wait;
            assert!(retries < 100, "must give up, not retry forever");
        }
        assert_eq!(warns, 1, "one WARN per run of errors");
        assert_eq!(
            &waits[..6],
            &[0, 100, 200, 400, 800, 1000].map(Duration::from_millis),
            "retry at once, then 100 ms doubling to the 1 s cap"
        );
        let gave_up_at = now - t0;
        assert!(
            gave_up_at >= GIVE_UP_AFTER && gave_up_at < GIVE_UP_AFTER + MAX_BACKOFF,
            "gives up about {GIVE_UP_AFTER:?} in, not at {gave_up_at:?}"
        );
        // The old loop: thousands of WARNs a second. Now about ten lines in all.
        assert!(retries <= 10, "{retries} retries");
    }

    #[test]
    fn a_vanished_device_gives_up_at_once() {
        let mut errs = CaptureErrors::default();
        assert_eq!(errs.error(Instant::now(), true), CaptureStep::GiveUp);
    }

    /// A good frame ends the run; the next failure is a new run with its own WARN.
    #[test]
    fn a_good_frame_ends_the_run() {
        let t0 = Instant::now();
        let mut errs = CaptureErrors::default();
        assert_eq!(
            errs.error(t0, false),
            CaptureStep::Retry {
                warn: true,
                wait: Duration::ZERO
            }
        );
        assert!(matches!(
            errs.error(t0, false),
            CaptureStep::Retry { warn: false, .. }
        ));
        assert_eq!(errs.run_len(), 2);
        assert_eq!(errs.ok(), 2);
        assert_eq!(errs.ok(), 0);
        // Much later: the old run's start must not count against the new one.
        let later = t0 + Duration::from_secs(60);
        assert_eq!(
            errs.error(later, false),
            CaptureStep::Retry {
                warn: true,
                wait: Duration::ZERO
            }
        );
        assert_eq!(errs.run_age(later), Duration::ZERO);
    }

    #[test]
    fn enodev_anywhere_in_the_chain_is_a_vanished_device() {
        let gone = anyhow::Error::from(std::io::Error::from_raw_os_error(ENODEV))
            .context("capturing frame");
        assert!(is_device_gone(&gone));
        let busy =
            anyhow::Error::from(std::io::Error::from_raw_os_error(16)).context("capturing frame");
        assert!(!is_device_gone(&busy));
        assert!(!is_device_gone(&anyhow::anyhow!("decoding MJPEG frame")));
    }

    /// S2: only a closed channel ends the thread; a full one drops the pose.
    #[test]
    fn a_full_channel_drops_the_pose_and_a_closed_one_ends_the_thread() {
        let (tx, rx) = mpsc::sync_channel::<u32>(1);
        assert!(send_pose(&tx, 1));
        assert!(send_pose(&tx, 2), "full: keep going");
        assert_eq!(rx.try_recv(), Ok(1));
        assert!(
            rx.try_recv().is_err(),
            "the pose sent while full was dropped"
        );
        drop(rx);
        assert!(!send_pose(&tx, 3), "disconnected: stop");
    }

    #[test]
    fn the_backoff_sleep_wakes_on_shutdown() {
        let flag = AtomicBool::new(true);
        let t = Instant::now();
        assert!(sleep_unless_shutdown(Duration::from_secs(5), &flag));
        assert!(t.elapsed() < Duration::from_millis(100));
        let flag = AtomicBool::new(false);
        assert!(!sleep_unless_shutdown(Duration::from_millis(10), &flag));
    }
}
