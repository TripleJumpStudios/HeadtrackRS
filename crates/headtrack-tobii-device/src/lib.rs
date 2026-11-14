//! Tobii eye-tracker USB device layer for HeadtrackRS.
//!
//! PORTED from TobiiLab `crates/tobii-device` (a local, unpublished prototype)
//! on 2026-10-03 (task P03). Faithful port: modules usb, wire, live, image, framing,
//! replay and the shared types in this file. Wire-protocol authority:
//! TobiiLab `docs/WIRE_SPEC.md`; provenance and implementation boundary:
//! TobiiLab `docs/PROVENANCE.md` (own bounded decoder from local observations; no
//! upstream TTP reader, handshake or initialization capture is included).
//! Deviation: the lab-only `tobii_timing::emit` call in `live.rs` was removed (see
//! docs/P03_TOBII_DEVICE.md).
//!
//! Device-independent contracts and bounded replay input. Live TTP is gated on
//! protocol provenance; the lab framing module is NOT a Tobii wire decoder.
pub mod framing;
pub mod image;
pub mod replay;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Validity {
    Valid,
    Invalid,
    Missing,
    Stale,
    WrongSession,
    ClockRegression,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceKind {
    Live,
    Replay,
    Synthetic,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Stamp {
    pub session: u64,
    pub sequence: u64,
    pub device_us: u64,
    /// Monotonic time since this runtime started. Never a wall-clock epoch.
    pub received_us: u64,
}
#[derive(Clone, Debug)]
pub struct Measurement<T> {
    pub stamp: Stamp,
    pub validity: Validity,
    pub value: Option<T>,
}
impl<T> Measurement<T> {
    pub fn status(&self, session: u64, now_us: u64, ttl: Duration) -> Validity {
        if self.stamp.session != session {
            return Validity::WrongSession;
        }
        let Some(age) = now_us.checked_sub(self.stamp.received_us) else {
            return Validity::ClockRegression;
        };
        if age as u128 > ttl.as_micros() {
            return Validity::Stale;
        }
        if self.value.is_none() {
            return Validity::Missing;
        }
        self.validity
    }
    pub fn fresh(&self, session: u64, now_us: u64, ttl: Duration) -> Option<&T> {
        (self.status(session, now_us, ttl) == Validity::Valid)
            .then_some(self.value.as_ref())
            .flatten()
    }
}
#[derive(Clone, Debug)]
pub struct Gaze {
    pub normalized: [f64; 2],
    pub eyes_present: [Option<bool>; 2],
}
/// Checks both ordering domains. Invalid records must also advance this guard:
/// a later invalid sample cannot be replaced by an older valid one.
#[derive(Default)]
pub struct SequenceGuard {
    last: Option<Stamp>,
}
impl SequenceGuard {
    pub fn accept(&mut self, stamp: Stamp) -> bool {
        if let Some(last) = self.last {
            if stamp.session < last.session
                || (stamp.session == last.session
                    && (stamp.sequence <= last.sequence
                        || stamp.device_us <= last.device_us
                        || stamp.received_us < last.received_us))
            {
                return false;
            }
        }
        self.last = Some(stamp);
        true
    }
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// Events-per-second over a sliding ~1 s window, from a monotonically
/// increasing counter. A rate, not a cumulative count: it falls to 0 within a
/// window when the counter stops advancing.
#[derive(Clone, Debug, Default)]
pub struct RateMeter {
    window_start: Option<(u64, u64)>, // (time_us, count)
    rate: f32,
}
impl RateMeter {
    pub const WINDOW_US: u64 = 1_000_000;
    /// Feed the current counter value at `now_us`; returns the latest rate.
    pub fn update(&mut self, count: u64, now_us: u64) -> f32 {
        match self.window_start {
            None => self.window_start = Some((now_us, count)),
            Some((t0, c0)) => {
                if count < c0 || now_us < t0 {
                    // Counter or clock reset: start a new window.
                    self.window_start = Some((now_us, count));
                    self.rate = 0.0;
                } else if now_us - t0 >= Self::WINDOW_US {
                    self.rate = (count - c0) as f32 * 1e6 / (now_us - t0) as f32;
                    self.window_start = Some((now_us, count));
                }
            }
        }
        self.rate
    }
    pub fn rate(&self) -> f32 {
        self.rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rate_meter_reports_a_rate_not_a_total() {
        let mut m = RateMeter::default();
        assert_eq!(m.update(0, 0), 0.0);
        // 33 events in the first second.
        assert_eq!(m.update(20, 600_000), 0.0); // window not complete yet
        assert!((m.update(33, 1_000_000) - 33.0).abs() < 1e-3);
        // Next second: 30 more → 30/s, not 63.
        assert!((m.update(63, 2_000_000) - 30.0).abs() < 1e-3);
        // Counter stalls: the next full window reads 0.
        assert_eq!(m.update(63, 3_000_000), 0.0);
        // Counter reset (new session) restarts the window.
        assert_eq!(m.update(5, 3_100_000), 0.0);
        assert!((m.update(35, 4_100_000) - 30.0).abs() < 1e-3);
    }
    #[test]
    fn refresh_does_not_renew_acquisition() {
        let s = Measurement {
            stamp: Stamp {
                session: 1,
                sequence: 1,
                device_us: 10,
                received_us: 10,
            },
            validity: Validity::Valid,
            value: Some(42),
        };
        assert_eq!(s.fresh(1, 20, Duration::from_micros(100)), Some(&42));
        for now in 111..200 {
            assert_eq!(
                s.status(1, now, Duration::from_micros(100)),
                Validity::Stale
            );
        }
        assert_eq!(
            s.status(2, 20, Duration::from_micros(100)),
            Validity::WrongSession
        );
        assert_eq!(
            s.status(1, 0, Duration::from_micros(100)),
            Validity::ClockRegression
        );
    }
    #[test]
    fn order_guard_rejects_old_callbacks_and_clock_reset() {
        let mut g = SequenceGuard::default();
        let a = Stamp {
            session: 1,
            sequence: 1,
            device_us: 5,
            received_us: 5,
        };
        assert!(g.accept(a));
        assert!(!g.accept(a));
        assert!(!g.accept(Stamp {
            sequence: 2,
            device_us: 4,
            ..a
        }));
        assert!(g.accept(Stamp {
            session: 2,
            device_us: 1,
            ..a
        }));
        assert!(!g.accept(a));
    }
}
pub mod live;
pub mod usb;
pub mod wire;
