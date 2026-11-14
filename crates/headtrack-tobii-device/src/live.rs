//! App-owned acquisition worker with capacity-one snapshots and bounded retries.
use crate::{
    image::GrayImage,
    usb::{self, Event, Session, Stats, UsbErrorKind},
    Gaze, Measurement, RateMeter, Stamp, Validity,
};
use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Opening,
    Streaming,
    Recovering,
    Unsupported,
    Closed,
}
#[derive(Clone)]
pub struct Snapshot {
    pub state: State,
    pub message: String,
    pub session: u64,
    pub gaze: Option<Measurement<Gaze>>,
    /// Raw sensor coordinates; units/signs and suitability for head translation are unvalidated.
    pub eye_origins: Option<Measurement<[Option<[f64; 3]>; 2]>>,
    pub image: Option<Measurement<GrayImage>>,
    pub stats: Stats,
    pub stream_started_us: u64,
    /// Typed cause of the last failed open/stream (`Recovering`/`Unsupported`);
    /// `None` while opening or streaming. Survives the per-retry reset to
    /// `Opening`, so a status display does not flicker between attempts.
    pub error: Option<UsbErrorKind>,
    /// Serial of the device this worker has opened (kept across reopen cycles).
    pub serial: Option<String>,
    /// IR images per second, measured over the last ~1 s window (0 when not streaming).
    pub image_fps: f32,
}
impl Default for Snapshot {
    fn default() -> Self {
        Self {
            state: State::Opening,
            message: "Opening Tobii USB session…".into(),
            session: 0,
            gaze: None,
            eye_origins: None,
            image: None,
            stats: Stats::default(),
            stream_started_us: 0,
            error: None,
            serial: None,
            image_fps: 0.0,
        }
    }
}
/// Once-per-second DEBUG diagnostic of the gaze counters (7a triage): are gaze samples
/// invalid (`valid_gaze` stays flat while `gaze` grows) or dropped (`malformed` grows)?
/// Enable with `RUST_LOG=warn,headtrack_tobii_device::live=debug`.
struct CounterLog {
    last_us: u64,
    last: Stats,
}
impl CounterLog {
    fn new(now_us: u64) -> Self {
        Self { last_us: now_us, last: Stats::default() }
    }
    /// Returns per-interval deltas `(gaze, valid_gaze, malformed, images)` once at least one
    /// second has passed since the previous report, else `None`.
    fn due(&mut self, now_us: u64, stats: &Stats) -> Option<[u64; 4]> {
        if now_us.saturating_sub(self.last_us) < 1_000_000 {
            return None;
        }
        let d = [
            stats.gaze.saturating_sub(self.last.gaze),
            stats.valid_gaze.saturating_sub(self.last.valid_gaze),
            stats.malformed.saturating_sub(self.last.malformed),
            stats.images.saturating_sub(self.last.images),
        ];
        self.last_us = now_us;
        self.last = stats.clone();
        Some(d)
    }
}
pub struct Service {
    shared: Arc<Mutex<Snapshot>>,
    cancel: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}
impl Service {
    pub fn start(serial: Option<String>, generations: Arc<AtomicU64>, clock: Instant) -> Self {
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let out = shared.clone();
        let cancel = Arc::new(AtomicBool::new(false));
        let stop = cancel.clone();
        let worker = thread::spawn(move || {
            let mut selected = serial;
            let mut state = Snapshot::default();
            let mut last_error: Option<UsbErrorKind> = None;
            let publish = |state: &Snapshot| {
                *out.lock().unwrap_or_else(|e| e.into_inner()) = state.clone();
            };
            while !stop.load(Ordering::Acquire) {
                state = Snapshot {
                    session: generations.fetch_add(1, Ordering::AcqRel) + 1,
                    error: last_error,
                    serial: selected.clone(),
                    ..Default::default()
                };
                let mut image_rate = RateMeter::default();
                let mut counter_log = CounterLog::new(clock.elapsed().as_micros() as u64);
                publish(&state);
                let result = (|| -> Result<(), usb::UsbError> {
                    let mut device = Session::open(selected.as_deref(), &stop)?;
                    if selected.is_none() {
                        selected = device.serial.clone();
                    }
                    state.serial = device.serial.clone();
                    last_error = None;
                    state.error = None;
                    state.state = State::Streaming;
                    state.message = "Live USB · realm 0 / protocol 1.8 · firmware unknown".into();
                    state.stream_started_us = clock.elapsed().as_micros() as u64;
                    publish(&state);
                    while !stop.load(Ordering::Acquire) {
                        for event in device.poll(&stop)? {
                            let received_us = clock.elapsed().as_micros() as u64;
                            match event {
                                Event::Gaze(g) => {
                                    state.eye_origins = Some(Measurement {
                                        stamp: Stamp {
                                            session: state.session,
                                            sequence: g.counter as u64,
                                            device_us: g.device_us,
                                            received_us,
                                        },
                                        validity: if g.validity == Validity::Valid
                                            && g.eye_origins.iter().all(Option::is_some)
                                        {
                                            Validity::Valid
                                        } else {
                                            Validity::Invalid
                                        },
                                        value: Some(g.eye_origins),
                                    });
                                    state.gaze = Some(Measurement {
                                        stamp: Stamp {
                                            session: state.session,
                                            sequence: g.counter as u64,
                                            device_us: g.device_us,
                                            received_us,
                                        },
                                        validity: g.validity,
                                        value: Some(g.gaze),
                                    })
                                }
                                Event::Image { device_us, image } => {
                                    state.image = Some(Measurement {
                                        stamp: Stamp {
                                            session: state.session,
                                            sequence: device.stats.images,
                                            device_us,
                                            received_us,
                                        },
                                        validity: Validity::Valid,
                                        value: Some(image),
                                    })
                                }
                            }
                        }
                        state.stats = device.stats.clone();
                        if let Some([g, v, m, i]) =
                            counter_log.due(clock.elapsed().as_micros() as u64, &device.stats)
                        {
                            tracing::debug!(
                                "tobii counters: per-second deltas gaze={g} valid_gaze={v} malformed={m} images={i}; totals gaze={} valid_gaze={} malformed={} images={}",
                                device.stats.gaze, device.stats.valid_gaze, device.stats.malformed, device.stats.images
                            );
                        }
                        state.image_fps = image_rate.update(
                            device.stats.images,
                            clock.elapsed().as_micros() as u64,
                        );
                        publish(&state);
                    }
                    Ok(())
                })();
                state.gaze = None;
                state.eye_origins = None;
                state.image = None;
                state.image_fps = 0.0;
                if stop.load(Ordering::Acquire) {
                    break;
                }
                match result {
                    Err(e) => {
                        state.state = if e.retryable() {
                            State::Recovering
                        } else {
                            State::Unsupported
                        };
                        state.message = e.to_string();
                        last_error = Some(e.kind());
                        state.error = last_error;
                        publish(&state);
                        if !e.retryable() {
                            while !stop.load(Ordering::Acquire) {
                                thread::sleep(Duration::from_millis(10));
                            }
                            break;
                        }
                    }
                    Ok(()) => break,
                }
                let deadline = Instant::now() + Duration::from_millis(750);
                while Instant::now() < deadline && !stop.load(Ordering::Acquire) {
                    thread::sleep(Duration::from_millis(10));
                }
            }
            state.state = State::Closed;
            state.message = "USB session closed and interface released".into();
            state.gaze = None;
            state.eye_origins = None;
            state.image = None;
            state.image_fps = 0.0;
            publish(&state);
        });
        Self {
            shared,
            cancel,
            thread: Some(worker),
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        self.shared
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}
impl Drop for Service {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
#[cfg(test)]
mod counter_log_tests {
    use super::*;
    #[test]
    fn reports_deltas_once_per_second() {
        let mut log = CounterLog::new(0);
        let mut s = Stats { gaze: 100, valid_gaze: 0, malformed: 3, images: 20, ..Default::default() };
        assert_eq!(log.due(999_999, &s), None);
        assert_eq!(log.due(1_000_000, &s), Some([100, 0, 3, 20]));
        assert_eq!(log.due(1_500_000, &s), None);
        s.gaze = 160;
        s.valid_gaze = 60;
        assert_eq!(log.due(2_000_000, &s), Some([60, 60, 0, 0]));
    }
}
