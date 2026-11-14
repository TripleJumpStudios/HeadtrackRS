//! `TobiiInput` — the Tobii Eye Tracker 5 as a HeadtrackRS [`InputSource`] (task P06).
//!
//! Two threads, both owned by `TobiiInput` and joined on drop:
//! - the USB worker (`headtrack_tobii_device::live::Service`), which owns the device;
//! - the head worker (this module), which owns the [`Estimator`], evaluates each new
//!   fresh IR image once, maintains the neutral, and publishes the latest
//!   neutral-relative rotation plus a [`TobiiStatus`] for the GUI.
//!
//! Output is the **raw** neutral-relative rotation: no Accela or any other filter
//! here — the engine pipeline filters every source the same way. Translation is
//! held at 0 (`SourceKind::Tobii.provides_translation() == false`). Gaze goes to
//! `TobiiStatus` only and never into a `RawPose`.
//!
//! Freshness: a rotation older than [`POSE_TTL_US`] is never emitted, and no
//! sample is ever emitted twice (`PoseGate`). The neutral is taken automatically
//! from the first stable acquisition of each USB session and again after every
//! `recenter()`; while it is being captured nothing is emitted.
use crate::{neutral, Estimator, ModelError};
use headtrack_core::{
    input::{GazePoint, TobiiState, TobiiStatus},
    InputSource, RawPose,
};
use headtrack_tobii_device::{
    live::{self, Snapshot},
    usb::UsbErrorKind,
    RateMeter, Stamp,
};
use std::{
    path::Path,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

/// A rotation (or IR image) older than this is never used or emitted.
pub const POSE_TTL_US: u64 = 250_000;
const POSE_TTL: Duration = Duration::from_micros(POSE_TTL_US);
/// Minimum interval between status publications to the GUI sink.
const STATUS_INTERVAL_US: u64 = 50_000;
/// Display name (also `SourceKind::Tobii.label()`).
pub const NAME: &str = "Tobii Eye Tracker 5";

/// One neutral-relative head rotation, canonical axes: yaw right +, pitch up +,
/// roll right-ear-down +. Degrees.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PoseSample {
    pub stamp: Stamp,
    pub rotation: [f32; 3],
}

// ── Neutral + latest rotation (pure, testable) ──────────────────────────────

/// Turns per-frame image-axis angles into neutral-relative head rotation.
pub(crate) struct HeadTracker {
    session: u64,
    collector: neutral::Collector,
    collecting: bool,
    neutral: Option<[f32; 3]>,
    /// Frames received at or before this instant are not neutral candidates
    /// (they predate the Recenter press).
    recenter_after_us: u64,
    latest: Option<PoseSample>,
    /// `None` = no frame evaluated this session; `Some(false)` = last frame had no face.
    face: Option<bool>,
    reason: String,
    poses: u64,
    rate: RateMeter,
}

impl HeadTracker {
    pub fn new() -> Self {
        Self {
            session: 0,
            collector: neutral::Collector::default(),
            collecting: true,
            neutral: None,
            recenter_after_us: 0,
            latest: None,
            face: None,
            reason: String::new(),
            poses: 0,
            rate: RateMeter::default(),
        }
    }

    /// A new USB session (first open, or a reopen after unplug/error) starts
    /// from scratch: the tracker may have moved, so the neutral is re-taken.
    pub fn begin_session(&mut self, session: u64) {
        if session != self.session {
            self.session = session;
            self.collector.clear();
            self.collecting = true;
            self.neutral = None;
            self.latest = None;
            self.face = None;
            self.reason.clear();
            self.poses = 0;
            self.rate = RateMeter::default();
        }
    }

    /// One evaluated IR frame. `angles` is the estimator's image-axis rotation,
    /// `None` when it found no usable face (`reason` says why).
    pub fn observe(&mut self, stamp: Stamp, angles: Option<[f32; 3]>, reason: &str, now_us: u64) {
        self.begin_session(stamp.session);
        self.reason.clear();
        self.reason.push_str(reason);
        let Some(a) = angles else {
            self.collector.clear();
            self.face = Some(false);
            return;
        };
        self.face = Some(true);
        if self.collecting && stamp.received_us > self.recenter_after_us {
            if let Some(n) = self.collector.observe(stamp, a, now_us) {
                self.neutral = Some(n);
                self.collecting = false;
            }
        }
        if !self.collecting {
            if let Some(r) = self.neutral.and_then(|n| neutral::head_rotation(n, a)) {
                self.latest = Some(PoseSample { stamp, rotation: r });
                self.poses += 1;
            }
        }
    }

    /// Restart neutral capture. Nothing is emitted until the new neutral is taken.
    pub fn recenter(&mut self, pressed_at_us: u64) {
        self.collector.clear();
        self.collecting = true;
        self.recenter_after_us = pressed_at_us;
        self.latest = None;
    }

    pub fn latest(&self) -> Option<PoseSample> {
        self.latest
    }
    pub fn collecting(&self) -> bool {
        self.collecting
    }
    /// `Some(fraction of the still window held)` while the neutral is being
    /// taken, `None` once it is.
    pub fn neutral_progress(&self) -> Option<f32> {
        self.collecting.then(|| self.collector.progress())
    }
    pub fn face(&self) -> Option<bool> {
        self.face
    }
    pub fn reason(&self) -> &str {
        &self.reason
    }
    /// Head-pose rate over the last ~1 s.
    pub fn pose_rate(&mut self, now_us: u64) -> f32 {
        self.rate.update(self.poses, now_us)
    }
}

// ── Emission gate (pure, testable) ──────────────────────────────────────────

/// Decides whether the latest sample may be handed to the engine: only if it is
/// newer than anything already emitted and younger than the TTL.
#[derive(Default)]
pub(crate) struct PoseGate {
    last: Option<(u64, u64)>,
}

impl PoseGate {
    pub fn take(&mut self, sample: Option<PoseSample>, now_us: u64) -> Option<RawPose> {
        let s = sample?;
        let key = (s.stamp.session, s.stamp.sequence);
        if self.last.is_some_and(|last| key <= last) {
            return None; // already emitted, or older than what was
        }
        let age = now_us.checked_sub(s.stamp.received_us)?; // clock regression: refuse
        if age > POSE_TTL_US {
            return None; // stale: never emitted, and a newer sample is still welcome
        }
        self.last = Some(key);
        let [yaw, pitch, roll] = s.rotation;
        Some(RawPose {
            yaw,
            pitch,
            roll,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp_us: s.stamp.device_us,
        })
    }
}

// ── Status mapping (pure, testable) — P05 §"Contract gaps" table ────────────

fn state_from_error(kind: UsbErrorKind) -> TobiiState {
    match kind {
        UsbErrorKind::NoDevice => TobiiState::NoDevice,
        UsbErrorKind::PermissionDenied => TobiiState::PermissionDenied,
        UsbErrorKind::Busy => TobiiState::Busy,
        UsbErrorKind::Disconnected | UsbErrorKind::Other => TobiiState::Lost,
    }
}

pub(crate) fn map_status(dev: &Snapshot, tracker: &mut HeadTracker, now_us: u64) -> TobiiStatus {
    let state = match dev.state {
        // A retry after a known failure keeps showing that failure: the worker
        // re-enters Opening every ~750 ms while e.g. the device is absent.
        live::State::Opening => dev.error.map_or(TobiiState::Opening, state_from_error),
        live::State::Recovering => state_from_error(dev.error.unwrap_or(UsbErrorKind::Other)),
        // Non-retryable (unsupported firmware/protocol, two trackers, closed):
        // nothing usable is attached. The message carries the reason.
        live::State::Unsupported | live::State::Closed => TobiiState::NoDevice,
        live::State::Streaming => match tracker.face() {
            None => TobiiState::Opening, // stream up, no frame evaluated yet
            Some(false) => TobiiState::NoFace,
            Some(true) => TobiiState::Streaming,
        },
    };
    let message = match state {
        TobiiState::Streaming if tracker.collecting() => {
            "Hold still, facing the screen: capturing neutral (2 s)".to_string()
        }
        TobiiState::Streaming => "Tracking head rotation".to_string(),
        TobiiState::NoFace => tracker.reason().to_string(),
        _ => dev.message.clone(),
    };
    let streaming = matches!(state, TobiiState::Streaming | TobiiState::NoFace);
    TobiiStatus {
        state,
        fps: if streaming { tracker.pose_rate(now_us) } else { 0.0 },
        last_sample_age_ms: tracker
            .latest()
            .and_then(|p| now_us.checked_sub(p.stamp.received_us))
            .map(|us| (us / 1000).min(u32::MAX as u64) as u32),
        serial: dev.serial.clone(),
        gaze: dev
            .gaze
            .as_ref()
            .and_then(|g| g.fresh(dev.session, now_us, POSE_TTL))
            .map(|g| GazePoint {
                x: g.normalized[0] as f32,
                y: g.normalized[1] as f32,
            }),
        message,
        // Only while frames are arriving: before that the GUI shows "Starting".
        neutral_progress: if streaming { tracker.neutral_progress() } else { None },
    }
}

// ── The InputSource ─────────────────────────────────────────────────────────

#[derive(Default)]
struct Shared {
    latest: Option<PoseSample>,
    recenter_requests: u64,
    recenter_at_us: u64,
}

/// The Tobii as a head source. Dropping it stops both threads and releases the
/// USB interface before `drop` returns, so a new `TobiiInput` (or TobiiLab) can
/// open the device immediately afterwards.
pub struct TobiiInput {
    clock: Instant,
    shared: Arc<Mutex<Shared>>,
    gate: PoseGate,
    quit: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl TobiiInput {
    /// Load and verify the models in `models_root` (normally `assets/models/tobii`),
    /// then start the USB and head workers. `serial: None` = the single connected
    /// tracker. `status` receives a [`TobiiStatus`] about every 50 ms.
    ///
    /// Fails only on the models; a missing or busy device is a *status*, not an
    /// error, and the USB worker keeps retrying.
    pub fn open(
        models_root: &Path,
        serial: Option<String>,
        status: Arc<Mutex<TobiiStatus>>,
    ) -> Result<Self, ModelError> {
        let estimator = Estimator::load(models_root)?;
        let clock = Instant::now();
        let shared = Arc::new(Mutex::new(Shared::default()));
        let quit = Arc::new(AtomicBool::new(false));
        let worker = {
            let shared = shared.clone();
            let quit = quit.clone();
            thread::Builder::new()
                .name("tobii-head".into())
                .spawn(move || head_worker(estimator, serial, clock, shared, status, quit))
                .expect("spawn tobii-head thread")
        };
        Ok(Self {
            clock,
            shared,
            gate: PoseGate::default(),
            quit,
            worker: Some(worker),
        })
    }

    fn now_us(&self) -> u64 {
        self.clock.elapsed().as_micros() as u64
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn head_worker(
    estimator: Estimator,
    serial: Option<String>,
    clock: Instant,
    shared: Arc<Mutex<Shared>>,
    status: Arc<Mutex<TobiiStatus>>,
    quit: Arc<AtomicBool>,
) {
    let device = live::Service::start(serial, Arc::new(AtomicU64::new(0)), clock);
    let now = || clock.elapsed().as_micros() as u64;
    let mut tracker = HeadTracker::new();
    let mut handled_recenter = 0;
    let mut last_image: Option<(u64, u64)> = None;
    let mut last_status_us: Option<u64> = None;
    let mut last_state = None;
    while !quit.load(Ordering::Acquire) {
        let (requests, pressed_at) = {
            let s = lock(&shared);
            (s.recenter_requests, s.recenter_at_us)
        };
        if requests != handled_recenter {
            handled_recenter = requests;
            tracker.recenter(pressed_at);
        }
        let snap = device.snapshot();
        if snap.state == live::State::Streaming {
            tracker.begin_session(snap.session);
        }
        let mut evaluated = false;
        if let Some(m) = &snap.image {
            let key = (m.stamp.session, m.stamp.sequence);
            if last_image != Some(key) {
                last_image = Some(key);
                if let Some(image) = m.fresh(snap.session, now(), POSE_TTL) {
                    match estimator.evaluate(image, None) {
                        Ok(e) => tracker.observe(
                            m.stamp,
                            e.estimate.map(|p| p.image_angles_deg),
                            &e.reason,
                            now(),
                        ),
                        Err(err) => tracker.observe(
                            m.stamp,
                            None,
                            &format!("Head inference error: {err:#}"),
                            now(),
                        ),
                    }
                    evaluated = true;
                }
            }
        }
        {
            // Publish only if no Recenter arrived while this frame was evaluated:
            // a pre-recenter rotation must never reach the engine after the press.
            let mut s = lock(&shared);
            if s.recenter_requests == handled_recenter {
                s.latest = tracker.latest();
            }
        }
        let t = now();
        let mapped = map_status(&snap, &mut tracker, t);
        if last_state != Some(mapped.state)
            || last_status_us.is_none_or(|p| t.saturating_sub(p) >= STATUS_INTERVAL_US)
        {
            last_state = Some(mapped.state);
            last_status_us = Some(t);
            *lock(&status) = mapped;
        }
        if !evaluated {
            thread::sleep(Duration::from_millis(3));
        }
    }
    // Dropping the service cancels and joins the USB thread, which releases the
    // interface (vendor close 0x42) before this thread exits.
    drop(device);
}

impl InputSource for TobiiInput {
    fn name(&self) -> &str {
        NAME
    }

    fn poll(&mut self) -> Option<RawPose> {
        let latest = lock(&self.shared).latest;
        let now = self.now_us();
        self.gate.take(latest, now)
    }

    /// IR exposure → USB → inference. Inference dominates (~10–20 ms).
    fn latency_hint_us(&self) -> u32 {
        40_000
    }

    fn recenter(&mut self) {
        let now = self.now_us();
        let mut s = lock(&self.shared);
        s.latest = None;
        s.recenter_requests += 1;
        s.recenter_at_us = now;
    }
}

impl Drop for TobiiInput {
    fn drop(&mut self) {
        self.quit.store(true, Ordering::Release);
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use headtrack_tobii_device::{Gaze, Measurement, Validity};

    fn stamp(session: u64, seq: u64, t: u64) -> Stamp {
        Stamp {
            session,
            sequence: seq,
            device_us: t,
            received_us: t,
        }
    }
    fn sample(session: u64, seq: u64, t: u64, yaw: f32) -> Option<PoseSample> {
        Some(PoseSample {
            stamp: stamp(session, seq, t),
            rotation: [yaw, 0.0, 0.0],
        })
    }

    /// Feed `n` frames 100 ms apart with constant angles, starting at `t0`.
    fn feed(tr: &mut HeadTracker, session: u64, seq0: u64, t0: u64, n: u64, a: [f32; 3]) -> u64 {
        let mut t = t0;
        for i in 0..n {
            t = t0 + i * 100_000;
            tr.observe(stamp(session, seq0 + i, t), Some(a), "ok", t);
        }
        t
    }

    #[test]
    fn gate_emits_each_sample_once() {
        let mut g = PoseGate::default();
        let s = sample(1, 5, 1_000, 3.0);
        let p = g.take(s, 1_010).expect("fresh sample is emitted");
        assert_eq!((p.yaw, p.x, p.y, p.z, p.timestamp_us), (3.0, 0.0, 0.0, 0.0, 1_000));
        // The same sample, polled again and again while still fresh: never re-emitted.
        for now in [1_020, 50_000, 200_000] {
            assert!(g.take(s, now).is_none());
        }
        // An older sample (late publication) is refused too.
        assert!(g.take(sample(1, 4, 1_500, 9.0), 1_600).is_none());
        // A newer one goes through.
        assert!(g.take(sample(1, 6, 2_000, 4.0), 2_100).is_some());
    }

    #[test]
    fn gate_never_emits_stale_samples() {
        let mut g = PoseGate::default();
        // Exactly at the TTL: still fresh. One microsecond past: stale.
        assert!(g.take(sample(1, 1, 0, 1.0), POSE_TTL_US).is_some());
        assert!(g.take(sample(1, 2, 0, 1.0), POSE_TTL_US + 1).is_none());
        // A stale sample that was refused is not remembered as emitted…
        assert!(g.take(sample(1, 3, 1_000_000, 1.0), 1_000_010).is_some());
        // …and a sample stamped in the future (clock regression) is refused.
        assert!(g.take(sample(1, 4, 2_000_000, 1.0), 1_999_999).is_none());
    }

    #[test]
    fn gate_accepts_a_new_session_with_lower_sequence() {
        let mut g = PoseGate::default();
        assert!(g.take(sample(1, 500, 1_000, 1.0), 1_000).is_some());
        // Replug: session 2 restarts sequence numbering at 1.
        assert!(g.take(sample(2, 1, 5_000, 1.0), 5_000).is_some());
        assert!(g.take(sample(1, 501, 6_000, 1.0), 6_000).is_none());
    }

    #[test]
    fn first_stable_acquisition_takes_the_neutral_automatically() {
        let mut tr = HeadTracker::new();
        assert!(tr.collecting());
        // 2 s of steady frames at a mount-tilted pose → neutral captured.
        let t = feed(&mut tr, 1, 1, 100_000, 21, [-25.0, 3.0, 1.0]);
        assert!(!tr.collecting(), "neutral should be taken after 2 s still");
        let p = tr.latest().expect("pose after neutral");
        assert!(p.rotation.iter().all(|v| v.abs() < 1e-3), "{:?}", p.rotation);
        assert_eq!(p.stamp.received_us, t);
    }

    #[test]
    fn nothing_is_emitted_while_the_neutral_is_captured() {
        let mut tr = HeadTracker::new();
        feed(&mut tr, 1, 1, 100_000, 10, [0.0; 3]); // only 1 s
        assert!(tr.collecting());
        assert!(tr.latest().is_none());
    }

    #[test]
    fn recenter_restarts_the_neutral_and_drops_the_old_rotation() {
        let mut tr = HeadTracker::new();
        let t = feed(&mut tr, 1, 1, 100_000, 21, [0.0; 3]);
        assert!(tr.latest().is_some());
        tr.recenter(t);
        assert!(tr.collecting());
        assert!(tr.latest().is_none(), "old-neutral rotation must not survive Recenter");
        // A frame received before the press is not a neutral candidate.
        tr.observe(stamp(1, 100, t - 1), Some([10.0, 0.0, 0.0]), "ok", t);
        assert!(tr.collecting());
        // Two seconds still at a new pose → new neutral, rotation ~0 there.
        feed(&mut tr, 1, 200, t + 100_000, 21, [10.0, 0.0, 0.0]);
        assert!(!tr.collecting());
        let r = tr.latest().unwrap().rotation;
        assert!(r.iter().all(|v| v.abs() < 1e-3), "{r:?}");
    }

    #[test]
    fn a_new_usb_session_retakes_the_neutral() {
        let mut tr = HeadTracker::new();
        feed(&mut tr, 1, 1, 100_000, 21, [0.0; 3]);
        assert!(!tr.collecting());
        tr.begin_session(2);
        assert!(tr.collecting());
        assert!(tr.latest().is_none());
        assert_eq!(tr.face(), None);
    }

    #[test]
    fn rotation_follows_the_head_after_neutral() {
        let mut tr = HeadTracker::new();
        let t = feed(&mut tr, 1, 1, 100_000, 21, [0.0; 3]);
        // Image +Y is head-right (measured sign; neutral.rs): yaw +20.
        tr.observe(stamp(1, 50, t + 100_000), Some([0.0, 20.0, 0.0]), "ok", t + 100_000);
        let r = tr.latest().unwrap().rotation;
        assert!((r[0] - 20.0).abs() < 1e-3 && r[1].abs() < 1e-3 && r[2].abs() < 1e-3, "{r:?}");
    }

    #[test]
    fn no_face_frames_clear_the_neutral_collection() {
        let mut tr = HeadTracker::new();
        feed(&mut tr, 1, 1, 100_000, 15, [0.0; 3]);
        tr.observe(stamp(1, 99, 1_700_000), None, "No face detected", 1_700_000);
        assert_eq!(tr.face(), Some(false));
        assert_eq!(tr.reason(), "No face detected");
        // The 1.5 s already collected does not count: 2 s more are needed.
        feed(&mut tr, 1, 100, 1_800_000, 15, [0.0; 3]);
        assert!(tr.collecting());
    }

    fn snap(state: live::State, error: Option<UsbErrorKind>) -> Snapshot {
        Snapshot {
            state,
            error,
            message: "usb says".into(),
            session: 1,
            serial: Some("IS5FF-100000000001".into()),
            ..Default::default()
        }
    }

    #[test]
    fn device_states_map_per_the_p05_table() {
        use live::State::*;
        use UsbErrorKind as K;
        let cases = [
            (Opening, None, TobiiState::Opening),
            (Opening, Some(K::NoDevice), TobiiState::NoDevice),
            (Recovering, Some(K::NoDevice), TobiiState::NoDevice),
            (Recovering, Some(K::PermissionDenied), TobiiState::PermissionDenied),
            (Opening, Some(K::PermissionDenied), TobiiState::PermissionDenied),
            (Recovering, Some(K::Busy), TobiiState::Busy),
            (Recovering, Some(K::Disconnected), TobiiState::Lost),
            (Recovering, Some(K::Other), TobiiState::Lost),
            (Recovering, None, TobiiState::Lost),
            (Unsupported, Some(K::Other), TobiiState::NoDevice),
            (Closed, None, TobiiState::NoDevice),
            (Streaming, None, TobiiState::Opening), // no frame evaluated yet
        ];
        for (dev, err, want) in cases {
            let mut tr = HeadTracker::new();
            let s = map_status(&snap(dev, err), &mut tr, 0);
            assert_eq!(s.state, want, "{dev:?} {err:?}");
            assert_eq!(s.fps, 0.0);
            assert_eq!(s.serial.as_deref(), Some("IS5FF-100000000001"));
            if want != TobiiState::Opening || err.is_some() {
                assert_eq!(s.message, "usb says", "USB text reaches the tab for {dev:?}");
            }
        }
    }

    #[test]
    fn streaming_maps_face_and_noface_with_age() {
        let mut tr = HeadTracker::new();
        let t = feed(&mut tr, 1, 1, 100_000, 21, [0.0; 3]);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, t + 30_000);
        assert_eq!(s.state, TobiiState::Streaming);
        assert_eq!(s.last_sample_age_ms, Some(30));
        assert_eq!(s.message, "Tracking head rotation");
        tr.observe(stamp(1, 99, t + 100_000), None, "No face detected", t + 100_000);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, t + 110_000);
        assert_eq!(s.state, TobiiState::NoFace);
        assert_eq!(s.message, "No face detected");
    }

    #[test]
    fn collecting_neutral_says_so() {
        let mut tr = HeadTracker::new();
        feed(&mut tr, 1, 1, 100_000, 5, [0.0; 3]);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, 600_000);
        assert_eq!(s.state, TobiiState::Streaming);
        assert!(s.message.contains("capturing neutral"), "{}", s.message);
        assert_eq!(s.last_sample_age_ms, None, "no rotation sample yet");
        // 5 samples 100 ms apart = 0.4 s of the 2 s window.
        let p = s.neutral_progress.expect("progress while collecting");
        assert!((p - 0.2).abs() < 1e-6, "{p}");
    }

    #[test]
    fn neutral_progress_runs_until_the_neutral_and_again_after_recenter() {
        let mut tr = HeadTracker::new();
        // Device up, no frame evaluated yet: "Starting", no countdown.
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, 1_000);
        assert_eq!((s.state, s.neutral_progress), (TobiiState::Opening, None));
        let t = feed(&mut tr, 1, 1, 100_000, 21, [0.0; 3]);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, t + 10_000);
        assert_eq!(s.neutral_progress, None, "neutral taken: tracking");
        tr.recenter(t + 20_000);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, t + 30_000);
        assert_eq!(s.neutral_progress, Some(0.0), "recenter restarts the countdown");
        tr.observe(stamp(1, 99, t + 100_000), None, "No face detected", t + 100_000);
        let s = map_status(&snap(live::State::Streaming, None), &mut tr, t + 110_000);
        assert_eq!((s.state, s.neutral_progress), (TobiiState::NoFace, Some(0.0)));
        let s = map_status(&snap(live::State::Opening, None), &mut tr, t + 120_000);
        assert_eq!(s.neutral_progress, None, "no countdown unless frames arrive");
    }

    #[test]
    fn gaze_reaches_the_status_only_when_fresh_and_valid() {
        let mut tr = HeadTracker::new();
        let mut dev = snap(live::State::Streaming, None);
        dev.gaze = Some(Measurement {
            stamp: stamp(1, 1, 1_000),
            validity: Validity::Valid,
            value: Some(Gaze {
                normalized: [0.25, 0.75],
                eyes_present: [Some(true), Some(true)],
            }),
        });
        let g = map_status(&dev, &mut tr, 2_000).gaze.expect("fresh valid gaze");
        assert_eq!((g.x, g.y), (0.25, 0.75));
        assert!(map_status(&dev, &mut tr, 1_000 + POSE_TTL_US + 1).gaze.is_none());
        dev.gaze.as_mut().unwrap().validity = Validity::Invalid;
        assert!(map_status(&dev, &mut tr, 2_000).gaze.is_none());
    }

    /// End-to-end `evaluate` on synthetic IR frames (no face, no person —
    /// P06 privacy rule): the full detector path runs and reports "no face".
    #[test]
    fn synthetic_frames_without_a_face_report_no_face() {
        use headtrack_tobii_device::image::GrayImage;
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/models/tobii");
        let est = Estimator::load(&root).expect("shipped models load");
        let flat = GrayImage::new(280, 280, 280, &[96u8; 280 * 280]).unwrap();
        // Deterministic gradient + LCG noise.
        let mut x = 0x2545_f491_u32;
        let noise: Vec<u8> = (0..280 * 280)
            .map(|i| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((i % 280) as u32 / 2 + (x >> 26)) as u8
            })
            .collect();
        let noisy = GrayImage::new(280, 280, 280, &noise).unwrap();
        for img in [flat, noisy] {
            let e = est.evaluate(&img, None).expect("evaluate runs");
            assert!(e.estimate.is_none(), "{}", e.reason);
            assert_eq!(e.reason, "No face detected");
        }
        // The only IR mode supported is 280×280; anything else is an error, not a guess.
        let small = GrayImage::new(64, 64, 64, &[0u8; 64 * 64]).unwrap();
        assert!(est.evaluate(&small, None).is_err());
    }
}
