use crate::pipeline::DistanceScalePolicy;
use crate::pose::RawPose;

/// Any source of raw head pose data.
///
/// Implementors run on the daemon's input thread.  `poll` is called in a
/// tight loop at the daemon's target rate (~250 Hz); return `None` when no
/// new frame is ready (e.g. a 30 fps webcam source will return `None` ~220
/// times per second).
///
/// # Implementing a new source
/// 1. Create a new crate `headtrack-input-<name>`.
/// 2. Implement this trait on your driver struct.
/// 3. Add a `HeadSource` variant and open it in `headtrack-engine/src/source.rs`
///    (`open_source`), behind its own Cargo feature. There is no general registry:
///    the engine knows each source by name.
pub trait InputSource: Send + 'static {
    /// Human-readable name shown in the UI and logs.
    fn name(&self) -> &str;

    /// Return the latest raw pose, or `None` if no new frame is available.
    ///
    /// Must not block.  Use internal buffering / async channels if the
    /// underlying hardware API is event-driven.
    fn poll(&mut self) -> Option<RawPose>;

    /// Best-effort estimate of end-to-end capture latency in microseconds.
    ///
    /// Informational only: no stage consumes it since the prediction stage was
    /// removed (SL2). Return `0` if unknown or negligible.
    fn latency_hint_us(&self) -> u32;

    /// The user pressed Recenter.
    ///
    /// The engine calls this *before* the pipeline's CenterStage recenter.
    /// Default: no-op — the webcam has no source-side neutral. A source that
    /// reports rotation relative to a captured neutral (the Tobii) restarts
    /// that capture here.
    fn recenter(&mut self) {}
}

/// Which kind of head source is active — the single place every
/// "is this the Tobii?" decision is answered. Engine and GUI match on this,
/// never on [`InputSource::name`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SourceKind {
    Webcam,
    Tobii,
}

impl SourceKind {
    /// How translation thresholds follow seating distance for this source.
    pub fn distance_scale_policy(self) -> DistanceScalePolicy {
        match self {
            Self::Webcam => DistanceScalePolicy::FromStandoff,
            Self::Tobii => DistanceScalePolicy::None,
        }
    }

    /// Whether the engine should feed this source's Z into the standoff
    /// estimate. The Tobii's translation is held at 0, which is not a standoff.
    pub fn observes_standoff(self) -> bool {
        matches!(self, Self::Webcam)
    }

    /// Whether X/Y/Z carry real data. False for the Tobii (3DOF rotation;
    /// translation held at 0 and labelled unavailable in the GUI).
    pub fn provides_translation(self) -> bool {
        matches!(self, Self::Webcam)
    }

    /// Display name.
    pub fn label(self) -> &'static str {
        match self {
            Self::Webcam => "Webcam",
            Self::Tobii => "Tobii Eye Tracker 5",
        }
    }
}

/// Tobii device state as shown in the Tobii tab.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TobiiState {
    /// USB 2104:0313 is not present.
    #[default]
    NoDevice,
    /// Present but cannot be opened — the udev rule is missing.
    PermissionDenied,
    /// Present but claimed by another process.
    Busy,
    /// USB handshake in progress.
    Opening,
    /// Frames arriving and a face is tracked.
    Streaming,
    /// Frames arriving, no face in the IR image.
    NoFace,
    /// Was streaming; transfers stopped (unplug, USB error). Reopening.
    Lost,
}

/// A gaze point in normalised display coordinates (0..1, origin top-left).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GazePoint {
    pub x: f32,
    pub y: f32,
}

/// Snapshot of the Tobii source for the GUI. Published by the Tobii service
/// thread; the engine owns the sharing, so this type holds no locks.
///
/// Gaze lives **only** here. No output adapter sees this type, and `RawPose`
/// has no gaze field — gaze cannot reach a sim by construction.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TobiiStatus {
    pub state: TobiiState,
    /// IR head-pose rate over the last second.
    pub fps: f32,
    /// Age of the newest sample; `None` = no sample this session.
    pub last_sample_age_ms: Option<u32>,
    pub serial: Option<String>,
    /// For the Tobii tab only — never sent to a sim.
    pub gaze: Option<GazePoint>,
    /// Human-readable detail for the tab, e.g. the USB error text.
    pub message: String,
    /// `Some(p)` while the source is taking its neutral (at first start and
    /// after every recenter): `p` in `0.0..=1.0` is how much of the required
    /// still window has been held so far. Moving resets it to 0. `None` when
    /// no neutral is being taken. The GUI draws its hold-still countdown from
    /// this; nothing is sent to a sim while it is `Some`.
    pub neutral_progress: Option<f32>,
}
