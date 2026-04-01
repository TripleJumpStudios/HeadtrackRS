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
/// 3. Register it in the daemon's input registry.
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
    /// The prediction stage uses this to compensate for hardware delay.
    /// Return `0` if unknown or negligible.
    fn latency_hint_us(&self) -> u32;
}
