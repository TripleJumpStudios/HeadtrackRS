use crate::pose::Pose;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum OutputError {
    #[error("output disconnected: {0}")]
    Disconnected(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

/// Any consumer of processed head pose data.
///
/// Output adapters live in their own crates (`headtrack-output-*`) and are
/// registered in the daemon at startup.  Multiple targets can be active
/// simultaneously — they all receive the same processed pose.
///
/// # Implementing a new target
/// 1. Create `headtrack-output-<name>`.
/// 2. Implement this trait.
/// 3. Register in the daemon's output registry.
pub trait OutputTarget: Send + 'static {
    /// Human-readable name shown in the UI and logs.
    fn name(&self) -> &str;

    /// Deliver a processed pose to the target.
    ///
    /// Called once per pipeline cycle.  Must not block significantly;
    /// use async channels or a background thread if the transport is slow.
    fn write(&mut self, pose: &Pose) -> Result<(), OutputError>;
}
