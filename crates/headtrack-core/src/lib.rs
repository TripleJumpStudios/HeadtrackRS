pub mod config;
pub mod filter;
pub mod input;
pub mod output;
pub mod pipeline;
pub mod pose;

// Re-export the most commonly used types at the crate root.
pub use input::InputSource;
pub use output::{OutputError, OutputTarget};
pub use pipeline::{Pipeline, PipelineStage};
pub use pose::{Pose, RawPose};
