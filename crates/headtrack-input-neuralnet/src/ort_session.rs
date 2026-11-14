//! Shared ONNX Runtime session construction.
//!
//! One place to apply session options that should hold for every model we load,
//! so a stability/performance trade is a single knob rather than one per model.
//!
//! # `HEADTRACK_ORT_OPT_LEVEL` — the NCHWc escape hatch
//!
//! ORT's level-3 pass (`ORT_ENABLE_LAYOUT`) rewrites convolutions into the
//! NCHWc memory layout for spatial locality. It is on by default (sessions
//! commit at `ORT_ENABLE_ALL`) and it is worth having: measured on the
//! 6DRepNet MobileNetV3-Large at 224×224, it is **2.67 ms/inference with the
//! pass versus 3.57 ms without** — a third of the cost of that model.
//!
//! It is also the only thing the observed inference failures have in common.
//! Every one of them named a node the pass created:
//!
//! ```text
//! Non-zero status code returned while running ReorderInput node.
//!     Name:'ReorderInput_token_224'  Status Message: GetElementType is not implemented
//! Non-zero status code returned while running Conv node.
//!     Name:'/features/.../Relu_output_0_nchwc'  Status Message: GetElementType is not implemented
//! ```
//!
//! They are rare (one or two across a multi-hour session, under heavy sim load)
//! and the tracker already absorbs them by holding the last learned rotation,
//! so paying 34 % more inference cost on every frame to avoid them is a bad
//! trade by default. On a machine where they are frequent it is a good one:
//!
//! ```bash
//! HEADTRACK_ORT_OPT_LEVEL=level2   # extended fusions, no NCHWc rewrite
//! HEADTRACK_ORT_OPT_LEVEL=level1   # basic rewrites only
//! HEADTRACK_ORT_OPT_LEVEL=all      # the default, stated explicitly
//! ```

use anyhow::{Context, Result};
use ort::session::builder::{GraphOptimizationLevel, SessionBuilder};
use ort::session::Session;
use tracing::{info, warn};

/// Read the requested optimization level, or `None` to leave ORT at its default.
///
/// An unrecognised value warns and is ignored — a typo in an env var must not
/// silently change how inference runs.
fn requested_level() -> Option<GraphOptimizationLevel> {
    let raw = std::env::var("HEADTRACK_ORT_OPT_LEVEL").ok()?;
    let level = match raw.trim().to_ascii_lowercase().as_str() {
        "" => return None,
        "level1" | "basic" => GraphOptimizationLevel::Level1,
        "level2" | "extended" => GraphOptimizationLevel::Level2,
        "level3" | "layout" => GraphOptimizationLevel::Level3,
        "all" | "default" => GraphOptimizationLevel::All,
        "disable" | "off" => GraphOptimizationLevel::Disable,
        other => {
            warn!(
                "HEADTRACK_ORT_OPT_LEVEL={other:?} is not a level \
                 (level1/level2/level3/all/disable) — using the ORT default"
            );
            return None;
        }
    };
    Some(level)
}

/// A session builder carrying the shared options.
///
/// `what` names the model in error messages, so a failure says which session
/// could not be built (SCRFD or 6DRepNet; SL3 deleted the other two).
pub(crate) fn builder(what: &str) -> Result<SessionBuilder> {
    let builder = Session::builder().with_context(|| format!("creating {what} session builder"))?;
    match requested_level() {
        Some(level) => {
            info!("{what}: ONNX graph optimization pinned to {level:?} (HEADTRACK_ORT_OPT_LEVEL)");
            // `with_optimization_level` hands the builder back inside its error
            // type, which is not a `std::error::Error`, so anyhow's `context`
            // does not apply — same shape as the SCRFD log-level call.
            builder
                .with_optimization_level(level)
                .map_err(|e| anyhow::anyhow!("setting {what} optimization level: {e}"))
        }
        None => Ok(builder),
    }
}
