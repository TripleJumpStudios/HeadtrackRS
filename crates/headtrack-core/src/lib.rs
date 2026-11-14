pub mod config;
pub mod input;
pub mod output;
pub mod pipeline;
pub mod pose;

// Re-export the most commonly used types at the crate root.
pub use input::{InputSource, SourceKind, TobiiState, TobiiStatus};
pub use output::{OutputError, OutputTarget};
pub use pipeline::{Pipeline, PipelineStage};
pub use pose::{Pose, RawPose, AXIS_NAMES};

/// Read a boolean `HEADTRACK_*` environment flag.
///
/// Truthy: `1`, `true`, `yes`, `on` (any case, surrounding whitespace ignored).
/// Everything else — including unset, empty, and unrecognised values — is
/// `false`, so a typo fails safe to the default rather than silently enabling
/// something.
///
/// Single definition on purpose: this logic was copy-pasted across five call
/// sites that had already drifted (some accepted `yes`, some did not, none
/// trimmed whitespace), which made "did my env var take effect?" unanswerable
/// without reading each one.
pub fn env_flag(name: &str) -> bool {
    matches!(
        std::env::var(name)
            .ok()
            .as_deref()
            .map(str::trim)
            .map(str::to_ascii_lowercase)
            .as_deref(),
        Some("1") | Some("true") | Some("yes") | Some("on")
    )
}

/// Read a boolean `HEADTRACK_*` flag that **disables** something.
///
/// Mirrors [`env_flag`]: returns `true` when the variable is set to `off` or
/// `0`, `false` otherwise. For opt-out switches where the feature is on by
/// default.
pub fn env_flag_off(name: &str) -> bool {
    matches!(
        std::env::var(name)
            .ok()
            .as_deref()
            .map(str::trim)
            .map(str::to_ascii_lowercase)
            .as_deref(),
        Some("off") | Some("0") | Some("false") | Some("no")
    )
}

#[cfg(test)]
mod env_flag_tests {
    use super::*;

    /// Env vars are process-global; this test owns one name nothing else uses.
    #[test]
    fn env_flag_accepts_truthy_forms_and_fails_safe() {
        const K: &str = "HEADTRACK_TEST_ENV_FLAG_ONLY";
        for v in ["1", "true", "TRUE", " yes ", "On"] {
            std::env::set_var(K, v);
            assert!(env_flag(K), "{v:?} should be truthy");
        }
        for v in ["0", "off", "", "maybe", "2"] {
            std::env::set_var(K, v);
            assert!(!env_flag(K), "{v:?} should be falsy");
        }
        std::env::remove_var(K);
        assert!(!env_flag(K), "unset should be falsy");
    }

    #[test]
    fn env_flag_off_detects_opt_out() {
        const K: &str = "HEADTRACK_TEST_ENV_FLAG_OFF_ONLY";
        for v in ["off", "0", "FALSE", " no "] {
            std::env::set_var(K, v);
            assert!(env_flag_off(K), "{v:?} should read as opt-out");
        }
        for v in ["1", "on", "", "yes"] {
            std::env::set_var(K, v);
            assert!(!env_flag_off(K), "{v:?} should not read as opt-out");
        }
        std::env::remove_var(K);
        assert!(!env_flag_off(K), "unset must not opt out");
    }
}
