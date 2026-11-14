//! Every `(stage, param)` pair the GUI's controls push to the engine.
//!
//! `Pipeline::set_param` returns `false` for an unknown stage or parameter and
//! every GUI call site discards that result, so a wrong name fails silently:
//! the slider moves, the label updates, and nothing reaches the pipeline. This
//! module exists to catch that.
//!
//! **What is now impossible.** Stage names are no longer written here or at the
//! call sites — both use the constants in
//! [`headtrack_core::pipeline::stage_names`], so a mistyped or renamed *stage*
//! is a compile error. Before SL3 this module's header admitted the hole it
//! could not close: a control edited without editing this list left the test
//! passing while the control was broken. For stage names that hole is gone.
//!
//! **What this still guards.** The *parameter* half is still composed as text
//! (`"{axis}.smoothing"`), so `every_gui_control_addresses_a_real_pipeline_param`
//! remains a `#[cfg(test)]` tripwire for those, with the same caveat as before:
//! it can only check the parameters listed below. Keep them in step when adding
//! a control.

use headtrack_core::pipeline::stage_names;
use headtrack_core::pose::AXIS_NAMES;

/// The twelve cross-axis coefficient ids, as written in `tabs/advanced.rs`.
pub const CROSS_AXIS_PARAMS: [&str; 12] = [
    "yaw_to_pitch",
    "yaw_to_roll",
    "yaw_to_x",
    "yaw_to_y",
    "yaw_to_z",
    "pitch_to_yaw",
    "pitch_to_y",
    "pitch_to_z",
    "roll_to_x",
    "roll_to_z",
    "z_to_pitch",
    "z_to_y",
];

/// Build the full `(stage, param)` set the GUI is capable of sending.
pub fn all_control_params() -> Vec<(&'static str, String)> {
    let mut out: Vec<(&'static str, String)> = Vec::new();

    for axis in AXIS_NAMES {
        // Tuning: axis on/off chips, and `toggle_tracking`.
        out.push((stage_names::AXIS_MASK, axis.to_string()));
        // Tuning + Advanced: response curve.
        out.push((stage_names::RESPONSE_CURVE, format!("{axis}.sensitivity")));
        out.push((stage_names::RESPONSE_CURVE, format!("{axis}.exponent")));
        // Tuning (grouped smoothing) + Advanced (per-axis Accela).
        out.push((stage_names::ACCELA, format!("{axis}.deadzone")));
        out.push((stage_names::ACCELA, format!("{axis}.smoothing")));
        out.push((stage_names::ACCELA, format!("{axis}.exponent")));
        // Tuning (grouped) + Advanced — bare axis name, not `axis.param`.
        out.push((stage_names::DEADZONE, axis.to_string()));
        // Advanced: rate limiter.
        out.push((stage_names::SLEW_LIMIT, axis.to_string()));
        // Advanced: centre-stage recenter range.
        out.push((stage_names::CENTER, format!("{axis}.recenter_range")));
    }

    for param in CROSS_AXIS_PARAMS {
        out.push((stage_names::CROSS_AXIS, param.to_string()));
    }

    out.push((stage_names::CENTER, "drift_rate".to_string()));
    out.push((stage_names::CENTER, "z.drift_mult".to_string()));

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use headtrack_core::pipeline::build_default_pipeline;

    /// Every parameter a GUI control can send must be one the pipeline
    /// actually accepts.
    ///
    /// Guards the silent-no-op failure mode: `set_param` returns `false` for an
    /// unknown stage or parameter and every GUI call site discards that result.
    /// A renamed stage or a mistyped key would leave the control looking alive
    /// while changing nothing in the pipeline.
    #[test]
    fn every_gui_control_addresses_a_real_pipeline_param() {
        let mut pipeline = build_default_pipeline();
        let mut rejected = Vec::new();

        for (stage, param) in all_control_params() {
            if !pipeline.set_param(stage, &param, 1.0) {
                rejected.push(format!("{stage} / {param}"));
            }
        }

        assert!(
            rejected.is_empty(),
            "these GUI controls address parameters the pipeline rejects, so moving them \
             would silently do nothing:\n  {}",
            rejected.join("\n  ")
        );
    }

    /// The cross-axis list must stay in step with `ProfileConfig`, otherwise a
    /// newly added coefficient is saved and loaded but has no control.
    ///
    /// `pitch_to_yaw` and `pitch_to_z` were in exactly that state before the
    /// GUI overhaul: carried in the profile, pushed by `profile_params`, and
    /// editable only by hand-editing the TOML.
    #[test]
    fn cross_axis_control_list_matches_the_profile() {
        let cfg = headtrack_core::config::ProfileConfig::default();
        let in_profile: Vec<String> = headtrack_core::pipeline::profile_params(&cfg)
            .into_iter()
            .filter(|(stage, _, _)| *stage == stage_names::CROSS_AXIS)
            .map(|(_, param, _)| param)
            .collect();

        let mut have: Vec<String> = CROSS_AXIS_PARAMS.iter().map(|s| s.to_string()).collect();
        let mut want = in_profile;
        have.sort();
        want.sort();

        assert_eq!(
            have, want,
            "the Advanced tab's cross-axis controls have drifted from ProfileConfig"
        );
    }
}
