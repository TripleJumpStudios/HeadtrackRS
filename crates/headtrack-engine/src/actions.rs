//! Bindable engine actions — the single definition shared by every hotkey
//! tier (controller buttons, the XDG GlobalShortcuts portal, and — via the
//! same ids — anything future).
//!
//! Ids are stable API: users bind keys/buttons to them and renames orphan
//! those bindings silently.

use crate::EngineCmd;

/// Seat nudge per activation, metres (matches the GUI's ±1 cm buttons).
#[cfg(feature = "output-simconnect")]
const SEAT_STEP: f32 = 0.01;

/// `(action id, human description)` for every bindable action.
pub(crate) fn action_defs() -> Vec<(&'static str, &'static str)> {
    #[allow(unused_mut)]
    let mut defs = vec![("recenter", "Recenter head tracking")];
    #[cfg(feature = "output-simconnect")]
    defs.extend_from_slice(&[
        ("msfs-camera-toggle", "MSFS: release / re-grab the camera (external views)"),
        ("msfs-seat-reset", "MSFS: reset seat position"),
        ("msfs-seat-left", "MSFS: seat left 1 cm"),
        ("msfs-seat-right", "MSFS: seat right 1 cm"),
        ("msfs-seat-down", "MSFS: seat down 1 cm"),
        ("msfs-seat-up", "MSFS: seat up 1 cm"),
        ("msfs-seat-back", "MSFS: seat aft 1 cm"),
        ("msfs-seat-fwd", "MSFS: seat forward 1 cm"),
    ]);
    defs
}

/// Map an action id to its engine command.
pub(crate) fn cmd_for(id: &str) -> Option<EngineCmd> {
    match id {
        "recenter" => Some(EngineCmd::Recenter),
        #[cfg(feature = "output-simconnect")]
        "msfs-camera-toggle" => Some(EngineCmd::SimConnectToggleEnabled),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-reset" => Some(EngineCmd::MsfsSeatReset),
        // EYEPOINT +X = LEFT (verified in-cockpit 2026-08-01) — hence the
        // inverted-looking signs on the X axis.
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-left" => Some(EngineCmd::MsfsSeatNudge { axis: 0, delta_m: SEAT_STEP }),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-right" => Some(EngineCmd::MsfsSeatNudge { axis: 0, delta_m: -SEAT_STEP }),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-down" => Some(EngineCmd::MsfsSeatNudge { axis: 1, delta_m: -SEAT_STEP }),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-up" => Some(EngineCmd::MsfsSeatNudge { axis: 1, delta_m: SEAT_STEP }),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-back" => Some(EngineCmd::MsfsSeatNudge { axis: 2, delta_m: -SEAT_STEP }),
        #[cfg(feature = "output-simconnect")]
        "msfs-seat-fwd" => Some(EngineCmd::MsfsSeatNudge { axis: 2, delta_m: SEAT_STEP }),
        _ => None,
    }
}
