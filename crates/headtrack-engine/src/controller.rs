//! Controller / HOTAS button bindings — the in-app, focus-independent hotkey
//! tier.
//!
//! Joystick-class evdev devices are readable by the seated user on modern
//! Linux (udev `uaccess`), and evdev is multi-reader — the sim reading the
//! stick doesn't stop us reading it too. So a button bound here fires
//! mid-flight regardless of window focus, with no portal, no desktop dialogs
//! and no scripting: the bind UI lives entirely in the Keybinds tab.
//!
//! Capture is hardened for flight hardware: HOTAS maintained switches park on
//! a pressed button permanently, so at capture start we snapshot every
//! currently-pressed button (`EVIOCGKEY`) and never accept those — only a
//! fresh press binds. A short grace period additionally swallows input queued
//! behind the Bind click, and a timeout ends an abandoned capture.

use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use evdev::{Device, EventSummary, KeyCode};
use tracing::{debug, info, warn};

use headtrack_core::config::{CameraConfig, ControllerBinding};

use crate::actions::{action_defs, cmd_for};
use crate::{EngineCmd, EngineState};

/// Poll cadence for the listener thread (evdev reads are non-blocking).
const POLL: Duration = Duration::from_millis(10);
/// Device hotplug rescan cadence.
const RESCAN: Duration = Duration::from_secs(3);
/// Input arriving this soon after Bind was clicked is queued noise, not intent.
const CAPTURE_GRACE: Duration = Duration::from_millis(250);
/// An abandoned capture ends itself.
const CAPTURE_TIMEOUT: Duration = Duration::from_secs(15);

/// Capture-control commands from the GUI (via the run loop).
pub(crate) enum ControllerCmd {
    StartCapture { action: String },
    CancelCapture,
    Clear { action: String },
}

/// Joystick/gamepad button code ranges (BTN_JOYSTICK, BTN_GAMEPAD, and the
/// full BTN_TRIGGER_HAPPY block — VKB/Virpil HOTAS map up to 64 buttons
/// there, 0x2c0–0x2ff). Deliberately excludes mouse buttons and keyboard keys.
fn is_button(code: u16) -> bool {
    matches!(code, 0x120..=0x12f | 0x130..=0x13e | 0x2c0..=0x2ff)
}

/// A device qualifies when it exposes at least one joystick-class button.
fn is_controller(dev: &Device) -> bool {
    dev.supported_keys()
        .is_some_and(|keys| keys.iter().any(|k| is_button(k.0)))
}

fn device_name(dev: &Device) -> String {
    dev.name()
        .unwrap_or("unknown controller")
        .trim()
        .to_string()
}

/// Human label for a button code ("BTN_TRIGGER", "BTN_TRIGGER_HAPPY5", …).
fn button_label(code: u16) -> String {
    format!("{:?}", KeyCode::new(code))
}

struct Capture {
    action: String,
    started: Instant,
    /// `(device name, code)` pressed at capture start — never accepted.
    initial: HashSet<(String, u16)>,
}

/// Publish the full binding view for the GUI: `(action id, description,
/// bound label)` in definition order.
fn publish(state: &EngineState, bindings: &[ControllerBinding]) {
    let list: Vec<(String, String, Option<String>)> = action_defs()
        .into_iter()
        .map(|(id, desc)| {
            let label = bindings
                .iter()
                .find(|b| b.action == id)
                .map(|b| b.label.clone());
            (id.to_string(), desc.to_string(), label)
        })
        .collect();
    if let Ok(mut g) = state.controller_bindings.lock() {
        *g = list;
    }
}

fn publish_capture(state: &EngineState, action: Option<&str>) {
    if let Ok(mut c) = state.controller_capture.lock() {
        *c = action.map(String::from);
    }
}

fn save(bindings: &[ControllerBinding]) {
    let mut cfg = CameraConfig::load();
    cfg.controller_bindings = bindings.to_vec();
    let _ = cfg.save();
}

/// Snapshot every currently-pressed button across `devices`.
fn pressed_now(devices: &[(PathBuf, Device)]) -> HashSet<(String, u16)> {
    let mut set = HashSet::new();
    for (_, dev) in devices {
        let name = device_name(dev);
        if let Ok(keys) = dev.get_key_state() {
            for k in keys.iter() {
                set.insert((name.clone(), k.0));
            }
        }
    }
    set
}

/// Refresh the device list: open newly-appeared controllers (non-blocking),
/// drop vanished ones. Returns whether the set changed.
fn rescan(devices: &mut Vec<(PathBuf, Device)>) -> bool {
    let mut changed = false;
    let found: Vec<(PathBuf, Device)> = evdev::enumerate()
        .filter(|(_, d)| is_controller(d))
        .collect();
    let before = devices.len();
    devices.retain(|(p, _)| found.iter().any(|(fp, _)| fp == p));
    changed |= devices.len() != before;
    for (path, dev) in found {
        if devices.iter().any(|(p, _)| *p == path) {
            continue; // already open — drop the duplicate handle
        }
        if dev.set_nonblocking(true).is_err() {
            continue;
        }
        info!(
            "controller bindings: found '{}' ({})",
            device_name(&dev),
            path.display()
        );
        devices.push((path, dev));
        changed = true;
    }
    changed
}

pub(crate) fn spawn(
    cmd_tx: tokio::sync::mpsc::Sender<EngineCmd>,
    state: Arc<EngineState>,
    ctrl_rx: std::sync::mpsc::Receiver<ControllerCmd>,
) {
    if let Err(e) = std::thread::Builder::new()
        .name("controller-bindings".into())
        .spawn(move || run(cmd_tx, state, ctrl_rx))
    {
        warn!("controller bindings: couldn't spawn listener thread: {e}");
    }
}

fn run(
    cmd_tx: tokio::sync::mpsc::Sender<EngineCmd>,
    state: Arc<EngineState>,
    ctrl_rx: std::sync::mpsc::Receiver<ControllerCmd>,
) {
    let mut bindings: Vec<ControllerBinding> = CameraConfig::load().controller_bindings;
    let mut devices: Vec<(PathBuf, Device)> = Vec::new();
    let mut last_scan = Instant::now() - RESCAN;
    let mut capture: Option<Capture> = None;
    publish(&state, &bindings);

    loop {
        if last_scan.elapsed() >= RESCAN {
            last_scan = Instant::now();
            if rescan(&mut devices) {
                // Device set changed mid-capture: fold newly-visible held
                // buttons into the never-accept set (a maintained switch on a
                // just-plugged device must not bind).
                if let Some(cap) = capture.as_mut() {
                    cap.initial.extend(pressed_now(&devices));
                }
            }
        }

        while let Ok(cmd) = ctrl_rx.try_recv() {
            match cmd {
                ControllerCmd::StartCapture { action } => {
                    capture = Some(Capture {
                        initial: pressed_now(&devices),
                        started: Instant::now(),
                        action: action.clone(),
                    });
                    publish_capture(&state, Some(&action));
                }
                ControllerCmd::CancelCapture => {
                    capture = None;
                    publish_capture(&state, None);
                }
                ControllerCmd::Clear { action } => {
                    bindings.retain(|b| b.action != action);
                    save(&bindings);
                    publish(&state, &bindings);
                    info!("controller bindings: cleared '{action}'");
                }
            }
        }

        if capture
            .as_ref()
            .is_some_and(|c| c.started.elapsed() > CAPTURE_TIMEOUT)
        {
            capture = None;
            publish_capture(&state, None);
            info!("controller bindings: capture timed out");
        }

        // Collect fresh button-down events across all controllers.
        let mut presses: Vec<(String, u16)> = Vec::new();
        let mut dead: Vec<PathBuf> = Vec::new();
        for (path, dev) in devices.iter_mut() {
            let name = device_name(dev);
            match dev.fetch_events() {
                Ok(events) => {
                    for ev in events {
                        if let EventSummary::Key(_, key, 1) = ev.destructure() {
                            if is_button(key.0) {
                                presses.push((name.clone(), key.0));
                            }
                        }
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => {
                    debug!("controller bindings: '{name}' read failed ({e}) — dropping");
                    dead.push(path.clone());
                }
            }
        }
        devices.retain(|(p, _)| !dead.contains(p));

        for (dev_name, code) in presses {
            if let Some(cap) = capture.take() {
                let fresh = cap.started.elapsed() >= CAPTURE_GRACE
                    && !cap.initial.contains(&(dev_name.clone(), code));
                if !fresh {
                    // Parked switch or queued input — keep waiting.
                    capture = Some(cap);
                    continue;
                }
                let label = format!("{dev_name} · {}", button_label(code));
                // One binding per action, one action per button.
                bindings.retain(|b| {
                    b.action != cap.action && !(b.device == dev_name && b.code == code)
                });
                bindings.push(ControllerBinding {
                    action: cap.action.clone(),
                    device: dev_name,
                    code,
                    label: label.clone(),
                });
                save(&bindings);
                publish(&state, &bindings);
                publish_capture(&state, None);
                info!("controller bindings: '{}' bound to {label}", cap.action);
            } else if let Some(b) = bindings
                .iter()
                .find(|b| b.code == code && b.device == dev_name)
            {
                if let Some(cmd) = cmd_for(&b.action) {
                    debug!("controller bindings: '{}' fired ({})", b.action, b.label);
                    let _ = cmd_tx.try_send(cmd);
                }
            }
        }

        std::thread::sleep(POLL);
    }
}
