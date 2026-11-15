//! Desktop-global hotkeys via the XDG GlobalShortcuts portal.
//!
//! The sims swallow keyboard input while focused and the GUI's own hotkeys are
//! focus-gated, so mid-flight actions (recenter, MSFS seat nudges, camera
//! release) need a mechanism that fires regardless of focus. The portal is the
//! sanctioned Wayland answer: HeadtrackRS registers named actions once, the
//! desktop shows its standard binding dialog, the user assigns (and can later
//! rebind) keys in System Settings, and `Activated` signals arrive here over
//! D-Bus even while the sim has focus. Supported by KDE Plasma 5.27+ and
//! GNOME 48+.
//!
//! Graceful degradation: if the portal is missing or the user dismisses the
//! bind dialog, this task logs one line and exits — the command socket
//! (`headtrack-cmd.sock`) remains the scripting-tier fallback for the same
//! actions.

use std::process::{Child, Stdio};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ashpd::desktop::global_shortcuts::{GlobalShortcuts, NewShortcut, Shortcut};
use ashpd::WindowIdentifier;
use futures_util::StreamExt;
use tokio::sync::mpsc;
use tracing::{debug, info, warn};

use crate::actions::{action_defs, cmd_for};
use crate::{EngineCmd, EngineState};

/// Fresh `NewShortcut` list for a bind call (no preferred triggers: the
/// desktop's bind dialog owns key assignment, and the user can rebind any
/// time in System Settings → Shortcuts).
fn make_defs() -> Vec<NewShortcut> {
    action_defs()
        .into_iter()
        .map(|(id, desc)| NewShortcut::new(id, desc))
        .collect()
}

/// Publish the bound triggers into `EngineState` so the Keybinds tab can show
/// what each action is currently bound to (the DE's own dialog notoriously
/// doesn't echo the captured key). Ordered by our definition order.
fn publish(state: &EngineState, shortcuts: &[Shortcut]) {
    let list: Vec<(String, String)> = action_defs()
        .into_iter()
        .filter_map(|(id, desc)| {
            shortcuts
                .iter()
                .find(|s| s.id() == id)
                .map(|s| (desc.to_string(), s.trigger_description().to_string()))
        })
        .collect();
    if let Ok(mut g) = state.global_shortcuts.lock() {
        *g = Some(list);
    }
}

/// After opening the shortcut settings, further "Change bindings…" clicks
/// within this long do not launch it again. System Settings takes a second or
/// two to appear, and on 2026-11-01 four clicks made while it was starting
/// opened it four times in 2.5 s (D3).
const SETTINGS_REOPEN_GUARD: Duration = Duration::from_secs(10);

/// Whether a settings launch at `now` is allowed, given the last one.
fn may_open_settings(last_opened: Option<Instant>, now: Instant) -> bool {
    match last_opened {
        Some(t) => now.saturating_duration_since(t) >= SETTINGS_REOPEN_GUARD,
        None => true,
    }
}

/// Opens the shortcut settings at most once per [`SETTINGS_REOPEN_GUARD`],
/// and reaps the launched process so it does not linger as a zombie.
#[derive(Default)]
struct SettingsLauncher {
    child: Option<Child>,
    opened_at: Option<Instant>,
}

impl SettingsLauncher {
    fn open(&mut self) {
        self.reap();
        let now = Instant::now();
        if !may_open_settings(self.opened_at, now) {
            info!("global shortcuts: shortcut settings were just opened — not opening them again");
            return;
        }
        if let Some(child) = open_shortcut_settings() {
            self.child = Some(child);
            self.opened_at = Some(now);
        }
    }

    fn reap(&mut self) {
        if let Some(c) = &mut self.child {
            if !matches!(c.try_wait(), Ok(None)) {
                self.child = None;
            }
        }
    }
}

/// Launch the desktop's shortcuts settings page (best effort). Used when the
/// portal's bind call doesn't present a dialog of its own. Returns the child,
/// for the caller to reap.
fn open_shortcut_settings() -> Option<Child> {
    let desktop = std::env::var("XDG_CURRENT_DESKTOP")
        .unwrap_or_default()
        .to_uppercase();
    let candidates: &[(&str, &[&str])] = if desktop.contains("KDE") {
        // Plasma: the Shortcuts KCM lists portal-registered applications.
        &[
            ("systemsettings", &["kcm_keys"]),
            ("kcmshell6", &["kcm_keys"]),
        ]
    } else if desktop.contains("GNOME") {
        &[("gnome-control-center", &["keyboard"])]
    } else {
        warn!(
            "global shortcuts: don't know how to open shortcut settings on \
             desktop {desktop:?} — edit them in your system settings"
        );
        return None;
    };
    for (bin, args) in candidates {
        // Its stdout/stderr are not ours: inherited, KDE's own startup chatter
        // (libkbolt, qt.qpa.services) landed in this app's log (L7).
        if let Ok(child) = std::process::Command::new(bin)
            .args(*args)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
        {
            info!("global shortcuts: opened {bin} for shortcut editing");
            return Some(child);
        }
    }
    warn!("global shortcuts: could not launch a shortcut settings app");
    None
}

/// The bound triggers, for the log: `Recenter = F10, …`, `(none bound)` if empty.
fn describe(shortcuts: &[Shortcut]) -> String {
    let bound: Vec<String> = shortcuts
        .iter()
        .filter(|s| !s.trigger_description().is_empty())
        .map(|s| format!("{} = {}", s.id(), s.trigger_description()))
        .collect();
    if bound.is_empty() {
        "(none bound)".into()
    } else {
        bound.join(", ")
    }
}

/// Register the shortcuts and forward `Activated` signals as engine commands.
/// `rebind_rx` re-opens the desktop's binding dialog on the live session (the
/// GUI's "Change bindings…" button). Runs for the engine's lifetime; exits
/// (with one info log, `EngineState::global_shortcuts` left `None`) when the
/// portal is unavailable. Spawned from `engine_init`.
pub(crate) async fn run(
    cmd_tx: mpsc::Sender<EngineCmd>,
    state: Arc<EngineState>,
    rebind_rx: mpsc::UnboundedReceiver<()>,
) {
    if let Err(e) = run_inner(cmd_tx, &state, rebind_rx).await {
        info!(
            "global shortcuts: portal unavailable ({e}) — desktop hotkeys still \
             work via the command socket (see the Keybinds tab)"
        );
    }
}

async fn run_inner(
    cmd_tx: mpsc::Sender<EngineCmd>,
    state: &EngineState,
    mut rebind_rx: mpsc::UnboundedReceiver<()>,
) -> Result<(), ashpd::Error> {
    let portal = GlobalShortcuts::new().await?;
    // The session handle must stay alive — dropping it unregisters us.
    let session = portal.create_session().await?;

    let defs = make_defs();
    let n = defs.len();
    let request = portal
        .bind_shortcuts(&session, &defs, &WindowIdentifier::default())
        .await?;
    let bound = request.response()?;
    publish(state, bound.shortcuts());
    info!(
        "global shortcuts: bound triggers: {}",
        describe(bound.shortcuts())
    );
    info!(
        "global shortcuts: {}/{n} actions registered with the desktop portal — \
         assign keys in the bind dialog, System Settings → Shortcuts, or the \
         Keybinds tab's \"Change bindings…\" button",
        bound.shortcuts().len()
    );

    let mut activated = portal.receive_activated().await?;
    let mut changed = portal.receive_shortcuts_changed().await?;
    let mut settings = SettingsLauncher::default();
    loop {
        tokio::select! {
            sig = activated.next() => {
                let Some(signal) = sig else { break };
                let id = signal.shortcut_id();
                match cmd_for(id) {
                    Some(cmd) => {
                        // INFO: one line per key press, a user action. It is
                        // what tells "the desktop never sent it" (D4) from
                        // "the engine ignored it".
                        info!("global shortcuts: '{id}' activated");
                        let _ = cmd_tx.send(cmd).await;
                    }
                    None => info!("global shortcuts: unknown action id '{id}' activated"),
                }
            }
            sig = changed.next() => {
                // Bindings edited in System Settings — refresh the display.
                if let Some(c) = sig {
                    publish(state, c.shortcuts());
                    info!("global shortcuts: bindings changed: {}", describe(c.shortcuts()));
                }
            }
            req = rebind_rx.recv() => {
                let Some(()) = req else { break };
                // Try to re-open the desktop's binding dialog on the live
                // session. Portal implementations differ: KDE only shows the
                // dialog for NEW shortcuts and returns immediately for an
                // already-bound session — so if the round-trip comes back
                // faster than any human could have used a dialog, fall back to
                // launching the system settings shortcuts page instead (where
                // portal-registered shortcuts are editable).
                let t0 = std::time::Instant::now();
                let result = portal.bind_shortcuts(&session, &make_defs(), &WindowIdentifier::default()).await;
                // Clicks queued while the round-trip ran are the same request.
                let mut coalesced = 0;
                while rebind_rx.try_recv().is_ok() {
                    coalesced += 1;
                }
                if coalesced > 0 {
                    debug!("global shortcuts: {coalesced} repeated rebind click(s) coalesced");
                }
                match result {
                    Ok(request) => match request.response() {
                        Ok(b) => {
                            publish(state, b.shortcuts());
                            if t0.elapsed() < std::time::Duration::from_millis(1500) {
                                info!(
                                    "global shortcuts: portal returned without showing a \
                                     dialog — opening the system shortcuts settings"
                                );
                                settings.open();
                            } else {
                                info!("global shortcuts: bindings updated via rebind dialog");
                            }
                        }
                        Err(e) => info!("global shortcuts: rebind dialog dismissed ({e})"),
                    },
                    Err(e) => warn!("global shortcuts: rebind request failed: {e}"),
                }
            }
        }
    }

    drop(session);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_open_once_per_guard_window() {
        let t0 = Instant::now();
        assert!(may_open_settings(None, t0));
        // D3: the clicks 1.3, 1.7 and 2.0 s after the first open nothing.
        for ms in [1_300, 1_740, 2_010] {
            assert!(!may_open_settings(Some(t0), t0 + Duration::from_millis(ms)));
        }
        assert!(may_open_settings(Some(t0), t0 + SETTINGS_REOPEN_GUARD));
        // A clock that steps backwards never opens a burst.
        assert!(!may_open_settings(Some(t0 + Duration::from_secs(1)), t0));
    }
}
