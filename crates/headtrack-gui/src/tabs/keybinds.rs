use crate::{colors, ui, HeadtrackApp, HotkeyAction};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_engine::EngineCmd;

/// Narrowest the shared label column of the binding grids may be.
const LABEL_MIN_W: f32 = 130.0;

/// Column gap of the binding grids.
const GRID_GAP: f32 = 10.0;

/// How long "Change bindings…" rests after a click, saying it is opening the
/// settings. Long enough for KDE System Settings to appear; the engine also
/// refuses a second launch for 10 s (`global_shortcuts::SETTINGS_REOPEN_GUARD`).
const REBIND_FEEDBACK: std::time::Duration = std::time::Duration::from_secs(4);

impl HeadtrackApp {
    // ── Keybinds ──────────────────────────────────────────────────────────────

    pub(crate) fn ui_keybinds(&mut self, ui: &mut egui::Ui) {
        ui::page_header(ui, "Keybinds", |_ui| {});

        // One label column for the three binding grids, so their buttons and
        // values start at the same x. Each grid used to size its own column,
        // and the in-app buttons sat ~200 px left of the controller ones (8b).
        let label_w = self.keybind_label_width(ui);

        // Controller first, then the keyboard: in-app hotkeys, with the
        // desktop shortcuts as a disclosure under them (L6, 2026-11-02: two
        // separate keyboard cards confused, and the desktop path read as the
        // main one), then scripting.
        self.ui_keybinds_controller(ui, label_w);
        ui.add_space(ui::ROW_GAP);
        self.ui_keybinds_keyboard(ui, label_w);
        ui.add_space(ui::ROW_GAP);
        self.ui_keybinds_scripting(ui);
        ui.add_space(8.0);
    }

    /// Width of the widest action label in any of the three binding grids,
    /// at least [`LABEL_MIN_W`] and at most 40 % of the page.
    fn keybind_label_width(&self, ui: &egui::Ui) -> f32 {
        let mut labels: Vec<String> = vec!["Recenter".into(), "Toggle tracking".into()];
        if let Ok(b) = self.state.controller_bindings.lock() {
            labels.extend(b.iter().map(|(_, desc, _)| desc.clone()));
        }
        if let Ok(Some(list)) = self.state.global_shortcuts.lock().as_deref() {
            labels.extend(list.iter().map(|(desc, _)| desc.clone()));
        }
        let font = egui::FontId::proportional(colors::TEXT_BODY);
        let widest = ui.fonts(|f| {
            labels
                .into_iter()
                .map(|l| f.layout_no_wrap(l, font.clone(), colors::TEXT).rect.width())
                .fold(0.0_f32, f32::max)
        });
        widest.ceil().clamp(LABEL_MIN_W, (ui.available_width() * 0.4).max(LABEL_MIN_W))
    }

    /// Group header: heading on the left, the tier's state mark beside it,
    /// then one sentence on what the tier can and cannot do.
    fn tier_header(ui: &mut egui::Ui, title: &str, tier: &str, tone: ui::Tone, blurb: &str) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 12.0;
            ui::heading(ui, title);
            ui::state(ui, tier, tone);
        });
        ui.add_space(4.0);
        ui::note(ui, blurb, colors::DIM);
        ui.add_space(8.0);
    }

    // ── Tier 1: controller / HOTAS ───────────────────────────────────────────

    /// Device-level evdev bindings — global by nature, bound entirely in-app.
    /// The only tier that works while a sim has focus without desktop setup.
    fn ui_keybinds_controller(&mut self, ui: &mut egui::Ui, label_w: f32) {
        ui::group(ui, None, |ui| {
            Self::tier_header(
                ui,
                "Controller / HOTAS buttons",
                "Primary",
                ui::Tone::Accent,
                "Works while a sim has focus; only a fresh press binds, so switches already held \
                 are ignored.",
            );

            let bindings = self
                .state
                .controller_bindings
                .lock()
                .map(|g| g.clone())
                .unwrap_or_default();
            let capturing = self
                .state
                .controller_capture
                .lock()
                .ok()
                .and_then(|c| c.clone());

            if bindings.is_empty() {
                ui.label(ui::icon_text(
                    ph::GAME_CONTROLLER,
                    "No joystick-class devices detected.",
                    colors::DIM,
                ));
                return;
            }

            egui::Grid::new("controller_bind_grid")
                .spacing([GRID_GAP, 6.0])
                .min_col_width(LABEL_MIN_W)
                .show(ui, |ui| {
                    for (action, desc, label) in &bindings {
                        row_label(ui, desc, label_w);

                        let is_capturing = capturing.as_deref() == Some(action.as_str());
                        // Bindings read like "VKB-Sim (C) Alex Oz 2023 S-TECS
                        // MODERN THROTTLE MAX STEM · BTN_TRIGGER_HAPPY18". Shown
                        // whole, one of them sets the width of every button in
                        // the grid. The button name is the part that identifies
                        // the binding; the device goes in the tooltip.
                        let text = if is_capturing {
                            "Press a button\u{2026} (click to cancel)".to_string()
                        } else {
                            match label {
                                Some(l) => short_binding(l),
                                None => "Unbound".into(),
                            }
                        };
                        let kind = if is_capturing {
                            ui::ButtonKind::GhostActive
                        } else {
                            ui::ButtonKind::Normal
                        };
                        let mut resp = ui::tool_button(ui, "", &text, kind);
                        if let Some(full) = label {
                            resp = resp.on_hover_text(full);
                        }
                        if resp.clicked() {
                            if is_capturing {
                                self.cmd.send(EngineCmd::ControllerBindCancel);
                            } else {
                                self.cmd.send(EngineCmd::ControllerBindStart {
                                    action: action.clone(),
                                });
                            }
                        }

                        if ui::tool_button_enabled(
                            ui,
                            label.is_some(),
                            "",
                            "Clear",
                            ui::ButtonKind::Normal,
                        )
                        .clicked()
                        {
                            self.cmd.send(EngineCmd::ControllerBindClear {
                                action: action.clone(),
                            });
                        }
                        ui.end_row();
                    }
                });

            // The engine updates capture state asynchronously — keep repainting
            // so the bound button appears the moment it is pressed.
            if capturing.is_some() {
                ui.ctx()
                    .request_repaint_after(std::time::Duration::from_millis(100));
            }
        });
    }

    // ── Tier 2: the keyboard ─────────────────────────────────────────────────

    /// In-app hotkeys, bound right here, and under them the way to make a key
    /// work while a sim has focus: the desktop's shortcut settings.
    fn ui_keybinds_keyboard(&mut self, ui: &mut egui::Ui, label_w: f32) {
        ui::group(ui, None, |ui| {
            Self::tier_header(
                ui,
                "Keyboard hotkeys",
                "Window only",
                ui::Tone::Idle,
                "Fire while this window has keyboard focus. For a key that works while a sim \
                 has focus, set it up as a desktop shortcut below.",
            );

            egui::Grid::new("hotkey_grid")
                .spacing([GRID_GAP, 8.0])
                .min_col_width(LABEL_MIN_W)
                .show(ui, |ui| {
                    self.hotkey_row(ui, "Recenter", HotkeyAction::Recenter, label_w);
                    self.hotkey_row(ui, "Toggle tracking", HotkeyAction::Toggle, label_w);
                });

            ui.add_space(8.0);
            ui::disclosure(ui, "desktop_shortcuts", "Use keys while a sim has focus", |ui| {
                self.ui_desktop_shortcuts(ui, label_w);
            });
        });
    }

    /// The XDG GlobalShortcuts portal: what the desktop has bound, and the
    /// button that opens its settings.
    fn ui_desktop_shortcuts(&mut self, ui: &mut egui::Ui, label_w: f32) {
        // Live mirror from the engine's portal task: None = unavailable.
        let shortcuts = self
            .state
            .global_shortcuts
            .lock()
            .ok()
            .and_then(|g| g.clone());

        let Some(list) = shortcuts else {
            ui::note(
                ui,
                "Your desktop has no shortcuts portal (it needs KDE Plasma 5.27+ or GNOME 48+). \
                 A controller button above, or the command socket below, works instead.",
                colors::DIM,
            );
            return;
        };

        ui::note(
            ui,
            "Desktop shortcuts work in every window. Assign the keys in your desktop's \
             shortcut settings; they show here once set.",
            colors::DIM,
        );
        ui.add_space(6.0);
        // The disclosure indents its body. Narrow the label column by the same
        // amount so the values still start where the other grids' buttons do
        // (`the_keybind_grids_share_their_label_column`).
        let label_w = (label_w - ui.spacing().indent).max(0.0);
        egui::Grid::new("portal_shortcut_grid")
            .spacing([GRID_GAP, 5.0])
            .min_col_width(0.0)
            .show(ui, |ui| {
                for (desc, trigger) in &list {
                    row_label(ui, desc, label_w);
                    if trigger.is_empty() {
                        ui.label(
                            egui::RichText::new("Not assigned")
                                .size(colors::TEXT_BODY)
                                .color(colors::DIM),
                        );
                    } else {
                        ui.label(
                            egui::RichText::new(trigger)
                                .monospace()
                                .size(colors::TEXT_SM)
                                .color(colors::TEXT_HI),
                        );
                    }
                    ui.end_row();
                }
            });
        ui.add_space(8.0);
        let waiting = self
            .rebind_clicked_at
            .is_some_and(|t| t.elapsed() < REBIND_FEEDBACK);
        if ui::tool_button_enabled(
            ui,
            !waiting,
            ph::SLIDERS_HORIZONTAL,
            "Change bindings\u{2026}",
            ui::ButtonKind::Normal,
        )
        .clicked()
        {
            self.cmd.send(EngineCmd::RebindGlobalShortcuts);
            self.rebind_clicked_at = Some(std::time::Instant::now());
        }
        if waiting {
            ui.add_space(4.0);
            ui::activity(ui, "Opening your desktop's shortcut settings\u{2026}");
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }
    }

    fn hotkey_row(&mut self, ui: &mut egui::Ui, label: &str, action: HotkeyAction, label_w: f32) {
        row_label(ui, label, label_w);

        let capturing = self.hotkey_capture == Some(action);
        let current = match action {
            HotkeyAction::Recenter => self.hotkey_recenter,
            HotkeyAction::Toggle => self.hotkey_toggle,
        };

        let text = if capturing {
            "Press a key\u{2026} (Esc to cancel)".to_string()
        } else {
            current
                .map(|k| k.name().to_string())
                .unwrap_or_else(|| "Unbound".into())
        };
        let kind = if capturing {
            ui::ButtonKind::GhostActive
        } else {
            ui::ButtonKind::Normal
        };

        if ui::tool_button(ui, "", &text, kind).clicked() {
            // Toggle capture mode; the key itself is grabbed in `handle_hotkeys`.
            if capturing {
                self.hotkey_capture = None;
            } else {
                self.hotkey_capture = Some(action);
                // Capture hardening: keys already down now (a HOTAS maintained
                // switch parked on one) must never be captured.
                self.hotkey_capture_started = std::time::Instant::now();
                self.hotkey_capture_initial = ui.input(|i| i.keys_down.clone());
            }
        }

        if ui::tool_button_enabled(ui, current.is_some(), "", "Clear", ui::ButtonKind::Normal)
            .clicked()
        {
            match action {
                HotkeyAction::Recenter => self.hotkey_recenter = None,
                HotkeyAction::Toggle => self.hotkey_toggle = None,
            }
            self.hotkey_capture = None;
            self.save_hotkeys();
        }
        ui.end_row();
    }

    // ── Tier 3: in-sim commands and the command socket ───────────────────────

    fn ui_keybinds_scripting(&mut self, ui: &mut egui::Ui) {
        ui::group(ui, None, |ui| {
            Self::tier_header(
                ui,
                "Commands and scripting",
                "Scripting",
                ui::Tone::Idle,
                "For X-Plane's own binding screen, Stream Decks and scripts.",
            );

            ui::disclosure(ui, "xp_cmds", "X-Plane 12 commands", |ui| {
                ui::note(
                    ui,
                    "In X-Plane: Settings \u{2192} Keyboard or Joystick, then search \"headtrack-rs\".",
                    colors::DIM,
                );
                ui.add_space(4.0);
                copyable_command_grid(
                    ui,
                    "xp_cmd_grid",
                    &[
                        ("headtrack-rs/recenter", "Recenter head tracking"),
                        ("headtrack-rs/seat_forward", "Move pilot seat forward"),
                        ("headtrack-rs/seat_back", "Move pilot seat aft"),
                        ("headtrack-rs/seat_up", "Move pilot seat up"),
                        ("headtrack-rs/seat_down", "Move pilot seat down"),
                        ("headtrack-rs/seat_left", "Move pilot seat left"),
                        ("headtrack-rs/seat_right", "Move pilot seat right"),
                        ("headtrack-rs/seat_reset", "Reset pilot seat offset"),
                    ],
                );
            });

            ui::disclosure(ui, "socket_cmds", "Command socket", |ui| {
                ui::note(ui, "The same actions as one-liners; works while a sim has focus.", colors::DIM);
                ui.add_space(4.0);
                copyable_command_grid(
                    ui,
                    "socket_cmd_grid",
                    &[
                        ("recenter", "Recenter head tracking"),
                        ("msfs-enabled toggle", "Release or re-grab the MSFS camera"),
                        ("msfs-seat z 0.01", "Nudge the MSFS seat (x/y/z, \u{00b1} m)"),
                        ("msfs-seat reset", "Reset this aircraft's MSFS seat"),
                    ],
                );
                ui.add_space(6.0);
                let shell = "printf '<command>' | socat - UNIX-CONNECT:$XDG_RUNTIME_DIR/headtrack-cmd.sock";
                ui.horizontal(|ui| {
                    if ui::action_icon(ui, ph::COPY, "Copy the shell form").clicked() {
                        ui.ctx().copy_text(shell.to_string());
                    }
                    // One line: a command is data, so it truncates rather than
                    // wraps, and the whole of it is on hover and on the clipboard.
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(shell)
                                .monospace()
                                .size(colors::TEXT_SM)
                                .color(colors::DIM),
                        )
                        .truncate(),
                    )
                    .on_hover_text(shell);
                });
            });
        });
    }
}

/// Shorten a controller binding label to the button name.
///
/// Engine labels are `"<device name> · <BTN_NAME>"`. The device name is long
/// enough to set the width of a whole grid column, and is identical for every
/// binding on the same stick, so it carries no information in the list — it
/// goes in the tooltip instead.
fn short_binding(label: &str) -> String {
    label
        .rsplit_once(" \u{00b7} ")
        .map(|(_, button)| button.to_string())
        .unwrap_or_else(|| label.to_string())
}

/// Command reference rows, each with a copy button — these are meant to be
/// pasted, and used to be plain uncopyable text.
fn copyable_command_grid(ui: &mut egui::Ui, id: &str, rows: &[(&str, &str)]) {
    egui::Grid::new(id)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            for (cmd, desc) in rows {
                if ui::action_icon(ui, ph::COPY, "Copy to clipboard").clicked() {
                    ui.ctx().copy_text((*cmd).to_string());
                }
                ui.label(
                    egui::RichText::new(*cmd)
                        .monospace()
                        .size(colors::TEXT_SM)
                        .color(colors::TEXT_HI),
                );
                ui.label(egui::RichText::new(*desc).size(colors::TEXT_BODY).color(colors::DIM));
                ui.end_row();
            }
        });
}

/// The action name at the head of a binding row, in a column exactly `width`
/// wide so every binding grid on the page shares it.
fn row_label(ui: &mut egui::Ui, text: &str, width: f32) {
    ui.allocate_ui_with_layout(
        egui::vec2(width, 0.0),
        egui::Layout::left_to_right(egui::Align::Center),
        |ui| {
            ui.set_min_width(width);
            ui.add(
                egui::Label::new(egui::RichText::new(text).size(colors::TEXT_BODY).color(colors::TEXT))
                    .truncate(),
            );
        },
    );
}
