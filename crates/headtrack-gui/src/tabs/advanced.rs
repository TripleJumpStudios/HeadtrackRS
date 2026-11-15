use crate::{axis_max, colors, ui, HeadtrackApp, AXIS_LABELS, AXIS_NAMES, AXIS_UNITS};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::pipeline::stage_names;
use headtrack_engine::{EngineCmd, RecordingStatus};
use std::sync::atomic::Ordering;

/// Per-axis defaults used when the recenter-range override is first enabled.
/// Matches the shipping profile's `[center] recenter_range`.
const RECENTER_RANGE_DEFAULT: [f32; 6] = [12.0, 12.0, 4.0, 10.0, 10.0, 60.0];

impl HeadtrackApp {
    // ── Advanced ──────────────────────────────────────────────────────────────

    pub(crate) fn ui_advanced(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        ui::page_header(ui, "Advanced", |_ui| {});
        // Safety line, not a box: it is the page's one consequence sentence.
        let mut job = ui::icon_text(
            ph::WARNING,
            "Changes reach every simulator, and extreme values can cause motion sickness.",
            colors::WARN,
        );
        job.wrap.max_width = ui.available_width();
        ui.label(job);

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Live axes"), |ui| {
            self.ui_advanced_live_axes(ui, pose_vals);
        });

        ui.add_space(ui::ROW_GAP);
        self.ui_advanced_axis_editor(ui, pose_vals);

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Cross-axis compensation"), |ui| {
            self.ui_advanced_cross_axis(ui);
        });

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Centre stage"), |ui| {
            self.ui_advanced_center_stage(ui);
        });

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Diagnostics recorder"), |ui| {
            self.ui_advanced_recorder(ui);
        });
        ui.add_space(8.0);
    }

    fn ui_advanced_live_axes(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        for i in 0..6 {
            let selected = self.dev_selected_axis == i;
            let active = self.axis_active[i];
            let value = pose_vals[i];

            // Reserve paint slots *before* the row is drawn. egui paints in
            // call order, so filling the highlight afterwards would cover the
            // row's own label, value and bar.
            let bg_slot = ui.painter().add(egui::Shape::Noop);
            let accent_slot = ui.painter().add(egui::Shape::Noop);

            let row = ui.horizontal(|ui| {
                ui.add_space(6.0);
                ui.add_sized(
                    [50.0, 20.0],
                    egui::Label::new(egui::RichText::new(AXIS_LABELS[i]).monospace().color(
                        if active {
                            ui::axis_color(i)
                        } else {
                            colors::DIMMER
                        },
                    ))
                    .selectable(false),
                );
                ui.add_sized(
                    [76.0, 20.0],
                    egui::Label::new(
                        egui::RichText::new(format!(
                            "{:>6}{}",
                            ui::signed(value, 1),
                            AXIS_UNITS[i]
                        ))
                        .monospace()
                        .color(colors::TEXT),
                    )
                    .selectable(false),
                );
                let bar_w = (ui.available_width() - 40.0).clamp(80.0, 260.0);
                ui::draw_bar(ui, value, axis_max(i), active, bar_w);
                if !active {
                    ui.label(
                        egui::RichText::new("off")
                            .size(colors::TEXT_SM)
                            .color(colors::DIMMER),
                    )
                    .on_hover_text("Disabled in the Tuning tab");
                }
            });

            // Whole-row click target; the highlight is painted behind it.
            let rect = row.response.rect;
            let resp = ui
                .interact(
                    rect,
                    egui::Id::new(("adv_axis_row", i)),
                    egui::Sense::click(),
                )
                .on_hover_text("Click to edit this axis");
            if resp.clicked() {
                self.dev_selected_axis = i;
            }
            if selected || resp.hovered() {
                ui.painter().set(
                    bg_slot,
                    egui::Shape::rect_filled(
                        rect.expand2(egui::vec2(4.0, 1.0)),
                        6.0,
                        if selected {
                            colors::PANEL_HI
                        } else {
                            colors::PANEL_HI.gamma_multiply(0.5)
                        },
                    ),
                );
            }
            if selected {
                ui.painter().set(
                    accent_slot,
                    egui::Shape::rect_filled(
                        egui::Rect::from_min_max(
                            egui::pos2(rect.left() - 4.0, rect.top() + 1.0),
                            egui::pos2(rect.left() - 1.0, rect.bottom() - 1.0),
                        ),
                        2.0,
                        colors::ORANGE,
                    ),
                );
            }
            ui.add_space(2.0);
        }
    }

    fn ui_advanced_axis_editor(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        let i = self.dev_selected_axis;
        let is_rotation = i < 3;
        let label = format!("Axis editor — {}", AXIS_LABELS[i]);

        ui::group(ui, Some(&label), |ui| {
            let width = ui.available_width().min(560.0);
            ui::draw_curve_editor(
                ui,
                &[(
                    self.profile.curve[i].sensitivity,
                    self.profile.curve[i].exponent,
                    pose_vals[i],
                    ui::axis_color(i),
                )],
                is_rotation,
                width,
                150.0,
            );

            ui.add_space(8.0);
            ui::subheading(ui, "Response curve");
            ui.add_space(4.0);
            ui::param_row(ui, 3, |ui| {
                if ui::param_cell(
                    ui,
                    "Responsiveness (gain)",
                    &mut self.profile.curve[i].sensitivity,
                    0.1..=5.0,
                    2,
                ) {
                    let v = self.profile.curve[i].sensitivity;
                    self.send_set(
                        stage_names::RESPONSE_CURVE,
                        &format!("{}.sensitivity", AXIS_NAMES[i]),
                        v,
                    );
                    self.profile_dirty = true;
                }
                if ui::param_cell(
                    ui,
                    "Centre stability (exp)",
                    &mut self.profile.curve[i].exponent,
                    0.3..=3.0,
                    2,
                ) {
                    let v = self.profile.curve[i].exponent;
                    self.send_set(
                        stage_names::RESPONSE_CURVE,
                        &format!("{}.exponent", AXIS_NAMES[i]),
                        v,
                    );
                    self.profile_dirty = true;
                }
            });

            ui.add_space(10.0);
            ui::subheading(ui, "Accela filter");
            ui.add_space(4.0);
            ui::param_row(ui, 3, |ui| {
                // Accela's own displacement deadband — distinct from the
                // DeadzoneStage cell below, which is centred on neutral.
                if ui::param_cell(
                    ui,
                    "Hold zone",
                    &mut self.profile.accela[i].deadzone,
                    0.0..=5.0,
                    2,
                ) {
                    let v = self.profile.accela[i].deadzone;
                    self.send_set(
                        stage_names::ACCELA,
                        &format!("{}.deadzone", AXIS_NAMES[i]),
                        v,
                    );
                    self.profile_dirty = true;
                }
                if ui::param_cell(
                    ui,
                    "Smoothing",
                    &mut self.profile.accela[i].smoothing,
                    0.01..=10.0,
                    2,
                ) {
                    let v = self.profile.accela[i].smoothing;
                    self.send_set(
                        stage_names::ACCELA,
                        &format!("{}.smoothing", AXIS_NAMES[i]),
                        v,
                    );
                    self.profile_dirty = true;
                }
                if ui::param_cell(
                    ui,
                    "Curve",
                    &mut self.profile.accela[i].exponent,
                    0.5..=3.0,
                    2,
                ) {
                    let v = self.profile.accela[i].exponent;
                    self.send_set(
                        stage_names::ACCELA,
                        &format!("{}.exponent", AXIS_NAMES[i]),
                        v,
                    );
                    self.profile_dirty = true;
                }
            });

            ui.add_space(10.0);
            ui::subheading(ui, "Limits");
            ui.add_space(4.0);
            ui::param_row(ui, 3, |ui| {
                let dz_max = if is_rotation { 5.0_f32 } else { 20.0 };
                let dz_label = if is_rotation {
                    "Deadzone (°)"
                } else {
                    "Deadzone (mm)"
                };
                if ui::param_cell(ui, dz_label, &mut self.profile.deadzone[i], 0.0..=dz_max, 1) {
                    let v = self.profile.deadzone[i];
                    self.send_set(stage_names::DEADZONE, AXIS_NAMES[i], v);
                    self.profile_dirty = true;
                }
                // Rate limiter. Saved in every profile and applied by the
                // engine since Phase 2, but never editable in the GUI until
                // now — the shipping profile's hand-tuned pitch = 80 °/s was
                // reachable only by editing the TOML by hand.
                let slew_label = if is_rotation {
                    "Slew limit (°/s)"
                } else {
                    "Slew limit (mm/s)"
                };
                if ui::param_cell(
                    ui,
                    slew_label,
                    &mut self.profile.slew_limit[i],
                    10.0..=1000.0,
                    0,
                ) {
                    let v = self.profile.slew_limit[i];
                    self.send_set(stage_names::SLEW_LIMIT, AXIS_NAMES[i], v);
                    self.profile_dirty = true;
                }
            });
        });
    }

    /// All twelve cross-axis coefficients, grouped by the axis they read from.
    ///
    /// `pitch_to_yaw` and `pitch_to_z` are new here: both were already carried
    /// in `ProfileConfig` and pushed by `profile_params`, but had no control.
    fn ui_advanced_cross_axis(&mut self, ui: &mut egui::Ui) {
        ui::note(
            ui,
            "Cancels bleed between axes, such as a yaw that drags pitch with it.",
            colors::DIM,
        );
        ui.add_space(6.0);

        let mut edits: Vec<(&'static str, f32)> = Vec::new();
        {
            let x = &mut self.profile.cross_axis;
            // (label, param id, value, symmetric range limit)
            let mut from_yaw: [(&str, &'static str, &mut f32, f32); 5] = [
                ("\u{2192} Pitch", "yaw_to_pitch", &mut x.yaw_to_pitch, 1.0),
                ("\u{2192} Roll", "yaw_to_roll", &mut x.yaw_to_roll, 1.0),
                ("\u{2192} X", "yaw_to_x", &mut x.yaw_to_x, 1.0),
                ("\u{2192} Y", "yaw_to_y", &mut x.yaw_to_y, 1.0),
                ("\u{2192} Z", "yaw_to_z", &mut x.yaw_to_z, 1.0),
            ];
            let mut from_pitch: [(&str, &'static str, &mut f32, f32); 3] = [
                ("\u{2192} Yaw", "pitch_to_yaw", &mut x.pitch_to_yaw, 1.0),
                ("\u{2192} Y", "pitch_to_y", &mut x.pitch_to_y, 2.0),
                // Verified at −1.5, so a ±1 range would silently clip it.
                ("\u{2192} Z", "pitch_to_z", &mut x.pitch_to_z, 2.0),
            ];
            let mut from_roll: [(&str, &'static str, &mut f32, f32); 2] = [
                ("\u{2192} X", "roll_to_x", &mut x.roll_to_x, 3.0),
                ("\u{2192} Z", "roll_to_z", &mut x.roll_to_z, 1.0),
            ];
            let mut from_z: [(&str, &'static str, &mut f32, f32); 2] = [
                ("\u{2192} Pitch", "z_to_pitch", &mut x.z_to_pitch, 1.0),
                ("\u{2192} Y", "z_to_y", &mut x.z_to_y, 1.0),
            ];

            for (group_label, rows) in [
                ("From yaw", &mut from_yaw[..]),
                ("From pitch", &mut from_pitch[..]),
                ("From roll", &mut from_roll[..]),
                ("From Z", &mut from_z[..]),
            ] {
                ui::subheading(ui, group_label);
                ui.add_space(3.0);
                for chunk in rows.chunks_mut(3) {
                    ui::param_row(ui, 3, |ui| {
                        for (label, param, value, limit) in chunk.iter_mut() {
                            if ui::param_cell(ui, label, value, -*limit..=*limit, 2) {
                                edits.push((param, **value));
                            }
                        }
                    });
                }
                ui.add_space(5.0);
            }
        }

        for (param, value) in edits {
            self.send_set(stage_names::CROSS_AXIS, param, value);
            self.profile_dirty = true;
        }
    }

    fn ui_advanced_center_stage(&mut self, ui: &mut egui::Ui) {
        ui::note(
            ui,
            "Drifts the neutral point back to centre while you hold still.",
            colors::DIM,
        );
        ui.add_space(6.0);

        ui::param_row(ui, 3, |ui| {
            if ui::param_cell(
                ui,
                "Drift correction (/s)",
                &mut self.profile.center.drift_rate,
                // Upper bound raised from 0.2 when the shipped default became
                // 0.2 (2026-08-17) — a slider that starts pinned at its own
                // maximum reads as broken and can only be turned down.
                0.0..=0.4,
                3,
            ) {
                let v = self.profile.center.drift_rate;
                self.send_set(stage_names::CENTER, "drift_rate", v);
                self.profile_dirty = true;
            }
            if ui::param_cell(
                ui,
                "Z drift multiplier",
                &mut self.profile.center.z_drift_mult,
                1.0..=10.0,
                1,
            ) {
                let v = self.profile.center.z_drift_mult;
                self.send_set(stage_names::CENTER, "z.drift_mult", v);
                self.profile_dirty = true;
            }
        });

        ui.add_space(10.0);

        // Recenter range: drift fades to zero this far from the baseline, so a
        // gaze held off-centre is never absorbed. Stored as an Option — None
        // leaves the stage's own constructor defaults in place, which is why
        // this is an on/off row rather than a plain set of sliders.
        let mut enabled = self.profile.center.recenter_range.is_some();
        // Two cells of the three-cell grid below, so the chip ends on a grid
        // line rather than at `kv_row`'s 660 px cap, mid-way through the third
        // cell (10a). The full row would put key and chip at opposite margins.
        let span = ui::param_span(ui, 3, 2);
        let toggle = ui
            .allocate_ui_with_layout(
                egui::vec2(span, 0.0),
                egui::Layout::top_down(egui::Align::Min),
                |ui| ui::toggle_row(ui, "Override recenter range", &mut enabled),
            )
            .inner;
        if toggle
            .on_hover_text(
                "Per-axis distance from the baseline at which auto-recentre drift fades to \
                 zero. Unchecked, the pipeline stage's own defaults apply.",
            )
            .changed()
        {
            self.profile.center.recenter_range = enabled.then_some(RECENTER_RANGE_DEFAULT);
            if let Some(ranges) = self.profile.center.recenter_range {
                for (i, axis) in AXIS_NAMES.iter().enumerate() {
                    self.send_set(
                        stage_names::CENTER,
                        &format!("{axis}.recenter_range"),
                        ranges[i],
                    );
                }
            }
            self.profile_dirty = true;
        }

        let mut edits: Vec<(usize, f32)> = Vec::new();
        if let Some(ref mut ranges) = self.profile.center.recenter_range {
            ui.add_space(4.0);
            for row in [[0usize, 1, 2], [3, 4, 5]] {
                ui::param_row(ui, 3, |ui| {
                    for i in row {
                        let max = if i < 3 { 45.0 } else { 200.0 };
                        let label = format!("{} ({})", AXIS_LABELS[i], AXIS_UNITS[i]);
                        if ui::param_cell(ui, &label, &mut ranges[i], 0.0..=max, 1) {
                            edits.push((i, ranges[i]));
                        }
                    }
                });
            }
        }
        for (i, value) in edits {
            self.send_set(
                stage_names::CENTER,
                &format!("{}.recenter_range", AXIS_NAMES[i]),
                value,
            );
            self.profile_dirty = true;
        }
    }

    // ── Diagnostics recorder ─────────────────────────────────────────────────

    fn ui_advanced_recorder(&mut self, ui: &mut egui::Ui) {
        // Privacy: say what is written. Pose numbers and parameter changes,
        // never a picture.
        ui::note(
            ui,
            "Records pose and parameter changes to CSV for tuning analysis and bug reports.",
            colors::DIM,
        );
        ui.add_space(2.0);
        ui::kv_row(ui, "Folder", |ui| {
            let room = ui.available_width();
            ui::path_label(ui, &self.diag_directory, room, colors::DIM);
        })
        .on_hover_text("Change this in Settings \u{2192} Application");
        ui.add_space(4.0);

        let status = RecordingStatus::from_u8(self.state.recording_status.load(Ordering::Relaxed));
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            self.ui_advanced_recorder_controls(ui, status);
        });
    }

    fn ui_advanced_recorder_controls(&mut self, ui: &mut egui::Ui, status: RecordingStatus) {
        match status {
            RecordingStatus::Idle => {
                if ui::tool_button(ui, ph::RECORD, "Record", ui::ButtonKind::Normal).clicked() {
                    let path = diag_default_path(&self.diag_directory);
                    self.diag_recording_path = path.clone();
                    self.diag_recording_start = Some(std::time::Instant::now());
                    self.cmd.send(EngineCmd::StartRecording {
                        path,
                        size_limit_mb: self.diag_size_limit_mb,
                    });
                }
                // The app's single size-limit control. Settings used to carry a
                // second copy of this same field.
                ui.label(
                    egui::RichText::new("Max size")
                        .size(colors::TEXT_BODY)
                        .color(colors::DIM),
                );
                ui.add(
                    egui::DragValue::new(&mut self.diag_size_limit_mb)
                        .range(1..=500)
                        .suffix(" MB")
                        .speed(1.0),
                );
            }

            RecordingStatus::Active => {
                let elapsed = self
                    .diag_recording_start
                    .map(|s| s.elapsed().as_secs())
                    .unwrap_or(0);
                let bytes = self.state.recording_bytes.load(Ordering::Relaxed);
                let limit_bytes = self.diag_size_limit_mb as u64 * 1024 * 1024;
                let frac = (bytes as f32 / limit_bytes as f32).min(1.0);

                if ui::tool_button(ui, ph::RECORD, "Stop", ui::ButtonKind::Primary).clicked() {
                    self.diag_recording_start = None;
                    self.cmd.send(EngineCmd::StopRecording);
                }
                ui.add(
                    egui::ProgressBar::new(frac)
                        .desired_width(130.0)
                        .text(format!("{:02}:{:02}", elapsed / 60, elapsed % 60))
                        .fill(colors::ORANGE_DIM),
                );
                let path = self.diag_recording_path.display().to_string();
                let room = ui.available_width();
                ui::path_label(ui, &path, room, colors::DIM);
            }

            RecordingStatus::LimitReached => {
                self.diag_recording_start = None;
                ui.label(ui::icon_text(
                    ph::WARNING,
                    "Size limit reached",
                    colors::WARN,
                ));
                if ui::tool_button(ui, "", "Dismiss", ui::ButtonKind::Normal).clicked() {
                    self.cmd.send(EngineCmd::StopRecording);
                }
            }

            RecordingStatus::Error => {
                self.diag_recording_start = None;
                let err = self
                    .state
                    .recording_error
                    .lock()
                    .ok()
                    .and_then(|g| g.clone())
                    .unwrap_or_else(|| "unknown error".to_string());
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.label(ui::icon_text(ph::WARNING, "Recording failed", colors::ERR));
                        if ui::tool_button(ui, "", "Dismiss", ui::ButtonKind::Normal).clicked() {
                            self.cmd.send(EngineCmd::StopRecording);
                        }
                    });
                    ui.label(
                        egui::RichText::new(err)
                            .monospace()
                            .size(colors::TEXT_SM)
                            .color(colors::DIM),
                    );
                });
            }
        }
    }
}

pub(crate) fn diag_default_path(dir: &str) -> std::path::PathBuf {
    use std::time::SystemTime;
    let ts = SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    std::path::PathBuf::from(dir).join(format!("headtrack_record_{ts}.csv"))
}
