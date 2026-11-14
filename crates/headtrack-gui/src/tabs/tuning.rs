use headtrack_core::pipeline::stage_names;
use crate::{colors, ui, HeadtrackApp, TuningGroup, AXIS_LABELS, AXIS_NAMES, AXIS_UNITS};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_engine::EngineCmd;

impl HeadtrackApp {
    // ── Tuning ────────────────────────────────────────────────────────────────

    pub(crate) fn ui_tuning(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        ui::page_header(ui, "Tuning", |_ui| {});

        self.ui_tuning_group_selector(ui);
        ui.add_space(ui::ROW_GAP);

        let group = self.tuning_group;

        // Two rows on the shared page grid, so the gutter lines up vertically
        // and all four groups square off against the same left and right edges.
        //
        //   Active axes │ Response
        //   Feel        │ Head position
        //
        // `Even`: both sides are peers — controls on the left, their readout on
        // the right.
        ui::two_columns(ui, "tuning-axes", ui::Split::Even, |c| {
            c.group(Some("Active axes"), |ui| {
                ui::note(ui, "An axis turned off is frozen; the sim stops receiving it.", colors::DIM);
                ui.add_space(8.0);
                self.ui_tuning_axis_toggles(ui);
            });
            c.group(Some("Response"), |ui| {
                self.ui_tuning_comparison(ui, pose_vals, group);
            });
        });

        ui.add_space(ui::ROW_GAP);

        ui::two_columns(ui, "tuning-feel", ui::Split::Even, |c| {
            c.group(Some("Feel"), |ui| {
                // The Tobii is 3DOF: X/Y/Z are held at zero, so translation
                // tuning would edit values that change nothing.
                let editable = group.is_rotation() || self.source_kind.provides_translation();
                if !editable {
                    ui::note(ui, "Translation not available with Tobii.", colors::WARN);
                    ui.add_space(6.0);
                }
                ui.add_enabled_ui(editable, |ui| self.ui_tuning_sliders(ui, group));
                ui.add_space(10.0);
                self.ui_tuning_buttons(ui, group, editable);
            });
            c.group(Some("Head position"), |ui| {
                let w = ui.available_width();
                let z = self.source_kind.provides_translation().then_some(pose_vals[5]);
                ui::draw_pose_bubble(ui, pose_vals[0], pose_vals[1], pose_vals[2], z, w, 190.0);
            });
        });
        ui.add_space(8.0);
    }

    fn ui_tuning_group_selector(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            for (group, label) in [
                (TuningGroup::Look, "Look \u{00b7} Yaw / Pitch / Roll"),
                (TuningGroup::Move, "Move \u{00b7} X / Y / Z"),
            ] {
                if ui::segment(ui, label, self.tuning_group == group).clicked() {
                    self.tuning_group = group;
                }
            }
        });
    }

    /// The six axis-mask toggles, promoted out of the old Dev tab.
    ///
    /// These write the same `axis-mask` stage that `toggle_tracking` drives, so
    /// the effective mask stays `tracking_enabled && axis_active[i]` from both
    /// directions.
    fn ui_tuning_axis_toggles(&mut self, ui: &mut egui::Ui) {
        for row in [[0usize, 1, 2], [3, 4, 5]] {
            ui.horizontal(|ui| {
                for i in row {
                    let enabled = i < 3 || self.source_kind.provides_translation();
                    let clicked = ui
                        .add_enabled_ui(enabled, |ui| {
                            ui::toggle_chip(ui, AXIS_LABELS[i], self.axis_active[i], ui::axis_color(i))
                        })
                        .inner
                        .clicked();
                    if clicked {
                        self.axis_active[i] = !self.axis_active[i];
                        let on = self.tracking_enabled && self.axis_active[i];
                        self.send_set(stage_names::AXIS_MASK, AXIS_NAMES[i], if on { 1.0 } else { 0.0 });
                    }
                }
            });
            ui.add_space(4.0);
        }
        if !self.tracking_enabled {
            ui::note(ui, "Tracking is paused; axes resume as set here.", colors::WARN);
        }
    }

    /// Head input versus what the response curve makes of it.
    fn ui_tuning_comparison(
        &self,
        ui: &mut egui::Ui,
        pose_vals: &[f32; 6],
        group: TuningGroup,
    ) {
        egui::Grid::new(match group {
            TuningGroup::Look => "look_telem",
            TuningGroup::Move => "move_telem",
        })
        .spacing([18.0, 5.0])
        .show(ui, |ui| {
            ui.label("");
            for &i in group.axes() {
                ui.label(
                    egui::RichText::new(AXIS_LABELS[i])
                        .monospace()
                        .size(colors::TEXT_SM)
                        .color(ui::axis_color(i)),
                );
            }
            ui.end_row();

            ui.label(egui::RichText::new("Head").size(colors::TEXT_BODY).color(colors::DIM));
            for &i in group.axes() {
                ui.label(
                    egui::RichText::new(format!("{:.1}{}", pose_vals[i].abs(), AXIS_UNITS[i]))
                        .font(egui::FontId::new(colors::TEXT_BODY, colors::mono_bold()))
                        .color(colors::TEXT_HI),
                );
            }
            ui.end_row();

            // Deliberately "Curve out", not "Camera": this is the response
            // curve alone. Accela, the deadzone and cross-axis compensation all
            // act after it, so calling the number a camera angle overstates it.
            ui.label(egui::RichText::new("Curve out").size(colors::TEXT_BODY).color(colors::DIM));
            for &i in group.axes() {
                let head = pose_vals[i].abs();
                let out = self.profile.curve[i].sensitivity * head.powf(self.profile.curve[i].exponent);
                ui.label(
                    egui::RichText::new(format!("{:.1}{}", out, AXIS_UNITS[i]))
                        .font(egui::FontId::new(colors::TEXT_BODY, colors::mono_bold()))
                        .color(ui::axis_color(i)),
                );
            }
            ui.end_row();
        })
        .response
        .on_hover_text(
            "Response curve output only — smoothing, deadzone and cross-axis compensation \
             are applied after this stage.",
        );
    }

    fn ui_tuning_sliders(&mut self, ui: &mut egui::Ui, group: TuningGroup) {
        let rep = group.representative();
        let mut gain = self.profile.curve[rep].sensitivity;
        let mut exponent = self.profile.curve[rep].exponent;
        let mut smoothing = self.profile.accela[rep].smoothing;
        let mut deadzone = self.profile.deadzone[rep];

        // Deadzone is the one grouped control with a real unit, which is why
        // Look/Move are split by unit rather than by intuition.
        let (dz_max, dz_label) = if group.is_rotation() {
            (5.0_f32, "Ignore small movements (°)")
        } else {
            (20.0_f32, "Ignore small movements (mm)")
        };

        // One cell per row: the Feel column shares its row with the pose
        // bubble, and at the minimum window size two cells side by side left
        // each slider a stub and pushed the longest label past the column.
        ui::param_row(ui, 1, |ui| {
            if ui::param_cell(ui, "Responsiveness", &mut gain, 0.1..=3.0, 2) {
                self.set_group(group, |p, i| p.curve[i].sensitivity = gain);
                self.push_group(group, stage_names::RESPONSE_CURVE, "sensitivity", gain);
            }
        });
        ui::param_row(ui, 1, |ui| {
            if ui::param_cell(ui, "Centre stability", &mut exponent, 1.0..=3.0, 2) {
                self.set_group(group, |p, i| p.curve[i].exponent = exponent);
                self.push_group(group, stage_names::RESPONSE_CURVE, "exponent", exponent);
            }
        });
        ui::param_row(ui, 1, |ui| {
            if ui::param_cell(ui, "Smoothness", &mut smoothing, 0.01..=10.0, 2) {
                self.set_group(group, |p, i| p.accela[i].smoothing = smoothing);
                self.push_group(group, stage_names::ACCELA, "smoothing", smoothing);
            }
        });
        ui::param_row(ui, 1, |ui| {
            if ui::param_cell(ui, dz_label, &mut deadzone, 0.0..=dz_max, 1) {
                self.set_group(group, |p, i| p.deadzone[i] = deadzone);
                // The deadzone stage keys on the bare axis name, not `axis.param`.
                for &i in group.axes() {
                    self.send_set(stage_names::DEADZONE, AXIS_NAMES[i], deadzone);
                }
                self.profile_dirty = true;
            }
        });
    }

    /// `editable` false greys Reset (Tobii translation) but keeps Recenter.
    fn ui_tuning_buttons(&mut self, ui: &mut egui::Ui, group: TuningGroup, editable: bool) {
        ui.horizontal(|ui| {
            if ui::tool_button_enabled(ui, editable, ph::ARROW_CLOCKWISE, "Reset group", ui::ButtonKind::Normal)
                .on_hover_text("Restore this group's four sliders to their built-in defaults")
                .clicked()
            {
                let defaults = crate::tabs::tobii::builtin_profile(self.source_kind);
                for &i in group.axes() {
                    self.profile.curve[i].sensitivity = defaults.curve[i].sensitivity;
                    self.profile.curve[i].exponent = defaults.curve[i].exponent;
                    self.profile.accela[i].smoothing = defaults.accela[i].smoothing;
                    self.profile.deadzone[i] = defaults.deadzone[i];
                    let axis = AXIS_NAMES[i];
                    self.send_set(
                        stage_names::RESPONSE_CURVE,
                        &format!("{axis}.sensitivity"),
                        defaults.curve[i].sensitivity,
                    );
                    self.send_set(
                        stage_names::RESPONSE_CURVE,
                        &format!("{axis}.exponent"),
                        defaults.curve[i].exponent,
                    );
                    self.send_set(
                        stage_names::ACCELA,
                        &format!("{axis}.smoothing"),
                        defaults.accela[i].smoothing,
                    );
                    self.send_set(stage_names::DEADZONE, axis, defaults.deadzone[i]);
                }
                self.profile_dirty = true;
            }
            if ui::tool_button(ui, ph::CROSSHAIR_SIMPLE, "Recenter", ui::ButtonKind::Normal)
                .clicked()
            {
                self.cmd.send(EngineCmd::Recenter);
            }
        });
    }

    /// Apply an edit to every axis in `group` and mark the profile dirty.
    fn set_group(&mut self, group: TuningGroup, mut edit: impl FnMut(&mut headtrack_core::config::ProfileConfig, usize)) {
        for &i in group.axes() {
            edit(&mut self.profile, i);
        }
        self.profile_dirty = true;
    }

    /// Push one `{axis}.{param}` value to `stage` for every axis in `group`.
    fn push_group(&self, group: TuningGroup, stage: &str, param: &str, value: f32) {
        for &i in group.axes() {
            self.send_set(stage, &format!("{}.{param}", AXIS_NAMES[i]), value);
        }
    }
}
