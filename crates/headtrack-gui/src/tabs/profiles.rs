use crate::{colors, ui, HeadtrackApp};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::config::ProfileConfig;

impl HeadtrackApp {
    // ── Profiles ──────────────────────────────────────────────────────────────

    pub(crate) fn ui_profiles(&mut self, ui: &mut egui::Ui) {
        ui::page_header(ui, "Profiles", |ui| {
            if self.profile_dirty {
                ui::state(ui, "Unsaved changes", ui::Tone::Warn);
            }
        });

        ui::group(ui, Some("Active profile"), |ui| {
            self.ui_profiles_active(ui);
        });

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Saved profiles"), |ui| {
            self.ui_profiles_saved(ui);
        });

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Process auto-switch"), |ui| {
            // Not "Phase 4": a roadmap label means nothing to a user (5a).
            ui::state(ui, "Planned", ui::Tone::Idle);
            ui.add_space(4.0);
            ui::note(
                ui,
                "Planned: load a profile automatically when a game is detected.",
                colors::DIM,
            );
        });
        ui.add_space(8.0);
    }

    fn ui_profiles_active(&mut self, ui: &mut egui::Ui) {
        ui::kv_row(ui, "Name", |ui| {
            let width = ui.available_width().clamp(140.0, 300.0);
            ui::text_field(ui, &mut self.profile_name, "Profile name\u{2026}", width);
        });

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            if ui::tool_button(ui, ph::FLOPPY_DISK, "Save profile", ui::ButtonKind::Primary)
                .clicked()
            {
                self.save_active_profile();
            }
            if ui::tool_button(
                ui,
                ph::ARROW_CLOCKWISE,
                "Reset to defaults",
                ui::ButtonKind::Normal,
            )
            .clicked()
            {
                let defaults = crate::tabs::tobii::builtin_profile(self.source_kind);
                self.apply_profile(&defaults);
                self.profile_status = "Reset to defaults".into();
            }
        });

        if !self.profile_status.is_empty() {
            ui.add_space(6.0);
            ui::note(ui, &self.profile_status, colors::DIM);
        }
    }

    fn save_active_profile(&mut self) {
        let cfg = self.to_profile();
        let filename = cfg
            .name
            .replace(|c: char| !c.is_alphanumeric() && c != '-' && c != '_', "_");
        let path = ProfileConfig::profiles_dir().join(format!("{filename}.toml"));
        match cfg.save(&path) {
            Ok(()) => {
                let _ = cfg.save(&ProfileConfig::active_config_path_for(self.source_kind));
                self.profile_status = format!("Saved to {}", path.display());
                self.profiles_list = ProfileConfig::list_profiles();
                self.profile_dirty = false;
            }
            Err(e) => self.profile_status = format!("Save failed: {e}"),
        }
    }

    fn ui_profiles_saved(&mut self, ui: &mut egui::Ui) {
        if ui::tool_button(ui, ph::ARROW_CLOCKWISE, "Refresh", ui::ButtonKind::Ghost).clicked() {
            self.profiles_list = ProfileConfig::list_profiles();
        }
        ui.add_space(6.0);

        if self.profiles_list.is_empty() {
            ui::note(ui, "No saved profiles.", colors::DIM);
            return;
        }

        let mut load_path: Option<std::path::PathBuf> = None;
        let mut delete_path: Option<std::path::PathBuf> = None;
        let profiles = self.profiles_list.clone();

        for (name, path) in &profiles {
            let is_active = *name == self.profile_name;
            let confirming = self.profile_pending_delete.as_deref() == Some(path.as_path());

            // One line per profile, no box: the name truncates so the action
            // cluster on the right always has its room. A right-to-left cluster
            // does not reserve its own width, so the name gets an explicit one.
            let row_w = ui.available_width().min(colors::ROW_MAX_WIDTH);
            let actions_w = if confirming { 350.0 } else { 150.0 };
            let active_w = if is_active { 80.0 } else { 0.0 };
            let name_w = (row_w - actions_w - active_w).max(60.0);

            let bg = ui.painter().add(egui::Shape::Noop);
            let row = ui.allocate_ui_with_layout(
                egui::vec2(row_w, colors::CONTROL_H),
                egui::Layout::left_to_right(egui::Align::Center),
                |ui| {
                    ui.set_max_width(row_w);
                    ui.add_space(8.0);
                    ui.allocate_ui_with_layout(
                        egui::vec2(name_w, colors::CONTROL_H),
                        egui::Layout::left_to_right(egui::Align::Center),
                        |ui| {
                            ui.set_max_width(name_w);
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(name)
                                        .font(egui::FontId::new(colors::TEXT_MD, colors::semibold()))
                                        .color(if is_active { colors::ORANGE } else { colors::TEXT }),
                                )
                                .truncate(),
                            )
                            .on_hover_text(name);
                        },
                    );
                    if is_active {
                        ui::state(ui, "Active", ui::Tone::Accent);
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if confirming {
                            // Deleting used to happen on the first click, with
                            // no way back.
                            if ui::tool_button(ui, ph::TRASH, "Delete", ui::ButtonKind::Danger)
                                .clicked()
                            {
                                delete_path = Some(path.clone());
                            }
                            if ui::tool_button(ui, "", "Cancel", ui::ButtonKind::Normal).clicked() {
                                self.profile_pending_delete = None;
                            }
                            ui.label(
                                egui::RichText::new("Delete this profile?")
                                    .size(colors::TEXT_BODY)
                                    .color(colors::WARN),
                            );
                        } else {
                            if ui::action_icon(ui, ph::TRASH, "Delete this profile").clicked() {
                                self.profile_pending_delete = Some(path.clone());
                            }
                            if ui::tool_button(ui, ph::FOLDER_OPEN, "Load", ui::ButtonKind::Normal)
                                .clicked()
                            {
                                load_path = Some(path.clone());
                            }
                        }
                    });
                },
            );
            // The active row is raised, as the active nav row is.
            if is_active {
                ui.painter().set(
                    bg,
                    egui::Shape::rect_filled(
                        row.response.rect,
                        egui::CornerRadius::same(colors::CONTROL_RADIUS),
                        colors::PANEL_HI,
                    ),
                );
            }
            ui.add_space(4.0);
        }

        if let Some(path) = load_path {
            match ProfileConfig::load(&path) {
                Ok(cfg) => {
                    self.apply_profile(&cfg);
                    let _ = cfg.save(&ProfileConfig::active_config_path_for(self.source_kind));
                    self.profile_status = format!("Loaded: {}", cfg.name);
                    self.profile_pending_delete = None;
                }
                Err(e) => self.profile_status = format!("Load failed: {e}"),
            }
        }
        if let Some(path) = delete_path {
            match std::fs::remove_file(&path) {
                Ok(()) => self.profile_status = "Profile deleted".into(),
                Err(e) => self.profile_status = format!("Delete failed: {e}"),
            }
            self.profiles_list = ProfileConfig::list_profiles();
            self.profile_pending_delete = None;
        }
    }
}
