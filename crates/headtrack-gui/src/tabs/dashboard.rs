use crate::{
    colors, find_xplane_plugin_so, sc_bin64_path, ui, AXIS_LABELS, AXIS_UNITS, HeadtrackApp,
};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::input::{SourceKind, TobiiState};
use headtrack_engine::EngineCmd;
use std::sync::atomic::Ordering;

/// Full-scale deflection for the rotation gauges, degrees.
const ROT_GAUGE_MAX: f32 = 30.0;
/// Full-scale deflection for the translation gauges, millimetres.
const TRANS_GAUGE_MAX: f32 = 60.0;

impl HeadtrackApp {
    /// Decode the engine's JPEG preview into a texture and its pixel size, or
    /// `None` before the first frame arrives.
    fn preview_texture(&mut self, ui: &mut egui::Ui) -> Option<(egui::TextureId, egui::Vec2)> {
        let mut new_image = None;
        if let Ok(lock) = self.state.preview.lock() {
            let (seq, ref jpeg) = *lock;
            if seq != self.preview_last_seq {
                if let Some(ref data) = jpeg {
                    self.preview_last_seq = seq;
                    new_image = Some(data.clone());
                }
            }
        }

        if let Some(jpeg) = new_image {
            if let Ok(img) = image::load_from_memory_with_format(&jpeg, image::ImageFormat::Jpeg) {
                let rgba = img.to_rgba8();
                let dims = [rgba.width() as usize, rgba.height() as usize];
                let pixels = rgba.as_flat_samples();
                let color_image = egui::ColorImage::from_rgba_unmultiplied(dims, pixels.as_slice());
                self.preview_texture = Some(ui.ctx().load_texture(
                    "camera_preview",
                    color_image,
                    egui::TextureOptions::LINEAR,
                ));
            }
        }

        self.preview_texture
            .as_ref()
            .map(|t| (t.id(), t.size_vec2()))
    }

    /// Aspect ratio of the live preview, or 16:9 before the first frame lands
    /// (so a placeholder is not a wildly different shape from the feed that
    /// replaces it).
    pub(crate) fn preview_aspect(&self) -> f32 {
        self.preview_texture
            .as_ref()
            .map(|t| t.size_vec2())
            .filter(|s| s.y > 0.0)
            .map_or(16.0 / 9.0, |s| s.x / s.y)
    }

    /// Draw the camera preview into a `width` × `height` box, image centred and
    /// aspect-correct inside it.
    ///
    /// The box is exactly what the caller asks for; it is never silently
    /// shrunk. An earlier version derived the box from the image and clamped it
    /// to a caller max, which meant that whenever the clamp bound, the box came
    /// out narrower than the space it was given — and `allocate_exact_size`
    /// places at the cursor, so the whole remainder became a gap on the *right*.
    /// The feed appeared to drift out of its own frame as the window resized.
    ///
    /// Callers choose the box, and so choose the trade: pass `width / aspect`
    /// to hug the picture exactly (Camera), or the height of the row to fill a
    /// card that a taller neighbour has stretched (Dashboard).
    ///
    /// Shared by the Dashboard and the Camera tab — on the latter it means V4L2
    /// changes are visible where they are made, rather than a tab away.
    pub(crate) fn draw_camera_preview(
        &mut self,
        ui: &mut egui::Ui,
        width: f32,
        height: f32,
    ) -> f32 {
        // The Tobii has no camera picture to show (its IR frames never leave
        // the tracker thread), and the texture would be the webcam's last frame.
        let tobii = self.source_kind == SourceKind::Tobii;
        let tex = if tobii { None } else { self.preview_texture(ui) };
        let aspect = self.preview_aspect();

        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
        let painter = ui.painter_at(rect);

        painter.rect_filled(rect, 10.0, egui::Color32::BLACK);
        if let Some((id, _)) = tex {
            // Centred and aspect-correct: any leftover is split evenly as
            // letterbox rather than pooling on one side.
            let inner = rect.shrink(1.0);
            let scale = (inner.width() / aspect).min(inner.height());
            let img = egui::Rect::from_center_size(
                inner.center(),
                egui::vec2(scale * aspect, scale),
            );
            painter.image(
                id,
                img,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }
        painter.rect_stroke(
            rect,
            10.0,
            egui::Stroke::new(1.0_f32, colors::STROKE_SOFT),
            egui::StrokeKind::Inside,
        );

        if tobii {
            // No picture, so the box shows the head instead: the Tuning pose
            // bubble, moving with the pose the sims get (3c). Rotation only;
            // the gaze point is never shown here, as it is never sent.
            let status = self.tobii_status();
            let st = status.state;
            // Until it tracks, the box tells the user what to do (D1/D2):
            // starting, hold still with a countdown, or face the tracker.
            let prompt = super::tobii::tobii_prompt(&status);
            let streaming = st == TobiiState::Streaming && prompt.is_none();
            if let Some(prompt) = prompt {
                ui::hud_overlay(&painter, rect, true, None);
                ui::source_prompt(ui, &painter, rect, prompt);
                return height;
            }
            if streaming {
                let pose = *self.state.latest_pose.lock().unwrap_or_else(|e| e.into_inner());
                let bubble = egui::Rect::from_min_max(
                    rect.left_top() + egui::vec2(24.0, 24.0),
                    rect.right_bottom() - egui::vec2(24.0, 34.0),
                );
                if bubble.width() > 40.0 && bubble.height() > 40.0 {
                    ui::paint_pose_bubble(ui, &painter, bubble, pose.yaw, pose.pitch, pose.roll, None);
                }
            }
            let caption = if streaming {
                "Tobii tracking \u{00b7} no camera picture".to_string()
            } else {
                format!("Tobii: {}", super::tobii::status_label(&status).0.to_lowercase())
            };
            ui::hud_overlay(&painter, rect, streaming, Some(&caption));
            return height;
        }

        let locked = tex.is_some() && self.state.camera_connected.load(Ordering::Relaxed);
        let caption = if locked {
            None
        } else if tex.is_some() {
            Some("Camera offline")
        } else {
            Some("Awaiting face lock")
        };
        ui::hud_overlay(&painter, rect, locked, caption);
        height
    }

    // ── Dashboard ─────────────────────────────────────────────────────────────

    pub(crate) fn ui_dashboard(&mut self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        let fps = *self.state.fps.lock().unwrap_or_else(|e| e.into_inner());
        let connected = self.state.camera_connected.load(Ordering::Relaxed);

        ui::page_header(ui, "Dashboard", |ui| {
            match (self.source_health(connected), self.tracking_enabled) {
                (Err((why, tone)), _) => ui::state(ui, &why, tone),
                (Ok(()), false) => ui::state(ui, "Paused", ui::Tone::Warn),
                (Ok(()), true) => ui::state(ui, "Tracking active", ui::Tone::Ok),
            };
        });

        self.ui_dashboard_toolbar(ui);
        ui.add_space(ui::ROW_GAP);

        // `Lead`: the feed is the visual anchor, the status column is readouts.
        let total = ui.available_width();

        // The feed box grows to match the status column beside it, so the row
        // has no dead space under a shorter picture. The status column is
        // content-driven and taller, so this cannot be predicted with
        // constants — it is converged via `dash_feed_h`.
        let feed_h = self.dash_feed_h;

        let mut drawn_h = feed_h;
        let feed_label = match self.source_kind {
            SourceKind::Webcam => "Camera feed",
            SourceKind::Tobii => "Tobii",
        };
        let row = ui::two_columns(ui, "dash-row", ui::Split::Lead, |c| {
            c.group(Some(feed_label), |ui| {
                let inner = ui.available_width();
                drawn_h = self.draw_camera_preview(ui, inner, feed_h);
            });
            c.group(Some("Status"), |ui| {
                self.ui_dashboard_status_rows(ui, connected, fps);
            });
        });
        // Heading + spacing, measured rather than assumed.
        let chrome = row.lead_height - drawn_h;
        let target_feed_h = (row.row_height - chrome).clamp(150.0, 320.0);
        if (target_feed_h - self.dash_feed_h).abs() > 0.5 {
            self.dash_feed_h = target_feed_h;
            ui.ctx().request_repaint();
        }

        ui.add_space(ui::ROW_GAP);
        // Pinned to the same `total` the row above was built from, so the
        // gauges' right edge lines up with the status column's.
        ui.allocate_ui_with_layout(
            egui::vec2(total, 0.0),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                ui::group(ui, Some("Live pose"), |ui| {
                    self.ui_dashboard_pose_cluster(ui, pose_vals);
                });
            },
        );
        ui.add_space(8.0);
    }

    fn ui_dashboard_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;

            let (icon, label) = if self.tracking_enabled {
                (ph::PAUSE, "Pause tracking")
            } else {
                (ph::PLAY, "Resume tracking")
            };
            if ui::tool_button(ui, icon, label, ui::ButtonKind::Primary).clicked() {
                self.toggle_tracking();
            }

            let recenter_lit = self
                .recenter_flash
                .map(|t| t.elapsed().as_secs_f32() < 0.4)
                .unwrap_or(false);
            let kind = if recenter_lit {
                ui::ButtonKind::GhostActive
            } else {
                ui::ButtonKind::Normal
            };
            if ui::tool_button(ui, ph::CROSSHAIR_SIMPLE, "Recenter", kind).clicked() {
                self.trigger_recenter();
            }

            let overlay_kind = if self.landmark_overlay {
                ui::ButtonKind::GhostActive
            } else {
                ui::ButtonKind::Ghost
            };
            // The overlay draws on the webcam preview, which the Tobii has none of.
            let webcam = self.source_kind == SourceKind::Webcam;
            if ui::tool_button_enabled(ui, webcam, ph::BOUNDING_BOX, "Overlay", overlay_kind)
                .on_hover_text("Draw the detected face box and landmarks on the preview")
                .clicked()
            {
                self.landmark_overlay = !self.landmark_overlay;
                self.cmd.send(EngineCmd::Overlay {
                    enabled: self.landmark_overlay,
                });
            }

            if let Some(t) = self.recenter_flash {
                if t.elapsed().as_secs_f32() < 1.5 {
                    ui::state(ui, "Recentered", ui::Tone::Ok);
                } else {
                    self.recenter_flash = None;
                }
            }
        });
    }

    fn ui_dashboard_status_rows(&mut self, ui: &mut egui::Ui, connected: bool, fps: f32) {
        ui::kv_row(ui, "Status", |ui| {
            match self.source_health(connected) {
                Err((why, tone)) => ui::state(ui, &why, tone),
                Ok(()) if self.tracking_enabled => ui::state(ui, "Tracking", ui::Tone::Ok),
                Ok(()) => ui::state(ui, "Paused", ui::Tone::Warn),
            };
        });

        // Names the active head source: the webcam's device name, or the Tobii.
        let source_name = self.source_name();
        ui::kv_row(ui, "Source", |ui| {
            mono_value(ui, &source_name);
        });

        // Profile names are user-authored and routinely long ("MSFS TRANSLATION
        // v7 (v6 + pitch_to_yaw 0.25 -> 0.05)"), so this value must truncate or
        // it drags the whole column wider than its slot.
        let profile_name = self.profile_name.clone();
        let dirty = self.profile_dirty;
        ui::kv_row(ui, "Profile", |ui| {
            if dirty {
                ui::state(ui, "Unsaved", ui::Tone::Warn);
            }
            mono_value(ui, &profile_name);
        });

        ui::kv_row(ui, "Frame rate", |ui| {
            ui.label(
                egui::RichText::new(format!("{fps:.0} fps"))
                    .font(egui::FontId::new(colors::TEXT_BODY, colors::mono_bold()))
                    .color(colors::ORANGE),
            );
        });

        ui.add_space(6.0);
        ui::subheading(ui, "Outputs");
        ui.add_space(4.0);
        self.ui_dashboard_outputs(ui);
    }

    /// Per-simulator link state.
    ///
    /// Only MSFS has a live signal (the SimConnect session). The X-Plane plugin
    /// and the Star Citizen bridge push over a socket/SHM with no back-channel,
    /// so those rows report whether the integration is *installed* — the word
    /// says "configured", never "connected", because the GUI cannot know.
    fn ui_dashboard_outputs(&self, ui: &mut egui::Ui) {
        let msfs_linked = self.msfs_link();
        let xplane_ready = !self.xplane_path.trim().is_empty()
            && std::path::Path::new(self.xplane_path.trim())
                .join("Resources/plugins/headtrack/64/lin.xpl")
                .exists();
        let sc_ready = sc_bin64_path(self.sc_prefix_path.trim())
            .join("NPClient64.dll")
            .exists();

        for (name, ok, live, hint) in [
            (
                "MSFS 2024",
                msfs_linked,
                true,
                "SimConnect TCP session state, updated live",
            ),
            (
                "X-Plane 12",
                xplane_ready,
                false,
                "Whether lin.xpl is installed in the X-Plane plugins folder",
            ),
            (
                "Star Citizen",
                sc_ready,
                false,
                "Whether NPClient64.dll is installed in the SC Bin64 folder",
            ),
        ] {
            ui::kv_row(ui, name, |ui| {
                let (word, tone) = match (live, ok) {
                    (true, true) => ("Linked", ui::Tone::Ok),
                    (true, false) => ("Waiting", ui::Tone::Idle),
                    (false, true) => ("Configured", ui::Tone::Ok),
                    (false, false) => ("Not set up", ui::Tone::Idle),
                };
                ui::state(ui, word, tone);
            })
            .on_hover_text(hint);
        }

        // Surfaced here because a missing plugin binary is the usual reason a
        // freshly built tree cannot install the X-Plane side.
        if find_xplane_plugin_so().is_none() {
            ui::note(ui, "X-Plane plugin binary not built.", colors::DIM);
        }
    }

    /// The instrument cluster: rotation and translation gauge groups.
    ///
    /// On the page grid with the same `Lead` split as the feed + Status row
    /// above, so Translation starts where Status does. A hand-rolled 50/50
    /// split here put it ~60 px left of Status (3a).
    fn ui_dashboard_pose_cluster(&self, ui: &mut egui::Ui, pose_vals: &[f32; 6]) {
        ui::two_columns(ui, "dash-pose", ui::Split::Lead, |c| {
            for (group, title, unit, axes, max) in [
                (0usize, "Rotation", "degrees", [0usize, 1, 2], ROT_GAUGE_MAX),
                (1, "Translation", "millimetres", [3, 4, 5], TRANS_GAUGE_MAX),
            ] {
                c.group(None, |ui| {
                    let unit = if group == 1 && !self.source_kind.provides_translation() {
                        "not available with Tobii"
                    } else {
                        unit
                    };
                    ui::group_rule(ui, title, unit, ui::axis_family_color(axes[0]));
                    for i in axes {
                        ui::axis_gauge(
                            ui,
                            AXIS_LABELS[i],
                            pose_vals[i],
                            max,
                            AXIS_UNITS[i],
                            ui::axis_color(i),
                        );
                    }
                });
            }
        });
    }
}

/// A device or profile name as a monospace value: one line, truncated, the
/// whole name on hover.
fn mono_value(ui: &mut egui::Ui, text: &str) {
    ui.add(
        egui::Label::new(
            egui::RichText::new(text)
                .monospace()
                .size(colors::TEXT_SM)
                .color(colors::TEXT_HI),
        )
        .truncate(),
    )
    .on_hover_text(text);
}
