use crate::{colors, enumerate_cameras, ui, HeadtrackApp, CAMERA_RESOLUTIONS};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::config::{CameraConfig, FovPreset};
use headtrack_engine::EngineCmd;

/// Controls worth showing without opening the full driver list. Matched on a
/// lowercased substring because V4L2 names vary between drivers
/// ("Exposure, Auto" / "Auto Exposure", "White Balance Temperature", …).
const COMMON_CONTROLS: [&str; 5] = ["brightness", "contrast", "gain", "exposure", "white balance"];

/// The line `Optimise for tracking` leaves under its button, for `applied`
/// controls set.
pub(crate) fn optimise_message(applied: usize) -> String {
    match applied {
        0 => format!("{}  No exposure controls found on this camera.", ph::WARNING),
        1 => format!(
            "{}  Manual exposure on (1 control set). Raise Gain if the picture is dark.",
            ph::CHECK
        ),
        n => format!(
            "{}  Manual exposure on ({n} controls set). Raise Gain if the picture is dark.",
            ph::CHECK
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::{control_label, optimise_message};

    #[test]
    fn v4l2_names_read_like_words() {
        for (raw, want) in [
            ("White Balance, Automatic", "Auto white balance"),
            ("White Balance Temperature, Auto", "Auto white balance"),
            ("White Balance Temperature", "White balance temperature"),
            ("Exposure Time, Absolute", "Exposure time"),
            ("Exposure (Absolute)", "Exposure"),
            ("Exposure, Dynamic Framerate", "Low-light frame-rate drop"),
            ("Auto Exposure", "Exposure mode"),
            ("Focus, Automatic Continuous", "Autofocus"),
            ("Focus, Absolute", "Focus"),
            ("Gain, Automatic", "Auto gain"),
            ("Power Line Frequency", "Anti-flicker"),
            ("Backlight Compensation", "Backlight compensation"),
            ("Brightness", "Brightness"),
            ("Pan, Relative", "Pan (relative)"),
            ("LED1 Mode", "LED1 mode"),
            ("User Controls", "Image"),
            ("Camera Controls", "Camera"),
            ("", ""),
        ] {
            assert_eq!(control_label(raw), want, "{raw:?}");
        }
    }

    #[test]
    fn the_optimise_line_counts_in_english() {
        assert!(optimise_message(1).contains("(1 control set)"), "{}", optimise_message(1));
        assert!(optimise_message(2).contains("(2 controls set)"));
        assert!(optimise_message(0).contains("No exposure controls"));
    }
}

impl HeadtrackApp {
    // ── Camera Controls ───────────────────────────────────────────────────────

    pub(crate) fn ui_camera(&mut self, ui: &mut egui::Ui) {
        ui::page_header(ui, "Camera", |_ui| {});
        ui::subtitle(
            ui,
            "Hardware settings apply to the live tracking capture, not just the preview.",
        );

        // Preview first: image controls below are pointless if you cannot see
        // what they do without switching tabs.
        //
        // `Trail`: unlike the Dashboard the preview is here to confirm framing,
        // not to be watched, so the device controls take the wider column.
        // `Lead` rather than a narrow preview: the image is aspect-locked to
        // its column width, so widening the column is the only thing that makes
        // it taller.
        //
        // Moving field-of-view under the preview was tried — optics with the
        // picture they change — and it simply inverted the imbalance, leaving
        // Device nearly empty. FOV is a device property; it stays in Device.
        ui::two_columns(ui, "camera-row", ui::Split::Lead, |c| {
            c.group(Some("Preview"), |ui| {
                // Hug the picture: the box is the frame, so there are no bars.
                let inner = ui.available_width();
                let h = inner / self.preview_aspect();
                self.draw_camera_preview(ui, inner, h);
            });
            c.group(Some("Device"), |ui| self.ui_camera_device(ui));
        });

        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Image"), |ui| self.ui_camera_image(ui));
        ui.add_space(8.0);
    }

    // ── Device group ─────────────────────────────────────────────────────────

    fn ui_camera_device(&mut self, ui: &mut egui::Ui) {
        field_label(ui, "Source");
        ui.horizontal(|ui| {
            let selected_label = self
                .cameras
                .iter()
                .find(|c| c.index == self.selected_camera)
                .map(|c| c.label())
                .unwrap_or_else(|| format!("/dev/video{}", self.selected_camera));

            let width = (ui.available_width() - colors::CONTROL_H - 8.0).clamp(120.0, 300.0);
            ui::combo(ui, "camera_select", &selected_label, width, |ui| {
                for cam in &self.cameras {
                    if ui
                        .selectable_label(cam.index == self.selected_camera, cam.label())
                        .clicked()
                    {
                        self.selected_camera = cam.index;
                        self.v4l2_last_camera = u32::MAX;
                        self.cmd.send(EngineCmd::SwitchCamera { index: cam.index });
                    }
                }
            });
            if ui::icon_button(ui, ph::ARROW_CLOCKWISE, ui::ButtonKind::Normal)
                .on_hover_text("Rescan for cameras")
                .clicked()
            {
                self.cameras = enumerate_cameras();
            }
        });

        ui.add_space(8.0);
        field_label(ui, "Resolution");
        let width = ui.available_width().clamp(120.0, 320.0);
        ui::combo(
            ui,
            "camera_resolution",
            CAMERA_RESOLUTIONS[self.camera_res_idx].2,
            width,
            |ui| {
                for (i, (w, h, label)) in CAMERA_RESOLUTIONS.iter().enumerate() {
                    if ui.selectable_label(self.camera_res_idx == i, *label).clicked() {
                        self.camera_res_idx = i;
                        self.cmd.send(EngineCmd::SetResolution {
                            width: *w,
                            height: *h,
                        });
                    }
                }
            },
        );
        ui::note(ui, "Changing this restarts the camera (~1 s).", colors::DIM);

        ui.add_space(10.0);
        self.ui_camera_fov(ui);
    }

    fn ui_camera_fov(&mut self, ui: &mut egui::Ui) {
        field_label(ui, "Diagonal field of view");
        ui.horizontal(|ui| {
            let step = ui::stepper(ui, |ui| {
                ui.add_sized(
                    [56.0, colors::CONTROL_H],
                    egui::DragValue::new(&mut self.camera_fov)
                        .range(20.0..=150.0)
                        .speed(0.0)
                        .suffix("\u{00b0}")
                        .fixed_decimals(0),
                );
            });
            match step {
                Some(ui::Step::Dec) => self.camera_fov = (self.camera_fov - 1.0).max(20.0),
                Some(ui::Step::Inc) => self.camera_fov = (self.camera_fov + 1.0).min(150.0),
                None => {}
            }
            ui.add_space(6.0);
            if ui::tool_button(ui, "", "Apply", ui::ButtonKind::Normal).clicked() {
                self.cmd.send(EngineCmd::SetFov {
                    degrees: self.camera_fov,
                });
            }
        });
        ui::note(ui, "Affects depth (Z) estimation accuracy.", colors::DIM);

        ui.add_space(10.0);
        ui::subheading(ui, "Presets");
        ui.add_space(4.0);
        self.ui_camera_fov_presets(ui);
    }

    /// Saved FOV presets as chips.
    ///
    /// These used to live inside a combo box whose entries carried their own
    /// delete buttons — deleting a preset meant opening a dropdown to click an
    /// ✕ inside it.
    fn ui_camera_fov_presets(&mut self, ui: &mut egui::Ui) {
        let mut cfg = CameraConfig::load();
        let mut apply: Option<usize> = None;
        let mut delete: Option<usize> = None;

        if cfg.fov_presets.is_empty() {
            ui::note(ui, "No presets saved.", colors::DIM);
        } else {
            ui.horizontal_wrapped(|ui| {
                for (i, p) in cfg.fov_presets.iter().enumerate() {
                    let active = (p.fov - self.camera_fov).abs() < 0.5;
                    if ui::toggle_chip(
                        ui,
                        &format!("{}  {}\u{00b0}", p.name, p.fov as i32),
                        active,
                        colors::ORANGE,
                    )
                    .clicked()
                    {
                        apply = Some(i);
                    }
                    if ui::action_icon(ui, ph::X, &format!("Delete preset \"{}\"", p.name))
                        .clicked()
                    {
                        delete = Some(i);
                    }
                }
            });
        }

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ui::text_field(ui, &mut self.fov_preset_name, "Preset name\u{2026}", 150.0);
            if ui::tool_button(ui, ph::FLOPPY_DISK, "Save", ui::ButtonKind::Normal).clicked() {
                let name = self.fov_preset_name.trim().to_owned();
                if !name.is_empty() {
                    if let Some(p) = cfg.fov_presets.iter_mut().find(|p| p.name == name) {
                        p.fov = self.camera_fov;
                    } else {
                        cfg.fov_presets.push(FovPreset {
                            name,
                            fov: self.camera_fov,
                        });
                    }
                    let _ = cfg.save();
                    self.fov_preset_name.clear();
                }
            }
        });

        if let Some(i) = apply {
            self.camera_fov = cfg.fov_presets[i].fov.clamp(20.0, 150.0);
            self.cmd.send(EngineCmd::SetFov {
                degrees: self.camera_fov,
            });
        }
        if let Some(i) = delete {
            cfg.fov_presets.remove(i);
            let _ = cfg.save();
        }
    }

    // ── Image group (V4L2) ───────────────────────────────────────────────────

    fn ui_camera_image(&mut self, ui: &mut egui::Ui) {
        // Refresh the control list when the camera changes.
        if self.v4l2_last_camera != self.selected_camera {
            self.v4l2_last_camera = self.selected_camera;
            self.v4l2_optimize_status = None;
            if let Ok(dev) = v4l::Device::with_path(format!("/dev/video{}", self.selected_camera)) {
                if let Ok(controls) = dev.query_controls() {
                    self.v4l2_controls = controls;
                }
                self.v4l2_device = Some(dev);
            } else {
                self.v4l2_device = None;
                self.v4l2_controls.clear();
            }
        }

        self.ui_camera_optimize(ui);
        ui.add_space(10.0);

        if self.v4l2_controls.is_empty() {
            ui::note(ui, "This driver exposes no V4L2 controls.", colors::DIM);
            return;
        }

        let Some(dev) = &self.v4l2_device else { return };

        // Common controls up front; everything else behind a disclosure, so the
        // tab no longer opens with twenty driver sliders.
        let is_common = |desc: &v4l::control::Description| {
            let n = desc.name.to_lowercase();
            desc.typ != v4l::control::Type::CtrlClass
                && COMMON_CONTROLS.iter().any(|k| n.contains(k))
        };

        for desc in self.v4l2_controls.iter().filter(|d| is_common(d)) {
            v4l2_row(ui, dev, desc);
        }

        ui.add_space(8.0);
        ui::disclosure(ui, "v4l2_all", "All camera controls", |ui| {
            // Driver-declared sections: a CtrlClass entry opens a new group.
            let mut section_name: Option<String> = None;
            let mut pending: Vec<&v4l::control::Description> = Vec::new();

            let flush = |ui: &mut egui::Ui,
                         name: &Option<String>,
                         items: &mut Vec<&v4l::control::Description>| {
                if items.is_empty() {
                    return;
                }
                if let Some(name) = name {
                    ui::subheading(ui, &control_label(name));
                    ui.add_space(3.0);
                }
                for desc in items.iter() {
                    v4l2_row(ui, dev, desc);
                }
                ui.add_space(6.0);
                items.clear();
            };

            for desc in &self.v4l2_controls {
                if desc.typ == v4l::control::Type::CtrlClass {
                    flush(ui, &section_name, &mut pending);
                    section_name = Some(desc.name.clone());
                } else {
                    pending.push(desc);
                }
            }
            flush(ui, &section_name, &mut pending);
        });
    }

    fn ui_camera_optimize(&mut self, ui: &mut egui::Ui) {
        if ui::tool_button(ui, ph::LIGHTNING, "Optimise for tracking", ui::ButtonKind::Primary)
            .clicked()
        {
            self.apply_tracking_optimisation();
        }
        ui::note(
            ui,
            "Turns off auto-exposure priority, which drops the frame rate in low light.",
            colors::DIM,
        );

        // Fading status message. The glyph is drawn through `icon_text` so it
        // resolves in the icon family, not Inter's look-alike codepoints.
        if let Some((msg, t)) = &self.v4l2_optimize_status {
            let age = t.elapsed().as_secs_f32();
            if age < 8.0 {
                let alpha = 1.0 - age / 8.0;
                let (glyph, rest, col) = match msg.strip_prefix(ph::CHECK) {
                    Some(rest) => (ph::CHECK, rest, colors::OK),
                    None => (
                        ph::WARNING,
                        msg.strip_prefix(ph::WARNING).unwrap_or(msg),
                        colors::WARN,
                    ),
                };
                ui.add_space(4.0);
                let mut job = ui::icon_text(glyph, rest.trim_start(), col.gamma_multiply(alpha));
                job.wrap.max_width = ui.available_width();
                ui.label(job);
                ui.ctx().request_repaint();
            } else {
                self.v4l2_optimize_status = None;
            }
        }
    }

    fn apply_tracking_optimisation(&mut self) {
        let Some(dev) = &self.v4l2_device else { return };
        let mut applied = 0usize;
        for desc in &self.v4l2_controls {
            let n = desc.name.to_lowercase();
            if n.contains("exposure")
                && n.contains("priority")
                && dev
                    .set_control(v4l::control::Control {
                        id: desc.id,
                        value: v4l::control::Value::Boolean(false),
                    })
                    .is_ok()
            {
                applied += 1;
            }
            if (n == "auto exposure" || n == "exposure, auto")
                && dev
                    .set_control(v4l::control::Control {
                        id: desc.id,
                        value: v4l::control::Value::Integer(1),
                    })
                    .is_ok()
            {
                applied += 1;
            }
        }
        if let Ok(controls) = dev.query_controls() {
            self.v4l2_controls = controls;
        }
        self.v4l2_optimize_status = Some((optimise_message(applied), std::time::Instant::now()));
    }
}

/// One driver control: fixed-width name on the left, widget filling the rest.
///
/// Laid out by hand rather than with `egui::Grid`: a grid cell narrow enough to
/// keep the names aligned squeezed the sliders down to a bare knob and let long
/// names overlap them.
fn v4l2_row(ui: &mut egui::Ui, dev: &v4l::Device, desc: &v4l::control::Description) {
    ui.horizontal(|ui| {
        let name_w = (ui.available_width() * 0.4).clamp(120.0, 210.0);
        // Left-aligned in a fixed cell, like every other page's labels.
        // `add_sized` lays its widget out centred-and-justified, which put a
        // short name at the far right of its cell (6c).
        ui.allocate_ui_with_layout(
            egui::vec2(name_w, 22.0),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.set_min_width(name_w);
                // The readable name; the driver's own stays on hover, for
                // anyone matching it against v4l2-ctl or a bug report.
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(control_label(&desc.name))
                            .size(colors::TEXT_BODY)
                            .color(colors::TEXT),
                    )
                    .truncate()
                    .selectable(false),
                )
                .on_hover_text(&desc.name);
            },
        );
        let avail = ui.available_width();
        ui_v4l2_control_widget(ui, dev, desc, avail);
    });
    ui.add_space(3.0);
}

/// Render a single V4L2 control widget (slider / on-off chip / combo).
fn ui_v4l2_control_widget(
    ui: &mut egui::Ui,
    dev: &v4l::Device,
    desc: &v4l::control::Description,
    avail: f32,
) {
    let Ok(val) = dev.control(desc.id) else {
        ui.label(egui::RichText::new("\u{2014}").size(colors::TEXT_BODY).color(colors::DIMMER));
        return;
    };

    match desc.typ {
        v4l::control::Type::Integer => {
            if let v4l::control::Value::Integer(mut current) = val.value {
                ui.horizontal(|ui| {
                    // Leave room for the drag value; never let the rail vanish.
                    ui.spacing_mut().slider_width = (avail - 96.0).clamp(90.0, 280.0);
                    let changed_s = ui
                        .add(
                            egui::Slider::new(&mut current, desc.minimum..=desc.maximum)
                                .show_value(false),
                        )
                        .changed();
                    let changed_d = ui
                        .add(
                            egui::DragValue::new(&mut current)
                                .range(desc.minimum..=desc.maximum)
                                .speed(1),
                        )
                        .changed();
                    if changed_s || changed_d {
                        let _ = dev.set_control(v4l::control::Control {
                            id: desc.id,
                            value: v4l::control::Value::Integer(current),
                        });
                    }
                });
            }
        }
        v4l::control::Type::Boolean => {
            if let v4l::control::Value::Boolean(mut current) = val.value {
                let label = if current { "On" } else { "Off" };
                if ui::toggle_chip(ui, label, current, colors::ORANGE).clicked() {
                    current = !current;
                    let _ = dev.set_control(v4l::control::Control {
                        id: desc.id,
                        value: v4l::control::Value::Boolean(current),
                    });
                }
            }
        }
        v4l::control::Type::Menu => {
            if let v4l::control::Value::Integer(mut current) = val.value {
                if let Some(items) = &desc.items {
                    let label = items
                        .iter()
                        .find(|(idx, _)| *idx as i64 == current)
                        .map(|(_, item)| menu_item_name(item))
                        .unwrap_or_else(|| format!("{current}"));
                    ui::combo(ui, desc.id, &label, (avail - 20.0).clamp(120.0, 240.0), |ui| {
                        for (idx, item) in items {
                            let name = menu_item_name(item);
                            if ui
                                .selectable_value(&mut current, *idx as i64, &name)
                                .changed()
                            {
                                let _ = dev.set_control(v4l::control::Control {
                                    id: desc.id,
                                    value: v4l::control::Value::Integer(current),
                                });
                            }
                        }
                    });
                }
            }
        }
        _ => {
            ui.label("");
        }
    }
}

/// A V4L2 control or class name as a person would say it (6c): the kernel's
/// names read like index entries (`White Balance, Automatic`,
/// `Exposure Time, Absolute`). Display only; everything that matches on a
/// control still uses the driver's own name.
pub(crate) fn control_label(raw: &str) -> String {
    let raw = raw.trim();
    // Names whose plain reading says something else, or nothing useful.
    let fixed = match raw.to_ascii_lowercase().as_str() {
        "user controls" => Some("Image"),
        "camera controls" => Some("Camera"),
        "auto exposure" | "exposure, auto" => Some("Exposure mode"),
        "exposure, dynamic framerate" | "exposure, auto priority" => Some("Low-light frame-rate drop"),
        "power line frequency" => Some("Anti-flicker"),
        "white balance temperature, auto" => Some("Auto white balance"),
        "focus, automatic continuous" | "focus, auto" => Some("Autofocus"),
        _ => None,
    };
    if let Some(f) = fixed {
        return f.to_string();
    }
    // `Head, Tail` → by what the tail says; `Head (Absolute)` likewise.
    let (head, tail) = match raw.split_once(", ") {
        Some((h, t)) => (h, Some(t)),
        None => match raw.strip_suffix(')').and_then(|r| r.split_once(" (")) {
            Some((h, t)) => (h, Some(t)),
            None => (raw, None),
        },
    };
    let words = match tail.map(str::to_ascii_lowercase).as_deref() {
        None | Some("absolute") => head.to_string(),
        Some("automatic") | Some("auto") => format!("Auto {head}"),
        Some(t) => format!("{head} ({t})"),
    };
    sentence_case(&words)
}

/// First letter up, the rest of each word down, except acronyms (`LED1`, `HDR`).
fn sentence_case(s: &str) -> String {
    s.split(' ')
        .enumerate()
        .map(|(i, w)| {
            let acronym = w.len() > 1 && w.chars().all(|c| !c.is_lowercase());
            if acronym {
                w.to_string()
            } else {
                // Driver names are ASCII; only the first letter is raised.
                let mut lower = w.to_lowercase();
                if i == 0 {
                    if let Some(first) = lower.get_mut(0..1) {
                        first.make_ascii_uppercase();
                    }
                }
                lower
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn menu_item_name(item: &v4l::control::MenuItem) -> String {
    match item {
        v4l::control::MenuItem::Name(s) => s.clone(),
        v4l::control::MenuItem::Value(v) => v.to_string(),
    }
}

/// The label above a control in a narrow column — a label, never a sentence.
fn field_label(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).size(colors::TEXT_BODY).color(colors::DIM));
}
