use crate::{
    colors, find_npclient_dll, find_xplane_plugin_so, sc_bin64_path, ui, HeadtrackApp, VERSION,
};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::config::CameraConfig;
use headtrack_engine::EngineCmd;

impl HeadtrackApp {
    // ── Settings ──────────────────────────────────────────────────────────────

    pub(crate) fn ui_settings(&mut self, ui: &mut egui::Ui) {
        ui::page_header(ui, "Settings", |_ui| {});

        ui::section(ui, "Application");
        ui::group(ui, None, |ui| self.ui_settings_application(ui));

        ui::section(ui, "Simulators");
        ui::group(ui, Some("MSFS 2024"), |ui| self.ui_settings_msfs(ui));
        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("X-Plane 12"), |ui| self.ui_settings_xplane(ui));
        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("Star Citizen"), |ui| self.ui_settings_sc(ui));

        ui::section(ui, "System");
        ui::group(ui, Some("Pose model"), |ui| self.ui_settings_pose_model(ui));
        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("IPC"), |ui| self.ui_settings_ipc(ui));
        ui.add_space(ui::ROW_GAP);
        ui::group(ui, Some("About"), |ui| self.ui_settings_about(ui));
        ui.add_space(8.0);
    }

    // ── Application ──────────────────────────────────────────────────────────

    fn ui_settings_application(&mut self, ui: &mut egui::Ui) {
        if ui::toggle_row(ui, "Close minimises to tray", &mut self.minimize_to_tray).changed() {
            let mut cfg = CameraConfig::load();
            cfg.minimize_to_tray = self.minimize_to_tray;
            let _ = cfg.save();
        }

        if ui::toggle_row(ui, "Show advanced tuning", &mut self.show_advanced)
            .on_hover_text(
                "Adds the Advanced tab: per-axis filter internals, cross-axis compensation, \
                 rate limits and the diagnostics recorder. Those controls need measured data to \
                 set sensibly — leave this off unless you are calibrating.",
            )
            .changed()
        {
            let mut cfg = CameraConfig::load();
            cfg.show_advanced = self.show_advanced;
            let _ = cfg.save();
            // Never leave the user staring at a tab that just disappeared.
            if !self.show_advanced && self.active_tab == crate::Tab::Advanced {
                self.active_tab = crate::Tab::Dashboard;
            }
        }

        ui.add_space(10.0);
        ui::subheading(ui, "Diagnostics folder");
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let width = (ui.available_width() - 110.0).clamp(160.0, 420.0);
            if ui::path_field(ui, "diag_directory", &mut self.diag_directory, "", width).changed() {
                let mut cfg = CameraConfig::load();
                cfg.diag_directory = self.diag_directory.clone();
                let _ = cfg.save();
            }
            if ui::tool_button(ui, ph::FOLDER_OPEN, "Browse", ui::ButtonKind::Normal).clicked() {
                if let Some(folder) = rfd::FileDialog::new()
                    .set_title("Select diagnostics folder")
                    .set_directory(&self.diag_directory)
                    .pick_folder()
                {
                    self.diag_directory = folder.display().to_string();
                    let mut cfg = CameraConfig::load();
                    cfg.diag_directory = self.diag_directory.clone();
                    let _ = cfg.save();
                }
            }
        });
        // The recorder itself lives in Advanced; without this line a user
        // filing a bug report has no way to discover where it went.
        ui::note(
            ui,
            if self.show_advanced {
                "Recording is in the Advanced tab."
            } else {
                "Recording is in the Advanced tab; turn it on above."
            },
            colors::DIM,
        );
    }

    // ── MSFS ─────────────────────────────────────────────────────────────────

    /// MSFS 2024 output settings: camera smoothing (output-side only), the
    /// stream/release toggle, and the per-aircraft seat position offset.
    fn ui_settings_msfs(&mut self, ui: &mut egui::Ui) {
        use std::sync::atomic::Ordering;

        let linked = self.msfs_link();
        ui::kv_row(ui, "SimConnect", |ui| {
            if linked {
                ui::state(ui, "Linked", ui::Tone::Ok);
            } else {
                ui::state(ui, "Waiting on port 5557", ui::Tone::Idle);
            }
        });

        // ── One-time SimConnect.xml setup ────────────────────────────────────
        // MSFS only accepts our TCP connection if its own config names port 5557,
        // and it can silently drop that entry during a major update. This was a
        // hand-edit buried in the docs; it is a button now.
        let msfs_dir = crate::detect_msfs_simconnect_dir();
        match &msfs_dir {
            Some(dir) => {
                let configured = std::fs::read_to_string(dir.join("SimConnect.xml"))
                    .map(|c| c.contains("<Port>5557</Port>"))
                    .unwrap_or(false);
                ui::kv_row(ui, "SimConnect.xml", |ui| {
                    if configured {
                        ui::state(ui, "Port 5557 set", ui::Tone::Ok);
                    } else {
                        ui::state(ui, "Needs port 5557", ui::Tone::Warn);
                    }
                });
                // Where the file is — the button below writes here.
                let dir_text = dir.display().to_string();
                ui::kv_row(ui, "Prefix folder", |ui| {
                    let room = ui.available_width();
                    ui::path_label(ui, &dir_text, room, colors::DIM);
                });
                ui.add_space(2.0);
                if ui::tool_button(
                    ui,
                    ph::WRENCH,
                    if configured { "Re-check SimConnect.xml" } else { "Set up SimConnect.xml" },
                    ui::ButtonKind::Normal,
                )
                .on_hover_text(
                    "Adds a port 5557 listener to MSFS's SimConnect.xml. Existing entries are \
                     kept and the original file is backed up beside it. Re-run this if tracking \
                     stops connecting after a sim update — MSFS can rewrite the file.",
                )
                .clicked()
                {
                    self.msfs_setup_status = Some(match crate::install_simconnect_xml(dir) {
                        Ok(crate::SimConnectSetup::AlreadyConfigured) => (
                            "Already configured: port 5557 is present, nothing changed.".to_string(),
                            true,
                        ),
                        Ok(crate::SimConnectSetup::Created(p)) => (
                            format!(
                                "Created {} in the prefix folder. Restart MSFS.",
                                p.file_name().unwrap_or_default().to_string_lossy()
                            ),
                            true,
                        ),
                        Ok(crate::SimConnectSetup::Amended { backup, .. }) => (
                            format!(
                                "Added port 5557. Original saved as {}. Restart MSFS.",
                                backup.file_name().unwrap_or_default().to_string_lossy()
                            ),
                            true,
                        ),
                        Err(e) => (format!("Could not set it up: {e}"), false),
                    });
                }
            }
            None => {
                ui::kv_row(ui, "SimConnect.xml", |ui| {
                    ui::state(ui, "MSFS 2024 not found", ui::Tone::Err);
                });
                ui::note(
                    ui,
                    "No MSFS 2024 Proton prefix in any Steam library; the manual steps are in \
                     docs/MSFS_TESTING.md.",
                    colors::DIM,
                );
            }
        }
        if let Some((msg, ok)) = &self.msfs_setup_status {
            ui.add_space(4.0);
            ui::note(ui, msg, if *ok { colors::OK } else { colors::ERR });
        }

        ui.add_space(12.0);
        if ui::param_cell(ui, "Camera response", &mut self.msfs_smoothing, 4.0..=40.0, 1) {
            self.cmd
                .send(EngineCmd::SetSimConnectSmoothing(self.msfs_smoothing));
            let mut cfg = CameraConfig::load();
            cfg.msfs_camera_smoothing = self.msfs_smoothing;
            let _ = cfg.save();
        }
        ui::note(
            ui,
            "MSFS only; lower is smoother with more latency, and about 15 matches X-Plane.",
            colors::DIM,
        );

        // Backed by the ENGINE's mirror rather than a local field, so toggles
        // from the command socket / controller bindings show up here too.
        ui.add_space(8.0);
        let mut streaming = self.state.msfs_enabled.load(Ordering::Relaxed);
        if ui::toggle_row(ui, "Stream camera to MSFS", &mut streaming)
            .on_hover_text(
                "Turn off to release the camera back to MSFS (external views and in-sim camera \
                 controls work again). Also bindable to a HOTAS button, or `msfs-enabled \
                 0|1|toggle` on the command socket. Pause Tracking releases it too.",
            )
            .changed()
        {
            self.cmd.send(EngineCmd::SimConnectEnabled(streaming));
        }

        ui.add_space(10.0);
        ui::subheading(ui, "Seat position");
        ui.add_space(2.0);
        ui::note(ui, "Saved per aircraft and re-applied next flight.", colors::DIM)
            .on_hover_text(
                "MSFS locks its own camera controls while head tracking holds the camera, so \
                 the pilot eyepoint is adjusted here.",
            );
        ui.add_space(6.0);

        let title = self
            .state
            .msfs_title
            .lock()
            .map(|t| t.clone())
            .unwrap_or_default();
        ui::kv_row(ui, "Aircraft", |ui| {
            if title.is_empty() {
                ui.label(
                    egui::RichText::new("Not identified; default seat")
                        .size(colors::TEXT_BODY)
                        .color(colors::DIM),
                );
            } else {
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(&title)
                            .monospace()
                            .size(colors::TEXT_SM)
                            .color(colors::TEXT_HI),
                    )
                    .truncate(),
                )
                .on_hover_text(&title);
            }
        });

        let seat: [f32; 3] = std::array::from_fn(|i| {
            f32::from_bits(self.state.msfs_seat[i].load(Ordering::Relaxed))
        });
        // EYEPOINT axis directions, verified in-cockpit 2026-08-01:
        // +X = LEFT (hence sign −1 on that row), +Y = up, +Z = forward.
        // `sign` makes "+" and a positive display always mean the SECOND word
        // of the row label.
        let rows: [(&str, usize, f32); 3] = [
            ("Left / Right", 0, -1.0),
            ("Down / Up", 1, 1.0),
            ("Aft / Forward", 2, 1.0),
        ];
        egui::Grid::new("msfs_seat_grid")
            .spacing([10.0, 5.0])
            .min_col_width(110.0)
            .show(ui, |ui| {
                for (label, axis, sign) in rows {
                    ui.label(egui::RichText::new(label).size(colors::TEXT_BODY).color(colors::TEXT));
                    // Same `−  value  +` control as the Camera FOV, with a
                    // read-only middle: the offset is an engine mirror, nudged
                    // rather than typed.
                    let step = ui::stepper(ui, |ui| {
                        // `+0.0 * -1.0` is negative zero; `ui::signed`
                        // shows it, and anything rounding to it, as "0 cm".
                        let cm = seat[axis] * 100.0 * sign;
                        ui.add_sized(
                            [64.0, colors::CONTROL_H],
                            egui::Label::new(
                                egui::RichText::new(format!("{} cm", ui::signed(cm, 0)))
                                    .monospace()
                                    .size(colors::TEXT_SM)
                                    .color(colors::TEXT_HI),
                            ),
                        );
                    });
                    if let Some(dir) = step {
                        let delta = if dir == ui::Step::Inc { 0.01 } else { -0.01 };
                        self.cmd.send(EngineCmd::MsfsSeatNudge {
                            axis: axis as u8,
                            delta_m: delta * sign,
                        });
                    }
                    ui.end_row();
                }
            });
        ui.add_space(4.0);
        if ui::tool_button(ui, ph::ARROW_COUNTER_CLOCKWISE, "Reset seat", ui::ButtonKind::Normal)
            .clicked()
        {
            self.cmd.send(EngineCmd::MsfsSeatReset);
        }
    }

    // ── X-Plane ──────────────────────────────────────────────────────────────

    fn ui_settings_xplane(&mut self, ui: &mut egui::Ui) {
        let so_path = find_xplane_plugin_so();
        let installed = !self.xplane_path.trim().is_empty()
            && std::path::Path::new(self.xplane_path.trim())
                .join("Resources/plugins/headtrack/64/lin.xpl")
                .exists();

        install_state(ui, "Plugin", installed, "Installed", "Not installed");
        ui.add_space(4.0);

        field_label(ui, "X-Plane 12 folder");
        ui.horizontal(|ui| {
            let width = (ui.available_width() - 110.0).clamp(160.0, 420.0);
            ui::path_field(ui, "xplane_path", &mut self.xplane_path, "e.g. /home/user/X-Plane 12", width);
            if ui::tool_button(ui, ph::FOLDER_OPEN, "Browse", ui::ButtonKind::Normal).clicked() {
                let start = if std::path::Path::new(&self.xplane_path).is_dir() {
                    self.xplane_path.clone()
                } else {
                    std::env::var("HOME").unwrap_or_default()
                };
                if let Some(folder) = rfd::FileDialog::new()
                    .set_title("Select X-Plane 12 folder")
                    .set_directory(start)
                    .pick_folder()
                {
                    self.xplane_path = folder.display().to_string();
                }
            }
        });

        ui.add_space(6.0);
        match &so_path {
            Some(p) => path_row(ui, "Source", &p.display().to_string()),
            None => {
                ui::note(
                    ui,
                    "Plugin binary not found; build it with cargo build -p headtrack-xplane --release.",
                    colors::WARN,
                );
            }
        }

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let can_install = so_path.is_some() && !self.xplane_path.trim().is_empty();
            if ui::tool_button_enabled(
                ui,
                can_install,
                ph::DOWNLOAD_SIMPLE,
                "Install X-Plane 12 plugin",
                ui::ButtonKind::Normal,
            )
            .clicked()
            {
                if let Some(so) = &so_path {
                    let xp = std::path::Path::new(self.xplane_path.trim());
                    let plugin_dir = xp.join("Resources/plugins/headtrack/64");
                    let result = std::fs::create_dir_all(&plugin_dir)
                        .and_then(|_| std::fs::copy(so, plugin_dir.join("lin.xpl")).map(|_| ()));
                    self.xplane_install_status = Some(match result {
                        Ok(()) => (
                            "Installed lin.xpl in the X-Plane 12 folder, under \
                             Resources/plugins/headtrack/64."
                                .to_string(),
                            true,
                        ),
                        Err(e) => (format!("Install failed: {e}"), false),
                    });
                }
            }
        });
        if let Some((msg, ok)) = &self.xplane_install_status {
            ui.add_space(4.0);
            ui::note(ui, msg, if *ok { colors::OK } else { colors::ERR });
        }
    }

    // ── Star Citizen ─────────────────────────────────────────────────────────

    fn ui_settings_sc(&mut self, ui: &mut egui::Ui) {
        let dll_path = find_npclient_dll();
        let bin64 = sc_bin64_path(self.sc_prefix_path.trim());
        let bin64_ok = bin64.is_dir();
        let installed = bin64.join("NPClient64.dll").exists();

        install_state(ui, "Bridge", installed, "Installed", "Not set up");
        ui.add_space(4.0);

        field_label(ui, "Wine prefix");
        ui.horizontal(|ui| {
            let width = (ui.available_width() - 110.0).clamp(160.0, 420.0);
            ui::path_field(ui, "sc_prefix_path", &mut self.sc_prefix_path, "e.g. /home/user/Games/star-citizen", width);
            if ui::tool_button(ui, ph::FOLDER_OPEN, "Browse", ui::ButtonKind::Normal).clicked() {
                let start = if std::path::Path::new(&self.sc_prefix_path).is_dir() {
                    self.sc_prefix_path.clone()
                } else {
                    std::env::var("HOME").unwrap_or_default()
                };
                if let Some(folder) = rfd::FileDialog::new()
                    .set_title("Select Star Citizen Wine prefix folder")
                    .set_directory(start)
                    .pick_folder()
                {
                    self.sc_prefix_path = folder.display().to_string();
                }
            }
        });

        ui.add_space(6.0);
        if bin64_ok {
            path_row(ui, "Bin64", &bin64.display().to_string());
        } else {
            ui::note(ui, "SC Bin64 not found; check the Wine prefix path above.", colors::WARN);
        }
        match &dll_path {
            Some(p) => path_row(ui, "Source", &p.display().to_string()),
            None => {
                ui::note(
                    ui,
                    "NPClient64.dll not found; check the AppImage, or run from the project root.",
                    colors::WARN,
                );
            }
        }

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let busy = self.sc_install_pending.is_some();
            let can_setup =
                !busy && dll_path.is_some() && bin64_ok && !self.sc_prefix_path.trim().is_empty();
            if ui::tool_button_enabled(
                ui,
                can_setup,
                ph::DOWNLOAD_SIMPLE,
                if busy { "Configuring\u{2026}" } else { "Set up Wine bridge" },
                ui::ButtonKind::Normal,
            )
            .clicked()
            {
                if let Some(dll_src) = &dll_path {
                    let prefix = self.sc_prefix_path.trim().to_string();
                    match std::fs::copy(dll_src, bin64.join("NPClient64.dll")) {
                        Err(e) => {
                            self.sc_install_status = Some((format!("DLL copy failed: {e}"), false));
                        }
                        Ok(_) => {
                            // DLL copy is instant; the registry write can block
                            // several seconds while Wine initialises — run it
                            // off the UI thread.
                            // The spinner below says the registry part is
                            // still running.
                            self.sc_install_status = Some(("DLL installed.".into(), true));
                            let (tx, rx) = std::sync::mpsc::channel();
                            self.sc_install_pending = Some(rx);
                            std::thread::spawn(move || {
                                let reg_key = r"HKCU\Software\NaturalPoint\NATURALPOINT\NPClient Location";
                                let reg_val = r"C:\Program Files\Roberts Space Industries\StarCitizen\LIVE\Bin64\";
                                let result = std::process::Command::new("wine")
                                    .env("WINEPREFIX", &prefix)
                                    .args([
                                        "reg", "add", reg_key, "/v", "Path", "/t", "REG_SZ", "/d",
                                        reg_val, "/f",
                                    ])
                                    .output()
                                    .map_err(|e| format!("wine not found: {e}"))
                                    .and_then(|out| {
                                        if out.status.success() {
                                            Ok(())
                                        } else {
                                            Err(format!(
                                                "wine reg add failed: {}",
                                                String::from_utf8_lossy(&out.stderr).trim()
                                            ))
                                        }
                                    });
                                let _ = tx.send(match result {
                                    Ok(()) => (
                                        "NPClient64.dll installed and registry key set. Enable \
                                         TrackIR in the Star Citizen settings."
                                            .to_string(),
                                        true,
                                    ),
                                    Err(e) => (e, false),
                                });
                            });
                        }
                    }
                }
            }
        });
        if self.sc_install_pending.is_some() {
            ui.add_space(4.0);
            ui::activity(ui, "Configuring the registry\u{2026}");
        }
        if let Some((msg, ok)) = &self.sc_install_status {
            ui.add_space(4.0);
            ui::note(ui, msg, if *ok { colors::OK } else { colors::ERR });
        }

        ui.add_space(12.0);
        ui::subheading(ui, "View smoothing");
        ui.add_space(4.0);
        let cell = ui.scope(|ui| {
            ui::param_cell(ui, "View response", &mut self.sc_smoothing, 0.0..=40.0, 1)
        });
        let changed = cell.inner;
        cell.response.on_hover_text(
                "Star Citizen pulls pose from shared memory on its own schedule and applies no \
                 smoothing of its own, so this is the only damping on that path. X-Plane \
                 interpolates on the sim's frame clock and MSFS has its own slider.",
        );
        ui::note(
            ui,
            "Star Citizen only; lower is smoother with more latency, and 0 sends the raw \
             pipeline output.",
            colors::DIM,
        );
        if changed {
            self.cmd.send(EngineCmd::SetScSmoothing(self.sc_smoothing));
            let mut cfg = CameraConfig::load();
            cfg.sc_output_smoothing = self.sc_smoothing;
            let _ = cfg.save();
        }
    }

    // ── System ───────────────────────────────────────────────────────────────

    fn ui_settings_pose_model(&self, ui: &mut egui::Ui) {
        ui::note(ui, "Bundled AI models; no internet required.", colors::DIM);
        ui.add_space(4.0);
        for (label, value) in [
            ("Face detector", "face_detection_yunet_2023mar.onnx (227 KB)"),
            ("Rotation model", "head-pose-mobilenetv3-large.onnx (16 MB)"),
        ] {
            ui::kv_row(ui, label, |ui| {
                let room = ui.available_width();
                ui::path_label(ui, value, room, colors::TEXT_HI);
            });
        }
    }

    fn ui_settings_ipc(&self, ui: &mut egui::Ui) {
        ui::kv_row(ui, "Pose socket", |ui| {
            if ui::tool_button(ui, ph::COPY, "Copy", ui::ButtonKind::Normal).clicked() {
                ui.ctx().copy_text(self.ipc_socket_label.clone());
            }
            let room = ui.available_width();
            ui::path_label(ui, &self.ipc_socket_label, room, colors::TEXT_HI);
        });
    }

    fn ui_settings_about(&mut self, ui: &mut egui::Ui) {
        ui::note(ui, "Open-source head tracking for Linux flight simulators.", colors::DIM);
        ui.add_space(4.0);
        ui::kv_row(ui, "Version", |ui| {
            ui.label(
                egui::RichText::new(VERSION)
                    .monospace()
                    .size(colors::TEXT_SM)
                    .color(colors::TEXT_HI),
            );
        });
        ui.add_space(4.0);

        let icon = self.github_texture.get_or_insert_with(|| {
            let bytes = include_bytes!("../../../../assets/icons/github-mark-white.png");
            let img = image::load_from_memory(bytes).expect("github icon").to_rgba8();
            let dims = [img.width() as usize, img.height() as usize];
            let color_image =
                egui::ColorImage::from_rgba_unmultiplied(dims, img.as_flat_samples().as_slice());
            ui.ctx()
                .load_texture("github-icon", color_image, egui::TextureOptions::LINEAR)
        });

        ui.horizontal(|ui| {
            let sized = egui::load::SizedTexture::new(icon.id(), egui::vec2(15.0, 15.0));
            ui.add(egui::Image::new(sized));
            ui.hyperlink_to(
                "View on GitHub",
                "https://github.com/TripleJumpStudios/HeadtrackRS",
            );
            ui.add_space(14.0);
            ui.hyperlink_to(
                "Report a bug",
                "https://github.com/TripleJumpStudios/HeadtrackRS/issues",
            );
        });
    }
}

/// Whether a simulator integration is installed, as a key and a state mark.
fn install_state(ui: &mut egui::Ui, key: &str, ok: bool, yes: &str, no: &str) {
    ui::kv_row(ui, key, |ui| {
        if ok {
            ui::state(ui, yes, ui::Tone::Ok);
        } else {
            ui::state(ui, no, ui::Tone::Idle);
        }
    });
}

/// The label above a field — a label, never a sentence.
fn field_label(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).size(colors::TEXT_BODY).color(colors::DIM));
}

/// A located file or folder: a key, then the path in monospace, elided from
/// the left to the room the row has, the whole path on hover.
fn path_row(ui: &mut egui::Ui, key: &str, path: &str) {
    ui::kv_row(ui, key, |ui| {
        let room = ui.available_width();
        ui::path_label(ui, path, room, colors::DIM);
    });
}
