use crate::{colors, ui, HeadtrackApp};
#[cfg(feature = "dev-fixtures")]
use crate::{Tab, TuningGroup};
use eframe::egui;
use egui_phosphor::regular as ph;
use headtrack_core::config::ProfileConfig;
use headtrack_core::input::{GazePoint, SourceKind, TobiiState, TobiiStatus};
use headtrack_engine::EngineCmd;

/// Dev fixture: `HEADTRACK_GUI_FAKE_TOBII=<state>` makes the GUI show the Tobii
/// as the head source in `<state>` (or `neutral`: taking its neutral, the
/// hold-still countdown), with a made-up status, while the engine
/// runs whatever it was started with. The engine is never told, and the real
/// device is never opened. It exists so every Tobii-tab state can be
/// screenshotted on a machine with no tracker (P10).
/// Compiled only with the `dev-fixtures` feature (and in tests).
#[cfg(feature = "dev-fixtures")]
pub(crate) const FAKE_TOBII_ENV: &str = "HEADTRACK_GUI_FAKE_TOBII";

#[cfg(any(test, feature = "dev-fixtures"))]
pub(crate) fn fake_tobii_state(value: &str) -> Option<TobiiState> {
    Some(match value {
        "no-device" => TobiiState::NoDevice,
        "permission-denied" => TobiiState::PermissionDenied,
        "busy" => TobiiState::Busy,
        "opening" => TobiiState::Opening,
        "streaming" => TobiiState::Streaming,
        "no-face" => TobiiState::NoFace,
        "lost" => TobiiState::Lost,
        _ => return None,
    })
}

/// The status a fixture name stands for: one of the states above, or
/// `neutral`, which is streaming while the neutral is taken (the hold-still
/// countdown, 40 % held).
#[cfg(any(test, feature = "dev-fixtures"))]
pub(crate) fn fake_tobii_status(value: &str) -> Option<TobiiStatus> {
    if value == "neutral" {
        return Some(fake_neutral_status(0.4));
    }
    fake_tobii_state(value).map(fake_status)
}

/// The fixture status while the neutral is taken, `progress` of it held.
#[cfg(any(test, feature = "dev-fixtures"))]
pub(crate) fn fake_neutral_status(progress: f32) -> TobiiStatus {
    TobiiStatus {
        fps: 0.0,
        last_sample_age_ms: None,
        message: "Hold still, facing the screen: capturing neutral (2 s)".into(),
        neutral_progress: Some(progress),
        ..fake_status(TobiiState::Streaming)
    }
}

/// Dev fixture: `HEADTRACK_GUI_PAGE=<page>` opens the GUI on that page, so
/// screenshots need no input automation. `tuning-move` is Tuning with the
/// translation group selected.
#[cfg(feature = "dev-fixtures")]
pub(crate) const PAGE_ENV: &str = "HEADTRACK_GUI_PAGE";

#[cfg(feature = "dev-fixtures")]
pub(crate) fn fixture_page(value: &str) -> Option<(Tab, TuningGroup)> {
    Some(match value {
        "dashboard" => (Tab::Dashboard, TuningGroup::Look),
        "tuning" => (Tab::Tuning, TuningGroup::Look),
        "tuning-move" => (Tab::Tuning, TuningGroup::Move),
        "profiles" => (Tab::Profiles, TuningGroup::Look),
        "camera" => (Tab::Camera, TuningGroup::Look),
        "tobii" => (Tab::Tobii, TuningGroup::Look),
        "settings" => (Tab::Settings, TuningGroup::Look),
        _ => return None,
    })
}

#[cfg(any(test, feature = "dev-fixtures"))]
pub(crate) fn fake_status(state: TobiiState) -> TobiiStatus {
    let live = matches!(state, TobiiState::Streaming | TobiiState::NoFace);
    TobiiStatus {
        state,
        fps: if live { 33.0 } else { 0.0 },
        last_sample_age_ms: match state {
            TobiiState::Streaming | TobiiState::NoFace => Some(12),
            TobiiState::Lost => Some(2400),
            _ => None,
        },
        serial: (!matches!(state, TobiiState::NoDevice)).then(|| "IS5FF-100000000000".into()),
        gaze: (state == TobiiState::Streaming).then_some(GazePoint { x: 0.62, y: 0.41 }),
        message: match state {
            TobiiState::PermissionDenied => "USB permission denied (udev rule missing)".into(),
            TobiiState::Busy => "USB device busy".into(),
            TobiiState::Lost => "USB transfer error: device disconnected".into(),
            _ => String::new(),
        },
        neutral_progress: None,
    }
}

/// The profile the engine runs for `kind` when it opens that source, read the
/// same way (`source::load_profile_into`): the source's active file, else its
/// built-in default. The GUI mirrors it so the sliders show what is running.
pub(crate) fn active_profile_for(kind: SourceKind) -> ProfileConfig {
    ProfileConfig::load(&ProfileConfig::active_config_path_for(kind))
        .unwrap_or_else(|_| builtin_profile(kind))
}

/// The built-in profile for `kind` — what "Reset" restores. The webcam's
/// Default would switch the Tobii's held translation back on.
pub(crate) fn builtin_profile(kind: SourceKind) -> ProfileConfig {
    match kind {
        SourceKind::Webcam => ProfileConfig::default(),
        SourceKind::Tobii => ProfileConfig::tobii_default(),
    }
}

/// How long the Tobii source needs the user still to take its neutral
/// (`headtrack-input-tobii` `neutral::NEUTRAL_SPAN_US`). Only the countdown's
/// number uses it; the ring is drawn from the source's own progress.
const NEUTRAL_HOLD_S: f32 = 2.0;

/// What the user must do before the Tobii tracks, if anything: drawn over the
/// Dashboard feed and the Tobii tab's head box until tracking starts (D1/D2).
pub(crate) fn tobii_prompt(status: &TobiiStatus) -> Option<ui::SourcePrompt> {
    match (status.state, status.neutral_progress) {
        (TobiiState::Opening, _) => Some(ui::SourcePrompt::Starting),
        (TobiiState::Streaming, Some(p)) => Some(ui::SourcePrompt::HoldStill {
            progress: p.clamp(0.0, 1.0),
            seconds_left: (1.0 - p.clamp(0.0, 1.0)) * NEUTRAL_HOLD_S,
        }),
        (TobiiState::NoFace, Some(_)) => Some(ui::SourcePrompt::FaceTracker),
        _ => None,
    }
}

/// The label and tone for the whole status: the state's, except that a
/// stream still taking its neutral is not tracking yet ("Hold still", amber).
pub(crate) fn status_label(status: &TobiiStatus) -> (&'static str, ui::Tone) {
    if status.state == TobiiState::Streaming && status.neutral_progress.is_some() {
        ("Hold still", ui::Tone::Warn)
    } else {
        (state_label(status.state), state_tone(status.state))
    }
}

/// Short state label, shared by the tab, the Dashboard and the status bar.
pub(crate) fn state_label(state: TobiiState) -> &'static str {
    match state {
        TobiiState::NoDevice => "No tracker found",
        TobiiState::PermissionDenied => "Permission needed",
        TobiiState::Busy => "In use elsewhere",
        TobiiState::Opening => "Starting",
        TobiiState::Streaming => "Tracking",
        TobiiState::NoFace => "No face",
        TobiiState::Lost => "Connection lost",
    }
}

pub(crate) fn state_tone(state: TobiiState) -> ui::Tone {
    match state {
        TobiiState::Streaming => ui::Tone::Ok,
        TobiiState::Opening | TobiiState::NoFace | TobiiState::Lost => ui::Tone::Warn,
        TobiiState::NoDevice | TobiiState::PermissionDenied | TobiiState::Busy => ui::Tone::Err,
    }
}

/// What the user should know or do, per state.
fn state_guidance(state: TobiiState) -> &'static str {
    match state {
        TobiiState::NoDevice => {
            "Plug in the Tobii Eye Tracker 5; it is picked up when it appears."
        }
        TobiiState::PermissionDenied => {
            "Linux will not open the tracker until a USB rule is installed; setup asks for \
             your password once."
        }
        TobiiState::Busy => {
            "Another program, such as a second HeadtrackRS, has the tracker open; close it \
             and this one retries."
        }
        TobiiState::Opening => "Starting the tracker.",
        TobiiState::Streaming => "Tracking your head.",
        TobiiState::NoFace => "No face in view; sit facing the tracker.",
        TobiiState::Lost => {
            "The tracker stopped sending (unplugged, or a USB error); reconnecting."
        }
    }
}

impl HeadtrackApp {
    /// The head source the GUI shows — the engine's, unless the dev fixture
    /// pins the Tobii.
    pub(crate) fn current_source_kind(&self) -> SourceKind {
        if self.fake_tobii.is_some() {
            return SourceKind::Tobii;
        }
        self.state
            .head_source
            .lock()
            .map(|s| s.kind())
            .unwrap_or(SourceKind::Webcam)
    }

    pub(crate) fn tobii_status(&self) -> TobiiStatus {
        if let Some(status) = &self.fake_tobii {
            return status.clone();
        }
        self.state.tobii.lock().map(|s| s.clone()).unwrap_or_default()
    }

    /// Once per frame: when the head source kind changes, the engine has
    /// already loaded that kind's active profile, so the GUI mirrors it (no
    /// push back) and drops the webcam's last preview frame.
    pub(crate) fn sync_source_kind(&mut self) {
        let kind = self.current_source_kind();
        if kind == self.source_kind {
            return;
        }
        self.source_kind = kind;
        let cfg = active_profile_for(kind);
        self.profile_name = cfg.name.clone();
        self.profile = cfg;
        self.profile_dirty = false;
        self.preview_texture = None;
    }

    /// Whether the active source is delivering poses, and if not, the
    /// Dashboard / status-bar words for why. For the Tobii, `camera_connected`
    /// only means its source thread runs; the device state decides.
    ///
    /// The error carries its tone: the Tobii's transient states (starting, no
    /// face, connection lost) are amber, as on the Tobii page header, so the
    /// status bar does not show red for a state the page shows amber (7d).
    pub(crate) fn source_health(&self, connected: bool) -> Result<(), (String, ui::Tone)> {
        match self.source_kind {
            SourceKind::Webcam if connected => Ok(()),
            SourceKind::Webcam => Err(("No camera".into(), ui::Tone::Err)),
            SourceKind::Tobii => {
                let status = self.tobii_status();
                let (label, tone) = status_label(&status);
                let running = connected || self.fake_tobii.is_some();
                if running && status.state == TobiiState::Streaming && status.neutral_progress.is_none() {
                    Ok(())
                } else if running {
                    Err((format!("Tobii: {}", label.to_lowercase()), tone))
                } else {
                    Err(("Tobii not ready".into(), ui::Tone::Err))
                }
            }
        }
    }

    /// The Dashboard's name for the active source.
    pub(crate) fn source_name(&self) -> String {
        match self.source_kind {
            SourceKind::Tobii => "Tobii Eye Tracker 5".into(),
            SourceKind::Webcam => self
                .cameras
                .iter()
                .find(|c| c.index == self.selected_camera)
                .map(|c| c.name.clone())
                .unwrap_or_else(|| format!("/dev/video{}", self.selected_camera)),
        }
    }

    // ── Tobii tab ─────────────────────────────────────────────────────────────

    pub(crate) fn ui_tobii(&mut self, ui: &mut egui::Ui) {
        let active = self.source_kind == SourceKind::Tobii;
        let status = self.tobii_status();

        ui::page_header(ui, "Tobii Eye Tracker 5", |ui| {
            if !headtrack_engine::TOBII_SUPPORTED && self.fake_tobii.is_none() {
                ui::state(ui, "Not in this build", ui::Tone::Idle);
            } else if active {
                let (label, tone) = status_label(&status);
                ui::state(ui, label, tone);
            } else {
                ui::state(ui, "Not in use", ui::Tone::Idle);
            }
        });
        ui::subtitle(ui, "Head rotation only; translation is not available with Tobii.");

        ui::two_columns(ui, "tobii-row", ui::Split::Even, |c| {
            c.group(Some("Device"), |ui| self.ui_tobii_device(ui, active, &status));
            c.group(Some("Head source"), |ui| self.ui_tobii_source(ui, active));
        });

        ui.add_space(ui::ROW_GAP);
        // The head marker is the live sign the tracker is working (3c), and it
        // moves with the pose the sims get. Gaze sits beside it when shown.
        let prompt = if active { tobii_prompt(&status) } else { None };
        let streaming = active && status.state == TobiiState::Streaming && prompt.is_none();
        let pose = *self.state.latest_pose.lock().unwrap_or_else(|e| e.into_inner());
        let rotation = streaming.then_some([pose.yaw, pose.pitch, pose.roll]);
        if SHOW_GAZE {
            ui::two_columns(ui, "tobii-live", ui::Split::Even, |c| {
                c.group(Some("Head"), |ui| {
                    ui::note(ui, "Head rotation, as sent to the sim.", colors::DIM);
                    ui.add_space(8.0);
                    draw_head_box(ui, rotation, prompt);
                });
                c.group(Some("Gaze"), |ui| {
                    ui::note(ui, "Gaze is shown here only; it is never sent to a sim.", colors::DIM);
                    ui.add_space(8.0);
                    let gaze = if active { status.gaze } else { None };
                    draw_gaze_box(ui, gaze);
                });
            });
        } else {
            // One full-width card, the head box centred in it: no empty
            // column where the Gaze card was.
            ui::group(ui, Some("Head"), |ui| {
                ui::note(ui, "Head rotation, as sent to the sim.", colors::DIM);
                ui.add_space(8.0);
                ui.vertical_centered(|ui| draw_head_box(ui, rotation, prompt));
            });
        }
        ui.add_space(8.0);
    }

    fn ui_tobii_device(&mut self, ui: &mut egui::Ui, active: bool, status: &TobiiStatus) {
        if !headtrack_engine::TOBII_SUPPORTED && self.fake_tobii.is_none() {
            ui::note(ui, "This build of HeadtrackRS has no Tobii support.", colors::DIM);
            return;
        }
        if !active {
            // The status is only live while the Tobii is the source; when it is
            // not, the device is closed and its last state means nothing.
            ui::note(ui, "Closed while the webcam is the head source.", colors::DIM);
            return;
        }

        ui::kv_row(ui, "Status", |ui| {
            let (label, tone) = status_label(status);
            ui::state(ui, label, tone);
        });
        let serial = status.serial.clone().unwrap_or_else(|| "\u{2014}".into());
        ui::kv_row(ui, "Serial", |ui| mono(ui, &serial));
        let rate = if status.fps > 0.0 {
            format!("{:.0} Hz", status.fps)
        } else {
            "\u{2014}".into()
        };
        ui::kv_row(ui, "Head-pose rate", |ui| mono(ui, &rate));
        let age = match status.last_sample_age_ms {
            Some(ms) if ms < 1000 => format!("{ms} ms"),
            Some(ms) => format!("{:.1} s", ms as f32 / 1000.0),
            None => "\u{2014}".into(),
        };
        ui::kv_row(ui, "Last sample", |ui| mono(ui, &age));

        ui.add_space(6.0);
        let guidance = if status.state == TobiiState::Streaming && status.neutral_progress.is_some() {
            // Taking the neutral: the stream is up but nothing is tracked yet.
            "Hold still, facing the screen; tracking starts when that is your straight ahead."
        } else {
            state_guidance(status.state)
        };
        ui::note(ui, guidance, colors::TEXT);
        if !status.message.is_empty() {
            ui.add_space(2.0);
            ui.label(
                egui::RichText::new(&status.message)
                    .monospace()
                    .size(colors::TEXT_SM)
                    .color(colors::DIM),
            );
        }
        if status.state == TobiiState::PermissionDenied {
            ui.add_space(8.0);
            self.ui_tobii_usb_setup(ui);
        }
    }

    /// The PermissionDenied state's action: `crate::tobii_setup` installs the
    /// udev rule through pkexec, only on this click.
    fn ui_tobii_usb_setup(&mut self, ui: &mut egui::Ui) {
        use crate::tobii_setup::{rule_installed, SetupState, RULE_PATH};

        self.tobii_setup.poll();
        let state = self.tobii_setup.state().clone();
        let installed = rule_installed();

        let (note, tone) = match &state {
            // Drawn as an activity spinner below, not as a note.
            SetupState::Running => (String::new(), colors::DIM),
            SetupState::Done => (
                "USB rule installed; if the tracker does not start, unplug it and plug it back in."
                    .to_string(),
                ui::Tone::Ok.color(),
            ),
            SetupState::Failed(e) => (e.clone(), colors::WARN),
            SetupState::Idle if installed => (
                format!(
                    "The rule is already at {RULE_PATH} but access is still refused; replug the \
                     tracker, or log out and back in."
                ),
                colors::DIM,
            ),
            SetupState::Idle => (String::new(), colors::DIM),
        };

        let running = state == SetupState::Running;
        let label = match (&state, installed) {
            (SetupState::Failed(_), _) => "Try again",
            (_, true) => "Install the rule again",
            _ => "Set up USB access",
        };
        let clicked = ui::tool_button_enabled(ui, !running, ph::SHIELD_CHECK, label, ui::ButtonKind::Primary)
            .on_hover_text(format!(
                "Installs one udev rule at {RULE_PATH} so your user can open the tracker. \
                 Asks for your password once."
            ))
            .clicked();
        // The screenshot fixture never runs pkexec, whatever is clicked.
        if clicked && self.fake_tobii.is_none() {
            self.tobii_setup.start();
        }
        if running {
            ui.add_space(6.0);
            ui::activity(ui, "Waiting for your password\u{2026}");
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(250));
        }
        if !note.is_empty() {
            ui.add_space(6.0);
            ui::note(ui, &note, tone);
        }
    }

    fn ui_tobii_source(&mut self, ui: &mut egui::Ui, active: bool) {
        let supported = headtrack_engine::TOBII_SUPPORTED || self.fake_tobii.is_some();
        ui::kv_row(ui, "In use", |ui| mono(ui, if active { "Tobii" } else { "Webcam" }));
        let text = if active {
            "Drives yaw, pitch and roll; X, Y and Z are held at zero."
        } else {
            "Switching closes the webcam and loads the Tobii's own profile."
        };
        ui::note(ui, text, colors::DIM);
        ui.add_space(10.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;
            if active {
                if ui::tool_button(ui, ph::CROSSHAIR_SIMPLE, "Recenter", ui::ButtonKind::Normal)
                    .on_hover_text("Take the current head position as straight ahead")
                    .clicked()
                {
                    self.trigger_recenter();
                }
                if ui::tool_button(ui, ph::CAMERA, "Use the webcam", ui::ButtonKind::Ghost).clicked() {
                    self.cmd.send(EngineCmd::SwitchSourceKind { kind: SourceKind::Webcam });
                }
            } else if ui::tool_button_enabled(
                ui,
                supported,
                ph::EYE,
                "Use the Tobii",
                ui::ButtonKind::Primary,
            )
            .clicked()
            {
                self.cmd.send(EngineCmd::SwitchSourceKind { kind: SourceKind::Tobii });
            }
        });
    }
}

/// Whether the Tobii tab shows the Gaze card. Off for v0.2.0: the device marks
/// every gaze sample invalid (869 of 869 in the 2026-11-02 live batch), likely
/// because no display area or calibration has been written to it. Gaze is a
/// future update (docs/KNOWN_BUGS.md); the counters stay in the DEBUG log.
const SHOW_GAZE: bool = false;

/// The gaze box's size: 16:9, at most 320 wide. The head box matches it so
/// the two cards square off.
fn live_box_size(ui: &egui::Ui) -> egui::Vec2 {
    let w = ui.available_width().min(320.0);
    egui::vec2(w, w * 9.0 / 16.0)
}

/// The Tuning pose bubble while the tracker streams; a placeholder otherwise,
/// because a held pose would look like a working tracker.
fn draw_head_box(ui: &mut egui::Ui, rotation: Option<[f32; 3]>, prompt: Option<ui::SourcePrompt>) {
    let size = live_box_size(ui);
    match (rotation, prompt) {
        (_, Some(prompt)) => {
            let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 8.0, colors::PANEL);
            painter.rect_stroke(
                rect,
                8.0,
                egui::Stroke::new(1.0_f32, colors::STROKE_SOFT),
                egui::StrokeKind::Inside,
            );
            ui::source_prompt(ui, &painter, rect, prompt);
        }
        (Some([yaw, pitch, roll]), None) => {
            ui::draw_pose_bubble(ui, yaw, pitch, roll, None, size.x, size.y);
        }
        (None, None) => {
            let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 8.0, colors::PANEL);
            painter.rect_stroke(
                rect,
                8.0,
                egui::Stroke::new(1.0_f32, colors::STROKE_SOFT),
                egui::StrokeKind::Inside,
            );
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No head pose",
                egui::FontId::proportional(colors::TEXT_BODY),
                colors::DIM,
            );
        }
    }
}

/// A 16:9 box standing in for the screen, with a dot where the user looks.
fn draw_gaze_box(ui: &mut egui::Ui, gaze: Option<GazePoint>) {
    let (rect, _) = ui.allocate_exact_size(live_box_size(ui), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 8.0, colors::PANEL);
    painter.rect_stroke(
        rect,
        8.0,
        egui::Stroke::new(1.0_f32, colors::STROKE_SOFT),
        egui::StrokeKind::Inside,
    );
    match gaze {
        Some(g) => {
            let p = egui::pos2(
                rect.left() + g.x.clamp(0.0, 1.0) * rect.width(),
                rect.top() + g.y.clamp(0.0, 1.0) * rect.height(),
            );
            painter.circle_filled(p, 7.0, colors::ORANGE.gamma_multiply(0.35));
            painter.circle_filled(p, 3.5, colors::ORANGE);
        }
        None => {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No gaze",
                egui::FontId::proportional(colors::TEXT_BODY),
                colors::DIM,
            );
        }
    }
}

/// A short value — a serial, a rate, an age — in monospace.
fn mono(ui: &mut egui::Ui, text: &str) {
    ui.label(
        egui::RichText::new(text)
            .monospace()
            .size(colors::TEXT_SM)
            .color(colors::TEXT_HI),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_fixture_name_maps_to_its_state() {
        for (name, want) in [
            ("no-device", TobiiState::NoDevice),
            ("permission-denied", TobiiState::PermissionDenied),
            ("busy", TobiiState::Busy),
            ("opening", TobiiState::Opening),
            ("streaming", TobiiState::Streaming),
            ("no-face", TobiiState::NoFace),
            ("lost", TobiiState::Lost),
        ] {
            assert_eq!(fake_tobii_state(name), Some(want));
            assert_eq!(fake_status(want).state, want);
        }
        assert_eq!(fake_tobii_state("Streaming"), None);
        assert_eq!(fake_tobii_state(""), None);
        let n = fake_tobii_status("neutral").expect("neutral fixture");
        assert_eq!((n.state, n.neutral_progress), (TobiiState::Streaming, Some(0.4)));
    }

    /// D1/D2: every state that is not tracking yet tells the user what to do,
    /// and a tracking or failed source shows no prompt.
    #[test]
    fn the_prompt_covers_starting_and_the_neutral_only() {
        use ui::SourcePrompt as P;
        assert_eq!(tobii_prompt(&fake_status(TobiiState::Opening)), Some(P::Starting));
        assert_eq!(
            tobii_prompt(&fake_neutral_status(0.25)),
            Some(P::HoldStill { progress: 0.25, seconds_left: 1.5 })
        );
        let mut no_face = fake_status(TobiiState::NoFace);
        assert_eq!(tobii_prompt(&no_face), None, "after the neutral: tracking, face lost");
        no_face.neutral_progress = Some(0.0);
        assert_eq!(tobii_prompt(&no_face), Some(P::FaceTracker));
        for s in [
            TobiiState::NoDevice,
            TobiiState::PermissionDenied,
            TobiiState::Busy,
            TobiiState::Streaming,
            TobiiState::Lost,
        ] {
            assert_eq!(tobii_prompt(&fake_status(s)), None, "{s:?}");
        }
        assert_eq!(status_label(&fake_neutral_status(0.5)), ("Hold still", ui::Tone::Warn));
        assert_eq!(status_label(&fake_status(TobiiState::Streaming)).0, "Tracking");
    }

    #[test]
    fn reset_restores_each_sources_own_builtin() {
        assert_eq!(builtin_profile(SourceKind::Webcam).name, ProfileConfig::default().name);
        let t = builtin_profile(SourceKind::Tobii);
        assert_eq!(t.name, ProfileConfig::TOBII_DEFAULT_NAME);
        for i in 3..6 {
            assert_eq!(t.curve[i].sensitivity, 0.0, "Tobii translation must stay off");
        }
    }

    #[test]
    fn gaze_appears_only_while_streaming() {
        for s in [
            TobiiState::NoDevice,
            TobiiState::PermissionDenied,
            TobiiState::Busy,
            TobiiState::Opening,
            TobiiState::NoFace,
            TobiiState::Lost,
        ] {
            assert!(fake_status(s).gaze.is_none(), "{s:?}");
        }
        assert!(fake_status(TobiiState::Streaming).gaze.is_some());
    }
}
