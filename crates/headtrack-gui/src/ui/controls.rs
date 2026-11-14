//! Interactive primitives: buttons, state marks, chips, selectors, parameter
//! cells.
//!
//! **House rule:** there are zero raw `ui.button` / `egui::Button::new` call
//! sites in `tabs/`. Everything interactive routes through here or through
//! [`super::shell`], which is what stops tabs drifting apart visually.
//! HangarRS's `hangar-ui/src/controls.rs` is the sibling of this file; the
//! skins, the state marks and the control geometry are kept identical to it.

use super::theme;
use eframe::egui;
use egui::{Color32, CornerRadius, RichText, Stroke};

/// Indeterminate work: a spinner and what is being waited on. No percentage is
/// implied when the worker has no total.
pub fn activity(ui: &mut egui::Ui, label: &str) {
    ui.horizontal(|ui| {
        ui.add(egui::Spinner::new().size(16.0).color(theme::ORANGE));
        ui.label(RichText::new(label).size(theme::TEXT_BODY).color(theme::TEXT));
    });
}

/// Visual weight of a [`tool_button`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ButtonKind {
    /// Filled accent — one per toolbar, the action the user most likely wants.
    Primary,
    /// Panel fill with a border — the default.
    Normal,
    /// Transparent until hovered — secondary/toggle actions.
    Ghost,
    /// Ghost, but currently on (accent text).
    GhostActive,
    /// Destructive action — outlined in the error colour.
    Danger,
}

/// Icon + label as one laid-out run, so the icon resolves in the icon family
/// while the label stays in the UI font. Phosphor glyphs live in the Private
/// Use Area, which Inter also populates — see [`theme::ICONS`].
///
/// Use for any widget that shows a glyph next to words — a plain
/// `format!("{icon} {text}")` puts the glyph in the proportional chain, where
/// Inter's PUA alternates intercept it.
pub fn icon_text(icon_glyph: &str, text: &str, fg: Color32) -> egui::text::LayoutJob {
    use egui::text::{LayoutJob, TextFormat};
    let mut job = LayoutJob::default();
    let fmt = |font| TextFormat {
        font_id: font,
        color: fg,
        valign: egui::Align::Center,
        ..Default::default()
    };
    if icon_glyph.is_empty() {
        job.append(text, 0.0, fmt(egui::FontId::new(theme::TEXT_BODY, theme::semibold())));
    } else {
        job.append(icon_glyph, 0.0, fmt(egui::FontId::new(15.0, theme::icons())));
        job.append(text, 8.0, fmt(egui::FontId::new(theme::TEXT_BODY, theme::semibold())));
    }
    job
}

/// Toolbar button: Phosphor icon + label, styled by [`ButtonKind`].
pub fn tool_button(ui: &mut egui::Ui, icon: &str, text: &str, kind: ButtonKind) -> egui::Response {
    let (fill, stroke, fg) = button_skin(kind);
    let response = ui.add(
        egui::Button::new(icon_text(icon, text, fg))
            .fill(fill)
            .stroke(stroke)
            .corner_radius(CornerRadius::same(theme::CONTROL_RADIUS))
            .min_size(egui::vec2(0.0, theme::CONTROL_H)),
    );
    focus_outline(ui, &response);
    response
}

/// [`tool_button`] that can be greyed out.
///
/// Wraps `add_enabled_ui` so a disabled action keeps the project's geometry
/// instead of falling back to a bare `egui::Button` of a different shape.
pub fn tool_button_enabled(
    ui: &mut egui::Ui,
    enabled: bool,
    icon: &str,
    text: &str,
    kind: ButtonKind,
) -> egui::Response {
    ui.add_enabled_ui(enabled, |ui| tool_button(ui, icon, text, kind))
        .inner
}

/// Icon-only action with a screen-reader label and a matching tooltip.
///
/// The replacement for `ui.small_button(icon(…))`, which took egui's default
/// height and radius — a different shape from every other button.
pub fn action_icon(ui: &mut egui::Ui, glyph: &str, label: &str) -> egui::Response {
    let response = icon_button(ui, glyph, ButtonKind::Ghost).on_hover_text(label);
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), label));
    response
}

/// Persistent disclosure for detail that would otherwise crowd the page.
pub fn disclosure(
    ui: &mut egui::Ui,
    id: impl std::hash::Hash,
    title: &str,
    add: impl FnOnce(&mut egui::Ui),
) {
    egui::CollapsingHeader::new(
        RichText::new(title)
            .font(egui::FontId::new(theme::TEXT_MD, theme::semibold()))
            .color(theme::TEXT),
    )
    .id_salt(id)
    // Closed in the app. Open in test builds, so the headless render tests
    // measure what is behind every disclosure, not just its header.
    .default_open(cfg!(test))
    .show(ui, add);
}

/// Explicit skins must retain a visible keyboard focus indicator.
pub(crate) fn focus_outline(ui: &egui::Ui, response: &egui::Response) {
    if response.has_focus() {
        ui.painter().rect_stroke(
            response.rect.expand(2.0),
            CornerRadius::same(4),
            Stroke::new(1.0_f32, theme::ORANGE),
            egui::StrokeKind::Outside,
        );
    }
}

/// Semantic colour of a [`state`] mark.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tone {
    Ok,
    Warn,
    Err,
    Accent,
    Idle,
}

impl Tone {
    /// The palette colour this tone paints with.
    pub fn color(self) -> Color32 {
        match self {
            Tone::Ok => theme::OK,
            Tone::Warn => theme::WARN,
            Tone::Err => theme::ERR,
            Tone::Accent => theme::ORANGE,
            Tone::Idle => theme::DIM,
        }
    }

    /// The Phosphor glyph a [`state`] mark paints for this tone.
    ///
    /// All five are from the circle family, so a column of them reads as one
    /// set. Each tone has a different **shape**, so colour is never the sole
    /// carrier of the meaning.
    pub fn glyph(self) -> &'static str {
        use egui_phosphor::regular as icons;
        match self {
            Tone::Ok => icons::CHECK_CIRCLE,
            Tone::Warn => icons::WARNING_CIRCLE,
            Tone::Err => icons::X_CIRCLE,
            Tone::Accent => icons::DOT_OUTLINE,
            Tone::Idle => icons::CIRCLE,
        }
    }
}

/// Fixed box the state glyph is centred in, whatever it is.
pub const GLYPH_SLOT: f32 = 16.0;

/// Point size the glyph is painted at inside [`GLYPH_SLOT`].
const GLYPH_SIZE: f32 = 14.0;

/// Phosphor glyph + word, both in the tone's colour — the row-level echo of
/// the nav rail, which pairs a glyph and a sentence-case label the same way.
///
/// **The word is always drawn**, so colour is never the sole carrier of the
/// meaning. Replaces the old rounded status pill and the uppercase badge.
///
/// The glyph sits in a fixed [`GLYPH_SLOT`] rather than one `LayoutJob` with
/// the word, because Phosphor's glyphs do not share an advance width and a
/// column of mixed tones would go ragged down its left edge.
pub fn state(ui: &mut egui::Ui, text: &str, tone: Tone) -> egui::Response {
    let color = tone.color();
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 8.0;
        let (rect, _) =
            ui.allocate_exact_size(egui::vec2(GLYPH_SLOT, GLYPH_SLOT), egui::Sense::hover());
        ui.painter().text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            tone.glyph(),
            egui::FontId::new(GLYPH_SIZE, theme::icons()),
            color,
        );
        ui.add(
            egui::Label::new(
                RichText::new(text)
                    .font(egui::FontId::new(theme::TEXT_BODY, theme::semibold()))
                    .color(color),
            )
            .selectable(false),
        );
    })
    .response
}

/// A filled circle — status-bar dots, legend markers.
pub fn dot(ui: &mut egui::Ui, color: Color32, size: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    ui.painter().circle_filled(rect.center(), size * 0.5, color);
}

/// On/off chip. Tinted with the supplied colour when on, flat grey when off.
///
/// Also the house replacement for a checkbox: `toggle_chip(ui, "On"/"Off", …)`.
pub fn toggle_chip(ui: &mut egui::Ui, text: &str, on: bool, color: Color32) -> egui::Response {
    let (fill, stroke, fg) = if on {
        (
            color.gamma_multiply(0.16),
            Stroke::new(1.0_f32, color.gamma_multiply(0.55)),
            color,
        )
    } else {
        (theme::WELL, Stroke::new(1.0_f32, theme::STROKE_SOFT), theme::DIM)
    };
    let response = ui.add(
        egui::Button::new(RichText::new(text).color(fg))
            .fill(fill)
            .stroke(stroke)
            .corner_radius(CornerRadius::same(7))
            .min_size(egui::vec2(74.0, 28.0)),
    );
    focus_outline(ui, &response);
    response
}

/// A labelled on/off row: the label on the left, an `On`/`Off` chip held to
/// the right, the pair capped at [`theme::ROW_MAX_WIDTH`]. Flips `on` and
/// returns `true` when clicked.
///
/// The replacement for `ui.checkbox`, whose 16 px box is the one control in
/// the app that does not share [`theme::CONTROL_H`] geometry.
pub fn toggle_row(ui: &mut egui::Ui, label: &str, on: &mut bool) -> egui::Response {
    let mut clicked = None;
    let row = super::layout::kv_row(ui, label, |ui| {
        let r = toggle_chip(ui, if *on { "On" } else { "Off" }, *on, theme::ORANGE);
        clicked = Some(r);
    });
    let mut resp = clicked.unwrap_or(row);
    if resp.clicked() {
        *on = !*on;
        resp.mark_changed();
    }
    resp
}

/// Segmented-control button — the Tuning group selector.
pub fn segment(ui: &mut egui::Ui, text: &str, active: bool) -> egui::Response {
    let response = ui.add(
        egui::Button::new(RichText::new(text).color(if active { Color32::WHITE } else { theme::DIM }))
            .fill(if active { theme::ORANGE_DIM } else { Color32::TRANSPARENT })
            .stroke(if active {
                Stroke::new(1.0_f32, theme::ORANGE.gamma_multiply(0.6))
            } else {
                Stroke::NONE
            })
            .corner_radius(CornerRadius::same(7))
            .min_size(egui::vec2(0.0, 28.0)),
    );
    focus_outline(ui, &response);
    response
}

/// A drop-down selector on the shared control geometry: [`theme::CONTROL_H`]
/// tall, `width` wide (or what is left of the row), its text truncated.
///
/// The caller draws the entries, so a selection can carry a side effect —
/// the camera list re-sends its switch even when the same device is picked.
pub fn combo(
    ui: &mut egui::Ui,
    id_salt: impl std::hash::Hash,
    selected_text: &str,
    width: f32,
    add: impl FnOnce(&mut egui::Ui),
) -> egui::Response {
    let width = width.min(ui.available_width());
    // A box exactly `width` wide. With `truncate()`, egui lays the selected
    // text out against `available_width()` and treats `.width()` only as a
    // minimum, so a long camera name filled the whole column and pushed the
    // refresh button beside it off the page.
    ui.allocate_ui(egui::vec2(width, theme::CONTROL_H), |ui| {
        ui.set_max_width(width);
        ui.spacing_mut().interact_size.y = theme::CONTROL_H;
        egui::ComboBox::from_id_salt(id_salt)
            .selected_text(RichText::new(selected_text).color(theme::TEXT))
            .width(width)
            .truncate()
            .show_ui(ui, add)
            .response
    })
    .inner
}

/// Width of the value box in a [`param_cell`] — room for "−2.00" or "1000"
/// in the button face.
const PARAM_VALUE_W: f32 = 58.0;

/// Room either side of a [`param_cell`] slider for the knob's overhang.
const KNOB_INSET: f32 = 2.0;

/// Gap between the cells of a [`param_row`].
const PARAM_GAP: f32 = 12.0;

/// Labelled slider + drag value. Returns true if the value changed.
///
/// Every tuning control in the app is built from this, so the signature is
/// deliberately unchanged. The cell is exactly as wide as its slider plus its
/// value box, and the label wraps inside that width rather than widening the
/// row past the page.
pub fn param_cell(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    decimals: usize,
) -> bool {
    let cell_w = ui.spacing().slider_width + ui.spacing().item_spacing.x + PARAM_VALUE_W;
    let mut changed = false;
    ui.vertical(|ui| {
        ui.set_width(cell_w);
        ui.label(RichText::new(label).size(theme::TEXT_BODY).color(theme::DIM));
        ui.add_space(1.0);
        ui.horizontal(|ui| {
            // The value box is a fixed slot, and its text is the 15 px button
            // face: at the default 10 px padding "−2.00" and "0.200" outgrew
            // the slot and pushed the row past its column.
            ui.spacing_mut().button_padding.x = 4.0;
            // The knob is drawn at radius + hover expansion + a centred
            // stroke, so at either end of the range it overhangs the slider's
            // rect by up to 1.5 px, and a cell at the page or column edge
            // clipped it. Inset the slider inside the same cell width.
            ui.add_space(KNOB_INSET);
            ui.spacing_mut().slider_width -= 2.0 * KNOB_INSET;
            changed |= ui
                .add(egui::Slider::new(value, range.clone()).show_value(false))
                .changed();
            ui.add_space(KNOB_INSET);
            // Pinned rather than sized to content: a `DragValue` fits itself to
            // the text it happens to be showing, so "1.00" and "0.8" came out
            // different widths and a column of value boxes did not line up.
            changed |= ui
                .add_sized(
                    egui::vec2(PARAM_VALUE_W, ui.spacing().interact_size.y),
                    egui::DragValue::new(value)
                        .fixed_decimals(decimals)
                        .range(range)
                        .speed(0.001),
                )
                .changed();
        });
    });
    changed
}

/// A row of `cells` [`param_cell`]s that divides the available width between
/// them, so the row can never run past its column whatever the window size.
pub fn param_row(ui: &mut egui::Ui, cells: usize, add: impl FnOnce(&mut egui::Ui)) {
    let n = cells.max(1) as f32;
    let cell = ((ui.available_width() - PARAM_GAP * (n - 1.0)) / n).max(0.0);
    ui.horizontal_top(|ui| {
        ui.spacing_mut().item_spacing.x = PARAM_GAP;
        // Two px of slack per cell: egui rounds widget rects to whole pixels.
        ui.spacing_mut().slider_width =
            (cell - PARAM_GAP - PARAM_VALUE_W - 2.0).clamp(40.0, 240.0);
        add(ui);
    });
    ui.add_space(4.0);
}

/// Width of `span` adjacent cells of a `cells`-cell [`param_row`], gaps
/// between them included, so a control above a row can end on its grid.
pub fn param_span(ui: &egui::Ui, cells: usize, span: usize) -> f32 {
    let n = cells.max(1) as f32;
    let cell = ((ui.available_width() - PARAM_GAP * (n - 1.0)) / n).max(0.0);
    let k = span.clamp(1, cells.max(1)) as f32;
    cell * k + PARAM_GAP * (k - 1.0)
}

/// Which end of a [`stepper`] was pressed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Step {
    Dec,
    Inc,
}

/// Square icon-only button, same height and radius as [`tool_button`].
pub fn icon_button(ui: &mut egui::Ui, glyph: &str, kind: ButtonKind) -> egui::Response {
    let (fill, stroke, fg) = button_skin(kind);
    let response = ui.add(
        egui::Button::new(RichText::new(glyph).family(theme::icons()).size(15.0).color(fg))
            .fill(fill)
            .stroke(stroke)
            .corner_radius(CornerRadius::same(theme::CONTROL_RADIUS))
            .min_size(egui::vec2(theme::CONTROL_H, theme::CONTROL_H)),
    );
    focus_outline(ui, &response);
    response
}

/// A `−  value  +` cluster. The caller draws the middle, so the same control
/// serves an editable value (Camera FOV) and a read-only engine mirror (the
/// MSFS seat offsets).
///
/// Returns which end was pressed, if any.
pub fn stepper(ui: &mut egui::Ui, middle: impl FnOnce(&mut egui::Ui)) -> Option<Step> {
    let mut out = None;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        // U+2212 MINUS SIGN, not a hyphen: it matches the "+" optically.
        if step_button(ui, "\u{2212}").clicked() {
            out = Some(Step::Dec);
        }
        middle(ui);
        if step_button(ui, "+").clicked() {
            out = Some(Step::Inc);
        }
    });
    out
}

/// One end of a [`stepper`]. Text rather than an icon — Phosphor has no
/// plus/minus that reads as cleanly at this size.
fn step_button(ui: &mut egui::Ui, glyph: &str) -> egui::Response {
    let (fill, stroke, fg) = button_skin(ButtonKind::Normal);
    let response = ui.add(
        egui::Button::new(RichText::new(glyph).font(egui::FontId::new(15.0, theme::semibold())).color(fg))
            .fill(fill)
            .stroke(stroke)
            .corner_radius(CornerRadius::same(theme::CONTROL_RADIUS))
            .min_size(egui::vec2(theme::CONTROL_H, theme::CONTROL_H)),
    );
    focus_outline(ui, &response);
    response
}

/// Single-line text input on the project's control geometry.
pub fn text_field(ui: &mut egui::Ui, text: &mut String, hint: &str, width: f32) -> egui::Response {
    let width = width.min(ui.available_width()).max(0.0);
    ui.add_sized(
        egui::vec2(width, theme::CONTROL_H),
        egui::TextEdit::singleline(text)
            .hint_text(hint)
            .margin(egui::Margin::symmetric(8, 6))
            .background_color(theme::WELL),
    )
}

/// [`text_field`] for a filesystem path: while it does not have focus and the
/// path is too long for it, it shows the *end* of the path (`…/TesterCSVData`),
/// like every read-only path row. egui's single-line edit always scrolls an
/// unfocused field back to the start, which cut off exactly the part that
/// tells two folders apart (9a). Clicking edits the real text.
pub fn path_field(
    ui: &mut egui::Ui,
    id_salt: &str,
    text: &mut String,
    hint: &str,
    width: f32,
) -> egui::Response {
    const MARGIN: egui::Margin = egui::Margin::symmetric(8, 6);
    let width = width.min(ui.available_width()).max(0.0);
    let id = ui.make_persistent_id(id_salt);
    let font = egui::TextStyle::Body.resolve(ui.style());
    let room = width - f32::from(MARGIN.left) - f32::from(MARGIN.right) - 2.0;
    let focused = ui.memory(|m| m.has_focus(id));
    let elided = (!focused)
        .then(|| super::layout::elide_to_width(ui, text, &font, room))
        .filter(|e| e != text);

    let mut edit = egui::TextEdit::singleline(text)
        .id(id)
        .hint_text(hint)
        .margin(MARGIN)
        .background_color(theme::WELL);
    if elided.is_some() {
        edit = edit.text_color(Color32::TRANSPARENT);
    }
    let mut response = ui.add_sized(egui::vec2(width, theme::CONTROL_H), edit);
    if let Some(shown) = elided {
        response = response.on_hover_text(text.as_str());
        let inner = response.rect.shrink2(egui::vec2(f32::from(MARGIN.left), 0.0));
        ui.painter_at(response.rect).text(
            inner.left_center(),
            egui::Align2::LEFT_CENTER,
            shown,
            font,
            ui.visuals().widgets.inactive.text_color(),
        );
    }
    response
}

/// Fill / stroke / foreground for a [`ButtonKind`], shared by every button
/// primitive so a new kind cannot be styled differently in two places.
fn button_skin(kind: ButtonKind) -> (Color32, Stroke, Color32) {
    match kind {
        ButtonKind::Primary => (theme::ORANGE, Stroke::NONE, theme::BG),
        ButtonKind::Normal => (theme::PANEL_HI, Stroke::new(1.0_f32, theme::STROKE), theme::TEXT),
        ButtonKind::Ghost => (Color32::TRANSPARENT, Stroke::NONE, theme::DIM),
        ButtonKind::GhostActive => (
            theme::ORANGE.gamma_multiply(0.12),
            Stroke::new(1.0_f32, theme::ORANGE.gamma_multiply(0.5)),
            theme::ORANGE,
        ),
        ButtonKind::Danger => (
            theme::ERR.gamma_multiply(0.10),
            Stroke::new(1.0_f32, theme::ERR.gamma_multiply(0.55)),
            theme::ERR,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    fn frame(build: impl FnMut(&mut egui::Ui)) {
        crate::ui::layout::tests::frame(egui::vec2(600.0, 400.0), build);
    }

    /// **The reason [`GLYPH_SLOT`] is fixed.** Phosphor's glyphs do not share an
    /// advance width, so laying the mark out as one `LayoutJob` would put the
    /// word at a different x for every tone, and a column of rows would go
    /// ragged down its left edge.
    #[test]
    fn every_tones_word_starts_at_the_same_x_and_box() {
        let seen: RefCell<Vec<(Tone, egui::Vec2)>> = RefCell::new(Vec::new());
        frame(|ui| {
            for tone in [Tone::Ok, Tone::Warn, Tone::Err, Tone::Accent, Tone::Idle] {
                let resp = state(ui, "linked", tone);
                seen.borrow_mut().push((tone, resp.rect.size()));
            }
        });
        let seen = seen.into_inner();
        assert_eq!(seen.len(), 5, "every tone should have drawn");
        let (_, first) = seen[0];
        for (tone, size) in &seen {
            assert!(
                (size.x - first.x).abs() < 0.5 && (size.y - first.y).abs() < 0.5,
                "{tone:?} is {size:?}, others are {first:?}"
            );
        }
    }

    #[test]
    fn a_selector_uses_the_shared_control_geometry() {
        let rect = RefCell::new(egui::Rect::NOTHING);
        frame(|ui| {
            *rect.borrow_mut() = combo(ui, "geometry-test", "Balanced", 220.0, |_| {}).rect;
        });
        let rect = rect.into_inner();
        assert!((rect.width() - 220.0).abs() < 0.5, "{rect:?}");
        assert!((rect.height() - theme::CONTROL_H).abs() < 0.5, "{rect:?}");
    }

    /// With `truncate()`, egui sized the selected text to the whole row
    /// rather than to `width`, so a long camera name pushed the refresh button
    /// beside it off the page (6a).
    #[test]
    fn a_selector_with_a_long_label_keeps_its_width() {
        let rect = RefCell::new(egui::Rect::NOTHING);
        frame(|ui| {
            ui.horizontal(|ui| {
                let long = "Logitech BRIO 4K Stream Edition with a long V4L2 card name (/dev/video0)";
                *rect.borrow_mut() = combo(ui, "long-label", long, 220.0, |_| {}).rect;
            });
        });
        let rect = rect.into_inner();
        assert!((rect.width() - 220.0).abs() < 0.5, "{rect:?}");
    }

    /// At either end of its range the slider knob must stay inside its cell,
    /// hover expansion and stroke included, or a cell at the page edge clips
    /// it (4a).
    #[test]
    fn a_slider_knob_stays_inside_its_cell_at_both_ends() {
        for v0 in [0.0_f32, 1.0] {
            let ctx = egui::Context::default();
            theme::install(&ctx);
            let cell = RefCell::new(egui::Rect::NOTHING);
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 200.0))),
                ..Default::default()
            };
            let out = ctx.run(input, |ctx| {
                egui::CentralPanel::default().frame(egui::Frame::NONE).show(ctx, |ui| {
                    let mut v = v0;
                    let r = ui.scope(|ui| param_cell(ui, "Centre stability", &mut v, 0.0..=1.0, 2));
                    *cell.borrow_mut() = r.response.rect;
                });
            });
            let cell = cell.into_inner();
            let hover = ctx.style().visuals.widgets.hovered;
            let mut circles = Vec::new();
            fn walk(s: &egui::Shape, out: &mut Vec<egui::epaint::CircleShape>) {
                match s {
                    egui::Shape::Circle(c) => out.push(*c),
                    egui::Shape::Vec(v) => v.iter().for_each(|s| walk(s, out)),
                    _ => {}
                }
            }
            for cs in &out.shapes {
                walk(&cs.shape, &mut circles);
            }
            assert!(!circles.is_empty(), "no knob drawn");
            for c in circles {
                let reach = c.radius + hover.expansion + hover.fg_stroke.width / 2.0;
                assert!(
                    c.center.x - reach >= cell.left() - 0.01 && c.center.x + reach <= cell.right() + 0.01,
                    "value {v0}: knob {:.1}..{:.1} outside cell {:.1}..{:.1}",
                    c.center.x - reach,
                    c.center.x + reach,
                    cell.left(),
                    cell.right()
                );
            }
        }
    }

    /// A row of parameter cells must fit the width it was given, whatever
    /// that width is — this is what keeps the Advanced axis editor inside a
    /// minimum-size window.
    #[test]
    fn a_param_row_fits_its_width() {
        for width in [450.0_f32, 640.0, 900.0] {
            let seen = RefCell::new((0.0_f32, 0.0_f32));
            crate::ui::layout::tests::frame(egui::vec2(width, 400.0), |ui| {
                let left = ui.cursor().min.x;
                let avail = ui.available_width();
                let mut v = [0.5_f32; 3];
                param_row(ui, 3, |ui| {
                    for (i, v) in v.iter_mut().enumerate() {
                        param_cell(ui, &format!("Centre stability {i}"), v, 0.0..=1.0, 2);
                    }
                });
                *seen.borrow_mut() = (ui.min_rect().right() - left, avail);
            });
            let (used, avail) = seen.into_inner();
            assert!(used <= avail + 0.5, "at {width}: row used {used:.1} of {avail:.1} px");
        }
    }
}
