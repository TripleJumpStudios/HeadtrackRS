//! The application chrome: brand block, nav rail rows, status-bar items.
//!
//! These live in the design system rather than in `main.rs` for the same
//! reason every other control does — the nav row is a button, and the house
//! rule is that no raw `egui::Button::new` exists outside `ui/`. Ported from
//! HangarRS's `hangar-ui/src/shell.rs`.

use super::{controls::Tone, theme};
use eframe::egui;
use egui::{Color32, CornerRadius, RichText, Stroke};

/// Width of the left nav rail.
pub const NAV_WIDTH: f32 = 174.0;

/// Height of the bottom status bar.
pub const STATUS_HEIGHT: f32 = 30.0;

/// Height of one nav row.
const NAV_ROW_H: f32 = 40.0;

/// The product name as the wordmark: "Headtrack" in the text colour and "RS"
/// in the accent, in Noto Sans SemiBold, the way HangarRS sets its own name
/// (`website/index.html`: system-ui at weight 600, the suffix in `#ff8c00`).
pub fn wordmark(size: f32) -> egui::text::LayoutJob {
    let font = egui::FontId::new(size, theme::wordmark());
    let mut job = egui::text::LayoutJob::default();
    for (part, color) in [("Headtrack", theme::TEXT_HI), ("RS", theme::ORANGE)] {
        job.append(part, 0.0, egui::TextFormat::simple(font.clone(), color));
    }
    job
}

/// The brand block at the top of the nav rail: the app logo above the
/// wordmark. No tagline (maintainer, 2026-11-02: "ditch the subtext").
pub fn brand_block(ui: &mut egui::Ui, logo: egui::load::SizedTexture) {
    ui.vertical_centered(|ui| {
        ui.add(egui::Image::new(logo));
        ui.add_space(10.0);
        ui.add(egui::Label::new(wordmark(theme::TEXT_XL)).extend());
    });
}

/// Rail height (inside its margins) the full [`brand_block`], the longest nav
/// list and the footer need without touching. Below it the rail uses
/// [`brand_block_compact`]. Measured as `available_height()` at the top of the
/// rail with the 2026-08 brand block (logo, name and a tagline): the full
/// stack needed 614 px (it overran a 608 px rail by 6), plus 8 px of air. The
/// tagline has since gone, so this is now conservative.
/// `the_rail_footer_never_overlaps_the_nav_list` sweeps the window heights
/// around it.
pub const RAIL_FULL_H: f32 = 622.0;

/// [`brand_block`] for a short window: the logo beside the wordmark rather
/// than above it.
pub fn brand_block_compact(ui: &mut egui::Ui, logo: egui::load::SizedTexture) {
    ui.horizontal(|ui| {
        ui.add_space(4.0);
        ui.add(egui::Image::new(egui::load::SizedTexture::new(logo.id, egui::vec2(36.0, 36.0))));
        ui.add_space(4.0);
        ui.add(egui::Label::new(wordmark(theme::TEXT_MD)).extend());
    });
}

/// One nav rail row: icon + label, accent bar and raised fill when active.
///
/// The active marker is painted into the rail's left *margin*, outside the
/// button rect, so the label text does not shift when a row becomes active.
pub fn nav_item(ui: &mut egui::Ui, glyph: &str, label: &str, active: bool) -> egui::Response {
    let fg = if active { theme::ORANGE } else { theme::DIM };

    // Icon and label are separate runs: the icon must resolve in the icon
    // family, not the proportional chain (see `theme::ICONS`).
    let mut job = egui::text::LayoutJob::default();
    let fmt = |font| egui::TextFormat {
        font_id: font,
        color: fg,
        valign: egui::Align::Center,
        ..Default::default()
    };
    job.append(glyph, 0.0, fmt(egui::FontId::new(16.0, theme::icons())));
    job.append(label, 10.0, fmt(egui::FontId::proportional(theme::TEXT_MD)));

    let background = ui.painter().add(egui::Shape::Noop);
    let response = ui.add(
        egui::Button::new(job)
            .fill(if active { theme::PANEL_HI } else { Color32::TRANSPARENT })
            .stroke(Stroke::NONE)
            .corner_radius(CornerRadius::same(4))
            .min_size(egui::vec2(ui.available_width(), NAV_ROW_H)),
    );
    if response.hovered() && !active {
        ui.painter().set(
            background,
            egui::Shape::rect_filled(response.rect, CornerRadius::same(4), theme::PANEL),
        );
    }
    super::controls::focus_outline(ui, &response);
    response.widget_info(|| {
        egui::WidgetInfo::selected(egui::WidgetType::SelectableLabel, ui.is_enabled(), active, label)
    });

    if active {
        let r = response.rect;
        let accent = egui::Rect::from_min_max(
            egui::pos2(r.left() - 12.0, r.center().y - 10.0),
            egui::pos2(r.left() - 9.0, r.center().y + 10.0),
        );
        ui.painter().rect_filled(accent, 2.0, theme::ORANGE);
    }
    ui.add_space(2.0);
    response
}

/// Hairline between nav groups, inset from the rail edges.
pub fn nav_separator(ui: &mut egui::Ui) {
    ui.add_space(9.0);
    let width = (ui.available_width() - 16.0).max(0.0);
    ui.horizontal(|ui| {
        ui.add_space(8.0);
        let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 1.0), egui::Sense::hover());
        ui.painter()
            .hline(rect.x_range(), rect.center().y, Stroke::new(1.0_f32, theme::STROKE_SOFT));
    });
    ui.add_space(9.0);
}

/// Vertical hairline between status-bar items.
pub fn status_sep(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(1.0, 12.0), egui::Sense::hover());
    ui.painter()
        .vline(rect.center().x, rect.y_range(), Stroke::new(1.0_f32, theme::STROKE));
}

/// Status-bar item with a leading state dot.
pub fn status_dot(ui: &mut egui::Ui, text: &str, tone: Tone) {
    super::controls::dot(ui, tone.color(), 7.0);
    ui.label(RichText::new(text).small().color(theme::TEXT));
}

/// Monospaced chrome figure — counts, versions, rates, socket names.
pub fn status_mono(ui: &mut egui::Ui, text: &str, color: Color32) {
    ui.label(RichText::new(text).monospace().size(12.0).color(color));
}
