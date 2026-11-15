//! Shared visual language for the HeadtrackRS GUI.
//!
//! egui is immediate-mode: there is no retained widget tree to restyle, so
//! "the design system" is this module. Tab code composes these primitives
//! instead of hand-rolling frames and painters, which is what kept the old
//! tabs drifting apart visually.
//!
//! It is the sibling of HangarRS's `hangar-ui` crate (which was ported from
//! here and has since moved ahead); `docs/UI_FINISH.md` records the round that
//! brought the two back into line. Three house rules:
//!
//! * **Zero raw `ui.button` / `egui::Button::new` in `tabs/`.** Everything
//!   interactive routes through [`controls`] or [`shell`].
//! * **No containers.** A sentence-case heading and air group things; nothing
//!   in [`layout`] draws a box.
//! * **Five text sizes** ([`theme::TEXT_SM`] … [`theme::TEXT_XL`]) in a page
//!   body, and no numeric size literal in `tabs/`.

pub mod controls;
pub mod gauge;
pub mod layout;
pub mod shell;
pub mod theme;

pub use controls::{
    action_icon, activity, combo, disclosure, icon_button, icon_text, param_cell, param_row,
    param_span, path_field, segment, state, stepper, text_field, toggle_chip, toggle_row,
    tool_button, tool_button_enabled, ButtonKind, Step, Tone,
};
pub use gauge::{
    axis_gauge, draw_bar, draw_curve_editor, draw_pose_bubble, hud_overlay, paint_pose_bubble,
    source_prompt, SourcePrompt,
};
pub use layout::{
    group, group_rule, heading, kv_row, note, page_header, path_label, section, subheading,
    subtitle, two_columns, Split, ROW_GAP,
};

use eframe::egui::{self, Color32};

/// `value` to `decimals` places with an explicit sign, and no sign at all when
/// it rounds to zero.
///
/// Every signed readout goes through this. Rust formats negative zero, and
/// anything that rounds to it (−0.04 at one place), as `-0.0`, which is what
/// the Tobii's held-at-zero translation showed on the Dashboard and Advanced.
pub fn signed(value: f32, decimals: usize) -> String {
    let magnitude = format!("{:.*}", decimals, value.abs());
    if magnitude.bytes().all(|b| b == b'0' || b == b'.') {
        magnitude
    } else if value < 0.0 {
        format!("-{magnitude}")
    } else {
        format!("+{magnitude}")
    }
}

/// Per-axis colours, three shades within each family.
///
/// The family split (rotation blue / translation red) is the mockup's grouping
/// device; the shades preserve the per-axis identity the curve editor and live
/// axis rows rely on.
pub const AXIS_COLORS: [Color32; 6] = [
    Color32::from_rgb(0x61, 0x9b, 0xf6), // yaw   — rotation base
    Color32::from_rgb(0x8f, 0xb8, 0xf9), // pitch — rotation light
    Color32::from_rgb(0x3f, 0x79, 0xd4), // roll  — rotation deep
    Color32::from_rgb(0xee, 0x5e, 0x5e), // x     — translation base
    Color32::from_rgb(0xf5, 0x90, 0x90), // y     — translation light
    Color32::from_rgb(0xc9, 0x40, 0x40), // z     — translation deep
];

/// Colour for axis `i` (0..6), saturating at Z for out-of-range indices.
pub fn axis_color(i: usize) -> Color32 {
    AXIS_COLORS[i.min(5)]
}

/// Family colour for axis `i`: rotation blue for 0–2, translation red for 3–5.
pub fn axis_family_color(i: usize) -> Color32 {
    if i < 3 {
        theme::ROT
    } else {
        theme::TRN
    }
}

/// Wrap a central panel's content in the project's page geometry: a scroll
/// area, a width-capped column, and the overflow fade under a floating
/// scrollbar.
///
/// Every tab goes through this, so no tab can quietly grow a different content
/// width, and every tab reports what it measured.
pub fn page(ui: &mut egui::Ui, add: impl FnOnce(&mut egui::Ui)) -> PageMetrics {
    layout::take_column_overflow(ui.ctx());
    let out = egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            let width = ui
                .available_width()
                .clamp(theme::CONTENT_MIN_WIDTH, theme::CONTENT_MAX_WIDTH);
            ui.set_max_width(width);
            add(ui);
        });

    // Overflow affordance. The scrollbars float, so without this a tab that
    // runs past the window looks truncated rather than scrollable.
    let hidden_below = out.content_size.y - out.state.offset.y - out.inner_rect.height();
    layout::scroll_fade(ui.painter(), out.inner_rect, hidden_below);

    PageMetrics {
        content_width: out.content_size.x,
        available_width: out.inner_rect.width(),
        column_overflow: layout::take_column_overflow(ui.ctx()),
    }
}

/// What a page's content actually measured.
///
/// **A panel that runs past the right edge cannot be seen in a headless render
/// test**: egui clips it and the test still passes. The usual cause is a label
/// inside `ui.horizontal`, which does not wrap but extends the row. Measuring
/// the content turns that into a failing test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PageMetrics {
    /// How wide the content wanted to be.
    pub content_width: f32,
    /// How wide it was allowed to be.
    pub available_width: f32,
    /// How far the worst [`two_columns`] column drew past its own slot. The
    /// slot clips it and the row claims only its own width, so this is not in
    /// `content_width`.
    pub column_overflow: f32,
}

impl PageMetrics {
    /// How far past the right edge the content ran. Zero when it fits.
    ///
    /// A pixel of slack: rounding in a scroll area's own measurement is not a
    /// layout bug.
    pub fn overflow(&self) -> f32 {
        let page = self.content_width - self.available_width;
        (page.max(self.column_overflow) - 1.0).max(0.0)
    }
}

#[cfg(test)]
mod signed_tests {
    use super::signed;

    #[test]
    // `0.0 * -1.0` is kept on purpose: it is negative zero *computed*, the way
    // it reaches `signed` in use; clippy's `-0.0` would repeat the first line.
    #[allow(clippy::neg_multiply)]
    fn negative_zero_and_values_that_round_to_it_have_no_sign() {
        assert_eq!(signed(-0.0, 1), "0.0");
        assert_eq!(signed(0.0, 1), "0.0");
        assert_eq!(signed(-0.04, 1), "0.0");
        assert_eq!(signed(0.0 * -1.0, 0), "0");
        assert_eq!(signed(-0.4, 0), "0");
    }

    #[test]
    fn everything_else_keeps_its_sign() {
        assert_eq!(signed(-0.06, 1), "-0.1");
        assert_eq!(signed(1.25, 1), "+1.2");
        assert_eq!(signed(37.0, 0), "+37");
        assert_eq!(signed(-15.0, 0), "-15");
    }
}
