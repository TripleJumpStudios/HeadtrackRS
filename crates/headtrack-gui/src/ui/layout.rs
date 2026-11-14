//! Structural primitives: page headers, sections, groups, key/value rows,
//! dividers, the two-column page grid and the scroll fade.
//!
//! Ported back from HangarRS's `hangar-ui/src/layout.rs` (itself a port of this
//! file), and held to the same rule: **no containers.** A sentence-case heading
//! and air do the grouping, the way the nav rail does it. There is no boxed
//! card left in this module, so a box cannot be nested inside another one.

use super::theme;
use eframe::egui;
use egui::{Align, Layout, RichText, Stroke};

/// Group heading — sentence case, [`theme::TEXT_LG`] semibold.
///
/// [`group`] has no box, so these words carry the grouping on their own.
pub fn heading(ui: &mut egui::Ui, text: &str) {
    ui.label(
        RichText::new(text)
            .font(egui::FontId::new(theme::TEXT_LG, theme::semibold()))
            .color(theme::TEXT_HI),
    );
}

/// A label for a cluster of rows *inside* a group — one rung below
/// [`heading`], at [`theme::TEXT_MD`] semibold (a row's own title size).
///
/// Replaces the tracked-out uppercase eyebrows the GUI used to group rows
/// with. The ladder is section (XL) › group heading (LG) › this (MD) › body.
pub fn subheading(ui: &mut egui::Ui, text: &str) {
    ui.label(
        RichText::new(text)
            .font(egui::FontId::new(theme::TEXT_MD, theme::semibold()))
            .color(theme::TEXT_HI),
    );
}

/// Standard page header: title on the left, an action/state cluster right.
///
/// No kicker line above the title: the nav rail already groups the tabs.
pub fn page_header(ui: &mut egui::Ui, title: &str, right: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(title).heading().color(theme::TEXT_HI));
        ui.with_layout(Layout::right_to_left(Align::Center), right);
    });
    ui.add_space(12.0);
}

/// One consequence line under a page header — what the page's controls act
/// on. Never rationale: a subtitle that explains belongs in the docs.
pub fn subtitle(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).size(theme::TEXT_BODY).color(theme::DIM));
    ui.add_space(10.0);
}

/// Body prose — one sentence of consequence or failure, [`theme::TEXT_BODY`]
/// in `color` (`DIM` or brighter; `DIMMER` is for single words).
///
/// Always in the caller's vertical flow, where it wraps. Inside
/// `ui.horizontal` a label does not wrap; it extends the row past the page.
pub fn note(ui: &mut egui::Ui, text: &str, color: egui::Color32) -> egui::Response {
    ui.label(RichText::new(text).size(theme::TEXT_BODY).color(color))
}

// ── The page grid ────────────────────────────────────────────────────────────
//
// Every tab lays out on one grid, so the same group is the same fraction of
// the window whichever tab it is on. A hand-rolled split per tab is how they
// drifted: 0.42 on Dashboard, 0.54 on Tuning, 0.44 on Camera, once.

/// The single gap between the two columns of a [`two_columns`] row.
///
/// Wider than HangarRS's 14 px because the columns are no longer boxed: with
/// no card edge to stop the eye, the gutter alone has to separate them.
pub const GUTTER: f32 = 28.0;

/// Space between a page header and the first row, and between rows.
///
/// [`group`] has no outline, so this gap is the only thing separating two
/// stacked groups.
pub const ROW_GAP: f32 = 18.0;

/// How a two-column row divides its width.
///
/// Named choices rather than a free fraction: the ratio is a design decision
/// that should be made once and reused, not re-invented per tab.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Split {
    /// 50/50 — two peers of comparable weight.
    Even,
    /// 58/42 — a visual lead (camera feed, plot) beside a column of readouts.
    Lead,
}

impl Split {
    /// Fraction of the content width (gutter already removed) taken by the
    /// lead column.
    fn lead_fraction(self) -> f32 {
        match self {
            Split::Even => 0.50,
            Split::Lead => 0.58,
        }
    }
}

/// Geometry of a completed [`two_columns`] row.
pub struct RowGeom {
    /// Content height of the lead column as drawn this frame.
    pub lead_height: f32,
    /// Height of the row — the taller of the two columns.
    pub row_height: f32,
}

/// Column builder handed to [`two_columns`]. Call [`Cols::group`] twice.
///
/// One builder rather than two closure arguments: tab code captures
/// `&mut self`, and two closures passed side by side would both hold that
/// borrow at once. Adding columns in sequence keeps only one alive at a time.
pub struct Cols<'u> {
    ui: &'u mut egui::Ui,
    /// Top-left of the row, in screen space. Both columns are placed from here.
    origin: egui::Pos2,
    widths: [f32; 2],
    /// Height available to the row, used as the slot height before the content
    /// decides its own.
    height: f32,
    /// Clip rect of the parent, captured before the first child borrows it.
    parent_clip: egui::Rect,
    next: usize,
    /// Drawn content rect, per column.
    slots: [Option<egui::Rect>; 2],
}

impl Cols<'_> {
    /// Add the next column as a [`group`]: optional heading, then content.
    ///
    /// The slot rect is computed from the row origin, not from a cursor the
    /// previous column advanced: otherwise one over-wide child (a label in a
    /// layout that does not wrap) pushes the second column off the window. The
    /// clip rect keeps the painting inside the slot too.
    pub fn group(&mut self, label: Option<&str>, add: impl FnOnce(&mut egui::Ui)) {
        let i = self.next.min(1);
        let x = self.origin.x + if i == 0 { 0.0 } else { self.widths[0] + GUTTER };
        let slot = egui::Rect::from_min_size(
            egui::pos2(x, self.origin.y),
            egui::vec2(self.widths[i], self.height),
        );
        // Clip at the column's sides only. The slot's height is whatever the
        // parent had available, which a caller may legitimately make zero
        // (the Dashboard's Live pose row, sized by its content): clipping to
        // it hid every gauge while they were still painted.
        let clip = egui::Rect::from_x_y_ranges(slot.x_range(), self.parent_clip.y_range())
            .intersect(self.parent_clip);

        let mut child = self.ui.new_child(
            egui::UiBuilder::new()
                .max_rect(slot)
                .layout(Layout::top_down(Align::Min)),
        );
        child.set_clip_rect(clip);
        child.set_width(slot.width());
        if let Some(label) = label {
            heading(&mut child, label);
            child.add_space(8.0);
        }
        add(&mut child);

        // The clip rect hides a column that drew past its slot, and the row
        // claims only its own width, so nothing downstream would see it.
        // Report it to `page` instead, which folds it into `PageMetrics`.
        let over = child.min_rect().right() - slot.right();
        if over > 0.0 {
            note_column_overflow(self.ui.ctx(), over);
        }

        self.slots[i] = Some(child.min_rect());
        self.next += 1;
    }
}

/// Memory key for the widest column overflow drawn since the last [`take_column_overflow`].
fn column_overflow_id() -> egui::Id {
    egui::Id::new("ui::layout::column_overflow")
}

fn note_column_overflow(ctx: &egui::Context, px: f32) {
    let id = column_overflow_id();
    ctx.data_mut(|d| {
        let worst: f32 = d.get_temp(id).unwrap_or(0.0);
        d.insert_temp(id, worst.max(px));
    });
}

/// How far the worst [`two_columns`] column ran past its slot since the last
/// call, and reset it. [`super::page`] calls this around its content.
pub fn take_column_overflow(ctx: &egui::Context) -> f32 {
    ctx.data_mut(|d| d.remove_temp::<f32>(column_overflow_id()).unwrap_or(0.0))
}

/// Two columns on the page grid.
///
/// Columns are placed at an exact size from the row origin: inside a
/// `horizontal_top`, `set_width` is only advisory, and rows came out several
/// px short of a full-width group below them.
pub fn two_columns(
    ui: &mut egui::Ui,
    id_source: &str,
    split: Split,
    build: impl FnOnce(&mut Cols<'_>),
) -> RowGeom {
    let id = ui.id().with(id_source);
    let prev: f32 = ui.data(|d| d.get_temp(id)).unwrap_or(0.0);

    let content = (ui.available_width() - GUTTER).max(0.0);
    let lead_w = (content * split.lead_fraction()).max(0.0);
    let widths = [lead_w, (content - lead_w).max(0.0)];

    let origin = ui.cursor().min;
    let height = ui.available_height().max(0.0);
    let parent_clip = ui.clip_rect();

    let slots = {
        let mut cols = Cols {
            ui,
            origin,
            widths,
            height,
            parent_clip,
            next: 0,
            slots: [None, None],
        };
        build(&mut cols);
        cols.slots
    };

    let row_height = slots
        .iter()
        .flatten()
        .map(|r| r.height())
        .fold(0.0_f32, f32::max);

    // The columns were drawn into child Uis, which do not move the parent's
    // cursor. Claim the row explicitly so whatever follows starts below it.
    ui.allocate_rect(
        egui::Rect::from_min_size(origin, egui::vec2(content + GUTTER, row_height)),
        egui::Sense::hover(),
    );

    // Stored purely so a caller converging its own content height (the
    // Dashboard feed) has last frame's row to aim at. Nothing in this function
    // reads it back, so it cannot influence layout.
    if (row_height - prev).abs() > 0.5 {
        ui.data_mut(|d| d.insert_temp(id, row_height));
        ui.ctx().request_repaint();
    }

    RowGeom {
        lead_height: slots[0].map_or(0.0, |r| r.height()),
        row_height,
    }
}

/// A named group of rows aligned to the page margin.
///
/// No box: a heading and air do the grouping, as on the nav rail. [`section`]
/// is the heavier tool for a page that needs more separation than a heading
/// gives it.
pub fn group<R>(ui: &mut egui::Ui, label: Option<&str>, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    ui.vertical(|ui| {
        // Fill the column rather than shrink-wrapping the content, so a group
        // holding two sliders lines up with the group above it.
        ui.set_width(ui.available_width());
        if let Some(label) = label {
            heading(ui, label);
            ui.add_space(8.0);
        }
        add(ui)
    })
    .inner
}

/// Section heading: orange tick + title at [`theme::TEXT_XL`] — the page's
/// spine.
pub fn section(ui: &mut egui::Ui, label: &str) {
    ui.add_space(14.0);
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_at_least(egui::vec2(3.0, 19.0), egui::Sense::hover());
        ui.painter().rect_filled(rect, 2.0, theme::ORANGE);
        ui.add_space(6.0);
        ui.label(
            RichText::new(label)
                .font(egui::FontId::new(theme::TEXT_XL, theme::semibold()))
                .color(theme::TEXT_HI),
        );
    });
    ui.add_space(8.0);
}

/// Key on the left, caller-drawn value on the right.
///
/// No hairline under the row: the gap does that job for a stack of short rows.
/// Capped at [`theme::ROW_MAX_WIDTH`], not the page's width: a right-to-left
/// value takes whatever it is given, which would put key and value at
/// opposite margins of the page.
pub fn kv_row(ui: &mut egui::Ui, key: &str, value: impl FnOnce(&mut egui::Ui)) -> egui::Response {
    let width = ui.available_width().min(theme::ROW_MAX_WIDTH);
    let resp = ui
        .allocate_ui_with_layout(
            egui::vec2(width, 0.0),
            Layout::left_to_right(Align::Center),
            |ui| {
                ui.set_max_width(width);
                ui.add(
                    egui::Label::new(RichText::new(key).size(theme::TEXT_BODY).color(theme::DIM))
                        .selectable(false),
                );
                ui.with_layout(Layout::right_to_left(Align::Center), value);
            },
        )
        .response;
    ui.add_space(6.0);
    resp
}

/// Group heading inside the pose cluster: a sentence-case title in the axis
/// family's colour, a dim unit, then a rule filling the remaining width.
pub fn group_rule(ui: &mut egui::Ui, title: &str, unit: &str, color: egui::Color32) {
    ui.horizontal(|ui| {
        // Set rather than inherited: callers that zero `item_spacing.x` to make
        // a column layout exact would otherwise run the title into the unit.
        ui.spacing_mut().item_spacing.x = 8.0;
        ui.label(
            RichText::new(title)
                .font(egui::FontId::new(theme::TEXT_MD, theme::semibold()))
                .color(color),
        );
        ui.add(
            egui::Label::new(RichText::new(unit).size(theme::TEXT_SM).color(theme::DIM))
                .truncate(),
        );
        let width = ui.available_width();
        if width > 4.0 {
            let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 1.0), egui::Sense::hover());
            ui.painter().hline(
                rect.x_range(),
                rect.center().y,
                Stroke::new(1.0_f32, theme::STROKE_SOFT),
            );
        }
    });
    ui.add_space(4.0);
}

/// Elide `text` from the left so it fits `max_width` in `font`.
///
/// Measured rather than counted: a character limit is only right for one
/// column width, and a right-aligned run that does not fit grows *leftwards*,
/// over its own label. The candidate is laid out and trimmed until it fits,
/// because neither a glyph width nor a character count survives contact with
/// egui 0.31: it rounds every glyph advance to whole pixels (7.8 → 8 px at
/// 13 px mono), and JetBrains Mono has no `…`, which falls back to a wider
/// face.
pub fn elide_to_width(ui: &egui::Ui, text: &str, font: &egui::FontId, max_width: f32) -> String {
    let measure = |s: &str| {
        ui.fonts(|f| {
            f.layout_no_wrap(s.to_owned(), font.clone(), egui::Color32::WHITE)
                .rect
                .width()
        })
    };
    if measure(text) <= max_width {
        return text.to_owned();
    }
    let glyph = (measure("0000000000") / 10.0).max(1.0);
    let mut keep = (max_width / glyph).floor().max(0.0) as usize + 1;
    loop {
        let candidate = elide_left(text, keep);
        if keep == 0 || measure(&candidate) <= max_width {
            return candidate;
        }
        keep -= 1;
    }
}

/// A path as a monospace value, elided from the left to `max_width`, full path
/// on hover. The distinguishing part of a path is its tail, so that is what
/// survives.
pub fn path_label(ui: &mut egui::Ui, path: &str, max_width: f32, color: egui::Color32) -> egui::Response {
    let font = egui::FontId::monospace(theme::TEXT_SM);
    let shown = elide_to_width(ui, path, &font, max_width);
    ui.add(egui::Label::new(RichText::new(shown).font(font).color(color)))
        .on_hover_text(path)
}

/// Trim to at most `max` characters, keeping the **end**.
fn elide_left(s: &str, max: usize) -> String {
    let chars: Vec<char> = s.chars().collect();
    if chars.len() <= max {
        return s.to_owned();
    }
    if max <= 1 {
        return "…".repeat(max);
    }
    let tail: String = chars[chars.len() - (max - 1)..].iter().collect();
    format!("…{tail}")
}

/// Fade the bottom edge of a scrolling region to signal content below.
///
/// Scrollbars here are floating, so a tab whose content runs past the window
/// otherwise looks truncated rather than scrollable. A fade costs no layout
/// space and keeps all four page insets equal.
///
/// `remaining` is how much content is still hidden; the fade eases in over the
/// first `FADE_H` of it so arriving at the bottom does not end with a hard
/// switch-off.
pub fn scroll_fade(painter: &egui::Painter, viewport: egui::Rect, remaining: f32) {
    const FADE_H: f32 = 28.0;

    if remaining <= 0.5 || viewport.height() <= FADE_H {
        return;
    }
    let strength = (remaining / FADE_H).clamp(0.0, 1.0);

    let top = viewport.bottom() - FADE_H;
    let clear =
        egui::Color32::from_rgba_unmultiplied(theme::BG.r(), theme::BG.g(), theme::BG.b(), 0);
    let solid = theme::BG.gamma_multiply(strength);

    // A vertex-coloured mesh rather than a stack of thin rectangles: egui
    // interpolates in premultiplied alpha, so this is one shape and a true
    // gradient instead of visible banding.
    let mut mesh = egui::Mesh::default();
    mesh.colored_vertex(egui::pos2(viewport.left(), top), clear);
    mesh.colored_vertex(egui::pos2(viewport.right(), top), clear);
    mesh.colored_vertex(egui::pos2(viewport.left(), viewport.bottom()), solid);
    mesh.colored_vertex(egui::pos2(viewport.right(), viewport.bottom()), solid);
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(1, 2, 3);
    painter.add(egui::Shape::mesh(mesh));
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::cell::RefCell;

    /// Run one frame of a root `Ui` with no window or GPU behind it, the real
    /// fonts installed.
    pub(crate) fn frame(size: egui::Vec2, mut build: impl FnMut(&mut egui::Ui)) {
        let ctx = egui::Context::default();
        theme::install(&ctx);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, size)),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| build(ui));
        });
    }

    /// A `kv_row` puts its value in a right-to-left layout, which takes every
    /// pixel it is offered, so uncapped the key and value sit at opposite
    /// margins of the page. It is capped at [`theme::ROW_MAX_WIDTH`] whatever
    /// the page is.
    #[test]
    fn a_key_and_its_value_never_end_up_at_opposite_margins() {
        for page in [1600.0_f32, 900.0, 620.0, 400.0] {
            let seen: RefCell<Option<(f32, f32)>> = RefCell::new(None);
            frame(egui::vec2(page, 400.0), |ui| {
                let row_left = ui.cursor().min.x;
                kv_row(ui, "Pose socket", |ui| {
                    let r = ui.label("headtrack.sock");
                    *seen.borrow_mut() = Some((r.rect.right(), row_left));
                });
            });
            let (value_right, row_left) = seen.into_inner().expect("the row should have drawn");
            let span = value_right - row_left;
            assert!(
                span <= theme::ROW_MAX_WIDTH + 0.5,
                "at a {page} px page the value ends {span:.0} px from the key, past the \
                 {} px row measure",
                theme::ROW_MAX_WIDTH
            );
        }
    }

    /// Lay out one row and report `(lead, trail, available_width)`, with the
    /// lead group asked to draw `lead_content_w` px of content.
    fn row_geometry(lead_content_w: f32) -> (egui::Rect, egui::Rect, f32) {
        let seen: RefCell<Vec<egui::Rect>> = RefCell::new(Vec::new());
        let avail = RefCell::new(0.0_f32);

        frame(egui::vec2(1000.0, 700.0), |ui| {
            *avail.borrow_mut() = ui.available_width();
            two_columns(ui, "t", Split::Lead, |cols| {
                cols.group(None, |ui| {
                    seen.borrow_mut().push(ui.max_rect());
                    ui.allocate_space(egui::vec2(lead_content_w, 20.0));
                });
                cols.group(None, |ui| {
                    seen.borrow_mut().push(ui.max_rect());
                });
            });
        });

        let seen = seen.into_inner();
        assert_eq!(seen.len(), 2, "both columns should have drawn");
        (seen[0], seen[1], avail.into_inner())
    }

    /// If the cursor advanced by the width the first column actually *drew*,
    /// one over-wide child would push the second column past the window edge.
    /// The trailing column must land in the identical place whether the lead
    /// holds 10 px of content or twice the page width.
    #[test]
    fn the_trailing_column_does_not_move_with_the_lead_columns_content() {
        let (narrow_lead, narrow_trail, _) = row_geometry(10.0);
        let (wide_lead, wide_trail, _) = row_geometry(2000.0);

        assert!(
            (narrow_trail.left() - wide_trail.left()).abs() < 0.5,
            "the trailing column moved {:.1} px when the lead overflowed",
            (narrow_trail.left() - wide_trail.left()).abs(),
        );
        assert!(
            (narrow_lead.width() - wide_lead.width()).abs() < 0.5,
            "the lead column changed width with its content"
        );
    }

    /// The row divides on the ratio `Split` promises, and stays inside the page.
    #[test]
    fn a_lead_row_is_58_percent_and_fits_the_page() {
        let (lead, trail, avail) = row_geometry(2000.0);
        let content = avail - GUTTER;

        assert!(
            (trail.left() - lead.left() - (lead.width() + GUTTER)).abs() < 0.5,
            "trail column misplaced: lead.left={} trail.left={}",
            lead.left(),
            trail.left(),
        );
        assert!(
            trail.right() <= lead.left() + avail + 0.5,
            "trail column ran past the page: right={} limit={}",
            trail.right(),
            lead.left() + avail
        );
        assert!(
            (lead.width() / content - 0.58).abs() < 0.01,
            "lead column is {:.1}% of the row, expected 58%",
            100.0 * lead.width() / content
        );
    }

    /// A column that draws past its slot is clipped, and the row claims only
    /// its own width, so the page's content width cannot see it. That is how
    /// the Camera Source row ran 40 px off the page with the overflow test
    /// green (6a). The column must report it, and `page` must count it.
    #[test]
    fn a_column_that_overflows_its_slot_makes_the_page_overflow() {
        for (extra, overflows) in [(0.0_f32, false), (40.0, true)] {
            let seen: RefCell<Option<super::super::PageMetrics>> = RefCell::new(None);
            frame(egui::vec2(1000.0, 700.0), |ui| {
                let m = super::super::page(ui, |ui| {
                    two_columns(ui, "t", Split::Lead, |cols| {
                        cols.group(None, |_| {});
                        cols.group(None, |ui| {
                            let w = ui.available_width() + extra;
                            ui.allocate_space(egui::vec2(w, 20.0));
                        });
                    });
                });
                *seen.borrow_mut() = Some(m);
            });
            let m = seen.into_inner().expect("page drew");
            assert_eq!(
                m.overflow() > 0.0,
                overflows,
                "trailing column {extra} px past its slot: overflow {:.1}",
                m.overflow()
            );
        }
    }

    /// A column must not paint outside its own slot, even when its content
    /// overflows — otherwise the neighbour is drawn over rather than displaced.
    #[test]
    fn a_columns_clip_rect_is_its_own_slot() {
        let seen: RefCell<Vec<(egui::Rect, egui::Rect)>> = RefCell::new(Vec::new());

        frame(egui::vec2(1000.0, 700.0), |ui| {
            two_columns(ui, "t", Split::Even, |cols| {
                cols.group(None, |ui| {
                    seen.borrow_mut().push((ui.max_rect(), ui.clip_rect()));
                });
                cols.group(None, |ui| {
                    seen.borrow_mut().push((ui.max_rect(), ui.clip_rect()));
                });
            });
        });

        let seen = seen.borrow();
        let (_, lead_clip) = seen[0];
        let (trail_rect, _) = seen[1];
        assert!(
            lead_clip.right() <= trail_rect.left() + 0.5,
            "the lead column can paint into the trailing one: clip right={} trail left={}",
            lead_clip.right(),
            trail_rect.left()
        );
    }

    // ── Path elision ────────────────────────────────────────────────────────

    const LONG: &str =
        "/mnt/steam/SteamLibrary/steamapps/common/Star Citizen/drive_c/Program Files/Bin64";

    /// A path elided to a width must measure no wider than that width in the
    /// real font. Asserted as a width, because a character count is only ever
    /// right for one column.
    #[test]
    fn an_elided_path_never_exceeds_the_width_it_was_given() {
        frame(egui::vec2(1180.0, 760.0), |ui| {
            let font = egui::FontId::monospace(theme::TEXT_SM);
            for room in [0.0, 12.0, 40.0, 120.0, 320.0, 415.0, 872.0, 4000.0] {
                let out = elide_to_width(ui, LONG, &font, room);
                let width = ui.fonts(|f| {
                    f.layout_no_wrap(out.clone(), font.clone(), egui::Color32::WHITE)
                        .rect
                        .width()
                });
                assert!(width <= room + 0.5, "elided to {width:.1} px for {room:.1} px: {out:?}");
            }
        });
    }

    #[test]
    fn elision_keeps_the_tail_and_marks_the_cut() {
        let out = elide_left(LONG, 40);
        assert!(out.starts_with('…'));
        assert!(out.ends_with("Program Files/Bin64"));
        assert_eq!(out.chars().count(), 40);
        assert_eq!(elide_left("/short/path", 40), "/short/path");
    }

    /// Degenerate widths must not panic or produce something longer than
    /// asked, and multi-byte characters must not be split.
    #[test]
    fn tiny_budgets_degrade_gracefully() {
        assert_eq!(elide_left("/some/path", 0), "");
        assert_eq!(elide_left("/some/path", 1), "…");
        assert_eq!(elide_left("/some/path", 2).chars().count(), 2);
        assert_eq!(elide_left("ααααααααααααααααααααααα", 10).chars().count(), 10);
    }
}
