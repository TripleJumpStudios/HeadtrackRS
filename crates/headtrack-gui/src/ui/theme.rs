//! Palette, typography and the one-shot egui style install.
//!
//! The palette is lifted verbatim from `assets/headtrackrs.html` — which is the
//! shipping build's colour scheme made explicit rather than a new direction.
//! HangarRS's `hangar-ui/src/theme.rs` was ported from this file; the tokens,
//! the five-size type scale and the control geometry are kept identical to it
//! so the two apps read as one suite.

use eframe::egui;
use egui::{Color32, CornerRadius, FontFamily, FontId, Stroke, TextStyle};

// ── Palette ──────────────────────────────────────────────────────────────────

/// Window background.
pub const BG: Color32 = Color32::from_rgb(0x1b, 0x1b, 0x1b);
/// Sidebar + status-bar background.
pub const SIDEBAR: Color32 = Color32::from_rgb(0x1e, 0x1e, 0x23);
/// Card / panel surface.
pub const PANEL: Color32 = Color32::from_rgb(0x21, 0x22, 0x27);
/// Raised panel — hover fills, selected rows.
pub const PANEL_HI: Color32 = Color32::from_rgb(0x28, 0x2a, 0x30);
/// Panel border.
pub const STROKE: Color32 = Color32::from_rgb(0x33, 0x35, 0x3b);
/// Hairline dividers and card outlines.
pub const STROKE_SOFT: Color32 = Color32::from_rgb(0x2a, 0x2c, 0x31);
/// Recessed wells — bar troughs, chart backgrounds.
pub const WELL: Color32 = Color32::from_rgb(0x16, 0x16, 0x19);

/// Primary accent. Unchanged since v0.1.0 — the brand colour.
pub const ORANGE: Color32 = Color32::from_rgb(0xff, 0x8c, 0x00);
/// Filled-accent background (primary buttons, progress fills).
pub const ORANGE_DIM: Color32 = Color32::from_rgb(0x82, 0x48, 0x00);

pub const TEXT: Color32 = Color32::from_rgb(0xca, 0xca, 0xca);
pub const TEXT_HI: Color32 = Color32::from_rgb(0xe9, 0xe9, 0xea);
/// Secondary text — labels, captions, help lines.
///
/// Lifted from the mockup's `#7c7f86`, which measured **3.96:1** against
/// [`PANEL`] and so missed WCAG AA (4.5:1) for the 10–11 px sizes it is used
/// at. Now 6.9:1. The mockup's greys were chosen on a bright display; they were
/// genuinely hard to read.
pub const DIM: Color32 = Color32::from_rgb(0xa8, 0xab, 0xb2);
/// Incidental text — units, paths, disabled states. Was `#5a5d63` at
/// **2.40:1**, a clear AA failure; now 4.9:1.
pub const DIMMER: Color32 = Color32::from_rgb(0x8b, 0x8f, 0x96);

pub const OK: Color32 = Color32::from_rgb(0x5c, 0xc9, 0x8b);
pub const ERR: Color32 = Color32::from_rgb(0xee, 0x5e, 0x5e);
pub const WARN: Color32 = Color32::from_rgb(0xdc, 0xb4, 0x32);

/// Rotation family base (yaw / pitch / roll).
pub const ROT: Color32 = Color32::from_rgb(0x61, 0x9b, 0xf6);
/// Translation family base (x / y / z).
pub const TRN: Color32 = Color32::from_rgb(0xee, 0x5e, 0x5e);

/// Splash background — darker than [`BG`] so the fade-in has somewhere to come
/// from.
pub const BG_DARK: Color32 = Color32::from_rgb(18, 18, 20);

/// Breathing room between a tab's content and the window edges.
///
/// Applied identically on all four sides of the central panel, so the gap to
/// the sidebar, to the right edge and to the status bar all match. Relying on
/// egui's default panel margin gave a comfortable left gap and almost nothing
/// on the right, which read as content jammed against the window edge.
pub const PAGE_INSET: i8 = 24;

/// Maximum width of a tab's content column.
///
/// Without a cap, groups stretch to whatever the window is and every
/// right-aligned value (the status rows, the profile buttons, the Keybinds tier
/// marks) ends up marooned against the far edge, metres from the label it
/// belongs to.
pub const CONTENT_MAX_WIDTH: f32 = 900.0;

/// Minimum width of a tab's content column, so narrow windows still lay out.
pub const CONTENT_MIN_WIDTH: f32 = 360.0;

/// How wide a key/value row is allowed to get.
///
/// [`CONTENT_MAX_WIDTH`] caps the **page**; this caps a **row**. At page width
/// a key and its value sit 700 px apart, too far to read as a pair.
pub const ROW_MAX_WIDTH: f32 = 660.0;

/// Height of every interactive control that can share a row: buttons, steppers,
/// text fields, selectors. One constant so a `−  84°  +  Apply` row lines up
/// instead of stacking three different egui defaults next to each other.
pub const CONTROL_H: f32 = 34.0;

/// Corner radius for those controls.
pub const CONTROL_RADIUS: u8 = 4;

// ── The type scale ───────────────────────────────────────────────────────────
//
// Five sizes, shared with HangarRS. Anything in a page body that is not one of
// these is drift: `grep -rn '\.size([0-9]\|FontId::new([0-9]' src/tabs` must
// return nothing.
//
// The pairing with colour matters as much as the number. `DIMMER` is for a
// word, never for a paragraph: prose is `TEXT_BODY` in `DIM` or brighter.

/// Dense metadata inside a row: paths, counts, units, device names. The one
/// size that may be monospace.
pub const TEXT_SM: f32 = 13.0;

/// Prose. Every sentence in the application is this size.
pub const TEXT_BODY: f32 = 14.0;

/// A row's own title — the thing the row is *about*.
pub const TEXT_MD: f32 = 15.0;

/// A group heading.
pub const TEXT_LG: f32 = 17.0;

/// A [`super::layout::section`] — the page's spine, and the only thing between
/// a group heading and the page title.
pub const TEXT_XL: f32 = 19.0;

// ── Named text styles ────────────────────────────────────────────────────────

/// Tracked-out uppercase micro-label (the mockup's `.eyebrow` / `.panel-label`).
pub const EYEBROW: &str = "Eyebrow";
/// Large tabular readout for gauge values.
pub const TELEMETRY: &str = "Telemetry";
/// Inter SemiBold — egui's `strong()` only recolours, so real weight needs its
/// own family.
pub const SEMIBOLD: &str = "InterSemiBold";
/// JetBrains Mono Bold, for emphasised numerics.
pub const MONO_BOLD: &str = "MonoBold";
/// Noto Sans SemiBold, for the sidebar wordmark only. It is the face the
/// HangarRS wordmark renders in (system-ui, weight 600), so the two products
/// read as one family.
pub const WORDMARK: &str = "Wordmark";
/// Phosphor icon glyphs.
///
/// Icons get their **own family** rather than riding the Proportional fallback
/// chain. Inter maps 745 Private Use Area codepoints for its stylistic
/// alternates, eight of which collide with the Phosphor icons this GUI uses —
/// with Inter ahead in the chain, `CROSSHAIR_SIMPLE` rendered as an arrow and
/// `GEAR_SIX` / `CAMERA` / `KEYBOARD` as stray marks. Selecting the family
/// explicitly makes icon rendering independent of chain order, and immune to
/// any future text font that also squats on the PUA.
pub const ICONS: &str = "Icons";

/// Font family handle for [`SEMIBOLD`].
pub fn semibold() -> FontFamily {
    FontFamily::Name(SEMIBOLD.into())
}

/// Font family handle for [`MONO_BOLD`].
pub fn mono_bold() -> FontFamily {
    FontFamily::Name(MONO_BOLD.into())
}

/// Font family handle for [`WORDMARK`].
pub fn wordmark() -> FontFamily {
    FontFamily::Name(WORDMARK.into())
}

/// Font family handle for [`ICONS`].
pub fn icons() -> FontFamily {
    FontFamily::Name(ICONS.into())
}

/// [`EYEBROW`] text style handle.
pub fn eyebrow_style() -> TextStyle {
    TextStyle::Name(EYEBROW.into())
}

/// [`TELEMETRY`] text style handle.
pub fn telemetry_style() -> TextStyle {
    TextStyle::Name(TELEMETRY.into())
}

/// egui has no letter-spacing, and the mockup's micro-labels lean on it
/// heavily. Interleaving U+2009 THIN SPACE is the closest available
/// approximation — good enough at 10 px, where the alternative is a label that
/// reads as ordinary small text.
pub fn track_out(s: &str) -> String {
    let mut out = String::with_capacity(s.len() * 2);
    for (i, c) in s.chars().enumerate() {
        if i > 0 {
            out.push('\u{2009}');
        }
        out.push(c);
    }
    out
}

// ── Install ──────────────────────────────────────────────────────────────────

/// Register the bundled fonts and apply the whole visual style.
///
/// Called once from the `eframe::run_native` creation closure. Everything the
/// GUI's look depends on that is *not* per-widget lives here.
pub fn install(ctx: &egui::Context) {
    ctx.set_fonts(font_definitions());
    // egui keeps one Style per theme. The app is dark-only, so pin the
    // preference *and* write the same Style into both slots — otherwise a
    // system light-theme signal repaints the app in egui's stock light
    // palette, which no part of this design system accounts for.
    ctx.set_theme(egui::ThemePreference::Dark);
    ctx.all_styles_mut(install_style);
}

/// Build the font set: bundled text faces, the two weight-specific families,
/// and the isolated icon family.
///
/// Pure so it can be asserted on directly in tests.
fn font_definitions() -> egui::FontDefinitions {
    let mut fonts = egui::FontDefinitions::default();

    for (name, bytes) in [
        ("Inter", &include_bytes!("../../../../assets/fonts/Inter-Regular.ttf")[..]),
        ("InterSemiBold", &include_bytes!("../../../../assets/fonts/Inter-SemiBold.ttf")[..]),
        ("JetBrainsMono", &include_bytes!("../../../../assets/fonts/JetBrainsMono-Regular.ttf")[..]),
        ("JetBrainsMonoBold", &include_bytes!("../../../../assets/fonts/JetBrainsMono-Bold.ttf")[..]),
        ("NotoSansSemiBold", &include_bytes!("../../../../assets/fonts/NotoSans-SemiBold.ttf")[..]),
    ] {
        fonts
            .font_data
            .insert(name.to_owned(), egui::FontData::from_static(bytes).into());
    }

    // Inter and JetBrains Mono become the primaries; egui's bundled Ubuntu /
    // Hack stay behind them as fallbacks so any glyph Inter lacks still
    // renders rather than turning into tofu.
    fonts
        .families
        .entry(FontFamily::Proportional)
        .or_default()
        .insert(0, "Inter".to_owned());
    fonts
        .families
        .entry(FontFamily::Monospace)
        .or_default()
        .insert(0, "JetBrainsMono".to_owned());

    fonts.families.insert(
        FontFamily::Name(SEMIBOLD.into()),
        vec!["InterSemiBold".to_owned(), "Inter".to_owned()],
    );
    fonts.families.insert(
        FontFamily::Name(MONO_BOLD.into()),
        vec!["JetBrainsMonoBold".to_owned(), "JetBrainsMono".to_owned()],
    );
    fonts.families.insert(
        FontFamily::Name(WORDMARK.into()),
        vec!["NotoSansSemiBold".to_owned(), "InterSemiBold".to_owned()],
    );

    // Phosphor gets its own family and is deliberately NOT added to the
    // Proportional chain: see the note on [`ICONS`]. It also cannot go first
    // in that chain, because it maps 28 ASCII codepoints (a–z among them) and
    // would swallow ordinary lowercase text.
    fonts.font_data.insert(
        "phosphor".to_owned(),
        egui_phosphor::Variant::Regular.font_data().into(),
    );
    // Phosphor first so it owns the Private Use Area, then Inter behind it.
    // Inter is not there to render icons — it is the fallback epaint needs for
    // the replacement glyphs ('◻', '?'); a family that cannot supply those logs
    // "Failed to find replacement characters" on every startup. Order is what
    // keeps the PUA collision fixed: within this family Phosphor always wins.
    fonts.families.insert(
        FontFamily::Name(ICONS.into()),
        vec!["phosphor".to_owned(), "Inter".to_owned()],
    );

    fonts
}

/// Apply the whole visual style to one [`egui::Style`].
///
/// Takes the style rather than the context so it can be asserted on directly,
/// and so `all_styles_mut` can run it once per theme.
fn install_style(style: &mut egui::Style) {
    style.text_styles.insert(TextStyle::Heading, FontId::new(24.0, semibold()));
    style.text_styles.insert(TextStyle::Body, FontId::proportional(TEXT_MD));
    style.text_styles.insert(TextStyle::Button, FontId::new(TEXT_MD, semibold()));
    style.text_styles.insert(TextStyle::Small, FontId::proportional(TEXT_SM));
    style.text_styles.insert(TextStyle::Monospace, FontId::monospace(13.0));
    style.text_styles.insert(eyebrow_style(), FontId::monospace(11.0));
    style.text_styles.insert(telemetry_style(), FontId::new(15.0, mono_bold()));

    let v = &mut style.visuals;
    v.dark_mode = true;
    v.panel_fill = BG;
    v.window_fill = PANEL;
    v.extreme_bg_color = WELL;
    v.faint_bg_color = PANEL_HI;
    v.override_text_color = None;
    v.hyperlink_color = ORANGE;
    v.selection.bg_fill = ORANGE.gamma_multiply(0.35);
    v.selection.stroke = Stroke::new(1.0_f32, TEXT_HI);
    v.window_stroke = Stroke::new(1.0_f32, STROKE);
    v.window_corner_radius = CornerRadius::same(10);
    // Sliders paint their filled portion from `selection.bg_fill`; turning it
    // on globally is what gives every slider the accent-filled rail.
    v.slider_trailing_fill = true;

    let w = &mut v.widgets;
    // Applies to every widget box, including the ~16 px checkbox — at radius 8
    // that renders as a circle and reads as a radio button. Buttons that want a
    // softer corner set their own via `.corner_radius()`.
    let radius = CornerRadius::same(4);

    w.noninteractive.bg_fill = PANEL;
    w.noninteractive.weak_bg_fill = PANEL;
    w.noninteractive.bg_stroke = Stroke::new(1.0_f32, STROKE_SOFT);
    w.noninteractive.fg_stroke = Stroke::new(1.0_f32, TEXT);
    w.noninteractive.corner_radius = radius;

    // `bg_fill` is the slider rail and the checkbox box; `weak_bg_fill` is the
    // button surface. They must differ — with both set to PANEL the rail was
    // the same colour as the card behind it and simply vanished, leaving a
    // handle floating in space.
    w.inactive.bg_fill = WELL;
    w.inactive.weak_bg_fill = PANEL;
    w.inactive.bg_stroke = Stroke::new(1.0_f32, STROKE);
    w.inactive.fg_stroke = Stroke::new(1.0_f32, TEXT);
    w.inactive.corner_radius = radius;

    w.hovered.bg_fill = WELL.gamma_multiply(1.6);
    w.hovered.weak_bg_fill = PANEL_HI;
    w.hovered.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(0x42, 0x45, 0x4b));
    w.hovered.fg_stroke = Stroke::new(1.0_f32, TEXT_HI);
    w.hovered.corner_radius = radius;

    w.active.bg_fill = WELL.gamma_multiply(1.9);
    w.active.weak_bg_fill = PANEL_HI;
    w.active.bg_stroke = Stroke::new(1.0_f32, ORANGE);
    w.active.fg_stroke = Stroke::new(1.0_f32, Color32::WHITE);
    w.active.corner_radius = radius;

    w.open.bg_fill = PANEL_HI;
    w.open.weak_bg_fill = PANEL_HI;
    w.open.bg_stroke = Stroke::new(1.0_f32, STROKE);
    w.open.fg_stroke = Stroke::new(1.0_f32, TEXT_HI);
    w.open.corner_radius = radius;

    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(10.0, 6.0);
    style.spacing.slider_rail_height = 5.0;
    style.spacing.slider_width = 160.0;
    style.spacing.interact_size.y = 22.0;
    style.spacing.scroll.bar_width = 9.0;
    // Floating scrollbars overlay the content instead of reserving a lane.
    // A reserved lane is dead space on every tab that fits, which made the
    // right-hand inset visibly wider than the left; floating keeps all four
    // page insets equal, and the bar rides over the right margin — which is
    // empty by construction — on the tabs that do scroll.
    style.spacing.scroll.floating = true;
    style.spacing.scroll.floating_allocated_width = 0.0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn track_out_interleaves_thin_spaces() {
        assert_eq!(track_out("AB"), "A\u{2009}B");
        assert_eq!(track_out("A"), "A");
        assert_eq!(track_out(""), "");
    }

    /// Icons must resolve through their own family, never the proportional
    /// fallback chain.
    ///
    /// Inter maps 745 Private Use Area codepoints for its stylistic alternates,
    /// eight of which collide with the Phosphor icons this GUI uses. With
    /// Phosphor merely appended to `Proportional`, Inter wins those codepoints
    /// and the wrong glyph is drawn — `CROSSHAIR_SIMPLE` rendered as an arrow.
    /// Phosphor also cannot simply go *first*, because it maps 28 ASCII
    /// codepoints (a–z) and would swallow ordinary lowercase text.
    #[test]
    fn icons_live_in_their_own_family() {
        let fonts = font_definitions();

        let icons = &fonts.families[&FontFamily::Name(ICONS.into())];
        assert_eq!(
            icons.first(),
            Some(&"phosphor".to_owned()),
            "Phosphor must lead the Icons family, or Inter's PUA alternates win again"
        );
        assert!(
            icons.len() > 1,
            "the Icons family needs a text fallback behind Phosphor, otherwise epaint \
             cannot find replacement glyphs and warns on every startup"
        );
        assert!(
            !fonts.families[&FontFamily::Proportional].contains(&"phosphor".to_owned()),
            "Phosphor must stay out of the Proportional chain: it maps 28 ASCII \
             codepoints (a–z) and would swallow ordinary lowercase text"
        );
        assert_eq!(
            fonts.families[&FontFamily::Proportional].first(),
            Some(&"Inter".to_owned()),
            "Inter must lead the proportional chain"
        );
    }

    /// The bundled font files must actually be present and parseable as
    /// TrueType — a truncated or LFS-pointer file would otherwise only fail at
    /// runtime, after the window opens.
    #[test]
    fn bundled_fonts_are_truetype() {
        for bytes in [
            &include_bytes!("../../../../assets/fonts/Inter-Regular.ttf")[..],
            &include_bytes!("../../../../assets/fonts/Inter-SemiBold.ttf")[..],
            &include_bytes!("../../../../assets/fonts/JetBrainsMono-Regular.ttf")[..],
            &include_bytes!("../../../../assets/fonts/JetBrainsMono-Bold.ttf")[..],
            &include_bytes!("../../../../assets/fonts/NotoSans-SemiBold.ttf")[..],
        ] {
            assert!(bytes.len() > 10_000, "font file suspiciously small");
            // TrueType outlines start with the 0x00010000 sfnt version tag.
            assert_eq!(&bytes[..4], &[0x00, 0x01, 0x00, 0x00], "not a TrueType font");
        }
    }

    /// The slider rail and the button surface must not be the same colour.
    ///
    /// With both set to PANEL the rail vanishes into the page behind it.
    #[test]
    fn bg_fill_and_weak_bg_fill_differ() {
        let mut style = egui::Style::default();
        install_style(&mut style);
        let w = &style.visuals.widgets;
        for s in [&w.inactive, &w.hovered, &w.active] {
            assert_ne!(
                s.bg_fill, s.weak_bg_fill,
                "bg_fill (rail) and weak_bg_fill (button surface) must differ"
            );
        }
    }

    /// Global widget radius is 4, not 8: at 8 a 16 px checkbox renders as a
    /// circle and reads as a radio button.
    #[test]
    fn global_widget_radius_is_four() {
        let mut style = egui::Style::default();
        install_style(&mut style);
        assert_eq!(style.visuals.widgets.inactive.corner_radius, CornerRadius::same(4));
    }

    /// The five sizes are a strict ladder; two equal rungs would let a heading
    /// and the body it groups read as the same weight.
    #[test]
    fn the_type_scale_is_five_ascending_sizes() {
        let scale = [TEXT_SM, TEXT_BODY, TEXT_MD, TEXT_LG, TEXT_XL];
        assert!(scale.windows(2).all(|w| w[0] < w[1]), "{scale:?}");
        assert_eq!(scale, [13.0, 14.0, 15.0, 17.0, 19.0], "shared with HangarRS's theme.rs");
    }
}
