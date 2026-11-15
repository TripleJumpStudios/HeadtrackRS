//! Painter-driven instruments: axis gauges, bars, the pose bubble, the curve
//! editor, and the camera-feed HUD framing.

use super::theme;
use eframe::egui;
use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, StrokeKind};

/// Centre-tick gauge row: label, bipolar bar with a glowing knob, and a
/// right-aligned tabular value with a dim unit suffix.
///
/// This is the Dashboard's instrument cluster row — the mockup's `.gauge`.
pub fn axis_gauge(
    ui: &mut egui::Ui,
    label: &str,
    value: f32,
    max: f32,
    unit: &str,
    color: Color32,
) {
    const LABEL_W: f32 = 42.0;
    const VALUE_W: f32 = 76.0;
    const GAP: f32 = 12.0;
    const ROW_H: f32 = 24.0;

    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, ROW_H), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let mid_y = rect.center().y;

    painter.text(
        egui::pos2(rect.left(), mid_y),
        Align2::LEFT_CENTER,
        label,
        FontId::new(theme::TEXT_SM, theme::mono_bold()),
        color,
    );

    // Value + unit as one right-aligned run so the unit hugs the number.
    let mut job = egui::text::LayoutJob::default();
    job.append(
        &super::signed(value, 1),
        0.0,
        egui::TextFormat {
            font_id: FontId::new(theme::TEXT_BODY, theme::mono_bold()),
            color: theme::TEXT_HI,
            ..Default::default()
        },
    );
    job.append(
        unit,
        2.0,
        egui::TextFormat {
            font_id: FontId::monospace(theme::TEXT_SM),
            color: theme::DIM,
            ..Default::default()
        },
    );
    let galley = painter.layout_job(job);
    let galley_size = galley.size();
    painter.galley(
        egui::pos2(rect.right() - galley_size.x, mid_y - galley_size.y * 0.5),
        galley,
        Color32::PLACEHOLDER,
    );

    let bar = Rect::from_min_max(
        egui::pos2(rect.left() + LABEL_W + GAP, mid_y - 4.0),
        egui::pos2(rect.right() - VALUE_W - GAP, mid_y + 4.0),
    );
    if bar.width() > 8.0 {
        bipolar_bar(&painter, bar, value, max, color, true);
    }
}

/// Draw the trough, centre tick, fill and knob of a bipolar bar.
fn bipolar_bar(
    painter: &egui::Painter,
    rect: Rect,
    value: f32,
    max: f32,
    color: Color32,
    knob: bool,
) {
    painter.rect_filled(rect, 5.0, theme::WELL);
    painter.rect_stroke(
        rect,
        5.0,
        Stroke::new(1.0_f32, theme::STROKE_SOFT),
        StrokeKind::Inside,
    );

    let center_x = rect.center().x;
    painter.line_segment(
        [
            egui::pos2(center_x, rect.top() - 2.0),
            egui::pos2(center_x, rect.bottom() + 2.0),
        ],
        Stroke::new(1.0_f32, theme::STROKE),
    );

    let norm = if max > 0.0 {
        (value / max).clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let end_x = center_x + norm * rect.width() * 0.5;
    let fill = Rect::from_min_max(
        egui::pos2(center_x.min(end_x), rect.top() + 1.0),
        egui::pos2(center_x.max(end_x), rect.bottom() - 1.0),
    );
    if fill.width() > 0.5 {
        painter.rect_filled(fill, 4.0, color.gamma_multiply(0.85));
    }

    if knob {
        let pos = egui::pos2(end_x, rect.center().y);
        // Cheap glow: a wide translucent disc under the solid knob.
        painter.circle_filled(pos, 7.0, color.gamma_multiply(0.25));
        painter.circle_filled(pos, 4.0, color);
    }
}

/// Compact bar used by the Advanced tab's live-axis rows.
///
/// Signature preserved from the pre-overhaul GUI.
pub fn draw_bar(ui: &mut egui::Ui, value: f32, max_val: f32, active: bool, width: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, 14.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let inner = Rect::from_center_size(rect.center(), egui::vec2(width, 8.0));

    let norm = (value / max_val).clamp(-1.0, 1.0);
    let color = if !active {
        theme::DIMMER
    } else if norm.abs() > 0.85 {
        theme::ERR
    } else if norm.abs() > 0.6 {
        theme::WARN
    } else {
        theme::ROT
    };
    bipolar_bar(&painter, inner, value, max_val, color, false);
}

/// Yaw/pitch position bubble with a roll arc and a Z-driven radius.
///
/// Signature preserved from the pre-overhaul GUI.
pub fn draw_pose_bubble(
    ui: &mut egui::Ui,
    yaw: f32,
    pitch: f32,
    roll: f32,
    z: Option<f32>,
    width: f32,
    height: f32,
) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    paint_pose_bubble(ui, &painter, rect, yaw, pitch, roll, z);
}

/// The pose bubble painted into `rect` without allocating, for drawing over
/// another widget: the Dashboard's Tobii feed box, which has no picture and
/// shows the head moving instead (3c).
pub fn paint_pose_bubble(
    ui: &egui::Ui,
    painter: &egui::Painter,
    rect: Rect,
    yaw: f32,
    pitch: f32,
    roll: f32,
    z: Option<f32>,
) {
    // `None` for a source with no translation (the Tobii): the bubble keeps
    // its size and the caption drops Z rather than showing a held zero.
    let z_mm = z.unwrap_or(0.0);
    let (width, height) = (rect.width(), rect.height());

    painter.rect_filled(rect, 8.0, theme::WELL);
    painter.rect_stroke(
        rect,
        8.0,
        Stroke::new(1.0_f32, theme::STROKE_SOFT),
        StrokeKind::Inside,
    );

    let (cx, cy) = (rect.center().x, rect.center().y);
    let cross = Stroke::new(1.0_f32, theme::STROKE_SOFT);
    painter.line_segment(
        [
            egui::pos2(rect.left() + 8.0, cy),
            egui::pos2(rect.right() - 8.0, cy),
        ],
        cross,
    );
    painter.line_segment(
        [
            egui::pos2(cx, rect.top() + 8.0),
            egui::pos2(cx, rect.bottom() - 8.0),
        ],
        cross,
    );

    let bx = cx + (yaw / 90.0).clamp(-1.0, 1.0) * width * 0.30;
    let by = cy - (pitch / 90.0).clamp(-1.0, 1.0) * height * 0.27;
    let radius = (width.min(height) * 0.13).max(16.0) * (1.0 + (z_mm / 150.0).clamp(-0.35, 0.55));

    painter.circle_stroke(
        egui::pos2(bx, by),
        radius,
        Stroke::new(1.5_f32, theme::ORANGE.gamma_multiply(0.65)),
    );
    painter.circle_filled(egui::pos2(bx, by), 2.5, theme::ORANGE);

    // Roll indicator: a short arc riding the top of the bubble.
    let roll_rad = roll.to_radians();
    let arc_half = 18.0_f32.to_radians();
    const N: usize = 24;
    let arc: Vec<Pos2> = (0..=N)
        .map(|k| {
            let frac = k as f32 / N as f32;
            let a = roll_rad - arc_half + frac * arc_half * 2.0 - std::f32::consts::FRAC_PI_2;
            egui::pos2(bx + radius * a.cos(), by + radius * a.sin())
        })
        .collect();
    painter.add(egui::Shape::line(
        arc,
        Stroke::new(3.5_f32, super::axis_color(2)),
    ));

    // Axis names spelled out: a bare `Y` here meant yaw, beside `Z` the
    // translation axis, and `Y` is the translation axis everywhere else (4b).
    // Z goes on a line of its own when the one-line caption would not fit.
    let font = FontId::monospace(theme::TEXT_SM);
    let rot = format!(
        "Yaw {}°  Pitch {}°  Roll {}°",
        super::signed(yaw, 0),
        super::signed(pitch, 0),
        super::signed(roll, 0)
    );
    let caption = match z {
        None => rot,
        Some(z) => {
            let one_line = format!("{rot}  Z {}mm", super::signed(z, 0));
            let fits = ui.fonts(|f| {
                f.layout_no_wrap(one_line.clone(), font.clone(), theme::DIM)
                    .rect
                    .width()
            }) <= width - 16.0;
            if fits {
                one_line
            } else {
                format!("{rot}\nZ {}mm", super::signed(z, 0))
            }
        }
    };
    painter.text(
        egui::pos2(cx, rect.bottom() - 6.0),
        Align2::CENTER_BOTTOM,
        caption,
        font,
        theme::DIM,
    );
}

/// Response-curve plot with a live position dot.
///
/// Signature preserved from the pre-overhaul GUI.
pub fn draw_curve_editor(
    ui: &mut egui::Ui,
    curves: &[(f32, f32, f32, Color32)],
    is_rotation: bool,
    width: f32,
    height: f32,
) {
    let max_x: f32 = if is_rotation { 45.0 } else { 80.0 };
    let unit = if is_rotation { "°" } else { "mm" };

    // Left pad fits a five-character tick label ("400mm") at TEXT_SM monospace.
    let (lpad, rpad, bpad, tpad) = (46.0, 14.0, 20.0, 8.0);
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(width, height + bpad + tpad),
        egui::Sense::hover(),
    );
    let painter = ui.painter_at(rect);

    let chart = Rect::from_min_max(
        egui::pos2(rect.left() + lpad, rect.top() + tpad),
        egui::pos2(rect.right() - rpad, rect.top() + tpad + height),
    );
    painter.rect_filled(chart, 6.0, theme::WELL);
    painter.rect_stroke(
        chart,
        6.0,
        Stroke::new(1.0_f32, theme::STROKE_SOFT),
        StrokeKind::Inside,
    );

    let raw_max_y = curves
        .iter()
        .map(|(s, e, _, _)| s * max_x.powf(*e))
        .fold(max_x * 0.2, f32::max);
    let max_y = nice_ceil(raw_max_y);

    let to_sx = |x: f32| chart.left() + (x / max_x).clamp(0.0, 1.0) * chart.width();
    let to_sy = |y: f32| chart.bottom() - (y / max_y).clamp(0.0, 1.2) * chart.height();

    let grid = Stroke::new(1.0_f32, theme::STROKE_SOFT);
    for i in 1..=4_u32 {
        let x = max_x * i as f32 / 4.0;
        let sx = to_sx(x);
        painter.line_segment(
            [egui::pos2(sx, chart.top()), egui::pos2(sx, chart.bottom())],
            grid,
        );
        painter.text(
            egui::pos2(sx, chart.bottom() + 3.0),
            Align2::CENTER_TOP,
            format!("{x:.0}{unit}"),
            FontId::monospace(theme::TEXT_SM),
            theme::DIMMER,
        );
    }
    for i in 1..=3_u32 {
        let y = max_y * i as f32 / 3.0;
        let sy = to_sy(y);
        painter.line_segment(
            [egui::pos2(chart.left(), sy), egui::pos2(chart.right(), sy)],
            grid,
        );
        painter.text(
            egui::pos2(chart.left() - 5.0, sy),
            Align2::RIGHT_CENTER,
            format!("{y:.0}{unit}"),
            FontId::monospace(theme::TEXT_SM),
            theme::DIMMER,
        );
    }

    // Dashed 1:1 reference line — where the curve would sit at unity gain.
    let unity: Vec<Pos2> = (0..=60)
        .map(|i| {
            let x = max_x * i as f32 / 60.0;
            egui::pos2(to_sx(x), to_sy(x))
        })
        .collect();
    for chunk in unity.chunks(3) {
        if let [a, _, c] = chunk {
            painter.line_segment([*a, *c], Stroke::new(1.0_f32, theme::DIMMER));
        }
    }

    const N: usize = 120;
    for &(sens, exp, live_input, color) in curves {
        let pts: Vec<Pos2> = (0..=N)
            .map(|j| {
                let x = max_x * j as f32 / N as f32;
                egui::pos2(to_sx(x), to_sy(sens * x.powf(exp)))
            })
            .collect();
        painter.add(egui::Shape::line(pts, Stroke::new(2.0_f32, color)));

        let lx = live_input.abs().clamp(0.0, max_x);
        let pos = egui::pos2(
            to_sx(lx),
            to_sy(sens * lx.powf(exp)).clamp(chart.top(), chart.bottom()),
        );
        painter.circle_filled(pos, 5.0, color);
        painter.circle_stroke(pos, 5.0, Stroke::new(1.5_f32, Color32::WHITE));
    }
}

fn nice_ceil(v: f32) -> f32 {
    if !v.is_finite() || v <= 0.0 {
        return 10.0;
    }
    let mag = 10_f32.powf(v.log10().floor());
    ((v / mag).ceil() * mag).max(1.0)
}

/// Gunsight framing painted over the camera preview: four corner brackets, a
/// centre reticle when there is no lock, and a caption strip.
pub fn hud_overlay(painter: &egui::Painter, rect: Rect, locked: bool, caption: Option<&str>) {
    let arm = (rect.width() * 0.06).clamp(14.0, 28.0);
    let inset = 10.0;
    let stroke = Stroke::new(2.0_f32, theme::ORANGE.gamma_multiply(0.85));

    for (corner, dx, dy) in [
        (rect.left_top(), 1.0, 1.0),
        (rect.right_top(), -1.0, 1.0),
        (rect.left_bottom(), 1.0, -1.0),
        (rect.right_bottom(), -1.0, -1.0),
    ] {
        let origin = egui::pos2(corner.x + dx * inset, corner.y + dy * inset);
        painter.line_segment([origin, egui::pos2(origin.x + dx * arm, origin.y)], stroke);
        painter.line_segment([origin, egui::pos2(origin.x, origin.y + dy * arm)], stroke);
    }

    if !locked {
        let c = rect.center();
        let r = (rect.height() * 0.16).clamp(20.0, 40.0);
        let thin = Stroke::new(1.5_f32, theme::ORANGE.gamma_multiply(0.5));
        painter.circle_stroke(c, r * 0.55, thin);
        for (dx, dy) in [(0.0, -1.0), (0.0, 1.0), (-1.0, 0.0), (1.0, 0.0)] {
            painter.line_segment(
                [
                    egui::pos2(c.x + dx * r * 0.75, c.y + dy * r * 0.75),
                    egui::pos2(c.x + dx * r * 1.5, c.y + dy * r * 1.5),
                ],
                thin,
            );
        }
    }

    if let Some(caption) = caption {
        painter.text(
            egui::pos2(rect.center().x, rect.bottom() - 10.0),
            Align2::CENTER_BOTTOM,
            caption,
            FontId::monospace(theme::TEXT_SM),
            if locked { theme::OK } else { theme::DIM },
        );
    }
}

/// What a head source that is not tracking yet needs from the user. Drawn over
/// its feed box until tracking starts, so nobody has to find it in a status
/// line (D1/D2).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SourcePrompt {
    /// The device is opening; nothing to do but sit facing the screen.
    Starting,
    /// The neutral is being taken: `progress` (0..=1) of the still window is
    /// held, `seconds_left` of it remain if the user stays still.
    HoldStill { progress: f32, seconds_left: f32 },
    /// The neutral is wanted, but there is no face in view.
    FaceTracker,
}

impl SourcePrompt {
    /// The headline and the one-line instruction under it.
    pub fn words(self) -> (&'static str, &'static str) {
        match self {
            SourcePrompt::Starting => ("Starting the tracker", "Sit facing the screen"),
            SourcePrompt::HoldStill { .. } => {
                ("Hold still", "Face the screen until the ring fills")
            }
            SourcePrompt::FaceTracker => ("Face the tracker", "No face in view yet"),
        }
    }
}

/// Paints `prompt` centred in `rect`: a ring (a countdown that fills while the
/// user holds still, a spinner while starting) over a headline and one line of
/// instruction. Asks for repaints while shown, so the ring moves.
pub fn source_prompt(ui: &egui::Ui, painter: &egui::Painter, rect: Rect, prompt: SourcePrompt) {
    let r = (rect.height() * 0.16).clamp(18.0, 34.0);
    let head = theme::TEXT_XL;
    let sub = theme::TEXT_SM;
    let total = 2.0 * r + 10.0 + head + 6.0 + sub;
    let top = rect.center().y - total / 2.0;
    let c = egui::pos2(rect.center().x, top + r);

    // Soft well behind the words, so they read over a pose bubble or nothing.
    painter.rect_filled(
        Rect::from_center_size(
            egui::pos2(rect.center().x, rect.center().y),
            egui::vec2((rect.width() - 16.0).min(320.0), total + 24.0),
        ),
        8.0,
        theme::BG_DARK.gamma_multiply(0.85),
    );

    let track = Stroke::new(4.0_f32, theme::ORANGE_DIM.gamma_multiply(0.6));
    painter.circle_stroke(c, r, track);
    let arc = |from: f32, sweep: f32| {
        let n = ((sweep.abs() / std::f32::consts::TAU) * 64.0)
            .ceil()
            .max(2.0) as usize;
        (0..=n)
            .map(|i| {
                let a = from + sweep * i as f32 / n as f32;
                egui::pos2(c.x + r * a.cos(), c.y + r * a.sin())
            })
            .collect::<Vec<Pos2>>()
    };
    let lit = Stroke::new(4.0_f32, theme::ORANGE);
    let up = -std::f32::consts::FRAC_PI_2;
    match prompt {
        SourcePrompt::Starting => {
            let t = ui.input(|i| i.time) as f32;
            painter.add(egui::Shape::line(
                arc(up + t * 4.0, std::f32::consts::FRAC_PI_2),
                lit,
            ));
        }
        SourcePrompt::HoldStill {
            progress,
            seconds_left,
        } => {
            let p = progress.clamp(0.0, 1.0);
            if p > 0.0 {
                painter.add(egui::Shape::line(arc(up, p * std::f32::consts::TAU), lit));
            }
            painter.text(
                c,
                Align2::CENTER_CENTER,
                format!("{:.1}", seconds_left.max(0.0)),
                FontId::new(theme::TEXT_LG, theme::mono_bold()),
                theme::TEXT_HI,
            );
        }
        SourcePrompt::FaceTracker => {
            painter.text(
                c,
                Align2::CENTER_CENTER,
                egui_phosphor::regular::USER_FOCUS,
                FontId::new(theme::TEXT_XL, theme::icons()),
                theme::DIM,
            );
        }
    }

    let (headline, line) = prompt.words();
    let y = top + 2.0 * r + 10.0;
    painter.text(
        egui::pos2(rect.center().x, y),
        Align2::CENTER_TOP,
        headline,
        FontId::new(head, theme::semibold()),
        theme::TEXT_HI,
    );
    painter.text(
        egui::pos2(rect.center().x, y + head + 6.0),
        Align2::CENTER_TOP,
        line,
        FontId::proportional(sub),
        theme::DIM,
    );
    ui.ctx()
        .request_repaint_after(std::time::Duration::from_millis(33));
}

#[cfg(test)]
mod tests {
    use super::nice_ceil;

    #[test]
    fn nice_ceil_rounds_up_to_a_readable_axis_max() {
        assert_eq!(nice_ceil(0.0), 10.0);
        assert_eq!(nice_ceil(f32::NAN), 10.0);
        assert_eq!(nice_ceil(42.0), 50.0);
        assert_eq!(nice_ceil(50.0), 50.0);
        assert_eq!(nice_ceil(3.2), 4.0);
    }
}
