/// Headtrack RS icon generator.
///
/// Reproduces the splash screen reticle as a static 256×256 PNG:
///   - Transparent background (blends with any surface)
///   - Outer ring: white, low alpha (faint HUD look)
///   - Inner dashed ring: white, slightly higher alpha
///   - Four orange cardinal ticks INSIDE the outer ring (matching splash code)
///   - Orange filled center dot
///
/// Run from workspace root:
///   cargo run --manifest-path tools/gen-icon/Cargo.toml -- assets/icons/headtrack-rs.png

use tiny_skia::*;

const SIZE: u32 = 256;
const CX: f32 = 128.0;
const CY: f32 = 128.0;

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "headtrack-rs.png".to_string());

    let mut pixmap = Pixmap::new(SIZE, SIZE).expect("failed to create pixmap");
    // Transparent background — blends cleanly with the sidebar or any surface.
    pixmap.fill(Color::TRANSPARENT);

    let outer_r: f32 = 96.0;
    let inner_r: f32 = 77.0; // same ratio as splash (96/120 * 96)

    // ── Outer ring — faint white (matches splash: from_white_alpha(20), bumped
    //    slightly for a static icon where there's no motion to aid perception) ──
    {
        let path = PathBuilder::from_circle(CX, CY, outer_r).unwrap();
        let mut paint = Paint::default();
        paint.set_color_rgba8(255, 255, 255, 55);
        paint.anti_alias = true;
        let stroke = Stroke { width: 1.5, ..Default::default() };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }

    // ── Inner dashed ring — white, slightly brighter (matches splash: white_alpha(40)) ──
    {
        let path = PathBuilder::from_circle(CX, CY, inner_r).unwrap();
        let mut paint = Paint::default();
        paint.set_color_rgba8(255, 255, 255, 85);
        paint.anti_alias = true;
        // 36-dash pattern matching splash (every other segment drawn)
        let dash = StrokeDash::new(vec![8.5, 5.5], 0.0).unwrap();
        let stroke = Stroke { width: 1.5, dash: Some(dash), ..Default::default() };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }

    // ── Orange cardinal ticks — INSIDE the outer ring, pointing inward ──────
    // Splash: tick_outer = outer_r - 2, tick_inner = tick_outer - 16
    {
        let tick_outer = outer_r - 2.0;
        let tick_inner = tick_outer - 18.0;

        let mut paint = Paint::default();
        paint.set_color_rgba8(255, 140, 0, 255); // #FF8C00
        paint.anti_alias = true;
        let stroke = Stroke {
            width: 2.5,
            line_cap: LineCap::Butt,
            ..Default::default()
        };

        // 0°=right, 90°=down, 180°=left, 270°=up  (screen coords)
        for deg in [0.0_f32, 90.0, 180.0, 270.0] {
            let rad = deg.to_radians();
            let dx = rad.cos();
            let dy = rad.sin();
            let mut pb = PathBuilder::new();
            pb.move_to(CX + dx * tick_inner, CY + dy * tick_inner);
            pb.line_to(CX + dx * tick_outer, CY + dy * tick_outer);
            let path = pb.finish().unwrap();
            pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
        }
    }

    // ── Orange centre dot ────────────────────────────────────────────────────
    {
        let path = PathBuilder::from_circle(CX, CY, 7.0).unwrap();
        let mut paint = Paint::default();
        paint.set_color_rgba8(255, 140, 0, 255);
        paint.anti_alias = true;
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

    pixmap.save_png(&out).expect("failed to save PNG");
    println!("wrote {out}");
}
