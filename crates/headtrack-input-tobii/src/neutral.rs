//! Neutral orientation is a rotation transform, never component-wise Euler subtraction.
use headtrack_tobii_device::Stamp;
use nalgebra::Rotation3;
/// A neutral needs this long a still run (device clock)…
const NEUTRAL_SPAN_US: u64 = 2_000_000;
/// …and at least this many samples in it.
const NEUTRAL_MIN_SAMPLES: usize = 20;
#[derive(Default)]
pub struct Collector {
    samples: Vec<(Stamp, [f32; 3])>,
}
impl Collector {
    pub fn observe(&mut self, stamp: Stamp, angles: [f32; 3], now_us: u64) -> Option<[f32; 3]> {
        if angles.iter().any(|v| !v.is_finite())
            || now_us < stamp.received_us
            || now_us - stamp.received_us > 250_000
        {
            self.samples.clear();
            return None;
        }
        if let Some((last, _)) = self.samples.last() {
            if last.session == stamp.session && last.sequence == stamp.sequence {
                return None;
            }
            if last.session != stamp.session
                || stamp.device_us <= last.device_us
                || stamp.device_us - last.device_us > 250_000
            {
                self.samples.clear();
            }
        }
        if let Some((_, first)) = self.samples.first() {
            if (0..3).any(|i| (first[i] - angles[i]).abs() > 2.) {
                self.samples.clear();
            }
        }
        self.samples.push((stamp, angles));
        if self.samples.len() >= NEUTRAL_MIN_SAMPLES
            && stamp.device_us - self.samples[0].0.device_us >= NEUTRAL_SPAN_US
        {
            let neutral = std::array::from_fn(|i| {
                let mut v: Vec<_> = self.samples.iter().map(|(_, a)| a[i]).collect();
                v.sort_by(f32::total_cmp);
                v[v.len() / 2]
            });
            self.samples.clear();
            return Some(neutral);
        }
        if self.samples.len() > 128 {
            self.samples.remove(0);
        }
        None
    }
    pub fn clear(&mut self) {
        self.samples.clear();
    }
    /// How far the current still run is toward a neutral, `0.0..=1.0`: the
    /// lesser of its span over the 2 s window and its count over the 20
    /// samples [`Self::observe`] requires. For the GUI's countdown only; it
    /// decides nothing.
    pub fn progress(&self) -> f32 {
        let (Some((first, _)), Some((last, _))) = (self.samples.first(), self.samples.last()) else {
            return 0.0;
        };
        let span = last.device_us.saturating_sub(first.device_us) as f32 / NEUTRAL_SPAN_US as f32;
        let count = self.samples.len() as f32 / NEUTRAL_MIN_SAMPLES as f32;
        span.min(count).clamp(0.0, 1.0)
    }
}
fn rotation(a: [f32; 3]) -> Rotation3<f64> {
    Rotation3::from_euler_angles(
        (a[0] as f64).to_radians(),
        (a[1] as f64).to_radians(),
        (a[2] as f64).to_radians(),
    )
}
pub fn relative_image_angles(neutral: [f32; 3], current: [f32; 3]) -> Option<[f32; 3]> {
    if neutral.iter().chain(&current).any(|v| !v.is_finite()) {
        return None;
    }
    let (x, y, z) = (rotation(neutral).inverse() * rotation(current)).euler_angles();
    Some([
        x.to_degrees() as f32,
        y.to_degrees() as f32,
        z.to_degrees() as f32,
    ])
}
/// Direction signs observed in ir-motion-20260907 after orientation recentering:
/// right +imageY, up -imageX, right-ear tilt -imageZ. Gain remains unvalidated.
pub fn head_rotation(neutral: [f32; 3], current: [f32; 3]) -> Option<[f32; 3]> {
    relative_image_angles(neutral, current).map(|a| [a[1], -a[0], -a[2]])
}
#[cfg(test)]
mod tests {
    use super::*;
    fn stamp(n: u64, t: u64) -> Stamp {
        Stamp {
            session: 1,
            sequence: n,
            device_us: t,
            received_us: t,
        }
    }
    #[test]
    fn neutral_rejects_repeats_motion_staleness_and_short_dwell() {
        let mut c = Collector::default();
        for _ in 0..100 {
            assert!(c.observe(stamp(1, 1), [0.; 3], 1).is_none());
        }
        for n in 2..40 {
            assert!(c
                .observe(
                    stamp(n, n * 10_000),
                    [if n % 2 == 0 { 10. } else { 0. }, 0., 0.],
                    n * 10_000
                )
                .is_none());
        }
        assert!(c.observe(stamp(50, 500_000), [0.; 3], 900_000).is_none());
        for n in 1..21 {
            assert!(c
                .observe(stamp(n, n * 100_000), [1., 2., 3.], n * 100_000)
                .is_none());
        }
        assert_eq!(
            c.observe(stamp(21, 2_100_000), [1., 2., 3.], 2_100_000),
            Some([1., 2., 3.])
        );
    }
    #[test]
    fn progress_rises_while_still_and_resets_on_motion_or_neutral() {
        let mut c = Collector::default();
        assert_eq!(c.progress(), 0.0);
        // 11 samples 100 ms apart = 1 s span, 11/20 count → the span is the lesser.
        for n in 0..11 {
            c.observe(stamp(n + 1, n * 100_000), [1., 2., 3.], n * 100_000);
        }
        assert!((c.progress() - 0.5).abs() < 1e-6, "{}", c.progress());
        // A move beyond 2° restarts the run.
        c.observe(stamp(12, 1_100_000), [9., 2., 3.], 1_100_000);
        assert_eq!(c.progress(), 0.0);
        // Sparse samples (5 Hz): 2 s needs 20 of them, so the count is the lesser.
        let mut d = Collector::default();
        for n in 0..5 {
            d.observe(stamp(n + 1, n * 200_000), [0.; 3], n * 200_000);
        }
        assert!((d.progress() - 0.25).abs() < 1e-6, "{}", d.progress());
        // Taking the neutral empties the collector.
        let mut e = Collector::default();
        for n in 0..=21 {
            e.observe(stamp(n + 1, n * 100_000), [0.; 3], n * 100_000);
        }
        assert_eq!(e.progress(), 0.0);
    }
    #[test]
    fn neutral_removes_camera_mount_rotation_before_euler_decomposition() {
        let n = [-25., 3., 1.];
        let motion = Rotation3::from_euler_angles(0., 30f64.to_radians(), 0.);
        let (x, y, z) = (rotation(n) * motion).euler_angles();
        let cur = [
            x.to_degrees() as f32,
            y.to_degrees() as f32,
            z.to_degrees() as f32,
        ];
        let a = head_rotation(n, cur).unwrap();
        assert!((a[0] - 30.).abs() < 1e-4);
        assert!(a[1].abs() < 1e-4 && a[2].abs() < 1e-4);
    }
}
