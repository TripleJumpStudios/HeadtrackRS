/// One Euro Filter — adaptive low-pass filter for a single f32 signal.
///
/// Adjusts its cutoff frequency based on signal speed:
/// - **Low speed** (head still) → low cutoff → heavy smoothing, no jitter.
/// - **High speed** (fast move) → high cutoff → minimal lag, responsive feel.
///
/// This is what makes the camera feel natural rather than laggy or jittery —
/// you get stillness *and* responsiveness without a tradeoff between them.
///
/// ## Parameters
/// - `min_cutoff`: minimum cutoff frequency in Hz. Lower → smoother when still,
///   but more lag at low speeds. Start around `1.0`.
/// - `beta`: speed coefficient. Higher → filter opens faster for quick moves.
///   Start around `0.007` for head tracking.
///
/// ## Reference
/// Casiez et al., "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy
/// Input in Interactive Systems", CHI 2012.
pub struct OneEuroFilter {
    min_cutoff: f32,
    beta: f32,
    /// Derivative signal cutoff (Hz). Fixed at 1.0; rarely needs tuning.
    d_cutoff: f32,
    x_prev: Option<f32>,
    dx_prev: f32,
}

impl OneEuroFilter {
    pub fn new(min_cutoff: f32, beta: f32) -> Self {
        Self {
            min_cutoff,
            beta,
            d_cutoff: 1.0,
            x_prev: None,
            dx_prev: 0.0,
        }
    }

    /// Filter a new sample `x` taken `dt` seconds after the previous one.
    ///
    /// Returns the filtered value. On the first call, returns `x` unchanged
    /// and initialises internal state.
    pub fn filter(&mut self, x: f32, dt: f32) -> f32 {
        let Some(x_prev) = self.x_prev else {
            self.x_prev = Some(x);
            return x;
        };

        // Estimate instantaneous derivative and smooth it.
        let dx_raw = (x - x_prev) / dt;
        let alpha_d = low_pass_alpha(self.d_cutoff, dt);
        let dx_hat = alpha_d * dx_raw + (1.0 - alpha_d) * self.dx_prev;

        // Raise the cutoff proportionally to signal speed.
        let cutoff = self.min_cutoff + self.beta * dx_hat.abs();
        let alpha = low_pass_alpha(cutoff, dt);

        let x_hat = alpha * x + (1.0 - alpha) * x_prev;

        self.x_prev = Some(x_hat);
        self.dx_prev = dx_hat;

        x_hat
    }

    pub fn reset(&mut self) {
        self.x_prev = None;
        self.dx_prev = 0.0;
    }
}

/// α for a first-order low-pass filter with the given cutoff (Hz) and
/// sample period `dt` (seconds).
fn low_pass_alpha(cutoff: f32, dt: f32) -> f32 {
    // α = dt / (dt + τ)  where τ = 1 / (2π · cutoff)
    let tau = 1.0 / (std::f32::consts::TAU * cutoff.max(0.001));
    dt / (dt + tau)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_is_returned_unchanged() {
        let mut f = OneEuroFilter::new(1.0, 0.007);
        assert_eq!(f.filter(42.0, 0.016), 42.0);
    }

    #[test]
    fn steady_signal_converges() {
        let mut f = OneEuroFilter::new(1.0, 0.007);
        let mut out = 0.0_f32;
        for _ in 0..200 {
            out = f.filter(10.0, 0.016);
        }
        // After enough samples at a constant value, output should be very close.
        assert!((out - 10.0).abs() < 0.01, "output {out} did not converge to 10.0");
    }

    #[test]
    fn filter_reduces_jitter() {
        let mut f = OneEuroFilter::new(1.0, 0.007);
        // Alternate +/- 1.0 around zero — pure high-frequency noise.
        let mut prev = 0.0_f32;
        let mut total_swing = 0.0_f32;
        for i in 0..100 {
            let noisy = if i % 2 == 0 { 1.0 } else { -1.0 };
            let out = f.filter(noisy, 0.016);
            total_swing += (out - prev).abs();
            prev = out;
        }
        // Filtered swing should be much less than the raw 2.0 per step.
        assert!(total_swing / 100.0 < 0.5);
    }
}
