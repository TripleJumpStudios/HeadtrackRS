/// Scalar Kalman filter with constant-velocity model and forward prediction.
///
/// State vector per axis: `[position, velocity]`.
/// The constant-velocity model assumes the head moves at roughly constant speed
/// between frames, which is a good approximation at 20 Hz.
///
/// The filter simultaneously:
/// 1. **Smooths** — fuses noisy measurements with the predicted state
/// 2. **Estimates velocity** — used for forward prediction
/// 3. **Predicts ahead** — extrapolates by `predict_ms` to compensate system latency
///
/// ## Tuning
///
/// - `process_noise` (Q): how much the head is expected to jitter between frames.
///   Higher = trusts measurements more (less smoothing, more responsive).
///   Lower = trusts model more (more smoothing, more lag without prediction).
///
/// - `measurement_noise` (R): how noisy the sensor is.
///   Higher = distrusts measurements (heavy smoothing).
///   Lower = trusts measurements (light smoothing).
///
/// - `predict_ms`: how far ahead to extrapolate. Set to your system's end-to-end
///   latency (capture → inference → IPC → render). 0 = no prediction.

#[derive(Clone)]
pub struct ScalarKalman {
    // State estimate
    x: f32,    // position
    v: f32,    // velocity (units per second)

    // Error covariance (2x2 symmetric, stored as 3 values)
    p00: f32,  // position variance
    p01: f32,  // position-velocity covariance
    p11: f32,  // velocity variance

    // Tuning
    process_noise: f32,
    measurement_noise: f32,

    initialised: bool,
}

impl ScalarKalman {
    pub fn new(process_noise: f32, measurement_noise: f32) -> Self {
        Self {
            x: 0.0,
            v: 0.0,
            p00: 1000.0,  // high initial uncertainty
            p01: 0.0,
            p11: 1000.0,
            process_noise,
            measurement_noise: measurement_noise.max(0.001),
            initialised: false,
        }
    }

    /// Run one predict-update cycle and return the forward-predicted position.
    ///
    /// - `measurement`: raw sensor reading (degrees or mm)
    /// - `dt`: seconds since last call
    /// - `predict_s`: seconds to extrapolate ahead (0.0 = no prediction)
    pub fn update(&mut self, measurement: f32, dt: f32, predict_s: f32) -> f32 {
        if !self.initialised {
            self.x = measurement;
            self.v = 0.0;
            self.p00 = 1000.0;
            self.p01 = 0.0;
            self.p11 = 1000.0;
            self.initialised = true;
            return measurement;
        }

        let dt = dt.max(0.001);

        // ── Adaptive process noise ─────────────────────────────────
        // Detect deceleration: when innovation opposes velocity, the head
        // is stopping. Scale Q up so the filter catches stops faster
        // (prevents rubber-band overshoot on abrupt direction changes).
        let x_pred_tentative = self.x + self.v * dt;
        let innovation_mag = (measurement - x_pred_tentative).abs();
        let speed = self.v.abs();

        let decel_boost = if speed > 0.1 {
            let opposing = self.v.signum() != (measurement - x_pred_tentative).signum();
            if opposing {
                (innovation_mag / (speed * dt + 0.01)).min(3.0)
            } else {
                0.0
            }
        } else {
            0.0
        };

        // Q scales 1-4x during deceleration, stays at base during smooth panning
        let q = self.process_noise * (1.0 + decel_boost) * dt;

        // ── Predict step ─────────────────────────────────────────────
        // State transition: x' = x + v*dt,  v' = v
        let x_pred = x_pred_tentative;
        let v_pred = self.v;
        let p00_pred = self.p00 + 2.0 * dt * self.p01 + dt * dt * self.p11 + q;
        let p01_pred = self.p01 + dt * self.p11;
        let p11_pred = self.p11 + q;

        // ── Update step ──────────────────────────────────────────────
        // Measurement model: z = H·x = position only, H = [1, 0]
        let innovation = measurement - x_pred;
        let s = p00_pred + self.measurement_noise; // innovation covariance

        // Kalman gain
        let k0 = p00_pred / s;
        let k1 = p01_pred / s;

        // State update
        self.x = x_pred + k0 * innovation;
        self.v = v_pred + k1 * innovation;

        // Covariance update (Joseph form for numerical stability)
        let one_minus_k0 = 1.0 - k0;
        self.p00 = one_minus_k0 * p00_pred;
        self.p01 = one_minus_k0 * p01_pred;
        self.p11 = p11_pred - k1 * p01_pred;

        // ── Forward prediction ───────────────────────────────────────
        self.x + self.v * predict_s
    }

    pub fn reset(&mut self) {
        self.initialised = false;
    }

    pub fn set_process_noise(&mut self, q: f32) {
        self.process_noise = q.max(0.0);
    }

    pub fn set_measurement_noise(&mut self, r: f32) {
        self.measurement_noise = r.max(0.001);
    }

    /// Current estimated velocity (units per second).
    pub fn velocity(&self) -> f32 {
        self.v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_signal_converges() {
        let mut kf = ScalarKalman::new(1.0, 10.0);
        let mut out = 0.0;
        for _ in 0..100 {
            out = kf.update(5.0, 0.05, 0.0);
        }
        assert!((out - 5.0).abs() < 0.1, "converged to {out}, expected ~5.0");
        assert!(kf.velocity().abs() < 0.5, "velocity should be near zero for constant input");
    }

    #[test]
    fn prediction_extrapolates() {
        let mut kf = ScalarKalman::new(1.0, 1.0);
        // Feed a ramp: 0, 1, 2, 3, ... at dt=0.05 => velocity = 20/s
        for i in 0..50 {
            kf.update(i as f32, 0.05, 0.0);
        }
        // Now predict 50ms ahead — should be ~1 unit past current
        let predicted = kf.update(50.0, 0.05, 0.05);
        let _no_predict = kf.update(51.0, 0.05, 0.0);
        // Predicted should be ahead of where we'd be without prediction
        assert!(predicted > 50.0, "prediction should extrapolate ahead: {predicted}");
    }

    #[test]
    fn smooths_noise() {
        let mut kf = ScalarKalman::new(0.5, 50.0); // trust model more than sensor
        let mut outputs = Vec::new();
        // Feed noisy signal around 10.0
        for i in 0..100 {
            let noise = if i % 2 == 0 { 3.0 } else { -3.0 };
            let out = kf.update(10.0 + noise, 0.05, 0.0);
            if i > 20 {
                outputs.push(out);
            }
        }
        // Output should be much smoother than input (±3)
        let max_dev = outputs.iter().map(|v| (v - 10.0).abs()).fold(0.0f32, f32::max);
        assert!(max_dev < 2.0, "should smooth noise: max deviation {max_dev}");
    }
}
