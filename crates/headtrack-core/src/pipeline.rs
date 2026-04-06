use crate::pose::Pose;

/// A single composable stage in the pose processing pipeline.
///
/// Stages receive a [`Pose`] and return a (possibly modified) [`Pose`].
/// `dt` is the elapsed time in seconds since the last [`process`](PipelineStage::process)
/// call; use it for any time-dependent math (filters, prediction).
///
/// # Adding a new stage
/// Implement this trait, then push a `Box<dyn PipelineStage>` into [`Pipeline::new`].
pub trait PipelineStage: Send + 'static {
    fn name(&self) -> &str;
    fn process(&mut self, pose: Pose, dt: f32) -> Pose;

    /// Called when the input source is interrupted or recalibrated.
    /// Reset any accumulated filter state so the pipeline starts clean.
    fn reset(&mut self) {}

    /// Update a named parameter at runtime (e.g. from a GUI slider).
    /// Returns `true` if the parameter was recognised and applied.
    fn set_param(&mut self, _param: &str, _value: f32) -> bool {
        false
    }

    /// Returns a list of all current parameters for this stage.
    fn get_params(&self) -> Vec<(String, f32)> {
        Vec::new()
    }
}

/// An ordered chain of [`PipelineStage`]s.
///
/// Each stage's output is the next stage's input.  The order matches the
/// design document: deadzone → filter → curves → prediction → shaping.
pub struct Pipeline {
    stages: Vec<Box<dyn PipelineStage>>,
}

impl Pipeline {
    pub fn new(stages: Vec<Box<dyn PipelineStage>>) -> Self {
        Self { stages }
    }

    /// Run the full chain and return the final processed [`Pose`].
    pub fn process(&mut self, pose: Pose, dt: f32) -> Pose {
        self.stages
            .iter_mut()
            .fold(pose, |p, stage| stage.process(p, dt))
    }

    /// Reset all stages (e.g. after a tracking loss or recenter).
    pub fn reset(&mut self) {
        for stage in &mut self.stages {
            stage.reset();
        }
    }

    /// Forward a parameter update to the named stage.
    /// Returns `true` if the stage was found and accepted the parameter.
    pub fn set_param(&mut self, stage: &str, param: &str, value: f32) -> bool {
        for s in &mut self.stages {
            if s.name() == stage {
                return s.set_param(param, value);
            }
        }
        false
    }

    /// Returns a snapshot of all parameters from all stages.
    /// Format: `(stage_name, param_name, current_value)`.
    pub fn get_all_params(&self) -> Vec<(String, String, f32)> {
        let mut out = Vec::new();
        for s in &self.stages {
            let name = s.name().to_string();
            for (p, v) in s.get_params() {
                out.push((name.clone(), p, v));
            }
        }
        out
    }

    pub fn stage_names(&self) -> impl Iterator<Item = &str> {
        self.stages.iter().map(|s| s.name())
    }
}

/// Built-in pipeline stages, ready to box and push into a [`Pipeline`].
pub mod stages {
    use super::PipelineStage;
    use crate::{filter::kalman::ScalarKalman, filter::PoseFilter, pose::Pose};

    // ── Z-Axis Median Pre-Filter ──────────────────────────────────────

    /// Z-axis 3-sample running median — kills single-frame depth spikes.
    ///
    /// Only filters Z; all other axes pass through unchanged.
    /// A median of 3 is the simplest non-linear filter that rejects
    /// single-frame outliers with zero latency on real movement.
    /// Per-axis 3-sample median filter for single-frame outlier rejection.
    ///
    /// Enabled axes are independently median-filtered; disabled axes pass through.
    /// Default: pitch and Z enabled (the two axes most prone to NeuralNet outliers).
    ///
    /// Parameters via `set_param`:
    /// - `"yaw"`, `"pitch"`, `"roll"`, `"x"`, `"y"`, `"z"` — 1.0 to enable, 0.0 to disable
    pub struct MedianFilterStage {
        bufs:    [[f32; 3]; 6],
        idx:     [usize; 6],
        count:   [u8; 6],
        enabled: [bool; 6],
    }

    impl Default for MedianFilterStage {
        fn default() -> Self { Self::new() }
    }

    impl MedianFilterStage {
        /// pitch (index 1) and Z (index 5) enabled by default.
        pub fn new() -> Self {
            Self {
                bufs:    [[0.0; 3]; 6],
                idx:     [0; 6],
                count:   [0; 6],
                //          yaw    pitch  roll   x      y      z
                enabled: [false, true,  false, false, false, true],
            }
        }

        /// Branchless median of 3 values via min/max.
        #[inline(always)]
        fn median3(a: f32, b: f32, c: f32) -> f32 {
            let lo = a.min(b);
            let hi = a.max(b);
            hi.min(c).max(lo)
        }
    }

    impl PipelineStage for MedianFilterStage {
        fn name(&self) -> &str { "median-filter" }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
            let mut out = vals;
            for i in 0..6 {
                if !self.enabled[i] { continue; }
                self.bufs[i][self.idx[i]] = vals[i];
                self.idx[i] = (self.idx[i] + 1) % 3;
                if self.count[i] < 3 { self.count[i] += 1; }
                if self.count[i] >= 3 {
                    out[i] = Self::median3(self.bufs[i][0], self.bufs[i][1], self.bufs[i][2]);
                }
            }
            Pose {
                yaw: out[0], pitch: out[1], roll: out[2],
                x: out[3], y: out[4], z: out[5],
                timestamp_us: pose.timestamp_us,
            }
        }

        fn reset(&mut self) {
            self.idx   = [0; 6];
            self.count = [0; 6];
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::filter::AXIS_NAMES;
            if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == param) {
                self.enabled[idx] = value > 0.5;
                return true;
            }
            false
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n.to_string(), if self.enabled[i] { 1.0 } else { 0.0 })
            }).collect()
        }
    }

    // Keep the old name as an alias so any external code that references it still compiles.
    #[allow(dead_code)]
    pub type ZMedianStage = MedianFilterStage;

    // ── Axis Mask (per-axis pause) ────────────────────────────────────────

    /// Zeroes out paused axes so they hold at center (neutral).
    ///
    /// Use `set_param("yaw", 0.0)` to pause yaw, `set_param("yaw", 1.0)` to
    /// resume.  Any value ≤ 0.5 is treated as paused.
    pub struct AxisMaskStage {
        /// `true` = axis is active (passes through), `false` = paused (zeroed).
        pub active: [bool; 6],
    }

    impl Default for AxisMaskStage {
        fn default() -> Self { Self::new() }
    }

    impl AxisMaskStage {
        /// All axes active by default.
        pub fn new() -> Self {
            Self { active: [true; 6] }
        }
    }

    impl PipelineStage for AxisMaskStage {
        fn name(&self) -> &str {
            "axis-mask"
        }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            Pose {
                yaw:   if self.active[0] { pose.yaw }   else { 0.0 },
                pitch: if self.active[1] { pose.pitch } else { 0.0 },
                roll:  if self.active[2] { pose.roll }  else { 0.0 },
                x:     if self.active[3] { pose.x }     else { 0.0 },
                y:     if self.active[4] { pose.y }     else { 0.0 },
                z:     if self.active[5] { pose.z }     else { 0.0 },
                timestamp_us: pose.timestamp_us,
            }
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            let idx = match param {
                "yaw"   => 0,
                "pitch" => 1,
                "roll"  => 2,
                "x"     => 3,
                "y"     => 4,
                "z"     => 5,
                _ => return false,
            };
            self.active[idx] = value > 0.5;
            true
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n.to_string(), if self.active[i] { 1.0 } else { 0.0 })
            }).collect()
        }
    }

    // ── One Euro Filter ──────────────────────────────────────────────────────

    /// Runs all six pose axes through the One Euro Filter with per-axis parameters.
    ///
    /// Parameters can be set globally or per-axis:
    /// - `set_param("min_cutoff", v)` — sets all axes
    /// - `set_param("yaw.min_cutoff", v)` — sets just yaw
    /// - `set_param("yaw.beta", v)` — sets just yaw's beta
    pub struct OneEuroStage {
        filter: PoseFilter,
    }

    impl OneEuroStage {
        pub fn new(min_cutoff: f32, beta: f32) -> Self {
            Self {
                filter: PoseFilter::new(min_cutoff, beta),
            }
        }

        pub fn filter(&self) -> &PoseFilter {
            &self.filter
        }
    }

    impl PipelineStage for OneEuroStage {
        fn name(&self) -> &str {
            "one-euro-filter"
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            self.filter.filter(pose, dt)
        }

        fn reset(&mut self) {
            self.filter.reset();
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::filter::AXIS_NAMES;

            // Per-axis: "yaw.min_cutoff", "x.beta", etc.
            if let Some((axis_name, field)) = param.split_once('.') {
                if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == axis_name) {
                    return self.filter.set_axis_param(idx, field, value);
                }
                return false;
            }

            // Global: applies to all axes at once.
            match param {
                "min_cutoff" | "beta" => {
                    let mut ok = false;
                    for i in 0..6 {
                        ok |= self.filter.set_axis_param(i, param, value);
                    }
                    ok
                }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            let mut out = Vec::new();
            for (i, &n) in AXIS_NAMES.iter().enumerate() {
                out.push((format!("{}.min_cutoff", n), self.filter.get_axis_param(i, "min_cutoff").unwrap_or(0.0)));
                out.push((format!("{}.beta", n), self.filter.get_axis_param(i, "beta").unwrap_or(0.0)));
            }
            out
        }
    }

    // ── Response Curves ────────────────────────────────────────────────────

    /// Per-axis power curve for sensitivity / acceleration.
    ///
    /// Applies `output = sign(input) * sensitivity * |input|^exponent`.
    ///
    /// - `exponent > 1.0` → small movements stay small, large movements amplified
    ///   (acceleration / dead feel near center)
    /// - `exponent < 1.0` → more sensitive near center, compressed at extremes
    /// - `exponent = 1.0` → linear (sensitivity is just a multiplier)
    /// - `sensitivity` → overall scale factor per axis
    ///
    /// Parameters via `set_param`:
    /// - `"yaw.exponent"`, `"pitch.sensitivity"`, etc. (per-axis)
    /// - `"exponent"`, `"sensitivity"` (all axes)
    pub struct ResponseCurveStage {
        exponent: [f32; 6],
        sensitivity: [f32; 6],
    }

    impl Default for ResponseCurveStage {
        fn default() -> Self { Self::new() }
    }

    impl ResponseCurveStage {
        pub fn new() -> Self {
            Self {
                //            yaw  pitch roll  x    y    z
                exponent:    [1.2, 1.2,  1.0,  1.0, 1.1, 1.0],
                sensitivity: [1.0, 1.0,  1.0,  1.0, 1.0, 1.5],
            }
        }

        /// Fast power approximation for exponents near 1.0.
        /// Uses IEEE 754 bit trick for ln approximation.
        /// Accurate to ~0.5% for x in [0, 100] and e in [0.5, 2.0].
        #[inline(always)]
        fn fast_powf(x: f32, e: f32) -> f32 {
            if x <= 0.0 { return 0.0; }
            if (e - 1.0).abs() < 0.001 { return x; }
            if (e - 2.0).abs() < 0.001 { return x * x; }
            // x^e ≈ x * (1 + (e-1) * ln(x)), using IEEE 754 fast ln
            let bits = x.to_bits() as i32;
            let ln_approx = (bits - 0x3F80_0000) as f32 * 8.262_958e-8;
            x * (1.0 + (e - 1.0) * ln_approx)
        }

        fn apply_curve(value: f32, exponent: f32, sensitivity: f32) -> f32 {
            let sign = value.signum();
            let abs = value.abs();
            sign * sensitivity * Self::fast_powf(abs, exponent)
        }
    }

    impl PipelineStage for ResponseCurveStage {
        fn name(&self) -> &str {
            "response-curve"
        }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
            let out: [f32; 6] = std::array::from_fn(|i| {
                Self::apply_curve(vals[i], self.exponent[i], self.sensitivity[i])
            });
            Pose {
                yaw: out[0], pitch: out[1], roll: out[2],
                x: out[3], y: out[4], z: out[5],
                timestamp_us: pose.timestamp_us,
            }
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::filter::AXIS_NAMES;

            // Per-axis: "yaw.exponent", "pitch.sensitivity"
            if let Some((axis_name, field)) = param.split_once('.') {
                let idx = match AXIS_NAMES.iter().position(|&n| n == axis_name) {
                    Some(i) => i,
                    None => return false,
                };
                return match field {
                    "exponent" => { self.exponent[idx] = value.max(0.1); true }
                    "sensitivity" => { self.sensitivity[idx] = value.max(0.0); true }
                    _ => false,
                };
            }

            // Global
            match param {
                "exponent" => {
                    let v = value.max(0.1);
                    self.exponent = [v; 6];
                    true
                }
                "sensitivity" => {
                    let v = value.max(0.0);
                    self.sensitivity = [v; 6];
                    true
                }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            let mut out = Vec::new();
            for (i, &n) in AXIS_NAMES.iter().enumerate() {
                out.push((format!("{}.exponent", n), self.exponent[i]));
                out.push((format!("{}.sensitivity", n), self.sensitivity[i]));
            }
            out
        }
    }

    // ── Cross-Axis Compensation ──────────────────────────────────────────

    /// Subtracts bleed from one axis into others.
    ///
    /// The main use case: webcam pose estimation causes yaw to bleed into
    /// pitch and Y (turning right makes the view drift up). This stage
    /// applies per-frame correction: `pitch -= yaw_delta * yaw_to_pitch`.
    ///
    /// Parameters via `set_param`:
    /// - `"yaw_to_pitch"` — fraction of yaw change subtracted from pitch
    /// - `"yaw_to_y"` — fraction of yaw change subtracted from Y
    /// - `"yaw_to_roll"` — fraction of yaw change subtracted from roll
    /// - `"pitch_to_y"` — fraction of pitch change subtracted from Y
    /// - `"yaw_to_x"` — fraction of yaw change subtracted from X
    /// - `"z_to_y"` — fraction of Z change subtracted from Y
    /// - `"z_to_pitch"` — fraction of Z change subtracted from pitch
    /// - `"roll_to_z"` — roll magnitude subtracted from Z
    /// - `"roll_to_x"` — roll angle (signed) subtracted from X
    /// - `"yaw_to_z"` — yaw magnitude subtracted from Z
    ///
    /// `roll_to_z`, `roll_to_x`, and `yaw_to_z` compensate NeuralNet model artifacts:
    /// head tilt/turn changes the face bounding box profile, which the model
    /// misreads as depth/lateral change. Measure coefficients from diagnostic CSVs
    /// with other axes masked and deliberate roll/yaw sweeps recorded.
    pub struct CrossAxisCompStage {
        yaw_to_pitch: f32,
        yaw_to_x: f32,
        yaw_to_y: f32,
        yaw_to_roll: f32,
        yaw_to_z: f32,
        pitch_to_y: f32,
        z_to_y: f32,
        z_to_pitch: f32,
        roll_to_z: f32,
        roll_to_x: f32,
        /// Neck pivot distance in mm (~170mm for adults). Used for
        /// geometric yaw→X/Y compensation that's camera-independent.
        pivot_mm: f32,
    }

    impl Default for CrossAxisCompStage {
        fn default() -> Self { Self::new() }
    }

    impl CrossAxisCompStage {
        pub fn new() -> Self {
            Self {
                yaw_to_pitch: 0.0,
                yaw_to_x: -0.40,
                yaw_to_y: -0.40,
                yaw_to_roll: -0.55,
                yaw_to_z: 0.0,   // measured per-camera from diagnostic CSV; 0 = disabled
                pitch_to_y: 0.0,
                z_to_y: 0.0,
                z_to_pitch: 0.0,
                roll_to_z: 0.0,  // measured per-camera from diagnostic CSV; 0 = disabled
                roll_to_x: 0.0,  // measured per-camera from diagnostic CSV; 0 = disabled
                pivot_mm: 170.0,
            }
        }
    }

    impl PipelineStage for CrossAxisCompStage {
        fn name(&self) -> &str {
            "cross-axis"
        }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            let yaw_rad = pose.yaw.to_radians();
            let _pitch_rad = pose.pitch.to_radians();

            // Geometric offset from neck pivot point (~170mm).
            // When you twist your neck left/right, your face translates slightly sideways and backwards.
            let geo_x = -self.pivot_mm * yaw_rad.sin();
            let geo_y = self.pivot_mm * 0.15 * yaw_rad.sin().abs(); 
            let geo_z = self.pivot_mm * (1.0 - yaw_rad.cos());

            Pose {
                yaw: pose.yaw,
                pitch: pose.pitch - pose.yaw * self.yaw_to_pitch - pose.z * self.z_to_pitch,
                roll: pose.roll - pose.yaw * self.yaw_to_roll,
                x: pose.x - geo_x - pose.yaw * self.yaw_to_x - pose.roll * self.roll_to_x,
                y: pose.y - geo_y - pose.yaw * self.yaw_to_y - pose.pitch * self.pitch_to_y - pose.z * self.z_to_y,
                z: pose.z - geo_z - pose.yaw.abs() * self.yaw_to_z - pose.roll.abs() * self.roll_to_z,
                timestamp_us: pose.timestamp_us,
            }
        }

        fn reset(&mut self) {}

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            match param {
                "yaw_to_pitch" => { self.yaw_to_pitch = value; true }
                "yaw_to_x" => { self.yaw_to_x = value; true }
                "yaw_to_y" => { self.yaw_to_y = value; true }
                "yaw_to_roll" => { self.yaw_to_roll = value; true }
                "yaw_to_z" => { self.yaw_to_z = value; true }
                "pitch_to_y" => { self.pitch_to_y = value; true }
                "z_to_y" => { self.z_to_y = value; true }
                "z_to_pitch" => { self.z_to_pitch = value; true }
                "roll_to_z" => { self.roll_to_z = value; true }
                "roll_to_x" => { self.roll_to_x = value; true }
                "pivot_mm" => { self.pivot_mm = value.max(0.0); true }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            vec![
                ("yaw_to_pitch".to_string(), self.yaw_to_pitch),
                ("yaw_to_x".to_string(), self.yaw_to_x),
                ("yaw_to_y".to_string(), self.yaw_to_y),
                ("yaw_to_roll".to_string(), self.yaw_to_roll),
                ("yaw_to_z".to_string(), self.yaw_to_z),
                ("pitch_to_y".to_string(), self.pitch_to_y),
                ("z_to_y".to_string(), self.z_to_y),
                ("z_to_pitch".to_string(), self.z_to_pitch),
                ("roll_to_z".to_string(), self.roll_to_z),
                ("roll_to_x".to_string(), self.roll_to_x),
                ("pivot_mm".to_string(), self.pivot_mm),
            ]
        }
    }

    // ── Slew Rate Limiter ───────────────────────────────────────────────────

    /// Per-axis slew rate limiter — caps how fast each axis can change per second.
    ///
    /// Primary purpose: kill single-frame outlier spikes from noisy inputs
    /// (especially Z depth-from-face-size) without adding latency.  Real movement
    /// passes through; only physically-impossible jumps get clamped.
    ///
    /// Parameters via `set_param`:
    /// - `"yaw"`, `"pitch"`, etc. — max rate for that axis (°/s or mm/s)
    /// - `"<axis>.max_rate"` — same thing, explicit form
    pub struct SlewLimitStage {
        /// Max change per second for each axis [yaw, pitch, roll, x, y, z].
        max_rate: [f32; 6],
        prev: Option<Pose>,
    }

    impl Default for SlewLimitStage {
        fn default() -> Self { Self::new() }
    }

    impl SlewLimitStage {
        /// Defaults: rotation 500°/s, XY 500mm/s, Z 200mm/s.
        ///
        /// Rotation and XY are high enough that real movement passes through.
        /// Z at 200 mm/s = 10mm per frame at 20Hz — real forward-lean is
        /// ~100-150mm/s, so this still allows it while killing the 300+ mm
        /// single-frame spikes from face detection flicker.
        pub fn new() -> Self {
            Self {
                //             yaw    pitch  roll   x      y      z
                max_rate: [500.0, 500.0, 500.0, 300.0, 300.0, 200.0],
                prev: None,
            }
        }
    }

    impl PipelineStage for SlewLimitStage {
        fn name(&self) -> &str {
            "slew-limit"
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            let Some(prev) = self.prev else {
                self.prev = Some(pose);
                return pose;
            };

            // Cap dt so a webcam freeze doesn't open the slew gate wide.
            // At 20 Hz, normal dt is ~0.05s; cap at 0.1s (10 Hz floor).
            let dt_safe = dt.clamp(0.001, 0.1);
            let prev_vals = [prev.yaw, prev.pitch, prev.roll, prev.x, prev.y, prev.z];
            let in_vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];

            let mut out = [0.0_f32; 6];
            for i in 0..6 {
                let delta = in_vals[i] - prev_vals[i];
                let max_delta = self.max_rate[i] * dt_safe;
                out[i] = prev_vals[i] + delta.clamp(-max_delta, max_delta);
            }

            let result = Pose {
                yaw: out[0], pitch: out[1], roll: out[2],
                x: out[3], y: out[4], z: out[5],
                timestamp_us: pose.timestamp_us,
            };
            self.prev = Some(result);
            result
        }

        fn reset(&mut self) {
            self.prev = None;
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::filter::AXIS_NAMES;

            // "z.max_rate" or just "z" — both set the max rate
            let (axis_name, _field) = param.split_once('.').unwrap_or((param, "max_rate"));
            if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == axis_name) {
                self.max_rate[idx] = value.max(1.0);
                return true;
            }
            false
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (format!("{}.max_rate", n), self.max_rate[i])
            }).collect()
        }
    }

    // ── Center / offset ──────────────────────────────────────────────────────

    /// Velocity-gated adaptive centering.
    ///
    /// Subtracts an adaptive baseline so the startup position (or last
    /// `reset()`) is the neutral point.  Unlike a simple constant-rate
    /// drift, the correction is **gated by head velocity**:
    ///
    /// - When the head is still (velocity below `still_threshold`), the
    ///   baseline drifts toward the current raw pose at `drift_rate`.
    /// - When the head is moving, drift correction is suppressed so
    ///   intentional movements don't contaminate the baseline.
    ///
    /// This prevents the problem where looking around for a while causes
    /// "center" to shift toward wherever you spent time looking.
    pub struct CenterStage {
        baseline: Option<Pose>,
        prev_pose: Option<Pose>,
        /// Base drift rate when still, in fraction-per-second.
        drift_rate: f32,
        /// Per-axis multiplier on drift_rate [yaw, pitch, roll, x, y, z].
        drift_mult: [f32; 6],
        /// Velocity (deg/s or mm/s) below which the head is considered still.
        still_threshold: f32,
    }

    impl Default for CenterStage {
        fn default() -> Self { Self::new() }
    }

    impl CenterStage {
        pub fn new() -> Self {
            Self {
                baseline: None,
                prev_pose: None,
                drift_rate: 0.05,
                //                yaw  pitch roll  x    y    z
                drift_mult: [1.0, 1.0, 1.0, 1.0, 1.0, 3.2],
                still_threshold: 3.0, // deg/s or mm/s — low enough to mean "not moving"
            }
        }
    }

    impl PipelineStage for CenterStage {
        fn name(&self) -> &str {
            "center"
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            match param {
                "drift_rate" => { self.drift_rate = value.max(0.0); true }
                "still_threshold" => { self.still_threshold = value.max(0.0); true }
                "yaw.drift_mult" => { self.drift_mult[0] = value.max(0.0); true }
                "pitch.drift_mult" => { self.drift_mult[1] = value.max(0.0); true }
                "roll.drift_mult" => { self.drift_mult[2] = value.max(0.0); true }
                "x.drift_mult" => { self.drift_mult[3] = value.max(0.0); true }
                "y.drift_mult" => { self.drift_mult[4] = value.max(0.0); true }
                "z.drift_mult" => { self.drift_mult[5] = value.max(0.0); true }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            let mut out = vec![
                ("drift_rate".to_string(), self.drift_rate),
                ("still_threshold".to_string(), self.still_threshold),
            ];
            for (i, &n) in AXIS_NAMES.iter().enumerate() {
                out.push((format!("{}.drift_mult", n), self.drift_mult[i]));
            }
            out
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            let b = self.baseline.get_or_insert(pose);

            // Compute velocity: max axis speed across all 6 axes.
            let velocity = if let Some(prev) = &self.prev_pose {
                let inv_dt = if dt > 0.0001 { 1.0 / dt } else { 0.0 };
                let speeds = [
                    (pose.yaw - prev.yaw).abs() * inv_dt,
                    (pose.pitch - prev.pitch).abs() * inv_dt,
                    (pose.roll - prev.roll).abs() * inv_dt,
                    (pose.x - prev.x).abs() * inv_dt,
                    (pose.y - prev.y).abs() * inv_dt,
                    (pose.z - prev.z).abs() * inv_dt,
                ];
                speeds.iter().cloned().fold(0.0_f32, f32::max)
            } else {
                0.0
            };
            self.prev_pose = Some(pose);

            // Gate: only correct when still. Smooth the gate so it doesn't
            // snap on/off — use a soft ramp from 1.0 (fully still) to 0.0
            // (moving at 2x threshold).
            let gate = ((1.0 - velocity / self.still_threshold.max(0.01)) * 2.0).clamp(0.0, 1.0);
            let base_alpha = self.drift_rate * gate * dt;

            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
            let b_vals = [b.yaw, b.pitch, b.roll, b.x, b.y, b.z];
            let mut new_b = [0.0f32; 6];
            for i in 0..6 {
                let alpha = (base_alpha * self.drift_mult[i]).min(1.0);
                new_b[i] = b_vals[i] + alpha * (vals[i] - b_vals[i]);
            }
            b.yaw   = new_b[0];
            b.pitch = new_b[1];
            b.roll  = new_b[2];
            b.x     = new_b[3];
            b.y     = new_b[4];
            b.z     = new_b[5];

            let b = *b;
            Pose {
                yaw:          pose.yaw   - b.yaw,
                pitch:        pose.pitch - b.pitch,
                roll:         pose.roll  - b.roll,
                x:            pose.x     - b.x,
                y:            pose.y     - b.y,
                z:            pose.z     - b.z,
                timestamp_us: pose.timestamp_us,
            }
        }

        fn reset(&mut self) {
            self.baseline = None;
            self.prev_pose = None;
        }
    }

    // ── Kalman Prediction ─────────────────────────────────────────────────

    /// Per-axis Kalman filter with forward prediction.
    ///
    /// Each axis runs an independent scalar Kalman filter with a
    /// constant-velocity model.  This simultaneously smooths the signal
    /// and estimates velocity, then extrapolates by `predict_ms` to
    /// compensate end-to-end system latency.
    ///
    /// Rotation and translation axes have independent noise tuning:
    /// rotation is already fairly clean from the ONNX quaternion output,
    /// while translation (especially Z) is derived from bounding box
    /// geometry and is inherently noisier.
    ///
    /// Parameters via `set_param`:
    /// - `"predict_ms"` — forward prediction lookahead (default 30ms)
    /// - `"rot_process_noise"` — process noise Q for yaw/pitch/roll
    /// - `"rot_measurement_noise"` — measurement noise R for yaw/pitch/roll
    /// - `"pos_process_noise"` — process noise Q for x/y/z
    /// - `"pos_measurement_noise"` — measurement noise R for x/y/z
    /// - `"<axis>.process_noise"` — per-axis process noise override
    /// - `"<axis>.measurement_noise"` — per-axis measurement noise override
    pub struct PredictionStage {
        filters: [ScalarKalman; 6],
        predict_ms: f32,
    }

    impl Default for PredictionStage {
        fn default() -> Self { Self::new() }
    }

    impl PredictionStage {
        pub fn new() -> Self {
            // Tuned 2026-03-19 via 814s in-sim CSV analysis with param logging.
            //
            // Rotation: low Q (1.2) = slow velocity adaptation = prevents rubber-band
            // overshoot on direction reversal. R=0.7 trusts measurements closely.
            // Previous Q=5.0 caused overshoot; Q=1.2 cut max overshoot from 51° to 35°.
            let rot_q = 1.2;
            let rot_r = 0.7;

            // Translation: moderate Q, low R = trust bounding-box measurements.
            // Previous R=50.0 made translation feel sluggish; R=10 is responsive
            // without amplifying jitter (slew limiter + deadzone catch outliers).
            let pos_q = 2.5;
            let pos_r = 10.0;

            // Z: same Q/R as X/Y — the heavy One Euro (beta=0.10) + slew limiter
            // + 3.0mm deadzone handle Z noise upstream. No need for extra Kalman damping.
            let z_q = 2.5;
            let z_r = 10.0;

            Self {
                filters: [
                    ScalarKalman::new(rot_q, rot_r),   // yaw
                    ScalarKalman::new(rot_q, rot_r),   // pitch
                    ScalarKalman::new(rot_q, rot_r),   // roll
                    ScalarKalman::new(pos_q, pos_r),   // x
                    ScalarKalman::new(pos_q, pos_r),   // y
                    ScalarKalman::new(z_q,   z_r),     // z
                ],
                predict_ms: 10.0, // 10ms forward prediction — compensates system latency
            }
        }
    }

    impl PipelineStage for PredictionStage {
        fn name(&self) -> &str {
            "prediction"
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
            let predict_s = self.predict_ms / 1000.0;
            let out: [f32; 6] = std::array::from_fn(|i| {
                self.filters[i].update(vals[i], dt, predict_s)
            });
            Pose {
                yaw: out[0], pitch: out[1], roll: out[2],
                x: out[3], y: out[4], z: out[5],
                timestamp_us: pose.timestamp_us,
            }
        }

        fn reset(&mut self) {
            for f in &mut self.filters {
                f.reset();
            }
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::filter::AXIS_NAMES;

            match param {
                "predict_ms" => {
                    self.predict_ms = value.clamp(0.0, 100.0);
                    true
                }
                "rot_process_noise" => {
                    for i in 0..3 { self.filters[i].set_process_noise(value); }
                    true
                }
                "rot_measurement_noise" => {
                    for i in 0..3 { self.filters[i].set_measurement_noise(value); }
                    true
                }
                "pos_process_noise" => {
                    for i in 3..6 { self.filters[i].set_process_noise(value); }
                    true
                }
                "pos_measurement_noise" => {
                    for i in 3..6 { self.filters[i].set_measurement_noise(value); }
                    true
                }
                _ => {
                    // Per-axis: "yaw.process_noise", "z.measurement_noise"
                    if let Some((axis_name, field)) = param.split_once('.') {
                        if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == axis_name) {
                            return match field {
                                "process_noise" => {
                                    self.filters[idx].set_process_noise(value);
                                    true
                                }
                                "measurement_noise" => {
                                    self.filters[idx].set_measurement_noise(value);
                                    true
                                }
                                _ => false,
                            };
                        }
                    }
                    false
                }
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            let mut out = vec![("predict_ms".to_string(), self.predict_ms)];
            for (i, &n) in AXIS_NAMES.iter().enumerate() {
                out.push((format!("{}.process_noise", n), self.filters[i].process_noise()));
                out.push((format!("{}.measurement_noise", n), self.filters[i].measurement_noise()));
            }
            out
        }
    }

    // ── Soft Deadzone ──────────────────────────────────────────────────────

    /// Per-axis soft deadzone with smooth cubic ease-in.
    ///
    /// Kills micro-jitter at rest without introducing a hard "pop" when
    /// movement starts.  The blend region spans one deadzone width beyond the
    /// threshold, giving C1 continuity with the identity function.
    ///
    /// Behaviour:
    /// - `|x| <= dz`           → 0 (fully suppressed)
    /// - `dz < |x| <= 2·dz`   → cubic ease from 0 to linear
    /// - `|x| > 2·dz`         → `sign(x) · (|x| − dz)` (shifted identity)
    ///
    /// Parameters via `set_param`:
    /// - `"yaw"`, `"pitch"`, `"roll"`, `"x"`, `"y"`, `"z"` — deadzone radius
    pub struct DeadzoneStage {
        /// Deadzone radius per axis [yaw°, pitch°, roll°, x mm, y mm, z mm].
        dz: [f32; 6],
    }

    impl Default for DeadzoneStage {
        fn default() -> Self { Self::new() }
    }

    impl DeadzoneStage {
        pub fn new() -> Self {
            Self {
                //       yaw   pitch  roll   x     y     z
                dz: [0.8,  0.8,   0.5,  2.0,  2.0,  3.0],
            }
        }

        /// Soft deadzone for a single axis value.
        fn apply(x: f32, dz: f32) -> f32 {
            if dz <= 0.0 {
                return x;
            }
            let abs_x = x.abs();
            if abs_x <= dz {
                // Inside deadzone — fully suppressed.
                0.0
            } else if abs_x <= 2.0 * dz {
                // Blend region: cubic Hermite ease from 0 to linear.
                // t ∈ [0, 1] across one dz width.
                // f(t) = dz · t² · (2 − t), with f(0)=0, f'(0)=0, f(1)=dz, f'(1)=1·dz.
                let t = (abs_x - dz) / dz;
                x.signum() * dz * t * t * (2.0 - t)
            } else {
                // Past blend region — shifted identity.
                x.signum() * (abs_x - dz)
            }
        }
    }

    impl PipelineStage for DeadzoneStage {
        fn name(&self) -> &str {
            "deadzone"
        }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            Pose {
                yaw:   Self::apply(pose.yaw,   self.dz[0]),
                pitch: Self::apply(pose.pitch, self.dz[1]),
                roll:  Self::apply(pose.roll,  self.dz[2]),
                x:     Self::apply(pose.x,     self.dz[3]),
                y:     Self::apply(pose.y,     self.dz[4]),
                z:     Self::apply(pose.z,     self.dz[5]),
                timestamp_us: pose.timestamp_us,
            }
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            let idx = match param {
                "yaw"   => Some(0),
                "pitch" => Some(1),
                "roll"  => Some(2),
                "x"     => Some(3),
                "y"     => Some(4),
                "z"     => Some(5),
                _ => None,
            };
            if let Some(i) = idx {
                self.dz[i] = value.max(0.0);
                true
            } else {
                false
            }
        }

        fn get_params(&self) -> Vec<(String, f32)> {
            use crate::filter::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n.to_string(), self.dz[i])
            }).collect()
        }
    }
}
