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
    fn name(&self) -> &'static str;
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
    fn get_params(&self) -> Vec<(&'static str, f32)> {
        Vec::new()
    }

    /// Tell the stage how far the user is sitting from the camera, so that
    /// noise-denominated parameters track the noise instead of the millimetre.
    ///
    /// See [`DistanceScale`]. Stages whose parameters are pure gains or angles
    /// ignore this; only stages holding a translation *threshold* in millimetres
    /// need it. Authored values are never modified — the scale is applied where
    /// the parameter is used, so the GUI keeps showing what the user typed.
    fn set_distance_scale(&mut self, _scale: DistanceScale) {}
}

/// Per-axis multipliers that make a translation tuning independent of how far
/// the user happens to sit from their camera.
///
/// **Why this exists.** Depth from a landmark baseline is `Z = f·B/b`, so a
/// fixed landmark error `e` in pixels produces
///
/// ```text
/// ΔZ = e · Z²/(f·B)      depth  error grows with the SQUARE of standoff
/// ΔX = e · Z/f           lateral error grows LINEARLY
/// ```
///
/// Measured on real landmark constellations replayed across 450–1000 mm
/// (`scripts/cue_pitch_eval.py`'s sibling analysis): **Z ∝ d^2.00, X ∝ d^1.08,
/// Y ∝ d^1.00** — the prediction to two decimals, for every multi-pair cue.
///
/// The practical consequence is large and not obvious: a user at 900 mm sees
/// **4×** the depth noise of one at 450 mm from the identical tracker. A single
/// fixed tuning cannot serve both, and telling users where to sit is not a
/// product. So thresholds are authored once at [`REFERENCE_DISTANCE_MM`] and
/// scaled here against the standoff the engine actually measures.
///
/// Rotation is untouched: an angle is an angle at any distance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceScale {
    /// Multiplier for X and Y thresholds — linear in standoff.
    pub lateral: f32,
    /// Multiplier for Z thresholds — quadratic in standoff.
    pub depth: f32,
}

/// The standoff every shipped profile is authored against, in millimetres, **as
/// the `diagonals` depth cue reports it**. The flown tuning was validated at a
/// reported ~620 mm under the old IPD cue; the multi-pair cues read the same seat
/// ~1.27× farther (back-to-back on 2026-08-20: IPD 583 mm, diagonals 743 mm), and
/// every diagonals session flown on that tuning pools to a median of 786 mm. Both
/// routes land here. Was 610 until P13 (2026-10-18) — an IPD-scale number that
/// widened every translation deadzone ~1.6× on Z at the seat the tuning was flown
/// from. See `docs/P13_RETUNE.md`.
pub const REFERENCE_DISTANCE_MM: f32 = 785.0;

/// Standoff range the scaling is trusted over. Outside it the scale is clamped:
/// a face detected at 200 mm or 3 m is far more likely to be a bad depth reading
/// than a real seating position, and letting that drive the tuning would let one
/// bad frame detune the whole pipeline.
pub const MIN_TRUSTED_DISTANCE_MM: f32 = 350.0;
/// Upper end of the trusted standoff range. See [`MIN_TRUSTED_DISTANCE_MM`].
pub const MAX_TRUSTED_DISTANCE_MM: f32 = 1100.0;

impl DistanceScale {
    /// Neutral scale — every threshold used exactly as authored.
    pub const NONE: Self = Self { lateral: 1.0, depth: 1.0 };

    /// Scale for a measured standoff in millimetres (sign-insensitive).
    pub fn from_standoff_mm(standoff_mm: f32) -> Self {
        let d = standoff_mm
            .abs()
            .clamp(MIN_TRUSTED_DISTANCE_MM, MAX_TRUSTED_DISTANCE_MM);
        let ratio = d / REFERENCE_DISTANCE_MM;
        Self { lateral: ratio, depth: ratio * ratio }
    }

    /// Per-axis multiplier in pipeline axis order
    /// `[yaw, pitch, roll, x, y, z]`. Rotation axes are always 1.0.
    pub fn per_axis(self) -> [f32; 6] {
        [1.0, 1.0, 1.0, self.lateral, self.lateral, self.depth]
    }
}

/// Whether a head source's translation thresholds follow seating distance.
/// Chosen per source by [`SourceKind::distance_scale_policy`](crate::input::SourceKind::distance_scale_policy).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistanceScalePolicy {
    /// Scale from the measured standoff (webcam: landmark depth noise grows
    /// with the square of distance).
    FromStandoff,
    /// Always [`DistanceScale::NONE`] (Tobii: translation is held at 0 and
    /// there is no landmark-baseline depth to scale against).
    None,
}

impl DistanceScalePolicy {
    /// The scale this policy gives for a measured standoff in millimetres.
    pub fn scale_for(self, standoff_mm: f32) -> DistanceScale {
        match self {
            Self::FromStandoff => DistanceScale::from_standoff_mm(standoff_mm),
            Self::None => DistanceScale::NONE,
        }
    }
}

/// An ordered chain of [`PipelineStage`]s.
///
/// Each stage's output is the next stage's input. The live order is defined by
/// [`build_default_pipeline`] — currently:
/// `center → median-filter → slew-limit → cross-axis → accela → response-curve
/// → deadzone → axis-mask`.
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
    /// Push the standoff-derived scale to every stage that holds a translation
    /// threshold. Cheap and idempotent — the caller decides how often to call it.
    pub fn set_distance_scale(&mut self, scale: DistanceScale) {
        for s in &mut self.stages {
            s.set_distance_scale(scale);
        }
    }

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
    pub fn get_all_params(&self) -> Vec<(&'static str, &'static str, f32)> {
        let mut out = Vec::new();
        for s in &self.stages {
            let name = s.name();
            for (p, v) in s.get_params() {
                out.push((name, p, v));
            }
        }
        out
    }

    pub fn stage_names(&self) -> impl Iterator<Item = &str> {
        self.stages.iter().map(|s| s.name())
    }
}

/// Build the production pipeline with the tuned default stage chain.
///
/// This is the **single source of truth** for stage order. Both the engine run
/// loop and the offline replay tool (`cargo run -p headtrack-core --example
/// replay`) call it, so what is tuned offline is exactly what ships.
///
/// One chain, no backend switch. The legacy One-Euro + Kalman-prediction stack
/// was removed in Snow Leopard 2 after Accela won both the offline bake-off
/// (`scripts/filter_bakeoff.py`) and the in-sim FB2 A/B — see
/// `docs/MSFS_TICK_INVESTIGATION.md`. Accela sits in the post-cross-axis slot
/// so it also cleans the coupling stage's output.
pub fn build_default_pipeline() -> Pipeline {
    use stages::*;

    Pipeline::new(vec![
        Box::new(CenterStage::new()),
        Box::new(MedianFilterStage::new()),
        Box::new(SlewLimitStage::new()),
        Box::new(CrossAxisCompStage::new()),
        Box::new(AccelaStage::new()),
        Box::new(ResponseCurveStage::new()),
        Box::new(DeadzoneStage::new()),
        Box::new(AxisMaskStage::new()),
    ])
}

/// Canonical stage names — the identifiers [`Pipeline::set_param`] dispatches on.
///
/// `Pipeline::set_param` matches the caller's string against `stage.name()` and
/// returns `false` when nothing matches. The GUI ignores that return, so before
/// SL3 a mistyped stage string was silent: the slider moved, the label updated,
/// and nothing reached the pipeline. `param_names.rs` documented the hole it
/// could not close — a test can only check the strings someone remembered to
/// add to it.
///
/// These constants close it. Every `PipelineStage::name()` returns one, and
/// every caller that addresses a stage by name uses one, so a typo is a
/// compile error and renaming a stage is a single edit.
///
/// (Named `stage_names` because `stages` already holds the stage *types*.)
pub mod stage_names {
    pub const CENTER: &str = "center";
    pub const MEDIAN_FILTER: &str = "median-filter";
    pub const SLEW_LIMIT: &str = "slew-limit";
    pub const CROSS_AXIS: &str = "cross-axis";
    pub const ACCELA: &str = "accela";
    pub const RESPONSE_CURVE: &str = "response-curve";
    pub const DEADZONE: &str = "deadzone";
    pub const AXIS_MASK: &str = "axis-mask";

    /// Every stage name, in pipeline order.
    pub const ALL: [&str; 8] = [
        CENTER,
        MEDIAN_FILTER,
        SLEW_LIMIT,
        CROSS_AXIS,
        ACCELA,
        RESPONSE_CURVE,
        DEADZONE,
        AXIS_MASK,
    ];
}

/// Flatten a [`ProfileConfig`](crate::config::ProfileConfig) into the exact
/// list of `(stage, param, value)` triples that define it.
///
/// **This is the single walk over a profile.** Two consumers iterate it: the
/// engine/replay path via [`apply_config`], and the GUI, which pushes the same
/// triples through its command channel. Before this existed the two walks were
/// hand-maintained copies and had silently diverged — `cross_axis.yaw_to_z` was
/// in the GUI's list and missing from the applier's, so engine startup, profile
/// reload and offline replay all disagreed with what the GUI displayed for any
/// profile that used it. Adding a field in one place and not the other is no
/// longer possible.
///
/// Guarded by `apply_config_pushes_every_profile_field`, which fills a profile
/// with unique sentinels and fails by name if any never reaches the pipeline.
pub fn profile_params(cfg: &crate::config::ProfileConfig) -> Vec<(&'static str, String, f32)> {
    use crate::pose::AXIS_NAMES;
    use stage_names as sn;
    let mut out = Vec::with_capacity(64);

    for (i, axis) in AXIS_NAMES.iter().enumerate() {
        out.push((sn::ACCELA, format!("{axis}.deadzone"), cfg.accela[i].deadzone));
        out.push((sn::ACCELA, format!("{axis}.smoothing"), cfg.accela[i].smoothing));
        out.push((sn::ACCELA, format!("{axis}.exponent"), cfg.accela[i].exponent));
        out.push((sn::RESPONSE_CURVE, format!("{axis}.sensitivity"), cfg.curve[i].sensitivity));
        out.push((sn::RESPONSE_CURVE, format!("{axis}.exponent"), cfg.curve[i].exponent));
        out.push((sn::DEADZONE, (*axis).to_string(), cfg.deadzone[i]));
        out.push((sn::SLEW_LIMIT, (*axis).to_string(), cfg.slew_limit[i]));
    }

    let x = &cfg.cross_axis;
    for (name, value) in [
        ("yaw_to_pitch", x.yaw_to_pitch),
        ("yaw_to_x", x.yaw_to_x),
        ("yaw_to_y", x.yaw_to_y),
        ("yaw_to_roll", x.yaw_to_roll),
        ("yaw_to_z", x.yaw_to_z),
        ("pitch_to_y", x.pitch_to_y),
        ("pitch_to_yaw", x.pitch_to_yaw),
        ("pitch_to_z", x.pitch_to_z),
        ("z_to_y", x.z_to_y),
        ("z_to_pitch", x.z_to_pitch),
        ("roll_to_z", x.roll_to_z),
        ("roll_to_x", x.roll_to_x),
    ] {
        out.push((sn::CROSS_AXIS, name.to_string(), value));
    }

    out.push((sn::CENTER, "drift_rate".to_string(), cfg.center.drift_rate));
    out.push((sn::CENTER, "z.drift_mult".to_string(), cfg.center.z_drift_mult));
    if let Some(ranges) = cfg.center.recenter_range {
        for (i, axis) in AXIS_NAMES.iter().enumerate() {
            out.push((sn::CENTER, format!("{axis}.recenter_range"), ranges[i]));
        }
    }

    out
}

/// Apply every parameter in a [`ProfileConfig`](crate::config::ProfileConfig)
/// onto an already-built [`Pipeline`].
///
/// A push to a stage that is not in the chain no-ops silently
/// (`Pipeline::set_param` returns `false` for an unknown stage name).
///
/// Note: applying [`ProfileConfig::default`](crate::config::ProfileConfig) is
/// *not* identical to an untouched [`build_default_pipeline`] — a few built-in
/// profile defaults diverge from the stage constructors (notably cross-axis
/// `z_to_y`, `-0.50` in the profile vs `0.0` in the stage). Callers that need to
/// reproduce the engine exactly must mirror its branch: apply only when a saved
/// profile exists, otherwise leave the pipeline untouched.
pub fn apply_config(pipeline: &mut Pipeline, cfg: &crate::config::ProfileConfig) {
    for (stage, param, value) in profile_params(cfg) {
        pipeline.set_param(stage, &param, value);
    }
}

/// Built-in pipeline stages, ready to box and push into a [`Pipeline`].
pub mod stages {
    use super::PipelineStage;
    use crate::pose::Pose;

    // ── Z-Axis Median Pre-Filter ──────────────────────────────────────

    /// Per-axis 3-sample running median — kills single-frame depth spikes.
    ///
    /// Enabled axes are independently median-filtered; disabled axes pass through.
    /// Default: pitch and Z enabled (the two axes most prone to NeuralNet outliers).
    /// Median-of-3 rejects single-frame outliers with zero latency on real movement.
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
        /// yaw (index 0), pitch (index 1) and Z (index 5) enabled by default.
        /// Yaw is the noisiest rotation axis under the new PnP pipeline (sub-
        /// pixel landmark drift translates straight into yaw step-quantization
        /// that the eye picks up as "popping"). A 3-tap median rejects single-
        /// frame outliers with effectively zero perceptual lag and was the
        /// single biggest perceptual smoothness win in A/B testing 2026-05-10.
        pub fn new() -> Self {
            Self {
                bufs:    [[0.0; 3]; 6],
                idx:     [0; 6],
                count:   [0; 6],
                //          yaw   pitch roll   x      y      z
                enabled: [true, true, false, false, false, true],
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
        fn name(&self) -> &'static str { crate::pipeline::stage_names::MEDIAN_FILTER }

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
            use crate::pose::AXIS_NAMES;
            if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == param) {
                self.enabled[idx] = value > 0.5;
                return true;
            }
            false
        }

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            use crate::pose::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n, if self.enabled[i] { 1.0 } else { 0.0 })
            }).collect()
        }
    }

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
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::AXIS_MASK
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

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            use crate::pose::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n, if self.active[i] { 1.0 } else { 0.0 })
            }).collect()
        }
    }

    // ── Accela displacement filter ────────────────────────────────────────────

    /// opentrack-style displacement smoother — the research-selected alternative
    /// to One-Euro + Kalman prediction. Per axis, each frame:
    /// ```text
    ///   d = input - last_output                 (displacement from our own output)
    ///   if |d| <= deadzone: hold                (kills at-rest noise pre-smoothing)
    ///   d -= sign(d)*deadzone
    ///   gain = min((|d|/smoothing)^exponent, max_gain)   (units/s; the Accela curve)
    ///   step = sign(d)*gain*dt, clamped to never overshoot d
    ///   last_output += step
    /// ```
    /// No velocity state and no prediction → it cannot escalate (as output nears
    /// input, d→0, gain→0). Faithful to `scripts/filter_bakeoff.py::Accela` so the
    /// offline bake-off and production agree. The default backend since 2026-08-01
    /// (the pipeline's only smoother since Snow Leopard 2).
    /// Upper bound on the gain curve's output — a safety clamp against a
    /// pathological input spike, not a tuning knob.
    const DEFAULT_MAX_GAIN: f32 = 600.0;

    pub struct AccelaStage {
        last: [Option<f32>; 6],
        deadzone: [f32; 6],
        smoothing: [f32; 6],
        exponent: [f32; 6],
        max_gain: [f32; 6],
        /// Standoff scaling for the translation axes — applied to `deadzone`
        /// **only**.
        ///
        /// `smoothing` is in axis units too, but it sets how fast the filter
        /// converges on its target, and a real head movement is the same number
        /// of millimetres at any distance. Scaling it made the filter sluggish
        /// on genuine motion for no noise benefit the deadzone was not already
        /// providing — measured on a real capture as 50% of true Z travel lost
        /// against a 43% noise reduction, i.e. very nearly a wash.
        dist: [f32; 6],
    }

    impl Default for AccelaStage {
        fn default() -> Self {
            Self::new()
        }
    }

    impl AccelaStage {
        /// Per-axis defaults from the bake-off frontier: light on rotation
        /// (responsive), heavier on Z (chronic-noise axis; lag imperceptible).
        ///
        /// The numbers live in [`ProfileConfig::default_accela`] and are read
        /// from there, not copied. Until SL3 both sites held the same literals
        /// with a comment insisting they match and a test checking it; sharing
        /// one definition makes the check unnecessary.
        ///
        /// `max_gain` is not among them by design — it is a safety clamp, not a
        /// tuning parameter, and is deliberately absent from the profile (see
        /// [`AccelaAxis`](crate::config::AccelaAxis)).
        pub fn new() -> Self {
            let axes = crate::config::ProfileConfig::default_accela();
            Self {
                last: [None; 6],
                deadzone: std::array::from_fn(|i| axes[i].deadzone),
                smoothing: std::array::from_fn(|i| axes[i].smoothing),
                exponent: std::array::from_fn(|i| axes[i].exponent),
                max_gain: [DEFAULT_MAX_GAIN; 6],
                dist: crate::pipeline::DistanceScale::NONE.per_axis(),
            }
        }

        fn step_axis(&mut self, i: usize, x: f32, dt: f32) -> f32 {
            let Some(y) = self.last[i] else {
                self.last[i] = Some(x);
                return x;
            };
            let mut d = x - y;
            let dz = self.deadzone[i] * self.dist[i];
            let sm = self.smoothing[i];
            if d.abs() <= dz {
                return y;
            }
            d -= dz.copysign(d);
            let ad = d.abs();
            let xn = ad / sm.max(1e-4);
            let gain = xn.powf(self.exponent[i]).min(self.max_gain[i]);
            let mut step = (gain * dt).copysign(d);
            if step.abs() > ad {
                step = d; // never overshoot the (post-deadzone) input
            }
            let yn = y + step;
            self.last[i] = Some(yn);
            yn
        }
    }

    impl PipelineStage for AccelaStage {
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::ACCELA
        }

        fn set_distance_scale(&mut self, scale: crate::pipeline::DistanceScale) {
            self.dist = scale.per_axis();
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
            let out: [f32; 6] = std::array::from_fn(|i| self.step_axis(i, vals[i], dt));
            Pose {
                yaw: out[0],
                pitch: out[1],
                roll: out[2],
                x: out[3],
                y: out[4],
                z: out[5],
                timestamp_us: pose.timestamp_us,
            }
        }

        fn reset(&mut self) {
            self.last = [None; 6];
        }

        fn set_param(&mut self, param: &str, value: f32) -> bool {
            use crate::pose::AXIS_NAMES;
            let Some((axis, field)) = param.split_once('.') else {
                return false;
            };
            let Some(i) = AXIS_NAMES.iter().position(|&n| n == axis) else {
                return false;
            };
            match field {
                "deadzone" => self.deadzone[i] = value.max(0.0),
                "smoothing" => self.smoothing[i] = value.max(1e-4),
                "exponent" => self.exponent[i] = value.max(0.1),
                "max_gain" => self.max_gain[i] = value.max(0.0),
                _ => return false,
            }
            true
        }

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            const NAMES: [&str; 24] = [
                "yaw.deadzone", "yaw.smoothing", "yaw.exponent", "yaw.max_gain",
                "pitch.deadzone", "pitch.smoothing", "pitch.exponent", "pitch.max_gain",
                "roll.deadzone", "roll.smoothing", "roll.exponent", "roll.max_gain",
                "x.deadzone", "x.smoothing", "x.exponent", "x.max_gain",
                "y.deadzone", "y.smoothing", "y.exponent", "y.max_gain",
                "z.deadzone", "z.smoothing", "z.exponent", "z.max_gain",
            ];
            let mut out = Vec::with_capacity(24);
            for i in 0..6 {
                out.push((NAMES[i * 4], self.deadzone[i]));
                out.push((NAMES[i * 4 + 1], self.smoothing[i]));
                out.push((NAMES[i * 4 + 2], self.exponent[i]));
                out.push((NAMES[i * 4 + 3], self.max_gain[i]));
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
                exponent:    [1.2, 1.2,  1.0,  1.0, 1.0, 1.0],
                // Z sensitivity was 1.5 until 2026-08-17. It was measured as
                // the whole of the Z-rest amplification (the depth cue is
                // already the noisiest axis; scaling it up amplified the
                // noise, not the signal), fixed in the tuned profile, and only
                // now folded back into the default everyone starts from.
                sensitivity: [1.0, 1.0,  1.0,  1.0, 1.0, 1.0],
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
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::RESPONSE_CURVE
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
            use crate::pose::AXIS_NAMES;

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

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            const NAMES: &[&str] = &[
                "yaw.exponent", "yaw.sensitivity",
                "pitch.exponent", "pitch.sensitivity",
                "roll.exponent", "roll.sensitivity",
                "x.exponent", "x.sensitivity",
                "y.exponent", "y.sensitivity",
                "z.exponent", "z.sensitivity",
            ];
            let mut out = Vec::with_capacity(NAMES.len());
            for i in 0..6 {
                out.push((NAMES[i * 2],     self.exponent[i]));
                out.push((NAMES[i * 2 + 1], self.sensitivity[i]));
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
        pitch_to_yaw: f32,
        pitch_to_z: f32,
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
                // yaw→X/Y/roll were −0.40/−0.40/−0.55 until 2026-08-17, from
                // the era when PnP drove rotation. The tuned profile has
                // zeroed them since v1 — the geometric `pivot_mm` term already
                // covers the yaw→X/Y case, and stacking both over-corrects.
                yaw_to_x: 0.0,
                yaw_to_y: 0.0,
                yaw_to_roll: 0.0,
                yaw_to_z: 0.0,   // measured per-camera from diagnostic CSV; 0 = disabled
                pitch_to_y: 1.75,  // nod → eye-centroid rise; measured
                pitch_to_yaw: 0.05, // residual heading pull when looking down/up
                pitch_to_z: -1.5,   // nod → IPD foreshortening reads as depth
                z_to_y: 0.0,
                z_to_pitch: 0.0,
                roll_to_z: 0.0,  // measured per-camera from diagnostic CSV; 0 = disabled
                roll_to_x: 0.0,  // measured per-camera from diagnostic CSV; 0 = disabled
                pivot_mm: 170.0,
            }
        }
    }

    impl PipelineStage for CrossAxisCompStage {
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::CROSS_AXIS
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
                yaw: pose.yaw - pose.pitch * self.pitch_to_yaw,
                pitch: pose.pitch - pose.yaw * self.yaw_to_pitch - pose.z * self.z_to_pitch,
                roll: pose.roll - pose.yaw * self.yaw_to_roll,
                x: pose.x - geo_x - pose.yaw * self.yaw_to_x - pose.roll * self.roll_to_x,
                y: pose.y - geo_y - pose.yaw * self.yaw_to_y - pose.pitch * self.pitch_to_y - pose.z * self.z_to_y,
                z: pose.z - geo_z - pose.yaw.abs() * self.yaw_to_z - pose.roll.abs() * self.roll_to_z - pose.pitch * self.pitch_to_z,
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
                "pitch_to_yaw" => { self.pitch_to_yaw = value; true }
                "pitch_to_z" => { self.pitch_to_z = value; true }
                "z_to_y" => { self.z_to_y = value; true }
                "z_to_pitch" => { self.z_to_pitch = value; true }
                "roll_to_z" => { self.roll_to_z = value; true }
                "roll_to_x" => { self.roll_to_x = value; true }
                "pivot_mm" => { self.pivot_mm = value.max(0.0); true }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            vec![
                ("yaw_to_pitch", self.yaw_to_pitch),
                ("yaw_to_x",     self.yaw_to_x),
                ("yaw_to_y",     self.yaw_to_y),
                ("yaw_to_roll",  self.yaw_to_roll),
                ("yaw_to_z",     self.yaw_to_z),
                ("pitch_to_y",   self.pitch_to_y),
                ("pitch_to_yaw", self.pitch_to_yaw),
                ("pitch_to_z",   self.pitch_to_z),
                ("z_to_y",       self.z_to_y),
                ("z_to_pitch",   self.z_to_pitch),
                ("roll_to_z",    self.roll_to_z),
                ("roll_to_x",    self.roll_to_x),
                ("pivot_mm",     self.pivot_mm),
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
        /// Retained so the stage can accept the scale without acting on it.
        ///
        /// Deliberately **unused**: this cap is sized to sit just above genuine
        /// head movement, and genuine movement is the same mm/s however far the
        /// user sits. Widening it with standoff would admit *larger* noise
        /// spikes — the opposite of the intent. Kept as a field so the reason
        /// lives next to the decision rather than in a commit message.
        #[allow(dead_code)]
        dist: [f32; 6],
    }

    impl Default for SlewLimitStage {
        fn default() -> Self { Self::new() }
    }

    impl SlewLimitStage {
        /// Defaults: yaw/roll 500°/s, pitch 80°/s, XY 300mm/s, Z 200mm/s.
        ///
        /// Yaw, roll and XY are high enough that real movement passes through.
        /// Z at 200 mm/s = 10mm per frame at 20Hz — real forward-lean is
        /// ~100-150mm/s, so this still allows it while killing the 300+ mm
        /// single-frame spikes from face detection flicker. Pitch is held to
        /// 80°/s because it is the axis that spikes: eyelid and expression
        /// changes move the landmark set vertically, and a real nod does not
        /// exceed it.
        pub fn new() -> Self {
            Self {
                //             yaw    pitch  roll   x      y      z
                max_rate: [500.0,  80.0, 500.0, 300.0, 300.0, 200.0],
                prev: None,
                dist: crate::pipeline::DistanceScale::NONE.per_axis(),
            }
        }
    }

    impl PipelineStage for SlewLimitStage {
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::SLEW_LIMIT
        }

        fn set_distance_scale(&mut self, scale: crate::pipeline::DistanceScale) {
            self.dist = scale.per_axis();
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
            use crate::pose::AXIS_NAMES;

            // "z.max_rate" or just "z" — both set the max rate
            let (axis_name, _field) = param.split_once('.').unwrap_or((param, "max_rate"));
            if let Some(idx) = AXIS_NAMES.iter().position(|&n| n == axis_name) {
                self.max_rate[idx] = value.max(1.0);
                return true;
            }
            false
        }

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            const NAMES: &[&str] = &[
                "yaw.max_rate", "pitch.max_rate", "roll.max_rate",
                "x.max_rate", "y.max_rate", "z.max_rate",
            ];
            NAMES.iter().enumerate().map(|(i, &n)| (n, self.max_rate[i])).collect()
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
        /// Frames collected since the last [`reset`](PipelineStage::reset), used
        /// to build the baseline from a robust median instead of one sample.
        /// See [`CenterStage::BASELINE_WINDOW_S`].
        baseline_samples: Vec<[f32; 6]>,
        /// Seconds of *still* samples gathered so far for the baseline.
        baseline_elapsed_s: f32,
        /// Wall-clock seconds since the last reset, so a user who never holds
        /// still cannot keep the window open forever.
        baseline_age_s: f32,
        /// EMA-smoothed pose used for gate velocity estimation. Raw
        /// frame-to-frame deltas are dominated by tracker jitter (15–20 deg/s
        /// on a perfectly still head — far above any usable still threshold),
        /// which held the gate shut on 100% of frames; smoothing cancels the
        /// zero-mean jitter while preserving true motion velocity.
        smoothed: Option<[f32; 6]>,
        /// EMA-smoothed per-axis speed of the smoothed pose.
        speed: [f32; 6],
        /// Base drift rate when still, in fraction-per-second.
        drift_rate: f32,
        /// Per-axis multiplier on drift_rate [yaw, pitch, roll, x, y, z].
        drift_mult: [f32; 6],
        /// Velocity (deg/s or mm/s) below which the head is considered still.
        still_threshold: f32,
        /// Per-axis multiplier on still_threshold [yaw, pitch, roll, x, y, z].
        /// Z's smoothed wander velocity at rest (~4 mm/s) sits above the base
        /// threshold, so it gets 3× headroom.
        still_mult: [f32; 6],
        /// Per-axis recenter range [deg, deg, deg, mm, mm, mm]. Drift is full
        /// inside half this offset from baseline and fades to zero at it: a
        /// held off-center gaze (reading a switch at −14° pitch) is deliberate
        /// and must not be absorbed into the baseline — absorbing it pays back
        /// as a visible overshoot past center on the return. Noise wander
        /// lives near center and still gets recentered.
        recenter_range: [f32; 6],
        /// Per-axis unconditional drift floor, bypassing both the stillness
        /// gate and the range fade. Zero means a pose that is moving or
        /// beyond the recenter range is never absorbed — correct for
        /// rotations (held gazes are deliberate) but a trap for translation:
        /// noisy depth can park the pose far off-center while its own noise
        /// velocity keeps the stillness gate shut, so the baseline never
        /// catches up and the view stays stuck until a manual recenter. A
        /// small always-on leak (z: ~16 s time constant) is negligible over
        /// a seconds-long deliberate lean but recovers persistent offsets.
        recenter_floor: [f32; 6],
    }

    impl Default for CenterStage {
        fn default() -> Self { Self::new() }
    }

    impl CenterStage {
        /// Per-axis median of the collected baseline samples.
        ///
        /// A median and not a mean: the errors being rejected are excursions,
        /// which are large and one-sided, and a mean has a breakdown point of
        /// zero — one bad frame moves it.
        fn median_per_axis(samples: &[[f32; 6]]) -> [f32; 6] {
            let mut out = [0.0; 6];
            let mut col: Vec<f32> = Vec::with_capacity(samples.len());
            for (axis, slot) in out.iter_mut().enumerate() {
                col.clear();
                col.extend(samples.iter().map(|s| s[axis]));
                col.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let n = col.len();
                *slot = if n == 0 {
                    0.0
                } else if n % 2 == 1 {
                    col[n / 2]
                } else {
                    0.5 * (col[n / 2 - 1] + col[n / 2])
                };
            }
            out
        }

        /// Gate velocity smoothing time constant (seconds).
        const GATE_TAU: f32 = 0.3;

        /// How long after a recenter the baseline keeps refining, in seconds.
        ///
        /// **Why this is not simply the pose at the instant of the press.** The
        /// baseline used to be exactly that, and depth is the axis it is read
        /// from — the axis more than 50 mm from the truth on 45% of frames. So a
        /// press had roughly even odds of defining "centre" as a wrong place,
        /// and worse than even odds in practice, because users press recenter
        /// *because* the view looks wrong, which is when the cue is worst.
        ///
        /// Swept over **21 real presses across three sessions**, mean error
        /// baked into the baseline against window length:
        ///
        /// | window | mean | worst | >50 mm |
        /// |---|---:|---:|---:|
        /// | instant | 35.9 mm | 96.8 mm | 29% |
        /// | 1.0 s | 35.8 mm | 80.7 mm | 29% |
        /// | 2.0 s | 31.0 mm | 69.8 mm | 19% |
        /// | 2.5 s | 28.4 mm | 68.2 mm | **10%** |
        /// | 3.0 s | 27.4 mm | 66.3 mm | 10% |
        ///
        /// Note **1 s is no better than instant on the mean** — an earlier
        /// version of this constant was 1.0 s, justified on a five-press subset
        /// that did not survive the fuller data. The gain only appears past
        /// ~1.5 s, because the excursions are slow: a short median sits *inside*
        /// one. 2.5 s is chosen as the knee — 3.0 s buys 1 mm for another half
        /// second of settling.
        ///
        /// This is a real but modest fix, not a cure. It cannot help at all when
        /// an excursion outlasts the window, which is why the honest headline is
        /// ">50 mm errors cut from 29% of presses to 10%", not "recenter fixed".
        const BASELINE_WINDOW_S: f32 = 2.5;

        /// Hard cap on how long the window may stay open in wall-clock terms.
        ///
        /// Collection only advances while the head is still (see
        /// [`process`](PipelineStage::process)), so a user who presses recenter
        /// and keeps moving would otherwise hold the window open indefinitely
        /// and have a much later pose folded into their "centre".
        const BASELINE_MAX_AGE_S: f32 = 6.0;

        /// Cap on retained samples, so a fast camera cannot grow the buffer
        /// without bound if `dt` is reported implausibly small.
        const BASELINE_MAX_SAMPLES: usize = 400;

        pub fn new() -> Self {
            Self {
                baseline: None,
                baseline_samples: Vec::new(),
                baseline_elapsed_s: 0.0,
                baseline_age_s: 0.0,
                smoothed: None,
                speed: [0.0; 6],
                drift_rate: 0.2,
                //                yaw  pitch roll  x    y    z
                drift_mult: [1.0, 1.0, 1.0, 1.0, 1.0, 3.2],
                still_threshold: 3.0, // deg/s or mm/s — low enough to mean "not moving"
                //               yaw  pitch roll  x    y    z
                still_mult: [1.0, 1.0, 1.0, 1.0, 1.0, 3.0],
                // Rest wander is <1° / ~2mm / ~30mm (z); deliberate offsets
                // are >5° / >50mm — ranges sit between the two regimes. Yaw
                // and pitch run wider than that floor because a held gaze out
                // of the side window is a deliberate offset that drift must
                // not absorb; 4° was narrow enough to swallow one in seconds.
                //                   yaw   pitch roll  x     y     z
                recenter_range: [12.0, 12.0, 4.0, 10.0, 10.0, 60.0],
                //                   yaw  pitch roll  x     y     z
                recenter_floor: [0.0, 0.0, 0.0, 0.05, 0.05, 0.10],
            }
        }
    }

    impl PipelineStage for CenterStage {
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::CENTER
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
                "yaw.still_mult"   => { self.still_mult[0] = value.max(0.0); true }
                "pitch.still_mult" => { self.still_mult[1] = value.max(0.0); true }
                "roll.still_mult"  => { self.still_mult[2] = value.max(0.0); true }
                "x.still_mult" => { self.still_mult[3] = value.max(0.0); true }
                "y.still_mult" => { self.still_mult[4] = value.max(0.0); true }
                "z.still_mult" => { self.still_mult[5] = value.max(0.0); true }
                "yaw.recenter_range"   => { self.recenter_range[0] = value.max(0.0); true }
                "pitch.recenter_range" => { self.recenter_range[1] = value.max(0.0); true }
                "roll.recenter_range"  => { self.recenter_range[2] = value.max(0.0); true }
                "x.recenter_range" => { self.recenter_range[3] = value.max(0.0); true }
                "y.recenter_range" => { self.recenter_range[4] = value.max(0.0); true }
                "z.recenter_range" => { self.recenter_range[5] = value.max(0.0); true }
                "yaw.recenter_floor"   => { self.recenter_floor[0] = value.clamp(0.0, 1.0); true }
                "pitch.recenter_floor" => { self.recenter_floor[1] = value.clamp(0.0, 1.0); true }
                "roll.recenter_floor"  => { self.recenter_floor[2] = value.clamp(0.0, 1.0); true }
                "x.recenter_floor" => { self.recenter_floor[3] = value.clamp(0.0, 1.0); true }
                "y.recenter_floor" => { self.recenter_floor[4] = value.clamp(0.0, 1.0); true }
                "z.recenter_floor" => { self.recenter_floor[5] = value.clamp(0.0, 1.0); true }
                _ => false,
            }
        }

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            const DRIFT_MULT_NAMES: &[&str] = &[
                "yaw.drift_mult", "pitch.drift_mult", "roll.drift_mult",
                "x.drift_mult", "y.drift_mult", "z.drift_mult",
            ];
            const STILL_MULT_NAMES: &[&str] = &[
                "yaw.still_mult", "pitch.still_mult", "roll.still_mult",
                "x.still_mult", "y.still_mult", "z.still_mult",
            ];
            const RECENTER_RANGE_NAMES: &[&str] = &[
                "yaw.recenter_range", "pitch.recenter_range", "roll.recenter_range",
                "x.recenter_range", "y.recenter_range", "z.recenter_range",
            ];
            let mut out = Vec::with_capacity(
                2 + DRIFT_MULT_NAMES.len() + STILL_MULT_NAMES.len() + RECENTER_RANGE_NAMES.len(),
            );
            out.push(("drift_rate", self.drift_rate));
            out.push(("still_threshold", self.still_threshold));
            for (i, &n) in DRIFT_MULT_NAMES.iter().enumerate() {
                out.push((n, self.drift_mult[i]));
            }
            for (i, &n) in STILL_MULT_NAMES.iter().enumerate() {
                out.push((n, self.still_mult[i]));
            }
            for (i, &n) in RECENTER_RANGE_NAMES.iter().enumerate() {
                out.push((n, self.recenter_range[i]));
            }
            out
        }

        fn process(&mut self, pose: Pose, dt: f32) -> Pose {
            let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];

            // Refine the baseline over BASELINE_WINDOW_S rather than trusting the
            // single frame the user's press happened to land on. The first frame
            // still takes effect immediately — recentre must feel instant — and
            // each subsequent frame improves the estimate until the window closes.
            //
            // Only *still* frames count. A longer window is strictly better
            // against excursions, but only if it is not quietly averaging in the
            // user looking away a moment after they pressed the button. Reusing
            // the stage's own stillness velocity from the previous frame keeps
            // one definition of "still" in the stage rather than two.
            self.baseline_age_s += dt.max(0.0);
            let mut v_norm = 0.0_f32;
            for i in 0..6 {
                let thr = (self.still_threshold * self.still_mult[i]).max(0.01);
                v_norm = v_norm.max(self.speed[i] / thr);
            }
            if self.baseline_elapsed_s < Self::BASELINE_WINDOW_S
                && self.baseline_age_s < Self::BASELINE_MAX_AGE_S
                && self.baseline_samples.len() < Self::BASELINE_MAX_SAMPLES
                && (v_norm <= 1.0 || self.baseline_samples.is_empty())
            {
                self.baseline_samples.push(vals);
                self.baseline_elapsed_s += dt.max(0.0);
                let m = Self::median_per_axis(&self.baseline_samples);
                self.baseline = Some(Pose {
                    yaw: m[0], pitch: m[1], roll: m[2],
                    x: m[3], y: m[4], z: m[5],
                    timestamp_us: pose.timestamp_us,
                });
            }
            let b = self.baseline.get_or_insert(pose);

            // Velocity for the stillness gate: EMA the pose, differentiate the
            // smoothed signal, EMA the speed, normalize per axis by its still
            // threshold, and take the max across axes.
            let alpha_g = dt / (dt + Self::GATE_TAU);
            let inv_dt = if dt > 0.0001 { 1.0 / dt } else { 0.0 };
            let sm = self.smoothed.get_or_insert(vals);
            let mut v_norm = 0.0_f32;
            for i in 0..6 {
                let prev = sm[i];
                sm[i] += alpha_g * (vals[i] - prev);
                let inst = (sm[i] - prev).abs() * inv_dt;
                self.speed[i] += alpha_g * (inst - self.speed[i]);
                let thr = (self.still_threshold * self.still_mult[i]).max(0.01);
                v_norm = v_norm.max(self.speed[i] / thr);
            }

            // Gate: only correct when still. Smooth the gate so it doesn't
            // snap on/off — use a soft ramp from 1.0 (fully still) to 0.0
            // (moving at the still threshold).
            let gate = ((1.0 - v_norm) * 2.0).clamp(0.0, 1.0);

            let b_vals = [b.yaw, b.pitch, b.roll, b.x, b.y, b.z];
            let mut new_b = [0.0f32; 6];
            for i in 0..6 {
                // Fade drift to zero as the pose moves away from baseline —
                // full inside half the recenter range, none beyond it. The
                // unconditional floor then guarantees a minimum leak.
                let off = (vals[i] - b_vals[i]).abs();
                let range = self.recenter_range[i].max(0.01);
                let w = ((1.0 - off / range) * 2.0).clamp(0.0, 1.0);
                let g = (gate * w).max(self.recenter_floor[i].min(1.0));
                let alpha = (self.drift_rate * dt * self.drift_mult[i] * g).min(1.0);
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
            self.smoothed = None;
            self.speed = [0.0; 6];
            self.baseline_samples.clear();
            self.baseline_elapsed_s = 0.0;
            self.baseline_age_s = 0.0;
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
        /// Standoff scaling — a deadzone exists to swallow noise, and the
        /// translation noise it is sized against grows with distance.
        dist: [f32; 6],
    }

    impl Default for DeadzoneStage {
        fn default() -> Self { Self::new() }
    }

    impl DeadzoneStage {
        pub fn new() -> Self {
            Self {
                //       yaw   pitch  roll   x     y     z
                dz: [0.8,  0.8,   0.5,  2.0,  2.1, 10.0],
                dist: crate::pipeline::DistanceScale::NONE.per_axis(),
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
        fn name(&self) -> &'static str {
            crate::pipeline::stage_names::DEADZONE
        }

        fn set_distance_scale(&mut self, scale: crate::pipeline::DistanceScale) {
            self.dist = scale.per_axis();
        }

        fn process(&mut self, pose: Pose, _dt: f32) -> Pose {
            Pose {
                yaw:   Self::apply(pose.yaw,   self.dz[0] * self.dist[0]),
                pitch: Self::apply(pose.pitch, self.dz[1] * self.dist[1]),
                roll:  Self::apply(pose.roll,  self.dz[2] * self.dist[2]),
                x:     Self::apply(pose.x,     self.dz[3] * self.dist[3]),
                y:     Self::apply(pose.y,     self.dz[4] * self.dist[4]),
                z:     Self::apply(pose.z,     self.dz[5] * self.dist[5]),
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

        fn get_params(&self) -> Vec<(&'static str, f32)> {
            use crate::pose::AXIS_NAMES;
            AXIS_NAMES.iter().enumerate().map(|(i, &n)| {
                (n, self.dz[i])
            }).collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ProfileConfig;
    use crate::pose::Pose;

    /// A short synthetic motion sequence (a slow yaw sweep with a little Z).
    fn motion_sequence() -> Vec<Pose> {
        (0..120)
            .map(|i| {
                let t = i as f32;
                Pose {
                    yaw: (t * 0.1).sin() * 15.0,
                    pitch: (t * 0.07).cos() * 5.0,
                    roll: 0.0,
                    x: 0.0,
                    y: 0.0,
                    z: 50.0 + (t * 0.05).sin() * 20.0,
                    timestamp_us: (i as u64) * 33_333,
                }
            })
            .collect()
    }

    fn run(seq: &[Pose], mut pipeline: Pipeline) -> Vec<Pose> {
        seq.iter().map(|p| pipeline.process(*p, 1.0 / 30.0)).collect()
    }

    fn get_param(params: &[(&str, &str, f32)], stage: &str, param: &str) -> f32 {
        params
            .iter()
            .find(|(s, p, _)| *s == stage && *p == param)
            .map(|(_, _, v)| *v)
            .unwrap_or_else(|| panic!("missing {stage}:{param}"))
    }

    /// **The applier-completeness guard.**
    ///
    /// Every profile field that `apply_config` is supposed to push gets a
    /// *unique* sentinel value; after applying, each sentinel must be findable
    /// somewhere in `get_all_params()`. A field the applier silently forgets
    /// leaves its sentinel nowhere in the dump and fails here, by name.
    ///
    /// This exists because `cross_axis.yaw_to_z` was dropped for months: the
    /// GUI pushed it, the applier did not, so engine startup, engine
    /// profile-reload and offline replay all silently disagreed with what the
    /// GUI displayed — defeating the "replay == ships" guarantee the replay
    /// tool is built on. A per-field spot check would never have caught it;
    /// only exhaustiveness does.
    ///
    /// Sentinels are all >= 2.0 so that no stage's lower-bound clamp
    /// (`value.max(..)`, largest floor 1.0 on slew rate) can rewrite one and
    /// mask a genuine drop.
    #[test]
    fn apply_config_pushes_every_profile_field() {
        let mut cfg = ProfileConfig::default();

        // Hand out distinct sentinels, recording what each one was for so a
        // failure names the field rather than just a number.
        let mut next = 2.0_f32;
        let mut expect: Vec<(String, f32)> = Vec::new();
        let sentinel = |label: &str, expect: &mut Vec<(String, f32)>, next: &mut f32| -> f32 {
            let v = *next;
            *next += 0.25;
            expect.push((label.to_string(), v));
            v
        };

        for (i, axis) in AXES_TEST.iter().enumerate() {
            cfg.curve[i].sensitivity =
                sentinel(&format!("curve[{axis}].sensitivity"), &mut expect, &mut next);
            cfg.curve[i].exponent =
                sentinel(&format!("curve[{axis}].exponent"), &mut expect, &mut next);
            cfg.accela[i].deadzone =
                sentinel(&format!("accela[{axis}].deadzone"), &mut expect, &mut next);
            cfg.accela[i].smoothing =
                sentinel(&format!("accela[{axis}].smoothing"), &mut expect, &mut next);
            cfg.accela[i].exponent =
                sentinel(&format!("accela[{axis}].exponent"), &mut expect, &mut next);
            cfg.deadzone[i] = sentinel(&format!("deadzone[{axis}]"), &mut expect, &mut next);
            cfg.slew_limit[i] = sentinel(&format!("slew_limit[{axis}]"), &mut expect, &mut next);
        }

        cfg.cross_axis.yaw_to_pitch = sentinel("cross.yaw_to_pitch", &mut expect, &mut next);
        cfg.cross_axis.yaw_to_x = sentinel("cross.yaw_to_x", &mut expect, &mut next);
        cfg.cross_axis.yaw_to_y = sentinel("cross.yaw_to_y", &mut expect, &mut next);
        cfg.cross_axis.yaw_to_roll = sentinel("cross.yaw_to_roll", &mut expect, &mut next);
        cfg.cross_axis.yaw_to_z = sentinel("cross.yaw_to_z", &mut expect, &mut next);
        cfg.cross_axis.pitch_to_y = sentinel("cross.pitch_to_y", &mut expect, &mut next);
        cfg.cross_axis.pitch_to_yaw = sentinel("cross.pitch_to_yaw", &mut expect, &mut next);
        cfg.cross_axis.pitch_to_z = sentinel("cross.pitch_to_z", &mut expect, &mut next);
        cfg.cross_axis.z_to_y = sentinel("cross.z_to_y", &mut expect, &mut next);
        cfg.cross_axis.z_to_pitch = sentinel("cross.z_to_pitch", &mut expect, &mut next);
        cfg.cross_axis.roll_to_z = sentinel("cross.roll_to_z", &mut expect, &mut next);
        cfg.cross_axis.roll_to_x = sentinel("cross.roll_to_x", &mut expect, &mut next);


        cfg.center.drift_rate = sentinel("center.drift_rate", &mut expect, &mut next);
        cfg.center.z_drift_mult = sentinel("center.z_drift_mult", &mut expect, &mut next);
        let mut ranges = [0.0_f32; 6];
        for (i, r) in ranges.iter_mut().enumerate() {
            *r = sentinel(
                &format!("center.recenter_range[{}]", AXES_TEST[i]),
                &mut expect,
                &mut next,
            );
        }
        cfg.center.recenter_range = Some(ranges);

        let mut pipeline = build_default_pipeline();
        apply_config(&mut pipeline, &cfg);
        let params = pipeline.get_all_params();

        for (label, want) in &expect {
            let found = params.iter().any(|(_, _, v)| (v - want).abs() < 1e-6);
            assert!(
                found,
                "profile field `{label}` (sentinel {want}) never reached the pipeline — \
                 apply_config is dropping it"
            );
        }
    }

    const AXES_TEST: [&str; 6] = ["yaw", "pitch", "roll", "x", "y", "z"];

    /// The offline replay relies on `apply_config` actually wiring a profile's
    /// values into the live stages.
    #[test]
    fn apply_config_wires_params_through() {
        let mut cfg = ProfileConfig::default();
        cfg.deadzone[0] = 4.25; // yaw deadzone
        cfg.accela[0].deadzone = 0.33;
        cfg.accela[5].smoothing = 4.5;
        cfg.accela[2].exponent = 1.11;
        cfg.cross_axis.z_to_y = -0.123;

        let mut pipeline = build_default_pipeline();
        apply_config(&mut pipeline, &cfg);
        let params = pipeline.get_all_params();

        assert!((get_param(&params, "deadzone", "yaw") - 4.25).abs() < 1e-6);
        assert!((get_param(&params, "accela", "yaw.deadzone") - 0.33).abs() < 1e-6);
        assert!((get_param(&params, "accela", "z.smoothing") - 4.5).abs() < 1e-6);
        assert!((get_param(&params, "accela", "roll.exponent") - 1.11).abs() < 1e-6);
        assert!((get_param(&params, "cross-axis", "z_to_y") - (-0.123)).abs() < 1e-6);
        // The deleted legacy stages must not reappear in the chain.
        assert!(!params.iter().any(|(s, _, _)| *s == "one-euro-filter"));
        assert!(!params.iter().any(|(s, _, _)| *s == "prediction"));
    }

    /// The profile's accela defaults must equal the stage constructor exactly:
    /// a pre-accela profile (section absent → serde default) applied over a
    /// fresh Accela chain must not change any accela param. This is the
    /// executable guarantee that the post-flip default feel equals the
    /// flown-and-approved Accela constructor defaults.
    /// The engine has two ways to start — with a saved profile and without —
    /// and they must mean the same thing.
    ///
    /// They did not, for months: the stage constructors and
    /// `ProfileConfig::default()` drifted apart (cross-axis `z_to_y` −0.50 vs
    /// 0.0, `yaw_to_x`/`yaw_to_y`/`yaw_to_roll` set vs cleared, z sensitivity
    /// 1.5 vs 1.0), so a fresh install tracked differently from the same
    /// install one profile-save later, and the tuning that was actually flown
    /// reached nobody who had not imported a `.toml`. Both are now the
    /// shipped tuning; this keeps them that way.
    #[test]
    fn default_profile_is_the_no_profile_baseline() {
        let fresh = build_default_pipeline().get_all_params();
        let mut applied_pipeline = build_default_pipeline();
        apply_config(&mut applied_pipeline, &ProfileConfig::default());
        let applied = applied_pipeline.get_all_params();

        assert_eq!(fresh.len(), applied.len(), "stage/param set changed");
        let mismatched: Vec<_> = fresh
            .iter()
            .zip(applied.iter())
            .filter(|((s1, p1, v1), (s2, p2, v2))| s1 != s2 || p1 != p2 || v1 != v2)
            .map(|((s, p, v1), (_, _, v2))| format!("{s}.{p}: constructor {v1} vs profile {v2}"))
            .collect();
        assert!(
            mismatched.is_empty(),
            "ProfileConfig::default() disagrees with the stage constructors:\n  {}",
            mismatched.join("\n  ")
        );
    }

    /// P08: the Tobii profile is rotation-only. Even with translation and
    /// pitch on the input (cross-axis terms would turn pitch into Y/Z on the
    /// webcam profile), every X/Y/Z output is exactly zero, and rotation
    /// still gets through.
    #[test]
    fn tobii_default_holds_translation_at_zero() {
        let mut p = build_default_pipeline();
        apply_config(&mut p, &ProfileConfig::tobii_default());
        let seq: Vec<Pose> = motion_sequence()
            .into_iter()
            .map(|q| Pose { x: 30.0, y: -20.0, pitch: q.pitch * 4.0, ..q })
            .collect();
        let out = run(&seq, p);
        assert!(out.iter().all(|q| q.x == 0.0 && q.y == 0.0 && q.z == 0.0));
        assert!(out.iter().any(|q| q.yaw.abs() > 1.0), "rotation must pass");
        assert!(out.iter().any(|q| q.pitch.abs() > 1.0), "rotation must pass");
    }

    /// P08: the webcam's cross-axis couplings are all off for the Tobii, and
    /// the values that differ from the webcam default are the documented ones.
    #[test]
    fn tobii_default_turns_off_every_webcam_coupling() {
        let t = ProfileConfig::tobii_default();
        assert_eq!(t.name, "Tobii Default");
        let params = {
            let mut p = build_default_pipeline();
            apply_config(&mut p, &t);
            p.get_all_params()
        };
        // Every coupling coefficient the profile carries is 0. (The stage's
        // `pivot_mm` is neck geometry, not a profile field; whatever it adds
        // lands on X/Y/Z, which the 0 sensitivity zeroes — previous test.)
        let couplings: Vec<_> = profile_params(&t)
            .into_iter()
            .filter(|(stage, _, _)| *stage == crate::pipeline::stage_names::CROSS_AXIS)
            .collect();
        assert_eq!(couplings.len(), 12);
        for (_, param, value) in couplings {
            assert_eq!(value, 0.0, "cross-axis {param} must be 0 for the Tobii");
        }
        for axis in ["x", "y", "z"] {
            assert_eq!(get_param(&params, "response-curve", &format!("{axis}.sensitivity")), 0.0);
        }
        assert_eq!(get_param(&params, "center", "drift_rate"), 0.0);
        let d = ProfileConfig::default();
        assert_eq!(t.accela.map(|a| a.smoothing), d.accela.map(|a| a.smoothing));
        assert_eq!(t.slew_limit, d.slew_limit);
    }

    /// Two freshly-built pipelines must produce identical output for identical
    /// input — the offline replay's determinism guarantee.
    #[test]
    fn replay_is_deterministic() {
        let seq = motion_sequence();
        let a = run(&seq, build_default_pipeline());
        let b = run(&seq, build_default_pipeline());
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(b.iter()) {
            assert_eq!(x.yaw.to_bits(), y.yaw.to_bits());
            assert_eq!(x.z.to_bits(), y.z.to_bits());
        }
    }
}

/// Follows how far the user is sitting from the camera, so the pipeline's
/// translation thresholds can track the noise instead of the millimetre.
///
/// Three things make this harder than an average:
///
/// 1. **The signal it reads is the noisy one.** Depth is the worst axis, so a
///    fast follower would let a single 200 mm excursion retune the pipeline.
///    Hence a slow EMA — seconds, not frames.
/// 2. **Leaning is not moving the chair.** A pilot leaning forward to look past
///    a post is a transient; sitting closer is not. The time constant is chosen
///    long enough that normal in-seat movement averages out.
/// 3. **Retuning has a cost.** Every push changes filter behaviour, so it fires
///    only when the standoff has moved enough to matter, with hysteresis to stop
///    it dithering at the boundary.
pub struct StandoffTracker {
    /// EMA of |raw z| in mm, or `None` until the first usable sample.
    ema_mm: Option<f32>,
    /// Standoff the currently-applied scale was computed from.
    applied_mm: Option<f32>,
}

impl Default for StandoffTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl StandoffTracker {
    /// EMA time constant. Long enough that a lean averages out, short enough
    /// that genuinely moving the chair is picked up within a few seconds.
    const TAU_S: f32 = 4.0;
    /// Relative change in standoff needed before the scale is re-pushed. 8%
    /// of standoff is ~1.17× on the depth threshold — below that the retune is
    /// not worth the behaviour change.
    const HYSTERESIS: f32 = 0.08;

    pub fn new() -> Self {
        Self { ema_mm: None, applied_mm: None }
    }


    pub fn current_mm(&self) -> f32 {
        self.ema_mm.unwrap_or(REFERENCE_DISTANCE_MM)
    }

    /// Feed one raw (pre-pipeline) depth. Returns a new scale only when the
    /// standoff has moved past the hysteresis band.
    pub fn observe(&mut self, raw_z_mm: f32, dt: f32) -> Option<DistanceScale> {
        let d = raw_z_mm.abs();
        // Reject implausible depths outright rather than letting them drag the
        // EMA: a collapsed landmark pair reports metres, and that is precisely
        // the failure this must not react to.
        if !d.is_finite() || !(MIN_TRUSTED_DISTANCE_MM..=MAX_TRUSTED_DISTANCE_MM).contains(&d) {
            return None;
        }
        let ema = match self.ema_mm {
            None => d,
            Some(prev) => {
                let alpha = (dt / Self::TAU_S).clamp(0.0, 1.0);
                prev + alpha * (d - prev)
            }
        };
        self.ema_mm = Some(ema);

        let changed = match self.applied_mm {
            None => true,
            Some(applied) => (ema - applied).abs() / applied.max(1.0) > Self::HYSTERESIS,
        };
        if !changed {
            return None;
        }
        self.applied_mm = Some(ema);
        Some(DistanceScale::from_standoff_mm(ema))
    }
}

#[cfg(test)]
mod distance_scale_tests {
    use super::*;
    use crate::pose::Pose;

    fn pose(x: f32, y: f32, z: f32) -> Pose {
        Pose { yaw: 0.0, pitch: 0.0, roll: 0.0, x, y, z, timestamp_us: 0 }
    }

    /// The exponents are the whole point: depth noise grows with the SQUARE of
    /// standoff and lateral noise linearly. Measured on real landmark
    /// constellations replayed across 450–1000 mm as Z^2.00 / X^1.08 / Y^1.00.
    #[test]
    fn depth_scales_quadratically_and_lateral_linearly() {
        // 1.25x the reference is ~981 mm — inside the trusted band, so this
        // exercises the law rather than the clamp.
        let d = 1.25 * REFERENCE_DISTANCE_MM;
        assert!(d < MAX_TRUSTED_DISTANCE_MM, "{d} mm would exercise the clamp");
        let s = DistanceScale::from_standoff_mm(d);
        assert!((s.lateral - 1.25).abs() < 1e-5, "lateral {}", s.lateral);
        assert!((s.depth - 1.5625).abs() < 1e-5, "depth {}", s.depth);

        // The relationship that matters: depth is lateral squared, always.
        for d in [400.0, 500.0, 610.0, 750.0, 900.0, 1050.0] {
            let s = DistanceScale::from_standoff_mm(d);
            assert!(
                (s.depth - s.lateral * s.lateral).abs() < 1e-4,
                "at {d} mm: depth {} != lateral^2 {}",
                s.depth,
                s.lateral * s.lateral
            );
        }

        // 300 mm is below the trusted floor, so it clamps rather than
        // extrapolating into a region no data supports.
        let half = DistanceScale::from_standoff_mm(300.0);
        let clamped = MIN_TRUSTED_DISTANCE_MM / REFERENCE_DISTANCE_MM;
        assert!((half.lateral - clamped).abs() < 1e-5);
    }

    #[test]
    fn at_the_reference_distance_nothing_changes() {
        let s = DistanceScale::from_standoff_mm(REFERENCE_DISTANCE_MM);
        assert_eq!(s.per_axis(), DistanceScale::NONE.per_axis());
        // Sign of the raw depth must not matter: camera-frame Z is negative.
        assert_eq!(
            DistanceScale::from_standoff_mm(-800.0),
            DistanceScale::from_standoff_mm(800.0)
        );
    }

    /// Rotation is an angle; an angle is the same at any distance. A tuning
    /// that quietly changed yaw when the user moved their chair would be a bug.
    #[test]
    fn rotation_axes_are_never_scaled() {
        for d in [350.0, 610.0, 900.0, 5000.0] {
            let a = DistanceScale::from_standoff_mm(d).per_axis();
            assert_eq!(&a[..3], &[1.0, 1.0, 1.0], "at {d} mm");
        }
    }

    /// A deadzone exists to swallow noise, so far from the camera the Z deadzone
    /// must be wider — otherwise the same tuning that is calm at the reference
    /// distance passes noise straight through at the far end of the trusted band.
    #[test]
    fn deadzone_widens_with_standoff() {
        let mut dz = stages::DeadzoneStage::new();
        let near = dz.process(pose(0.0, 0.0, 18.0), 0.02).z;

        dz.set_distance_scale(DistanceScale::from_standoff_mm(MAX_TRUSTED_DISTANCE_MM));
        let far = dz.process(pose(0.0, 0.0, 18.0), 0.02).z;

        assert!(
            far.abs() < near.abs(),
            "18 mm of Z should survive at the reference distance ({near}) and be \
             swallowed at the far end of the trusted band ({far})"
        );
    }

    /// The slew cap must NOT move with standoff.
    ///
    /// It is sized to sit just above genuine head movement, and genuine movement
    /// is the same mm/s however far away the user sits. Widening it with
    /// distance would admit larger noise spikes — the opposite of the intent.
    #[test]
    fn slew_cap_is_independent_of_standoff() {
        let step = |scale: DistanceScale| {
            let mut s = stages::SlewLimitStage::new();
            s.set_distance_scale(scale);
            s.process(pose(0.0, 0.0, 0.0), 0.02);
            s.process(pose(0.0, 0.0, 500.0), 0.02).z
        };
        let near = step(DistanceScale::NONE);
        for d in [400.0, 900.0, 1100.0] {
            let far = step(DistanceScale::from_standoff_mm(d));
            assert!((far - near).abs() < 1e-3, "at {d} mm: {near} -> {far}");
        }
    }

    /// Accela's `smoothing` must not move with standoff either — same argument
    /// as the slew cap. Measured cost of getting this wrong: 50% of genuine Z
    /// travel lost for a 43% noise cut, very nearly a wash.
    #[test]
    fn accela_smoothing_is_independent_of_standoff() {
        let settle = |scale: DistanceScale| {
            let mut a = stages::AccelaStage::new();
            a.set_distance_scale(scale);
            let mut out = 0.0;
            for _ in 0..40 {
                out = a.process(pose(0.0, 0.0, 200.0), 0.02).z;
            }
            out
        };
        let near = settle(DistanceScale::NONE);
        let far = settle(DistanceScale::from_standoff_mm(MAX_TRUSTED_DISTANCE_MM));
        // Only the deadzone differs, so convergence on a large target is
        // near-identical — a few mm of deadzone, not a slower filter.
        assert!(
            (far - near).abs() < 5.0,
            "a large genuine move must converge at the same speed: {near} -> {far}"
        );
    }

    /// Guard against a single bad depth reading retuning the whole pipeline:
    /// a collapsed landmark pair reports metres, and that is exactly the failure
    /// this must not react to.
    #[test]
    fn implausible_standoffs_are_clamped_not_obeyed() {
        let wild = DistanceScale::from_standoff_mm(6300.0);
        let ceiling = DistanceScale::from_standoff_mm(MAX_TRUSTED_DISTANCE_MM);
        assert_eq!(wild, ceiling);
        assert_eq!(
            DistanceScale::from_standoff_mm(10.0),
            DistanceScale::from_standoff_mm(MIN_TRUSTED_DISTANCE_MM)
        );
    }
}

#[cfg(test)]
mod recenter_baseline_tests {
    use super::stages::CenterStage;
    use super::*;
    use crate::pose::Pose;

    fn p(z: f32) -> Pose {
        Pose { yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0.0, y: 0.0, z, timestamp_us: 0 }
    }

    /// The whole point: a recenter pressed while the depth cue is mid-excursion
    /// must not adopt the excursion as "centre".
    ///
    /// Scenario is taken from a real press — the excursion is transient, the
    /// user hits the button at its worst moment, and the cue recovers.
    #[test]
    fn an_excursion_at_the_press_is_not_baked_into_the_baseline() {
        let mut c = CenterStage::new();
        // 0.2 s of a −90 mm excursion, then recovery to the true −610 mm.
        //
        // The recovery is held for 4 s, not a fraction of one, because the
        // stillness gate closes on the excursion *ending* — a cue snapping back
        // to truth is indistinguishable from a head moving — and only reopens
        // once the velocity EMA decays. The window is 2.5 s of still samples
        // under a 6 s age cap precisely so there is room for that.
        for _ in 0..12 {
            c.process(p(-700.0), 1.0 / 60.0);
        }
        for _ in 0..240 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        // Once settled, true depth must report as centre, not as +90 mm.
        let out = c.process(p(-610.0), 1.0 / 60.0);
        assert!(
            out.z.abs() < 25.0,
            "baseline absorbed the excursion: true depth reads {} mm off centre",
            out.z
        );
    }

    /// Recentre must still feel instant. The first frame after a reset defines a
    /// usable centre; the window only refines it.
    #[test]
    fn the_first_frame_after_reset_takes_effect_immediately() {
        let mut c = CenterStage::new();
        let out = c.process(p(-610.0), 1.0 / 60.0);
        assert!(out.z.abs() < 1e-3, "first frame should read as centre, got {}", out.z);
    }

    /// After the window closes the baseline is fixed — a slow genuine lean must
    /// register as movement, not be quietly re-absorbed as a new centre.
    #[test]
    fn the_window_closes_and_stops_absorbing() {
        let mut c = CenterStage::new();
        for _ in 0..400 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        let out = c.process(p(-560.0), 1.0 / 60.0);
        assert!(out.z > 25.0, "a 50 mm lean after the window must show, got {}", out.z);
    }

    /// The window must not swallow the user looking away right after they press.
    ///
    /// A longer window is only better against excursions if it stays a window on
    /// a *stationary* head. Without the stillness gate a 2.5 s window would fold
    /// a deliberate move into the definition of centre — turning a fix into a
    /// worse version of the bug it replaced.
    #[test]
    fn moving_right_after_the_press_does_not_redefine_centre() {
        let mut c = CenterStage::new();
        // Half a second parked, then a sustained deliberate lean forward.
        for _ in 0..30 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        let mut z = -610.0;
        let mut last = 0.0;
        for _ in 0..120 {
            z += 1.5; // ~90 mm/s forward — clearly deliberate movement
            last = c.process(p(z), 1.0 / 60.0).z;
        }
        // The lean must read as a large offset, not be absorbed toward zero.
        assert!(
            last > 90.0,
            "a 180 mm deliberate lean was absorbed into the baseline: reads {last} mm"
        );
    }

    /// A user who never holds still must not hold the window open forever.
    #[test]
    fn the_window_ages_out_even_without_stillness() {
        let mut c = CenterStage::new();
        let mut z = -610.0;
        // Constant motion for well past BASELINE_MAX_AGE_S.
        for _ in 0..600 {
            z += if z > -500.0 { -2.0 } else { 2.0 };
            c.process(p(z), 1.0 / 60.0);
        }
        // Window closed: a fresh offset now registers rather than being absorbed.
        for _ in 0..30 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        let out = c.process(p(-560.0), 1.0 / 60.0);
        assert!(out.z.abs() > 1.0, "baseline still absorbing after age-out: {}", out.z);
    }

    /// A reset re-arms the window, or the second recenter of a session would
    /// silently do nothing.
    #[test]
    fn reset_rearms_the_window() {
        let mut c = CenterStage::new();
        for _ in 0..120 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        c.reset();
        for _ in 0..12 {
            c.process(p(-700.0), 1.0 / 60.0);
        }
        for _ in 0..240 {
            c.process(p(-610.0), 1.0 / 60.0);
        }
        let out = c.process(p(-610.0), 1.0 / 60.0);
        assert!(out.z.abs() < 25.0, "second recenter did not reject the excursion: {}", out.z);
    }
}
