pub mod kalman;
pub mod one_euro;

use crate::pose::Pose;
use one_euro::OneEuroFilter;

/// Per-axis filter parameters.
#[derive(Clone, Copy)]
pub struct AxisParams {
    pub min_cutoff: f32,
    pub beta: f32,
}

impl AxisParams {
    pub fn new(min_cutoff: f32, beta: f32) -> Self {
        Self { min_cutoff, beta }
    }
}

/// Axis indices — matches the order used throughout the codebase.
pub const AXIS_NAMES: [&str; 6] = ["yaw", "pitch", "roll", "x", "y", "z"];

/// Six independent One Euro Filters — one per pose axis, each with its own
/// `min_cutoff` and `beta` parameters for independent tuning.
pub struct PoseFilter {
    filters: [OneEuroFilter; 6],
    params: [AxisParams; 6],
}

impl PoseFilter {
    /// Create with uniform parameters across all axes.
    pub fn new(min_cutoff: f32, beta: f32) -> Self {
        let p = AxisParams::new(min_cutoff, beta);
        Self {
            filters: std::array::from_fn(|_| OneEuroFilter::new(min_cutoff, beta)),
            params: [p; 6],
        }
    }

    /// Get current per-axis parameters.
    pub fn axis_params(&self) -> &[AxisParams; 6] {
        &self.params
    }

    /// Update parameters for a single axis (by index 0..6).
    /// Resets that axis's filter so it reinitialises cleanly.
    pub fn set_axis_param(&mut self, axis: usize, param: &str, value: f32) -> bool {
        if axis >= 6 {
            return false;
        }
        match param {
            "min_cutoff" => self.params[axis].min_cutoff = value.max(0.001),
            "beta" => self.params[axis].beta = value.max(0.0),
            _ => return false,
        }
        let p = &self.params[axis];
        self.filters[axis] = OneEuroFilter::new(p.min_cutoff, p.beta);
        true
    }

    pub fn get_axis_param(&self, axis: usize, param: &str) -> Option<f32> {
        if axis >= 6 { return None; }
        match param {
            "min_cutoff" => Some(self.params[axis].min_cutoff),
            "beta" => Some(self.params[axis].beta),
            _ => None,
        }
    }

    pub fn filter(&mut self, pose: Pose, dt: f32) -> Pose {
        let vals = [pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z];
        let f = &mut self.filters;
        let filtered = [
            f[0].filter(vals[0], dt),
            f[1].filter(vals[1], dt),
            f[2].filter(vals[2], dt),
            f[3].filter(vals[3], dt),
            f[4].filter(vals[4], dt),
            f[5].filter(vals[5], dt),
        ];
        Pose {
            yaw: filtered[0],
            pitch: filtered[1],
            roll: filtered[2],
            x: filtered[3],
            y: filtered[4],
            z: filtered[5],
            timestamp_us: pose.timestamp_us,
        }
    }

    pub fn reset(&mut self) {
        for f in &mut self.filters {
            f.reset();
        }
    }
}
