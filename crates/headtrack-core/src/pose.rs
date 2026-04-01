use serde::{Deserialize, Serialize};

/// 6-DOF head pose — the universal currency of the entire system.
///
/// ## Coordinate convention (internal / canonical)
///
/// All components use this convention throughout the pipeline.
/// Output adapters are responsible for transforming to their target's space.
///
/// | Field   | Positive direction   | Unit    |
/// |---------|----------------------|---------|
/// | `yaw`   | right                | degrees |
/// | `pitch` | up                   | degrees |
/// | `roll`  | clockwise tilt       | degrees |
/// | `x`     | right                | mm      |
/// | `y`     | up                   | mm      |
/// | `z`     | forward (to screen)  | mm      |
///
/// `timestamp_us` is a monotonic microsecond counter from the input source.
/// It is used by the prediction stage; do not normalise it across sources.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub timestamp_us: u64,
}

impl Pose {
    pub fn zero(timestamp_us: u64) -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp_us,
        }
    }

    /// Apply `f` to the three rotation axes, leaving translations unchanged.
    pub fn map_rotations(self, mut f: impl FnMut(f32) -> f32) -> Self {
        Self {
            yaw: f(self.yaw),
            pitch: f(self.pitch),
            roll: f(self.roll),
            ..self
        }
    }

    /// Apply `f` to the three translation axes, leaving rotations unchanged.
    pub fn map_translations(self, mut f: impl FnMut(f32) -> f32) -> Self {
        Self {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
            ..self
        }
    }
}

/// Raw pose emitted by an [`InputSource`](crate::InputSource) before any pipeline processing.
///
/// Identical in structure to [`Pose`]; the type alias makes call-sites
/// self-documenting about which stage of processing the value is at.
pub type RawPose = Pose;
