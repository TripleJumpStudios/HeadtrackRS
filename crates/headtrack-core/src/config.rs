//! Profile configuration — serializable snapshot of all pipeline parameters.
//!
//! A [`ProfileConfig`] captures every tunable parameter in the pipeline so it
//! can be saved to / loaded from a TOML file.  The GUI writes these, and the
//! daemon can optionally load one at startup.

use serde::{Deserialize, Serialize};

/// Resolve XDG_CONFIG_HOME or fall back to `$HOME/.config`.
/// Prints a warning to stderr if both `$HOME` and `$XDG_CONFIG_HOME` are unset.
fn xdg_config_base() -> String {
    std::env::var("XDG_CONFIG_HOME").unwrap_or_else(|_| {
        let home = std::env::var("HOME").unwrap_or_else(|_| {
            eprintln!("warning: $HOME is not set — config will be written to /tmp");
            "/tmp".into()
        });
        format!("{home}/.config")
    })
}

/// Per-axis filter parameters (One Euro).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AxisFilter {
    pub min_cutoff: f32,
    pub beta: f32,
}

/// Per-axis response curve parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AxisCurve {
    pub sensitivity: f32,
    pub exponent: f32,
}

/// Cross-axis compensation coefficients.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CrossAxisComp {
    pub yaw_to_pitch: f32,
    pub yaw_to_x: f32,
    pub yaw_to_y: f32,
    pub yaw_to_roll: f32,
    pub pitch_to_y: f32,
    pub z_to_y: f32,
    pub z_to_pitch: f32,
}

/// Kalman prediction parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Prediction {
    pub predict_ms: f32,
    pub rot_process_noise: f32,
    pub rot_measurement_noise: f32,
    pub pos_process_noise: f32,
    pub pos_measurement_noise: f32,
}

/// Velocity-gated centering parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CenterConfig {
    pub drift_rate: f32,
    pub z_drift_mult: f32,
}

/// A named FOV preset the user can save and recall.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FovPreset {
    pub name: String,
    pub fov:  f32,
}

/// Camera-specific settings — stored separately from pipeline profiles.
///
/// Saved to `$XDG_CONFIG_HOME/headtrack-rs/camera.toml`.  Profile configs
/// capture pipeline tuning; this captures physical camera properties.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraConfig {
    /// Diagonal field of view in degrees for the active camera.
    ///
    /// `78.0` is the default — matches the Logitech C920 (most common webcam).
    /// Delan Cam 1 with face-tracking lens: `60.0`.
    pub fov_diag_deg: f32,
    /// Requested camera capture resolution (width, height).
    ///
    /// The engine passes this to V4L2 / nokhwa on every camera open.
    /// Default 640×480 is best for most use-cases; 1280×720 helps when far
    /// from the camera but halves tracking FPS on CPU-only setups.
    #[serde(default = "CameraConfig::default_resolution")]
    pub camera_resolution: (u32, u32),
    /// User-saved named FOV presets.
    #[serde(default)]
    pub fov_presets: Vec<FovPreset>,
    /// When true, closing the window hides to the system tray instead of quitting.
    #[serde(default = "CameraConfig::default_minimize_to_tray")]
    pub minimize_to_tray: bool,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_diag_deg: 78.0,
            camera_resolution: Self::default_resolution(),
            fov_presets: Vec::new(),
            minimize_to_tray: Self::default_minimize_to_tray(),
        }
    }
}

impl CameraConfig {
    pub fn default_resolution() -> (u32, u32) { (640, 480) }
    pub fn default_minimize_to_tray() -> bool { true }

    fn config_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(xdg_config_base()).join("headtrack-rs")
    }

    pub fn config_path() -> std::path::PathBuf {
        Self::config_dir().join("camera.toml")
    }

    /// Load from `camera.toml`, returning defaults if not found or invalid.
    pub fn load() -> Self {
        let Ok(contents) = std::fs::read_to_string(Self::config_path()) else {
            return Self::default();
        };
        toml::from_str(&contents).unwrap_or_default()
    }

    /// Persist to `camera.toml`.
    pub fn save(&self) -> std::io::Result<()> {
        let path = Self::config_path();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let s = toml::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, s)
    }
}

/// Complete pipeline profile — every tunable parameter.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProfileConfig {
    /// Human-readable profile name.
    pub name: String,

    /// Per-axis One Euro filter [yaw, pitch, roll, x, y, z].
    pub filter: [AxisFilter; 6],

    /// Per-axis response curves [yaw, pitch, roll, x, y, z].
    pub curve: [AxisCurve; 6],

    /// Cross-axis compensation.
    pub cross_axis: CrossAxisComp,

    /// Kalman prediction.
    pub prediction: Prediction,

    /// Per-axis deadzone [yaw, pitch, roll, x, y, z].
    pub deadzone: [f32; 6],

    /// Velocity-gated centering.
    pub center: CenterConfig,

    /// Per-axis slew limit (max rate/s) [yaw, pitch, roll, x, y, z].
    pub slew_limit: [f32; 6],
}

impl Default for ProfileConfig {
    fn default() -> Self {
        Self {
            name: "Default".into(),
            filter: [
                AxisFilter { min_cutoff: 0.01, beta: 0.03 },
                AxisFilter { min_cutoff: 0.01, beta: 0.03 },
                AxisFilter { min_cutoff: 0.01, beta: 0.03 },
                AxisFilter { min_cutoff: 0.01, beta: 0.03 },
                AxisFilter { min_cutoff: 0.01, beta: 0.03 },
                AxisFilter { min_cutoff: 0.001, beta: 0.10 },
            ],
            curve: [
                AxisCurve { sensitivity: 1.0, exponent: 1.2 },
                AxisCurve { sensitivity: 1.0, exponent: 1.2 },
                AxisCurve { sensitivity: 1.0, exponent: 1.0 },
                AxisCurve { sensitivity: 1.0, exponent: 1.0 },
                AxisCurve { sensitivity: 1.0, exponent: 1.1 },
                AxisCurve { sensitivity: 1.5, exponent: 1.0 },
            ],
            cross_axis: CrossAxisComp {
                yaw_to_pitch: 0.0,
                yaw_to_x: -0.40,
                yaw_to_y: -0.40,
                yaw_to_roll: -0.55,
                pitch_to_y: 0.0,
                z_to_y: -0.50,
                z_to_pitch: 0.0,
            },
            prediction: Prediction {
                predict_ms: 10.0,
                rot_process_noise: 1.2,
                rot_measurement_noise: 0.7,
                pos_process_noise: 2.5,
                pos_measurement_noise: 10.0,
            },
            deadzone: [0.8, 0.8, 0.5, 2.0, 2.0, 3.0],
            center: CenterConfig {
                drift_rate: 0.05,
                z_drift_mult: 3.2,
            },
            slew_limit: [500.0, 500.0, 500.0, 300.0, 300.0, 200.0],
        }
    }
}

impl ProfileConfig {
    /// Default config directory: `$XDG_CONFIG_HOME/headtrack-rs/profiles/`
    /// (falls back to `~/.config/headtrack-rs/profiles/`).
    pub fn profiles_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(xdg_config_base()).join("headtrack-rs/profiles")
    }

    /// Path to the "last used" config that the daemon loads on startup.
    pub fn active_config_path() -> std::path::PathBuf {
        std::path::PathBuf::from(xdg_config_base()).join("headtrack-rs/active.toml")
    }

    /// Serialize to TOML string.
    pub fn to_toml(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    /// Deserialize from TOML string.
    pub fn from_toml(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    /// Save to a file (creates parent directories).
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let toml_str = self.to_toml().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, e)
        })?;
        std::fs::write(path, toml_str)
    }

    /// Load from a TOML file.
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_toml(&contents).map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, e)
        })
    }

    /// List all `.toml` profile files in the profiles directory.
    pub fn list_profiles() -> Vec<(String, std::path::PathBuf)> {
        let dir = Self::profiles_dir();
        let Ok(entries) = std::fs::read_dir(&dir) else {
            return Vec::new();
        };
        let mut profiles = Vec::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|ext| ext == "toml") {
                if let Ok(contents) = std::fs::read_to_string(&path) {
                    if let Ok(cfg) = Self::from_toml(&contents) {
                        profiles.push((cfg.name, path));
                    }
                }
            }
        }
        profiles.sort_by(|a, b| a.0.cmp(&b.0));
        profiles
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_toml() {
        let cfg = ProfileConfig::default();
        let toml_str = cfg.to_toml().unwrap();
        let parsed = ProfileConfig::from_toml(&toml_str).unwrap();
        assert_eq!(cfg.name, parsed.name);
        assert_eq!(cfg.deadzone, parsed.deadzone);
        assert!((cfg.filter[5].beta - parsed.filter[5].beta).abs() < 1e-6);
        assert!((cfg.cross_axis.yaw_to_roll - parsed.cross_axis.yaw_to_roll).abs() < 1e-6);
    }

    #[test]
    fn save_and_load() {
        let dir = std::env::temp_dir().join("headtrack-test-profiles");
        let path = dir.join("test_profile.toml");
        let cfg = ProfileConfig::default();
        cfg.save(&path).unwrap();
        let loaded = ProfileConfig::load(&path).unwrap();
        assert_eq!(cfg.name, loaded.name);
        assert_eq!(cfg.slew_limit, loaded.slew_limit);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
