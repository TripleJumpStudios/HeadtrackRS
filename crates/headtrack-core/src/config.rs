//! Profile configuration — serializable snapshot of all pipeline parameters.
//!
//! A [`ProfileConfig`] captures every tunable parameter in the pipeline so it
//! can be saved to / loaded from a TOML file.  The GUI writes these, and the
//! daemon can optionally load one at startup.

use serde::{Deserialize, Serialize};

/// Resolve XDG_CONFIG_HOME or fall back to `$HOME/.config`.
/// Prints a warning to stderr if both `$HOME` and `$XDG_CONFIG_HOME` are unset.
fn xdg_config_base() -> String {
    std::env::var("XDG_CONFIG_HOME").unwrap_or_else(|_| format!("{}/.config", home_base()))
}

/// `$HOME`, or `/tmp` with a warning. Single definition so every path in this
/// module degrades the same way on a machine without `$HOME`.
fn home_base() -> String {
    std::env::var("HOME").unwrap_or_else(|_| {
        eprintln!("warning: $HOME is not set — falling back to /tmp");
        "/tmp".into()
    })
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
    pub yaw_to_z: f32,
    pub pitch_to_y: f32,
    pub z_to_y: f32,
    pub z_to_pitch: f32,
    pub roll_to_z: f32,
    #[serde(default)]
    pub roll_to_x: f32,
    #[serde(default)]
    pub pitch_to_yaw: f32,
    #[serde(default)]
    pub pitch_to_z: f32,
}

/// Velocity-gated centering parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CenterConfig {
    pub drift_rate: f32,
    pub z_drift_mult: f32,
    /// Per-axis recenter range [yaw°, pitch°, roll°, x mm, y mm, z mm]:
    /// drift fades to zero as the pose moves this far from baseline, so a
    /// held off-center gaze is never absorbed. `None` keeps the stage
    /// defaults.
    #[serde(default)]
    pub recenter_range: Option<[f32; 6]>,
}

/// A named FOV preset the user can save and recall.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FovPreset {
    pub name: String,
    pub fov: f32,
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
    ///
    /// Two defaults, on purpose (Q-UI6b, 2026-10-29): a fresh install (no
    /// file) gets [`default_resolution`](Self::default_resolution), 1280×720,
    /// the measured operating point (`docs/CAMERAS.md`). A file that lacks the
    /// line was written by someone who ran 640×480 and keeps it
    /// ([`legacy_resolution`](Self::legacy_resolution)), so no existing user is
    /// moved.
    #[serde(default = "CameraConfig::legacy_resolution")]
    pub camera_resolution: (u32, u32),
    /// User-saved named FOV presets.
    #[serde(default)]
    pub fov_presets: Vec<FovPreset>,
    /// When true, closing the window hides to the system tray instead of quitting.
    #[serde(default = "CameraConfig::default_minimize_to_tray")]
    pub minimize_to_tray: bool,
    /// Default directory for diagnostic recordings.
    #[serde(default = "CameraConfig::default_diag_directory")]
    pub diag_directory: String,
    /// In-app hotkey (egui key name, e.g. "F9") for Recenter — active only while
    /// the HeadtrackRS window is focused. `None` = unbound.
    #[serde(default)]
    pub hotkey_recenter: Option<String>,
    /// In-app hotkey (egui key name) for toggling tracking on/off.
    #[serde(default)]
    pub hotkey_toggle: Option<String>,
    /// MSFS 2024 camera-output smoothing LERP speed (higher = snappier, lower =
    /// smoother). Output-side only; does not affect the shared pipeline / XP / SC.
    #[serde(default = "CameraConfig::default_msfs_smoothing")]
    pub msfs_camera_smoothing: f32,
    /// Star Citizen output smoothing speed (higher = snappier, lower = smoother;
    /// 0 = off). Output-side only; does not affect the shared pipeline / XP / MSFS.
    #[serde(default = "CameraConfig::default_sc_smoothing")]
    pub sc_output_smoothing: f32,
    /// Reveals the GUI's Advanced tab (per-axis pipeline internals, cross-axis
    /// compensation, rate limits, the diagnostics recorder).
    ///
    /// Off by default: those controls need measured data to set sensibly, and
    /// the product ships to people who have none. Toggled in Settings.
    #[serde(default)]
    pub show_advanced: bool,
    /// Last window size in logical points, restored on next launch.
    ///
    /// Size only, deliberately: this is a Wayland-first application and winit
    /// cannot position a window on Wayland — the compositor decides. Storing a
    /// position would be a value that silently never applies.
    ///
    /// Validate with [`CameraConfig::restored_window_size`] rather than reading
    /// the field directly; a hand-edited or stale value must not be able to
    /// produce a window that cannot be used.
    #[serde(default)]
    pub window_size: Option<(f32, f32)>,
    /// The active head source. Missing → webcam index 0, which is what every
    /// build before P05 opened. Parsed leniently: an unrecognised `kind` falls
    /// back to the default with a warning rather than failing the whole file —
    /// see [`lenient_head_source`]. Change it through
    /// [`select_webcam`](Self::select_webcam) / [`select_tobii`](Self::select_tobii)
    /// so [`last_webcam`](Self::last_webcam) stays in step.
    #[serde(default, deserialize_with = "lenient_head_source")]
    pub head_source: HeadSource,
    /// The webcam to return to after a Tobii excursion. Without it,
    /// webcam 2 → Tobii → webcam would land on index 0.
    #[serde(default)]
    pub last_webcam: Option<WebcamSelection>,
    /// Controller/HOTAS button bindings for engine actions (recenter, MSFS
    /// seat nudges, camera toggle). Bound in-app via the Keybinds tab; read
    /// from evdev so they fire regardless of window focus. Kept last in the
    /// struct — serializes as an array of tables.
    #[serde(default)]
    pub controller_bindings: Vec<ControllerBinding>,
}

/// One controller-button → engine-action binding.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControllerBinding {
    /// Engine action id (e.g. `recenter`, `msfs-seat-fwd`).
    pub action: String,
    /// evdev device name the button lives on (trimmed exact match).
    pub device: String,
    /// EV_KEY code of the button.
    pub code: u16,
    /// Display label (e.g. "VKBsim Gladiator EVO R · BTN_TRIGGER").
    pub label: String,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_diag_deg: 78.0,
            camera_resolution: Self::default_resolution(),
            fov_presets: Vec::new(),
            minimize_to_tray: Self::default_minimize_to_tray(),
            diag_directory: Self::default_diag_directory(),
            hotkey_recenter: None,
            hotkey_toggle: None,
            msfs_camera_smoothing: Self::default_msfs_smoothing(),
            sc_output_smoothing: Self::default_sc_smoothing(),
            show_advanced: false,
            window_size: None,
            head_source: HeadSource::default(),
            last_webcam: None,
            controller_bindings: Vec::new(),
        }
    }
}

/// A persisted webcam choice.
///
/// The name is preferred on open: V4L2 indices are not stable across reboots
/// and replugs, so an index alone reopens the wrong device as often as it
/// fixes the "always index 0" bug. The index is the fallback when the name is
/// absent (older file) or no longer present.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WebcamSelection {
    /// V4L2 index at the time of selection.
    #[serde(default)]
    pub index: u32,
    /// V4L2 device (card) name at the time of selection.
    #[serde(default)]
    pub name: Option<String>,
}

/// Where head pose comes from. Persisted as `[head_source]` in `camera.toml`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum HeadSource {
    /// A V4L2 webcam through the NeuralNet tracker. See [`WebcamSelection`].
    Webcam {
        #[serde(default)]
        index: u32,
        #[serde(default)]
        name: Option<String>,
    },
    /// Tobii Eye Tracker 5 IR head rotation. `serial: None` means the single
    /// connected tracker; with two connected, opening without a serial fails.
    Tobii {
        #[serde(default)]
        serial: Option<String>,
    },
}

impl Default for HeadSource {
    fn default() -> Self {
        Self::Webcam {
            index: 0,
            name: None,
        }
    }
}

impl HeadSource {
    pub fn kind(&self) -> crate::input::SourceKind {
        match self {
            Self::Webcam { .. } => crate::input::SourceKind::Webcam,
            Self::Tobii { .. } => crate::input::SourceKind::Tobii,
        }
    }
}

/// Deserialize `head_source` without letting a bad value fail the file.
///
/// `CameraConfig::load` quarantines any file that fails to parse and returns
/// defaults — which costs the user every controller binding. A source kind this
/// build does not know (written by a newer build, or a hand edit) is not worth
/// that, so it resolves to the default source with a warning. Only the
/// `head_source` table is lenient; a malformed file is still an error.
fn lenient_head_source<'de, D>(d: D) -> Result<HeadSource, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = toml::Value::deserialize(d)?;
    match value.try_into::<HeadSource>() {
        Ok(source) => Ok(source),
        Err(e) => {
            eprintln!("warning: camera.toml [head_source] not understood ({e}) — using webcam 0");
            Ok(HeadSource::default())
        }
    }
}

impl CameraConfig {
    /// A fresh install's capture resolution: 1280×720 (Q-UI6b).
    pub fn default_resolution() -> (u32, u32) {
        (1280, 720)
    }
    /// What a `camera.toml` without a `camera_resolution` line ran at, and so
    /// keeps: the 640×480 default of every version before 0.2.0.
    pub fn legacy_resolution() -> (u32, u32) {
        (640, 480)
    }
    pub fn default_minimize_to_tray() -> bool {
        true
    }
    /// Matches the X-Plane plugin's `LERP_SPEED` so MSFS starts at the same feel.
    pub fn default_msfs_smoothing() -> f32 {
        15.0
    }
    /// Matches X-Plane's LERP speed, so all three targets start with the same feel.
    pub fn default_sc_smoothing() -> f32 {
        15.0
    }

    /// Kind of the active head source.
    pub fn source_kind(&self) -> crate::input::SourceKind {
        self.head_source.kind()
    }

    /// The webcam to open when the webcam is (or becomes) the source: the
    /// active one, else the one remembered from before a Tobii excursion,
    /// else index 0.
    pub fn webcam_selection(&self) -> WebcamSelection {
        match &self.head_source {
            HeadSource::Webcam { index, name } => WebcamSelection {
                index: *index,
                name: name.clone(),
            },
            HeadSource::Tobii { .. } => self.last_webcam.clone().unwrap_or_default(),
        }
    }

    /// Make a webcam the active source and remember it.
    pub fn select_webcam(&mut self, index: u32, name: Option<String>) {
        self.last_webcam = Some(WebcamSelection {
            index,
            name: name.clone(),
        });
        self.head_source = HeadSource::Webcam { index, name };
    }

    /// Make the Tobii the active source, remembering the current webcam so
    /// switching back reopens it rather than index 0.
    pub fn select_tobii(&mut self, serial: Option<String>) {
        if let HeadSource::Webcam { .. } = self.head_source {
            self.last_webcam = Some(self.webcam_selection());
        }
        self.head_source = HeadSource::Tobii { serial };
    }

    /// Window size on a first run, in logical points.
    ///
    /// The height is set so the Dashboard — the tab the app opens on — fits
    /// without scrolling. Measured: its content is 640 pt tall and the window
    /// chrome (status bar + insets) costs 66, so anything under ~706 clips the
    /// pose cluster. 740 leaves headroom for a larger UI font.
    pub const DEFAULT_WINDOW: (f32, f32) = (980.0, 740.0);
    /// Smallest window the GUI is usable in, in logical points. Also the floor
    /// a restored size is clamped to.
    pub const MIN_WINDOW: (f32, f32) = (860.0, 620.0);
    /// Ceiling for a restored size. Guards against a value saved on a much
    /// larger display coming back as a window bigger than the current screen,
    /// which some compositors will honour and leave partly unreachable.
    pub const MAX_WINDOW: (f32, f32) = (3840.0, 2160.0);

    /// The saved window size, if it is one we are willing to restore.
    ///
    /// Rejects non-finite values (a corrupt or hand-edited TOML) and clamps to
    /// [`MIN_WINDOW`]..[`MAX_WINDOW`]. Returning `None` means "use the built-in
    /// default", which is always usable.
    pub fn restored_window_size(&self) -> Option<(f32, f32)> {
        let (w, h) = self.window_size?;
        if !w.is_finite() || !h.is_finite() || w <= 0.0 || h <= 0.0 {
            return None;
        }
        Some((
            w.clamp(Self::MIN_WINDOW.0, Self::MAX_WINDOW.0),
            h.clamp(Self::MIN_WINDOW.1, Self::MAX_WINDOW.1),
        ))
    }
    /// Default directory for diagnostic CSV recordings.
    ///
    /// Goes through the same `$HOME` fallback (and warning) as every other
    /// path in this module rather than reading `$HOME` directly, so a machine
    /// without `$HOME` behaves consistently instead of silently writing
    /// diagnostics somewhere different from the config.
    pub fn default_diag_directory() -> String {
        home_base()
    }

    fn config_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(xdg_config_base()).join("headtrack-rs")
    }

    pub fn config_path() -> std::path::PathBuf {
        Self::config_dir().join("camera.toml")
    }

    /// Load from `camera.toml`, returning defaults if not found or invalid.
    /// Load from `camera.toml`, returning defaults if it is missing or invalid.
    ///
    /// An unparseable file is **moved aside** before the defaults are returned.
    /// Callers use a read-modify-write pattern (`load()`, tweak one field,
    /// `save()`), so silently falling back to defaults means the very next
    /// setting the user touches rewrites the file with those defaults — wiping
    /// every controller binding in one step, with no warning and no copy. A
    /// single mistyped line in a hand-edited config was enough to trigger it.
    pub fn load() -> Self {
        let path = Self::config_path();
        let Ok(contents) = std::fs::read_to_string(&path) else {
            return Self::default();
        };
        match toml::from_str(&contents) {
            Ok(cfg) => cfg,
            Err(e) => {
                let backup = path.with_file_name(format!(
                    "camera.invalid-{}.toml",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map(|d| d.as_secs())
                        .unwrap_or(0)
                ));
                let rescued = std::fs::rename(&path, &backup).is_ok();
                eprintln!("warning: {} is not valid TOML: {e}", path.display());
                if rescued {
                    eprintln!(
                        "warning: it has been moved to {} and defaults will be used — \
                         your controller bindings are in that file",
                        backup.display()
                    );
                } else {
                    eprintln!("warning: could not move it aside; using defaults");
                }
                Self::default()
            }
        }
    }

    /// Persist to `camera.toml`.
    ///
    /// Writes a sibling temp file and renames it into place, so the config is
    /// replaced atomically. Plain `fs::write` truncates first, which leaves a
    /// window where the file on disk is empty or half-written — and this config
    /// has many writers (GUI settings, the camera tab, the engine's FOV and
    /// window-size persistence, the controller binding editor) racing against
    /// readers on other threads. A reader that landed in that window got a TOML
    /// parse error and [`load`](Self::load) moved the (by then complete) file
    /// aside as invalid, costing the user every controller binding. Observed on
    /// 2026-08-17.
    pub fn save(&self) -> std::io::Result<()> {
        let path = Self::config_path();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let s = toml::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        write_atomic(&path, &s)
    }
}

/// Write `contents` to `path` atomically: a same-directory temp file, then
/// `rename` over the target. A concurrent reader sees either the whole old
/// file or the whole new one, never a partial write.
///
/// The temp name carries the pid so two processes writing at once cannot
/// scribble on each other's staging file. The rename is what makes it atomic;
/// it requires the temp file to be on the same filesystem, hence the sibling.
pub(crate) fn write_atomic(path: &std::path::Path, contents: &str) -> std::io::Result<()> {
    let tmp = path.with_extension(format!("tmp-{}", std::process::id()));
    // Scoped so the handle is closed (and flushed) before the rename.
    {
        use std::io::Write;
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(contents.as_bytes())?;
        f.sync_all()?;
    }
    match std::fs::rename(&tmp, path) {
        Ok(()) => Ok(()),
        Err(e) => {
            let _ = std::fs::remove_file(&tmp);
            Err(e)
        }
    }
}

/// Per-axis Accela displacement-filter parameters (the default smoothing
/// backend). `max_gain` is deliberately not part of the profile — it is a
/// safety clamp, live-settable via the command socket only.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AccelaAxis {
    /// Displacement deadband (axis units: deg or mm) — holds the output still.
    pub deadzone: f32,
    /// Displacement scale for the gain curve; higher = smoother/laggier.
    pub smoothing: f32,
    /// Gain-curve exponent; higher = softer near rest, faster at large moves.
    pub exponent: f32,
}

/// Complete pipeline profile — every tunable parameter.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProfileConfig {
    /// Human-readable profile name.
    pub name: String,

    /// Per-axis response curves [yaw, pitch, roll, x, y, z].
    pub curve: [AxisCurve; 6],

    /// Cross-axis compensation.
    pub cross_axis: CrossAxisComp,

    /// Per-axis deadzone [yaw, pitch, roll, x, y, z].
    pub deadzone: [f32; 6],

    /// Velocity-gated centering.
    pub center: CenterConfig,

    /// Per-axis slew limit (max rate/s) [yaw, pitch, roll, x, y, z].
    pub slew_limit: [f32; 6],

    /// Per-axis Accela displacement filter [yaw, pitch, roll, x, y, z]
    /// (default backend). The serde default is mandatory: profiles written
    /// before this section existed must keep parsing, and must resolve to the
    /// AccelaStage constructor values (the flown-and-approved defaults).
    #[serde(default = "ProfileConfig::default_accela")]
    pub accela: [AccelaAxis; 6],
}

impl Default for ProfileConfig {
    fn default() -> Self {
        Self {
            name: "Default".into(),
            //                          yaw  pitch roll   x    y    z
            curve: [
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.2,
                },
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.2,
                },
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.0,
                },
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.0,
                },
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.0,
                },
                AxisCurve {
                    sensitivity: 1.0,
                    exponent: 1.0,
                },
            ],
            cross_axis: CrossAxisComp {
                yaw_to_pitch: 0.0,
                yaw_to_x: 0.0,
                yaw_to_y: 0.0,
                yaw_to_roll: 0.0,
                yaw_to_z: 0.0,
                // Nodding slides Y and Z on a webcam: the eye centroid rises as
                // the head pitches and the IPD cue reads the foreshortening as
                // depth. Measured, not guessed — see docs/PIPELINE.md.
                pitch_to_y: 1.75,
                pitch_to_z: -1.5,
                // Residual after 6DRepNet took over rotation; the old 0.25
                // (calibrated against PnP rotation) over-corrected ~5×.
                pitch_to_yaw: 0.05,
                z_to_y: 0.0,
                z_to_pitch: 0.0,
                roll_to_z: 0.0,
                roll_to_x: 0.0,
            },
            //         yaw  pitch roll   x    y     z
            deadzone: [0.8, 0.8, 0.5, 2.0, 2.1, 10.0],
            center: CenterConfig {
                drift_rate: 0.2,
                z_drift_mult: 3.2,
                //                     yaw   pitch roll   x     y     z
                recenter_range: Some([12.0, 12.0, 4.0, 10.0, 10.0, 60.0]),
            },
            //            yaw   pitch  roll     x      y      z
            slew_limit: [500.0, 80.0, 500.0, 300.0, 300.0, 200.0],
            accela: Self::default_accela(),
        }
    }
}

impl ProfileConfig {
    /// Accela section defaults — the single definition of the shipped tuning.
    ///
    /// `AccelaStage::new()` reads these rather than holding its own copy, so a
    /// profile without an `[[accela]]` section and a freshly-built pipeline
    /// cannot disagree. Before SL3 both sites held the same literals, kept in
    /// step by a comment and a test.
    pub fn default_accela() -> [AccelaAxis; 6] {
        //                              yaw   pitch  roll   x     y     z
        let deadzone: [f32; 6] = [0.12, 0.12, 0.12, 0.40, 0.40, 1.00];
        let smoothing: [f32; 6] = [0.60, 0.60, 0.60, 2.00, 2.00, 3.00];
        let exponent: [f32; 6] = [1.40, 1.40, 1.40, 1.40, 1.40, 1.50];
        std::array::from_fn(|i| AccelaAxis {
            deadzone: deadzone[i],
            smoothing: smoothing[i],
            exponent: exponent[i],
        })
    }

    /// Default config directory: `$XDG_CONFIG_HOME/headtrack-rs/profiles/`
    /// (falls back to `~/.config/headtrack-rs/profiles/`).
    pub fn profiles_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(xdg_config_base()).join("headtrack-rs/profiles")
    }

    /// Path to the "last used" config that the daemon loads on startup.
    pub fn active_config_path() -> std::path::PathBuf {
        Self::active_config_path_for(crate::input::SourceKind::Webcam)
    }

    /// The active profile for one head source. The webcam keeps `active.toml`
    /// so every existing install keeps its tuning; the Tobii gets its own file
    /// because a webcam translation tuning means nothing to a 3DOF source.
    pub fn active_config_path_for(kind: crate::input::SourceKind) -> std::path::PathBuf {
        let file = match kind {
            crate::input::SourceKind::Webcam => "active.toml",
            crate::input::SourceKind::Tobii => "active-tobii.toml",
        };
        std::path::PathBuf::from(xdg_config_base())
            .join("headtrack-rs")
            .join(file)
    }

    /// Name of the built-in Tobii profile (P08).
    pub const TOBII_DEFAULT_NAME: &'static str = "Tobii Default";

    /// The built-in profile for the Tobii head source: 3DOF rotation only.
    ///
    /// Every difference from [`Default`](ProfileConfig::default) is deliberate,
    /// and starts from the one chain that has flown on the Tobii: TobiiLab's
    /// service, which ran the raw neutral-relative rotation through the Accela
    /// constructor defaults and nothing else (MSFS: "correct but a bit hot").
    ///
    /// - **Cross-axis all 0.** Every webcam term corrects a *webcam* artefact
    ///   (the eye centroid rising on a nod, the depth cue foreshortening,
    ///   6DRepNet's pitch→yaw residual). The Tobii's rotation comes from a
    ///   Face Mesh similarity fit with none of those couplings, and with
    ///   translation held at 0 the rotation→translation terms would *invent*
    ///   translation.
    /// - **Translation sensitivity 0.** The source sends x = y = z = 0; zero
    ///   gain guarantees nothing upstream (center, cross-axis) can put a
    ///   non-zero value on X/Y/Z either. P05: translation is held at 0.
    /// - **Rotation curve linear (exponent 1.0) and rotation deadzone 0.**
    ///   TobiiLab flew with neither; Accela's own deadband (0.12°) is the
    ///   rest-stillness mechanism, as it was there. The webcam's 1.2 exponent
    ///   and 0.8° deadzone were fitted to webcam noise, and the exponent would
    ///   make the already-hot large angles hotter.
    /// - **Center drift 0.** The Tobii has a source-side neutral (taken on
    ///   first acquisition and on Recenter); auto-drift exists to chase a
    ///   webcam's slowly wandering zero and would absorb a held gaze
    ///   (see docs/PIPELINE.md, center drift).
    /// - Slew limit and Accela: the shared defaults (spike guard + the flown filter).
    ///
    /// Pitch-up gain is left at 1.0 here; lowering it waits on live data (P18).
    pub fn tobii_default() -> Self {
        let base = Self::default();
        let rot = AxisCurve {
            sensitivity: 1.0,
            exponent: 1.0,
        };
        let none = AxisCurve {
            sensitivity: 0.0,
            exponent: 1.0,
        };
        Self {
            name: Self::TOBII_DEFAULT_NAME.into(),
            curve: [
                rot.clone(),
                rot.clone(),
                rot,
                none.clone(),
                none.clone(),
                none,
            ],
            cross_axis: CrossAxisComp {
                yaw_to_pitch: 0.0,
                yaw_to_x: 0.0,
                yaw_to_y: 0.0,
                yaw_to_roll: 0.0,
                yaw_to_z: 0.0,
                pitch_to_y: 0.0,
                z_to_y: 0.0,
                z_to_pitch: 0.0,
                roll_to_z: 0.0,
                roll_to_x: 0.0,
                pitch_to_yaw: 0.0,
                pitch_to_z: 0.0,
            },
            //         yaw  pitch roll  x    y    z
            deadzone: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            center: CenterConfig {
                drift_rate: 0.0,
                ..base.center
            },
            slew_limit: base.slew_limit,
            accela: base.accela,
        }
    }

    /// The profile a source runs when it has no active file.
    pub fn builtin_for(kind: crate::input::SourceKind) -> Self {
        match kind {
            crate::input::SourceKind::Webcam => Self::default(),
            crate::input::SourceKind::Tobii => Self::tobii_default(),
        }
    }

    /// Load the active profile of one source from `dir`: its file
    /// ([`active_config_path_for`](Self::active_config_path_for) naming), or the
    /// built-in when the file does not exist. Any other failure is returned so
    /// the caller can say a saved tuning is being ignored.
    pub fn load_active_in(
        dir: &std::path::Path,
        kind: crate::input::SourceKind,
    ) -> std::io::Result<Self> {
        let path = dir.join(
            Self::active_config_path_for(kind)
                .file_name()
                .expect("active profile path has a file name"),
        );
        match Self::load(&path) {
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Self::builtin_for(kind)),
            other => other,
        }
    }

    /// [`load_active_in`](Self::load_active_in) on the real config directory.
    pub fn load_active_for(kind: crate::input::SourceKind) -> std::io::Result<Self> {
        let path = Self::active_config_path_for(kind);
        let dir = path.parent().expect("active profile path has a parent");
        Self::load_active_in(dir, kind)
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
    ///
    /// Atomic, for the same reason as [`CameraConfig::save`]: `active.toml` is
    /// rewritten by the GUI while the engine may be reading it.
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let toml_str = self
            .to_toml()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        write_atomic(path, &toml_str)
    }

    /// Load from a TOML file.
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_toml(&contents)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
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

    /// A malformed `camera.toml` must be recognised as an error, not quietly
    /// deserialised into something lossy.
    ///
    /// This is the trigger for the data-loss path in [`CameraConfig::load`]:
    /// every caller does load → tweak one field → save, so a config that fails
    /// to parse and falls back to defaults gets written back as defaults,
    /// destroying the user's controller bindings. A duplicate key — trivially
    /// produced by a hand edit — is enough.
    #[test]
    fn malformed_camera_config_is_an_error_not_a_silent_default() {
        let duplicate_key = r#"
fov_diag_deg = 84.0
show_advanced = true
show_advanced = false
"#;
        assert!(
            toml::from_str::<CameraConfig>(duplicate_key).is_err(),
            "a duplicate key must fail to parse, so load() can rescue the file"
        );

        let truncated = "fov_diag_deg = 84.0\n[[controller_bindings]]\naction = \"recenter\"\n";
        assert!(
            toml::from_str::<CameraConfig>(truncated).is_err(),
            "a partially written binding table must fail to parse"
        );
    }

    /// Controller bindings must survive a normal save/load cycle — they are the
    /// most expensive thing in this file to recreate by hand.
    #[test]
    fn controller_bindings_survive_a_roundtrip() {
        let mut cfg = CameraConfig::default();
        cfg.controller_bindings.push(ControllerBinding {
            action: "recenter".into(),
            device: "VKBsim Gladiator EVO R".into(),
            code: 721,
            label: "VKBsim Gladiator EVO R · BTN_TRIGGER_HAPPY18".into(),
        });
        cfg.show_advanced = true;

        let s = toml::to_string_pretty(&cfg).expect("serialize");
        let parsed: CameraConfig = toml::from_str(&s).expect("deserialize");

        assert_eq!(parsed.controller_bindings.len(), 1);
        assert_eq!(parsed.controller_bindings[0].code, 721);
        assert_eq!(parsed.controller_bindings[0].action, "recenter");
        assert!(parsed.show_advanced);
    }

    /// A reader must never observe a half-written config.
    ///
    /// This is the mechanism that quarantined a perfectly good `camera.toml` on
    /// 2026-08-17: `fs::write` truncates before it writes, and this file has
    /// several writer threads (GUI settings, engine FOV/window-size, controller
    /// bindings) racing readers on others. A reader landing in the truncation
    /// window sees invalid TOML, `load()` renames the file aside, and every
    /// controller binding is gone. With the rename-into-place write, each read
    /// returns one whole version or the other.
    #[test]
    fn concurrent_writes_are_never_observed_half_written() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;

        let dir = std::env::temp_dir().join(format!("htrs-atomic-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("camera.toml");

        // Two versions long enough that a truncating write would leave a
        // readable-but-incomplete file for a meaningful window.
        let short = format!("fov_diag_deg = 78.0\n{}", "# pad\n".repeat(200));
        let long = format!("fov_diag_deg = 84.0\n{}", "# pad\n".repeat(4000));
        write_atomic(&path, &short).expect("seed");

        let stop = Arc::new(AtomicBool::new(false));
        let writer = {
            let (path, stop, short, long) =
                (path.clone(), Arc::clone(&stop), short.clone(), long.clone());
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    write_atomic(&path, &long).expect("write long");
                    write_atomic(&path, &short).expect("write short");
                }
            })
        };

        for _ in 0..2_000 {
            let seen = std::fs::read_to_string(&path).expect("config is always readable");
            assert!(
                seen == short || seen == long,
                "read a partial config: {} bytes",
                seen.len()
            );
        }
        stop.store(true, Ordering::Relaxed);
        writer.join().expect("writer thread");

        // No staging files left behind.
        let strays: Vec<_> = std::fs::read_dir(&dir)
            .expect("read dir")
            .filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned()))
            .filter(|n| n != "camera.toml")
            .collect();
        assert!(strays.is_empty(), "temp files left behind: {strays:?}");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn roundtrip_toml() {
        let mut cfg = ProfileConfig::default();
        cfg.accela[5].smoothing = 4.25; // non-default value must survive the trip
        let toml_str = cfg.to_toml().unwrap();
        let parsed = ProfileConfig::from_toml(&toml_str).unwrap();
        assert_eq!(cfg.name, parsed.name);
        assert_eq!(cfg.deadzone, parsed.deadzone);
        assert!((cfg.curve[5].sensitivity - parsed.curve[5].sensitivity).abs() < 1e-6);
        assert!((cfg.cross_axis.yaw_to_roll - parsed.cross_axis.yaw_to_roll).abs() < 1e-6);
        assert!((parsed.accela[5].smoothing - 4.25).abs() < 1e-6);
    }

    /// Profiles written before the accela section existed must keep parsing
    /// and must resolve to the constructor defaults. A missing serde default
    /// on a new ProfileConfig field silently destroys every saved profile
    /// (engine → "no profile"; GUI → unwrap_or_default) — this is the guard.
    #[test]
    fn pre_accela_profile_parses_to_defaults() {
        let toml_str = ProfileConfig::default().to_toml().unwrap();
        let stripped: String = {
            // Remove every [[accela]] table (header + its key lines).
            let mut out = String::new();
            let mut in_accela = false;
            for line in toml_str.lines() {
                let t = line.trim();
                if t == "[[accela]]" {
                    in_accela = true;
                    continue;
                }
                if in_accela {
                    if t.starts_with('[') {
                        in_accela = false; // next section begins
                    } else {
                        continue;
                    }
                }
                out.push_str(line);
                out.push('\n');
            }
            out
        };
        assert!(!stripped.contains("accela"), "strip failed: {stripped}");
        let parsed = ProfileConfig::from_toml(&stripped).expect("pre-accela profile must parse");
        for (got, want) in parsed
            .accela
            .iter()
            .zip(ProfileConfig::default_accela().iter())
        {
            assert!((got.deadzone - want.deadzone).abs() < 1e-6);
            assert!((got.smoothing - want.smoothing).abs() < 1e-6);
            assert!((got.exponent - want.exponent).abs() < 1e-6);
        }
    }

    /// **The v7-era profile compatibility guard.**
    ///
    /// Snow Leopard 2 deleted the legacy One-Euro + Kalman backend, and with it
    /// the `[[filter]]` and `[prediction]` sections of `ProfileConfig`. Every
    /// profile a user already has on disk still contains them — including the
    /// shipping `active.toml` ("MSFS TRANSLATION v7").
    ///
    /// serde ignores unknown keys by default, so those sections are skipped
    /// rather than rejected. That behaviour is load-bearing and invisible:
    /// adding `#[serde(deny_unknown_fields)]` to `ProfileConfig` would break
    /// every saved profile on the next launch, and the failure would look like
    /// "my tuning reset itself", not like a parse error. This test makes that
    /// regression fail loudly here instead.
    #[test]
    fn v7_era_profile_with_legacy_sections_still_parses() {
        // A faithful excerpt of a real v7-era profile: legacy sections present,
        // no [[accela]] section, and hand-tuned values that must survive.
        let v7 = r#"
name = "MSFS TRANSLATION v7 (v6 + pitch_to_yaw 0.25 -> 0.05)"
deadzone = [0.8, 0.8, 0.5, 2.0, 2.1, 10.0]
slew_limit = [500.0, 80.0, 500.0, 300.0, 300.0, 200.0]

[[filter]]
min_cutoff = 0.005
beta = 0.03

[[filter]]
min_cutoff = 0.025
beta = 0.03

[[filter]]
min_cutoff = 0.01
beta = 0.03

[[filter]]
min_cutoff = 0.01
beta = 0.03

[[filter]]
min_cutoff = 0.3
beta = 0.03

[[filter]]
min_cutoff = 0.001
beta = 0.1

[[curve]]
sensitivity = 1.0
exponent = 1.2

[[curve]]
sensitivity = 1.0
exponent = 1.2

[[curve]]
sensitivity = 1.0
exponent = 1.0

[[curve]]
sensitivity = 1.0
exponent = 1.0

[[curve]]
sensitivity = 1.0
exponent = 1.0

[[curve]]
sensitivity = 1.0
exponent = 1.0

[cross_axis]
yaw_to_pitch = 0.0
yaw_to_x = 0.0
yaw_to_y = 0.0
yaw_to_roll = 0.0
yaw_to_z = 0.0
pitch_to_y = 1.75
z_to_y = 0.0
z_to_pitch = 0.0
roll_to_z = 0.0
roll_to_x = 0.0
pitch_to_yaw = 0.05
pitch_to_z = -1.5

[prediction]
predict_ms = 9.0
rot_process_noise = 3.0
rot_measurement_noise = 1.0
pos_process_noise = 3.0
pos_measurement_noise = 10.0

[center]
drift_rate = 0.2
z_drift_mult = 3.2
recenter_range = [12.0, 12.0, 4.0, 10.0, 10.0, 60.0]
"#;

        let cfg = ProfileConfig::from_toml(v7)
            .expect("a v7-era profile with [[filter]]/[prediction] must still parse");

        // The surviving tuning must come through intact.
        assert_eq!(
            cfg.name,
            "MSFS TRANSLATION v7 (v6 + pitch_to_yaw 0.25 -> 0.05)"
        );
        assert!(
            (cfg.slew_limit[1] - 80.0).abs() < 1e-6,
            "hand-tuned pitch slew lost"
        );
        assert!(
            (cfg.deadzone[5] - 10.0).abs() < 1e-6,
            "widened Z deadzone lost"
        );
        assert!((cfg.cross_axis.pitch_to_y - 1.75).abs() < 1e-6);
        assert!((cfg.cross_axis.pitch_to_yaw - 0.05).abs() < 1e-6);
        assert!((cfg.cross_axis.pitch_to_z - (-1.5)).abs() < 1e-6);
        assert!((cfg.center.drift_rate - 0.2).abs() < 1e-6);
        assert_eq!(
            cfg.center.recenter_range,
            Some([12.0, 12.0, 4.0, 10.0, 10.0, 60.0])
        );

        // No [[accela]] section → the flown-and-approved constructor defaults.
        for (got, want) in cfg
            .accela
            .iter()
            .zip(ProfileConfig::default_accela().iter())
        {
            assert!((got.deadzone - want.deadzone).abs() < 1e-6);
            assert!((got.smoothing - want.smoothing).abs() < 1e-6);
            assert!((got.exponent - want.exponent).abs() < 1e-6);
        }

        // And it must apply cleanly to the live chain.
        let mut pipeline = crate::pipeline::build_default_pipeline();
        crate::pipeline::apply_config(&mut pipeline, &cfg);
        let params = pipeline.get_all_params();
        let slew_pitch = params
            .iter()
            .find(|(s, p, _)| *s == "slew-limit" && *p == "pitch.max_rate")
            .map(|(_, _, v)| *v)
            .expect("slew-limit pitch param");
        assert!((slew_pitch - 80.0).abs() < 1e-6);
    }

    /// A restored window size comes off disk, where it can be stale (saved on a
    /// much larger display), hand-edited, or corrupt. None of those may produce
    /// a window the user cannot resize back out of.
    #[test]
    fn restored_window_size_is_always_usable() {
        let mut cfg = CameraConfig::default();

        // Nothing saved yet → caller uses its own default.
        assert_eq!(cfg.restored_window_size(), None);

        // A sane value survives untouched.
        cfg.window_size = Some((1200.0, 800.0));
        assert_eq!(cfg.restored_window_size(), Some((1200.0, 800.0)));

        // Too small to use → raised to the minimum, not rejected: the user's
        // intent (a small window) is honoured as far as it can be.
        cfg.window_size = Some((100.0, 50.0));
        assert_eq!(cfg.restored_window_size(), Some(CameraConfig::MIN_WINDOW));

        // Saved on a 8K display, reopened on a laptop → clamped.
        cfg.window_size = Some((99_000.0, 99_000.0));
        assert_eq!(cfg.restored_window_size(), Some(CameraConfig::MAX_WINDOW));

        // Corrupt values fall back to the built-in default.
        for bad in [
            (f32::NAN, 700.0),
            (1000.0, f32::INFINITY),
            (0.0, 700.0),
            (-1200.0, 800.0),
        ] {
            cfg.window_size = Some(bad);
            assert_eq!(cfg.restored_window_size(), None, "{bad:?} must be rejected");
        }
    }

    /// The size must survive the TOML round-trip — it is written by one process
    /// and read by the next.
    #[test]
    fn window_size_round_trips_through_toml() {
        let cfg = CameraConfig {
            window_size: Some((1280.5, 742.25)),
            ..CameraConfig::default()
        };
        let text = toml::to_string_pretty(&cfg).expect("serialize");
        let back: CameraConfig = toml::from_str(&text).expect("deserialize");
        assert_eq!(back.window_size, Some((1280.5, 742.25)));
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
