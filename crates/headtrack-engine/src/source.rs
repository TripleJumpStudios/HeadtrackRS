//! Head-source selection, opening and persistence (task P07, on the P05 contract).
//!
//! Every "which source, which device, which profile" decision the run loop makes
//! goes through here, so the rules live in one place:
//!
//! - **Never two opens of one device.** [`replace_source`] drops the old source
//!   (which joins its threads and releases the device) *before* opening the new one.
//! - **Webcam precedence on open** (P05 §2): `HEADTRACK_CAMERA` env (startup only,
//!   never persisted) > V4L2 name match > saved index > 0. Indices move across
//!   reboots and replugs; names mostly do not (KNOWN_BUGS #2).
//! - **Each source has its own active profile** (P05 §7, P08): `active.toml` for
//!   the webcam, `active-tobii.toml` for the Tobii, falling back to the built-in
//!   ("Default" / "Tobii Default").
use std::{
    path::Path,
    sync::{atomic::{AtomicBool, Ordering}, Arc},
};

use headtrack_core::{
    config::{CameraConfig, HeadSource, ProfileConfig, WebcamSelection},
    pipeline::{apply_config, build_default_pipeline},
    InputSource, Pipeline, SourceKind,
};
use tracing::{info, warn};

use crate::EngineState;

/// `(index, card name)` of every V4L2 node, ascending index.
pub(crate) fn v4l2_devices() -> Vec<(u32, String)> {
    let Ok(dir) = std::fs::read_dir("/sys/class/video4linux") else {
        return Vec::new();
    };
    let mut out: Vec<(u32, String)> = dir
        .flatten()
        .filter_map(|e| {
            let index = e.file_name().to_str()?.strip_prefix("video")?.parse().ok()?;
            let name = std::fs::read_to_string(e.path().join("name")).ok()?;
            Some((index, name.trim().to_string()))
        })
        .collect();
    out.sort();
    out
}

/// Which V4L2 index to open for a saved selection.
///
/// A saved name wins over the saved index when that name is present. A camera
/// usually exposes two nodes with the same name (capture + metadata), so among
/// several matches the saved index is kept if it is one of them, otherwise the
/// lowest (the capture node). No name, or the name is gone: the saved index.
pub(crate) fn pick_webcam_index(sel: &WebcamSelection, devices: &[(u32, String)]) -> u32 {
    let Some(name) = sel.name.as_deref() else {
        return sel.index;
    };
    let matches: Vec<u32> = devices
        .iter()
        .filter(|(_, n)| n == name)
        .map(|(i, _)| *i)
        .collect();
    if matches.contains(&sel.index) {
        sel.index
    } else {
        matches.first().copied().unwrap_or(sel.index)
    }
}

/// The source to open at startup: the persisted one, unless `HEADTRACK_CAMERA`
/// names a webcam index (a one-off override, never persisted), or the build has
/// no Tobii support and the saved source is the Tobii.
pub(crate) fn startup_head_source(cfg: &CameraConfig, env_camera: Option<u32>) -> HeadSource {
    if let Some(index) = env_camera {
        return HeadSource::Webcam { index, name: None };
    }
    match &cfg.head_source {
        HeadSource::Tobii { .. } if !crate::TOBII_SUPPORTED => {
            warn!("camera.toml selects the Tobii, but this build has no Tobii support — using the webcam");
            let s = cfg.webcam_selection();
            HeadSource::Webcam { index: s.index, name: s.name }
        }
        other => other.clone(),
    }
}

/// Persist the active source in `camera.toml` (atomic write, P05 helpers).
pub(crate) fn persist_head_source(source: &HeadSource) -> std::io::Result<()> {
    let mut cfg = CameraConfig::load();
    match source {
        HeadSource::Webcam { index, name } => cfg.select_webcam(*index, name.clone()),
        HeadSource::Tobii { serial } => cfg.select_tobii(serial.clone()),
    }
    cfg.save()
}

/// Apply `kind`'s active profile to `pipeline`, logging which one runs.
///
/// Webcam with no file deliberately applies nothing (the stage constructors
/// *are* the built-in Default — guarded by `default_profile_is_the_no_profile_baseline`
/// — and the offline replay mirrors this branch). The Tobii with no file
/// applies the built-in Tobii Default.
pub(crate) fn load_profile_into(pipeline: &mut Pipeline, kind: SourceKind) {
    let path = ProfileConfig::active_config_path_for(kind);
    match ProfileConfig::load(&path) {
        Ok(cfg) => {
            apply_config(pipeline, &cfg);
            info!("loaded profile: {} ({})", cfg.name, kind.label());
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => match kind {
            SourceKind::Webcam => info!("no saved profile — using built-in defaults"),
            SourceKind::Tobii => {
                apply_config(pipeline, &ProfileConfig::tobii_default());
                info!("no saved Tobii profile — using built-in {}", ProfileConfig::TOBII_DEFAULT_NAME);
            }
        },
        Err(e) => {
            warn!(
                "profile at {} could not be read ({e}) — using built-in defaults",
                path.display()
            );
            if kind == SourceKind::Tobii {
                apply_config(pipeline, &ProfileConfig::tobii_default());
            }
        }
    }
}

/// A fresh pipeline carrying `kind`'s active profile.
pub(crate) fn pipeline_for(kind: SourceKind) -> Pipeline {
    let mut p = build_default_pipeline();
    load_profile_into(&mut p, kind);
    p
}

/// Swap the active source. The old one is dropped — threads joined, device
/// released — before `open` runs, so the same device is never open twice.
pub(crate) fn replace_source(
    slot: &mut Option<Box<dyn InputSource>>,
    open: impl FnOnce() -> Option<Box<dyn InputSource>>,
) {
    drop(slot.take());
    *slot = open();
}

/// Everything opening a source needs from the run loop.
// Each input feature reads a different subset; with one compiled out, its
// fields go unread.
#[cfg_attr(
    not(all(feature = "input-neuralnet", feature = "input-tobii")),
    allow(dead_code)
)]
pub(crate) struct OpenCtx<'a> {
    pub state: &'a Arc<EngineState>,
    pub models_dir: &'a Path,
    pub fov: f32,
    pub preview_shutdown: &'a mut Option<Arc<AtomicBool>>,
}

/// Open `source`. `None` = it could not be opened now (logged); the engine
/// keeps running sourceless and the user can switch again.
pub(crate) fn open_source(source: &HeadSource, ctx: OpenCtx<'_>) -> Option<Box<dyn InputSource>> {
    // No preview survives a source change: a Tobii has none, and a stale
    // webcam frame under the Tobii label would be wrong.
    if let Some(flag) = ctx.preview_shutdown.take() {
        flag.store(true, Ordering::Relaxed);
    }
    if let Ok(mut p) = ctx.state.preview.lock() {
        *p = (p.0 + 1, None);
    }
    if let Ok(mut h) = ctx.state.head_source.lock() {
        h.clone_from(source);
    }
    match source {
        HeadSource::Webcam { index, name } => {
            let sel = WebcamSelection { index: *index, name: name.clone() };
            let index = pick_webcam_index(&sel, &v4l2_devices());
            ctx.state.camera_index.store(index, Ordering::Relaxed);
            open_webcam(index, ctx)
        }
        HeadSource::Tobii { serial } => open_tobii(serial.clone(), ctx),
    }
}

#[cfg(feature = "input-neuralnet")]
fn open_webcam(index: u32, ctx: OpenCtx<'_>) -> Option<Box<dyn InputSource>> {
    use headtrack_input_neuralnet::NeuralNetInput;
    match NeuralNetInput::with_overlay(
        ctx.models_dir,
        index,
        ctx.fov,
        Arc::clone(&ctx.state.overlay_enabled),
    ) {
        Ok(s) => {
            info!("input source: {} (webcam {index})", s.name());
            crate::start_preview_writer(
                Arc::clone(s.preview_frame()),
                Arc::clone(&ctx.state.preview),
                ctx.preview_shutdown,
            );
            ctx.state.camera_connected.store(true, Ordering::Relaxed);
            Some(Box::new(s))
        }
        Err(e) => {
            // A missing model and a missing camera are different failures with
            // different fixes; reporting both as "no camera" sent people hunting
            // for a webcam problem.
            if let Some(m) = e.downcast_ref::<headtrack_input_neuralnet::MissingModel>() {
                tracing::error!("{m}");
                tracing::error!("tracking is disabled until the model is in place; switching cameras will not help");
            } else {
                warn!("no camera at index {index}: {e:#}");
                info!("engine running without a head source — pick a camera or the Tobii to connect");
            }
            None
        }
    }
}

#[cfg(not(feature = "input-neuralnet"))]
fn open_webcam(index: u32, _ctx: OpenCtx<'_>) -> Option<Box<dyn InputSource>> {
    info!("webcam {index} not opened: this build has no webcam support (input-neuralnet feature off)");
    None
}

#[cfg(feature = "input-tobii")]
fn open_tobii(serial: Option<String>, ctx: OpenCtx<'_>) -> Option<Box<dyn InputSource>> {
    use headtrack_input_tobii::TobiiInput;
    let root = ctx.models_dir.join("tobii");
    match TobiiInput::open(&root, serial, Arc::clone(&ctx.state.tobii)) {
        Ok(s) => {
            info!("input source: {}", s.name());
            ctx.state.camera_connected.store(true, Ordering::Relaxed);
            Some(Box::new(s))
        }
        Err(e) => {
            tracing::error!("Tobii head models could not be loaded: {e}");
            if let Ok(mut st) = ctx.state.tobii.lock() {
                *st = headtrack_core::TobiiStatus {
                    message: format!("Head models could not be loaded: {e}"),
                    ..Default::default()
                };
            }
            None
        }
    }
}

#[cfg(not(feature = "input-tobii"))]
fn open_tobii(_serial: Option<String>, _ctx: OpenCtx<'_>) -> Option<Box<dyn InputSource>> {
    warn!("the Tobii was selected, but this build has no Tobii support (input-tobii feature off)");
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use headtrack_core::RawPose;
    use std::sync::{atomic::AtomicUsize, Mutex};

    /// Tests that point XDG_CONFIG_HOME somewhere else hold this, so they never
    /// race each other (the variable is process-wide). Nothing here touches the
    /// real ~/.config.
    static ENV: Mutex<()> = Mutex::new(());

    struct ScratchConfig {
        dir: std::path::PathBuf,
        prev: Option<std::ffi::OsString>,
        _guard: std::sync::MutexGuard<'static, ()>,
    }
    impl ScratchConfig {
        fn new(tag: &str) -> Self {
            let guard = ENV.lock().unwrap_or_else(|e| e.into_inner());
            let dir = std::env::temp_dir().join(format!("htrs-engine-{tag}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            let prev = std::env::var_os("XDG_CONFIG_HOME");
            std::env::set_var("XDG_CONFIG_HOME", &dir);
            Self { dir, prev, _guard: guard }
        }
    }
    impl Drop for ScratchConfig {
        fn drop(&mut self) {
            match self.prev.take() {
                Some(v) => std::env::set_var("XDG_CONFIG_HOME", v),
                None => std::env::remove_var("XDG_CONFIG_HOME"),
            }
            let _ = std::fs::remove_dir_all(&self.dir);
        }
    }

    fn sel(index: u32, name: Option<&str>) -> WebcamSelection {
        WebcamSelection { index, name: name.map(str::to_string) }
    }

    #[test]
    fn webcam_name_beats_a_moved_index() {
        let devs = [
            (0, "Integrated Camera".to_string()),
            (1, "Integrated Camera".to_string()),
            (2, "Elgato Facecam MK.2".to_string()),
            (3, "Elgato Facecam MK.2".to_string()),
        ];
        // Saved at index 0, but the Facecam is now at 2/3 (replugged): name wins,
        // capture node (lowest) chosen.
        assert_eq!(pick_webcam_index(&sel(0, Some("Elgato Facecam MK.2")), &devs), 2);
        // Saved index is one of the name's nodes: keep it.
        assert_eq!(pick_webcam_index(&sel(3, Some("Elgato Facecam MK.2")), &devs), 3);
        // Name gone (unplugged): fall back to the saved index.
        assert_eq!(pick_webcam_index(&sel(1, Some("Logitech C920")), &devs), 1);
        // No name saved (older camera.toml): the index.
        assert_eq!(pick_webcam_index(&sel(2, None), &devs), 2);
        assert_eq!(pick_webcam_index(&sel(0, None), &[]), 0);
    }

    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static MAX_LIVE: AtomicUsize = AtomicUsize::new(0);
    struct Fake;
    impl Fake {
        fn open() -> Option<Box<dyn InputSource>> {
            let now = LIVE.fetch_add(1, Ordering::SeqCst) + 1;
            MAX_LIVE.fetch_max(now, Ordering::SeqCst);
            Some(Box::new(Fake))
        }
    }
    impl Drop for Fake {
        fn drop(&mut self) {
            LIVE.fetch_sub(1, Ordering::SeqCst);
        }
    }
    impl InputSource for Fake {
        fn name(&self) -> &str {
            "fake"
        }
        fn poll(&mut self) -> Option<RawPose> {
            None
        }
        fn latency_hint_us(&self) -> u32 {
            0
        }
    }

    #[test]
    fn a_device_is_never_open_twice_across_a_switch() {
        let mut slot: Option<Box<dyn InputSource>> = None;
        for _ in 0..5 {
            replace_source(&mut slot, Fake::open);
            assert_eq!(LIVE.load(Ordering::SeqCst), 1);
        }
        assert_eq!(MAX_LIVE.load(Ordering::SeqCst), 1, "old source must be dropped before the new opens");
        replace_source(&mut slot, || None);
        assert_eq!(LIVE.load(Ordering::SeqCst), 0);
    }

    /// KNOWN_BUGS #2: the camera choice survives a restart, by name and index,
    /// and a Tobii excursion returns to it.
    #[test]
    fn camera_selection_survives_a_restart() {
        let _cfg = ScratchConfig::new("persist");
        assert_eq!(
            startup_head_source(&CameraConfig::load(), None),
            HeadSource::Webcam { index: 0, name: None },
            "fresh install: webcam 0"
        );
        let cam = HeadSource::Webcam { index: 2, name: Some("Elgato Facecam MK.2".into()) };
        persist_head_source(&cam).unwrap();
        // "Restart": everything re-read from disk.
        assert_eq!(startup_head_source(&CameraConfig::load(), None), cam);
        // A one-off HEADTRACK_CAMERA override wins for that run only.
        assert_eq!(
            startup_head_source(&CameraConfig::load(), Some(4)),
            HeadSource::Webcam { index: 4, name: None }
        );
        assert_eq!(CameraConfig::load().head_source, cam, "the override is not persisted");
        // Tobii, then back: the webcam is remembered.
        persist_head_source(&HeadSource::Tobii { serial: None }).unwrap();
        let cfg = CameraConfig::load();
        assert_eq!(cfg.source_kind(), SourceKind::Tobii);
        assert_eq!(cfg.webcam_selection(), sel(2, Some("Elgato Facecam MK.2")));
    }

    #[cfg(feature = "input-tobii")]
    #[test]
    fn a_saved_tobii_source_starts_as_the_tobii() {
        let _cfg = ScratchConfig::new("tobii-start");
        persist_head_source(&HeadSource::Tobii { serial: Some("IS5FF".into()) }).unwrap();
        assert_eq!(
            startup_head_source(&CameraConfig::load(), None),
            HeadSource::Tobii { serial: Some("IS5FF".into()) }
        );
    }

    #[cfg(not(feature = "input-tobii"))]
    #[test]
    fn a_saved_tobii_source_falls_back_to_the_webcam_without_tobii_support() {
        let _cfg = ScratchConfig::new("tobii-off");
        persist_head_source(&HeadSource::Webcam { index: 1, name: None }).unwrap();
        persist_head_source(&HeadSource::Tobii { serial: None }).unwrap();
        assert_eq!(
            startup_head_source(&CameraConfig::load(), None),
            HeadSource::Webcam { index: 1, name: None }
        );
    }

    /// P08: webcam → Tobii → webcam loads each source's own active profile and
    /// leaves both files byte-identical.
    #[test]
    fn switching_sources_round_trips_both_profiles() {
        let _cfg = ScratchConfig::new("profiles");
        let mut webcam = ProfileConfig { name: "Mine".into(), ..Default::default() };
        webcam.deadzone[0] = 0.37;
        webcam.cross_axis.pitch_to_y = 1.9;
        let wpath = ProfileConfig::active_config_path_for(SourceKind::Webcam);
        webcam.save(&wpath).unwrap();
        let wbytes = std::fs::read(&wpath).unwrap();
        let tpath = ProfileConfig::active_config_path_for(SourceKind::Tobii);
        assert!(!tpath.exists());

        let expect_webcam = {
            let mut p = build_default_pipeline();
            apply_config(&mut p, &webcam);
            p.get_all_params()
        };
        let expect_tobii = {
            let mut p = build_default_pipeline();
            apply_config(&mut p, &ProfileConfig::tobii_default());
            p.get_all_params()
        };
        assert_ne!(expect_webcam, expect_tobii);

        assert_eq!(pipeline_for(SourceKind::Webcam).get_all_params(), expect_webcam);
        assert_eq!(pipeline_for(SourceKind::Tobii).get_all_params(), expect_tobii);
        assert_eq!(pipeline_for(SourceKind::Webcam).get_all_params(), expect_webcam);
        assert_eq!(std::fs::read(&wpath).unwrap(), wbytes, "webcam profile untouched");
        assert!(!tpath.exists(), "loading the built-in Tobii Default writes nothing");

        // With a saved Tobii profile, that one is used instead of the built-in.
        let mut tobii = ProfileConfig::tobii_default();
        tobii.curve[1].sensitivity = 0.6;
        tobii.save(&tpath).unwrap();
        let tbytes = std::fs::read(&tpath).unwrap();
        let got = pipeline_for(SourceKind::Tobii).get_all_params();
        assert!(got.contains(&("response-curve", "pitch.sensitivity", 0.6)));
        assert_eq!(pipeline_for(SourceKind::Webcam).get_all_params(), expect_webcam);
        assert_eq!(std::fs::read(&tpath).unwrap(), tbytes);
    }
}
