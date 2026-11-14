//! headtrack-rs engine — camera, ONNX inference, pipeline, outputs, IPC server.
//!
//! The engine owns all tracking infrastructure and exposes a simple API for
//! callers (GUI, headless daemon) to read live state and send commands.
//!
//! ```no_run
//! let engine = headtrack_engine::Engine::start().expect("engine start");
//! let state  = engine.state();   // Arc<EngineState> — clone and share freely
//! let cmd    = engine.cmd_sender();
//!
//! // Read pose directly from shared memory — no socket round-trip.
//! let pose = *state.latest_pose.lock().unwrap();
//!
//! // Send a command (non-blocking, safe from any thread).
//! cmd.send(headtrack_engine::EngineCmd::Recenter);
//! ```

use std::{
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use anyhow::Result;
use headtrack_core::{
    config::{CameraConfig, HeadSource},
    pipeline::{DistanceScale, StandoffTracker},
    InputSource, Pipeline, Pose, SourceKind, TobiiStatus,
};
#[cfg(feature = "input-neuralnet")]
use headtrack_input_neuralnet::PreviewFrame;
use headtrack_ipc::server::IpcServer;
#[cfg(feature = "output-simconnect")]
use headtrack_output_simconnect::SimConnectOutput;
#[cfg(feature = "output-wine")]
use headtrack_output_wine::WineShmWriter;
use tokio::sync::{broadcast, mpsc};
use tracing::{debug, info, warn};

/// Pseudo-stage name reserved for capture event markers. Matches no real stage
/// by design: `set marker <label> 1` changes nothing and exists only to write a
/// timestamped comment into the diagnostic CSVs.
const MARKER_STAGE: &str = "marker";

pub use headtrack_core::Pose as EnginePose;

mod actions;
mod controller;
mod global_shortcuts;
mod source;

/// Whether this build can open the Tobii Eye Tracker 5 (`input-tobii` feature).
/// The GUI greys the Tobii choice out when it is false.
pub const TOBII_SUPPORTED: bool = cfg!(feature = "input-tobii");
/// Whether this build can open a webcam (`input-neuralnet` feature).
pub const WEBCAM_SUPPORTED: bool = cfg!(feature = "input-neuralnet");

/// Build the log filter both binaries install.
///
/// `RUST_LOG` wins when set, `default` otherwise — but either way ONNX Runtime
/// is pinned to `warn` unless the caller named `ort` themselves.
///
/// ORT logs its arena bookkeeping at INFO: session options, every
/// `Reserving memory in BFCArena`, every `Extending BFCArena`. That is several
/// hundred lines per start-up, and `RUST_LOG=info` is exactly what someone
/// types when they need to read *our* start-up narrative — the levels described
/// in CLAUDE.md are a contract, and a third-party crate burying them breaks it.
/// `RUST_LOG=info,ort=info` still gets the full ORT firehose.
pub fn log_filter(default: &str) -> tracing_subscriber::EnvFilter {
    build_log_filter(std::env::var("RUST_LOG").ok().as_deref(), default)
}

fn build_log_filter(rust_log: Option<&str>, default: &str) -> tracing_subscriber::EnvFilter {
    use tracing_subscriber::EnvFilter;

    let requested = rust_log.map(str::trim).filter(|s| !s.is_empty());
    let filter = EnvFilter::new(requested.unwrap_or(default));

    // Only step aside if the caller addressed `ort` as a whole directive —
    // matching the bare substring would also fire on "report" or "output".
    let names_ort = requested.is_some_and(|s| {
        s.split(',')
            .any(|d| d.split('=').next().map(str::trim) == Some("ort"))
    });
    if names_ort {
        filter
    } else {
        filter.add_directive("ort=warn".parse().expect("static directive"))
    }
}

#[cfg(test)]
mod log_filter_tests {
    use super::build_log_filter;

    /// The case that prompted this: `RUST_LOG=info` used to hand ORT an INFO
    /// budget, and its arena logging buried the engine's start-up lines.
    #[test]
    fn ort_is_pinned_to_warn_unless_asked_for() {
        for input in [None, Some("info"), Some("warn,headtrack_engine=debug")] {
            let rendered = build_log_filter(input, "warn").to_string();
            assert!(
                rendered.contains("ort=warn"),
                "expected ort to be pinned for {input:?}, got {rendered}"
            );
        }
    }

    /// …but someone debugging ONNX itself must still be able to turn it up.
    #[test]
    fn an_explicit_ort_directive_is_left_alone() {
        let rendered = build_log_filter(Some("info,ort=debug"), "warn").to_string();
        assert!(rendered.contains("ort=debug"), "got {rendered}");
        assert!(!rendered.contains("ort=warn"), "got {rendered}");
    }

    /// A directive that merely contains the letters "ort" is not a claim on it.
    #[test]
    fn a_lookalike_target_does_not_count_as_asking_for_ort() {
        let rendered = build_log_filter(Some("headtrack_output_simconnect=debug"), "warn")
            .to_string();
        assert!(rendered.contains("ort=warn"), "got {rendered}");
    }
}

/// Shared camera preview: `(seq_counter, latest_jpeg)`.
///
/// The engine's preview writer increments `seq` on each new frame, letting
/// the GUI skip decoding when nothing changed. Compatible with egui's
/// per-repaint polling pattern.
pub type EnginePreview = Arc<Mutex<(u64, Option<Arc<Vec<u8>>>)>>;

// ── Public command enum ──────────────────────────────────────────────────────

/// Commands that can be sent to the engine pipeline from any caller.
#[derive(Debug)]
pub enum EngineCmd {
    Recenter,
    SetParam { stage: String, param: String, value: f32 },
    /// Make webcam `index` the head source (from the Tobii too) and persist it.
    SwitchCamera { index: u32 },
    /// Change the head source: the old source is closed and released first,
    /// the pipeline and every per-source estimate is reset, the new source's
    /// active profile is loaded, and the choice is persisted in `camera.toml`.
    SwitchSource { source: HeadSource },
    /// Switch to a source *kind*, letting the engine pick the device: the
    /// remembered webcam (`camera.toml` `last_webcam`), or the Tobii (the saved
    /// serial if the Tobii was the last source, else the single connected one).
    SwitchSourceKind { kind: SourceKind },
    SetFov { degrees: f32 },
    /// Change capture resolution and restart the camera.  Resolution is
    /// persisted to `camera.toml` so it survives restarts.
    SetResolution { width: u32, height: u32 },
    Overlay { enabled: bool },
    /// Begin writing a diagnostic CSV to `path`. Replaces any active recording.
    StartRecording { path: PathBuf, size_limit_mb: u32 },
    /// Stop the current recording and flush the file.
    StopRecording,
    /// Enable or disable SimConnect CameraSet streaming.
    /// When disabled, MSFS releases camera ownership (allows view switching).
    #[cfg(feature = "output-simconnect")]
    SimConnectEnabled(bool),
    /// Set the MSFS camera output smoothing LERP speed (higher = snappier).
    /// Output-side only; does not touch the shared pipeline.
    #[cfg(feature = "output-simconnect")]
    SetSimConnectSmoothing(f32),
    /// Star Citizen (Wine SHM) output smoothing speed. Output-side only.
    SetScSmoothing(f32),
    /// Flip SimConnect streaming (used by stateless callers — the command
    /// socket and global shortcuts — that can't know the current state).
    #[cfg(feature = "output-simconnect")]
    SimConnectToggleEnabled,
    /// Nudge one MSFS seat-offset axis (0=x, 1=y, 2=z) by `delta_m` metres.
    /// The adapter clamps and persists per aircraft; callers stay stateless.
    #[cfg(feature = "output-simconnect")]
    MsfsSeatNudge { axis: u8, delta_m: f32 },
    /// Zero the current aircraft's MSFS seat offset (persisted).
    #[cfg(feature = "output-simconnect")]
    MsfsSeatReset,
    /// Re-open the desktop's global-shortcuts binding dialog (XDG portal), so
    /// bindings can be changed after the first-launch registration.
    RebindGlobalShortcuts,
    /// Start capturing a controller button for `action` (next fresh press on
    /// any joystick-class device binds; parked switches are never accepted).
    ControllerBindStart { action: String },
    /// Cancel an in-progress controller capture.
    ControllerBindCancel,
    /// Remove the controller binding for `action`.
    ControllerBindClear { action: String },
}

// ── Recording status ─────────────────────────────────────────────────────────

/// Status of the in-GUI diagnostic recorder, stored as a `u8` in `EngineState`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecordingStatus {
    Idle         = 0,
    Active       = 1,
    LimitReached = 2,
    /// `StartRecording` failed (bad path, permissions, disk full, etc.).
    /// The human-readable cause is in `EngineState::recording_error`.
    Error        = 3,
}

impl RecordingStatus {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Active,
            2 => Self::LimitReached,
            3 => Self::Error,
            _ => Self::Idle,
        }
    }
}

// ── Shared state ─────────────────────────────────────────────────────────────

/// Live state shared between the engine and its callers.
///
/// All fields are independently thread-safe; callers can read without
/// holding a global lock and without any socket round-trip.
pub struct EngineState {
    pub latest_pose: Mutex<Pose>,
    pub fps: Mutex<f32>,
    /// The active head source is open (webcam capturing, or the Tobii source
    /// running — whose own device state is in `tobii`).
    pub camera_connected: AtomicBool,
    /// V4L2 index of the webcam (the active one, or the last one opened).
    pub camera_index: AtomicU32,
    /// The active head source as selected (persisted in `camera.toml`).
    pub head_source: Mutex<HeadSource>,
    /// Tobii device + tracking state for the Tobii tab. Written by the Tobii
    /// source while it runs; `TobiiStatus::default()` otherwise. Gaze lives
    /// only here and never reaches an output.
    pub tobii: Arc<Mutex<TobiiStatus>>,
    /// Stored as f32 bits. Updated whenever FOV changes (camera switch with
    /// auto-detect, or explicit SetFov).
    pub current_fov: AtomicU32,
    /// The engine writer updates this at ~7 Hz; callers check `seq` to detect
    /// new frames before decoding.
    pub preview: EnginePreview,
    pub overlay_enabled: Arc<AtomicBool>,
    pub recording_status: AtomicU8,
    pub recording_bytes: AtomicU64,
    /// Human-readable error message when `recording_status == Error`.
    /// Cleared whenever recording starts successfully or stops.
    pub recording_error: Mutex<Option<String>>,
    /// MSFS mirrors (updated once per run-loop tick from the SimConnect
    /// adapter; zeros/false/"" when the feature is off or MSFS absent).
    /// Seat offset in metres, stored as f32 bits per axis.
    pub msfs_seat: [AtomicU32; 3],
    /// Current aircraft TITLE ("" until identified).
    pub msfs_title: Mutex<String>,
    /// Whether CameraSet streaming is enabled (the GUI checkbox mirrors this,
    /// so socket-driven toggles show up in the UI).
    pub msfs_enabled: AtomicBool,
    /// Whether the SimConnect TCP session is up.
    pub msfs_connected: AtomicBool,
    /// Desktop global shortcuts registered via the XDG portal:
    /// `None` = portal unavailable (or not yet registered); `Some(list)` =
    /// bound actions as `(description, trigger)` — trigger is "" until the
    /// user assigns a key. Kept fresh via the portal's ShortcutsChanged signal.
    pub global_shortcuts: Mutex<Option<Vec<(String, String)>>>,
    /// Controller/HOTAS bindings view for the Keybinds tab:
    /// `(action id, description, bound button label)` in definition order.
    pub controller_bindings: Mutex<Vec<(String, String, Option<String>)>>,
    /// Action id currently capturing a controller button (`None` = idle).
    pub controller_capture: Mutex<Option<String>>,
}

// ── Command sender ───────────────────────────────────────────────────────────

/// Cheaply cloneable handle for sending commands to the engine.
///
/// Safe to hold in GUI widgets and call from `egui::App::update()`.
#[derive(Clone)]
pub struct EngineCmdSender(mpsc::Sender<EngineCmd>);

impl EngineCmdSender {
    /// Send a command. Non-blocking — drops the command if the channel is full
    /// (capacity 64; this should never happen under normal use).
    pub fn send(&self, cmd: EngineCmd) {
        if let Err(e) = self.0.try_send(cmd) {
            tracing::warn!("engine command dropped: {e}");
        }
    }
}

// ── Engine ───────────────────────────────────────────────────────────────────

/// The headtrack-rs engine.
///
/// Holds the tokio runtime and all running tasks. Dropping this value stops
/// the engine cleanly (all tasks are cancelled, the tokio runtime is shut down).
pub struct Engine {
    state:   Arc<EngineState>,
    cmd_tx:  mpsc::Sender<EngineCmd>,
    runtime: Option<tokio::runtime::Runtime>,
}

impl Drop for Engine {
    fn drop(&mut self) {
        if let Some(rt) = self.runtime.take() {
            // Signal SimConnect to send a zero-pose reset before shutdown.
            // This returns MSFS camera to neutral rather than freezing at the last head pose.
            #[cfg(feature = "output-simconnect")]
            {
                let _ = self.cmd_tx.try_send(EngineCmd::SimConnectEnabled(false));
                // Give the run loop and SimConnect task time to process the reset
                // (300ms ≈ 9 sim frames at 30 Hz — enough to guarantee one zero CameraSet).
                rt.block_on(async {
                    tokio::time::sleep(std::time::Duration::from_millis(300)).await;
                });
            }
            rt.shutdown_background();
        }
    }
}

impl Engine {
    /// Start the engine synchronously.
    ///
    /// Creates its own multi-thread tokio runtime. Blocks until:
    /// - The IPC pose socket is bound (X-Plane plugin / tester can connect)
    /// - The camera is either opened or confirmed absent (graceful cameraless start)
    ///
    /// Returns `Err` only on hard failures (e.g. can't bind the IPC socket).
    pub fn start() -> Result<Self> {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()?;

        let state = Arc::new(EngineState {
            latest_pose:      Mutex::new(Pose::zero(0)),
            fps:              Mutex::new(0.0),
            camera_connected: AtomicBool::new(false),
            camera_index:     AtomicU32::new(0),
            head_source:      Mutex::new(HeadSource::default()),
            tobii:            Arc::new(Mutex::new(TobiiStatus::default())),
            current_fov:      AtomicU32::new(CameraConfig::load().fov_diag_deg.to_bits()),
            preview:          Arc::new(Mutex::new((0, None))),
            overlay_enabled:  Arc::new(AtomicBool::new(false)),
            recording_status: AtomicU8::new(0),
            recording_bytes:  AtomicU64::new(0),
            recording_error:  Mutex::new(None),
            msfs_seat:        std::array::from_fn(|_| AtomicU32::new(0)),
            msfs_title:       Mutex::new(String::new()),
            msfs_enabled:     AtomicBool::new(true),
            msfs_connected:   AtomicBool::new(false),
            global_shortcuts: Mutex::new(None),
            controller_bindings: Mutex::new(Vec::new()),
            controller_capture:  Mutex::new(None),
        });

        let (cmd_tx, cmd_rx) = mpsc::channel::<EngineCmd>(64);

        // Run async init on the new runtime (binds IPC socket, opens camera,
        // spawns long-lived tasks) then return so the caller can proceed.
        rt.block_on(engine_init(Arc::clone(&state), cmd_tx.clone(), cmd_rx))?;

        Ok(Engine { state, cmd_tx, runtime: Some(rt) })
    }

    pub fn state(&self) -> Arc<EngineState> {
        Arc::clone(&self.state)
    }

    pub fn cmd_sender(&self) -> EngineCmdSender {
        EngineCmdSender(self.cmd_tx.clone())
    }
}

// ── Private: init and task spawning ─────────────────────────────────────────

/// Async initialisation: bind IPC socket, load config, open camera, spawn tasks.
///
/// Returns after all tasks are spawned — the tasks themselves run indefinitely
/// on the engine's tokio runtime.
async fn engine_init(
    state:  Arc<EngineState>,
    cmd_tx: mpsc::Sender<EngineCmd>,  // for the socket cmd_listener to forward into
    cmd_rx: mpsc::Receiver<EngineCmd>,
) -> Result<()> {
    // 1. Bind the IPC pose socket. Fail fast if this doesn't work.
    let socket_path = headtrack_ipc::socket_path();
    info!("IPC socket: {}", socket_path.display());
    let (server, pose_tx) = IpcServer::bind(&socket_path).await?;
    tokio::spawn(async move {
        if let Err(e) = server.serve().await {
            warn!("IPC server error: {e:#}");
        }
    });

    // 2. Decide the head source, then build its pipeline with its own active
    //    profile (active.toml for the webcam, active-tobii.toml for the Tobii;
    //    P05 §7, P08). `HEADTRACK_CAMERA` overrides for this run only.
    let mut camera_cfg = CameraConfig::load();
    let env_camera: Option<u32> = std::env::var("HEADTRACK_CAMERA")
        .ok()
        .and_then(|v| v.parse().ok());
    let head = source::startup_head_source(&camera_cfg, env_camera);
    let kind = head.kind();
    let pipeline = source::pipeline_for(kind);
    info!(
        "pipeline: {}",
        pipeline.stage_names().collect::<Vec<_>>().join(" → ")
    );

    // 3. Auto-detect FOV for a known webcam model.
    if let HeadSource::Webcam { index, name } = &head {
        let sel = headtrack_core::config::WebcamSelection { index: *index, name: name.clone() };
        let index = source::pick_webcam_index(&sel, &source::v4l2_devices());
        if let Some(name) = device_name(index) {
            if let Some(fov) = lookup_fov(&name) {
                if (fov - camera_cfg.fov_diag_deg).abs() > 0.1 {
                    info!("auto-detected FOV {fov}° for '{name}' (was {}°)", camera_cfg.fov_diag_deg);
                    camera_cfg.fov_diag_deg = fov;
                    let _ = camera_cfg.save();
                }
            }
        }
    }

    // 4. Open the source. A sourceless start is OK (no camera, no Tobii).
    let models_dir = resolve_models_dir();
    info!("models directory: {}", models_dir.display());
    let mut preview_shutdown: Option<Arc<AtomicBool>> = None;
    let source = source::open_source(
        &head,
        source::OpenCtx {
            state: &state,
            models_dir: &models_dir,
            fov: camera_cfg.fov_diag_deg,
            preview_shutdown: &mut preview_shutdown,
        },
    );

    // 5. Spawn the command socket listener (for external tools).
    let cmd_path = headtrack_ipc::cmd_socket_path();
    info!("command socket: {}", cmd_path.display());
    let cmd_tx_for_listener = cmd_tx.clone();
    let overlay_for_listener = Arc::clone(&state.overlay_enabled);
    tokio::spawn(async move {
        if let Err(e) = cmd_listener(cmd_path, cmd_tx_for_listener, overlay_for_listener).await {
            warn!("command listener error: {e:#}");
        }
    });

    // 5b. Register desktop-global shortcuts with the XDG portal (recenter,
    // MSFS seat nudges, camera toggle — they fire while the sim has focus).
    // Degrades to a single info log when the portal is absent. `rebind_tx`
    // lets the GUI re-open the desktop's binding dialog later.
    let (rebind_tx, rebind_rx) = tokio::sync::mpsc::unbounded_channel();
    tokio::spawn(global_shortcuts::run(cmd_tx.clone(), Arc::clone(&state), rebind_rx));

    // 5c. Controller/HOTAS button bindings — the in-app, focus-independent
    // hotkey tier (evdev listener thread; bind UI in the Keybinds tab).
    let (ctrl_tx, ctrl_rx) = std::sync::mpsc::channel::<controller::ControllerCmd>();
    controller::spawn(cmd_tx.clone(), Arc::clone(&state), ctrl_rx);

    // 6. Build the optional output adapters.
    #[cfg(feature = "output-wine")]
    let wine_shm = match WineShmWriter::open() {
        Ok(mut w) => {
            w.set_smoothing(camera_cfg.sc_output_smoothing);
            info!(
                "Wine SHM bridge ready (output smoothing {:.1})",
                camera_cfg.sc_output_smoothing
            );
            Some(w)
        }
        Err(e) => { warn!("Wine SHM unavailable: {e:#}"); None }
    };
    #[cfg(feature = "output-simconnect")]
    let simconnect = match SimConnectOutput::open(5557) {
        Ok(s) => {
            s.set_smoothing(camera_cfg.msfs_camera_smoothing);
            info!("SimConnect output ready (127.0.0.1:5557)");
            Some(s)
        }
        Err(e) => { warn!("SimConnect output unavailable: {e:#}"); None }
    };
    let outputs = EngineOutputs {
        #[cfg(feature = "output-wine")]
        wine_shm,
        #[cfg(feature = "output-simconnect")]
        simconnect,
    };

    // 7. Spawn the run loop — runs forever.
    let ctx = RunLoopCtx {
        state,
        source,
        head,
        current_fov: camera_cfg.fov_diag_deg,
        models_dir,
        pipeline,
        recorder: None,
        // Raw pre-pipeline pose dump (HEADTRACK_DUMP_RAW) — feeds the offline
        // workbench. Opened here so a failure is reported before the loop runs.
        raw_recorder: if dump_raw_enabled() {
            match RawPoseRecorder::open() {
                Ok(r) => Some(r),
                Err(e) => {
                    warn!("HEADTRACK_DUMP_RAW set but raw dump could not start: {e}");
                    None
                }
            }
        } else {
            None
        },
        preview_shutdown,
        last_raw_ts: None,
        last_pose: Pose::zero(0),
        standoff: StandoffTracker::new(),
        frames_since_input: 0,
        source_fresh: false,
    };

    tokio::spawn(run_loop(
        ctx,
        cmd_rx,
        pose_tx,
        outputs,
        rebind_tx,
        ctrl_tx,
    ));

    Ok(())
}

// ── Private: diagnostic recorder ─────────────────────────────────────────────

struct DiagRecorder {
    writer:           std::io::BufWriter<std::fs::File>,
    start:            Instant,
    bytes_written:    u64,
    size_limit_bytes: u64,
    line_buf:         String,
}

impl DiagRecorder {
    fn open(path: &std::path::Path, size_limit_mb: u32, pipeline: &Pipeline) -> std::io::Result<Self> {
        use std::io::Write;
        // Best-effort: create the destination directory so users can point at
        // a freshly-named folder without pre-creating it. Permission/disk
        // failures still surface from `File::create` below.
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let file = std::fs::File::create(path)?;
        let mut writer = std::io::BufWriter::new(file);

        // Header: fixed columns + all pipeline params
        let mut header = "time_s,yaw,pitch,roll,x,y,z,fps".to_string();
        for (stage, param, _) in pipeline.get_all_params() {
            header.push_str(&format!(",\"{}:{}\"", stage, param));
        }
        header.push('\n');

        const BANNER: &str = "# headtrack-rs diagnostic recording v1\n";
        // Which detector produced this recording. Without it, two captures a
        // week apart are indistinguishable, which defeats the purpose of
        // recording them while two detectors exist.
        let provenance = format!("# detector={}\n", active_detector());
        writer.write_all(BANNER.as_bytes())?;
        writer.write_all(provenance.as_bytes())?;
        writer.write_all(header.as_bytes())?;

        Ok(Self {
            writer,
            start: Instant::now(),
            bytes_written: (header.len() + BANNER.len() + provenance.len()) as u64,
            size_limit_bytes: size_limit_mb as u64 * 1024 * 1024,
            line_buf: String::with_capacity(256),
        })
    }

    /// Write a pose data row. Returns `true` if the size limit is now reached.
    fn write_pose(&mut self, pose: &Pose, fps: f32, pipeline: &Pipeline) -> bool {
        use std::fmt::Write as FmtWrite;
        use std::io::Write;
        let t = self.start.elapsed().as_secs_f64();
        self.line_buf.clear();
        let _ = write!(self.line_buf,
            "{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.1}",
            t, pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z, fps
        );
        for (_, _, value) in pipeline.get_all_params() {
            let _ = write!(self.line_buf, ",{value}");
        }
        self.line_buf.push('\n');

        self.bytes_written += self.line_buf.len() as u64;
        let _ = self.writer.write_all(self.line_buf.as_bytes());
        self.bytes_written >= self.size_limit_bytes
    }

    /// Write an event comment row. Returns `true` if the size limit is now reached.
    fn write_event(&mut self, msg: &str) -> bool {
        use std::io::Write;
        let t = self.start.elapsed().as_secs_f64();
        let line = format!("# {t:.4} {msg}\n");
        self.bytes_written += line.len() as u64;
        let _ = self.writer.write_all(line.as_bytes());
        self.bytes_written >= self.size_limit_bytes
    }

    fn flush(&mut self) {
        use std::io::Write;
        let _ = self.writer.flush();
    }
}

// ── Private: raw-pose dump (HEADTRACK_DUMP_RAW) ──────────────────────────────

/// Name of the active face detector, for stamping into capture files.
///
/// The neural-net input is an optional feature, so this reports `"n/a"` rather
/// than failing to build in a configuration that has no detector at all.
fn active_detector() -> &'static str {
    #[cfg(feature = "input-neuralnet")]
    {
        headtrack_input_neuralnet::active_detector_name()
    }
    #[cfg(not(feature = "input-neuralnet"))]
    {
        "n/a"
    }
}

/// True when `HEADTRACK_DUMP_RAW` is set to a truthy value.
fn dump_raw_enabled() -> bool {
    headtrack_core::env_flag("HEADTRACK_DUMP_RAW")
}

/// Captures the **pre-pipeline** RawPose each frame to a CSV whose schema matches
/// the offline replay tool (`headtrack-core --example replay`) and
/// `scripts/raw_from_calib.py`. Enabled by `HEADTRACK_DUMP_RAW=1`; the path can
/// be overridden with `HEADTRACK_DUMP_RAW_PATH` (default `/tmp/headtrack_raw_pose.csv`).
///
/// Synchronous like [`DiagRecorder`] — both run inside the loop's `block_in_place`
/// section, so file I/O doesn't starve the tokio runtime.
struct RawPoseRecorder {
    writer:     std::io::BufWriter<std::fs::File>,
    line_buf:   String,
    /// timestamp of the last row written — anchors event comments to the pose
    /// timeline (a marker lands between the row with this ts and the next).
    last_ts_us: u64,
}

impl RawPoseRecorder {
    fn open() -> std::io::Result<Self> {
        use std::io::Write;
        let path = std::env::var("HEADTRACK_DUMP_RAW_PATH")
            .unwrap_or_else(|_| "/tmp/headtrack_raw_pose.csv".to_string());
        let file = std::fs::File::create(&path)?;
        let mut writer = std::io::BufWriter::new(file);
        // Stamp the detector so the capture is self-describing.
        //
        // This goes AFTER the header, not before it, and that ordering is
        // load-bearing: every consumer of this format takes the *first* line as
        // the column names. `replay.rs` skips `#` lines only inside its data
        // loop, and `csv.DictReader` would silently adopt a leading comment as
        // the header. Placed here it is just another mid-file comment, which
        // the event markers written by `set marker tick 1` already are, so
        // every existing tool handles it unchanged.
        let detector = active_detector();
        writer.write_all(b"timestamp_us,dt_s,yaw,pitch,roll,x,y,z\n")?;
        writeln!(writer, "# detector={detector}")?;
        info!("HEADTRACK_DUMP_RAW=1 — raw pre-pipeline pose log ({detector}) -> {path}");
        Ok(Self { writer, line_buf: String::with_capacity(96), last_ts_us: 0 })
    }

    fn write_raw(&mut self, pose: &Pose, dt: f32) {
        use std::fmt::Write as FmtWrite;
        use std::io::Write;
        self.line_buf.clear();
        let _ = writeln!(self.line_buf,
            "{},{:.6},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
            pose.timestamp_us, dt, pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z
        );
        let _ = self.writer.write_all(self.line_buf.as_bytes());
        self.last_ts_us = pose.timestamp_us;
        // Diagnostic mode only — flush each line so a captured session survives an
        // abrupt app exit (the run loop has no explicit shutdown hook).
        let _ = self.writer.flush();
    }

    /// Write an event comment row (`# ts_us=<t> <msg>`). The tick-marker
    /// command (`set marker tick 1` on the command socket) lands here so
    /// human-marked events reach the raw CSV, not just the DiagRecorder /
    /// tester dumps. Consumers skip `#` lines (replay.rs; the bake-off's
    /// DictReader drops them on the float() parse).
    fn write_event(&mut self, msg: &str) {
        use std::io::Write;
        let _ = writeln!(self.writer, "# ts_us={} {msg}", self.last_ts_us);
        let _ = self.writer.flush();
    }
}

// ── Private: run loop ────────────────────────────────────────────────────────

const POLL_INTERVAL: Duration = Duration::from_millis(4); // ~250 Hz poll ceiling

/// Gated optional output adapters, bundled to simplify the `run_loop` parameter list.
struct EngineOutputs {
    #[cfg(feature = "output-wine")]
    wine_shm: Option<WineShmWriter>,
    #[cfg(feature = "output-simconnect")]
    simconnect: Option<SimConnectOutput>,
}


/// Everything the run loop threads through `dispatch_cmd`.
///
/// `run_loop` used to take 13 positional parameters and `dispatch_cmd` 10 of
/// its own, hand-forwarded at the call site — an easy place to pass the wrong
/// `&mut`, behind a `#[allow(clippy::too_many_arguments)]` that had stopped
/// meaning anything. Grouping them makes the forwarding a single `&mut ctx`.
struct RunLoopCtx {
    state:            Arc<EngineState>,
    source:           Option<Box<dyn InputSource>>,
    /// The selected head source (even when it failed to open).
    head:             HeadSource,
    current_fov:      f32,
    models_dir:       PathBuf,
    pipeline:         Pipeline,
    recorder:         Option<DiagRecorder>,
    raw_recorder:     Option<RawPoseRecorder>,
    preview_shutdown: Option<Arc<AtomicBool>>,
    // Per-source estimates — all reset on a source change (`reset_source_state`).
    /// Timestamp of the previous frame, for source-clock dt.
    last_raw_ts:        Option<u64>,
    last_pose:          Pose,
    standoff:           StandoffTracker,
    frames_since_input: u32,
    /// Set by `reset_source_state`; the run loop takes it to end any stall
    /// outage (the watchdog's clock and its once-per-outage WARN).
    source_fresh:       bool,
}

impl RunLoopCtx {
    fn kind(&self) -> SourceKind {
        self.head.kind()
    }
}

// `outputs` is genuinely unused when every output feature is compiled out;
// nothing else in this function is.
#[cfg_attr(
    not(any(feature = "output-wine", feature = "output-simconnect")),
    allow(unused_variables, unused_mut)
)]
async fn run_loop(
    mut ctx:      RunLoopCtx,
    mut cmd_rx:   mpsc::Receiver<EngineCmd>,
    pose_tx:      broadcast::Sender<Pose>,
    mut outputs:  EngineOutputs,
    rebind_tx:    tokio::sync::mpsc::UnboundedSender<()>,
    ctrl_tx:      std::sync::mpsc::Sender<controller::ControllerCmd>,
) {
    let mut interval = tokio::time::interval(POLL_INTERVAL);
    let mut last_tick = Instant::now();
    let mut fps_frame_count: u32 = 0;
    let mut fps_timer = Instant::now();
    let mut current_fps: f32 = 0.0;
    let mut last_pose_time = Instant::now();
    let mut stall_watch = StallWatch::default();
    #[cfg(feature = "output-simconnect")]
    // Last aircraft title mirrored into EngineState — compare-before-lock.
    #[cfg(feature = "output-simconnect")]
    let mut last_msfs_title = String::new();

    loop {
        interval.tick().await;

        while let Ok(cmd) = cmd_rx.try_recv() {
            #[cfg(feature = "output-simconnect")]
            if let EngineCmd::SimConnectEnabled(enabled) = cmd {
                if let Some(ref sc) = outputs.simconnect {
                    sc.set_enabled(enabled);
                    info!("SimConnect streaming {}", if enabled { "enabled" } else { "disabled" });
                }
                continue;
            }
            #[cfg(feature = "output-simconnect")]
            if let EngineCmd::SetSimConnectSmoothing(speed) = cmd {
                if let Some(ref sc) = outputs.simconnect {
                    sc.set_smoothing(speed);
                }
                continue;
            }
            #[cfg(feature = "output-wine")]
            if let EngineCmd::SetScSmoothing(speed) = cmd {
                if let Some(ref mut w) = outputs.wine_shm {
                    w.set_smoothing(speed);
                }
                continue;
            }
            #[cfg(feature = "output-simconnect")]
            if let EngineCmd::SimConnectToggleEnabled = cmd {
                if let Some(ref sc) = outputs.simconnect {
                    let now_enabled = sc.toggle_enabled();
                    info!(
                        "SimConnect streaming {} (toggle)",
                        if now_enabled { "enabled" } else { "disabled" }
                    );
                }
                continue;
            }
            #[cfg(feature = "output-simconnect")]
            if let EngineCmd::MsfsSeatNudge { axis, delta_m } = cmd {
                if let Some(ref sc) = outputs.simconnect {
                    sc.nudge_seat(axis as usize, delta_m);
                }
                continue;
            }
            #[cfg(feature = "output-simconnect")]
            if let EngineCmd::MsfsSeatReset = cmd {
                if let Some(ref sc) = outputs.simconnect {
                    sc.reset_seat();
                }
                continue;
            }
            if let EngineCmd::RebindGlobalShortcuts = cmd {
                let _ = rebind_tx.send(());
                continue;
            }
            if let EngineCmd::ControllerBindStart { ref action } = cmd {
                let _ = ctrl_tx.send(controller::ControllerCmd::StartCapture {
                    action: action.clone(),
                });
                continue;
            }
            if let EngineCmd::ControllerBindCancel = cmd {
                let _ = ctrl_tx.send(controller::ControllerCmd::CancelCapture);
                continue;
            }
            if let EngineCmd::ControllerBindClear { ref action } = cmd {
                let _ = ctrl_tx.send(controller::ControllerCmd::Clear {
                    action: action.clone(),
                });
                continue;
            }
            // The SC output smoother holds its own accumulator. Left alone, a
            // recenter would glide from the pre-recenter offset down to the new
            // zero — showing the user their old offset draining away instead of
            // a snap. Peeked here rather than in dispatch_cmd because that is
            // where `outputs` is in scope.
            #[cfg(feature = "output-wine")]
            if matches!(cmd, EngineCmd::Recenter) {
                if let Some(ref mut w) = outputs.wine_shm {
                    w.reset_smoothing();
                }
            }
            dispatch_cmd(cmd, &mut ctx).await;
        }
        // Mirror live MSFS adapter ctx.state into EngineState for the GUI
        // (seat offset, aircraft title, streaming enabled, connected).
        #[cfg(feature = "output-simconnect")]
        if let Some(ref sc) = outputs.simconnect {
            let seat = sc.seat_offset();
            for (slot, v) in ctx.state.msfs_seat.iter().zip(seat) {
                slot.store(v.to_bits(), Ordering::Relaxed);
            }
            ctx.state.msfs_enabled.store(sc.is_enabled(), Ordering::Relaxed);
            ctx.state.msfs_connected.store(sc.is_connected(), Ordering::Relaxed);
            let title = sc.current_title();
            if title != last_msfs_title {
                if let Ok(mut t) = ctx.state.msfs_title.lock() {
                    t.clone_from(&title);
                }
                last_msfs_title = title;
            }
        }

        // If the camera was just switched out, reset the stall watchdog so we
        // don't false-alarm during the 500ms V4L2 release + reopen window. A
        // completed source swap ends any outage too (`reset_source_state` sets
        // `source_fresh`; the swap can finish without this loop seeing `None`).
        if ctx.source.is_none() || std::mem::take(&mut ctx.source_fresh) {
            last_pose_time = Instant::now();
            stall_watch.reset();
        }

        let kind = ctx.kind();
        let Some(ref mut src) = ctx.source else { continue };

        // All synchronous work (pipeline math, mutex writes, file I/O, SHM/TCP
        // output writes) is wrapped in block_in_place so the tokio scheduler can
        // move this task off a worker thread during the blocking section, preventing
        // starvation of the 2-worker runtime on slow I/O.
        tokio::task::block_in_place(|| {
            if let Some(raw) = src.poll() {
                let now = Instant::now();
                // dt drives every dt-scaled filter in the pipeline (Accela
                // alpha, Kalman Q). Derive it from the frame's own timestamp:
                // the poll loop only *notices* frames on a 4 ms grid, so the
                // wall clock between polls swings ±40% frame-to-frame on a
                // perfectly steady camera. The camera clock is only rejected
                // when it's unusable (first frame, camera switch resetting the
                // epoch, stall > 0.5 s) — then fall back to the wall clock.
                let wall_dt = now.duration_since(last_tick).as_secs_f32();
                let ts_dt = ctx.last_raw_ts
                    .and_then(|prev| raw.timestamp_us.checked_sub(prev))
                    .map(|d| d as f32 / 1_000_000.0)
                    .filter(|d| *d > 0.0 && *d < 0.5);
                let dt = ts_dt.unwrap_or(wall_dt).max(0.001);
                ctx.last_raw_ts = Some(raw.timestamp_us);
                last_tick = now;
                last_pose_time = now;
                stall_watch.reset();

                // Tap the RawPose *before* any pipeline processing for offline tuning.
                if let Some(ref mut rr) = ctx.raw_recorder {
                    rr.write_raw(&raw, dt);
                }

                // Keep the translation tuning matched to how far the user is
                // actually sitting. Thresholds are authored at a reference
                // standoff; depth noise grows with its square and lateral noise
                // linearly, so a fixed tuning silently detunes as the chair
                // moves. See `DistanceScale`.
                // The Tobii's Z is held at 0, which is not a standoff (P05 §6).
                if kind.observes_standoff() {
                    if let Some(scale) = ctx.standoff.observe(raw.z, dt) {
                        ctx.pipeline.set_distance_scale(scale);
                        debug!(
                            "distance scale: standoff {:.0} mm -> lateral ×{:.2}, depth ×{:.2}",
                            ctx.standoff.current_mm(), scale.lateral, scale.depth
                        );
                    }
                }

                let processed = ctx.pipeline.process(raw, dt);
                ctx.last_pose = processed;
                ctx.frames_since_input = 0;

                // Update shared ctx.state for the GUI.
                if let Ok(mut p) = ctx.state.latest_pose.lock() {
                    *p = processed;
                }

                // FPS counter — updated once per second.
                fps_frame_count += 1;
                let elapsed = fps_timer.elapsed().as_secs_f32();
                if elapsed >= 1.0 {
                    current_fps = fps_frame_count as f32 / elapsed;
                    if let Ok(mut f) = ctx.state.fps.lock() {
                        *f = current_fps;
                    }
                    fps_frame_count = 0;
                    fps_timer = Instant::now();
                }

                // Diagnostic ctx.recorder — write pose row each frame.
                if let Some(ref mut rec) = ctx.recorder {
                    let limit = rec.write_pose(&processed, current_fps, &ctx.pipeline);
                    ctx.state.recording_bytes.store(rec.bytes_written, Ordering::Relaxed);
                    if limit {
                        rec.write_event("RECORDING_STOPPED: size limit reached");
                        rec.flush();
                        ctx.recorder = None;
                        ctx.state.recording_status.store(RecordingStatus::LimitReached as u8, Ordering::Relaxed);
                    }
                }

                // Broadcast to external IPC clients (X-Plane plugin, tester).
                let _ = pose_tx.send(processed);

                // Write to Wine SHM bridge (SC native Wine via Z:\dev\shm).
                #[cfg(feature = "output-wine")]
                if let Some(ref mut shm) = outputs.wine_shm {
                    shm.write(&processed);
                }

                // Write to SimConnect (MSFS 2024 via TCP Camera API).
                #[cfg(feature = "output-simconnect")]
                if let Some(ref sc) = outputs.simconnect {
                    sc.write(&processed);
                }
            } else {
                ctx.frames_since_input = ctx.frames_since_input.saturating_add(1);
                // During brief frame gaps keep Wine SHM fresh without polluting IPC Hz.
                #[cfg(feature = "output-wine")]
                if ctx.frames_since_input <= 10 && ctx.last_pose.timestamp_us > 0 {
                    if let Some(ref mut shm) = outputs.wine_shm {
                        shm.write(&ctx.last_pose);
                    }
                }
                // Watchdog: warn when the source stops delivering poses.
                //
                // This cannot distinguish "no face in frame" from "capture
                // thread died" — `InputSource::poll` returns `None` for both.
                // It used to claim the camera was frozen, which sent anyone
                // debugging a real fault straight to the wrong layer; simply
                // sitting out of frame produced it every 5 seconds. Say what is
                // actually known and point at the line that disambiguates: the
                // tracker's own per-second stats show whether frames are
                // arriving (`miss N` with a live frame count = no face).
                // That stats line is DEBUG — naming the level here matters,
                // otherwise this advice sends people looking for a line their
                // log level is hiding.
                //
                // It WARNs once per outage (CLAUDE.md log contract); while the
                // outage lasts the same text repeats at DEBUG at most every 5 s
                // (`StallWatch`, S1 of docs/FINAL_REVIEW.md).
                let stall = last_pose_time.elapsed();
                if stall > STALL_AFTER {
                    let neutral = if kind == SourceKind::Tobii {
                        ctx.state.tobii.lock().map(|t| t.neutral_progress).unwrap_or(None)
                    } else {
                        None
                    };
                    let expected = stall_is_expected(kind, neutral);
                    let line = stall_watch.tick(stall, expected, Instant::now());
                    if line != StallLine::Quiet {
                        let secs = stall.as_secs_f32();
                        let msg = match kind {
                            SourceKind::Webcam => format!(
                                "no pose from the webcam for {secs:.1}s — no face detected, or the \
                                 capture thread has stalled (run with RUST_LOG=debug and read the \
                                 tracker's per-second stats line to tell which)"
                            ),
                            SourceKind::Tobii => {
                                // The Tobii knows why: device state + reason are in its status.
                                let (state, message) = ctx
                                    .state
                                    .tobii
                                    .lock()
                                    .map(|t| (t.state, t.message.clone()))
                                    .unwrap_or_default();
                                if expected {
                                    format!(
                                        "no pose from the {} for {secs:.1}s — taking the neutral \
                                         ({state:?}: {message})",
                                        kind.label()
                                    )
                                } else {
                                    format!(
                                        "no pose from the {} for {secs:.1}s — {state:?}: {message} \
                                         (the Tobii tab shows the same)",
                                        kind.label()
                                    )
                                }
                            }
                        };
                        match line {
                            StallLine::Warn => warn!("{msg}"),
                            StallLine::Debug => debug!("{msg}"),
                            StallLine::Quiet => {}
                        }
                    }
                }
            }
        });
    }
}

/// Whether a pose stall is the expected kind, logged at DEBUG instead of WARN.
///
/// The Tobii sends no pose while it takes its neutral (first start and after
/// every recenter): the user is holding still on request, and the GUI says so
/// with a countdown. That is not "something is wrong" (CLAUDE.md log contract;
/// L5, 2026-11-02, where it warned at 2 s and 7 s of a normal capture). Any
/// other stall, including a Tobii one with no neutral under way, stays WARN.
fn stall_is_expected(kind: SourceKind, neutral_progress: Option<f32>) -> bool {
    kind == SourceKind::Tobii && neutral_progress.is_some()
}

/// A gap shorter than this is normal (a blink, a redetect).
const STALL_AFTER: Duration = Duration::from_secs(2);
/// While an outage lasts, a DEBUG reminder at most this often.
const STALL_REPEAT: Duration = Duration::from_secs(5);

/// What the stall watchdog logs on this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StallLine {
    Quiet,
    Warn,
    Debug,
}

/// Once-per-outage stall reporting (CLAUDE.md log contract: a loop "announces
/// once per outage, not once per retry"). S1, soak 2026-11-02: re-arming every
/// 5 s turned 63 outages into 421 WARN lines.
#[derive(Debug, Default)]
struct StallWatch {
    /// This outage has had its WARN.
    warned: bool,
    /// When this outage last logged anything.
    last_line: Option<Instant>,
}

impl StallWatch {
    /// A pose arrived or the source changed: any outage is over.
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// `stall`: time since the last pose. `expected`: [`stall_is_expected`].
    /// An expected stall (the Tobii neutral) is DEBUG only; if poses have still
    /// not resumed once it stops being expected, the outage gets its one WARN.
    fn tick(&mut self, stall: Duration, expected: bool, now: Instant) -> StallLine {
        if stall <= STALL_AFTER {
            return StallLine::Quiet;
        }
        if !expected && !self.warned {
            self.warned = true;
            self.last_line = Some(now);
            return StallLine::Warn;
        }
        if self.last_line.is_some_and(|t| now.saturating_duration_since(t) < STALL_REPEAT) {
            return StallLine::Quiet;
        }
        self.last_line = Some(now);
        StallLine::Debug
    }
}

/// Reset every per-source estimate: pipeline state, distance scale, standoff,
/// source-clock dt, the last pose the keep-alive re-sends, and the GUI's pose.
/// Nothing measured from one source may carry over to another (P05 §6, P07).
fn reset_source_state(ctx: &mut RunLoopCtx) {
    ctx.pipeline.reset();
    ctx.pipeline.set_distance_scale(DistanceScale::NONE);
    ctx.standoff = StandoffTracker::new();
    ctx.last_raw_ts = None;
    ctx.last_pose = Pose::zero(0);
    ctx.frames_since_input = 0;
    ctx.source_fresh = true;
    if let Ok(mut p) = ctx.state.latest_pose.lock() {
        *p = Pose::zero(0);
    }
}

/// Close the active source and open `new` in its place.
///
/// Order matters: the old source is dropped first (threads joined, device
/// released — `source::replace_source`), so a device is never open twice; a
/// webcam then gets 500 ms for V4L2 to let go. A change of source *kind* swaps
/// in that kind's active profile; a reopen of the same kind keeps the live
/// pipeline values (the user's unsaved slider moves) and only resets state.
async fn switch_source(ctx: &mut RunLoopCtx, new: HeadSource, persist: bool, context: &str) {
    let old_kind = ctx.kind();
    let new_kind = new.kind();
    info!("head source: {} → {} ({context})", old_kind.label(), new_kind.label());
    if let Some(rec) = ctx.recorder.as_mut() {
        rec.write_event(&format!("SOURCE: switching to {} ({context})", new_kind.label()));
    }
    if let Some(rr) = ctx.raw_recorder.as_mut() {
        rr.write_event(&format!("SOURCE: {}", new_kind.label()));
    }

    drop(ctx.source.take());
    ctx.state.camera_connected.store(false, Ordering::Relaxed);
    if old_kind == SourceKind::Webcam {
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
    if old_kind == SourceKind::Tobii {
        if let Ok(mut t) = ctx.state.tobii.lock() {
            *t = TobiiStatus::default();
        }
    }

    if new_kind != old_kind {
        ctx.pipeline = source::pipeline_for(new_kind);
    }
    reset_source_state(ctx);

    if persist {
        if let Err(e) = source::persist_head_source(&new) {
            warn!("could not save the head source to camera.toml: {e}");
        }
    }
    ctx.head = new;
    let opened = source::open_source(
        &ctx.head,
        source::OpenCtx {
            state: &ctx.state,
            models_dir: &ctx.models_dir,
            fov: ctx.current_fov,
            preview_shutdown: &mut ctx.preview_shutdown,
        },
    );
    source::replace_source(&mut ctx.source, || opened);
    let ok = ctx.source.is_some();
    if let Some(rec) = ctx.recorder.as_mut() {
        rec.write_event(&if ok {
            format!("SOURCE: connected {context}")
        } else {
            format!("ERROR: source open failed ({context})")
        });
    }
}

/// Reopen the active webcam (FOV or resolution change). The Tobii has neither,
/// so with the Tobii active the new setting is only saved for later.
async fn reopen_webcam(ctx: &mut RunLoopCtx, context: &str) {
    if ctx.kind() == SourceKind::Webcam {
        let head = ctx.head.clone();
        switch_source(ctx, head, false, context).await;
    }
}

/// Dispatch a single `EngineCmd`, mutating pipeline / source as needed.
///
/// Async because camera switches need a short sleep to let V4L2 release.
async fn dispatch_cmd(cmd: EngineCmd, ctx: &mut RunLoopCtx) {
    match cmd {
        EngineCmd::Recenter => {
            // Source first (the Tobii restarts its neutral), then CenterStage (P05 §4).
            if let Some(src) = ctx.source.as_mut() {
                src.recenter();
            }
            ctx.pipeline.reset();
            info!("pipeline reset (recenter)");
            fire_paramlog("recenter");
            if let Some(rec) = ctx.recorder.as_mut() { rec.write_event("RECENTER"); }
            if let Some(rr) = ctx.raw_recorder.as_mut() { rr.write_event("RECENTER"); }
        }

        EngineCmd::SetParam { ref stage, ref param, value } => {
            if ctx.pipeline.set_param(stage, param, value) {
                tracing::debug!("set {stage}.{param} = {value}");
            } else if stage == MARKER_STAGE {
                // `set marker <label> 1` is the documented way to drop a
                // timestamped event into a diagnostic capture — see
                // `scripts/mark.sh`. It is *meant* to match no stage, so the
                // fall-through is the mechanism working, not a fault. Logging it
                // at WARN broke the project's own logging contract (WARN means
                // something is wrong) and printed a scary line every time a
                // pilot marked a phase mid-flight.
                info!("marker: {param}");
            } else {
                warn!("unknown param: {stage}.{param}");
            }
            fire_paramlog(&format!("set {stage} {param} {value}"));
            if let Some(rec) = ctx.recorder.as_mut() {
                rec.write_event(&format!("PARAM: set {stage} {param} {value}"));
            }
            if let Some(rr) = ctx.raw_recorder.as_mut() {
                rr.write_event(&format!("PARAM: set {stage} {param} {value}"));
            }
        }

        EngineCmd::SwitchCamera { index } => {
            info!("switching to camera {index}...");
            fire_paramlog(&format!("switch-camera {index}"));
            if let Some(rec) = ctx.recorder.as_mut() {
                rec.write_event(&format!("CAMERA: switching to index {index}"));
            }

            // Auto-detect FOV for known camera models.
            let name = device_name(index);
            if let Some(fov) = name.as_deref().and_then(lookup_fov) {
                if (fov - ctx.current_fov).abs() > 0.1 {
                    info!(
                        "auto-detected FOV {fov}° for '{}' (was {}°)",
                        name.as_deref().unwrap_or_default(),
                        ctx.current_fov
                    );
                    ctx.current_fov = fov;
                    ctx.state.current_fov.store(fov.to_bits(), Ordering::Relaxed);
                    let _ = CameraConfig { fov_diag_deg: fov, ..CameraConfig::load() }.save();
                }
            }

            // Persisted with its V4L2 name, so the choice survives a restart
            // even if the index moves (KNOWN_BUGS #2).
            let context = format!("index={index} fov={}°", ctx.current_fov);
            switch_source(ctx, HeadSource::Webcam { index, name }, true, &context).await;
            if ctx.source.is_some() {
                info!("switched to camera {index} (FOV {}°)", ctx.current_fov);
            }
        }

        EngineCmd::SwitchSource { source } => {
            fire_paramlog(&format!("switch-source {}", source.kind().label()));
            switch_source(ctx, source, true, "user").await;
        }

        EngineCmd::SwitchSourceKind { kind } => {
            let cfg = CameraConfig::load();
            let source = match kind {
                SourceKind::Webcam => {
                    let s = cfg.webcam_selection();
                    HeadSource::Webcam { index: s.index, name: s.name }
                }
                SourceKind::Tobii => match cfg.head_source {
                    t @ HeadSource::Tobii { .. } => t,
                    HeadSource::Webcam { .. } => HeadSource::Tobii { serial: None },
                },
            };
            fire_paramlog(&format!("switch-source {}", kind.label()));
            switch_source(ctx, source, true, "user").await;
        }

        EngineCmd::SetFov { degrees } => {
            info!("setting FOV to {degrees}°");
            ctx.current_fov = degrees;
            ctx.state.current_fov.store(degrees.to_bits(), Ordering::Relaxed);
            let mut cam_cfg = CameraConfig::load();
            cam_cfg.fov_diag_deg = degrees;
            let _ = cam_cfg.save();
            reopen_webcam(ctx, &format!("fov={degrees}°")).await;
        }

        EngineCmd::SetResolution { width, height } => {
            info!("setting resolution to {width}×{height} — restarting camera...");
            let mut cam_cfg = CameraConfig::load();
            cam_cfg.camera_resolution = (width, height);
            let _ = cam_cfg.save();
            reopen_webcam(ctx, &format!("{width}×{height}")).await;
        }

        EngineCmd::Overlay { enabled } => {
            ctx.state.overlay_enabled.store(enabled, Ordering::Relaxed);
            info!("overlay {}", if enabled { "enabled" } else { "disabled" });
        }

        EngineCmd::StartRecording { path, size_limit_mb } => {
            // Stop any existing recording first.
            if let Some(ref mut rec) = ctx.recorder {
                rec.write_event("RECORDING_STOPPED: superseded by new recording");
                rec.flush();
            }
            ctx.recorder = None;
            match DiagRecorder::open(&path, size_limit_mb, &ctx.pipeline) {
                Ok(mut rec) => {
                    let cam_idx = ctx.state.camera_index.load(Ordering::Relaxed);
                    rec.write_event(&format!(
                        "RECORDING_STARTED: camera_index={cam_idx} size_limit={size_limit_mb}MB"
                    ));
                    ctx.state.recording_bytes.store(rec.bytes_written, Ordering::Relaxed);
                    ctx.state.recording_status.store(RecordingStatus::Active as u8, Ordering::Relaxed);
                    if let Ok(mut e) = ctx.state.recording_error.lock() { *e = None; }
                    ctx.recorder = Some(rec);
                    info!("diagnostic recording started: {}", path.display());
                }
                Err(e) => {
                    let msg = format!("{}: {e:#}", path.display());
                    warn!("failed to start diagnostic recording at {msg}");
                    if let Ok(mut slot) = ctx.state.recording_error.lock() {
                        *slot = Some(msg);
                    }
                    ctx.state.recording_status.store(RecordingStatus::Error as u8, Ordering::Relaxed);
                    ctx.state.recording_bytes.store(0, Ordering::Relaxed);
                }
            }
        }

        EngineCmd::StopRecording => {
            if let Some(ref mut rec) = ctx.recorder {
                rec.write_event("RECORDING_STOPPED: user request");
                rec.flush();
            }
            ctx.recorder = None;
            ctx.state.recording_status.store(RecordingStatus::Idle as u8, Ordering::Relaxed);
            ctx.state.recording_bytes.store(0, Ordering::Relaxed);
            if let Ok(mut e) = ctx.state.recording_error.lock() { *e = None; }
            info!("diagnostic recording stopped");
        }

        // SetScSmoothing is not feature-gated on the enum, so its arm cannot be
        // either: gating it with the SimConnect ones broke every build without
        // output-simconnect.
        EngineCmd::SetScSmoothing(_) => {
            // Handled before dispatch_cmd is called (or ignored without output-wine).
        }

        #[cfg(feature = "output-simconnect")]
        EngineCmd::SimConnectEnabled(_)
        | EngineCmd::SetSimConnectSmoothing(_)
        | EngineCmd::SimConnectToggleEnabled
        | EngineCmd::MsfsSeatNudge { .. }
        | EngineCmd::MsfsSeatReset => {
            // Handled before dispatch_cmd is called; unreachable in practice.
        }

        EngineCmd::RebindGlobalShortcuts
        | EngineCmd::ControllerBindStart { .. }
        | EngineCmd::ControllerBindCancel
        | EngineCmd::ControllerBindClear { .. } => {
            // Handled before dispatch_cmd is called; unreachable in practice.
        }
    }
}

// ── Private: command socket listener ────────────────────────────────────────

/// Parse a single text command from the command socket into an `EngineCmd`.
///
/// The `overlay` command is handled inline here (direct `AtomicBool` write,
/// bypasses the run-loop channel) and returns `None`. All other parse errors
/// emit a warning and also return `None`.
fn parse_cmd_line(line: &str, overlay_enabled: &Arc<AtomicBool>) -> Option<EngineCmd> {
    if line == "recenter" {
        return Some(EngineCmd::Recenter);
    }
    if let Some(rest) = line.strip_prefix("set ") {
        let parts: Vec<&str> = rest.splitn(3, ' ').collect();
        if parts.len() == 3 {
            if let Ok(value) = parts[2].parse::<f32>() {
                return Some(EngineCmd::SetParam {
                    stage: parts[0].to_string(),
                    param: parts[1].to_string(),
                    value,
                });
            }
            warn!("bad value in set command: {line:?}");
        } else {
            warn!("malformed set command: {line:?}");
        }
        return None;
    }
    if let Some(rest) = line.strip_prefix("switch-camera ") {
        if let Ok(index) = rest.trim().parse::<u32>() {
            return Some(EngineCmd::SwitchCamera { index });
        }
        warn!("bad camera index: {line:?}");
        return None;
    }
    if let Some(rest) = line.strip_prefix("switch-source ") {
        // `switch-source tobii` | `switch-source webcam [index]`
        let parts: Vec<&str> = rest.split_whitespace().collect();
        return match parts[..] {
            ["tobii"] => Some(EngineCmd::SwitchSourceKind { kind: SourceKind::Tobii }),
            ["webcam"] => Some(EngineCmd::SwitchSourceKind { kind: SourceKind::Webcam }),
            ["webcam", i] => match i.parse::<u32>() {
                Ok(index) => Some(EngineCmd::SwitchCamera { index }),
                Err(_) => {
                    warn!("bad webcam index: {line:?}");
                    None
                }
            },
            _ => {
                warn!("bad switch-source command (want `switch-source tobii` or `switch-source webcam [index]`): {line:?}");
                None
            }
        };
    }
    if let Some(rest) = line.strip_prefix("set-fov ") {
        if let Ok(degrees) = rest.trim().parse::<f32>() {
            return Some(EngineCmd::SetFov { degrees: degrees.clamp(1.0, 180.0) });
        }
        warn!("bad FOV value: {line:?}");
        return None;
    }
    if let Some(rest) = line.strip_prefix("overlay ") {
        // Overlay is handled directly (AtomicBool) without going through the
        // run loop, matching the original daemon's behaviour.
        overlay_enabled.store(rest.trim() == "1", Ordering::Relaxed);
        return None;
    }
    // MSFS seat + streaming commands: the command socket is the mechanism
    // that works while the sim has focus (GUI hotkeys are focus-gated), so
    // desktop/WM hotkeys and Stream Decks bind these one-liners.
    #[cfg(feature = "output-simconnect")]
    if let Some(rest) = line.strip_prefix("msfs-seat ") {
        let rest = rest.trim();
        if rest == "reset" {
            return Some(EngineCmd::MsfsSeatReset);
        }
        let parts: Vec<&str> = rest.split_whitespace().collect();
        if let [axis_s, delta_s] = parts[..] {
            let axis = match axis_s {
                "x" => Some(0u8),
                "y" => Some(1),
                "z" => Some(2),
                _ => None,
            };
            let delta = delta_s.parse::<f32>().ok().filter(|d| d.is_finite() && d.abs() <= 0.6);
            if let (Some(axis), Some(delta_m)) = (axis, delta) {
                return Some(EngineCmd::MsfsSeatNudge { axis, delta_m });
            }
        }
        warn!("bad msfs-seat command (want `msfs-seat <x|y|z> <delta_m ≤ 0.6>` or `msfs-seat reset`): {line:?}");
        return None;
    }
    #[cfg(feature = "output-simconnect")]
    if let Some(rest) = line.strip_prefix("msfs-enabled ") {
        return match rest.trim() {
            "0" => Some(EngineCmd::SimConnectEnabled(false)),
            "1" => Some(EngineCmd::SimConnectEnabled(true)),
            "toggle" => Some(EngineCmd::SimConnectToggleEnabled),
            _ => {
                warn!("bad msfs-enabled command (want 0, 1, or toggle): {line:?}");
                None
            }
        };
    }
    warn!("unknown command: {line:?}");
    None
}

/// Listen on the command socket for one-shot text commands from external tools.
///
/// Translates text commands into `EngineCmd` values and forwards them to the
/// run loop. This socket is the bridge for tools that can't link against the
/// engine crate directly (shell scripts, the X-Plane plugin, etc.).
async fn cmd_listener(
    path:            std::path::PathBuf,
    cmd_tx:          mpsc::Sender<EngineCmd>,
    overlay_enabled: Arc<AtomicBool>,
) -> anyhow::Result<()> {
    use tokio::io::AsyncReadExt;
    use tokio::net::UnixListener;

    let _ = std::fs::remove_file(&path);
    let listener = UnixListener::bind(&path)?;
    info!("command listener ready");

    loop {
        let (stream, _) = match listener.accept().await {
            Ok(conn) => conn,
            Err(e) => {
                warn!("command listener accept error: {e:#}");
                continue;
            }
        };
        let mut buf = String::new();
        if let Err(e) = stream.take(4096).read_to_string(&mut buf).await {
            warn!("command listener read error: {e:#}");
            continue;
        }
        if let Some(cmd) = parse_cmd_line(buf.trim(), &overlay_enabled) {
            let _ = cmd_tx.send(cmd).await;
        }
    }
}

// ── Private: preview writer ──────────────────────────────────────────────────

/// Copy JPEG frames from a tracker's preview slot into the shared engine preview.
#[cfg(feature = "input-neuralnet")]
///
/// Spawns a background thread that polls at ~7 Hz. Also writes frames to the
/// filesystem preview path for external tools (headtrack-tester CSV dump mode).
///
/// On camera switch, a new thread is spawned for the new tracker; the old thread
/// harmlessly polls an exhausted slot and does nothing until it is dropped.
fn start_preview_writer(
    src: PreviewFrame,
    dst: EnginePreview,
    prev_shutdown: &mut Option<Arc<AtomicBool>>,
) {
    // Signal the previous preview thread to exit before spawning a replacement.
    if let Some(flag) = prev_shutdown.take() {
        flag.store(true, Ordering::Relaxed);
    }

    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_clone = Arc::clone(&shutdown);
    let preview_path = headtrack_ipc::preview_path();
    if preview_path.is_none() {
        static NOTED: std::sync::Once = std::sync::Once::new();
        NOTED.call_once(|| {
            info!(
                "XDG_RUNTIME_DIR is unset: not writing the external camera preview file \
                 (it is a picture of you, and would otherwise land in a shared directory)"
            );
        });
    }

    std::thread::Builder::new()
        .name("preview-writer".into())
        .spawn(move || {
            let mut seq: u64 = 0;
            while !shutdown_clone.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(150));
                if let Some(jpeg) = src.take() {
                    seq += 1;
                    let jpeg = Arc::new(jpeg);
                    // Update shared in-memory preview for the GUI.
                    if let Ok(mut lock) = dst.lock() {
                        *lock = (seq, Some(Arc::clone(&jpeg)));
                    }
                    // Also write to the filesystem for external tools: owner-only
                    // and atomic, and never outside XDG_RUNTIME_DIR (S5).
                    if let Some(ref path) = preview_path {
                        let _ = write_private_atomic(path, jpeg.as_ref());
                    }
                }
            }
        })
        .ok();

    *prev_shutdown = Some(shutdown);
}

/// Write `bytes` to `path` as mode 0600, atomically (temp file, then rename).
///
/// The preview is a picture of the user's face (FINAL_REVIEW S5). `fs::write`
/// creates files `0666 & ~umask`, normally 0644. The temp file is chmodded
/// after opening as well, because `mode` only applies when a file is created
/// and a temp file left by an older version may already exist with 0644.
#[cfg(feature = "input-neuralnet")]
fn write_private_atomic(path: &std::path::Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
    let tmp = path.with_extension("tmp");
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(0o600)
        .open(&tmp)?;
    f.set_permissions(std::fs::Permissions::from_mode(0o600))?;
    f.write_all(bytes)?;
    drop(f);
    std::fs::rename(&tmp, path)
}

// ── Private: param log side-channel ─────────────────────────────────────────

/// Fire-and-forget write to the param log socket.
///
/// The headtrack-tester (in `--dump` mode) listens here to correlate parameter
/// changes with pose data in the CSV output. The socket may not exist; errors
/// are silently ignored.
fn fire_paramlog(msg: &str) {
    use std::io::Write;
    use std::os::unix::net::UnixStream;
    if let Ok(mut s) = UnixStream::connect(headtrack_ipc::paramlog_socket_path()) {
        let _ = s.write_all(msg.as_bytes());
    }
}

// ── Private: config helpers ──────────────────────────────────────────────────

/// Walk up from `start` looking for an `assets/models` directory.
///
/// Bounded rather than unbounded so a missing checkout cannot walk to `/`.
/// Six levels covers `target/<profile>/<exe>` and `target/<triple>/<profile>/<exe>`.
fn find_models_upwards(start: &Path) -> Option<PathBuf> {
    let mut dir = Some(start);
    for _ in 0..6 {
        let d = dir?;
        let candidate = d.join("assets").join("models");
        if candidate.is_dir() {
            return Some(candidate);
        }
        dir = d.parent();
    }
    None
}

/// Resolve the directory containing ONNX model files.
///
/// Order: explicit override, AppImage bundle, XDG data dir, then the source
/// checkout — located by walking up from the **executable**, and only then from
/// the working directory.
///
/// The executable walk is what makes a development build independent of where
/// it was launched from. This used to end at the bare relative path
/// `assets/models`, so `cargo run` from anywhere but the repo root failed with
/// a "file does not exist" error naming a model, and no camera — which reads
/// like a missing model rather than a wrong working directory.
fn resolve_models_dir() -> PathBuf {
    if let Ok(p) = std::env::var("HEADTRACK_MODELS") {
        return PathBuf::from(p);
    }
    if let Ok(appdir) = std::env::var("APPDIR") {
        let p = PathBuf::from(appdir).join("models");
        if p.exists() {
            return p;
        }
    }
    let data_home = std::env::var("XDG_DATA_HOME")
        .unwrap_or_else(|_| format!("{}/.local/share", std::env::var("HOME").unwrap_or_default()));
    let p = PathBuf::from(data_home).join("headtrack-rs").join("models");
    if p.exists() {
        return p;
    }
    if let Some(p) = std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().and_then(find_models_upwards))
    {
        return p;
    }
    if let Some(p) = std::env::current_dir().ok().and_then(|d| find_models_upwards(&d)) {
        return p;
    }
    // Nothing found. Returning the historical relative path keeps the failure
    // shape callers already handle; the log above says where we looked.
    PathBuf::from("assets/models")
}

const FOV_LOOKUP: &[(&str, f32)] = &[
    ("c920",       78.0),
    ("delan",      75.0),
    ("brio 500",   78.0),
    ("mx brio",    78.0),
    ("nexigo n60", 110.0),
    ("facecam",    84.0),
];

fn device_name(index: u32) -> Option<String> {
    std::fs::read_to_string(format!("/sys/class/video4linux/video{index}/name"))
        .ok()
        .map(|s| s.trim().to_string())
}

fn lookup_fov(name: &str) -> Option<f32> {
    let lower = name.to_lowercase();
    FOV_LOOKUP.iter().find(|(k, _)| lower.contains(k)).map(|(_, fov)| *fov)
}

// Re-export the IPC socket path so callers (GUI settings panel) don't need a
// direct dep on headtrack-ipc just for the display label.
pub use headtrack_ipc::socket_path as ipc_socket_path;

#[cfg(test)]
mod cmd_parse_tests {
    use super::*;

    fn parse(line: &str) -> Option<EngineCmd> {
        parse_cmd_line(line, &Arc::new(AtomicBool::new(false)))
    }

    #[test]
    fn switch_source_commands() {
        assert!(matches!(
            parse("switch-source tobii"),
            Some(EngineCmd::SwitchSourceKind { kind: SourceKind::Tobii })
        ));
        assert!(matches!(parse("switch-source webcam 3"), Some(EngineCmd::SwitchCamera { index: 3 })));
        assert!(matches!(
            parse("switch-source webcam"),
            Some(EngineCmd::SwitchSourceKind { kind: SourceKind::Webcam })
        ));
        assert!(parse("switch-source webcam x").is_none());
        assert!(parse("switch-source smoothtrack").is_none());
        assert!(parse("switch-source").is_none());
    }

    /// The command socket is the scripting tier: Stream Decks, shell one-liners
    /// and the X-Plane plugin menu all speak it, and none of them can see a
    /// `warn!`. A parser regression is therefore silent at the call site — the
    /// command simply does nothing. This table pins every accepted form and,
    /// just as importantly, every rejected one.
    #[test]
    fn accepts_every_documented_command() {
        assert!(matches!(parse("recenter"), Some(EngineCmd::Recenter)));

        match parse("set accela yaw.smoothing 0.6") {
            Some(EngineCmd::SetParam { stage, param, value }) => {
                assert_eq!(stage, "accela");
                assert_eq!(param, "yaw.smoothing");
                assert!((value - 0.6).abs() < 1e-6);
            }
            other => panic!("set: {other:?}"),
        }

        // Unknown stage/param names are accepted on purpose — that is what
        // makes `set marker tick 1` a zero-side-effect event marker.
        assert!(matches!(parse("set marker tick 1"), Some(EngineCmd::SetParam { .. })));

        assert!(matches!(parse("switch-camera 2"), Some(EngineCmd::SwitchCamera { index: 2 })));

        match parse("set-fov 78") {
            Some(EngineCmd::SetFov { degrees }) => assert!((degrees - 78.0).abs() < 1e-6),
            other => panic!("set-fov: {other:?}"),
        }
    }

    #[test]
    fn rejects_malformed_input_without_panicking() {
        for bad in [
            "",
            "   ",
            "reCenter",                 // case-sensitive by design
            "set",
            "set accela",               // too few tokens
            "set accela yaw.smoothing", // no value
            "set accela yaw.smoothing abc",
            "switch-camera",
            "switch-camera -1",         // u32 parse fails
            "switch-camera two",
            "set-fov",
            "set-fov wide",
            "nonsense",
        ] {
            assert!(parse(bad).is_none(), "{bad:?} should not parse");
        }
    }

    #[test]
    fn set_fov_is_clamped_to_a_sane_range() {
        // The socket takes input from scripts; an out-of-range FOV must be
        // clamped here rather than reaching the camera layer.
        match parse("set-fov 999") {
            Some(EngineCmd::SetFov { degrees }) => assert!((degrees - 180.0).abs() < 1e-6),
            other => panic!("high: {other:?}"),
        }
        match parse("set-fov 0") {
            Some(EngineCmd::SetFov { degrees }) => assert!((degrees - 1.0).abs() < 1e-6),
            other => panic!("low: {other:?}"),
        }
    }

    #[test]
    fn overlay_toggles_the_flag_and_yields_no_command() {
        let flag = Arc::new(AtomicBool::new(false));
        assert!(parse_cmd_line("overlay 1", &flag).is_none());
        assert!(flag.load(Ordering::Relaxed), "overlay 1 must set the flag");
        assert!(parse_cmd_line("overlay 0", &flag).is_none());
        assert!(!flag.load(Ordering::Relaxed), "overlay 0 must clear the flag");
        // Anything that isn't "1" reads as off rather than erroring.
        flag.store(true, Ordering::Relaxed);
        assert!(parse_cmd_line("overlay yes", &flag).is_none());
        assert!(!flag.load(Ordering::Relaxed));
    }

    #[cfg(feature = "output-simconnect")]
    #[test]
    fn msfs_seat_and_enabled_commands() {
        assert!(matches!(parse("msfs-seat reset"), Some(EngineCmd::MsfsSeatReset)));

        for (line, want_axis) in [("msfs-seat x 0.01", 0u8), ("msfs-seat y -0.02", 1), ("msfs-seat z 0.6", 2)] {
            match parse(line) {
                Some(EngineCmd::MsfsSeatNudge { axis, .. }) => assert_eq!(axis, want_axis, "{line}"),
                other => panic!("{line}: {other:?}"),
            }
        }

        // The ±0.6 m clamp is a safety limit, enforced at parse time so a
        // scripted typo can't fling the eyepoint out of the cockpit.
        for bad in [
            "msfs-seat x 0.7",
            "msfs-seat x -0.61",
            "msfs-seat w 0.1",   // bad axis
            "msfs-seat x",       // missing delta
            "msfs-seat x nan",
            "msfs-seat x inf",
            "msfs-seat",
        ] {
            assert!(parse(bad).is_none(), "{bad:?} should not parse");
        }

        assert!(matches!(parse("msfs-enabled 0"), Some(EngineCmd::SimConnectEnabled(false))));
        assert!(matches!(parse("msfs-enabled 1"), Some(EngineCmd::SimConnectEnabled(true))));
        assert!(matches!(parse("msfs-enabled toggle"), Some(EngineCmd::SimConnectToggleEnabled)));
        for bad in ["msfs-enabled", "msfs-enabled 2", "msfs-enabled on"] {
            assert!(parse(bad).is_none(), "{bad:?} should not parse");
        }
    }
}

#[cfg(test)]
mod models_dir_tests {
    use super::*;

    /// The upward walk is what makes a development build independent of the
    /// working directory it was launched from. Without it, `cargo run` from
    /// anywhere but the repo root resolved the bare relative path
    /// `assets/models`, failed to load the models, and surfaced as
    /// "no camera at index 0" — a symptom that points at the webcam rather
    /// than at the model path.
    #[test]
    fn finds_models_from_a_nested_start_directory() {
        let root = std::env::temp_dir().join(format!("ht-models-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let models = root.join("assets").join("models");
        std::fs::create_dir_all(&models).expect("temp tree");
        // Where a dev binary actually sits: <root>/target/release/
        let exe_dir = root.join("target").join("release");
        std::fs::create_dir_all(&exe_dir).expect("temp tree");

        assert_eq!(find_models_upwards(&exe_dir).as_deref(), Some(models.as_path()));
        assert_eq!(find_models_upwards(&root).as_deref(), Some(models.as_path()));

        // A tree with no models must not walk out to the filesystem root.
        let bare = root.join("elsewhere").join("deep");
        std::fs::create_dir_all(&bare).expect("temp tree");
        let _ = std::fs::remove_dir_all(&models);
        assert_eq!(find_models_upwards(&bare), None);

        let _ = std::fs::remove_dir_all(&root);
    }
}

#[cfg(test)]
mod stall_level_tests {
    use super::*;

    /// Only a Tobii taking its neutral is a quiet stall; a webcam never is,
    /// and a Tobii with no neutral under way is a real one.
    #[test]
    fn only_the_tobii_neutral_capture_is_an_expected_stall() {
        assert!(stall_is_expected(SourceKind::Tobii, Some(0.0)));
        assert!(stall_is_expected(SourceKind::Tobii, Some(0.6)));
        assert!(!stall_is_expected(SourceKind::Tobii, None));
        assert!(!stall_is_expected(SourceKind::Webcam, None));
        assert!(!stall_is_expected(SourceKind::Webcam, Some(0.5)));
    }

    /// Drive the watchdog at the run loop's 4 ms cadence; count WARN and DEBUG lines.
    fn drive(
        w: &mut StallWatch,
        t0: Instant,
        from_ms: u64,
        to_ms: u64,
        outage_ms: u64,
        expected: impl Fn(u64) -> bool,
    ) -> (usize, usize) {
        let (mut warns, mut debugs) = (0, 0);
        let mut ms = from_ms;
        while ms < to_ms {
            let stall = Duration::from_millis(ms - outage_ms);
            match w.tick(stall, expected(ms), t0 + Duration::from_millis(ms)) {
                StallLine::Warn => warns += 1,
                StallLine::Debug => debugs += 1,
                StallLine::Quiet => {}
            }
            ms += 4;
        }
        (warns, debugs)
    }

    /// S1 (soak 2026-11-02): a 60 s outage used to WARN 12 times.
    #[test]
    fn an_outage_warns_once_however_long_it_lasts() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        // WARN at 2.004 s; DEBUG at 7.004, 12.004, …, 57.004 s.
        assert_eq!(drive(&mut w, t0, 0, 60_000, 0, |_| false), (1, 11));
    }

    #[test]
    fn a_short_gap_is_silent_and_every_new_outage_warns_again() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        assert_eq!(drive(&mut w, t0, 0, 1_900, 0, |_| false), (0, 0));
        w.reset(); // a pose arrived
        assert_eq!(drive(&mut w, t0, 2_000, 20_000, 2_000, |_| false).0, 1);
        w.reset();
        assert_eq!(drive(&mut w, t0, 20_000, 40_000, 20_000, |_| false).0, 1);
    }

    /// L5 + S1: the neutral capture is DEBUG only; a stall that outlasts it
    /// (the face is lost) gets exactly one WARN.
    #[test]
    fn the_neutral_capture_never_warns_but_a_stall_that_outlasts_it_does() {
        let t0 = Instant::now();
        let mut w = StallWatch::default();
        assert_eq!(drive(&mut w, t0, 0, 8_000, 0, |_| true), (0, 2));
        assert_eq!(drive(&mut w, t0, 8_000, 60_000, 0, |_| false).0, 1);
    }
}

#[cfg(all(test, feature = "input-neuralnet"))]
mod preview_file_tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt;

    /// S5: the preview (a picture of the user) is owner-only, even when an
    /// older version left a world-readable temp file or preview behind.
    #[test]
    fn the_preview_file_is_owner_only_and_replaced_whole() {
        let dir = std::env::temp_dir().join(format!("htrs-preview-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("headtrack-preview.jpg");
        let tmp = path.with_extension("tmp");
        for p in [&path, &tmp] {
            std::fs::write(p, b"old").unwrap();
            std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o644)).unwrap();
        }

        write_private_atomic(&path, b"jpeg bytes").unwrap();

        let meta = std::fs::metadata(&path).unwrap();
        assert_eq!(meta.permissions().mode() & 0o777, 0o600);
        assert_eq!(std::fs::read(&path).unwrap(), b"jpeg bytes");
        assert!(!tmp.exists(), "the temp file was renamed into place");

        let _ = std::fs::remove_dir_all(&dir);
    }
}
