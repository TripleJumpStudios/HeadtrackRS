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
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use anyhow::Result;
use headtrack_core::{
    config::{CameraConfig, ProfileConfig},
    pipeline::stages::{
        AxisMaskStage, CenterStage, CrossAxisCompStage, DeadzoneStage, OneEuroStage,
        PredictionStage, ResponseCurveStage, SlewLimitStage, ZMedianStage,
    },
    InputSource, Pipeline, Pose,
};
use headtrack_input_neuralnet::{NeuralNetInput, PreviewFrame};
use headtrack_ipc::server::IpcServer;
use headtrack_output_simconnect::SimConnectOutput;
use headtrack_output_wine::WineShmWriter;
use tokio::sync::{broadcast, mpsc};
use tracing::{info, warn};

// Re-export so callers (GUI) don't need a direct dep on headtrack-input-neuralnet.
pub use headtrack_input_neuralnet::{PreviewFrame as EnginePreviewFrame, PreviewSlot as EnginePreviewSlot};
pub use headtrack_core::Pose as EnginePose;

/// Shared camera preview: `(seq_counter, latest_jpeg)`.
///
/// The engine's preview writer increments `seq` on each new frame, letting
/// the GUI skip decoding when nothing changed. Compatible with egui's
/// per-repaint polling pattern.
pub type EnginePreview = Arc<Mutex<(u64, Option<Vec<u8>>)>>;

// ── Public command enum ──────────────────────────────────────────────────────

/// Commands that can be sent to the engine pipeline from any caller.
#[derive(Debug)]
pub enum EngineCmd {
    Recenter,
    SetParam { stage: String, param: String, value: f32 },
    SwitchCamera { index: u32 },
    SetFov { degrees: f32 },
    /// Change capture resolution and restart the camera.  Resolution is
    /// persisted to `camera.toml` so it survives restarts.
    SetResolution { width: u32, height: u32 },
    Overlay { enabled: bool },
    /// Begin writing a diagnostic CSV to `path`. Replaces any active recording.
    StartRecording { path: PathBuf, size_limit_mb: u32 },
    /// Stop the current recording and flush the file.
    StopRecording,
}

// ── Recording status ─────────────────────────────────────────────────────────

/// Status of the in-GUI diagnostic recorder, stored as a `u8` in `EngineState`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecordingStatus {
    Idle         = 0,
    Active       = 1,
    LimitReached = 2,
}

impl RecordingStatus {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Active,
            2 => Self::LimitReached,
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
    /// Latest processed pose from the pipeline.
    pub latest_pose: Mutex<Pose>,
    /// Frames per second, updated approximately every second.
    pub fps: Mutex<f32>,
    /// Whether a camera input source is currently active.
    pub camera_connected: AtomicBool,
    /// Current camera device index.
    pub camera_index: AtomicU32,
    /// Current camera FOV in degrees, stored as f32 bits. Updated by the engine
    /// whenever FOV changes (camera switch with auto-detect, or explicit SetFov).
    pub current_fov: AtomicU32,
    /// Latest camera preview JPEG. The engine writer updates this at ~7 Hz;
    /// callers check `seq` to detect new frames before decoding.
    pub preview: EnginePreview,
    /// Whether bounding-box overlay is drawn on preview frames.
    pub overlay_enabled: Arc<AtomicBool>,
    /// Current diagnostic recording state (`RecordingStatus as u8`).
    pub recording_status: AtomicU8,
    /// Bytes written to the current diagnostic recording file.
    pub recording_bytes: AtomicU64,
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
            // Non-blocking shutdown — tasks are cancelled, not waited on.
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
            current_fov:      AtomicU32::new(CameraConfig::load().fov_diag_deg.to_bits()),
            preview:          Arc::new(Mutex::new((0, None))),
            overlay_enabled:  Arc::new(AtomicBool::new(false)),
            recording_status: AtomicU8::new(0),
            recording_bytes:  AtomicU64::new(0),
        });

        let (cmd_tx, cmd_rx) = mpsc::channel::<EngineCmd>(64);

        // Run async init on the new runtime (binds IPC socket, opens camera,
        // spawns long-lived tasks) then return so the caller can proceed.
        rt.block_on(engine_init(Arc::clone(&state), cmd_tx.clone(), cmd_rx))?;

        Ok(Engine { state, cmd_tx, runtime: Some(rt) })
    }

    /// Shared state — clone and hold as long as needed.
    pub fn state(&self) -> Arc<EngineState> {
        Arc::clone(&self.state)
    }

    /// Command sender — clone for every widget that needs to send commands.
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

    // 2. Build the pipeline with tuned defaults.
    let mut pipeline = Pipeline::new(vec![
        Box::new(CenterStage::new()),
        Box::new(ZMedianStage::new()),
        Box::new(SlewLimitStage::new()),
        Box::new(OneEuroStage::new(0.01, 0.03)),
        Box::new(CrossAxisCompStage::new()),
        Box::new(PredictionStage::new()),
        Box::new(ResponseCurveStage::new()),
        Box::new(DeadzoneStage::new()),
        Box::new(AxisMaskStage::new()),
    ]);
    // Z axis: low min_cutoff for heavy still-filtering, high beta for responsiveness.
    pipeline.set_param("one-euro-filter", "z.min_cutoff", 0.001);
    pipeline.set_param("one-euro-filter", "z.beta", 0.10);

    // Load saved profile if one exists.
    match ProfileConfig::load(&ProfileConfig::active_config_path()) {
        Ok(cfg) => {
            apply_config(&mut pipeline, &cfg);
            info!("loaded profile: {}", cfg.name);
        }
        Err(_) => info!("no saved profile — using built-in defaults"),
    }

    // 3. Load camera config and auto-detect FOV.
    let mut camera_cfg = CameraConfig::load();
    let camera_index: u32 = std::env::var("HEADTRACK_CAMERA")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    if let Some(name) = device_name(camera_index) {
        if let Some(fov) = lookup_fov(&name) {
            if (fov - camera_cfg.fov_diag_deg).abs() > 0.1 {
                info!("auto-detected FOV {fov}° for '{name}' (was {}°)", camera_cfg.fov_diag_deg);
                camera_cfg.fov_diag_deg = fov;
                let _ = camera_cfg.save();
            }
        }
    }

    state.camera_index.store(camera_index, Ordering::Relaxed);

    // 4. Attempt to open the initial camera. Cameraless start is OK.
    let models_dir = resolve_models_dir();
    info!("models dir: {}", models_dir.display());

    let mut preview_shutdown: Option<Arc<AtomicBool>> = None;
    let source: Option<Box<dyn InputSource>> = match NeuralNetInput::with_overlay(
        &models_dir,
        camera_index,
        camera_cfg.fov_diag_deg,
        Arc::clone(&state.overlay_enabled),
    ) {
        Ok(s) => {
            info!("input source: {}", s.name());
            state.camera_connected.store(true, Ordering::Relaxed);
            // Wire the tracker's preview into state.preview (+ filesystem).
            start_preview_writer(
                Arc::clone(s.preview_frame()),
                Arc::clone(&state.preview),
                &mut preview_shutdown,
            );
            Some(Box::new(s))
        }
        Err(e) => {
            warn!("no camera at index {camera_index}: {e:#}");
            info!("engine started without camera — send SwitchCamera to connect");
            None
        }
    };

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

    // 6. Open the Wine shared memory bridge (zero cost when companion not running).
    let wine_shm = match WineShmWriter::open() {
        Ok(w) => { info!("Wine SHM bridge ready"); Some(w) }
        Err(e) => { warn!("Wine SHM unavailable: {e:#}"); None }
    };

    // 6b. Open the SimConnect output for MSFS 2024 (auto-reconnects; MSFS need not be running).
    let simconnect = match SimConnectOutput::open(5557) {
        Ok(s) => { info!("SimConnect output ready (127.0.0.1:5557)"); Some(s) }
        Err(e) => { warn!("SimConnect output unavailable: {e:#}"); None }
    };

    // 7. Spawn the run loop — runs forever.
    tokio::spawn(run_loop(
        state,
        cmd_rx,
        source,
        camera_index,
        camera_cfg.fov_diag_deg,
        models_dir,
        pipeline,
        pose_tx,
        wine_shm,
        simconnect,
        preview_shutdown,
    ));

    Ok(())
}

// ── Private: diagnostic recorder ─────────────────────────────────────────────

struct DiagRecorder {
    writer:           std::io::BufWriter<std::fs::File>,
    start:            Instant,
    bytes_written:    u64,
    size_limit_bytes: u64,
}

impl DiagRecorder {
    fn open(path: &std::path::Path, size_limit_mb: u32) -> std::io::Result<Self> {
        use std::io::Write;
        let file = std::fs::File::create(path)?;
        let mut writer = std::io::BufWriter::new(file);
        let header = b"# headtrack-rs diagnostic recording v1\ntime_s,yaw,pitch,roll,x,y,z,fps\n";
        writer.write_all(header)?;
        Ok(Self {
            writer,
            start: Instant::now(),
            bytes_written: header.len() as u64,
            size_limit_bytes: size_limit_mb as u64 * 1024 * 1024,
        })
    }

    /// Write a pose data row. Returns `true` if the size limit is now reached.
    fn write_pose(&mut self, pose: &Pose, fps: f32) -> bool {
        use std::io::Write;
        let t = self.start.elapsed().as_secs_f64();
        let line = format!(
            "{:.4},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.1}\n",
            t, pose.yaw, pose.pitch, pose.roll, pose.x, pose.y, pose.z, fps
        );
        self.bytes_written += line.len() as u64;
        let _ = self.writer.write_all(line.as_bytes());
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

// ── Private: run loop ────────────────────────────────────────────────────────

const POLL_INTERVAL: Duration = Duration::from_millis(4); // ~250 Hz poll ceiling

#[allow(clippy::too_many_arguments)]
async fn run_loop(
    state:                Arc<EngineState>,
    mut cmd_rx:           mpsc::Receiver<EngineCmd>,
    mut source:           Option<Box<dyn InputSource>>,
    mut current_camera:   u32,
    mut current_fov:      f32,
    models_dir:           PathBuf,
    mut pipeline:         Pipeline,
    pose_tx:              broadcast::Sender<Pose>,
    mut wine_shm:         Option<WineShmWriter>,
    simconnect:           Option<SimConnectOutput>,
    mut preview_shutdown: Option<Arc<AtomicBool>>,
) {
    let mut interval = tokio::time::interval(POLL_INTERVAL);
    let mut last_tick = Instant::now();
    let mut last_pose = Pose::zero(0);
    let mut frames_since_input: u32 = 0;
    let mut fps_frame_count: u32 = 0;
    let mut fps_timer = Instant::now();
    let mut current_fps: f32 = 0.0;
    let mut last_pose_time = Instant::now();
    let mut stall_last_warned = Instant::now().checked_sub(Duration::from_secs(60)).unwrap_or_else(Instant::now);
    let mut recorder: Option<DiagRecorder> = None;

    loop {
        interval.tick().await;

        // Drain pending commands (non-blocking).
        while let Ok(cmd) = cmd_rx.try_recv() {
            dispatch_cmd(
                cmd,
                &mut pipeline,
                &mut source,
                &mut current_camera,
                &mut current_fov,
                &models_dir,
                &state,
                &mut recorder,
                &mut preview_shutdown,
            ).await;
        }
        // If the camera was just switched out, reset the stall watchdog so we
        // don't false-alarm during the 500ms V4L2 release + reopen window.
        if source.is_none() {
            last_pose_time = Instant::now();
        }

        let Some(ref mut src) = source else { continue };

        if let Some(raw) = src.poll() {
            let now = Instant::now();
            let dt  = now.duration_since(last_tick).as_secs_f32().max(0.001);
            last_tick = now;
            last_pose_time = now;

            let processed = pipeline.process(raw, dt);
            last_pose = processed;
            frames_since_input = 0;

            // Update shared state for the GUI.
            if let Ok(mut p) = state.latest_pose.lock() {
                *p = processed;
            }

            // FPS counter — updated once per second.
            fps_frame_count += 1;
            let elapsed = fps_timer.elapsed().as_secs_f32();
            if elapsed >= 1.0 {
                current_fps = fps_frame_count as f32 / elapsed;
                if let Ok(mut f) = state.fps.lock() {
                    *f = current_fps;
                }
                fps_frame_count = 0;
                fps_timer = Instant::now();
            }

            // Diagnostic recorder — write pose row each frame.
            if let Some(ref mut rec) = recorder {
                let limit = rec.write_pose(&processed, current_fps);
                state.recording_bytes.store(rec.bytes_written, Ordering::Relaxed);
                if limit {
                    rec.write_event("RECORDING_STOPPED: size limit reached");
                    rec.flush();
                    recorder = None;
                    state.recording_status.store(RecordingStatus::LimitReached as u8, Ordering::Relaxed);
                }
            }

            // Broadcast to external IPC clients (X-Plane plugin, tester).
            let _ = pose_tx.send(processed);

            // Write to Wine SHM bridge (SC native Wine via Z:\dev\shm).
            if let Some(ref mut shm) = wine_shm {
                shm.write(&processed);
            }

            // Write to SimConnect (MSFS 2024 via TCP Camera API).
            if let Some(ref sc) = simconnect {
                sc.write(&processed);
            }
        } else {
            frames_since_input = frames_since_input.saturating_add(1);
            // During brief frame gaps keep Wine SHM fresh without polluting IPC Hz.
            if frames_since_input <= 10 && last_pose.timestamp_us > 0 {
                if let Some(ref mut shm) = wine_shm {
                    shm.write(&last_pose);
                }
            }
            // Watchdog: warn if the connected source stops delivering poses.
            let stall = last_pose_time.elapsed();
            if stall > Duration::from_secs(2) && stall_last_warned.elapsed() > Duration::from_secs(5) {
                warn!("tracker stall: no pose for {:.1}s (camera thread may be frozen)", stall.as_secs_f32());
                stall_last_warned = Instant::now();
            }
        }
    }
}

/// Dispatch a single `EngineCmd`, mutating pipeline / source as needed.
///
/// Async because camera switches need a short sleep to let V4L2 release.
#[allow(clippy::too_many_arguments)]
async fn dispatch_cmd(
    cmd:              EngineCmd,
    pipeline:         &mut Pipeline,
    source:           &mut Option<Box<dyn InputSource>>,
    current_camera:   &mut u32,
    current_fov:      &mut f32,
    models_dir:       &std::path::Path,
    state:            &Arc<EngineState>,
    recorder:         &mut Option<DiagRecorder>,
    preview_shutdown: &mut Option<Arc<AtomicBool>>,
) {
    match cmd {
        EngineCmd::Recenter => {
            pipeline.reset();
            info!("pipeline reset (recenter)");
            fire_paramlog("recenter");
            if let Some(rec) = recorder.as_mut() { rec.write_event("RECENTER"); }
        }

        EngineCmd::SetParam { ref stage, ref param, value } => {
            if pipeline.set_param(stage, param, value) {
                tracing::debug!("set {stage}.{param} = {value}");
            } else {
                warn!("unknown param: {stage}.{param}");
            }
            fire_paramlog(&format!("set {stage} {param} {value}"));
            if let Some(rec) = recorder.as_mut() {
                rec.write_event(&format!("PARAM: set {stage} {param} {value}"));
            }
        }

        EngineCmd::SwitchCamera { index } => {
            info!("switching to camera {index}...");
            fire_paramlog(&format!("switch-camera {index}"));
            if let Some(rec) = recorder.as_mut() {
                rec.write_event(&format!("CAMERA: switching to index {index}"));
            }

            if let Some(name) = device_name(index) {
                if let Some(fov) = lookup_fov(&name) {
                    if (fov - *current_fov).abs() > 0.1 {
                        info!("auto-detected FOV {fov}° for '{name}' (was {current_fov}°)");
                        *current_fov = fov;
                        state.current_fov.store(fov.to_bits(), Ordering::Relaxed);
                        let _ = CameraConfig { fov_diag_deg: fov, ..CameraConfig::load() }.save();
                    }
                }
            }

            *source = None;
            state.camera_connected.store(false, Ordering::Relaxed);
            tokio::time::sleep(Duration::from_millis(500)).await;

            match NeuralNetInput::with_overlay(
                models_dir, index, *current_fov,
                Arc::clone(&state.overlay_enabled),
            ) {
                Ok(new_src) => {
                    start_preview_writer(
                        Arc::clone(new_src.preview_frame()),
                        Arc::clone(&state.preview),
                        preview_shutdown,
                    );
                    state.camera_index.store(index, Ordering::Relaxed);
                    state.camera_connected.store(true, Ordering::Relaxed);
                    *current_camera = index;
                    *source = Some(Box::new(new_src));
                    pipeline.reset();
                    info!("switched to camera {index} (FOV {current_fov}°)");
                    if let Some(rec) = recorder.as_mut() {
                        rec.write_event(&format!("CAMERA: connected index={index}"));
                    }
                }
                Err(e) => {
                    warn!("failed to switch to camera {index}: {e:#}");
                    if let Some(rec) = recorder.as_mut() {
                        rec.write_event(&format!("ERROR: camera {index} open failed"));
                    }
                }
            }
        }

        EngineCmd::SetFov { degrees } => {
            info!("setting FOV to {degrees}°");
            *current_fov = degrees;
            state.current_fov.store(degrees.to_bits(), Ordering::Relaxed);
            let mut cam_cfg = CameraConfig::load();
            cam_cfg.fov_diag_deg = degrees;
            let _ = cam_cfg.save();

            *source = None;
            state.camera_connected.store(false, Ordering::Relaxed);
            tokio::time::sleep(Duration::from_millis(500)).await;

            match NeuralNetInput::with_overlay(
                models_dir, *current_camera, *current_fov,
                Arc::clone(&state.overlay_enabled),
            ) {
                Ok(new_src) => {
                    start_preview_writer(
                        Arc::clone(new_src.preview_frame()),
                        Arc::clone(&state.preview),
                        preview_shutdown,
                    );
                    state.camera_connected.store(true, Ordering::Relaxed);
                    *source = Some(Box::new(new_src));
                    pipeline.reset();
                    info!("camera re-opened at FOV {degrees}°");
                }
                Err(e) => warn!("failed to re-open camera after FOV change: {e:#}"),
            }
        }

        EngineCmd::SetResolution { width, height } => {
            info!("setting resolution to {width}×{height} — restarting camera...");
            let mut cam_cfg = CameraConfig::load();
            cam_cfg.camera_resolution = (width, height);
            let _ = cam_cfg.save();

            *source = None;
            state.camera_connected.store(false, Ordering::Relaxed);
            tokio::time::sleep(Duration::from_millis(500)).await;

            match NeuralNetInput::with_overlay(
                models_dir, *current_camera, *current_fov,
                Arc::clone(&state.overlay_enabled),
            ) {
                Ok(new_src) => {
                    start_preview_writer(
                        Arc::clone(new_src.preview_frame()),
                        Arc::clone(&state.preview),
                        preview_shutdown,
                    );
                    state.camera_connected.store(true, Ordering::Relaxed);
                    *source = Some(Box::new(new_src));
                    pipeline.reset();
                    info!("camera restarted at {width}×{height}");
                }
                Err(e) => warn!("failed to reopen camera at {width}×{height}: {e:#}"),
            }
        }

        EngineCmd::Overlay { enabled } => {
            state.overlay_enabled.store(enabled, Ordering::Relaxed);
            info!("overlay {}", if enabled { "enabled" } else { "disabled" });
        }

        EngineCmd::StartRecording { path, size_limit_mb } => {
            // Stop any existing recording first.
            if let Some(ref mut rec) = *recorder {
                rec.write_event("RECORDING_STOPPED: superseded by new recording");
                rec.flush();
            }
            *recorder = None;
            match DiagRecorder::open(&path, size_limit_mb) {
                Ok(mut rec) => {
                    let cam_idx = state.camera_index.load(Ordering::Relaxed);
                    rec.write_event(&format!(
                        "RECORDING_STARTED: camera_index={cam_idx} size_limit={size_limit_mb}MB"
                    ));
                    state.recording_bytes.store(rec.bytes_written, Ordering::Relaxed);
                    state.recording_status.store(RecordingStatus::Active as u8, Ordering::Relaxed);
                    *recorder = Some(rec);
                    info!("diagnostic recording started: {}", path.display());
                }
                Err(e) => {
                    warn!("failed to start diagnostic recording at {}: {e:#}", path.display());
                }
            }
        }

        EngineCmd::StopRecording => {
            if let Some(ref mut rec) = *recorder {
                rec.write_event("RECORDING_STOPPED: user request");
                rec.flush();
            }
            *recorder = None;
            state.recording_status.store(RecordingStatus::Idle as u8, Ordering::Relaxed);
            state.recording_bytes.store(0, Ordering::Relaxed);
            info!("diagnostic recording stopped");
        }
    }
}

// ── Private: command socket listener ────────────────────────────────────────

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
        let (mut stream, _) = match listener.accept().await {
            Ok(conn) => conn,
            Err(e) => {
                warn!("command listener accept error: {e:#}");
                continue;
            }
        };
        let mut buf = String::new();
        if let Err(e) = stream.read_to_string(&mut buf).await {
            warn!("command listener read error: {e:#}");
            continue;
        }
        let line = buf.trim();

        if line == "recenter" {
            let _ = cmd_tx.send(EngineCmd::Recenter).await;
        } else if let Some(rest) = line.strip_prefix("set ") {
            let parts: Vec<&str> = rest.splitn(3, ' ').collect();
            if parts.len() == 3 {
                if let Ok(value) = parts[2].parse::<f32>() {
                    let _ = cmd_tx.send(EngineCmd::SetParam {
                        stage: parts[0].to_string(),
                        param: parts[1].to_string(),
                        value,
                    }).await;
                } else {
                    warn!("bad value in set command: {line:?}");
                }
            } else {
                warn!("malformed set command: {line:?}");
            }
        } else if let Some(rest) = line.strip_prefix("switch-camera ") {
            if let Ok(index) = rest.trim().parse::<u32>() {
                let _ = cmd_tx.send(EngineCmd::SwitchCamera { index }).await;
            } else {
                warn!("bad camera index: {line:?}");
            }
        } else if let Some(rest) = line.strip_prefix("set-fov ") {
            if let Ok(degrees) = rest.trim().parse::<f32>() {
                let _ = cmd_tx.send(EngineCmd::SetFov { degrees }).await;
            } else {
                warn!("bad FOV value: {line:?}");
            }
        } else if let Some(rest) = line.strip_prefix("overlay ") {
            // Overlay is handled directly (AtomicBool) without going through the
            // run loop, matching the original daemon's behaviour.
            overlay_enabled.store(rest.trim() == "1", Ordering::Relaxed);
        } else {
            warn!("unknown command: {line:?}");
        }
    }
}

// ── Private: preview writer ──────────────────────────────────────────────────

/// Copy JPEG frames from a tracker's preview slot into the shared engine preview.
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

    std::thread::Builder::new()
        .name("preview-writer".into())
        .spawn(move || {
            let mut seq: u64 = 0;
            while !shutdown_clone.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(150));
                if let Some(jpeg) = src.take() {
                    seq += 1;
                    // Update shared in-memory preview for the GUI.
                    if let Ok(mut lock) = dst.lock() {
                        *lock = (seq, Some(jpeg.clone()));
                    }
                    // Also write to filesystem for headtrack-tester compat.
                    let tmp = preview_path.with_extension("tmp");
                    if std::fs::write(&tmp, &jpeg).is_ok() {
                        let _ = std::fs::rename(&tmp, &preview_path);
                    }
                }
            }
        })
        .ok();

    *prev_shutdown = Some(shutdown);
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

/// Resolve the directory containing ONNX model files.
fn resolve_models_dir() -> PathBuf {
    if let Ok(p) = std::env::var("HEADTRACK_MODELS") {
        return PathBuf::from(p);
    }
    if let Ok(appdir) = std::env::var("APPDIR") {
        let p = PathBuf::from(appdir).join("models");
        if p.exists() { return p; }
    }
    let data_home = std::env::var("XDG_DATA_HOME")
        .unwrap_or_else(|_| format!("{}/.local/share", std::env::var("HOME").unwrap_or_default()));
    let p = PathBuf::from(data_home).join("headtrack-rs").join("models");
    if p.exists() { return p; }
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

fn apply_config(pipeline: &mut Pipeline, cfg: &ProfileConfig) {
    const AXES: [&str; 6] = ["yaw", "pitch", "roll", "x", "y", "z"];
    for (i, axis) in AXES.iter().enumerate() {
        pipeline.set_param("one-euro-filter", &format!("{axis}.min_cutoff"), cfg.filter[i].min_cutoff);
        pipeline.set_param("one-euro-filter", &format!("{axis}.beta"),       cfg.filter[i].beta);
        pipeline.set_param("response-curve",  &format!("{axis}.sensitivity"), cfg.curve[i].sensitivity);
        pipeline.set_param("response-curve",  &format!("{axis}.exponent"),    cfg.curve[i].exponent);
        pipeline.set_param("deadzone",  axis, cfg.deadzone[i]);
        pipeline.set_param("slew-limit", axis, cfg.slew_limit[i]);
    }
    pipeline.set_param("cross-axis", "yaw_to_pitch", cfg.cross_axis.yaw_to_pitch);
    pipeline.set_param("cross-axis", "yaw_to_x",     cfg.cross_axis.yaw_to_x);
    pipeline.set_param("cross-axis", "yaw_to_y",     cfg.cross_axis.yaw_to_y);
    pipeline.set_param("cross-axis", "yaw_to_roll",  cfg.cross_axis.yaw_to_roll);
    pipeline.set_param("cross-axis", "pitch_to_y",   cfg.cross_axis.pitch_to_y);
    pipeline.set_param("cross-axis", "z_to_y",       cfg.cross_axis.z_to_y);
    pipeline.set_param("cross-axis", "z_to_pitch",   cfg.cross_axis.z_to_pitch);
    pipeline.set_param("prediction", "predict_ms",           cfg.prediction.predict_ms);
    pipeline.set_param("prediction", "rot_process_noise",    cfg.prediction.rot_process_noise);
    pipeline.set_param("prediction", "rot_measurement_noise", cfg.prediction.rot_measurement_noise);
    pipeline.set_param("prediction", "pos_process_noise",    cfg.prediction.pos_process_noise);
    pipeline.set_param("prediction", "pos_measurement_noise", cfg.prediction.pos_measurement_noise);
    pipeline.set_param("center", "drift_rate",   cfg.center.drift_rate);
    pipeline.set_param("center", "z.drift_mult", cfg.center.z_drift_mult);
}

// Re-export the IPC socket path so callers (GUI settings panel) don't need a
// direct dep on headtrack-ipc just for the display label.
pub use headtrack_ipc::socket_path as ipc_socket_path;
