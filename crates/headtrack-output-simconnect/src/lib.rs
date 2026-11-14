//! MSFS 2024 SU5+ head tracking output via the SimConnect Camera API.
//!
//! # How it works
//!
//! A background tokio task maintains a persistent TCP connection to the MSFS
//! SimConnect server (127.0.0.1:5557, configured via SimConnect.xml). On
//! connect, we:
//!
//! 1. Send `MSG_OPEN` and wait for `RECV_OPEN`.
//! 2. Subscribe to the `Frame` system event for a per-frame send cadence, and
//!    to `Sim` / `SimStart` / `SimStop` for lifecycle reporting.
//! 3. Climb the [`BringUp`] ladder — `SubscribeToCameraStatusUpdate`, then
//!    `CameraAcquire` once the sim says it is not driving the camera itself,
//!    then `EnumerateCameraDefinitions` + the TITLE SimVar registration once
//!    the sim confirms we hold it, then
//!    `CameraSetUsingCameraDefinition("Pilot")` to seat the camera at the pilot
//!    eyepoint. Each rung waits for the sim's own `RECV_CAMERA_STATUS`, because
//!    MSFS accepts SimConnect connections (and claims to be "running") long
//!    before an aircraft exists, and sending the aircraft-facing calls into
//!    that window crashes the sim.
//!
//! The ladder starts climbing at connect and is driven **only** by camera
//! status, never by a lifecycle event — see [`BringUp`]. That is what makes it
//! start-order independent: it works whether the sim was launched before or
//! after us.
//!
//! On every Frame event, if the camera is acquired and not game-controlled,
//! we send a `CameraSet` with the latest pose in the EYEPOINT referential.
//!
//! On graceful shutdown / pause, we send `CameraRelease` so the sim can hand
//! the camera to other add-ons (or back to the user).
//!
//! # Why this replaces the legacy `CameraSetRelative6DOF` (`MSG_CAMERA_SET = 0x30`)
//!
//! Pre-SU5 we used `CameraSetRelative6DOF`, which has no acquire/release model —
//! once we called it, MSFS implicitly gave us the camera forever, with no way
//! to politely take it back during menu interactions. SU5 introduced an
//! explicit acquire/release model with a `bGameControlled` flag in
//! `RECV_CAMERA_STATUS`; the new `CameraSet` (`MSG_CAMERA_SET = 0x62`) only
//! works with that flow. Result: clean menu UI, no hijacking.
//!
//! # SimConnect.xml (one-time setup)
//!
//! Write to the MSFS 2024 user data directory in the Proton prefix:
//! ```text
//! ~/.local/share/Steam/steamapps/compatdata/
//!   2537590/pfx/drive_c/users/steamuser/AppData/Roaming/
//!   Microsoft Flight Simulator 2024/SimConnect.xml
//! ```
//! Note: MSFS may regenerate this file with a default (no port 5557) after
//! major updates. Check it first if SimConnect stops connecting.
//!
//! ```xml
//! <?xml version="1.0" encoding="Windows-1252"?>
//! <SimBase.Document Type="SimConnect" version="1,0">
//!   <SimConnect.Comm>
//!     <Disabled>False</Disabled>
//!     <Protocol>IPv4</Protocol>
//!     <Scope>local</Scope>
//!     <Address>127.0.0.1</Address>
//!     <Port>5557</Port>
//!     <MaxClients>64</MaxClients>
//!     <MaxRecvSize>41088</MaxRecvSize>
//!     <DisableNagle>1</DisableNagle>
//!   </SimConnect.Comm>
//! </SimBase.Document>
//! ```

pub(crate) mod connection;
pub(crate) mod protocol;

use headtrack_core::Pose;
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, atomic::{AtomicBool, AtomicU32, Ordering}};
use std::time::{Duration, Instant};
use tokio::sync::watch;
use tracing::{debug, info, warn};

// ---------------------------------------------------------------------------
// Pose → SIMCONNECT_DATA_CAMERA conversion
// ---------------------------------------------------------------------------

/// Sentinel value the Camera API treats as "do not modify this field".
/// Defined in `SimConnect.h` as `SIMCONNECT_CAMERA_IGNORE_FIELD = FLT_MAX`.
const CAMERA_IGNORE_FIELD: f64 = f32::MAX as f64;

/// Convert a headtrack-rs `Pose` to the position+rotation arguments for
/// `build_camera_set`.
///
/// Returns `(position_xyz_meters, pbh_degrees)` in the EYEPOINT referential.
///
/// Sign conventions verified in-cockpit against MSFS SU5 EYEPOINT (2026-05-06):
/// - X: pass-through  (legacy `CameraSetRelative6DOF` had it inverted; EYEPOINT does not)
/// - Y: pass-through
/// - Z: pass-through
/// - Pitch: inverted   (legacy was pass-through; EYEPOINT is opposite)
/// - Bank (roll): inverted
/// - Heading (yaw): pass-through
fn pose_to_camera(pose: &Pose) -> ([f64; 3], [f32; 3]) {
    let position = [
        ( pose.x as f64) / 1000.0,
        ( pose.y as f64) / 1000.0,
        ( pose.z as f64) / 1000.0,
    ];
    // PBH is { Pitch, Bank, Heading } per SimConnect.h.
    let pbh = [-pose.pitch, -pose.roll, pose.yaw];
    (position, pbh)
}

// ---------------------------------------------------------------------------
// Background connection task
// ---------------------------------------------------------------------------

const FRAME_EVENT_ID: u32 = 1;
/// `"Sim"` system event — `dwData` 1 = the user is in control of an aircraft,
/// 0 = menu, world map, or loading.
///
/// **Reporting only.** This used to gate the first rung of the bring-up, and
/// that was a bug in both directions: `Sim = 1` arrives during the loading
/// screen (so it is not proof of a live flight), and MSFS 2024 does *not*
/// replay the current state to a client that subscribes mid-flight — so an app
/// started after the sim never saw the transition and the ladder parked
/// forever. Measured 2026-08-19; see [`SimReady`].
const SIM_EVENT_ID: u32 = 2;
/// `"SimStart"` — flight-load edge. Redundant with `"Sim"` going to 1, kept as
/// a second path in case a sim build reports only one of them.
const SIM_START_EVENT_ID: u32 = 3;
/// `"SimStop"` — the matching flight-unload edge.
const SIM_STOP_EVENT_ID: u32 = 4;
const CLIENT_ID: &str = "headtrack-rs";

/// How often to report that the bring-up ladder is still parked below
/// streaming. "Connected but not tracking" is otherwise indistinguishable from
/// a fault, and there is deliberately **no timeout that gives up and sends the
/// bring-up anyway** — that is precisely what crashed the sim. If the ladder
/// never finishes, the log has to say why.
const WAIT_LOG_INTERVAL: Duration = Duration::from_secs(20);

/// Interval between periodic `CameraGet` readbacks while streaming. The
/// readback compares what MSFS actually applied against what we last sent —
/// the only way to notice the sim moving the eyepoint under us or silently
/// rejecting our sets.
const READBACK_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum interval between repeated RECV_EXCEPTION warnings once the first
/// few for a code have been logged. Rate-limited, never fully suppressed — a
/// persistent per-frame exception must stay visible in the log.
const EXCEPTION_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// Purpose of an in-flight `CameraGet` request. TCP and the sim's processing
/// are both in-order, so responses arrive in send order and a FIFO of these
/// keeps a periodic readback response from being mistaken for a bootstrap
/// neutral (which would compound the head offset into `pilot_neutral`).
#[derive(Clone, Copy, PartialEq)]
enum GetPurpose {
    /// Bootstrap: the response defines `pilot_neutral`.
    Neutral,
    /// Periodic readback: the response is compared against the last CameraSet.
    Readback,
}

/// Smallest angular difference between two headings/angles in degrees,
/// wrap-aware (359° vs 1° = 2°, not 358°).
fn angle_delta_deg(a: f32, b: f32) -> f32 {
    let d = (a - b).abs() % 360.0;
    if d > 180.0 { 360.0 - d } else { d }
}

/// Per-frame adapter trace (instrument I2), enabled by setting
/// `HEADTRACK_MSFS_TRACE` to an output CSV path. Records, for every CameraSet,
/// the dt/alpha actually used plus the target (pre-smoothing) and sent
/// (post-smoothing, absolute) camera values, so a tick in the rendered view
/// can be attributed to — or exonerated from — the output layer. Gate skips
/// and readback mismatches land as `#`-comment event lines. Every line is
/// flushed as written; diagnostic use only.
struct TraceWriter {
    writer: std::io::BufWriter<std::fs::File>,
    start:  Instant,
}

impl TraceWriter {
    fn from_env() -> Option<Self> {
        let path = std::env::var("HEADTRACK_MSFS_TRACE").ok()?;
        use std::io::Write;
        match std::fs::File::create(&path) {
            Ok(f) => {
                let mut writer = std::io::BufWriter::new(f);
                let _ = writer.write_all(
                    b"t_s,frame_rate,dt_s,alpha,\
                      target_x,target_y,target_z,target_p,target_b,target_h,\
                      sent_x,sent_y,sent_z,sent_p,sent_b,sent_h\n",
                );
                info!("HEADTRACK_MSFS_TRACE — per-frame adapter trace -> {path}");
                Some(Self { writer, start: Instant::now() })
            }
            Err(e) => {
                warn!("HEADTRACK_MSFS_TRACE set but could not create {path}: {e}");
                None
            }
        }
    }

    fn row(
        &mut self,
        frame_rate: f32,
        dt: f32,
        alpha: f32,
        target: &([f64; 3], [f32; 3]),
        sent: &([f64; 3], [f32; 3]),
    ) {
        use std::io::Write;
        let t = self.start.elapsed().as_secs_f64();
        let (tp, tb) = target;
        let (sp, sb) = sent;
        let _ = writeln!(
            self.writer,
            "{t:.4},{frame_rate:.1},{dt:.6},{alpha:.5},\
             {:.5},{:.5},{:.5},{:.4},{:.4},{:.4},\
             {:.5},{:.5},{:.5},{:.4},{:.4},{:.4}",
            tp[0], tp[1], tp[2], tb[0], tb[1], tb[2],
            sp[0], sp[1], sp[2], sb[0], sb[1], sb[2],
        );
        let _ = self.writer.flush();
    }

    fn event(&mut self, msg: &str) {
        use std::io::Write;
        let t = self.start.elapsed().as_secs_f64();
        let _ = writeln!(self.writer, "# {t:.4} {msg}");
        let _ = self.writer.flush();
    }
}

/// Send the `Frame` system event subscription. Returns `false` on send error
/// (caller reconnects).
async fn subscribe_frame(conn: &mut connection::SimConnectTcp) -> bool {
    let pkt = protocol::build_packet(
        protocol::MSG_SUBSCRIBE_TO_SYSTEM_EVENT,
        &protocol::build_subscribe_event(FRAME_EVENT_ID, "Frame"),
    );
    if let Err(e) = conn.send_packet(&pkt).await {
        warn!("SimConnect: subscribe Frame failed: {e:#}");
        return false;
    }
    true
}

/// Subscribe to the system events that tell us whether a flight is live.
/// Returns `false` on send error (caller reconnects).
async fn subscribe_sim_state(conn: &mut connection::SimConnectTcp) -> bool {
    for (id, name) in [
        (SIM_EVENT_ID, "Sim"),
        (SIM_START_EVENT_ID, "SimStart"),
        (SIM_STOP_EVENT_ID, "SimStop"),
    ] {
        let pkt = protocol::build_packet(
            protocol::MSG_SUBSCRIBE_TO_SYSTEM_EVENT,
            &protocol::build_subscribe_event(id, name),
        );
        if let Err(e) = conn.send_packet(&pkt).await {
            warn!("SimConnect: subscribe {name} failed: {e:#}");
            return false;
        }
    }
    true
}

/// How far through the Camera API bring-up this connection has got.
///
/// # Why this is a ladder and not a timer
///
/// MSFS opens its SimConnect port early in start-up, long before an aircraft
/// exists, and when headtrack-rs is already running the 2-second retry loop
/// connects into exactly that window. Sending the bring-up there took the sim
/// down — the bug whose workaround was "launch MSFS first, the app second".
///
/// The first attempt at a fix gated the bring-up on the `"Sim"` system event.
/// That failed on 2026-08-17: **MSFS 2024 reports `Sim = 1` while it is still
/// loading** (43 ms after connect, at a 85 fps loading-screen frame rate), so
/// the gate opened during start-up and the sim died 400 ms later. The captured
/// sequence is diagnostic — the sim answered `CameraAcquire` and the status
/// subscription (`acquired=1 game_controlled=true`) and then crashed *without*
/// ever returning the camera definition list. What it never survived was
/// `EnumerateCameraDefinitions`, which walks the **current aircraft's** camera
/// definitions, and the TITLE SimVar request against the user aircraft. Neither
/// of those exists mid-load.
///
/// So lifecycle events are not trusted as a readiness signal. There are two
/// rungs, and the second is unlocked by the sim's *own* camera status:
///
/// 1. [`WaitAcquired`](Self::WaitAcquired) — the starting rung.
///    `SubscribeToCameraStatusUpdate` and a single `CameraAcquire` go out
///    immediately after the handshake, and the acquire is retried while the
///    sim says the camera is free but not ours. Neither call is
///    aircraft-facing; the 2026-08-17 crash trace shows the sim answering both
///    happily *during* a load and dying only on what came next.
/// 2. [`Streaming`](Self::Streaming) — the sim confirmed we hold the camera, so
///    the aircraft-facing calls (`EnumerateCameraDefinitions`, TITLE
///    registration) are finally safe, and per-frame `CameraSet` runs.
///
/// # Why `CameraAcquire` is also the state query
///
/// **MSFS never reports camera status unless it changes.** Measured
/// 2026-08-20: six `SubscribeToCameraStatusUpdate` packets over 30 s drew no
/// reply at all, and the first status of that connection arrived 68 s in, as
/// the `bGameControlled` transition when the flight went live. Subscribing
/// tells the sim where to send *future* changes; it is not a question and it
/// gets no answer.
///
/// `CameraAcquire` **is** answered — 35 ms and 473 ms in the two runs measured.
/// So it doubles as the only way to ask "what is the camera doing right now",
/// which a client connecting into an already-live flight has no other means of
/// learning. It claims nothing it cannot give straight back, and `CameraSet` is
/// gated on the status regardless, so asking early is safe.
///
/// # Why there is no "wait for a live flight" rung
///
/// There was one, from 2026-08-18 to 2026-08-19, and it was a ship-blocking
/// regression: it advanced only when the `"Sim"` system event arrived, and that
/// event is a **transition** notification. Launch the sim first and connect to
/// an already-running flight and the transition has already happened — no event
/// ever arrives, the ladder parks on rung 0 forever and not one `CameraSet` is
/// sent. A failing run left a 0-byte trace file next to a 361 KB healthy one.
///
/// The rung was also unnecessary. The two calls that killed a loading sim are
/// `EnumerateCameraDefinitions` and the TITLE request, and both are gated on
/// `bGameControlled` — a property of the *camera status*, not of any lifecycle
/// event. Deleting the rung keeps the crash fix exactly as it was and makes
/// start order irrelevant.
///
/// `HEADTRACK_MSFS_READY_GATE=off` sends the whole bring-up at connect, the
/// pre-2026-08-17 behaviour. It is for A/B only — that is the ordering that
/// crashes the sim.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum BringUp {
    WaitAcquired,
    Streaming,
}

/// Which rung the ladder moves to, given the sim's latest camera status.
///
/// Split out as a pure function so the ordering is testable without a sim.
/// Every input is a field of `RECV_CAMERA_STATUS`: **no lifecycle event, no
/// timer and no elapsed time appears here.** That is the property that makes
/// the bring-up start-order independent, and the reason the 2026-08-19
/// regression cannot come back without this signature changing.
///
/// `None` means stay put.
fn next_rung(stage: BringUp, acquired: u32, game_controlled: bool) -> Option<BringUp> {
    match stage {
        // The sim confirms we hold the camera and is not driving it itself, so
        // the aircraft-facing calls are finally safe.
        BringUp::WaitAcquired if acquired == protocol::CAMERA_ACQUIRED && !game_controlled => {
            Some(BringUp::Streaming)
        }
        _ => None,
    }
}

/// Whether to (re)send `CameraAcquire` given the sim's latest camera status.
///
/// Pure and rate-limit-free so the *decision* is testable; the caller applies
/// [`REACQUIRE_INTERVAL`] on top.
///
/// This is what was missing on 2026-08-20 and it is worth being explicit about,
/// because the failure it caused was silent and permanent. Restarting the app
/// while sitting in the cockpit produced:
///
/// ```text
/// acquired=3 game_controlled=false   ← add-on camera not available
///   → CameraAcquire
/// acquired=0 game_controlled=false   ← now free … and nothing ever again
/// ```
///
/// One acquire moved the sim from "unavailable" to "free", which is exactly the
/// state a second acquire would have taken. But the retry lived inside the
/// `RECV_CAMERA_STATUS` handler, and MSFS only sends status **on change** — so
/// the state that needed a retry was precisely the state that generated no
/// further packets to trigger one. The retry therefore runs on the Frame tick
/// instead, which arrives every rendered frame regardless.
///
/// Note it fires for **any** non-acquired availability, not just
/// `CAMERA_NOT_ACQUIRED`: `CAMERA_USER_DISABLED` (3) is what the sim reports
/// after a mid-flight reconnect, and the run above shows an acquire clearing it.
///
/// # It deliberately does not consider the rung
///
/// Losing the camera while already streaming needs a retry just as much as
/// never having had it — more so, because it happens mid-flight with no restart
/// to paper over it. The first version of this check required
/// `stage == WaitAcquired`, which made the in-game **Camera ▸ Addons** toggle a
/// one-way door: flipping it off dropped us to `acquired=3`, then `0`, and
/// nothing ever asked for the camera again (measured 2026-08-20 — every
/// subsequent frame silently skipped by the send gate, recoverable only by
/// restarting the app). Ownership is a property of the camera, not of how far
/// through the bring-up we are.
fn should_acquire(acquired: u32, game_controlled: bool) -> bool {
    acquired != protocol::CAMERA_ACQUIRED && !game_controlled
}

/// Tracks what the sim has told us about the flight, **for logging only**.
///
/// The `"Sim"` state gates nothing. It is not proof of a live flight (it is
/// reported during the loading screen) and its absence is not proof of no
/// flight (it never arrives at all if we connected after the flight started).
/// Both halves of that have burned us; see [`BringUp`]. It is kept because it
/// is still the best thing to print when someone asks why the camera has not
/// come up yet.
struct SimReady {
    live: bool,
    bypassed: bool,
}

impl SimReady {
    fn new() -> Self {
        let bypassed = std::env::var("HEADTRACK_MSFS_READY_GATE")
            .is_ok_and(|v| v.eq_ignore_ascii_case("off"));
        if bypassed {
            warn!(
                "SimConnect: HEADTRACK_MSFS_READY_GATE=off — the full camera bring-up will be \
                 sent at connect; this is the ordering that crashes the sim during start-up"
            );
        }
        Self { live: false, bypassed }
    }

    /// Apply a `"Sim"` / `"SimStart"` / `"SimStop"` notification.
    fn on_event(&mut self, event_id: u32, dw_data: u32) {
        let live = match event_id {
            SIM_EVENT_ID => dw_data != 0,
            SIM_START_EVENT_ID => true,
            SIM_STOP_EVENT_ID => false,
            _ => return,
        };
        if live != self.live {
            info!(
                "SimConnect: sim state → {} (event {event_id}, dwData {dw_data})",
                if live { "running" } else { "menu / loading" }
            );
        }
        self.live = live;
    }
}

/// Send a single `CameraAcquire` packet. Returns `Err` only on TCP failure.
async fn send_camera_acquire(conn: &mut connection::SimConnectTcp) -> Result<(), String> {
    let acq = protocol::build_packet(
        protocol::MSG_CAMERA_ACQUIRE,
        &protocol::build_camera_acquire(CLIENT_ID),
    );
    conn.send_packet(&acq).await.map_err(|e| format!("CameraAcquire send failed: {e:#}"))
}

/// The pre-2026-08-17 bring-up: everything at once, immediately after the
/// handshake. Only reachable via `HEADTRACK_MSFS_READY_GATE=off`, and only so
/// the crash can be A/B'd against the ladder. Named to be hard to reach for by
/// accident.
async fn bring_up_camera_unsafe(conn: &mut connection::SimConnectTcp) -> bool {
    if let Err(e) = send_camera_acquire(conn).await {
        warn!("SimConnect: {e}");
        return false;
    }
    subscribe_camera_status(conn).await && request_aircraft_state(conn).await
}

/// Ask the sim to push us its camera state.
///
/// Read-only: it takes no ownership and changes nothing, which is why it is the
/// first camera call the bring-up ladder makes — it is how we find out whether
/// the sim is past its loading screen without having to trust a lifecycle
/// event. Returns `false` on send error (caller reconnects).
async fn subscribe_camera_status(conn: &mut connection::SimConnectTcp) -> bool {
    let sub = protocol::build_packet(
        protocol::MSG_SUBSCRIBE_TO_CAMERA_STATUS_UPDATE,
        &protocol::build_subscribe_camera_status(),
    );
    if let Err(e) = conn.send_packet(&sub).await {
        warn!("SimConnect: SubscribeToCameraStatusUpdate failed: {e:#}");
        return false;
    }
    true
}

/// The aircraft-facing half of the bring-up: enumerate the aircraft's camera
/// definitions (so we can seat the addon camera at the pilot eyepoint) and
/// register the TITLE SimVar for per-aircraft seat offsets.
///
/// **These are the calls that crashed the sim** when they were sent during
/// start-up — both read state that belongs to a loaded user aircraft. They are
/// held back until the sim has confirmed we hold the camera, which cannot
/// happen before a flight is live. See [`BringUp`]. Returns `false` on send
/// error (caller reconnects).
async fn request_aircraft_state(conn: &mut connection::SimConnectTcp) -> bool {
    let enumerate = protocol::build_packet(
        protocol::MSG_ENUMERATE_CAMERA_DEFINITIONS,
        &protocol::build_enumerate_camera_definitions(),
    );
    if let Err(e) = conn.send_packet(&enumerate).await {
        warn!("SimConnect: EnumerateCameraDefinitions failed: {e:#}");
        return false;
    }

    // Register the aircraft TITLE SimVar (per-aircraft seat offsets). A
    // sim-side rejection surfaces as RECV_EXCEPTION and only costs the
    // per-aircraft keying — the seat runs under the "default" key and the
    // camera path is untouched.
    let add_pkt = protocol::build_packet(
        protocol::MSG_ADD_TO_DATA_DEFINITION,
        &protocol::build_add_to_data_definition(
            TITLE_DEFINE_ID, "TITLE", "", protocol::DATATYPE_STRING256,
        ),
    );
    let req_pkt = protocol::build_packet(
        protocol::MSG_REQUEST_DATA_ON_SIM_OBJECT,
        &protocol::build_request_data_on_sim_object(
            TITLE_DEFINE_ID, TITLE_DEFINE_ID, protocol::PERIOD_VISUAL_FRAME,
        ),
    );
    if conn.send_packet(&add_pkt).await.is_err() || conn.send_packet(&req_pkt).await.is_err() {
        warn!("SimConnect: TITLE registration send failed — reconnecting");
        return false;
    }
    true
}

/// Pick the best camera definition name from an enumerated list, preferring
/// pilot/cockpit-style cameras over external ones.
///
/// Names are reported lower-cased before substring matching.  Order of priority
/// (first match wins):
///   1. exact match on a small set of known good names
///   2. contains "pilot" (most aircraft expose "Pilot" or "Co-Pilot")
///   3. contains "cockpit"
///   4. contains "captain"
///   5. fall back to the first non-empty name (better than nothing)
fn pick_pilot_camera(names: &[String]) -> Option<&str> {
    let lc: Vec<String> = names.iter().map(|s| s.to_lowercase()).collect();
    for (i, l) in lc.iter().enumerate() {
        if matches!(l.as_str(), "pilot" | "cockpit" | "captain") {
            return Some(&names[i]);
        }
    }
    for needle in ["pilot", "cockpit", "captain"] {
        if let Some(i) = lc.iter().position(|l| l.contains(needle)) {
            return Some(&names[i]);
        }
    }
    names.iter().find(|s| !s.is_empty()).map(String::as_str)
}

/// Minimum interval between re-acquire attempts.  The SU5 Camera API silently
/// drops `CameraAcquire` requests sent before the camera subsystem is fully
/// ready (e.g. during world-map load), so we have to retry — but we don't want
/// to spam every status update.
const REACQUIRE_INTERVAL: std::time::Duration = std::time::Duration::from_secs(3);

/// Wait for a RECV_OPEN packet and return the sim version triple.
async fn wait_for_recv_open(conn: &mut connection::SimConnectTcp) -> (u32, u32, u32) {
    loop {
        match conn.recv_packet().await {
            Ok((msg_id, payload)) if msg_id == protocol::RECV_OPEN => {
                return protocol::parse_recv_open_version(&payload).unwrap_or_else(|| {
                    warn!("SimConnect: RECV_OPEN payload too short");
                    (0, 0, 0)
                });
            }
            Ok(_) => continue,
            Err(e) => {
                warn!("SimConnect: recv error during handshake: {e:#}");
                return (0, 0, 0);
            }
        }
    }
}

/// Seat the addon camera at camera definition `def` and request the resulting
/// pilot-eye pose in EYEPOINT (the response defines `pilot_neutral`). The
/// caller pushes `GetPurpose::Neutral` on success.
async fn bootstrap_camera(
    conn: &mut connection::SimConnectTcp,
    def: &str,
) -> Result<(), String> {
    info!("SimConnect: bootstrapping addon camera with {def:?}");
    let pkt = protocol::build_packet(
        protocol::MSG_CAMERA_SET_USING_CAMERA_DEFINITION,
        &protocol::build_camera_set_using_camera_definition(def),
    );
    conn.send_packet(&pkt)
        .await
        .map_err(|e| format!("CameraSetUsingCameraDefinition send failed: {e:#}"))?;
    let get_pkt = protocol::build_packet(
        protocol::MSG_CAMERA_GET,
        &protocol::build_camera_get(protocol::POSITION_REFERENTIAL_EYEPOINT),
    );
    conn.send_packet(&get_pkt)
        .await
        .map_err(|e| format!("CameraGet(EYEPOINT) send failed: {e:#}"))?;
    Ok(())
}

/// Best-effort send of `CameraRelease` so the sim can hand the camera to
/// other add-ons. Errors are logged at debug — we're already on a teardown
/// path and there's nothing useful to do if this fails.
async fn release_camera(conn: &mut connection::SimConnectTcp) {
    let pkt = protocol::build_packet(
        protocol::MSG_CAMERA_RELEASE,
        &protocol::build_camera_release(""),
    );
    if let Err(e) = conn.send_packet(&pkt).await {
        debug!("SimConnect: CameraRelease send failed: {e:#}");
    }
}

// ---------------------------------------------------------------------------
// Per-aircraft seat offsets (MSFS analogue of the X-Plane plugin's seat store)
// ---------------------------------------------------------------------------
// While we hold the SU5 addon camera, ALL in-sim camera controls are locked
// out — including the pilot-seat position. The seat offset composes into every
// CameraSet (`neutral + seat + head-pose`) and is persisted per aircraft,
// keyed by the TITLE SimVar. Same TSV shape as `xplane-seats.tsv`.

/// Metres per seat nudge step (GUI buttons and shortcuts send ±this).
pub const SEAT_STEP: f32 = 0.01;
/// Seat offset clamp per axis, metres (cockpit envelope, matches X-Plane).
pub const SEAT_LIMIT: f32 = 0.6;
/// Seat-map key used until the aircraft TITLE has been identified.
const SEAT_KEY_DEFAULT: &str = "default";
/// Define/request ID for the TITLE SimVar (one SimVar per define_id; IDs ≥ 10
/// deliberately clear of the camera flow, matching the reference impl).
const TITLE_DEFINE_ID: u32 = 21;

/// Seat adjustment command from the GUI / command socket. Deltas only — the
/// connection task is the single authority for clamping and persistence.
#[derive(Clone, Copy, Debug)]
pub enum SeatCmd {
    Nudge { axis: usize, delta_m: f32 },
    Reset,
}

/// Receiver + read-out mirrors handed to the connection task.
struct SeatShared {
    rx:     tokio::sync::mpsc::UnboundedReceiver<SeatCmd>,
    mirror: Arc<[AtomicU32; 3]>,
    title:  Arc<std::sync::Mutex<String>>,
}

/// `$XDG_CONFIG_HOME/headtrack-rs/msfs-seats.tsv` (falls back to `~/.config`).
fn seat_config_path() -> std::path::PathBuf {
    let base = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| format!("{}/.config", std::env::var("HOME").unwrap_or_default()));
    std::path::PathBuf::from(base).join("headtrack-rs/msfs-seats.tsv")
}

/// Read the saved seat map: one `key\tx\ty\tz` line per aircraft.
fn read_seat_map() -> Vec<(String, [f32; 3])> {
    let mut out = Vec::new();
    let Ok(contents) = std::fs::read_to_string(seat_config_path()) else {
        return out;
    };
    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut f = line.split('\t');
        if let (Some(k), Some(x), Some(y), Some(z)) = (f.next(), f.next(), f.next(), f.next()) {
            if let (Ok(x), Ok(y), Ok(z)) = (x.parse(), y.parse(), z.parse()) {
                out.push((k.to_string(), [x, y, z]));
            }
        }
    }
    out
}

/// This aircraft's saved seat offset (zeros if none saved).
fn load_seat_for(key: &str) -> [f32; 3] {
    read_seat_map()
        .into_iter()
        .find(|(k, _)| k == key)
        .map(|(_, o)| o)
        .unwrap_or([0.0; 3])
}

/// Persist a seat offset under `key`, preserving other aircraft entries.
fn save_seat_for(key: &str, off: [f32; 3]) {
    let mut map = read_seat_map();
    if let Some(entry) = map.iter_mut().find(|(k, _)| k == key) {
        entry.1 = off;
    } else {
        map.push((key.to_string(), off));
    }
    let mut text =
        String::from("# headtrack-rs MSFS seat offsets — key<TAB>x<TAB>y<TAB>z (metres, EYEPOINT)\n");
    for (k, o) in &map {
        text.push_str(&format!("{}\t{:.4}\t{:.4}\t{:.4}\n", k, o[0], o[1], o[2]));
    }
    let path = seat_config_path();
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let _ = std::fs::write(&path, text);
}

/// TSV-safe seat-map key from an aircraft TITLE.
fn sanitize_seat_key(title: &str) -> String {
    let key = title.replace(['\t', '\n', '\r'], " ").trim().to_string();
    if key.is_empty() { SEAT_KEY_DEFAULT.to_string() } else { key }
}

/// Copy the live seat offset into the f32-bit atomic mirror (GUI read-out).
fn mirror_seat(mirror: &Arc<[AtomicU32; 3]>, seat: [f32; 3]) {
    for i in 0..3 {
        mirror[i].store(seat[i].to_bits(), Ordering::Relaxed);
    }
}

/// Drain pending seat commands: clamp, mirror, persist. Returns whether any
/// command was applied.
fn apply_seat_cmds(shared: &mut SeatShared, seat: &mut [f32; 3], key: &str) -> bool {
    let mut changed = false;
    while let Ok(cmd) = shared.rx.try_recv() {
        match cmd {
            SeatCmd::Nudge { axis, delta_m } if axis < 3 && delta_m.is_finite() => {
                seat[axis] = (seat[axis] + delta_m).clamp(-SEAT_LIMIT, SEAT_LIMIT);
                changed = true;
            }
            SeatCmd::Reset => {
                *seat = [0.0; 3];
                changed = true;
            }
            SeatCmd::Nudge { .. } => {}
        }
    }
    if changed {
        mirror_seat(&shared.mirror, *seat);
        save_seat_for(key, *seat);
        info!(
            "SimConnect: seat offset for '{key}' = [{:.3}, {:.3}, {:.3}] m",
            seat[0], seat[1], seat[2]
        );
    }
    changed
}

async fn run_connection(
    addr: String,
    pose_rx: watch::Receiver<Pose>,
    connected: Arc<AtomicBool>,
    enabled: Arc<AtomicBool>,
    smoothing: Arc<AtomicU32>,
    mut seat_shared: SeatShared,
) {
    // Optional per-frame trace (instrument I2). One file for the task's
    // lifetime — reconnects keep appending on the same time base.
    let mut trace = TraceWriter::from_env();

    // Per-aircraft seat offset — task-lifetime state so it survives reconnects.
    let mut seat_key = String::from(SEAT_KEY_DEFAULT);
    let mut seat: [f32; 3] = load_seat_for(&seat_key);
    mirror_seat(&seat_shared.mirror, seat);

    // Whether the current run of connect attempts has already been announced.
    // Reset on every successful connection so each new outage gets one line.
    let mut announced_attempt = false;

    loop {
        // Seat nudges apply (and persist) even while disconnected or paused.
        apply_seat_cmds(&mut seat_shared, &mut seat, &seat_key);

        // When tracking is paused, stay disconnected so MSFS releases camera ownership.
        if !enabled.load(Ordering::Relaxed) {
            tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
            continue;
        }

        // Announce once per disconnected episode, not once per retry. MSFS is
        // usually not running, so this loop spins every 2 s for the whole
        // session — at INFO that buried every other line in the log.
        if announced_attempt {
            debug!("SimConnect: retrying {addr}");
        } else {
            info!("SimConnect: connecting to {addr}");
            announced_attempt = true;
        }

        let mut conn = match connection::SimConnectTcp::connect(&addr).await {
            Ok(c) => c,
            Err(e) => {
                debug!("SimConnect: connect failed: {e:#} — retrying in 2s");
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                continue;
            }
        };

        // Handshake: send OPEN, wait for RECV_OPEN.
        let open_pkt = protocol::build_packet(protocol::MSG_OPEN, &protocol::build_open(CLIENT_ID));
        if let Err(e) = conn.send_packet(&open_pkt).await {
            warn!("SimConnect: send OPEN failed: {e:#}");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        let (major, minor, build) = wait_for_recv_open(&mut conn).await;
        if major == 0 {
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }
        if !protocol::version_ok(major, minor, build) {
            warn!(
                "SimConnect: sim version {major}.{minor}.{build} < {}.{}.{} — Camera API unavailable",
                protocol::MIN_VERSION_MAJOR, protocol::MIN_VERSION_MINOR, protocol::MIN_VERSION_BUILD,
            );
            tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
            continue;
        }

        info!("SimConnect: connected — sim version {major}.{minor}.{build}");
        announced_attempt = false;

        // The handshake, the system-event subscriptions, the camera-status
        // subscription and one `CameraAcquire` go out here. Nothing else:
        // everything that reaches into the *user aircraft* waits for the sim to
        // confirm we hold the camera — see [`BringUp`].
        if !subscribe_frame(&mut conn).await || !subscribe_sim_state(&mut conn).await {
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        let mut ready = SimReady::new();
        // Where we are on the bring-up ladder. `Streaming` is unlocked by the
        // sim's own camera status — never by a lifecycle event and never by a
        // timer. See [`BringUp`].
        let mut stage = if ready.bypassed { BringUp::Streaming } else { BringUp::WaitAcquired };
        if ready.bypassed {
            if !bring_up_camera_unsafe(&mut conn).await {
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                continue;
            }
        } else {
            // Subscribe, then ask. The subscribe only registers us for *future*
            // changes — MSFS sends nothing in reply to it (measured 2026-08-20:
            // six subscribes, 30 s, zero packets). `CameraAcquire` is answered,
            // so it is both the claim and the only available state query, and a
            // client connecting into an already-live flight has no other way to
            // learn `bGameControlled`. Neither call is aircraft-facing, which is
            // what makes this safe to send at connect.
            info!("SimConnect: connected — subscribing to camera status and claiming the camera");
            if !subscribe_camera_status(&mut conn).await {
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                continue;
            }
            if let Err(e) = send_camera_acquire(&mut conn).await {
                warn!("SimConnect: {e}");
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                continue;
            }
        }

        // Set only once every subscription is in place. Each failure above
        // `continue`s straight back to the reconnect loop without running the
        // `connected.store(false)` at the bottom, so flagging the connection
        // any earlier leaves the GUI reporting "connected" through a bring-up
        // that never happened.
        connected.store(true, Ordering::Relaxed);

        // Rate limit for the "still waiting" reminder while the ladder is
        // parked below Streaming.
        let mut last_wait_log = Instant::now();
        // Has the sim ever answered with a camera status on this connection?
        // The first one is logged at INFO with its latency: MSFS reports status
        // only on change, so how long that takes and what it says is the single
        // most diagnostic line in a bring-up that did not work.
        let mut camera_status_seen = false;
        let subscribed_at = Instant::now();
        // Rate limit for the add-on-camera-disabled warning. Separate from
        // `last_acquire` because the acquire retry resets that one, which would
        // suppress the warning exactly when it is most needed.
        let mut last_disabled_warn: Option<Instant> = None;

        // Camera ownership state. Both must be true (acquired) and false
        // (game_controlled) for us to send.
        let mut acquired_state: u32 = protocol::CAMERA_NOT_ACQUIRED;
        let mut game_controlled: bool = true;
        // Track the last `CameraAcquire` so the Frame-tick retry cannot spam.
        // One acquire already went out at connect, so starting the clock here
        // spaces the first retry a full REACQUIRE_INTERVAL behind it.
        let mut last_acquire: Instant = Instant::now();
        // Per-code exception counters, warnings rate-limited (never suppressed).
        let mut exception_counts: HashMap<u32, u64> = HashMap::new();
        let mut exception_last_warn: Option<Instant> = None;
        // Camera definition we'll use to bootstrap the addon camera into the
        // pilot eyepoint. Populated when RECV_CAMERA_DEFINITION_LIST arrives;
        // sent via MSG_CAMERA_SET_USING_CAMERA_DEFINITION on every transition
        // into `acquired=1` (so the user-disable/re-enable cycle re-seeds it).
        let mut pending_bootstrap_def: Option<String> = None;
        // Pilot eyepoint pose in the EYEPOINT referential, captured via
        // CameraGet after each bootstrap.  Per-frame CameraSet sends
        // `(neutral + pose_to_camera())` so head movement composes on top of
        // the bootstrap placement.  EYEPOINT is symmetric Get/Set (verified
        // 2026-05-07) — sending back the captured neutral keeps the camera at
        // pilot eye, unlike SIMOBJECT_DATUM which lands in the nose.
        let mut pilot_neutral: Option<([f64; 3], [f32; 3])> = None;
        // Output-side smoothing state: low-pass the per-frame camera offset
        // toward the latest pose (mirrors the XP plugin's LERP). Re-initialised
        // on each (re)acquire so the bootstrap placement never causes a jump.
        let mut smoothed_offset: Option<([f64; 3], [f32; 3])> = None;
        // Wall-clock fallback for the smoothing dt — only used when a Frame
        // event doesn't carry a usable fFrameRate. Updated on EVERY Frame
        // event, including gated ones, so a gate re-open can never produce a
        // giant dt (the old code updated it only on the send path, and the
        // first frame after a menu snapped the camera to the raw target).
        let mut last_frame = Instant::now();
        // Whether this connection's Frame events carry fFrameRate; logged once.
        let mut frame_rate_seen: Option<bool> = None;
        // FIFO of in-flight CameraGet purposes (see `GetPurpose`).
        let mut pending_gets: VecDeque<GetPurpose> = VecDeque::new();
        // Periodic readback cadence + the absolute values last sent, for
        // comparing what we sent against what MSFS actually applied.
        let mut last_readback = Instant::now();
        let mut last_sent: Option<([f64; 3], [f32; 3])> = None;
        let mut readback_last_warn: Option<Instant> = None;

        let disconnect_reason = loop {
            // Drain seat nudges once per received packet — Frame events arrive
            // every rendered frame while connected, so worst-case latency for a
            // nudge is one frame.
            apply_seat_cmds(&mut seat_shared, &mut seat, &seat_key);

            let (msg_id, payload) = match conn.recv_packet().await {
                Ok(p) => p,
                Err(e) => break format!("recv error: {e:#}"),
            };

            match msg_id {
                protocol::RECV_EVENT => {
                    let Some((event_id, _dw_data)) = protocol::parse_recv_event(&payload) else {
                        continue;
                    };
                    if event_id != FRAME_EVENT_ID {
                        continue;
                    }

                    // dt for the smoothing step — computed FIRST, before any
                    // gate can `continue`, so the wall-clock fallback never
                    // accumulates across skipped frames.
                    //
                    // Preferred source: the sim's own frame rate, delivered in
                    // this very packet (SIMCONNECT_RECV_EVENT_FRAME appends
                    // fFrameRate + fSimSpeed to the base event). That is the
                    // sim's render clock — the exact analogue of X-Plane's
                    // flight-loop dt. The wall clock between TCP drains is only
                    // a fallback: TCP coalescing hands one frame ~30 ms and the
                    // next ~0.2 ms, which renders as a freeze + double-step.
                    let wall_dt = last_frame.elapsed().as_secs_f32();
                    last_frame = Instant::now();
                    let frame_rate = protocol::parse_recv_event_frame_rate(&payload)
                        .map(|(fr, _sim_speed)| fr)
                        .filter(|fr| fr.is_finite() && (1.0..=1000.0).contains(fr));
                    if frame_rate_seen.is_none() {
                        frame_rate_seen = Some(frame_rate.is_some());
                        match frame_rate {
                            Some(fr) => info!(
                                "SimConnect: Frame events carry fFrameRate ({fr:.0} fps) — \
                                 smoothing on the sim's clock"
                            ),
                            None => warn!(
                                "SimConnect: Frame events do not carry fFrameRate — \
                                 smoothing on wall-clock dt (TCP-arrival jitter applies)"
                            ),
                        }
                    }
                    let dt = frame_rate.map_or_else(|| wall_dt.clamp(0.0, 0.1), |fr| 1.0 / fr);

                    if !enabled.load(Ordering::Relaxed) {
                        // Tracking paused: release the camera and disconnect.
                        // Unconditional — an acquire goes out at connect, so a
                        // release is always owed, whether or not the sim ever
                        // confirmed the claim.
                        release_camera(&mut conn).await;
                        break "tracking paused — camera released".to_string();
                    }

                    // Retry the acquire from the FRAME tick, not from the
                    // status handler, and at EVERY rung. Two reasons, both
                    // measured on 2026-08-20:
                    //
                    // 1. MSFS reports camera status only on change, so a state
                    //    that needs another acquire is exactly the state that
                    //    will never generate the packet to trigger one. Frame
                    //    events arrive every rendered frame regardless.
                    // 2. The camera can be taken away *after* we start
                    //    streaming — the in-game Camera ▸ Addons toggle does
                    //    precisely that — and that is when a retry matters
                    //    most, since there is no restart to paper over it.
                    if should_acquire(acquired_state, game_controlled)
                        && last_acquire.elapsed() >= REACQUIRE_INTERVAL
                    {
                        last_acquire = Instant::now();
                        info!(
                            "SimConnect: camera is free but not ours \
                             (acquired={acquired_state}, stage {stage:?}) — re-issuing \
                             CameraAcquire"
                        );
                        if let Err(e) = send_camera_acquire(&mut conn).await {
                            break e;
                        }
                    }

                    if stage != BringUp::Streaming {
                        // Say so periodically: "connected but not tracking" is
                        // otherwise indistinguishable from a fault.
                        if last_wait_log.elapsed() >= WAIT_LOG_INTERVAL {
                            last_wait_log = Instant::now();
                            info!(
                                "SimConnect: waiting to claim the camera (stage {stage:?}, \
                                 sim running={}, acquired={acquired_state}, \
                                 game_controlled={game_controlled}, camera status \
                                 seen={camera_status_seen}) — start a flight, and check the \
                                 in-game Camera ▸ Addons toggle if this persists in the cockpit",
                                ready.live
                            );
                        }
                        continue;
                    }

                    // Gate sends on the camera status the sim told us about.
                    if acquired_state != protocol::CAMERA_ACQUIRED || game_controlled {
                        debug!(
                            "SimConnect: skipping CameraSet — acquired={acquired_state} \
                             game_controlled={game_controlled}"
                        );
                        if let Some(tr) = trace.as_mut() {
                            tr.event(&format!(
                                "gate-skip acquired={acquired_state} game_controlled={game_controlled}"
                            ));
                        }
                        continue;
                    }

                    // Wait for the bootstrap+CameraGet round-trip so we have a
                    // neutral to compose on top of.  Per-frame send is
                    // `neutral + pose_to_camera()` in the EYEPOINT referential.
                    let Some((neutral_pos, neutral_pbh)) = pilot_neutral else {
                        continue;
                    };

                    let pose = *pose_rx.borrow();
                    let (target_pos, target_pbh) = pose_to_camera(&pose);

                    // True exponential smoothing toward the target offset. The
                    // old first-order form `(speed * dt).clamp(0.0, 1.0)`
                    // reached alpha = 1.0 for any dt >= 1/speed (66.7 ms at the
                    // default speed 15) — an instant snap to the raw target on
                    // every frame-time spike. This form stays below 1.0 for all
                    // finite dt and composes consistently across uneven frames.
                    let speed = f32::from_bits(smoothing.load(Ordering::Relaxed));
                    let alpha = 1.0 - (-speed * dt).exp();
                    let (sp, sb) = smoothed_offset.get_or_insert((target_pos, target_pbh));
                    for i in 0..3 {
                        sp[i] += alpha as f64 * (target_pos[i] - sp[i]);
                        sb[i] += alpha * (target_pbh[i] - sb[i]);
                    }

                    // Seat offset composes OUTSIDE the smoothing accumulator:
                    // nudges apply instantly, and a re-acquire (which drops
                    // `smoothed_offset`) never washes the seat position out.
                    let position = [
                        neutral_pos[0] + seat[0] as f64 + sp[0],
                        neutral_pos[1] + seat[1] as f64 + sp[1],
                        neutral_pos[2] + seat[2] as f64 + sp[2],
                    ];
                    let pbh = [
                        neutral_pbh[0] + sb[0],
                        neutral_pbh[1] + sb[1],
                        neutral_pbh[2] + sb[2],
                    ];

                    let payload = protocol::build_camera_set(
                        position,
                        protocol::POSITION_REFERENTIAL_EYEPOINT,
                        0, // user aircraft
                        pbh,
                        protocol::POSITION_REFERENTIAL_EYEPOINT,
                        0,
                        CAMERA_IGNORE_FIELD,
                        protocol::CAMERA_DATA_MASK_POSITION | protocol::CAMERA_DATA_MASK_ROTATION,
                    );
                    let pkt = protocol::build_packet(protocol::MSG_CAMERA_SET, &payload);
                    if let Err(e) = conn.send_packet(&pkt).await {
                        break format!("send CameraSet failed: {e:#}");
                    }
                    last_sent = Some((position, pbh));

                    if let Some(tr) = trace.as_mut() {
                        tr.row(
                            frame_rate.unwrap_or(0.0),
                            dt,
                            alpha,
                            &(target_pos, target_pbh),
                            &(position, pbh),
                        );
                    }

                    // Periodic readback: ask where the camera actually is so
                    // drift between what we send and what the sim applies is
                    // visible (`pilot_neutral` is otherwise never re-checked).
                    if last_readback.elapsed() >= READBACK_INTERVAL {
                        last_readback = Instant::now();
                        let get_pkt = protocol::build_packet(
                            protocol::MSG_CAMERA_GET,
                            &protocol::build_camera_get(protocol::POSITION_REFERENTIAL_EYEPOINT),
                        );
                        if let Err(e) = conn.send_packet(&get_pkt).await {
                            break format!("CameraGet(readback) send failed: {e:#}");
                        }
                        pending_gets.push_back(GetPurpose::Readback);
                    }
                }
                protocol::RECV_EVENT_PLAIN => {
                    // "Sim" / "SimStart" / "SimStop" — the readiness signals.
                    if let Some((event_id, dw_data)) = protocol::parse_recv_event(&payload) {
                        ready.on_event(event_id, dw_data);
                    }
                }
                protocol::RECV_CAMERA_STATUS => {
                    if let Some((new_acquired, new_game_ctl)) = protocol::parse_recv_camera_status(&payload) {
                        if !camera_status_seen {
                            camera_status_seen = true;
                            // The one line that says whether MSFS answers a
                            // subscription or only reports changes. Everything
                            // below this point in the bring-up depends on it.
                            info!(
                                "SimConnect: first camera status {:.1}s after connect — \
                                 acquired={new_acquired} game_controlled={new_game_ctl}",
                                subscribed_at.elapsed().as_secs_f32()
                            );
                        }
                        let prev_acquired = acquired_state;
                        if new_acquired != acquired_state || new_game_ctl != game_controlled {
                            info!(
                                "SimConnect: camera status acquired={new_acquired} \
                                 game_controlled={new_game_ctl} (was acquired={acquired_state} \
                                 game_controlled={game_controlled})"
                            );
                            acquired_state = new_acquired;
                            game_controlled = new_game_ctl;
                        }

                        // On every transition INTO ACQUIRED, seat the addon
                        // camera at the pilot definition.  This handles both
                        // the initial acquire (after the user enables the
                        // addon-camera toggle) and re-acquires after a
                        // disable/enable cycle.
                        if prev_acquired != protocol::CAMERA_ACQUIRED
                            && acquired_state == protocol::CAMERA_ACQUIRED
                        {
                            // Invalidate cached neutral so per-frame sends pause
                            // until the new CameraGet response arrives. Drop the
                            // smoothing state too, so it re-seats on the next
                            // frame, and the last-sent record so a readback from
                            // before the transition can't produce a stale warn.
                            pilot_neutral = None;
                            smoothed_offset = None;
                            last_sent = None;
                            last_frame = Instant::now();

                            if let Some(def) = pending_bootstrap_def.as_deref() {
                                match bootstrap_camera(&mut conn, def).await {
                                    Ok(()) => pending_gets.push_back(GetPurpose::Neutral),
                                    Err(e) => break e,
                                }
                            } else {
                                // The definition list hasn't arrived yet — its
                                // handler bootstraps the moment it does.
                                debug!("SimConnect: acquired but no cached camera definition yet");
                            }
                        }

                        // ── Bring-up ladder ─────────────────────────────────
                        //
                        // The sim's own report drives this, because it is the
                        // only thing that has proved trustworthy: `Sim = 1`
                        // arrives during the loading screen, but
                        // `bGameControlled` was still `true` there (captured
                        // 2026-08-17), and it is `false` exactly when the game
                        // is not driving the camera itself. The decision itself
                        // is in [`next_rung`], which is pure and tested.
                        //
                        // Evaluated on EVERY status packet, not only on ones
                        // that changed a value: the first status of a
                        // connection can carry `game_controlled == false`
                        // already (we connected mid-flight) and would otherwise
                        // match the local defaults and be discarded.
                        if next_rung(stage, acquired_state, game_controlled)
                            == Some(BringUp::Streaming)
                        {
                            info!(
                                "SimConnect: camera acquired — enumerating camera \
                                 definitions and registering TITLE"
                            );
                            if !request_aircraft_state(&mut conn).await {
                                break "aircraft-state request send failed".to_string();
                            }
                            stage = BringUp::Streaming;
                        }

                        // The re-acquire retry deliberately does NOT live here
                        // — see [`should_acquire`]. It runs on the Frame tick,
                        // because the states that need a retry are the ones
                        // that produce no further status packets.
                        //
                        // `CAMERA_USER_DISABLED` still warrants a word: it is
                        // the single most common "it just stopped working"
                        // cause. Note the retry clears it in practice (an
                        // acquire moves the sim from 3 to 0), so this only
                        // stays on screen when the in-game toggle really is off.
                        if acquired_state == protocol::CAMERA_USER_DISABLED
                            && last_disabled_warn
                                .is_none_or(|t| t.elapsed() >= WAIT_LOG_INTERVAL)
                        {
                            last_disabled_warn = Some(Instant::now());
                            warn!(
                                "SimConnect: the sim reports the add-on camera as unavailable \
                                 (acquired=3) — retrying; if this persists, enable it in-game \
                                 under Camera ▸ Addons"
                            );
                        }
                    }
                }
                protocol::RECV_CAMERA_DATA => {
                    if let Some((pos, pbh)) = protocol::parse_recv_camera_data(&payload) {
                        match pending_gets.pop_front() {
                            Some(GetPurpose::Readback) => {
                                // Compare what MSFS applied against what we last
                                // sent. POSITION ONLY: measured 2026-07-31,
                                // position agrees to sub-mm on every readback,
                                // but the Get's PBH tracks aircraft attitude
                                // (mismatches up to 74° with the head still) —
                                // rotation is not comparable frame-to-frame, so
                                // it is logged at debug and never warned on.
                                // Diagnostic only — no automatic re-sync, which
                                // would itself move the camera.
                                if let Some((sent_pos, sent_pbh)) = last_sent {
                                    let dpos = (0..3)
                                        .map(|i| (pos[i] - sent_pos[i]).abs())
                                        .fold(0.0f64, f64::max);
                                    let dpbh = (0..3)
                                        .map(|i| angle_delta_deg(pbh[i], sent_pbh[i]))
                                        .fold(0.0f32, f32::max);
                                    if dpos > 0.02 {
                                        if readback_last_warn
                                            .is_none_or(|t| t.elapsed() >= Duration::from_secs(60))
                                        {
                                            warn!(
                                                "SimConnect: camera POSITION readback deviates from \
                                                 last CameraSet — Δpos {dpos:.3} m (sim moving the \
                                                 eyepoint under us, or rejecting sets)"
                                            );
                                            readback_last_warn = Some(Instant::now());
                                        }
                                        if let Some(tr) = trace.as_mut() {
                                            tr.event(&format!(
                                                "readback-mismatch dpos_m={dpos:.4} dpbh_deg={dpbh:.3}"
                                            ));
                                        }
                                    } else {
                                        debug!(
                                            "SimConnect: camera readback position ok \
                                             (Δpos {dpos:.4} m; Δpbh {dpbh:.2}° — informational)"
                                        );
                                    }
                                }
                            }
                            // Bootstrap response — defines the pilot neutral.
                            // `None` (unexpected/unsolicited data) is treated the
                            // same as the legacy behaviour.
                            Some(GetPurpose::Neutral) | None => {
                                info!(
                                    "SimConnect: pilot neutral position={:?}m pbh={:?}deg (EYEPOINT)",
                                    pos, pbh
                                );
                                pilot_neutral = Some((pos, pbh));
                            }
                        }
                    }
                }
                protocol::RECV_CAMERA_DEFINITION_LIST => {
                    if let Some(names) = protocol::parse_recv_camera_definition_list(&payload) {
                        info!("SimConnect: camera definitions ({}): {:?}", names.len(), names);
                        match pick_pilot_camera(&names) {
                            Some(pick) => {
                                info!(
                                    "SimConnect: will bootstrap addon camera with {:?} on next acquire",
                                    pick
                                );
                                pending_bootstrap_def = Some(pick.to_string());
                                // Fast re-grab race: on a mid-flight re-enable
                                // MSFS grants the camera BEFORE this list
                                // arrives, so the acquire transition found no
                                // definition and skipped the bootstrap. If we
                                // are acquired and still waiting for a neutral,
                                // bootstrap NOW — otherwise the camera sits
                                // dead until the user toggles the in-game
                                // Addons feature to force another transition.
                                if acquired_state == protocol::CAMERA_ACQUIRED
                                    && !game_controlled
                                    && pilot_neutral.is_none()
                                {
                                    match bootstrap_camera(&mut conn, pick).await {
                                        Ok(()) => pending_gets.push_back(GetPurpose::Neutral),
                                        Err(e) => break e,
                                    }
                                }
                            }
                            None => warn!(
                                "SimConnect: no usable camera definitions returned — \
                                 staying at default addon position"
                            ),
                        }
                    }
                }
                protocol::RECV_SIMOBJECT_DATA => {
                    if protocol::parse_simobject_data_request_id(&payload) == Some(TITLE_DEFINE_ID) {
                        if let Some(title) = protocol::parse_simobject_data_string256(&payload) {
                            let key = sanitize_seat_key(&title);
                            // TITLE arrives every visual frame — act on change only.
                            if key != seat_key {
                                seat_key = key;
                                seat = load_seat_for(&seat_key);
                                mirror_seat(&seat_shared.mirror, seat);
                                if let Ok(mut t) = seat_shared.title.lock() {
                                    *t = title.clone();
                                }
                                info!(
                                    "SimConnect: aircraft '{title}' — seat offset \
                                     [{:.3}, {:.3}, {:.3}] m",
                                    seat[0], seat[1], seat[2]
                                );
                            }
                        }
                    }
                }
                protocol::RECV_EXCEPTION => {
                    if let Some(code) = protocol::parse_recv_exception(&payload) {
                        let n = {
                            let c = exception_counts.entry(code).or_insert(0);
                            *c += 1;
                            *c
                        };
                        // First few per code warn immediately; after that,
                        // rate-limit to one summary per interval. Never fully
                        // suppressed — a persistent per-frame exception (e.g.
                        // every CameraSet rejected) must stay visible.
                        let due = exception_last_warn
                            .is_none_or(|t| t.elapsed() >= EXCEPTION_WARN_INTERVAL);
                        if n <= 3 {
                            warn!(
                                "SimConnect: RECV_EXCEPTION code={code} — request invalid \
                                 or SimVar not writable (#{n})"
                            );
                            exception_last_warn = Some(Instant::now());
                        } else if due {
                            warn!(
                                "SimConnect: RECV_EXCEPTION code={code} still firing — \
                                 {n} total this connection"
                            );
                            exception_last_warn = Some(Instant::now());
                        } else {
                            debug!("SimConnect: RECV_EXCEPTION code={code}");
                        }
                    }
                }
                _ => {}
            }
        };

        connected.store(false, Ordering::Relaxed);
        if !exception_counts.is_empty() {
            warn!("SimConnect: exception totals this connection (code: count): {exception_counts:?}");
        }
        if enabled.load(Ordering::Relaxed) {
            // Closing the sim is a normal lifecycle event, not a fault — INFO
            // keeps `RUST_LOG=warn` meaning "something is actually wrong".
            info!("SimConnect: disconnected ({disconnect_reason}) — retrying in 2s");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
        } else {
            info!("SimConnect: {disconnect_reason} — will reconnect when tracking resumes");
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// SimConnect output adapter.
///
/// Call `write()` from the engine run loop each frame.  A background tokio
/// task maintains the connection and sends poses on each sim Frame event.
pub struct SimConnectOutput {
    tx: watch::Sender<Pose>,
    connected: Arc<AtomicBool>,
    enabled: Arc<AtomicBool>,
    /// Output-side smoothing LERP speed (`f32` bits). Higher = snappier.
    smoothing: Arc<AtomicU32>,
    /// Seat adjustment commands into the connection task (the authority).
    seat_tx: tokio::sync::mpsc::UnboundedSender<SeatCmd>,
    /// Read-out mirror of the live seat offset (`f32` bits, metres).
    seat: Arc<[AtomicU32; 3]>,
    /// Read-out mirror of the current aircraft TITLE ("" until identified).
    title: Arc<std::sync::Mutex<String>>,
}

/// Default smoothing LERP speed — matches the X-Plane plugin's `LERP_SPEED`.
pub const DEFAULT_SMOOTHING: f32 = 15.0;

impl SimConnectOutput {
    /// Create the output and start the background connection task.
    ///
    /// `port` is the TCP port configured in SimConnect.xml (default 5557).
    pub fn open(port: u16) -> anyhow::Result<Self> {
        let (tx, rx) = watch::channel(Pose::zero(0));
        let connected = Arc::new(AtomicBool::new(false));
        let enabled = Arc::new(AtomicBool::new(true));
        let smoothing = Arc::new(AtomicU32::new(DEFAULT_SMOOTHING.to_bits()));
        let (seat_tx, seat_rx) = tokio::sync::mpsc::unbounded_channel();
        let seat: Arc<[AtomicU32; 3]> = Arc::new(std::array::from_fn(|_| AtomicU32::new(0)));
        let title = Arc::new(std::sync::Mutex::new(String::new()));
        let addr = format!("127.0.0.1:{port}");
        // Dedicated thread + single-thread runtime. The engine's 2-worker
        // runtime wraps the whole pipeline + file I/O in `block_in_place` on
        // every tracker frame, and any scheduling delay there used to land
        // directly in the smoothing dt (and delay the CameraSet send). On its
        // own runtime, nothing the engine does can stall a Frame drain.
        {
            let connected = Arc::clone(&connected);
            let enabled = Arc::clone(&enabled);
            let smoothing = Arc::clone(&smoothing);
            let seat_shared = SeatShared {
                rx: seat_rx,
                mirror: Arc::clone(&seat),
                title: Arc::clone(&title),
            };
            std::thread::Builder::new()
                .name("simconnect".into())
                .spawn(move || {
                    match tokio::runtime::Builder::new_current_thread().enable_all().build() {
                        Ok(rt) => rt.block_on(run_connection(
                            addr, rx, connected, enabled, smoothing, seat_shared,
                        )),
                        Err(e) => warn!("SimConnect: failed to build runtime: {e:#}"),
                    }
                })
                .map_err(|e| anyhow::anyhow!("failed to spawn SimConnect thread: {e}"))?;
        }
        Ok(Self { tx, connected, enabled, smoothing, seat_tx, seat, title })
    }

    /// Returns `true` while MSFS is connected and the Camera API handshake has succeeded.
    #[inline]
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    /// Enable or disable CameraSet streaming.
    ///
    /// Disabling releases the camera and disconnects the TCP session, so the
    /// sim can hand the camera back to the user (or other add-ons). Enabling
    /// reconnects and re-acquires.
    #[inline]
    pub fn set_enabled(&self, v: bool) {
        self.enabled.store(v, Ordering::Relaxed);
    }

    /// Set the output-side smoothing LERP speed (higher = snappier, lower =
    /// smoother). Mirrors the X-Plane plugin's per-frame LERP; applied live.
    #[inline]
    pub fn set_smoothing(&self, speed: f32) {
        self.smoothing.store(speed.max(0.0).to_bits(), Ordering::Relaxed);
    }

    /// Nudge one seat axis by `delta_m` metres (clamped ±[`SEAT_LIMIT`] by the
    /// connection task, which also persists it under the current aircraft).
    #[inline]
    pub fn nudge_seat(&self, axis: usize, delta_m: f32) {
        let _ = self.seat_tx.send(SeatCmd::Nudge { axis, delta_m });
    }

    /// Reset the current aircraft's seat offset to zero (persisted).
    #[inline]
    pub fn reset_seat(&self) {
        let _ = self.seat_tx.send(SeatCmd::Reset);
    }

    /// The live seat offset in metres (read-out mirror).
    #[inline]
    pub fn seat_offset(&self) -> [f32; 3] {
        std::array::from_fn(|i| f32::from_bits(self.seat[i].load(Ordering::Relaxed)))
    }

    /// The current aircraft TITLE ("" until identified).
    #[inline]
    pub fn current_title(&self) -> String {
        self.title.lock().map(|t| t.clone()).unwrap_or_default()
    }

    /// Whether CameraSet streaming is currently enabled.
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    /// Flip streaming on/off; returns the NEW state. Used by the `msfs-enabled
    /// toggle` command — a stateless caller can't know the current value.
    #[inline]
    pub fn toggle_enabled(&self) -> bool {
        !self.enabled.fetch_xor(true, Ordering::Relaxed)
    }

    /// Signal the background task to release the camera on its next frame.
    ///
    /// Call this before dropping `SimConnectOutput` on graceful engine shutdown.
    #[inline]
    pub fn request_reset(&self) {
        // Disabling triggers the pause path in run_connection: CameraRelease, then disconnect.
        self.enabled.store(false, Ordering::Relaxed);
    }

    /// Push the latest pose to the background task (non-blocking).
    #[inline]
    pub fn write(&self, pose: &Pose) {
        let _ = self.tx.send(*pose);
    }
}

#[cfg(test)]
mod seat_and_mapping_tests {
    use super::*;

    /// Axis conventions are the classic silent bug in this project: nothing
    /// fails, the camera just moves the wrong way, and it takes a flight to
    /// notice. These were established in-cockpit — pin them.
    #[test]
    fn pose_to_camera_sign_conventions() {
        let pose = Pose {
            yaw: 10.0,
            pitch: 20.0,
            roll: 30.0,
            x: 100.0,
            y: 200.0,
            z: 300.0,
            timestamp_us: 0,
        };
        let (pos, pbh) = pose_to_camera(&pose);

        // Position: pass-through, millimetres → metres.
        assert!((pos[0] - 0.1).abs() < 1e-9, "x mm→m pass-through");
        assert!((pos[1] - 0.2).abs() < 1e-9, "y mm→m pass-through");
        assert!((pos[2] - 0.3).abs() < 1e-9, "z mm→m pass-through");

        // PBH is { Pitch, Bank, Heading }: pitch and bank inverted, heading not.
        assert!((pbh[0] - (-20.0)).abs() < 1e-6, "pitch must be inverted");
        assert!((pbh[1] - (-30.0)).abs() < 1e-6, "bank/roll must be inverted");
        assert!((pbh[2] - 10.0).abs() < 1e-6, "heading/yaw passes through");
    }

    #[test]
    fn pose_to_camera_zero_pose_is_zero_offset() {
        let (pos, pbh) = pose_to_camera(&Pose::zero(0));
        assert_eq!(pos, [0.0, 0.0, 0.0]);
        assert_eq!(pbh, [0.0, 0.0, 0.0]);
    }

    /// The seat key comes from the aircraft TITLE SimVar, which is arbitrary
    /// vendor text going straight into a TAB-separated file. A tab or newline
    /// in a title would corrupt every subsequent entry in the store.
    #[test]
    fn seat_keys_cannot_corrupt_the_tsv() {
        assert_eq!(sanitize_seat_key("Cessna 172"), "Cessna 172");
        assert_eq!(sanitize_seat_key("  padded  "), "padded");
        assert!(!sanitize_seat_key("bad\tkey").contains('\t'));
        assert!(!sanitize_seat_key("bad\nkey").contains('\n'));
        assert!(!sanitize_seat_key("bad\r\nkey").contains('\r'));
        // An empty or whitespace-only title must not produce an empty key.
        assert_eq!(sanitize_seat_key(""), SEAT_KEY_DEFAULT);
        assert_eq!(sanitize_seat_key("   "), SEAT_KEY_DEFAULT);
    }

    /// Seat offsets are per-aircraft and persist across sessions, so saving one
    /// aircraft must not disturb another's. The only test here that touches the
    /// environment — it owns `XDG_CONFIG_HOME` for its duration.
    #[test]
    fn seat_store_round_trips_and_preserves_other_aircraft() {
        let dir = std::env::temp_dir().join(format!("ht-seat-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let prev = std::env::var("XDG_CONFIG_HOME").ok();
        std::env::set_var("XDG_CONFIG_HOME", &dir);

        // Nothing saved yet → zeros, not an error.
        assert_eq!(load_seat_for("Cessna 172"), [0.0; 3]);

        save_seat_for("Cessna 172", [0.10, -0.05, 0.25]);
        save_seat_for("Boeing 747", [-0.30, 0.02, 0.00]);

        let c = load_seat_for("Cessna 172");
        assert!((c[0] - 0.10).abs() < 1e-4 && (c[1] + 0.05).abs() < 1e-4 && (c[2] - 0.25).abs() < 1e-4);
        let b = load_seat_for("Boeing 747");
        assert!((b[0] + 0.30).abs() < 1e-4 && (b[1] - 0.02).abs() < 1e-4 && b[2].abs() < 1e-4);

        // Updating one entry must leave the other intact.
        save_seat_for("Cessna 172", [0.0, 0.0, 0.40]);
        assert!((load_seat_for("Cessna 172")[2] - 0.40).abs() < 1e-4);
        assert!((load_seat_for("Boeing 747")[0] + 0.30).abs() < 1e-4);

        // A title with a tab must round-trip through the sanitizer, not split
        // the row.
        let key = sanitize_seat_key("Weird\tPlane");
        save_seat_for(&key, [0.01, 0.02, 0.03]);
        assert!((load_seat_for(&key)[1] - 0.02).abs() < 1e-4);
        assert!((load_seat_for("Boeing 747")[0] + 0.30).abs() < 1e-4, "store still intact");

        match prev {
            Some(v) => std::env::set_var("XDG_CONFIG_HOME", v),
            None => std::env::remove_var("XDG_CONFIG_HOME"),
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}

#[cfg(test)]
mod bring_up_ladder_tests {
    use super::*;

    /// The 2026-08-19 regression, as a test.
    ///
    /// Start the sim first, then the app, and there is no lifecycle transition
    /// left to observe — the flight is already live. The bring-up must still
    /// reach `Streaming` on camera status alone. The old code parked on a
    /// `WaitFlight` rung here and sent nothing for the rest of the session
    /// (0-byte trace file).
    #[test]
    fn ladder_climbs_when_the_sim_was_already_flying() {
        // The acquire that goes out at connect is answered with this.
        let stage = next_rung(BringUp::WaitAcquired, protocol::CAMERA_ACQUIRED, false)
            .expect("start streaming");
        assert_eq!(stage, BringUp::Streaming);
        assert_eq!(
            next_rung(stage, protocol::CAMERA_ACQUIRED, false),
            None,
            "no rung above Streaming"
        );
    }

    /// The 2026-08-20 deadlock, as a test.
    ///
    /// Restarting the app in the cockpit had the sim answer `acquired=3`, then
    /// `acquired=0` once we acquired — free, but not ours. That state needs
    /// another acquire, and it is also a state MSFS emits no further status
    /// for, so the retry must not be driven by status arrival.
    #[test]
    fn a_free_but_unowned_camera_is_retried() {
        for availability in [
            protocol::CAMERA_NOT_ACQUIRED,
            protocol::CAMERA_ACQUIRED_BY_OTHER,
            protocol::CAMERA_USER_DISABLED,
        ] {
            assert!(
                should_acquire(availability, false),
                "acquired={availability} with a free camera must retry"
            );
            assert_eq!(
                next_rung(BringUp::WaitAcquired, availability, false),
                None,
                "acquired={availability} is not ownership — must not advance"
            );
        }
    }

    /// Losing the camera **after** streaming has started must also retry.
    ///
    /// The in-game Camera ▸ Addons toggle takes the camera back mid-flight, and
    /// a retry that only ran during bring-up made that a one-way door: every
    /// subsequent frame was silently dropped by the send gate and only an app
    /// restart recovered it (2026-08-20). `should_acquire` therefore takes no
    /// rung at all — this test exists to stop one being added back.
    #[test]
    fn the_camera_is_reclaimed_after_it_is_lost_mid_flight() {
        // The toggle sequence exactly as the sim reported it.
        assert!(!should_acquire(protocol::CAMERA_ACQUIRED, false), "streaming: nothing to do");
        assert!(should_acquire(protocol::CAMERA_USER_DISABLED, false), "toggle off → reclaim");
        assert!(should_acquire(protocol::CAMERA_NOT_ACQUIRED, false), "freed → reclaim");
        assert!(!should_acquire(protocol::CAMERA_ACQUIRED, false), "reclaimed → settle");
    }

    /// The crash fix, as a test. While the game drives the camera — menus,
    /// loading screens, cutscenes — nothing aircraft-facing may go out, because
    /// `EnumerateCameraDefinitions` and the TITLE request are the two calls
    /// that took the sim down mid-load on 2026-08-17.
    #[test]
    fn nothing_advances_while_the_game_controls_the_camera() {
        for availability in [
            protocol::CAMERA_NOT_ACQUIRED,
            protocol::CAMERA_ACQUIRED,
            protocol::CAMERA_ACQUIRED_BY_OTHER,
            protocol::CAMERA_USER_DISABLED,
        ] {
            assert_eq!(
                next_rung(BringUp::WaitAcquired, availability, true),
                None,
                "acquired={availability}: must not enumerate while game-controlled"
            );
        }
    }

    /// The acquire retry must not hammer the sim during a load. It is the one
    /// camera call we send unprompted, and a menu is where restraint matters.
    #[test]
    fn the_acquire_retry_is_silent_while_game_controlled() {
        for availability in [
            protocol::CAMERA_NOT_ACQUIRED,
            protocol::CAMERA_ACQUIRED,
            protocol::CAMERA_ACQUIRED_BY_OTHER,
            protocol::CAMERA_USER_DISABLED,
        ] {
            assert!(
                !should_acquire(availability, true),
                "acquired={availability}: no retry while the game owns the camera"
            );
        }
    }

    /// A sim that is loading and *then* goes live must reach `Streaming`
    /// without any lifecycle event being consulted — the app-first order,
    /// verified in-sim 2026-08-18 and again 2026-08-20.
    #[test]
    fn ladder_climbs_when_the_sim_goes_live_later() {
        let mut stage = BringUp::WaitAcquired;
        // Loading screen: the sim owns the camera, so we sit still.
        for _ in 0..5 {
            assert_eq!(next_rung(stage, protocol::CAMERA_NOT_ACQUIRED, true), None);
            assert!(!should_acquire(protocol::CAMERA_NOT_ACQUIRED, true));
        }
        // The flight goes live; the camera is free but not yet ours.
        assert!(should_acquire(protocol::CAMERA_NOT_ACQUIRED, false));
        // The retry lands.
        stage = next_rung(stage, protocol::CAMERA_ACQUIRED, false).expect("start streaming");
        assert_eq!(stage, BringUp::Streaming);
    }
}
