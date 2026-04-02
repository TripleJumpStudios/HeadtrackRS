//! MSFS 2024 head tracking output via SimConnect TCP Camera API.
//!
//! # How it works
//!
//! A background tokio task maintains a persistent TCP connection to the MSFS
//! SimConnect server (127.0.0.1:5557, configured via SimConnect.xml).  On each
//! sim Frame event it reads the latest pose from a `watch` channel and sends a
//! `CameraSet` packet with the EYEPOINT referential.
//!
//! The engine calls `SimConnectOutput::write()` from the run loop at ~30 Hz.
//! `write()` is non-blocking — it puts the pose into a `watch::Sender` and
//! returns immediately.  The background task drains it on its own schedule,
//! always using the most recent value (watch semantics).
//!
//! Auto-reconnect: on any TCP error the task waits 2 s and reconnects.
//!
//! # SimConnect.xml (one-time setup)
//!
//! Write to the MSFS user data directory in the Proton prefix:
//! ```
//! ~/.var/app/com.valvesoftware.Steam/.local/share/Steam/steamapps/compatdata/
//!   2537590/pfx/drive_c/users/steamuser/AppData/Roaming/
//!   Microsoft Flight Simulator/SimConnect.xml
//! ```
//!
//! Minimal content:
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
//!
//! # Coordinate mapping
//!
//! SimConnect EYEPOINT uses meters and a quaternion (XYZW).  Signs are TBD and
//! must be verified in-cockpit; current mapping passes through headtrack-rs
//! canonical values with mm→m scaling and ZYX Euler→quaternion conversion.

pub mod connection;
pub mod protocol;

use headtrack_core::Pose;
use tokio::sync::watch;
use tracing::{info, warn};

// ---------------------------------------------------------------------------
// Pose → SimConnect coordinate conversion
// ---------------------------------------------------------------------------

/// Convert a headtrack-rs `Pose` to SimConnect EYEPOINT position (meters) and
/// orientation (quaternion XYZW).
///
/// Signs and axis mapping are empirical — verify in-cockpit and adjust.
fn pose_to_simconnect(pose: &Pose) -> [f32; 6] {
    [
        pose.x / 1000.0,
        pose.y / 1000.0,
        pose.z / 1000.0,
        pose.pitch, // SimConnect uses degrees directly for these
        pose.roll,
        pose.yaw,
    ]
}

// ---------------------------------------------------------------------------
// Background connection task
// ---------------------------------------------------------------------------

/// Unique client event ID used when subscribing to the "Frame" system event.
const FRAME_EVENT_ID: u32 = 1;
/// Request ID used for CameraAcquire.
const CAMERA_REQUEST_ID: u32 = 1;

async fn run_connection(addr: String, pose_rx: watch::Receiver<Pose>) {
    loop {
        info!("SimConnect: connecting to {addr}");

        let mut conn = match connection::SimConnectTcp::connect(&addr).await {
            Ok(c) => c,
            Err(e) => {
                warn!("SimConnect: connect failed: {e:#} — retrying in 2s");
                tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
                continue;
            }
        };

        // Handshake: OPEN → wait for RECV_OPEN.
        let open_pkt = protocol::build_packet(
            protocol::MSG_OPEN,
            &protocol::build_open("headtrack-rs"),
        );
        if let Err(e) = conn.send_packet(&open_pkt).await {
            warn!("SimConnect: send OPEN failed: {e:#}");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        // Wait for RECV_OPEN.
        let (major, minor, build) = loop {
            match conn.recv_packet().await {
                Ok((msg_id, payload)) if msg_id == protocol::RECV_OPEN => {
                    match protocol::parse_recv_open_version(&payload) {
                        Some(ver) => break ver,
                        None => {
                            warn!("SimConnect: RECV_OPEN payload too short");
                            break (0, 0, 0);
                        }
                    }
                }
                Ok(_) => continue, // ignore other messages during handshake
                Err(e) => {
                    warn!("SimConnect: recv error during handshake: {e:#}");
                    break (0, 0, 0); // trigger reconnect
                }
            }
        };

        if major == 0 {
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        if !protocol::version_ok(major, minor, build) {
            warn!(
                "SimConnect: sim version {major}.{minor}.{build} < {}.{}.{} — Camera API unavailable",
                protocol::MIN_VERSION_MAJOR,
                protocol::MIN_VERSION_MINOR,
                protocol::MIN_VERSION_BUILD,
            );
            tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
            continue;
        }

        info!("SimConnect: connected — sim version {major}.{minor}.{build}");

        // Subscribe to Frame events.
        let sub_pkt = protocol::build_packet(
            protocol::MSG_SUBSCRIBE_TO_SYSTEM_EVENT,
            &protocol::build_subscribe_event(FRAME_EVENT_ID, "Frame"),
        );
        if let Err(e) = conn.send_packet(&sub_pkt).await {
            warn!("SimConnect: subscribe Frame failed: {e:#}");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        // Acquire camera (EYEPOINT referential).
        let acquire_pkt = protocol::build_packet(
            protocol::MSG_CAMERA_ACQUIRE,
            &protocol::build_camera_acquire(
                CAMERA_REQUEST_ID,
                protocol::REFERENTIAL_EYEPOINT,
                "headtrack-rs",
            ),
        );
        if let Err(e) = conn.send_packet(&acquire_pkt).await {
            warn!("SimConnect: CameraAcquire failed: {e:#}");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            continue;
        }

        info!("SimConnect: camera acquired — streaming pose");

        // Main event loop: on each Frame event, push the latest pose.
        let disconnect_reason = loop {
            let (msg_id, _payload) = match conn.recv_packet().await {
                Ok(p) => p,
                Err(e) => break format!("recv error: {e:#}"),
            };

            if msg_id != protocol::RECV_EVENT {
                continue;
            }

            // Frame tick — send latest pose.
            let pose = *pose_rx.borrow();
            let data = pose_to_simconnect(&pose);
            let set_pkt = protocol::build_packet(
                protocol::MSG_CAMERA_SET,
                &protocol::build_camera_set(data),
            );
            if let Err(e) = conn.send_packet(&set_pkt).await {
                break format!("send CameraSet failed: {e:#}");
            }
        };

        // Try clean release before reconnecting.
        let release_pkt = protocol::build_packet(
            protocol::MSG_CAMERA_RELEASE,
            &protocol::build_camera_release(protocol::REFERENTIAL_EYEPOINT),
        );
        let _ = conn.send_packet(&release_pkt).await;

        warn!("SimConnect: disconnected ({disconnect_reason}) — retrying in 2s");
        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
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
}

impl SimConnectOutput {
    /// Create the output and start the background connection task.
    ///
    /// `port` is the TCP port configured in SimConnect.xml (default 5557).
    pub fn open(port: u16) -> anyhow::Result<Self> {
        let (tx, rx) = watch::channel(Pose::zero(0));
        let addr = format!("127.0.0.1:{port}");
        tokio::spawn(run_connection(addr, rx));
        Ok(Self { tx })
    }

    /// Push the latest pose to the background task (non-blocking).
    #[inline]
    pub fn write(&self, pose: &Pose) {
        // watch::Sender::send_modify avoids an allocation; ignore send errors
        // (they only occur if all receivers are dropped, which can't happen here
        // because `run_connection` holds one).
        let _ = self.tx.send(*pose);
    }
}
