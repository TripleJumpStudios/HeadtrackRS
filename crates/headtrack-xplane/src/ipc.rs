//! Blocking IPC receiver that runs on a background thread.
//!
//! Connects to the daemon socket, reads pose frames, and stores the latest
//! in a shared `Mutex<Option<Pose>>`.  Automatically reconnects when the
//! daemon restarts — the flight loop keeps writing the last known pose while
//! disconnected.

use headtrack_core::Pose;
use std::{
    io::{self, Read},
    os::unix::net::UnixStream,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::Duration,
};

const RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// Shared connection status — set by IPC thread, read by menu/flight loop.
pub static CONNECTED: AtomicBool = AtomicBool::new(false);

/// Start the IPC receiver background thread.
///
/// The returned `Arc` is shared with the flight loop callback.
pub fn start(socket_path: PathBuf) -> Arc<Mutex<Option<Pose>>> {
    let pose = Arc::new(Mutex::new(None::<Pose>));
    let pose_thread = Arc::clone(&pose);

    thread::Builder::new()
        .name("xplane-ipc-recv".into())
        .spawn(move || {
            loop {
                match UnixStream::connect(&socket_path) {
                    Ok(stream) => {
                        CONNECTED.store(true, Ordering::Relaxed);
                        let _ = ffi_debug("headtrack-rs: connected to daemon\n");
                        match recv_loop(&stream, &pose_thread) {
                            Ok(()) => {}
                            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
                                let _ = ffi_debug("headtrack-rs: daemon disconnected, reconnecting\n");
                            }
                            Err(_) => {}
                        }
                        CONNECTED.store(false, Ordering::Relaxed);
                        // Clear pose on disconnect so stale values aren't written.
                        if let Ok(mut p) = pose_thread.lock() {
                            *p = None;
                        }
                    }
                    Err(_) => {
                        // Daemon not running yet — silently retry.
                    }
                }
                thread::sleep(RECONNECT_DELAY);
            }
        })
        .expect("spawning IPC receiver thread");

    pose
}

/// Read pose frames from `stream` until EOF or error.
fn recv_loop(stream: &UnixStream, pose: &Arc<Mutex<Option<Pose>>>) -> io::Result<()> {
    let mut reader = io::BufReader::new(stream);
    loop {
        // Read 4-byte LE length prefix.
        let mut len_buf = [0u8; 4];
        reader.read_exact(&mut len_buf)?;
        let payload_len = u32::from_le_bytes(len_buf) as usize;

        let mut payload = vec![0u8; payload_len];
        reader.read_exact(&mut payload)?;

        if let Ok(p) = bincode::deserialize::<Pose>(&payload) {
            if let Ok(mut guard) = pose.lock() {
                *guard = Some(p);
            }
        }
    }
}

fn ffi_debug(s: &str) -> Option<()> {
    let c = std::ffi::CString::new(s).ok()?;
    unsafe { crate::ffi::XPLMDebugString(c.as_ptr()) };
    Some(())
}
