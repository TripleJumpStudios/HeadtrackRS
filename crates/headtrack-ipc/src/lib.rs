pub mod client;
pub mod codec;
pub mod server;

use std::path::PathBuf;

/// Default Unix domain socket path: `$XDG_RUNTIME_DIR/headtrack.sock`.
///
/// Plugins and the daemon must agree on this path.  Override via the
/// `HEADTRACK_SOCKET` environment variable for development.
pub fn socket_path() -> PathBuf {
    if let Ok(p) = std::env::var("HEADTRACK_SOCKET") {
        return PathBuf::from(p);
    }
    let runtime_dir = std::env::var("XDG_RUNTIME_DIR")
        .unwrap_or_else(|_| "/tmp".to_string());
    PathBuf::from(runtime_dir).join("headtrack.sock")
}

/// Command socket path: `$XDG_RUNTIME_DIR/headtrack-cmd.sock`.
///
/// The daemon listens here for one-shot text commands (e.g. `"recenter\n"`).
pub fn cmd_socket_path() -> PathBuf {
    if let Ok(p) = std::env::var("HEADTRACK_CMD_SOCKET") {
        return PathBuf::from(p);
    }
    let runtime_dir = std::env::var("XDG_RUNTIME_DIR")
        .unwrap_or_else(|_| "/tmp".to_string());
    PathBuf::from(runtime_dir).join("headtrack-cmd.sock")
}

/// Preview frame path: `$XDG_RUNTIME_DIR/headtrack-preview.jpg`.
///
/// The daemon writes JPEG preview frames here; the GUI reads them for the
/// camera preview display.
pub fn preview_path() -> PathBuf {
    let runtime_dir = std::env::var("XDG_RUNTIME_DIR")
        .unwrap_or_else(|_| "/tmp".to_string());
    PathBuf::from(runtime_dir).join("headtrack-preview.jpg")
}

/// Parameter log socket path: `$XDG_RUNTIME_DIR/headtrack-paramlog.sock`.
///
/// The tester (in dump mode) listens here to capture parameter changes from the
/// GUI with timestamps, correlating them with pose data in the CSV.
pub fn paramlog_socket_path() -> PathBuf {
    let runtime_dir = std::env::var("XDG_RUNTIME_DIR")
        .unwrap_or_else(|_| "/tmp".to_string());
    PathBuf::from(runtime_dir).join("headtrack-paramlog.sock")
}

/// Send a one-shot command to the daemon.
///
/// Connects to [`cmd_socket_path`], writes `cmd`, then closes the connection.
/// Also duplicates `set` commands to the param log socket (fire-and-forget)
/// so the tester can capture parameter changes with timestamps.
/// Blocking — suitable for plugin menu handlers (called on sim main thread).
pub fn send_cmd(cmd: &str) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::net::UnixStream;
    let mut stream = UnixStream::connect(cmd_socket_path())?;
    stream.write_all(cmd.as_bytes())?;

    // Duplicate to param log socket (ignore errors — tester may not be running).
    if cmd.starts_with("set ") || cmd.starts_with("switch-camera ") || cmd == "recenter" {
        if let Ok(mut log_stream) = UnixStream::connect(paramlog_socket_path()) {
            let _ = log_stream.write_all(cmd.as_bytes());
        }
    }

    Ok(())
}
