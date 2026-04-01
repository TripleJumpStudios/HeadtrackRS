use anyhow::{Context, Result};
use headtrack_core::Pose;
use std::path::Path;
use tokio::net::UnixStream;
use tracing::info;

use crate::codec;

/// IPC client for plugins (X-Plane, Wine bridge, etc.).
///
/// Connects to the daemon's Unix domain socket and reads pose frames.
/// The server sends frames at the daemon's pipeline rate; the client
/// is responsible for reading fast enough to avoid falling behind.
///
/// # Reconnection
/// The X-Plane plugin (and other consumers) should call [`IpcClient::connect`]
/// in a retry loop so the daemon can be restarted independently of the game.
pub struct IpcClient {
    stream: tokio::io::BufReader<UnixStream>,
}

impl IpcClient {
    /// Connect to the daemon socket at `socket_path`.
    pub async fn connect(socket_path: &Path) -> Result<Self> {
        let stream = UnixStream::connect(socket_path)
            .await
            .context("connecting to IPC socket")?;

        info!("Connected to daemon at {}", socket_path.display());

        Ok(Self {
            stream: tokio::io::BufReader::new(stream),
        })
    }

    /// Block until the next pose frame arrives.
    pub async fn recv(&mut self) -> Result<Pose> {
        codec::read_frame(&mut self.stream).await
    }
}
