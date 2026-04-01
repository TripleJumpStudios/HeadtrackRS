use anyhow::{Context, Result};
use headtrack_core::Pose;
use std::path::Path;
use tokio::{net::UnixListener, sync::broadcast};
use tracing::{debug, info, warn};

use crate::codec::{self, CHANNEL_CAPACITY};

/// IPC server that accepts plugin connections and broadcasts pose frames.
///
/// Obtain a [`broadcast::Sender<Pose>`] from [`IpcServer::bind`] and use it
/// to push processed poses from the pipeline loop.  The server distributes
/// each pose to all connected clients; lagged clients drop frames rather than
/// blocking the sender.
pub struct IpcServer {
    listener: UnixListener,
    tx: broadcast::Sender<Pose>,
}

impl IpcServer {
    /// Bind the Unix domain socket at `socket_path`.
    ///
    /// Returns `(server, sender)`.  Pass `sender` to the pipeline loop;
    /// call [`IpcServer::serve`] to start accepting connections.
    ///
    /// Any stale socket file at `socket_path` is removed before binding.
    pub async fn bind(socket_path: &Path) -> Result<(Self, broadcast::Sender<Pose>)> {
        // Clean up a stale socket from a previous run.
        let _ = tokio::fs::remove_file(socket_path).await;

        let listener =
            UnixListener::bind(socket_path).context("binding IPC socket")?;

        let (tx, _) = broadcast::channel(CHANNEL_CAPACITY);

        info!("IPC server bound to {}", socket_path.display());

        Ok((Self { listener, tx: tx.clone() }, tx))
    }

    /// Accept connections forever.  Each connection is handled in its own task.
    ///
    /// Blocks until the listener errors.  Spawn this with `tokio::spawn`.
    pub async fn serve(self) -> Result<()> {
        loop {
            let (stream, _addr) = self
                .listener
                .accept()
                .await
                .context("accepting IPC connection")?;

            let mut rx = self.tx.subscribe();

            debug!("IPC client connected");

            tokio::spawn(async move {
                let (_, mut writer) = tokio::io::split(stream);

                loop {
                    match rx.recv().await {
                        Ok(pose) => {
                            if codec::write_frame(&mut writer, &pose).await.is_err() {
                                break; // client disconnected
                            }
                        }
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!("IPC client lagged, dropped {n} frames");
                        }
                        Err(broadcast::error::RecvError::Closed) => break,
                    }
                }

                debug!("IPC client disconnected");
            });
        }
    }
}
