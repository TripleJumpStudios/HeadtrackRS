//! TCP framing: length-prefixed SimConnect packet send/recv.

use anyhow::{bail, Result};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

/// A connected SimConnect TCP session.
pub struct SimConnectTcp {
    stream: TcpStream,
}

impl SimConnectTcp {
    /// Connect to `addr` (e.g. `"127.0.0.1:5557"`).
    pub async fn connect(addr: &str) -> Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        // Disable Nagle — SimConnect SDK docs recommend it for low latency.
        stream.set_nodelay(true)?;
        Ok(Self { stream })
    }

    /// Send a pre-built packet (already includes the 4-byte size prefix).
    pub async fn send_packet(&mut self, pkt: &[u8]) -> Result<()> {
        self.stream.write_all(pkt).await?;
        Ok(())
    }

    /// Receive one SimConnect packet.
    ///
    /// Returns `(msg_id, payload)` where `payload` does not include the header.
    /// Returns an error on EOF or malformed data.
    pub async fn recv_packet(&mut self) -> Result<(u32, Vec<u8>)> {
        // Read 12-byte header: u32 size | u32 version | u32 msg_id
        let mut header = [0u8; 12];
        self.stream.read_exact(&mut header).await?;

        let size    = u32::from_le_bytes(header[0..4].try_into().unwrap()) as usize;
        let msg_id  = u32::from_le_bytes(header[8..12].try_into().unwrap());

        if size < 12 {
            bail!("SimConnect packet size {size} < header size 12");
        }
        if size > 65536 {
            bail!("SimConnect packet size {size} exceeds 64KB safety limit");
        }
        let payload_len = size - 12;
        let mut payload = vec![0u8; payload_len];
        if payload_len > 0 {
            self.stream.read_exact(&mut payload).await?;
        }

        Ok((msg_id, payload))
    }
}
