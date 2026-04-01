use anyhow::{Context, Result};
use headtrack_core::Pose;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// Capacity of the broadcast channel that distributes poses to IPC clients.
/// At 250 Hz a capacity of 64 gives ~256 ms of headroom before frames are dropped.
pub const CHANNEL_CAPACITY: usize = 64;

/// Encode a [`Pose`] as a length-prefixed bincode frame.
///
/// Wire format:
/// ```text
/// [ u32 LE payload_len ][ bincode(Pose) ]
/// ```
pub fn encode(pose: &Pose) -> Vec<u8> {
    // Pose is 7 fields (6×f32 + u64) = 32 bytes bincode. Pre-allocate once.
    let mut buf = Vec::with_capacity(4 + 32);
    buf.extend_from_slice(&[0u8; 4]); // placeholder for length prefix
    bincode::serialize_into(&mut buf, pose).expect("Pose serialization is infallible");
    let payload_len = (buf.len() - 4) as u32;
    buf[..4].copy_from_slice(&payload_len.to_le_bytes());
    buf
}

/// Read one length-prefixed frame from `reader` and deserialise it as a [`Pose`].
pub async fn read_frame<R: AsyncRead + Unpin>(reader: &mut R) -> Result<Pose> {
    let mut len_buf = [0u8; 4];
    reader
        .read_exact(&mut len_buf)
        .await
        .context("reading frame length")?;

    let payload_len = u32::from_le_bytes(len_buf) as usize;

    // Pose frames are 32 bytes. Reject anything absurdly large to prevent
    // a malicious client from forcing a multi-GB allocation (DoS).
    const MAX_PAYLOAD: usize = 256;
    anyhow::ensure!(
        payload_len <= MAX_PAYLOAD,
        "frame payload too large ({payload_len} bytes, max {MAX_PAYLOAD})"
    );

    let mut payload = vec![0u8; payload_len];
    reader
        .read_exact(&mut payload)
        .await
        .context("reading frame payload")?;

    bincode::deserialize(&payload).context("deserialising pose frame")
}

/// Zero-alloc pose encoding. Fixed 36-byte wire format.
///
/// Manually serializes to match bincode's output for `Pose`:
/// `[u32 LE length][f32 yaw][f32 pitch][f32 roll][f32 x][f32 y][f32 z][u64 timestamp_us]`
#[inline]
pub fn encode_stack(pose: &Pose) -> [u8; 36] {
    let mut buf = [0u8; 36];
    buf[0..4].copy_from_slice(&32u32.to_le_bytes());
    buf[4..8].copy_from_slice(&pose.yaw.to_le_bytes());
    buf[8..12].copy_from_slice(&pose.pitch.to_le_bytes());
    buf[12..16].copy_from_slice(&pose.roll.to_le_bytes());
    buf[16..20].copy_from_slice(&pose.x.to_le_bytes());
    buf[20..24].copy_from_slice(&pose.y.to_le_bytes());
    buf[24..28].copy_from_slice(&pose.z.to_le_bytes());
    buf[28..36].copy_from_slice(&pose.timestamp_us.to_le_bytes());
    buf
}

/// Write one length-prefixed frame to `writer`.
pub async fn write_frame<W: AsyncWrite + Unpin>(writer: &mut W, pose: &Pose) -> Result<()> {
    writer
        .write_all(&encode_stack(pose))
        .await
        .context("writing pose frame")
}
