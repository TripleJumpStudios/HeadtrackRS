use anyhow::{Context, Result};
use headtrack_core::Pose;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// Capacity of the broadcast channel that distributes poses to IPC clients.
/// At 250 Hz a capacity of 64 gives ~256 ms of headroom before frames are dropped.
pub const CHANNEL_CAPACITY: usize = 64;

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

#[cfg(test)]
mod tests {
    use super::*;

    /// Distinctive values in every field, so a transposition or a wrong offset
    /// shows up as a wrong field rather than a coincidentally equal one.
    fn sample() -> Pose {
        Pose {
            yaw: 1.5,
            pitch: -2.25,
            roll: 3.125,
            x: -4.0625,
            y: 5.5,
            z: -6.75,
            timestamp_us: 0x0123_4567_89AB_CDEF,
        }
    }

    /// The load-bearing claim in `encode_stack`'s doc comment.
    ///
    /// `encode_stack` is hand-written byte-shuffling; `read_frame` — the only
    /// decoder, used by the X-Plane plugin and `headtrack-tester` — deserialises
    /// with bincode. Nothing enforced that the two agree, so a field reordered
    /// in `Pose` would keep compiling, keep passing every other test, and feed
    /// every external client silently wrong numbers.
    #[test]
    fn encode_stack_matches_bincode() {
        let pose = sample();
        let bincode_payload = bincode::serialize(&pose).expect("Pose serialises");
        let stack = encode_stack(&pose);

        assert_eq!(
            bincode_payload.len(),
            32,
            "Pose is expected to bincode to exactly 32 bytes; encode_stack's \
             fixed 36-byte buffer and hard-coded length prefix both assume it"
        );
        assert_eq!(
            &stack[4..],
            &bincode_payload[..],
            "encode_stack no longer matches bincode — external clients decode \
             with bincode, so they would receive wrong values"
        );
        assert_eq!(
            u32::from_le_bytes(stack[..4].try_into().unwrap()),
            bincode_payload.len() as u32,
            "length prefix must describe the payload that follows it"
        );
    }

    /// The documented wire layout, asserted at explicit offsets.
    #[test]
    fn encode_stack_wire_layout_is_stable() {
        let pose = sample();
        let b = encode_stack(&pose);

        assert_eq!(b.len(), 36);
        assert_eq!(u32::from_le_bytes(b[0..4].try_into().unwrap()), 32);
        assert_eq!(f32::from_le_bytes(b[4..8].try_into().unwrap()), pose.yaw);
        assert_eq!(f32::from_le_bytes(b[8..12].try_into().unwrap()), pose.pitch);
        assert_eq!(f32::from_le_bytes(b[12..16].try_into().unwrap()), pose.roll);
        assert_eq!(f32::from_le_bytes(b[16..20].try_into().unwrap()), pose.x);
        assert_eq!(f32::from_le_bytes(b[20..24].try_into().unwrap()), pose.y);
        assert_eq!(f32::from_le_bytes(b[24..28].try_into().unwrap()), pose.z);
        assert_eq!(
            u64::from_le_bytes(b[28..36].try_into().unwrap()),
            pose.timestamp_us
        );
    }

    /// End-to-end through the real decoder: what the writer emits is what a
    /// plugin reads back.
    #[tokio::test]
    async fn write_frame_round_trips_through_read_frame() {
        let pose = sample();
        let mut wire = Vec::new();
        write_frame(&mut wire, &pose).await.expect("write");

        let mut cursor = std::io::Cursor::new(wire);
        let decoded = read_frame(&mut cursor).await.expect("read");

        assert_eq!(decoded.yaw, pose.yaw);
        assert_eq!(decoded.pitch, pose.pitch);
        assert_eq!(decoded.roll, pose.roll);
        assert_eq!(decoded.x, pose.x);
        assert_eq!(decoded.y, pose.y);
        assert_eq!(decoded.z, pose.z);
        assert_eq!(decoded.timestamp_us, pose.timestamp_us);
    }

    /// The `MAX_PAYLOAD` guard: a hostile length prefix must be rejected before
    /// the allocation, not after.
    #[tokio::test]
    async fn read_frame_rejects_an_oversized_length_prefix() {
        let mut wire = Vec::new();
        wire.extend_from_slice(&u32::MAX.to_le_bytes());

        let mut cursor = std::io::Cursor::new(wire);
        let err = read_frame(&mut cursor).await.expect_err("must reject");
        assert!(
            err.to_string().contains("too large"),
            "expected the size guard to fire, got: {err}"
        );
    }
}
