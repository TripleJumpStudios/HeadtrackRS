use headtrack_output_simconnect::{
    connection::SimConnectTcp,
    protocol::{self, build_open},
};
use tokio::time::{timeout, Duration};

async fn test_packet(conn: &mut SimConnectTcp, id: u32, payload_size: usize) -> anyhow::Result<()> {
    let payload = vec![0u8; payload_size];
    let mut pkt = Vec::with_capacity(16 + payload_size);
    let size = 16 + payload_size as u32;
    pkt.extend_from_slice(&size.to_le_bytes());
    pkt.extend_from_slice(&5u32.to_le_bytes());
    let masked_id = 0xf0000000 | id;
    pkt.extend_from_slice(&masked_id.to_le_bytes());
    pkt.extend_from_slice(&id.to_le_bytes()); // send_id
    pkt.extend_from_slice(&payload);
    conn.send_packet(&pkt).await?;
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut conn = SimConnectTcp::connect("127.0.0.1:5557").await?;
    println!("TCP connected.");

    // OPEN
    let pkt = protocol::build_packet(protocol::MSG_OPEN, &build_open("headtrack-rs-sweeper2"));
    conn.send_packet(&pkt).await?;
    loop {
        match timeout(Duration::from_secs(2), conn.recv_packet()).await {
            Ok(Ok((msg_id, _))) if msg_id == protocol::RECV_OPEN => break,
            _ => continue,
        }
    }

    // Sweep all payload sizes from 0 to 64 for 0x33 and 0x34
    // We do this to find exactly what size CameraSet and CameraRelease expect!
    for id in 0x32u32..=0x34u32 {
        for size in (0usize..=64).step_by(4) {
            test_packet(&mut conn, id, size).await?;
            let mut track_pkt = Vec::with_capacity(16 + size);
            let pkt_size = 16 + size as u32;
            track_pkt.extend_from_slice(&pkt_size.to_le_bytes());
            track_pkt.extend_from_slice(&5u32.to_le_bytes());
            let masked_id = 0xf0000000 | id;
            track_pkt.extend_from_slice(&masked_id.to_le_bytes());
            let send_id = id * 1000 + size as u32;
            track_pkt.extend_from_slice(&send_id.to_le_bytes()); 
            track_pkt.extend_from_slice(&vec![0u8; size]);
            conn.send_packet(&track_pkt).await?;
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    println!("Sent exhaustive size sweep for 0x32, 0x33, and 0x34.");

    // Wait for exceptions
    let mut exceptions = std::collections::HashSet::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() { break; }

        match timeout(remaining, conn.recv_packet()).await {
            Ok(Ok((msg_id, payload))) => {
                if msg_id == 0x01 && payload.len() >= 8 {
                    let exc = u32::from_le_bytes(payload[0..4].try_into().unwrap());
                    let send_id = u32::from_le_bytes(payload[4..8].try_into().unwrap());
                    exceptions.insert((send_id / 1000, send_id % 1000, exc));
                }
            }
            _ => {}
        }
    }

    println!("\n--- EXHAUSTIVE RESULTS ---");
    for id in 0x32u32..=0x34u32 {
        for size in (0u32..=64).step_by(4) {
            let threw_mismatch = exceptions.contains(&(id, size, 2));
            let threw_unrecognized = exceptions.contains(&(id, size, 3));
            if !threw_mismatch && !threw_unrecognized {
                println!("SUCCESS -> ID: 0x{:02x} | EXACT SIZE EXPECTED: {}", id, size);
            }
        }
    }
    
    Ok(())
}
