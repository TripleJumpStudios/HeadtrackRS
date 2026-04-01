use headtrack_output_simconnect::{
    connection::SimConnectTcp,
    protocol::{self, build_open},
};
use tokio::time::{timeout, Duration};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut conn = SimConnectTcp::connect("127.0.0.1:5557").await?;
    println!("TCP connected.");

    // Phase 1: OPEN
    let pkt = protocol::build_packet(protocol::MSG_OPEN, &build_open("headtrack-rs-sweeper"));
    conn.send_packet(&pkt).await?;
    println!("Sent OPEN.");

    // Receive OPEN response
    loop {
        match timeout(Duration::from_secs(2), conn.recv_packet()).await {
            Ok(Ok((msg_id, _payload))) => {
                if msg_id == protocol::RECV_OPEN {
                    println!("-> RECV_OPEN ✓");
                    break;
                }
            }
            _ => {
                println!("Timeout waiting for OPEN response.");
                return Ok(());
            }
        }
    }

    // Phase 2: Sweep undefined IDs to find CameraAcquire (264 bytes payload)
    // CameraAcquire payload: u32 RequestID + u32 Referential + char[256] name => 264 bytes
    let payload = protocol::build_camera_acquire(1, protocol::REFERENTIAL_EYEPOINT, "headtrack-rs");

    for id in 0x30u32..=0x50u32 {
        // Build raw 16-byte header
        // Send_ID is exactly the ID we are testing, so we can match it in exceptions
        let mut pkt = Vec::with_capacity(16 + payload.len());
        let size = 16 + payload.len() as u32;
        
        pkt.extend_from_slice(&size.to_le_bytes());
        pkt.extend_from_slice(&5u32.to_le_bytes()); // version
        
        let masked_id = 0xf0000000 | id;
        pkt.extend_from_slice(&masked_id.to_le_bytes());
        pkt.extend_from_slice(&id.to_le_bytes()); // Use the tested ID as our send_id !

        pkt.extend_from_slice(&payload);
        
        conn.send_packet(&pkt).await?;
    }
    println!("Sent sweep packets 0x30 to 0x40.");

    // Phase 3: Wait for frames or exceptions
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            println!("Sweep completed.");
            break;
        }

        match timeout(remaining, conn.recv_packet()).await {
            Ok(Ok((msg_id, payload))) => {
                if msg_id == 0x01 && payload.len() >= 8 {
                    let exc = u32::from_le_bytes(payload[0..4].try_into().unwrap());
                    let send_id = u32::from_le_bytes(payload[4..8].try_into().unwrap());
                    
                    let exc_name = match exc {
                        0 => "NONE",
                        1 => "ERROR",
                        2 => "SIZE_MISMATCH",
                        3 => "UNRECOGNIZED_ID",
                        7 => "NAME_UNRECOGNIZED",
                        _ => "OTHER",
                    };
                    println!("ID 0x{:02x} -> Exception: {} ({})", send_id, exc_name, exc);
                }
            }
            _ => {}
        }
    }

    Ok(())
}
