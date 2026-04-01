use headtrack_output_simconnect::{
    connection::SimConnectTcp,
    protocol::{self, build_open, build_packet, build_subscribe_event, build_camera_acquire, build_camera_set},
};
use tokio::time::{timeout, Duration};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let mut conn = SimConnectTcp::connect("127.0.0.1:5557").await?;
    println!("TCP connected.");

    // Phase 1: OPEN
    let pkt = build_packet(protocol::MSG_OPEN, &build_open("headtrack-rs-probe"));
    conn.send_packet(&pkt).await?;
    println!("Sent OPEN.");

    // Receive OPEN response
    loop {
        match timeout(Duration::from_secs(2), conn.recv_packet()).await {
            Ok(Ok((msg_id, _payload))) => {
                println!("Received msg_id=0x{msg_id:02x} payload_len={}", _payload.len());
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

    // Phase 2: SUBSCRIBE
    let p_sub = build_packet(
        protocol::MSG_SUBSCRIBE_TO_SYSTEM_EVENT,
        &build_subscribe_event(1, "Frame"),
    );
    conn.send_packet(&p_sub).await?;
    println!("Sent SUBSCRIBE Frame.");

    // Phase 3: CAMERA_ACQUIRE
    let p_acq = build_packet(
        protocol::MSG_CAMERA_ACQUIRE,
        &build_camera_acquire(1, protocol::REFERENTIAL_EYEPOINT, "headtrack-rs"),
    );
    conn.send_packet(&p_acq).await?;
    println!("Sent CAMERA_ACQUIRE.");

    // Phase 3.5: CAMERA_SET
    let p_set = build_packet(
        protocol::MSG_CAMERA_SET,
        &build_camera_set([0.1, 0.2, 0.3, 10.0, 5.0, 0.0]),
    );
    conn.send_packet(&p_set).await?;
    println!("Sent MSG_CAMERA_SET.");

    // Phase 4: Wait for frames or exceptions
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            println!("Test completed (5s timeout).");
            break;
        }

        match timeout(remaining, conn.recv_packet()).await {
            Ok(Ok((msg_id, payload))) => {
                println!("Received msg_id=0x{msg_id:02x} payload_len={}", payload.len());
                if payload.len() > 0 && msg_id == 0x01 { // 0x01 is often EXCEPTION
                    println!(" -> WARNING: Exception? First bytes: {:02x?}", &payload[..payload.len().min(16)]);
                }
            }
            Ok(Err(e)) => {
                println!("Recv error: {e}");
                break;
            }
            Err(_) => {
                break;
            }
        }
    }

    Ok(())
}
