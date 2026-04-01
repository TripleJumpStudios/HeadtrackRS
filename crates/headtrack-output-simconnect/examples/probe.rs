//! Phase 0 smoke test: connect to MSFS SimConnect and print the sim version.
//!
//! Run while MSFS is on the main menu (SimConnect.xml must be present):
//!   cargo run -p headtrack-output-simconnect --example probe

use headtrack_output_simconnect::{
    connection::SimConnectTcp,
    protocol::{
        self, build_open, build_packet, parse_recv_open_version, version_ok, RECV_OPEN,
    },
};
use tokio::time::{timeout, Duration};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5557);

    let addr = format!("127.0.0.1:{port}");
    println!("Connecting to SimConnect at {addr} ...");

    let mut conn = SimConnectTcp::connect(&addr).await?;
    println!("TCP connected.");

    // Phase 1: check if MSFS sends anything immediately on connect (before we send OPEN).
    println!("Checking for unsolicited message from sim (1s window)...");
    match timeout(Duration::from_secs(1), conn.recv_packet()).await {
        Ok(Ok((msg_id, payload))) => {
            println!("  Sim sent first: msg_id=0x{msg_id:02x} payload_len={}", payload.len());
            println!("  First 32 bytes: {:02x?}", &payload[..payload.len().min(32)]);
            if msg_id == RECV_OPEN {
                println!("  -> Sim sends RECV_OPEN on connect (server-first protocol)");
                print_open_version(&payload);
                return Ok(());
            }
        }
        Ok(Err(e)) => println!("  Recv error: {e}"),
        Err(_) => println!("  Nothing received — sim waits for client OPEN first."),
    }

    // Phase 2: send OPEN and wait for response.
    let pkt = build_packet(protocol::MSG_OPEN, &build_open("headtrack-rs-probe"));
    println!("\nSending OPEN ({} bytes total)...", pkt.len());
    println!("  Header: {:02x?}", &pkt[..12]);
    conn.send_packet(&pkt).await?;
    println!("Sent OPEN. Waiting for response (5s)...");

    // Receive up to 5 messages with a 5s total timeout.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            println!("Timeout — no response from sim.");
            println!("\nLikely cause: OPEN packet format rejected by MSFS.");
            println!("Try the port 500 fallback: cargo run --example probe -- 500");
            break;
        }

        match timeout(remaining, conn.recv_packet()).await {
            Ok(Ok((msg_id, payload))) => {
                println!("Received msg_id=0x{msg_id:02x} payload_len={}", payload.len());
                println!("  First 32 bytes: {:02x?}", &payload[..payload.len().min(32)]);
                if msg_id == RECV_OPEN {
                    println!("  -> RECV_OPEN ✓");
                    print_open_version(&payload);
                    break;
                } else {
                    println!("  -> Not RECV_OPEN, continuing...");
                }
            }
            Ok(Err(e)) => {
                println!("Recv error: {e}");
                break;
            }
            Err(_) => {
                println!("Timeout waiting for response.");
                break;
            }
        }
    }

    Ok(())
}

fn print_open_version(payload: &[u8]) {
    match parse_recv_open_version(payload) {
        Some((major, minor, build)) => {
            println!("Sim version: {major}.{minor}.{build}");
            if version_ok(major, minor, build) {
                println!("Camera API: AVAILABLE (>= 1.7.9) ✓");
            } else {
                println!(
                    "Camera API: UNAVAILABLE — need >= {}.{}.{}",
                    protocol::MIN_VERSION_MAJOR,
                    protocol::MIN_VERSION_MINOR,
                    protocol::MIN_VERSION_BUILD,
                );
            }
        }
        None => println!("RECV_OPEN payload too short to parse version (len={})", payload.len()),
    }
}
