/// Live pose monitor — connect to the daemon and print pose values in real time.
///
/// Run in a second terminal while the daemon is running:
///   cargo run --bin pose-monitor
use anyhow::Result;
use headtrack_ipc::{client::IpcClient, socket_path};
use std::time::Instant;

#[tokio::main]
async fn main() -> Result<()> {
    let path = socket_path();

    eprintln!("Connecting to daemon at {} ...", path.display());
    let mut client = IpcClient::connect(&path).await?;
    eprintln!("Connected. Move your head.\n");

    let mut last = Instant::now();
    let mut frames: u32 = 0;
    let mut fps: f32 = 0.0;

    loop {
        let pose = client.recv().await?;
        frames += 1;

        let now = Instant::now();
        let elapsed = now.duration_since(last).as_secs_f32();
        if elapsed >= 0.5 {
            fps = frames as f32 / elapsed;
            frames = 0;
            last = now;
        }

        // Overwrite the same line so output doesn't scroll.
        print!(
            "\r  YAW {:+7.2}°  PITCH {:+7.2}°  ROLL {:+7.2}°  |  X {:+6.1}mm  Y {:+6.1}mm  Z {:+6.1}mm  |  {:.1} fps   ",
            pose.yaw, pose.pitch, pose.roll,
            pose.x,   pose.y,    pose.z,
            fps,
        );

        use std::io::Write;
        std::io::stdout().flush().ok();
    }
}
