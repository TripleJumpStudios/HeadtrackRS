use anyhow::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() -> Result<()> {
    // The daemon is a terminal tool with no UI to report state, so it defaults
    // to the startup narrative. The GUI defaults to `warn` instead — nobody
    // reads its stdout, and it shows camera, profile and per-sim link status on
    // screen. `log_filter` keeps ORT's arena logging out of both.
    tracing_subscriber::fmt()
        .with_env_filter(headtrack_engine::log_filter("info"))
        .init();

    // Start the engine — opens camera, binds IPC socket, spawns all tasks.
    // This creates its own tokio runtime internally.
    let _engine = headtrack_engine::Engine::start()?;

    // Wait for SIGTERM or Ctrl-C, then drop the engine (shuts down cleanly).
    eprintln!("headtrackd running. Press Ctrl-C to stop.");
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = Arc::clone(&running);

    ctrlc::set_handler(move || {
        running_clone.store(false, Ordering::Relaxed);
    })?;

    while running.load(Ordering::Relaxed) {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    Ok(())
}
