use anyhow::Result;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("headtrackd=warn".parse()?),
        )
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
