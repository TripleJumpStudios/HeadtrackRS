//! Hardware test: needs a Tobii Eye Tracker 5 plugged in, the udev rule (or an ACL)
//! granting this user access, and NOTHING else holding the device (no HeadtrackRS,
//! no TobiiLab). Run only when the maintainer authorizes it:
//!
//! ```text
//! cargo test -p headtrack-input-tobii --test hardware -- --ignored --nocapture
//! ```
//!
//! It never needs a face in front of the tracker: "frames arriving" is either
//! `Streaming` or `NoFace`. Nothing is written to disk.
use headtrack_core::{InputSource, TobiiState, TobiiStatus};
use headtrack_input_tobii::TobiiInput;
use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

fn models() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/models/tobii")
}

fn count(dir: &str) -> usize {
    std::fs::read_dir(dir).map(|d| d.count()).unwrap_or(0)
}

/// Wait until frames arrive; returns the last status seen.
fn wait_for_frames(status: &Mutex<TobiiStatus>, src: &mut TobiiInput, limit: Duration) -> TobiiStatus {
    let start = Instant::now();
    loop {
        let _ = src.poll();
        let s = status.lock().unwrap().clone();
        if matches!(s.state, TobiiState::Streaming | TobiiState::NoFace) || start.elapsed() > limit {
            return s;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
#[ignore = "needs the Tobii hardware, closed by everything else; run only when authorized"]
fn stop_and_reopen_releases_the_device() {
    // Baselines are taken after round 1, not before it. nusb 0.2.7 starts one process-wide
    // "nusb-events" thread plus one epoll fd on the first device open and never stops them
    // (nusb-0.2.7/src/platform/linux_usbfs/events.rs, `register_fd`, lines 62-72). That is a
    // one-time cost, not a leak; rounds 2-3 must add nothing.
    let mut fds_before = 0;
    let mut threads_before = 0;
    for round in 1..=3 {
        let status = Arc::new(Mutex::new(TobiiStatus::default()));
        let mut src = TobiiInput::open(&models(), None, status.clone()).expect("models load");
        let s = wait_for_frames(&status, &mut src, Duration::from_secs(15));
        eprintln!(
            "round {round}: state={:?} fps={:.1} serial={:?} msg={:?}",
            s.state, s.fps, s.serial, s.message
        );
        assert!(
            matches!(s.state, TobiiState::Streaming | TobiiState::NoFace),
            "round {round}: device never streamed: {s:?}"
        );
        let t = Instant::now();
        drop(src);
        let took = t.elapsed();
        eprintln!("round {round}: drop took {took:?}");
        assert!(took < Duration::from_secs(2), "teardown took {took:?}");
        if round == 1 {
            fds_before = count("/proc/self/fd");
            threads_before = count("/proc/self/task");
        }
    }
    let fds_after = count("/proc/self/fd");
    let threads_after = count("/proc/self/task");
    eprintln!("fds {fds_before} -> {fds_after}, threads {threads_before} -> {threads_after}");
    assert!(fds_after <= fds_before, "fd leak: {fds_before} -> {fds_after}");
    assert!(threads_after <= threads_before, "thread leak: {threads_before} -> {threads_after}");
}
