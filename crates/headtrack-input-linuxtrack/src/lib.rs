//! Bridge to the LinuxTrack X-IR daemon.
//!
//! Connects to the LinuxTrack IPC socket (fwfa123 fork) and translates its
//! pose format into headtrack-rs [`RawPose`](headtrack_core::RawPose).
//!
//! Gives TrackIR 2-5, PS3 Eye, and Wiimote support without duplicating
//! any hardware driver work.
//!
//! Phase 2 deliverable.
