# HeadtrackRS

[![CodeQL](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/codeql.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/codeql.yml)
[![Clippy](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/clippy.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/clippy.yml)
[![Coverage](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/tarpaulin.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/tarpaulin.yml)
[![Quality Gate](https://sonarcloud.io/api/project_badges/measure?project=TripleJumpStudios_HeadtrackRS&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=TripleJumpStudios_HeadtrackRS)

Head tracking for Linux flight simulation — webcam-based, no extra hardware required.

Built natively in Rust for Linux. Not a wrapper around Opentrack.

> **Early Development** — HeadtrackRS is functional but actively developed. You may encounter bugs or rough edges. Bug reports and feedback are welcome via [GitHub Issues](../../issues).

---

## What It Does

HeadtrackRS turns any webcam into a 6-axis head tracker — yaw, pitch, roll, and X/Y/Z translation — and feeds that data directly into your flight simulator in real time. Move your head left to look left. Lean forward to peer over the instrument panel. It works the way you'd expect.

Supported simulators:
- **X-Plane 12** — native plugin, all 6 axes
- **Star Citizen** — via NPClient Wine bridge, full 6DOF
- **Microsoft Flight Simulator 2024** — via SimConnect TCP, full 6DOF

All three can run simultaneously with no conflicts.

---

## How It Works

1. Your webcam feed is analyzed frame-by-frame to detect your head and estimate its position and orientation.
2. That raw data passes through a signal processing pipeline — filters, response curves, and compensation — to remove jitter and make motion feel smooth and natural.
3. The processed pose is broadcast to whichever simulators you have running.

The GUI is the single entry point. Launch it and everything starts — the tracking engine runs in-process, no separate daemon to manage. A live camera preview and pose readout are on the dashboard so you can see exactly what the tracker is doing.

---

## Why Not Just Use Opentrack?

Opentrack is a great project, but it runs on Windows. Running it through Wine on Linux introduces latency, compatibility headaches, and fragility — especially for modern games like Star Citizen and MSFS 2024 that use Proton.

HeadtrackRS is a clean Linux-native implementation:

- **Single AppImage** — one file, no system dependencies, no install step
- **Native outputs** — talks directly to X-Plane via its plugin API, to Star Citizen via its NPClient interface, and to MSFS via SimConnect TCP. No Wine layer in the tracking path.
- **Pure Rust** — fast, low-overhead, no runtime surprises
- **Tunable from the GUI** — every filter and curve parameter is adjustable live with instant feedback

---

## Requirements

- Linux (x86_64)
- A webcam — see tested hardware below. 60fps cameras are recommended for the smoothest tracking.
- X-Plane 12, Star Citizen, and/or MSFS 2024 (only what you have)
- No GPU required — runs entirely on CPU

**Tested Hardware**

| Camera | Status | Driver | Notes |
|--------|--------|--------|-------|
| Elgato Facecam Mk.2 | Recommended | Native UVC | 60fps, smooth tracking |
| Logitech C920 | Working | Native UVC | 30fps, reliable |
| Delan Cam 1 | Working | V4L2 | Low framerate under investigation; may require `v4l2-ctl` tweaking |

---

## Getting Started

Download the latest AppImage from the [Releases](../../releases) page, mark it executable, and run it:

```bash
chmod +x headtrack-rs-*.AppImage
./headtrack-rs-*.AppImage
```

### X-Plane 12

Open the Settings tab and click the X-Plane plugin installer. Once installed, enable head tracking in X-Plane via **Plugins → HeadtrackRS → Enable**.

### Star Citizen (Wine / Proton)

Open **Settings → Star Citizen / Wine Bridge** in the GUI. The installer looks for your Star Citizen Wine prefix in common locations (`~/Games/star-citizen`, Lutris defaults, Bottles). If it isn't found automatically, click **Browse...** and select your Wine prefix directory — the folder that contains `drive_c/`. The installer:

1. Copies `NPClient64.dll` into `drive_c/Program Files/Roberts Space Industries/StarCitizen/LIVE/Bin64/`
2. Sets the required registry key (`HKCU\Software\NaturalPoint\...`) via `wine reg add`

After installation, enable head tracking in Star Citizen under **Comms, Social & Gameplay → Head Tracking → TrackIR**.

### MSFS 2024

No installer needed. HeadtrackRS connects to MSFS 2024 automatically via the SimConnect TCP API when the sim is running. One-time setup: create a `SimConnect.xml` file in your MSFS user data directory — see `docs/MSFS_TESTING.md` for the exact path and content.

---

## Building From Source

Requires Rust (stable) and a working V4L2 stack (standard on any modern Linux desktop).

```bash
cargo build --release
cargo run -p headtrack-gui --release
```

> **Fedora users:** You may need `sudo dnf install rust-src clang-devel v4l-utils-devel` before building.

---

## Security & Quality

HeadtrackRS accesses your webcam and communicates with running games — so security is taken seriously.

- **Memory safety** — Built in Rust, eliminating entire classes of vulnerabilities like buffer overflows and use-after-free by design.
- **Static analysis (CodeQL)** — Every pull request is audited by GitHub CodeQL for security issues including path injection (CWE-022) and command injection (CWE-078).
- **Code quality (SonarCloud)** — Continuously monitored for maintainability, code smells, and technical debt.
- **Lint enforcement (Clippy)** — `cargo clippy -- -D warnings` is enforced in CI; warnings are treated as errors.
- **Test coverage (Tarpaulin)** — Coverage is tracked and reported on every build.
- **AppImage scope** — The CodeQL scan covers the full bundled AppImage, not just top-level source. Dependencies are included in the audit.
- **Camera privacy** — HeadtrackRS does not record, store, or transmit camera footage. Frames are processed in memory and immediately discarded. No data ever leaves your machine.
- **Diagnostic recording** — For fine-tuning and camera compatibility testing, an optional CSV recorder is available under **Dev Tools** in the GUI. It records head pose data (not video) to a local file. This feature is entirely opt-in and off by default.

### AI Assistance & Auditing

This project uses AI-assisted development to accelerate implementation. To ensure correctness and security:

- Every line of code has been reviewed, refactored, and verified by the maintainer.
- All contributions are scanned by CodeQL and SonarCloud to catch issues regardless of origin.
- The maintainer is accountable for and able to explain all code in this repository.

---

## Roadmap

### Up Next
- **Per-camera profiles** — automatically apply the right settings when you switch cameras
- **Profile auto-switching** — load the right profile when a game is detected
- **IR point tracking** — for IR LED clips (Delan Cam, etc.), faster and cleaner than webcam-only tracking
- **Flatpak packaging** — native Flatpak distribution
- **Thorough integration testing** — extended in-sim sessions across all three simulators to validate and improve default settings

### Future
- Alternative pose models for better accuracy at extreme head angles (looking over your shoulder)
- Star Citizen face expression tracking via the same webcam pipeline
- Automated parameter tuning from recorded sessions

---

## License

MIT
