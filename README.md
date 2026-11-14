# HeadtrackRS

[![CodeQL](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/codeql.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/codeql.yml)
[![Clippy](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/clippy.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/clippy.yml)
[![Coverage](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/tarpaulin.yml/badge.svg)](https://github.com/TripleJumpStudios/HeadtrackRS/actions/workflows/tarpaulin.yml)
[![Quality Gate](https://sonarcloud.io/api/project_badges/measure?project=TripleJumpStudios_HeadtrackRS&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=TripleJumpStudios_HeadtrackRS)

Head tracking for Linux flight simulation — webcam-based, no extra hardware required.

Built natively in Rust for Linux. Not a wrapper around Opentrack.

> **Early Development** — HeadtrackRS is functional but actively developed. You may encounter bugs or rough edges. Bug reports and feedback are welcome via [GitHub Issues](../../issues).
>
> See [`CHANGELOG.md`](CHANGELOG.md) for what changed in the current release, including breaking changes to saved profiles.

---

## What It Does

HeadtrackRS turns any webcam into a 6-axis head tracker — yaw, pitch, roll, and X/Y/Z translation — and feeds that data directly into your flight simulator in real time. Move your head left to look left. Lean forward to peer over the instrument panel. It works the way you'd expect.

Own a **Tobii Eye Tracker 5**? HeadtrackRS can use it as the head source instead of the webcam. It
reads the tracker directly over USB, so no Tobii software is needed. The Tobii gives head rotation
only (yaw, pitch and roll), not position.

Supported simulators:
- **X-Plane 12** — native plugin, all 6 axes
- **Microsoft Flight Simulator 2024** — via SimConnect TCP, full 6DOF
- **Star Citizen** — via NPClient Wine bridge. Head rotation is verified working. Translation is sent but Star Citizen does not appear to apply it, in any game mode — see the roadmap

All three can run simultaneously with no conflicts.

---

## How It Works

1. **Find the face.** A neural detector locates your face and five key points. It does not run on every frame — optical flow carries those points between detections, which is what keeps CPU use low enough to leave the sim alone.
2. **Work out the pose.** Head orientation comes from a learned pose model rather than geometry, which holds up better at steep angles. Position comes from stable facial landmarks — a robust fit of the five key points for left/right and up/down, and for depth the distances from each eye to the opposite mouth corner, which hold up whether you turn or nod — with a rate gate that rejects depth changes faster than a head can physically move, so a blink does not become a lurch.
3. **Smooth it.** An eight-stage pipeline handles centering, cross-axis compensation, response curves and smoothing. The smoothing works on how far the signal has moved since the last output rather than how far you are from centre, which is what lets it hold still while you are holding a pose off to one side — the moment stillness matters most.
4. **Send it.** The processed pose goes to whichever simulators are running, all from the same pipeline.

The GUI is the single entry point. Launch it and everything starts — the tracking engine runs in-process, no separate daemon to manage. A live camera preview and pose readout are on the dashboard so you can see exactly what the tracker is doing. Every stage above is adjustable live, with the parameters you are most likely to want on the Tuning tab and the full set behind an Advanced toggle.

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
- *Optional:* a Tobii Eye Tracker 5 (USB `2104:0313`), as a rotation-only head source
- X-Plane 12, Star Citizen, and/or MSFS 2024 (only what you have)
- No GPU required — runs entirely on CPU

**Tested Hardware**

| Camera | Status | Driver | Notes |
|--------|--------|--------|-------|
| Elgato Facecam Mk.2 | Recommended | Native UVC | 60fps, smooth tracking |
| Logitech C920 | Working | Native UVC | 30fps, reliable |
| Delan Cam 1 | Working | V4L2 | ~11fps hard limit — a camera firmware limitation, not a HeadtrackRS one. Tracking works; it is simply less smooth |

---

## Getting Started

Download the latest AppImage from the [Releases](../../releases) page, mark it executable, and run it:

```bash
chmod +x headtrack-rs-*.AppImage
./headtrack-rs-*.AppImage
```

### Tobii Eye Tracker 5 (optional)

Plug the tracker in, open the **Tobii** tab and click **Use the Tobii**. The first time, Linux may
not let HeadtrackRS open the device. The tab then reads **Permission needed** and offers **Set up
USB access**. That button installs one udev rule (`/etc/udev/rules.d/70-headtrackrs-tobii.rules`)
that gives the logged-in user access to this one device. It asks for your password once. If the
tracker does not start within a few seconds, unplug it and plug it back in. To undo it, delete
that file. The same rule ships inside the AppImage under `udev/` if you prefer to copy it by hand.

The Tobii drives yaw, pitch and roll. X, Y and Z stay at zero, and the Tuning tab greys them out.
It keeps its own tuning, separate from the webcam's. **Use the webcam** switches back. The gaze
point appears in the Tobii tab only and is never sent to a simulator.

### X-Plane 12

Open the Settings tab and click the X-Plane plugin installer. Once installed, enable head tracking in X-Plane via **Plugins → HeadtrackRS → Enable**.

### Star Citizen (Wine / Proton)

Open **Settings → Star Citizen / Wine Bridge** in the GUI. The installer looks for your Star Citizen Wine prefix in common locations (`~/Games/star-citizen`, Lutris defaults, Bottles). If it isn't found automatically, click **Browse...** and select your Wine prefix directory — the folder that contains `drive_c/`. The installer:

1. Copies `NPClient64.dll` into `drive_c/Program Files/Roberts Space Industries/StarCitizen/LIVE/Bin64/`
2. Sets the required registry key (`HKCU\Software\NaturalPoint\...`) via `wine reg add`

After installation, open Star Citizen's settings and go to **Comms, FOIP, VR & Head Tracking → Head Tracking → General**. Two settings are needed:

- **Source** → `TrackIR`
- **Enabled** → `Yes`

There are further head-tracking options and keybindings in the same place; those are yours to set to taste.

**If Star Citizen cannot open your webcam,** turn off **FOIP** (Face Over IP) — it is in that same settings group. FOIP claims the camera exclusively and will block HeadtrackRS from opening it.

**Smoothing** — **Settings → Star Citizen** has a **View response** slider (default 15, matching
X-Plane's feel). Star Citizen pulls pose on its own schedule and applies no smoothing of its own,
so this is the only damping on that path. Set it to 0 for raw output.

You can start HeadtrackRS and Star Citizen in **either order**, and restarting HeadtrackRS while
the game is running resumes tracking without restarting the game.

*(Verified in-game 2026-08-18. Star Citizen does move its settings between patches, so if the names differ on your build, look for Head Tracking in the comms/FOIP settings group.)*

### MSFS 2024

Open **Settings → MSFS 2024** and click **Set up SimConnect.xml**. That is the whole setup.

MSFS only accepts connections on a port its own config file names, and that file lives deep inside the Proton prefix. The button finds it, adds the entry, and keeps a backup of the original — it will not disturb SimConnect entries belonging to other tools. HeadtrackRS then connects automatically whenever the sim is running.

Worth knowing: **a major sim update can rewrite that file and silently drop the entry.** If tracking stops connecting after an update, open the same panel and click the button again — it is safe to re-run and will tell you if nothing needed changing.

All three simulators run the same tuned pipeline — there are no per-sim pipeline overrides. What is MSFS-specific lives entirely on the output side:

- **Smoothing** — **Settings → MSFS** has a smoothing slider (default 15). X-Plane needs no equivalent because its plugin interpolates on the sim's own frame clock.
- **Seat position** — nudge the pilot eyepoint forward/back, left/right, up/down and HeadtrackRS remembers it **per aircraft**, keyed on the aircraft title. Bind the nudges to HOTAS buttons and adjust your seat in flight.
- **Camera release** — head tracking can hand the camera back to MSFS at any time (checkbox in **Settings → MSFS**, or a bound button) so external and drone views work normally. Re-enable and it re-seats itself at the pilot eyepoint.

**If tracking does nothing in MSFS,** check the in-game **Camera → Addons** menu first — an addon-camera "Disable Feature" toggle there gates everything HeadtrackRS sends, and it is easy to trip by accident.

---

## Hotkeys and Controller Bindings

Recentering while the sim has focus is the one thing you always need, so HeadtrackRS binds to your flight hardware directly.

Open the **Keybinds** tab, click **Bind**, and press a button on your stick, throttle or gamepad. That button is now bound — no scripting, no desktop configuration, and it works while the sim is focused because HeadtrackRS reads the device directly rather than waiting for keyboard focus. Reading the device does not take it away from the sim.

Bindable actions include recenter, pause/resume tracking, MSFS camera release, and the MSFS seat nudges.

Keyboard shortcuts are also available through your desktop's global-shortcuts settings, and every action has a command-socket equivalent for scripting. Plain in-app keyboard shortcuts only fire while the HeadtrackRS window has focus, which is why controller buttons are the recommended tier for in-flight use.

---

## Contributing Tracking Data

Tracking quality depends on the camera, the lighting and the room — and the shipped defaults were
derived from a handful of setups. A 15-second recording of *your* hardware is the single most
useful thing you can contribute, and it needs no build tools.

**The easiest useful capture** is 15 seconds of holding still. Open a terminal in the folder
where you saved the AppImage, paste this as one line, and press Enter:

```bash
HEADTRACK_DUMP_RAW=1 HEADTRACK_DUMP_RAW_PATH=~/headtrack-still.csv ./headtrack-rs-0.2.0-x86_64.AppImage
```

HeadtrackRS starts normally. Sit still and look at the camera for about 15 seconds, then close
the window. Your capture is at `~/headtrack-still.csv`.

<details>
<summary>What that line is doing</summary>

The two `NAME=value` pairs before the program name are settings that apply to this run only —
they turn the recorder on and say where to write. Nothing is changed permanently, and running the
AppImage normally afterwards behaves exactly as before. Change `headtrack-still.csv` to name each
capture after what you were doing (`headtrack-yaw.csv`, `headtrack-blink.csv`, and so on) — one
movement per file is far more useful than one long mixed recording.

If your AppImage has a different version number, use its actual filename.

</details>

The result is a plain-text CSV of numbers — head angles in degrees, position in millimetres.
**The recorder writes no image data**: no frames, no thumbnails, no landmarks, no identifiers.
Camera frames are processed in memory and discarded. You can open the file in a text editor and
see exactly what you would be sending.

Attach it to a GitHub issue with your camera model, resolution, lighting and rough distance from
the camera. If you are reporting a tracking problem rather than donating data, send a "hold
still" clip alongside the one that misbehaves — the still clip establishes your noise floor,
which is what makes the other one interpretable.

There is also an in-app recorder (**Settings → Show advanced tuning**, then the **Advanced** tab)
and a structured multi-clip protocol for anyone who wants to go further — see
`docs/CONTRIBUTING_DATA.md`.

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
- **Static analysis scope** — CodeQL analyses the Rust workspace on every push and pull request. Dependency auditing is not currently part of CI.
- **Tobii privacy** — The tracker's infrared images are processed in memory and discarded, the same as webcam frames. Gaze is shown in the Tobii tab and goes nowhere else: not to a file, not to a simulator.
- **Camera privacy** — HeadtrackRS never records or stores camera footage. Frames are processed in memory and immediately discarded, and nothing is ever transmitted off your machine. The released app contains **no code that can write a camera frame**. The one developer tool that saves frames (used to compare face detectors on identical footage) exists only in builds made with a non-default developer option (`--features headtrack-engine/frame-dump`), and is never part of a release or the AppImage. Setting `HEADTRACK_DUMP_FRAMES` on a release build only prints a warning.
- **Diagnostic recording** — For fine-tuning and camera compatibility testing, an optional CSV recorder is available in the GUI under **Settings → Diagnostics** (the recorder controls themselves live in the **Advanced** tab, which you enable via **Settings → Show advanced tuning**). It records head pose data (not video) to a local file. This feature is entirely opt-in and off by default.

### AI Assistance & Auditing

This project uses AI-assisted development to accelerate implementation. To ensure correctness and security:

- Every line of code has been reviewed, refactored, and verified by the maintainer.
- All contributions are scanned by CodeQL and SonarCloud to catch issues regardless of origin.
- The maintainer is accountable for and able to explain all code in this repository.

---

## Known Issues

- **Switching between the webcam and the Tobii mid-flight can freeze the view (MSFS 2024).**
  Pause tracking, switch the source, then resume; that works cleanly every time. A fix that
  does this pause for you is planned.
- **Tobii gaze is not shown yet; the device marks every gaze sample invalid, likely because no display area or calibration is written to it. A future update.**

Found something else? Please open an issue on the [Issues](../../issues) page.

---

## Roadmap

See [`CHANGELOG.md`](CHANGELOG.md) for what landed in 0.2.0 — MSFS 2024 full 6DOF on the SU5
Camera API, one-click SimConnect setup, HOTAS bindings, per-aircraft seat positioning, Star
Citizen output smoothing and reconnect-without-restarting-the-game, a rebuilt tracking front-end
with an MIT-licensed face detector, the Tobii Eye Tracker 5 as a head source, and a GUI overhaul.

### Up Next
- **More tuning** — the defaults are good, not finished. This is the area where contributed
  recordings help most: see [Contributing Tracking Data](#contributing-tracking-data). A 15-second
  clip from a camera or a room we have never tested is worth more than any amount of guessing here
- **Broader game support** — the three current targets are flight sims because that is what gets
  flown here, but the output layer is not flight-specific. Investigating what else the existing
  X-Plane / TrackIR / SimConnect paths can already reach, and what a fourth would take
- **Robust keybindings across every game** — controller bindings work well now; the goal is that
  every action is bindable and behaves identically whichever sim has focus, including the
  keyboard tier
- **Per-camera profiles** — automatically apply the right settings when you switch cameras
- **Profile auto-switching** — load the right profile when a game is detected
- **IR point tracking** — for IR LED clips (Delan Cam, etc.), faster and cleaner than webcam-only tracking
- **Flatpak packaging** — native Flatpak distribution

### Future
- **DCS World** — a fourth simulator target, planned for a future update
- **Tobii translation (X, Y, Z)** — the Tobii currently drives rotation only, and X/Y/Z are held at
  zero. The tracker can measure head position too, so the other three axes are a planned update
- **Star Citizen via Faceware / FOIP** — two related ideas sharing one route. Star Citizen
  accepts head tracking through its FOIP face-tracking path as well as TrackIR, and translation
  is currently sent over the TrackIR bridge but not applied by the game — so FOIP is worth
  investigating as a way in for the axes TrackIR does not carry. The same path would also open up
  **facial expression tracking**, which Star Citizen uses FOIP for natively. Both reuse the
  existing webcam pipeline. Long-range and speculative
- Alternative pose models for better accuracy at extreme head angles (looking over your shoulder)
- Phone and tablet input (SmoothTrack / Opentrack UDP) and LinuxTrack device support
- Automated parameter tuning from recorded sessions

The code-quality pass that headed this list is done — three refactor passes across this cycle,
detailed in the changelog's *Internal* section.

---

## License

MIT — see [`LICENSE`](LICENSE). Bundled fonts, models and libraries keep their own licences; see
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
