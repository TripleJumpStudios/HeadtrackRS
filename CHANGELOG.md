# Changelog

All notable changes to HeadtrackRS are documented here.

Format loosely follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project is pre-1.0: minor versions may carry breaking changes.

---

## [0.2.0] — 2026-11-14

The first release since v0.1.1 (2026-04-08), which was a pure refactor with no user-facing
changes. Six months of work sits between them.

Three things dominate this release: **MSFS 2024 became a first-class target** after Sim Update 5
changed the camera API out from under the old integration; **the tracking front-end was
rebuilt** around different models with a different approach to both rotation and translation;
and **the Tobii Eye Tracker 5 became a second head source**, read directly over USB with no
Tobii software installed.

Everything below passes every offline check and has been flown day to day in MSFS 2024 and
X-Plane 12, on the webcam and on the Tobii; the Tobii unplug/replug hardware test passes. Star
Citizen was not re-flown for this release. Known issues are listed in the README.

### ⚠️ Breaking changes

- **The filter backend was replaced.** One-Euro + Kalman prediction is gone; **Accela** is now the
  only smoothing backend. Profiles saved by v0.1.x still load, but their `[[filter]]` and
  `[prediction]` sections are **ignored** — those stages no longer exist. If you hand-tuned
  either, that tuning is not carried over and you will want to re-tune against the new
  `[[accela]]` section.
- **The default tuning changed.** A fresh install now ships the tuning that was actually flown,
  rather than constructor defaults that differed from it. If you were relying on v0.1.x defaults,
  expect a different feel out of the box.
- **`HEADTRACK_FILTER` and `HEADTRACK_LEGACY` were removed.** There is one filter backend and one
  tracker chain; neither has a runtime switch any more.
- **The face detector is now YuNet (MIT), and SCRFD is no longer shipped.** SCRFD's weights are
  licensed for non-commercial research only. `HEADTRACK_DETECTOR=scrfd` still selects it if you
  supply `scrfd-det-10g.onnx` yourself. The legacy localizer chain is deleted, so a missing model
  is a startup error that names the file and the directory it looked in, instead of a silent
  demotion.
- **Translation is derived differently by default.** X/Y come from a robust five-landmark fit and
  Z from the **`diagonals` depth cue** (each eye to the opposite mouth corner), not from the PnP
  translation vector or the interpupillary distance. `HEADTRACK_DEPTH_CUE=ipd` restores the
  single-pair cue; `HEADTRACK_TRANS_SOURCE=pnp` restores PnP translation.

### Added

- **Tobii Eye Tracker 5 as a head source.** HeadtrackRS reads the tracker (USB `2104:0313`)
  directly and estimates head rotation from its infrared camera with Google's MediaPipe models.
  No Tobii software is needed. It drives **yaw, pitch and roll only**: X, Y and Z are held at
  zero, and the GUI says "Translation not available with Tobii". A new **Tobii** tab shows the
  device state (no device, permission needed, busy, starting, streaming, no face, lost), switches
  the head source, and recenters. Gaze is drawn in that tab only and is never sent to a sim.
  *Switching source mid-flight can freeze tracking; pause, switch, then resume (README Known
  Issues).*
- **One-click USB access for the Tobii.** When Linux refuses to open the tracker, the Tobii tab's
  **Set up USB access** button installs one udev rule (`70-headtrackrs-tobii.rules`, `uaccess`
  for the logged-in user only) through `pkexec`. It asks for your password once, and only when
  you click.
- **Each head source keeps its own tuning.** The webcam uses `active.toml` and the Tobii uses
  `active-tobii.toml`, which starts from a rotation-only **Tobii Default**. Switching source loads
  that source's profile, and Reset restores that source's built-in values.
- **Recenter takes a 2.5 s stillness-gated median** instead of a single frame, so a press made
  mid-excursion no longer bakes the error in. Offline over 21 presses: presses baking in more
  than 50 mm of error fell from 29 % to 10 %.
- **Distance normalisation.** The translation deadzones scale with your measured seating distance
  (lateral ∝ d, depth ∝ d²), referenced to 785 mm. This buys consistency across seats, not
  accuracy.
- **MSFS 2024 full 6DOF via the SU5 Camera API.** The adapter moved off the legacy
  `CameraSetRelative6DOF` path onto `CameraSet` in the `EYEPOINT` referential, which is
  Get/Set-symmetric and therefore able to carry absolute position. Translation now works
  end-to-end in the cockpit. Wire-level details in `docs/SIMCONNECT_CAMERA_API_WIRE.md`.
- **MSFS per-aircraft seat positioning.** Nudge the pilot eyepoint forward/back, left/right,
  up/down; the offset is remembered per aircraft, keyed on the aircraft title. Bindable to HOTAS
  buttons for in-flight adjustment.
- **MSFS camera release.** Hand the camera back to the sim at any time so external and drone
  views behave normally, then re-enable and it re-seats itself at the pilot eyepoint — including
  on a mid-flight re-enable.
- **Controller / HOTAS button bindings, now the primary hotkey tier.** Bind recenter, tracking
  pause, camera release and the seat nudges directly to a stick, throttle or gamepad from the
  Keybinds tab. Reads the evdev device directly, so bindings fire **while the sim has focus** and
  the sim still sees the same button. No scripting, no desktop configuration.
- **A physiological Z rate gate.** Rejects depth steps faster than a head can physically move,
  holding them until they persist long enough to be real — which rides out blinks — plus
  angle-conditioned damping at extreme yaw/pitch. Transparent inside the normal envelope.
- **GUI visual overhaul.** Bundled typography (Inter, JetBrains Mono, Phosphor icons), a shared
  design-system module, fluid resizable layouts, and a consistent page grid across every tab.
  "Dev Tools" became **Advanced** and is hidden behind Settings ▸ *Show advanced tuning*.
- **Four pipeline parameters gained controls** — `slew_limit`, `center.recenter_range`,
  `cross_axis.pitch_to_yaw` and `cross_axis.pitch_to_z` were saved, loaded and pushed to the
  engine, but were editable only by hand-editing TOML.
- **Window size is remembered** between sessions.
- **Adjustable MSFS output smoothing** (Settings ▸ MSFS), defaulting to the same rate X-Plane's
  plugin uses.
- **`HEADTRACK_ORT_OPT_LEVEL`** — drops ONNX Runtime's NCHWc layout pass, which is the common
  factor in rare `GetElementType is not implemented` inference failures. Off by default; it costs
  ~34 % inference time.
- **One-click MSFS SimConnect setup.** Settings ▸ MSFS 2024 now has a **Set up SimConnect.xml**
  button. It locates the Proton prefix across every Steam library, adds the port-5557 listener MSFS
  needs, and keeps a backup — existing entries belonging to other tools are preserved, never
  replaced. The status line reads the file live, so you can see at a glance whether it is
  configured. Safe to re-run, which is the intended recovery when a sim update rewrites the file
  and tracking silently stops connecting. This used to be a hand-edited XML buried in the docs.
- **Star Citizen output smoothing**, with a **View response** slider in Settings ▸ Star Citizen.
  SC was the only target with no smoothing layer at all — X-Plane interpolates on the sim's own
  frame clock and MSFS has its own slider, but the shared-memory path handed the game raw pipeline
  output. Since all three run the identical pipeline, that difference was purely structural, and
  it is why SC read rougher than the other two. Defaults to 15, matching X-Plane's feel; set it to
  0 for the previous raw behaviour.
- **A data-contribution guide** (`docs/CONTRIBUTING_DATA.md`) for anyone who wants to send a
  capture, including what is and is not in the file.

### Changed

- **New installs capture at 1280×720** (the measured Z operating point, `docs/CAMERAS.md`), now
  the preset marked "(recommended)". Existing configs are unchanged: a saved choice is kept, and a
  `camera.toml` without a resolution line keeps 640×480.
- **The tracking front-end was rebuilt.** Detection moved to YuNet, rotation to a learned
  6DRepNet model, and landmark tracking to optical flow between periodic re-detects. The previous
  localizer + pose-estimator ONNX chain is gone entirely.
- **Release builds cannot write camera frames.** The developer frame capture used for the
  detector bake-off (`HEADTRACK_DUMP_FRAMES`) now exists only behind a non-default build feature.
  The AppImage contains no code that writes an image. The pose-only CSV recorder
  (`HEADTRACK_DUMP_RAW`) is unchanged.
- **ONNX Runtime is linked into the binaries.** The AppImage no longer carries a separate
  `libonnxruntime.so`, and it ships exactly the models in an explicit, checksummed list.
- **Accela replaced One-Euro + Kalman** after an in-sim A/B: zero tick events in 8 minutes,
  against 4 in 164 seconds on the old stack. Its deadband is anchored on displacement from the
  last output rather than offset from centre, which is what makes it hold still when you are
  holding a pose away from neutral.
- **Camera exposure, gain and white balance are locked** once they settle, so an auto-exposure
  step cannot shift landmark positions and manufacture a pose jump.
- **Logging is quieter and better levelled.** `RUST_LOG=info` went from 747 lines to 38; ONNX
  Runtime is pinned to warnings unless you ask for it explicitly.
- **Config writes are atomic.** See the corresponding fix below.
- All three simulators run the **identical pipeline** and receive byte-identical pose data.
  Everything sim-specific lives on the output side.

### Fixed

- **MSFS could crash if HeadtrackRS was already running when the sim launched.** The adapter was
  sending camera bring-up calls into a sim with no aircraft loaded. Gating on the `Sim` event was
  not enough — MSFS reports `Sim = 1` during its loading screen — so bring-up now climbs a ladder
  driven by the sim's own camera status and waits for `bGameControlled == false`. Sim-first,
  restarting the app mid-flight, and restarting it at a menu were verified in-sim on 2026-08-20.
  App-first survives the sim's load; *streaming through a full load from app-first is pending
  final live sitting* (`docs/KNOWN_BUGS.md` #0). The ladder consults no lifecycle event and no timer at all — an
  interim version gated its first rung on the `Sim` event and silently never connected to a sim
  that was already flying, because that event is a transition notification, not a state query.
- **A torn config write could destroy every HOTAS binding.** `camera.toml` was written
  non-atomically; a reader on another thread caught it half-written, quarantined it as invalid,
  and the next settings change wrote defaults over the top. Writes now go through a temp file and
  an atomic rename.
- **Models were not found unless the app was launched from the repo root**, which surfaced as
  "no camera at index 0" and sent people looking for a webcam problem.
- **A missing model is no longer reported as a missing camera.** The two failures have different
  fixes, and switching cameras cannot resolve a missing model — so it now says so.
- **Star Citizen never reconnected if HeadtrackRS was restarted mid-session** — and the cause
  was our own cleanup code. `WineShmWriter::drop` called `shm_unlink()`, which removes the shared
  segment's *name* while any existing mapping keeps the *inode* alive. Star Citizen carried on
  reading an orphaned inode (`(deleted)` in `/proc/<pid>/maps`) while the next engine start
  created a different inode under the same name, so the game never saw another byte for the rest
  of its session. The unlink reads as hygiene, which is why it survived several review passes and
  was even listed among the candidate *fixes*. `Drop` now unmaps and closes without unlinking, so
  a restart reattaches to the same segment. A second defect underneath it: `open()` zeroed the
  segment, resetting the sequence counter to 0 — the value the DLL reads as "no data yet" — so
  tracking would have blinked out at the moment of reattach even without the unlink.
  Because the segment now outlives the process, `Drop` also zeroes the sequence counter — the
  DLL's only freshness test — so a game started while HeadtrackRS is *not* running reads "no
  tracker" instead of a frozen stale head offset. **Verified in-game 2026-08-18**, and all three
  start orders work: the DLL retries opening the segment on every poll, so starting the game
  first is fine too. *(A game already running on an affected build cannot recover; restart both
  once.)*
- **Steam library detection never worked for a game outside the default library.** The
  `libraryfolders.vdf` parser used a bounded split, which left the closing quote attached to every
  path it returned — so each one pointed at a directory that could not exist. Anyone with a game on
  a second drive had to Browse to it manually, and the auto-detection appeared to do nothing. This
  affected X-Plane and the Star Citizen prefix as well as the new MSFS setup.
- **A panic was printed at every startup.** It came from the accessibility layer's D-Bus stack:
  zbus was built for a tokio reactor that the GUI thread does not run. The portal crate now uses
  zbus's own executor, so the panic is gone and the accessibility bridge comes up.
- **The selected camera was not remembered** between launches. It is now saved by device name.
- **Re-detect storms on webcam frame drops** are rate-limited.
- **Z rubber-banding and blink-induced depth pops** are much reduced by the Z gate and the
  `diagonals` cue. Not closed: see Known limitations.
- **The "no pose" warning repeated every 5 seconds for as long as you were out of frame.** It
  now warns once per outage; the reminders that follow are debug-level only. Taking the Tobii
  neutral never warns.
- **Unplugging the webcam could peg a CPU core and flood the log**, and the camera kept running
  in the background after you switched to another camera or the Tobii. Capture errors now back
  off, are logged once, and stop the camera after about 5 seconds (at once if the device is
  gone). Switching away from a camera now always releases it.
- **The external camera-preview file could be readable by other users.** It is now private to
  you (mode 0600), and it is not written at all when there is no private runtime directory
  (`XDG_RUNTIME_DIR` unset) rather than falling back to `/tmp`. The in-app preview is unchanged.
- **Running the test suite could touch a running Star Citizen session.** The Wine-bridge tests
  now use their own shared-memory segments instead of the live one.

### Removed

- The legacy localizer + pose-estimator tracker chain, and with it `head-localizer.onnx` and
  `head-pose-0.4-small-f32.onnx`.
- `2d106det.onnx`, orphaned when the dense 106-landmark pipeline was removed.
- The One-Euro and Kalman prediction stages.
- `HEADTRACK_FILTER`, `HEADTRACK_LEGACY`, `HEADTRACK_DENSE`, `HEADTRACK_DENSE_COMPARE`,
  `HEADTRACK_DUMP_CROP`.
- `scrfd-det-10g.onnx` from the shipped models (see Breaking changes).

Together these take **~18 MB off the shipped payload** and remove two ONNX sessions from startup.

### Known limitations

- **Star Citizen does not apply translation.** Rotation works. Every link we control was verified
  live — the engine writes all six axes at ~250 Hz, the game holds the segment mapped, and the
  DLL populates translation in the same struct and checksum as the rotation that demonstrably
  works — so the data reaches Star Citizen and the game does not use it. Confirmed across all
  game modes 2026-08-18. Nothing to fix on our side; a Faceware/FOIP route is on the roadmap as a
  possible alternative.
- **Webcam depth (Z) is much improved, not solved.** At rest it is good; during large head turns
  it can still drift tens of millimetres and return. See `docs/KNOWN_BUGS.md` #0b and #0d.
- **The Tobii is rotation only.** It provides no X, Y or Z.
- **Star Citizen can show an occasional mild hitch**, much smaller than the MSFS tick that was
  fixed this cycle. Not yet reproduced under measurement; the leading hypothesis is a torn read
  across the shared-memory boundary, which would need a seqlock on both sides to close.

### Internal

Three "Snow Leopard" passes ran through this cycle — deliberate draw-down work with no feature
content. Roughly 2,400 lines removed net, tests taken from 37 to 65, and every remaining
`#[allow]`, env flag and doc reference audited against the tree. The third pass targeted
*stale rationale* specifically: comments that correctly explained a decision, and then outlived
the thing they referenced. Notes in `docs/SNOW_LEOPARD_3_PLAN.md`.

---

## [0.1.1] — 2026-04-08

Zero-feature refactor pass ("Snow Leopard"). 24 of 25 outstanding code-quality issues resolved,
hot-path allocations eliminated, input and output adapters made optional Cargo features, dead
code removed. No user-facing changes.

## [0.1.0] — 2026-04-01

First release. X-Plane 12 native plugin, Star Citizen NPClient Wine bridge, MSFS 2024 over
SimConnect TCP, NeuralNet webcam tracking, the tuning GUI, FOV presets, output plugin installers
with Steam library detection, and minimize-to-tray.
