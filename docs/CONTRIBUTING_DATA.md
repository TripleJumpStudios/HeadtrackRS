# Contributing Tracking Data

HeadtrackRS is tuned against real recordings. The defaults that ship were derived from a handful
of cameras in one room, and the fastest way to make tracking better on *your* hardware is to send
a short recording of it.

You do not need to build anything. Everything below works with the AppImage.

---

## 1. What is in the file (and what is not)

A capture is a **plain-text CSV of numbers**. Open it in any text editor — it looks like this:

```
# headtrack-rs raw pose dump
time_s,yaw,pitch,roll,x,y,z,fps
0.033,0.412,-1.204,0.087,1.42,-0.63,3.10,30.1
0.066,0.418,-1.199,0.091,1.45,-0.61,3.14,30.0
```

- `yaw,pitch,roll` — head angles in degrees
- `x,y,z` — head position in millimetres
- `fps` — tracker rate at that moment
- Diagnostic captures add one column per tuning parameter, so a settings change mid-session is
  visible in the data

**There is no image data of any kind.** No frames, no thumbnails, no face landmarks, no
identifiers. Nothing in the file can reconstruct what you look like or what was behind you. The
camera feed is processed in memory and discarded; only these numbers are written.

The one thing worth knowing: a capture *is* a record of how your head moved for the length of the
recording. If you talk to someone during it, that is not captured — but the fact that your head
was moving is.

---

## 2. Pick a route

| You have | Use | Gives you |
|---|---|---|
| The AppImage, want the simplest thing | **Route A — in-app recorder** | Post-pipeline pose, exactly what the sim receives |
| The AppImage, want the most useful data | **Route B — raw dump** | Pre-pipeline pose, straight from the tracker |
| A source checkout | **Route C — `capture.sh`** | The structured 11-clip protocol |

**Route B is the most valuable.** The raw dump is the tracker's output *before* any filtering, so
it can be replayed offline against different tuning without you re-recording anything. A single
raw capture can test a dozen candidate settings.

---

## 3. Route A — the in-app recorder

1. **Settings** → turn on **Show advanced tuning**
2. Go to the **Advanced** tab → **Diagnostics**
3. Set a size limit if you want one, then **Start recording**
4. Do the movements (see §6), then **Stop recording**

The file lands in the directory shown in **Settings → Diagnostics** (by default under your XDG
data directory). This records the *processed* pose — useful for "here is what my sim actually
received when it felt wrong".

---

## 4. Route B — raw dump (AppImage, nothing to install)

Open a terminal in the folder where you saved the AppImage, paste this as **one line**, and
press Enter:

```bash
HEADTRACK_DUMP_RAW=1 HEADTRACK_DUMP_RAW_PATH=~/headtrack-still.csv ./headtrack-rs-0.2.0-x86_64.AppImage
```

HeadtrackRS starts normally. Do your movements, then close the window. Your capture is at
`~/headtrack-still.csv`.

The two `NAME=value` pairs before the program name apply to **this run only** — they switch the
recorder on and say where to write. Nothing is changed permanently; launching the AppImage
normally afterwards behaves exactly as before. If your AppImage has a different version number,
use its real filename.

The file is flushed line by line, so even a hard close leaves a valid CSV.

Record **one condition per file** and name the file after it. That is the single most useful
thing you can do — it means nobody has to label the data by hand, and the analysis is computed
per file automatically.

Useful extras, all optional:

```bash
# quieter log, but keep the tracker's per-second health line so you can confirm
# it tracked cleanly for the whole clip (look for "miss 0")
RUST_LOG=warn,headtrack_input_neuralnet=info

# pick a specific camera if you have more than one
HEADTRACK_CAMERA=2
```

---

## 5. Route C — `capture.sh` (source checkout only)

If you have the repo cloned:

```bash
scripts/capture.sh still     # then yaw, pitch, roll, x, y, z, blink, smile, talk, brows
```

It prints what to do for each condition, records to `~/headtrack-dataset/<condition>.csv`, and
tells you the frame count on exit. When you start a clip, the script prints what that clip
isolates and how to move.

> `capture.sh` runs `cargo run` from the checkout, so it does **not** work with the AppImage.
> Use Route B instead and name the files the same way.

---

## 6. What to record

If you only do one, do **`still`** — it is the highest-value clip, because at-rest noise is what
most tuning fights.

| Name it | Do this | ~ |
|---|---|---|
| `still` | Hold dead still, look straight at the camera, relaxed face | 15 s |
| `yaw` | Turn left ↔ right only. Eyes level, no tilt, no lean | 15 s |
| `pitch` | Nod up ↔ down only. No turn, no tilt | 15 s |
| `roll` | Tilt ear-to-shoulder only | 15 s |
| `x` | Face forward, slide head left ↔ right (do **not** turn) | 15 s |
| `y` | Face forward, move head up ↔ down (do **not** nod) | 15 s |
| `z` | Face forward, lean toward and away from the camera | 15 s |
| `blink` | Head still, blink normally then a few hard blinks | 15 s |
| `smile` | Head still, smile and relax repeatedly | 15 s |
| `talk` | Head still, read something aloud | 15 s |
| `brows` | Head still, raise and furrow eyebrows | 15 s |

The last four exist because facial expression leaks into position on a webcam tracker. Isolating
them is how that gets fixed.

Wait for the camera preview to appear before you start, and move **slowly** — the point is clean
signal, not a workout.

---

## 7. Marking a moment during a session

If something feels wrong mid-flight and you want it findable in the data, send a marker. It is a
no-op command that changes nothing but leaves a timestamped row in the CSV:

```bash
echo 'set marker tick 1' | socat - UNIX-CONNECT:"$XDG_RUNTIME_DIR/headtrack-cmd.sock"
# or, without socat:
echo 'set marker tick 1' | nc -U "$XDG_RUNTIME_DIR/headtrack-cmd.sock"
```

It lands as a comment row (`# ts_us=… set marker tick 1`) between the two pose rows either side
of it, so "it glitched *here*" survives into the analysis. Bind it to a HOTAS button via the
Keybinds tab if you want it reachable in flight.

---

## 8. Sending it

Open a GitHub issue and attach the CSV (or a zip of several). The numbers alone are useful, but
these five lines make them much more so:

```
Camera:      Logitech C920 (or: unknown USB cam, "1080p webcam" from Amazon)
Resolution:  640x480
Lighting:    overhead LED, curtains open / dim room, one lamp behind me
Distance:    ~60 cm from the camera
Sim:         X-Plane 12 / MSFS 2024 / Star Citizen / bench, no sim
```

Plus one sentence on what you noticed, if anything — *"Z jitters when I look down"* is worth more
than a paragraph of speculation about why.

**If you are reporting a problem rather than donating data,** the most useful pairing is a `still`
clip plus the specific condition that misbehaves. The `still` clip establishes your noise floor,
which is what makes the other clip interpretable.

---

## 9. What happens to it

Captures are replayed through the real pipeline offline — the same code that runs live, not a
re-implementation — so candidate tuning can be A/B'd without a camera or a sim. The replay
tool is `crates/headtrack-core/examples/replay.rs`.

That is why raw (Route B) captures are worth more than processed ones: a raw clip can be tested
against any tuning, forever. A processed clip only shows what one particular set of settings did.
