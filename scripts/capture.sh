#!/usr/bin/env bash
# capture.sh — record one single-condition raw-pose clip for the offline workbench.
#
# Launches headtrack-gui with the HEADTRACK_DUMP_RAW tap pointed at a per-condition
# file, prints what to do, and you Ctrl-C (or close the window) when done. One clean
# condition per file means zero labelling — the metrics summary is computed per file.
#
# Usage:
#   scripts/capture.sh <condition>
#   condition ∈ still yaw pitch roll x y z blink smile talk brows  (or any custom name)
#
# Output dir: $HT_DATASET_DIR (default ~/headtrack-dataset). Re-running a condition
# overwrites its file. See docs/CAPTURE_PROTOCOL.md for the full protocol.
#
# Runs `cargo run` from the checkout by default. Set HEADTRACK_BIN to use a built
# binary or an AppImage instead, so contributors without a Rust toolchain can use
# the same protocol:
#   HEADTRACK_BIN=./headtrack-rs-0.2.0-x86_64.AppImage scripts/capture.sh still
set -euo pipefail

COND="${1:-}"
OUTDIR="${HT_DATASET_DIR:-$HOME/headtrack-dataset}"

if [[ -z "$COND" ]]; then
    echo "usage: scripts/capture.sh <condition>"
    echo "  conditions: still yaw pitch roll x y z blink smile talk brows (or custom)"
    echo "  output dir: \$HT_DATASET_DIR (default ~/headtrack-dataset)"
    exit 2
fi

case "$COND" in
    still) HINT="Hold DEAD STILL. Look straight at the camera, neutral relaxed face." ;;
    yaw)   HINT="Turn head LEFT <-> RIGHT only (say 'no'). Eyes level, no tilt, no lean." ;;
    pitch) HINT="Nod UP <-> DOWN only (say 'yes'). No turn, no tilt, no lean." ;;
    roll)  HINT="Tilt head ear-to-shoulder, LEFT <-> RIGHT only. No turn, no nod, no lean." ;;
    x)     HINT="Face forward; SLIDE head LEFT <-> RIGHT (translate, do NOT turn)." ;;
    y)     HINT="Face forward; move head UP <-> DOWN (translate, do NOT nod)." ;;
    z)     HINT="Face forward; lean TOWARD and AWAY from the camera." ;;
    blink) HINT="Head DEAD STILL, facing forward. Blink normally, then a few hard blinks." ;;
    smile) HINT="Head DEAD STILL, facing forward. Smile / relax repeatedly; a few big grins." ;;
    talk)  HINT="Head DEAD STILL, facing forward. Talk continuously (read something aloud)." ;;
    brows) HINT="Head DEAD STILL, facing forward. Raise and furrow eyebrows repeatedly." ;;
    *)     HINT="Custom condition — do your thing." ;;
esac

mkdir -p "$OUTDIR"
OUT="$OUTDIR/$COND.csv"
# Repo root matters only for the cargo path; harmless otherwise.
cd "$(dirname "$0")/.." 2>/dev/null || true

cat <<EOF

=== capture: $COND ===
  $HINT
  Aim for ~15 s of clean, slow movement. Wait for the camera preview before starting.
  → $OUT
  Press Ctrl-C (or close the window) when done.

EOF

# Report the captured size on exit (the raw recorder flushes per line, so a
# Ctrl-C leaves a complete file).
#
# Stamp the pre-existing file first: without this, a run that never wrote anything
# (bad binary, camera busy, instant crash) reports whatever an earlier capture left
# behind as if it were fresh — which is how a stale clip ends up in a dataset.
BEFORE=""
[[ -f "$OUT" ]] && BEFORE=$(stat -c %Y%s "$OUT" 2>/dev/null || echo "")

report_capture() {
    echo
    if [[ ! -f "$OUT" ]]; then
        echo "no capture written: $OUT" >&2
        return
    fi
    local after
    after=$(stat -c %Y%s "$OUT" 2>/dev/null || echo "")
    if [[ -n "$BEFORE" && "$after" == "$BEFORE" ]]; then
        echo "no capture written this run — $OUT is unchanged from a previous capture." >&2
        echo "  Did the app start? Is the camera in use by something else?" >&2
        return
    fi
    local rows=$(($(wc -l < "$OUT") - 1))
    if (( rows <= 0 )); then
        echo "capture is empty: $OUT — no frames recorded." >&2
        return
    fi
    echo "saved $COND: $OUT  (${rows} frames)"
    echo "analyse: python3 scripts/filter_metrics.py --summary --raw \"$OUT\""
}
trap report_capture EXIT

# Quiet ORT, keep our tracker info lines (rotation source + per-second stats so you
# can confirm miss 0 / healthy tracking during the clip). Default mouth 0.10, 6DRepNet.
export RUST_LOG="${RUST_LOG:-warn,headtrack_input_neuralnet=info}"
export HEADTRACK_DUMP_RAW=1
export HEADTRACK_DUMP_RAW_PATH="$OUT"

if [[ -n "${HEADTRACK_BIN:-}" ]]; then
    if [[ ! -x "$HEADTRACK_BIN" ]]; then
        echo "HEADTRACK_BIN is not executable: $HEADTRACK_BIN" >&2
        echo "  (an AppImage needs: chmod +x $HEADTRACK_BIN)" >&2
        exit 1
    fi
    "$HEADTRACK_BIN"
else
    if ! command -v cargo >/dev/null; then
        echo "cargo not found, and HEADTRACK_BIN is not set." >&2
        echo "  Either install Rust, or point at a built binary / AppImage:" >&2
        echo "    HEADTRACK_BIN=./headtrack-rs-*.AppImage $0 $COND" >&2
        exit 1
    fi
    cargo run -p headtrack-gui
fi
