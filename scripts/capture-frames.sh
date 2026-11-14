#!/usr/bin/env bash
# capture-frames.sh — record one clip of full-resolution FRAMES for the detector bake-off.
#
# Companion to capture.sh, which records pose numbers. This one records the actual
# camera frames, because two face detectors can only be compared fairly on identical
# input. See docs/CAPTURE_PROTOCOL.md and the bake-off plan.
#
# Usage:
#   scripts/capture-frames.sh <condition>
#   condition ∈ yaw-sweep yaw-hold-45 yaw-hold-max pitch-extreme combined
#               dim backlit still z            (or any custom name)
#
# Output dir: $HT_BAKEOFF_DIR/<condition>/ (default ~/headtrack-bakeoff).
# Each clip gets its own directory of frame_NNNNNN.jpg plus frames.csv.
# Re-running a condition WIPES and re-records that clip.
#
# NOTE ON DISK: ~2.5 MB/s, so a 40 s clip is ~100 MB and the full set is ~1 GB.
# This deliberately writes under $HOME rather than the engine's own /tmp default,
# because /tmp is tmpfs (RAM-backed) on this and many distros.
#
# Like capture.sh, HEADTRACK_BIN can point at a built binary instead of building
# from the checkout. It must be built with `--features headtrack-engine/frame-dump`:
# release builds and the AppImage contain no frame capture, and only warn.
set -euo pipefail

COND="${1:-}"
OUTROOT="${HT_BAKEOFF_DIR:-$HOME/headtrack-bakeoff}"

if [[ -z "$COND" ]]; then
    cat >&2 <<'EOF'
usage: scripts/capture-frames.sh <condition>

  yaw-sweep      the decisive one — recall vs yaw
  yaw-hold-45    steady-state recall + noise at angle
  yaw-hold-max   same, at your physical limit
  pitch-extreme  recall vs pitch
  combined       check-six with natural roll
  dim            low light
  backlit        bright source behind you
  still          control: noise floor
  z              control: depth / IPD noise

  output dir: $HT_BAKEOFF_DIR (default ~/headtrack-bakeoff)
EOF
    exit 2
fi

case "$COND" in
    yaw-sweep)
        HINT="Turn SLOWLY left -> centre -> right -> centre, continuously. Eyes level, no tilt."
        EXTRA="Go to your real limit each way. Slow and smooth: ~40 s, 3-4 full sweeps.
  This clip decides the whole comparison — it is the one to get right." ;;
    yaw-hold-45)
        HINT="Turn ~45 degrees LEFT and HOLD ~10 s. Return to centre. Repeat to the RIGHT."
        EXTRA="Roughly where the far cheek starts to hide. Hold steady — this measures noise, not motion." ;;
    yaw-hold-max)
        HINT="Turn as far as you comfortably can LEFT, HOLD ~10 s. Centre. Repeat RIGHT."
        EXTRA="Your physical limit, past what you use in the sim. Expect the tracker to struggle; that is the point." ;;
    pitch-extreme)
        HINT="Look fully UP and HOLD ~8 s. Centre. Look fully DOWN and HOLD ~8 s. Repeat once."
        EXTRA="No turning — pitch only." ;;
    combined)
        HINT="Check six: turn hard over your LEFT shoulder with the natural head tilt. Hold. Repeat RIGHT."
        EXTRA="The realistic worst case — yaw and roll together, as you would actually do it in the cockpit." ;;
    dim)
        HINT="Turn the room lights down, then: 10 s still, followed by 2 slow yaw sweeps."
        EXTRA="Small detectors degrade fastest in low light. Dim, not dark — you should still be visible." ;;
    backlit)
        HINT="Put a bright window or screen BEHIND you, then: 10 s still, then 2 slow yaw sweeps."
        EXTRA="The other common bad case: your face is the darkest thing in frame." ;;
    still)
        HINT="Hold DEAD STILL, look straight at the camera, neutral relaxed face. ~20 s."
        EXTRA="Control clip. This sets the noise floor every other number is judged against." ;;
    z)
        HINT="Face forward; lean slowly TOWARD and AWAY from the camera. ~25 s."
        EXTRA="Control clip for depth — Z comes from interpupillary distance, so this tests the eye keypoints hardest." ;;
    *)
        HINT="Custom condition — do your thing."
        EXTRA="" ;;
esac

OUT="$OUTROOT/$COND"
cd "$(dirname "$0")/.." 2>/dev/null || true

# Wipe rather than append: frame_NNNNNN.jpg restarts at 0 every run, so leftovers
# from a previous take would interleave with this one and frames.csv would point
# at the wrong images.
rm -rf "$OUT"
mkdir -p "$OUT"

cat <<EOF

=== frame capture: $COND ===
  $HINT
EOF
[[ -n "$EXTRA" ]] && echo "  $EXTRA"
cat <<EOF

  Wait for the camera preview and for tracking to settle BEFORE you start moving.
  → $OUT
  Press Ctrl-C (or close the window) when done.

  Frame rate drops to ~22 fps while this runs. That is the JPEG encode, and it is expected.

EOF

report_capture() {
    echo
    local n
    n=$(find "$OUT" -maxdepth 1 -name 'frame_*.jpg' 2>/dev/null | wc -l)
    if (( n == 0 )); then
        echo "no frames written to $OUT" >&2
        echo "  Did the app start? Is the camera in use by something else?" >&2
        return
    fi
    local rows=0
    [[ -f "$OUT/frames.csv" ]] && rows=$(($(wc -l < "$OUT/frames.csv") - 1))
    echo "saved $COND: $n frames, $rows label rows, $(du -sh "$OUT" | cut -f1)"
    if (( rows != n )); then
        echo "  note: $rows rows vs $n images — rows without an image are frames the"
        echo "  tracker processed from grayscale only. Expected on some capture paths."
    fi
    # Detector-miss count is the headline: it is what the bake-off is about.
    if [[ -f "$OUT/frames.csv" ]]; then
        local miss
        miss=$(awk -F, 'NR>1 && $6=="none"' "$OUT/frames.csv" | wc -l)
        echo "  current tracker lost the face on $miss / $rows frames"
    fi
}
trap report_capture EXIT

export RUST_LOG="${RUST_LOG:-warn,headtrack_input_neuralnet=info}"
export HEADTRACK_DUMP_FRAMES=1
export HEADTRACK_DUMP_FRAMES_DIR="$OUT"

if [[ -n "${HEADTRACK_BIN:-}" ]]; then
    if [[ ! -x "$HEADTRACK_BIN" ]]; then
        echo "HEADTRACK_BIN is not executable: $HEADTRACK_BIN" >&2
        exit 1
    fi
    "$HEADTRACK_BIN"
else
    if ! command -v cargo >/dev/null; then
        echo "cargo not found, and HEADTRACK_BIN is not set." >&2
        exit 1
    fi
    # frame-dump is a non-default feature: release builds cannot write frames at all.
    cargo run -p headtrack-gui --features headtrack-engine/frame-dump
fi
