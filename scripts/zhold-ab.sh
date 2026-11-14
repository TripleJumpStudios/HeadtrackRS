#!/usr/bin/env bash
# zhold-ab.sh — matched A/B of the Z-gate hold rate, SCRFD vs YuNet.
#
#   scripts/zhold-ab.sh [seconds]      # default 30 s per detector
#
# WHAT THIS ANSWERS
#
# A live YuNet run showed `z-hold` firing on 60 of 60 frames, meaning the Z gate
# rejected every depth measurement and froze Z. The captured bake-off frames do
# NOT reproduce that — simulated at the live 60 fps step limit, YuNet trips 0.3 %
# on a still clip, identical to SCRFD. So either something about the live path
# differs from replay, or the behaviour is not detector-specific at all and has
# been there all along.
#
# That second possibility is the important one: if SCRFD holds just as hard, the
# depth cue has been frozen in normal use and the detector is a bystander.
#
# You cannot read this off the terminal while sitting still in front of the
# camera, so this script runs both, captures the logs, and prints the comparison
# at the end. Just sit normally and look at the camera.
#
# See docs/DETECTOR_BAKEOFF.md §5.
set -euo pipefail

SECS="${1:-30}"
OUT="${TMPDIR:-/tmp}/headtrack-zhold-ab"
mkdir -p "$OUT"
cd "$(dirname "$0")/.."

cat <<EOF

=== Z-gate hold A/B — ${SECS}s per detector ===

  Sit normally in front of the camera, face visible, for BOTH runs.
  Stay roughly still: this measures the noise floor, not motion.
  Do not touch the terminal — the summary prints at the end.

EOF
read -r -p "Press Enter when you are in position... " _ || true

run_one() {
    local det="$1"
    echo
    echo ">>> $det — ${SECS}s. Look at the camera."
    RUST_LOG=warn,headtrack_input_neuralnet=debug \
    HEADTRACK_DETECTOR="$det" \
        timeout "$SECS" cargo run -q -p headtrack-daemon > "$OUT/$det.log" 2>&1 || true
    echo "    done."
}

# Build once up front so the countdown is not eaten by compilation.
echo ">>> building..."
cargo build -q -p headtrack-daemon
run_one scrfd
run_one yunet

echo
echo "=============================================================="
echo " Z-GATE HOLD RATE — frames where the depth measurement was"
echo " rejected and Z frozen. Only windows with a tracked face count."
echo "=============================================================="
for det in scrfd yunet; do
    log="$OUT/$det.log"
    # Per-second stats lines look like:
    #   pipeline 60f: ... miss 12, ... z-hold 43 | infer ...
    # A window with `miss` equal to the frame count saw no face at all, so it
    # carries no depth measurement to hold and must not dilute the average.
    awk -v det="$det" '
        match($0, /pipeline ([0-9]+)f:/, m) {
            frames = m[1]
            miss = 0; hold = 0
            if (match($0, /miss ([0-9]+)/, a)) miss = a[1]
            if (match($0, /z-hold ([0-9]+)/, b)) hold = b[1]
            tracked = frames - miss
            if (tracked <= 0) next
            n++; tf += tracked; th += hold
            if (hold >= tracked) full++
        }
        END {
            if (n == 0) { printf "  %-7s no windows with a tracked face — was your face in frame?\n", det; exit }
            printf "  %-7s %6.1f%% of tracked frames held   (%d/%d over %d windows; %d window(s) at 100%%)\n", \
                   det, 100*th/tf, th, tf, n, full
        }' "$log"
done
echo
echo "  logs: $OUT/{scrfd,yunet}.log"
echo
echo "  Reading it:"
echo "   - both LOW (<5%)          the live 60/60 was a fluke or a transient; YuNet is clear"
echo "   - both HIGH               pre-existing Z-gate defect, NOT the detector — the more"
echo "                             serious finding, and it means Z has been frozen in normal use"
echo "   - only yunet HIGH         genuine YuNet regression; needs a fix before it can be default"
