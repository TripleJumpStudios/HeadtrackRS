#!/usr/bin/env bash
# res-ab.sh — capture resolution A/B, with everything else held fixed.
#
#   scripts/res-ab.sh low     # 960x540   (today's behaviour)
#   scripts/res-ab.sh high    # 1920x1080 (2x linear)
#
# WHY THIS TEST
#
# Depth error is e·Z²/(f·B), with e the landmark error in pixels. Two facts
# decide whether resolution helps:
#
#   * The DETECTORS letterbox the whole frame to a fixed 640 (YuNet) / 320
#     (SCRFD), so on a detect frame a bigger capture buys nothing.
#   * The detector runs once per ~300 frames. OPTICAL FLOW supplies the other
#     299, and it tracks at full capture resolution.
#
# Flow's error is roughly constant in pixels while f scales with resolution, so
# doubling capture resolution should nearly halve depth noise on the 99.7% of
# frames flow provides. That is the prediction under test — it is a prediction,
# not a result, and three predictions in this investigation have already failed.
#
# It matters because seating distance is a user variable we cannot control, and
# Z error grows with its SQUARE (measured: Z ∝ d^2.00). Resolution is the only
# lever found so far that pushes back on that without asking users to move.
#
# WHAT ELSE IS IN THIS BUILD
#
#   * depth cue defaults to `diagonals`
#   * recenter builds its baseline from a 1 s median instead of one frame
#   * translation deadzones scale with measured standoff
#
# HOW TO FLY IT: identical profile both runs, or the comparison is worthless.
# Sit where you normally sit and DO NOT adjust the chair between runs — this is
# a resolution test, and standoff is the one thing that would confound it.
#
#   1. settle, look ahead, still                        ~15 s
#   2. normal instrument scan, both sides                ~60 s
#   3. park, head still, ~30 s   <- the money segment
#   4. press recenter 3-4 times, deliberately including
#      once when the view looks WRONG                   ~30 s
#   5. park again, head still                           ~30 s
#
# Step 4 is the recenter test: previously a press had a ~45% chance of baking in
# >50 mm of error. Press it when it looks worst — that is the case that failed.
set -euo pipefail

MODE="${1:-}"
case "$MODE" in
    low)  RES="960x540"   ;;
    high) RES="1920x1080" ;;
    *) echo "usage: scripts/res-ab.sh <low|high>" >&2; exit 2 ;;
esac

cd "$(dirname "$0")/.."
OUT="${HT_AB_DIR:-$HOME/headtrack-ab}/msfs"
mkdir -p "$OUT"
LABEL="res-$MODE"

if [[ -e "$OUT/yunet-$LABEL.csv" ]]; then
    echo "ERROR: $OUT/yunet-$LABEL.csv exists — that is a whole sortie." >&2
    echo "  Move it aside or pick a new label before redoing this run." >&2
    exit 1
fi

export HEADTRACK_CAPTURE_RES="$RES"
export HEADTRACK_Z_GATE_TRACE="$OUT/$LABEL.zgate.csv"

echo
echo "=== capture resolution A/B: $MODE ($RES) ==="
echo "    cue=diagonals (default)   recenter=1s median   deadzones=distance-scaled"
echo "    DO NOT move your chair between the two runs."
echo

exec scripts/session.sh msfs yunet "$LABEL"
