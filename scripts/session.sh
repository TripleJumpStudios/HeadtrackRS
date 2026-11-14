#!/usr/bin/env bash
# session.sh — record one instrumented flying session, named so the set stays sortable.
#
#   scripts/session.sh <game> <detector> <label>
#
#   scripts/session.sh xplane yunet sweep
#   scripts/session.sh msfs   scrfd pattern-left
#   scripts/session.sh sc     yunet arena
#
#   game     ∈ xplane | msfs | sc
#   detector ∈ yunet  | scrfd
#   label      free text: sweep, pattern-left, pattern-right, arena, ...
#
# Output: $HT_AB_DIR/<game>/<detector>-<label>.csv   (default ~/headtrack-ab)
#
# WHAT GETS RECORDED, AND WHY IT IS TWO FILES FOR MSFS
#
# The raw pose dump is captured for every run. It is PRE-pipeline — the tracker's
# own output, before smoothing, centring or any output adapter. That is the right
# signal for comparing DETECTORS, because nothing downstream can hide or add to
# what the detector produced.
#
# It is also, for the same reason, blind to the game: the tracker does not know
# which sim is running, so an xplane raw dump and an msfs raw dump differ only
# because you moved differently. If you are chasing a per-game tick, the raw dump
# cannot show it.
#
# So where a game offers an output-side tap, this turns it on too:
#
#   msfs    HEADTRACK_MSFS_TRACE — every CameraSet: target vs actually-sent,
#           the frame rate the sim reported, and any gate skips. By far the best
#           instrumented of the three.
#   xplane  no built-in trace. For post-pipeline data run, in a second terminal:
#             cargo run -p headtrack-tester -- --dump <file>
#           which records what the plugin is actually being handed.
#   sc      scripts/shm_dump.py reads the shared segment the DLL reads.
#
# ORDER EFFECTS: alternate which detector goes first between runs. Lighting
# drifts, you get tired, and you learn the pattern — all of which favour whichever
# went second if you always run them in the same order.
set -euo pipefail

GAME="${1:-}"; DET="${2:-}"; LABEL="${3:-}"
ROOT="${HT_AB_DIR:-$HOME/headtrack-ab}"

usage() {
    cat >&2 <<'EOF'
usage: scripts/session.sh <game> <detector> <label>

  game      xplane | msfs | sc
  detector  yunet  | scrfd
  label     sweep | pattern-left | pattern-right | arena | ...

examples:
  scripts/session.sh xplane yunet sweep
  scripts/session.sh msfs   scrfd pattern-left
  scripts/session.sh sc     yunet arena
EOF
    exit 2
}

case "$GAME" in xplane|msfs|sc) ;; *) usage ;; esac
case "$DET"  in yunet|scrfd)    ;; *) usage ;; esac
[[ -n "$LABEL" ]] || usage

cd "$(dirname "$0")/.."

OUTDIR="$ROOT/$GAME"
mkdir -p "$OUTDIR"
RAW="$OUTDIR/$DET-$LABEL.csv"
TRACE="$OUTDIR/$DET-$LABEL.msfs-trace.csv"

# Refuse to clobber: with a dozen-plus runs, silently overwriting one is a whole
# sortie thrown away and you would not notice until analysis.
if [[ -e "$RAW" ]]; then
    echo "ERROR: $RAW already exists." >&2
    echo "  Pick another label, or delete it first if you meant to redo this run." >&2
    exit 1
fi

# Tracker lifecycle AND the engine/output side. The first version of this script
# set only `headtrack_input_neuralnet=info`, which silently hid every SimConnect
# line — including the bring-up `stage` reminder that says why MSFS is not
# connected. A diagnostic session that suppresses the diagnostics is worse than
# no logging at all, because the absence looks like evidence.
#
# All three are one-shot lifecycle events per the project's logging contract, so
# this stays quiet in normal running rather than spamming per frame.
export RUST_LOG="${RUST_LOG:-warn,headtrack_input_neuralnet=info,headtrack_engine=info,headtrack_output_simconnect=info}"
export HEADTRACK_DETECTOR="$DET"
export HEADTRACK_DUMP_RAW=1
export HEADTRACK_DUMP_RAW_PATH="$RAW"
[[ "$GAME" == "msfs" ]] && export HEADTRACK_MSFS_TRACE="$TRACE"

cat <<EOF

=== session: $GAME / $DET / $LABEL ===
  raw pose  -> $RAW
EOF
[[ "$GAME" == "msfs" ]] && echo "  msfs trace-> $TRACE"
cat <<EOF

  Start the sim if it is not already up — start order no longer matters for any
  of the three. Fly the profile, then close the HeadtrackRS window when done.

EOF

report() {
    echo
    if [[ ! -s "$RAW" ]]; then
        echo "NO DATA written to $RAW" >&2
        echo "  Tracking never produced a pose — was your face in frame?" >&2
        return
    fi
    local rows
    rows=$(grep -cv '^#' "$RAW" 2>/dev/null || echo 0)
    rows=$((rows - 1))   # header
    echo "saved: $RAW  ($rows frames)"
    [[ -s "$TRACE" ]] && echo "saved: $TRACE  ($(( $(wc -l < "$TRACE") - 1 )) CameraSet rows)"
    echo
    echo "compare when you have a matching pair:"
    echo "  python3 scripts/filter_metrics.py --summary --dir $OUTDIR"
}
trap report EXIT

exec cargo run --release -p headtrack-gui
