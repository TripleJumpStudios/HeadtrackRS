#!/usr/bin/env bash
# run-detector.sh — launch HeadtrackRS with a chosen face detector.
#
#   scripts/run-detector.sh yunet     # MIT-licensed YuNet
#   scripts/run-detector.sh scrfd     # InsightFace SCRFD (the current default)
#
# Extra arguments are passed to cargo, so:
#   scripts/run-detector.sh yunet --release
#   scripts/run-detector.sh scrfd -p headtrack-daemon
#
# WHY THIS IS A SCRIPT AND NOT A CARGO FEATURE
#
# Cargo aliases cannot set environment variables, and a feature flag would force
# a rebuild on every switch — which is exactly the wrong tax when the whole point
# is A/B-ing the two detectors back to back and tuning each. `HEADTRACK_DETECTOR`
# is read at tracker construction, so switching costs one restart and no compile.
# It is the same idiom the tracker already uses for HEADTRACK_ROT_SOURCE and
# HEADTRACK_TRANS_SOURCE.
#
# Both detectors produce the identical FaceDetection shape and the identical
# keypoint order, so everything downstream — PnP, the 6DRepNet crop, the whole
# 8-stage pipeline — is bit-for-bit the same code path. Any difference you feel
# is the detector, not the plumbing.
#
# Comparison data and the measured trade-offs: docs/DETECTOR_BAKEOFF.md
set -euo pipefail

DET="${1:-}"
shift || true

case "$DET" in
    yunet)
        BLURB="YuNet — MIT (c) 2020 Shiqi Yu, 228 KB, 640x640 input" ;;
    scrfd)
        BLURB="SCRFD — InsightFace, 16.9 MB. NON-COMMERCIAL RESEARCH LICENCE ONLY" ;;
    *)
        cat >&2 <<'EOF'
usage: scripts/run-detector.sh <yunet|scrfd> [extra cargo args]

  yunet   MIT licensed. Matched SCRFD's detection rate to 89 deg of yaw,
          2.5x faster, 74x smaller. Keypoints ~2.5x noisier at sustained
          yaw beyond ~40 deg.
  scrfd   The current default. Better keypoint precision at angle, but its
          weights are licensed for non-commercial academic research only,
          which is why the alternative exists.

See docs/DETECTOR_BAKEOFF.md for the measured comparison.
EOF
        exit 2 ;;
esac

cd "$(dirname "$0")/.."

# Keep the tracker's own per-second stats visible: the line now names the active
# detector, so you can confirm at a glance which one you are actually flying.
export RUST_LOG="${RUST_LOG:-warn,headtrack_input_neuralnet=info}"
export HEADTRACK_DETECTOR="$DET"

echo "=== detector: $BLURB ==="
echo

# Default to the GUI (the product's entry point), but only if the caller did not
# name a package themselves. `cargo run` REJECTS a repeated --package rather than
# taking the last one, so appending our default unconditionally would break
# `run-detector.sh yunet -p headtrack-daemon` with an unhelpful clap error.
want_pkg=1
for a in "$@"; do
    case "$a" in
        -p|--package|-p=*|--package=*) want_pkg=0 ;;
    esac
done

if (( want_pkg )); then
    exec cargo run -p headtrack-gui "$@"
else
    exec cargo run "$@"
fi
