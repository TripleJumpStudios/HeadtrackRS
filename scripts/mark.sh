#!/usr/bin/env bash
# mark.sh — drop a timestamped, zero-side-effect marker into the running capture.
#
#   scripts/mark.sh landing
#   scripts/mark.sh parked
#   scripts/mark.sh checklist
#
# Sends `set marker <label> 1` on the command socket. `marker` is not a real
# pipeline stage, so the engine logs it and changes nothing — but the param
# log-side channel writes it into the raw-dump CSV as a `#` comment with the
# frame timestamp, which is what makes a phase locatable afterwards.
#
# This matters more than it sounds. Two analyses this week were weakened because
# a protocol phase could not be located in the trace and had to be inferred from
# head stillness.
set -euo pipefail

LABEL="${1:-}"
if [[ -z "$LABEL" ]]; then
    echo "usage: scripts/mark.sh <label>    e.g. landing | parked | checklist" >&2
    exit 2
fi
# Keep it to one token so it round-trips through the socket's text protocol.
LABEL="${LABEL//[^A-Za-z0-9_-]/-}"

SOCK="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/headtrack-cmd.sock"
if [[ ! -S "$SOCK" ]]; then
    echo "no command socket at $SOCK — is HeadtrackRS running?" >&2
    exit 1
fi

if command -v socat >/dev/null 2>&1; then
    echo "set marker $LABEL 1" | socat - "UNIX-CONNECT:$SOCK"
elif command -v nc >/dev/null 2>&1; then
    echo "set marker $LABEL 1" | nc -U -w1 "$SOCK"
else
    echo "need socat or nc to talk to the command socket" >&2
    exit 1
fi
echo "marked: $LABEL"
