#!/usr/bin/env bash
# Install headtrack-rs as a systemd user service.
#
# Usage: bash packaging/systemd/install.sh <path-to-headtrack-rs-*.AppImage>
#
# Creates:
#   ~/.local/bin/headtrack-gui    — symlink → AppImage (GUI, default entry point)
#   ~/.local/bin/headtrackd       — symlink → AppImage (daemon-only, power users)
#   ~/.config/systemd/user/headtrackd.service  (not enabled by default)

set -euo pipefail

APPIMAGE="${1:?Usage: $0 <path-to-headtrack-rs-*.AppImage>}"
APPIMAGE="$(realpath "$APPIMAGE")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "$APPIMAGE" ]]; then
    echo "ERROR: AppImage not found: $APPIMAGE" >&2
    exit 1
fi

mkdir -p "$HOME/.local/bin" "$HOME/.config/systemd/user"

# Both entries are symlinks — AppRun dispatches based on ARGV0 (the symlink name).
# headtrack-gui → default GUI dispatch
# headtrackd    → daemon-only dispatch (power users / systemd service)
GUI_LINK="$HOME/.local/bin/headtrack-gui"
ln -sf "$APPIMAGE" "$GUI_LINK"
echo "Installed symlink: $GUI_LINK -> $APPIMAGE"

DAEMON_LINK="$HOME/.local/bin/headtrackd"
ln -sf "$APPIMAGE" "$DAEMON_LINK"
echo "Installed symlink: $DAEMON_LINK -> $APPIMAGE"

# Systemd service for the daemon.
SERVICE_DEST="$HOME/.config/systemd/user/headtrackd.service"
cp "$SCRIPT_DIR/headtrackd.service" "$SERVICE_DEST"
echo "Installed service:  $SERVICE_DEST"

systemctl --user daemon-reload

echo ""
echo "Installed. The GUI spawns and owns the daemon automatically — no service needed."
echo "Launch with:  headtrack-gui"
