#!/usr/bin/env bash
# Build a self-contained AppImage for headtrack-rs.
#
# Usage: ./packaging/appimage/build-appimage.sh
#
# Requirements:
#   - Rust toolchain (cargo)
#   - libonnxruntime installed (dnf install onnxruntime  or  apt install libonnxruntime)
#   - curl (to download appimagetool on first run)
#
# Output: headtrack-rs-<version>-x86_64.AppImage in the workspace root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARCH="x86_64"

# Extract version from workspace Cargo.toml.
VERSION=$(grep '^version' "$ROOT/Cargo.toml" | head -1 | sed 's/version = "\(.*\)"/\1/')
APPDIR="$SCRIPT_DIR/AppDir"
OUTPUT="$ROOT/headtrack-rs-${VERSION}-${ARCH}.AppImage"

# ---------------------------------------------------------------------------
# 1. appimagetool
# ---------------------------------------------------------------------------
APPIMAGETOOL="$SCRIPT_DIR/appimagetool-${ARCH}.AppImage"
if [[ ! -x "$APPIMAGETOOL" ]]; then
    echo ">>> Downloading appimagetool..."
    curl -fsSL \
        "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-${ARCH}.AppImage" \
        -o "$APPIMAGETOOL"
    chmod +x "$APPIMAGETOOL"
fi

# ---------------------------------------------------------------------------
# 2. Release builds
# ---------------------------------------------------------------------------
echo ">>> Building headtrackd + headtrack-gui + headtrack-xplane (release)..."
cd "$ROOT"
cargo build -p headtrack-daemon -p headtrack-gui -p headtrack-xplane --release

echo ">>> Building NPClient64.dll (x86_64-pc-windows-gnu)..."
cargo build --manifest-path crates/headtrack-npclient/Cargo.toml \
    --target x86_64-pc-windows-gnu --release --target-dir target/

# ---------------------------------------------------------------------------
# 3. Assemble AppDir
# ---------------------------------------------------------------------------
echo ">>> Assembling AppDir..."
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/lib" "$APPDIR/models" "$APPDIR/bridge"

# Linux binaries
cp target/release/headtrackd    "$APPDIR/usr/bin/headtrackd"
cp target/release/headtrack-gui "$APPDIR/usr/bin/headtrack-gui"
strip "$APPDIR/usr/bin/headtrackd" "$APPDIR/usr/bin/headtrack-gui"

# X-Plane plugin — placed next to the GUI binary so the in-app installer
# finds it via exe.parent().join("libheadtrack_xplane.so").
cp target/release/libheadtrack_xplane.so "$APPDIR/usr/bin/libheadtrack_xplane.so"

# AppRun
cp "$SCRIPT_DIR/AppRun" "$APPDIR/AppRun"
chmod +x "$APPDIR/AppRun"

# Desktop file
cat > "$APPDIR/headtrack-rs.desktop" << 'EOF'
[Desktop Entry]
Type=Application
Name=Headtrack RS
Exec=headtrack-gui
Icon=headtrack-rs
Categories=Utility;Game;
EOF

# Icon
cp "$ROOT/assets/icons/headtrack-rs.png" "$APPDIR/headtrack-rs.png"

# ONNX models
echo ">>> Copying models..."
cp "$ROOT/assets/models"/*.onnx "$APPDIR/models/"

# libonnxruntime — resolve via ldconfig, dereference symlinks with cp -L.
echo ">>> Bundling libonnxruntime..."
ORT_SO=$(ldconfig -p 2>/dev/null | grep 'libonnxruntime\.so' | grep -v providers | awk '{print $NF}' | head -1)
ORT_PS=$(ldconfig -p 2>/dev/null | grep 'libonnxruntime_providers_shared' | awk '{print $NF}' | head -1)

if [[ -z "$ORT_SO" ]]; then
    echo "ERROR: libonnxruntime.so not found." >&2
    echo "  Fedora:  sudo dnf install onnxruntime" >&2
    echo "  Ubuntu:  sudo apt install libonnxruntime" >&2
    echo "  Or set ORT_DYLIB_PATH to the .so path and re-run." >&2
    exit 1
fi

cp -L "$ORT_SO" "$APPDIR/lib/"
[[ -n "$ORT_PS" ]] && cp -L "$ORT_PS" "$APPDIR/lib/"

# Unversioned symlinks so ort's dlopen("libonnxruntime.so") resolves.
(cd "$APPDIR/lib" && for f in libonnxruntime*.so.*; do
    base="${f%%.so.*}.so"
    [[ -e "$base" ]] || ln -s "$f" "$base"
done)

# NPClient64.dll — Star Citizen Wine bridge, placed in bridge/ so the
# in-app installer finds it via exe.parent().join("bridge/NPClient64.dll").
echo ">>> Copying NPClient64.dll..."
cp target/x86_64-pc-windows-gnu/release/NPClient64.dll "$APPDIR/bridge/"

# ---------------------------------------------------------------------------
# 4. Build AppImage
# ---------------------------------------------------------------------------
echo ">>> Running appimagetool..."
ARCH="$ARCH" APPIMAGE_EXTRACT_AND_RUN=1 "$APPIMAGETOOL" "$APPDIR" "$OUTPUT"

echo ""
echo "Done: $OUTPUT"
echo ""
echo "Install wrappers (and optional systemd service file):"
echo "  bash packaging/systemd/install.sh $OUTPUT"
echo ""
echo "Then launch with:  headtrack-gui"
