#!/usr/bin/env bash
# Build a self-contained AppImage for headtrack-rs.
#
# Usage: ./packaging/appimage/build-appimage.sh
#
# Requirements:
#   - Rust toolchain (cargo) with the x86_64-pc-windows-gnu target + mingw64 gcc
#   - ORT_LIB_PATH pointing at a static libonnxruntime.a: ONNX Runtime is linked
#     into the binaries, so no libonnxruntime.so is bundled (the script refuses
#     to package a binary that still links one dynamically)
#   - curl (to download appimagetool on first run)
#
# Environment:
#   CARGO_TARGET_DIR          honoured (default: target/)
#   HEADTRACK_APPIMAGE_OUT    output path (default: the workspace root)
#
# Output: headtrack-rs-<version>-x86_64.AppImage in the workspace root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARCH="x86_64"

# Extract version from workspace Cargo.toml.
VERSION=$(grep '^version' "$ROOT/Cargo.toml" | head -1 | sed 's/version = "\(.*\)"/\1/')
APPDIR="$SCRIPT_DIR/AppDir"
OUTPUT="${HEADTRACK_APPIMAGE_OUT:-$ROOT/headtrack-rs-${VERSION}-${ARCH}.AppImage}"
TARGET="${CARGO_TARGET_DIR:-target}"
[[ "$TARGET" = /* ]] || TARGET="$ROOT/$TARGET"

# Every model file that ships, with its sha256. Explicit on purpose: a glob
# once bundled whatever happened to sit in assets/models (SCRFD, whose weights
# are non-commercial-only). Anything not listed here does not ship. The Tobii
# set mirrors assets/models/tobii/MANIFEST.json plus that manifest and NOTICE,
# minus Google's two model-card PDFs: they embed photographs, and no image of a
# person ships (P16). THIRD_PARTY_NOTICES.md links the originals.
MODEL_MANIFEST=(
    "face_detection_yunet_2023mar.onnx            8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4"
    "head-pose-mobilenetv3-large.onnx             3a68815fa00aba41ddc4e014bf631b637caba8619df71160383f1fee8c15a3c9"
    "tobii/face_detector.tflite                   b4578f35940bf5a1a655214a1cce5cab13eba73c1297cd78e1a04c2380b0152f"
    "tobii/face_landmarks_detector.tflite         c7d54204ce0448474c7f3fa9af494787c0965cbdd6f20fc72867e43046bd43d5"
    "tobii/canonical_face_model.obj               8bac80443397e113f41a8b565ea72c59390bc031d9defab289dba7bc0c54e618"
    "tobii/geometry.pbtxt                         55ca7c4c7bf195673b9d12a1b5d49d4b9c82b5ade8fed4b565496590866e359d"
    "tobii/LICENSE-MEDIAPIPE                      8707eef0533987efc5b155d64761eeb6e20793f50b9bd1a68dad1cf4719d0ed8"
    "tobii/MANIFEST.json                          -"
    "tobii/NOTICE.md                              -"
)

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
# No build-machine paths in what ships: panic locations and any debug info
# name sources as /build/... (registry, toolchain) and ./... (this tree)
# instead of the builder's home directory.
#
# The explicit --target keeps these RUSTFLAGS off build scripts and proc
# macros (cargo applies RUSTFLAGS to host units only when no --target is
# given). Nothing of those ships, and leaving them alone means their cached
# builds are reused. Release artifacts therefore land in $TARGET/$HOST/release.
HOST_TRIPLE="x86_64-unknown-linux-gnu"
REMAP="--remap-path-prefix=$HOME=/build --remap-path-prefix=$ROOT=."
REL="$TARGET/$HOST_TRIPLE/release"

echo ">>> Building headtrackd + headtrack-gui + headtrack-xplane (release)..."
cd "$ROOT"
RUSTFLAGS="${RUSTFLAGS:-} $REMAP" \
    cargo build -p headtrack-daemon -p headtrack-gui -p headtrack-xplane --release \
    --target "$HOST_TRIPLE"
for bin in headtrackd headtrack-gui; do
    if ldd "$REL/$bin" | grep -q libonnxruntime; then
        echo "ERROR: $bin links libonnxruntime.so dynamically; this script bundles none." >&2
        echo "  Build with ORT_LIB_PATH set to a static libonnxruntime.a." >&2
        exit 1
    fi
done

echo ">>> Building NPClient64.dll (x86_64-pc-windows-gnu)..."
# A RUSTFLAGS variable replaces .cargo/config.toml's rustflags for this
# target, so +crt-static (no libgcc/winpthread DLLs for Wine to find) is
# repeated here.
RUSTFLAGS="${RUSTFLAGS:-} -C target-feature=+crt-static $REMAP" \
    cargo build --manifest-path crates/headtrack-npclient/Cargo.toml \
    --target x86_64-pc-windows-gnu --release --target-dir "$TARGET"

# ---------------------------------------------------------------------------
# 3. Assemble AppDir
# ---------------------------------------------------------------------------
echo ">>> Assembling AppDir..."
rm -rf "$APPDIR"
LICDIR="$APPDIR/usr/share/licenses/headtrack-rs"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/models/tobii" "$APPDIR/bridge" "$APPDIR/udev" "$LICDIR"

# Linux binaries
cp "$REL/headtrackd"    "$APPDIR/usr/bin/headtrackd"
cp "$REL/headtrack-gui" "$APPDIR/usr/bin/headtrack-gui"
strip "$APPDIR/usr/bin/headtrackd" "$APPDIR/usr/bin/headtrack-gui"

# X-Plane plugin — placed next to the GUI binary so the in-app installer
# finds it via exe.parent().join("libheadtrack_xplane.so").
cp "$REL/libheadtrack_xplane.so" "$APPDIR/usr/bin/libheadtrack_xplane.so"

# AppRun
cp "$SCRIPT_DIR/AppRun" "$APPDIR/AppRun"
chmod +x "$APPDIR/AppRun"

# Desktop file
cat > "$APPDIR/headtrack-rs.desktop" << 'EOF'
[Desktop Entry]
Type=Application
Name=HeadtrackRS
Exec=headtrack-gui
Icon=headtrack-rs
Categories=Utility;Game;
EOF

# Icon: the top-level PNG is what AppImage tools and the .desktop's
# Icon=headtrack-rs resolve; the hicolor set is for desktop integrators.
cp "$ROOT/assets/icons/headtrack-rs.png" "$APPDIR/headtrack-rs.png"
for size in 16 32 48 64 128 256 512; do
    mkdir -p "$APPDIR/usr/share/icons/hicolor/${size}x${size}/apps"
    cp "$ROOT/assets/icons/headtrack-rs-$size.png" \
        "$APPDIR/usr/share/icons/hicolor/${size}x${size}/apps/headtrack-rs.png"
done
mkdir -p "$APPDIR/usr/share/icons/hicolor/scalable/apps"
cp "$ROOT/assets/icons/headtrack-rs.svg" "$APPDIR/usr/share/icons/hicolor/scalable/apps/headtrack-rs.svg"

# Models — exactly MODEL_MANIFEST, each verified against its pinned hash.
echo ">>> Copying models (explicit manifest)..."
for entry in "${MODEL_MANIFEST[@]}"; do
    read -r rel want <<< "$entry"
    src="$ROOT/assets/models/$rel"
    if [[ ! -f "$src" ]]; then
        echo "ERROR: manifest model missing: assets/models/$rel" >&2
        exit 1
    fi
    if [[ "$want" != "-" ]]; then
        got=$(sha256sum "$src" | awk '{print $1}')
        if [[ "$got" != "$want" ]]; then
            echo "ERROR: sha256 mismatch for assets/models/$rel" >&2
            echo "  want $want" >&2
            echo "  got  $got" >&2
            exit 1
        fi
    fi
    cp "$src" "$APPDIR/models/$rel"
done

# Tobii USB access rule — the same bytes the GUI's "Set up USB access" button
# installs (compiled in from here), shipped for users who prefer to copy it.
cp "$ROOT/packaging/udev/70-headtrackrs-tobii.rules" "$APPDIR/udev/"

# Licences: ours, the third-party notices, and the OFL texts the embedded
# fonts require to travel with them.
cp "$ROOT/LICENSE" "$ROOT/THIRD_PARTY_NOTICES.md" "$LICDIR/"
cp "$ROOT/assets/fonts/LICENSE-Inter.txt" "$ROOT/assets/fonts/LICENSE-JetBrainsMono.txt" \
    "$ROOT/assets/fonts/LICENSE-NotoSans.txt" "$LICDIR/"
# Model, ONNX Runtime and egui default-font licence texts (packaging/licenses/README.md
# says where each is from).
cp "$ROOT/packaging/licenses/"*.txt "$LICDIR/"

# NPClient64.dll — Star Citizen Wine bridge, placed in bridge/ so the
# in-app installer finds it via exe.parent().join("bridge/NPClient64.dll").
echo ">>> Copying NPClient64.dll..."
cp "$TARGET/x86_64-pc-windows-gnu/release/NPClient64.dll" "$APPDIR/bridge/"

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
