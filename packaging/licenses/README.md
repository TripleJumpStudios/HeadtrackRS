# Licence texts shipped in the AppImage

`build-appimage.sh` copies every `*.txt` file here into `usr/share/licenses/headtrack-rs/`,
with `LICENSE`, `THIRD_PARTY_NOTICES.md` and the three OFL texts of the fonts in `assets/fonts/`.

| File | Source |
|---|---|
| `LICENSE-YuNet.txt` | opencv_zoo `models/face_detection_yunet/LICENSE` (main), fetched by the taskmaster 2026-10-18; copy of `docs/evidence/q3/opencv_zoo-face_detection_yunet-LICENSE` |
| `LICENSE-6DRepNet-head-pose-estimation.txt` | MIT text with the copyright line of `yakhyo/head-pose-estimation`'s LICENSE (main), as recorded by the taskmaster 2026-10-18 |
| `ONNXRUNTIME-1.24.2-ThirdPartyNotices.txt` | microsoft/onnxruntime `v1.24.2/ThirdPartyNotices.txt`; copy of `docs/evidence/q3/onnxruntime-1.24.2-ThirdPartyNotices.txt` (sha256 `0e07b95f…`) |
| `LICENSE-egui-Ubuntu-Light-UFL-1.0.txt` | crate `epaint_default_fonts` 0.31.1, `fonts/UFL.txt` (Ubuntu Font Licence 1.0, for Ubuntu-Light), copied 2026-11-14 (sha256 `2f001510…`) |
| `LICENSE-egui-NotoEmoji-OFL-1.1.txt` | same crate, `fonts/OFL.txt` (SIL OFL 1.1, for NotoEmoji-Regular) (sha256 `6a73f954…`) |
| `LICENSE-egui-Hack.txt` | same crate, `fonts/Hack-Regular.txt` (MIT, plus the Bitstream Vera licence it carries) (sha256 `47c0cccb…`) |
| `LICENSE-egui-emoji-icon-font-MIT.txt` | same crate, `fonts/emoji-icon-font-mit-license.txt` (MIT, © 2014 John Slegers) (sha256 `b9d2c1d9…`) |
