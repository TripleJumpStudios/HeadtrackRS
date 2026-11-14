# Third-party notices

HeadtrackRS is MIT-licensed (`LICENSE`). It ships, or links into its binaries, the
third-party works below. This file travels with every build: the AppImage carries it in
`usr/share/licenses/headtrack-rs/`, together with `LICENSE`, the three font licences in
`assets/fonts/` and the licence texts in `packaging/licenses/` (YuNet, 6DRepNet, ONNX Runtime
third-party notices, and the four licences of egui's default fonts).

Generated and checked 2026-10-18 (release task P14). The Rust dependency list in §8 is
produced by `cargo tree -e normal` over the shipped packages; it is not edited by hand.

## 1. Fonts (embedded in `headtrack-gui`)

| Font | Copyright | Licence | Licence text |
|---|---|---|---|
| Inter (Regular, SemiBold) | © 2016 The Inter Project Authors | SIL Open Font License 1.1 | `assets/fonts/LICENSE-Inter.txt` |
| Noto Sans (SemiBold; the sidebar wordmark only) | © 2022 The Noto Project Authors | SIL Open Font License 1.1 | `assets/fonts/LICENSE-NotoSans.txt`. The licence is the one the font declares in its own name table (ID 13); Fedora's packaged `google-noto-fonts-common/LICENSE` is a stale Apache 2.0 text. |
| JetBrains Mono (Regular, Bold) | © 2020 The JetBrains Mono Project Authors | SIL Open Font License 1.1 | `assets/fonts/LICENSE-JetBrainsMono.txt` |
| Phosphor Icons (via the `egui-phosphor` crate) | © 2023 Phosphor Icons; crate © 2023 Romet Tagobert | MIT | crate's `LICENSE-MIT` |
| egui default fonts (`epaint_default_fonts` 0.31.1): Ubuntu Light, Hack, Noto Emoji, emoji-icon-font | Canonical Ltd; © 2018 Source Foundry Authors (Hack; Bitstream Vera © 2003 Bitstream Inc.); Google; © 2014 John Slegers (emoji-icon-font) | Ubuntu Font Licence 1.0; MIT plus the Bitstream Vera Licence (Hack); SIL OFL 1.1 (Noto Emoji); MIT (emoji-icon-font) | `LICENSE-egui-Ubuntu-Light-UFL-1.0.txt`, `LICENSE-egui-Hack.txt`, `LICENSE-egui-NotoEmoji-OFL-1.1.txt`, `LICENSE-egui-emoji-icon-font-MIT.txt`, copied unchanged from the crate's `fonts/` |

The OFL and the Ubuntu Font Licence require their text to accompany the fonts. All four OFL texts
(Inter, Noto Sans, JetBrains Mono, Noto Emoji), the Ubuntu Font Licence and the MIT texts of Hack
and emoji-icon-font are bundled.

## 2. Face detector: YuNet

`assets/models/face_detection_yunet_2023mar.onnx` (sha256 `8f2383e4…2552fa4`), from the OpenCV
Zoo (`models/face_detection_yunet`), unmodified. MIT License, Copyright (c) 2020 Shiqi Yu
<shiqi.yu@gmail.com>. Full text: `LICENSE-YuNet.txt`.

## 3. Head-pose model: 6DRepNet (MobileNetV3-Large)

`assets/models/head-pose-mobilenetv3-large.onnx` (sha256 `3a68815f…8c15a3c9`), from
[`yakhyo/head-pose-estimation`](https://github.com/yakhyo/head-pose-estimation): release asset
`weights/mobilenetv3_large.onnx`, byte-identical (the same sha256; also published under release
v0.0.1). Unmodified. MIT License, Copyright (c) 2024 Yakhyokhuja Valikhujaev. Full text:
`LICENSE-6DRepNet-head-pose-estimation.txt`. The model implements 6DRepNet (Hempel, Abdelrahman and
Al-Hamadi, *6D Rotation Representation for Unconstrained Head Pose Estimation*, 2022) on a
MobileNetV3-Large backbone. Provenance confirmed 2026-10-18 by hash match.

## 4. Tobii head estimator: Google MediaPipe models (Apache-2.0)

`assets/models/tobii/`, shipped as a set:
- `face_detector.tflite` (BlazeFace short range) and `face_landmarks_detector.tflite` (Face Mesh V2), unchanged from Google's version-1 `face_landmarker` bundle;
- **not** the two Google model cards. They state the Apache-2.0 licence of the weights, but
  they embed photographs, and nothing that shows a person ships. The originals:
  [BlazeFace (short range)](https://storage.googleapis.com/mediapipe-assets/MediaPipe%20BlazeFace%20Model%20Card%20%28Short%20Range%29.pdf),
  [Face Mesh V2](https://storage.googleapis.com/mediapipe-assets/Model%20Card%20MediaPipe%20Face%20Mesh%20V2.pdf).
  `MANIFEST.json` keeps their sha256 as provenance;
- the canonical geometry (`canonical_face_model.obj`, `geometry.pbtxt`, MediaPipe v0.10.21);
- `LICENSE-MEDIAPIPE` (Apache-2.0), `NOTICE.md`, and `MANIFEST.json` with the sha256 of every file.

Copyright 2020 The MediaPipe Authors, Apache-2.0. The derived `geometry_data.rs` in
`crates/headtrack-input-tobii` keeps its Apache attribution. Both `.tflite` files are
sha256-verified before they are parsed. No MediaPipe runtime ships; inference runs in tract.

### tract and the local patch

Inference uses [tract](https://github.com/sonos/tract) (Sonos; MIT OR Apache-2.0).
`vendor/tract-tflite/` is tract-tflite 0.23.6 with two narrow import additions for these models:
constant float16 `DEQUANTIZE` promotion and `PReLU` composition. It is used via
`[patch.crates-io]` in the workspace `Cargo.toml`. The patched snapshot is pinned by
`vendor/tract-tflite/TRACT_SNAPSHOT.json` and checked by the test `tract_snapshot_hashes_verify`.
Upstream's `LICENSE-MIT` and `LICENSE-APACHE` are kept in `vendor/tract-tflite/`. The only
changed file is `src/ops/array.rs`, which the snapshot records with its original and modified
hashes. tract is dual-licensed; HeadtrackRS uses it under the MIT option.

## 5. Tobii USB device layer: provenance statement

`crates/headtrack-tobii-device` reads the Tobii Eye Tracker 5 (USB `2104:0313`) directly over
USB (`nusb`). It is a port of TobiiLab's `crates/tobii-device` (2026-10-03). Provenance, from
TobiiLab `docs/PROVENANCE.md` (2026-09-07):

- The wire encoder and parser are an **independent implementation, written from fresh local
  observations** of message layouts and stream behaviour. The startup sequence differs from the
  community reference: direct realm-0 subscriptions, without its open-realm request.
- **Not copied or translated:** the njmill/tobii-linux C reader; the GPL-3.0 Aetherall/tobiifree
  implementation; the unlicensed simonvc/tobii_ffg initialization captures; any authentication
  key, captured display identity or initialization dump.
- No Tobii firmware, SDK, Stream Engine library or vendor DLL is used or shipped. The
  community executable that was observed is neither invoked nor shipped.
- This is source-informed engineering. It is **not** a formal clean-room claim and **not** legal
  clearance. TobiiLab's own words: "engineering evidence, not legal clearance or a formal
  clean-room claim."

"Tobii" is a trademark of Tobii AB. HeadtrackRS is not affiliated with or endorsed by Tobii AB.

## 6. ONNX Runtime (statically linked)

The webcam models run in ONNX Runtime (© Microsoft Corporation, MIT), linked statically into
`headtrackd` and `headtrack-gui` through the `ort` crate (pyke; MIT OR Apache-2.0). The linked
build is ONNX Runtime **1.24.2** (CPU; `ort-sys` 2.0.0-rc.12 `dist.txt`). No `libonnxruntime.so`
ships. ONNX Runtime's own third-party notices, covering the libraries compiled into it, ship as
`ONNXRUNTIME-1.24.2-ThirdPartyNotices.txt` (from the v1.24.2 tag).

## 7. Other bundled material

- `NPClient64.dll` (Star Citizen bridge) is built from `crates/headtrack-npclient` (MIT), using
  `winapi` (MIT OR Apache-2.0). It reimplements the NPClient interface and contains no NaturalPoint
  code. See §7a for the parts it ports from opentrack.

### 7a. opentrack (NPClient protocol material in `NPClient64.dll`)

`crates/headtrack-npclient` ports these pieces from [opentrack](https://github.com/opentrack/opentrack)'s
NPClient implementation, as its source comments say:
- the TIR5 checksum (`tir_cksum`);
- the XOR-encoded signature tables returned by `NP_GetSignature`;
- the `tir_data_t` struct layout;
- Star Citizen's game id and encryption key, from opentrack's supported-games list (`games.csv`).

opentrack is distributed under the ISC licence. The notices below are reproduced verbatim from
opentrack's `OPENTRACK-LICENSING.txt` ("core, various modules") and
`3rdparty-notices/FACETRACKNOIR-COPYING.txt` (opentrack is a fork of FaceTrackNoIR, the origin of
the supported-games list), fetched 2026-11-14:

```text
Copyright (c) 2012-2016 Stanislaw Halik

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.
```

```text
The opentrack project is a fork of "FaceTrackNoIR" software.

Original project's site located at <http://facetracknoir.sourceforge.net/home/default.htm>

Copyright (c) 2015, Wim Vriend

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.
```

opentrack's warranty disclaimer (`OPENTRACK-LICENSING.txt`) applies:

```text
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### 7b. Other assets

- `assets/icons/github-mark-white.png` is the GitHub mark, a trademark of GitHub, Inc., used only to link to the project's repository.
- `udev/70-headtrackrs-tobii.rules`: HeadtrackRS's own, MIT.

## 8. Rust dependencies

These crates are compiled into the shipped binaries (`headtrack-gui`, `headtrackd`,
`headtrack-xplane`, and `NPClient64.dll`). Licences are as each crate declares them;
"A OR B" means the crate may be used under either, and HeadtrackRS uses the MIT option
where one is offered. Regenerate with:

```
cargo tree --offline -p headtrack-gui -p headtrack-daemon -p headtrack-xplane -e normal --prefix none -f '{p}|{l}'
cargo tree --offline --manifest-path crates/headtrack-npclient/Cargo.toml --target x86_64-pc-windows-gnu -e normal --prefix none -f '{p}|{l}'
```

Notices that need more than a list entry:
- **IJG** (`mozjpeg`, `mozjpeg-sys`): this software is based in part on the work of the Independent JPEG Group.
- **MPL-2.0** (`dyn-eq`, used by tract): file-level copyleft. The crate's source is unmodified and available at crates.io/crates/dyn-eq.
- **Unicode-3.0** (`icu_*`, `zerovec`, `yoke`, …): Unicode data files and software. © 1991-2025 Unicode, Inc.
- **BSD-2/3-Clause, ISC, Zlib, BSL-1.0, 0BSD, BlueOak**: their notices are satisfied by this file together with each crate's licence text, which lives in its published source.


341 third-party crates, grouped by declared licence:

### MIT OR Apache-2.0 (146)

`accesskit v0.17.1`, `accesskit_atspi_common v0.10.1`, `accesskit_consumer v0.26.0`, `accesskit_unix v0.13.1`, `ahash v0.8.12`, `anyhow v1.0.102`, `arboard v3.6.1`, `arrayvec v0.7.6`, `as-raw-xcb-connection v1.0.1`, `async-broadcast v0.7.2`, `async-recursion v1.1.1`, `async-trait v0.1.89`, `bitflags v1.3.2`, `bitflags v2.11.0`, `block-buffer v0.10.4`, `cfg-if v1.0.4`, `cpufeatures v0.2.17`, `crc32fast v1.5.0`, `crossbeam-utils v0.8.21`, `crypto-common v0.1.7`, `ctrlc v3.5.2`, `digest v0.10.7`, `displaydoc v0.2.5`, `document-features v0.2.12`, `downcast-rs v1.2.1`, `downcast-rs v2.0.2`, `dyn-clone v1.0.20`, `dyn-hash v1.0.0`, `ecolor v0.31.1`, `eframe v0.31.1`, `egui v0.31.1`, `egui-phosphor v0.9.0`, `egui-winit v0.31.1`, `egui_glow v0.31.1`, `either v1.15.0`, `emath v0.31.1`, `enumflags2 v0.7.12`, `enumflags2_derive v0.7.12`, `epaint v0.31.1`, `erased-serde v0.4.10`, `errno v0.3.14`, `fdeflate v0.3.7`, `flate2 v1.1.9`, `form_urlencoded v1.2.2`, `futures-channel v0.3.32`, `futures-core v0.3.32`, `futures-io v0.3.32`, `futures-macro v0.3.32`, `futures-sink v0.3.32`, `futures-task v0.3.32`, `futures-util v0.3.32`, `getrandom v0.2.17`, `getrandom v0.3.4`, `half v2.7.1`, `hashbrown v0.15.5`, `hashbrown v0.16.1`, `hex v0.4.3`, `idna v1.1.0`, `image v0.25.10`, `inventory v0.3.24`, `itertools v0.15.0`, `itoa v1.0.17`, `lazy_static v1.5.0`, `libc v0.2.183`, `litrs v1.0.0`, `lock_api v0.4.14`, `log v0.4.29`, `maplit v1.0.2`, `matrixmultiply v0.3.10`, `memmap2 v0.9.10`, `ndarray v0.17.2`, `num-complex v0.4.6`, `num-integer v0.1.46`, `num-rational v0.4.2`, `num-traits v0.2.19`, `once_cell v1.21.4`, `ordered-stream v0.2.0`, `ort v2.0.0-rc.12`, `ort-sys v2.0.0-rc.12`, `parking_lot v0.12.5`, `parking_lot_core v0.9.12`, `paste v1.0.15`, `pastey v0.2.1`, `percent-encoding v2.3.2`, `piper v0.2.5`, `png v0.18.1`, `ppv-lite86 v0.2.21`, `primal-check v0.3.4`, `proc-macro-crate v3.5.0`, `proc-macro2 v1.0.106`, `profiling v1.0.17`, `quote v1.0.45`, `rand v0.8.5`, `rand_chacha v0.3.1`, `rand_core v0.6.4`, `rawpointer v0.2.1`, `regex v1.12.3`, `regex-automata v0.4.14`, `regex-syntax v0.8.10`, `rustfft v6.4.1`, `scoped-tls v1.0.1`, `scopeguard v1.2.0`, `serde v1.0.228`, `serde_core v1.0.228`, `serde_derive v1.0.228`, `serde_json v1.0.149`, `serde_repr v0.1.20`, `serde_spanned v0.6.9`, `sha1 v0.10.6`, `sha2 v0.10.9`, `signal-hook-registry v1.4.8`, `smallvec v1.15.1`, `smol_str v0.2.2`, `socket2 v0.6.3`, `stable_deref_trait v1.2.1`, `static_assertions v1.1.0`, `strength_reduce v0.2.4`, `string-interner v0.20.0`, `syn v2.0.117`, `thiserror v1.0.69`, `thiserror v2.0.18`, `thiserror-impl v1.0.69`, `thiserror-impl v2.0.18`, `thread_local v1.1.9`, `toml v0.8.23`, `toml_datetime v0.6.11`, `toml_datetime v1.0.0+spec-1.1.0`, `toml_edit v0.22.27`, `toml_edit v0.25.4+spec-1.1.0`, `toml_parser v1.0.9+spec-1.1.0`, `toml_write v0.1.2`, `tract-core v0.23.6`, `tract-data v0.23.6`, `tract-linalg v0.23.6`, `tract-tflite v0.23.6`, `transpose v0.2.3`, `ttf-parser v0.25.1`, `typeid v1.0.3`, `typenum v1.19.0`, `url v2.5.8`, `web-time v1.1.0`, `webbrowser v1.2.0`, `winapi v0.3.9`, `winapi-x86_64-pc-windows-gnu v0.4.0`, `x11rb v0.13.2`, `x11rb-protocol v0.13.2`

### MIT (82)

`ashpd v0.9.3`, `bincode v1.3.3`, `bitvec v1.1.1`, `bytes v1.11.1`, `calloop v0.13.0`, `calloop v0.14.4`, `calloop-wayland-source v0.3.0`, `calloop-wayland-source v0.4.1`, `derive-new v0.7.0`, `dlib v0.5.3`, `endi v1.1.1`, `funty v2.0.0`, `generic-array v0.14.7`, `glutin-winit v0.5.0`, `libm v0.2.16`, `matchers v0.2.0`, `memoffset v0.9.1`, `mio v1.1.1`, `nix v0.29.0`, `nix v0.31.2`, `nom v8.0.0`, `nom-language v0.1.0`, `nu-ansi-term v0.50.3`, `quick-xml v0.30.0`, `quick-xml v0.39.2`, `radium v0.7.0`, `rfd v0.17.2`, `rgb v0.8.53`, `scan_fmt v0.2.6`, `sctk-adwaita v0.10.1`, `sharded-slab v0.1.7`, `simd-adler32 v0.3.8`, `slab v0.4.12`, `smithay-client-toolkit v0.19.2`, `smithay-client-toolkit v0.20.0`, `smithay-clipboard v0.7.3`, `spin v0.9.8`, `strict-num v0.1.1`, `synstructure v0.13.2`, `tap v1.0.1`, `tokio v1.50.0`, `tokio-macros v2.6.1`, `tracing v0.1.44`, `tracing-attributes v0.1.31`, `tracing-core v0.1.36`, `tracing-log v0.2.0`, `tracing-subscriber v0.3.23`, `v4l v0.14.0`, `v4l2-sys-mit v0.3.0`, `wayland-backend v0.3.14`, `wayland-client v0.31.13`, `wayland-csd-frame v0.3.0`, `wayland-cursor v0.31.13`, `wayland-protocols v0.32.11`, `wayland-protocols-experimental v20250721.0.1`, `wayland-protocols-misc v0.3.11`, `wayland-protocols-plasma v0.3.11`, `wayland-protocols-wlr v0.3.11`, `wayland-scanner v0.31.9`, `wayland-sys v0.31.10`, `winnow v0.7.15`, `wyz v0.5.1`, `x11-dl v2.21.0`, `xcursor v0.3.10`, `xdg-home v1.3.0`, `xkbcommon-dl v0.4.2`, `zbus v4.4.0`, `zbus v5.14.0`, `zbus-lockstep v0.4.4`, `zbus-lockstep-macros v0.4.4`, `zbus_macros v4.4.0`, `zbus_macros v5.14.0`, `zbus_names v3.0.0`, `zbus_names v4.3.1`, `zbus_xml v4.0.0`, `zmij v1.0.21`, `zvariant v4.2.0`, `zvariant v5.10.0`, `zvariant_derive v4.2.0`, `zvariant_derive v5.10.0`, `zvariant_utils v2.1.0`, `zvariant_utils v3.3.0`

### Apache-2.0 OR MIT (34)

`async-channel v2.5.0`, `async-executor v1.14.0`, `async-fs v2.2.0`, `async-io v2.6.0`, `async-lock v3.4.2`, `async-net v2.0.0`, `async-task v4.7.1`, `atomic-waker v1.1.2`, `atspi v0.22.0`, `atspi-common v0.6.0`, `atspi-connection v0.6.0`, `atspi-proxies v0.6.0`, `bit-set v0.11.1`, `bit-vec v0.10.1`, `blocking v1.6.2`, `concurrent-queue v2.5.0`, `equivalent v1.0.2`, `evdev v0.13.2`, `event-listener v5.4.1`, `event-listener-strategy v0.5.4`, `fastrand v2.3.0`, `flume v0.11.1`, `futures-lite v2.6.1`, `idna_adapter v1.2.1`, `immutable-chunkmap v2.1.2`, `indexmap v2.13.0`, `nohash-hasher v0.2.0`, `nusb v0.2.7`, `parking v2.2.1`, `pin-project-lite v0.2.17`, `polling v3.11.0`, `pollster v0.4.0`, `utf8_iter v1.0.4`, `uuid v1.23.0`

### Apache-2.0 (19)

`ab_glyph v0.2.32`, `ab_glyph_rasterizer v0.1.10`, `accesskit_winit v0.23.1`, `approx v0.5.1`, `flatbuffers v25.12.19`, `gethostname v1.1.0`, `glutin v0.32.3`, `glutin_egl_sys v0.7.1`, `glutin_glx_sys v0.6.1`, `nalgebra v0.34.2`, `nalgebra-macros v0.3.0`, `nokhwa v0.10.10`, `nokhwa-bindings-linux v0.1.3`, `nokhwa-bindings-macos v0.2.3`, `nokhwa-bindings-windows v0.4.5`, `nokhwa-core v0.1.8`, `owned_ttf_parser v0.25.1`, `simba v0.9.1`, `winit v0.30.13`

### Unicode-3.0 (18)

`icu_collections v2.1.1`, `icu_locale_core v2.1.1`, `icu_normalizer v2.1.1`, `icu_normalizer_data v2.1.1`, `icu_properties v2.1.2`, `icu_properties_data v2.1.2`, `icu_provider v2.1.1`, `litemap v0.8.1`, `potential_utf v0.1.4`, `tinystr v0.8.2`, `writeable v0.6.2`, `yoke v0.8.1`, `yoke-derive v0.8.1`, `zerofrom v0.1.6`, `zerofrom-derive v0.1.6`, `zerotrie v0.2.3`, `zerovec v0.11.5`, `zerovec-derive v0.11.2`

### MIT OR Apache-2.0 OR Zlib (6)

`cursor-icon v1.2.0`, `glow v0.16.0`, `raw-window-handle v0.6.2`, `xkeysym v0.2.1`, `zune-core v0.5.1`, `zune-jpeg v0.5.13`

### Unlicense OR MIT (6)

`aho-corasick v1.1.4`, `byteorder v1.5.0`, `byteorder-lite v0.1.0`, `csv v1.4.0`, `csv-core v0.1.13`, `memchr v2.8.0`

### Apache-2.0 WITH LLVM-exception OR Apache-2.0 OR MIT (4)

`linux-raw-sys v0.12.1`, `linux-raw-sys v0.4.15`, `rustix v0.38.44`, `rustix v1.1.4`

### Zlib OR Apache-2.0 OR MIT (4)

`bytemuck v1.25.0`, `bytemuck_derive v1.10.2`, `safe_arch v0.7.4`, `wide v0.7.33`

### Zlib (3)

`foldhash v0.1.5`, `foldhash v0.2.0`, `nanorand v0.7.0`

### BSD-2-Clause OR Apache-2.0 OR MIT (2)

`zerocopy v0.8.42`, `zerocopy-derive v0.8.42`

### BSD-3-Clause (2)

`tiny-skia v0.11.4`, `tiny-skia-path v0.11.4`

### BSD-3-Clause OR Apache-2.0 (2)

`moxcms v0.8.1`, `pxfm v0.1.28`

### (MIT OR Apache-2.0) AND OFL-1.1 AND Ubuntu-font-1.0 (1)

`epaint_default_fonts v0.31.1`

### (MIT OR Apache-2.0) AND Unicode-3.0 (1)

`unicode-ident v1.0.24`

### 0BSD OR MIT OR Apache-2.0 (1)

`adler2 v2.0.1`

### Apache-2.0 AND MIT (1)

`dpi v0.1.2`

### Apache-2.0 OR BSL-1.0 (1)

`ryu v1.0.23`

### BSD-2-Clause (1)

`arrayref v0.3.9`

### BlueOak-1.0.0 OR MIT OR Apache-2.0 (1)

`anymap3 v1.1.0`

### IJG (1)

`mozjpeg v0.10.13`

### IJG AND Zlib AND BSD-3-Clause (1)

`mozjpeg-sys v2.2.3`

### ISC (1)

`libloading v0.8.9`

### MIT OR Zlib OR Apache-2.0 (1)

`miniz_oxide v0.8.9`

### MPL-2.0 (1)

`dyn-eq v0.1.3`

### Unlicense (1)

`ksni v0.3.3`
