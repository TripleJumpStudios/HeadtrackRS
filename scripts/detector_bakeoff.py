#!/usr/bin/env python3
"""Detector bake-off — can a permissively-licensed detector replace SCRFD?

SCRFD (InsightFace) is licensed for non-commercial academic research only, which
conflicts with shipping it in an MIT project. This compares candidates against it
on *identical frames*, captured by `scripts/capture-frames.sh`.

    scripts/detector_bakeoff.py --validate          # trust the SCRFD port first
    scripts/detector_bakeoff.py --detect            # run all detectors over all clips
    scripts/detector_bakeoff.py --summary           # the decision metrics

Why the SCRFD port has to be validated: the reference track has to be the *same*
detector the product actually runs, or every comparison is against a strawman. The
capture logs the Rust tracker's own SCRFD detections on the frames where it ran
them (~47 across the set), so `--validate` checks this Python decode against those
rows. If it does not agree to sub-pixel, nothing downstream is trustworthy.

Reads:  <dir>/<clip>/frames.csv + frame_NNNNNN.jpg
Writes: <dir>/<clip>/detections.csv
"""
import argparse
import csv
import glob
import os
import sys
import time

import numpy as np

# ── Model locations ─────────────────────────────────────────────────────────
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRFD_MODEL = os.path.join(REPO, "assets", "models", "scrfd-det-10g.onnx")
YUNET_MODEL = os.path.expanduser("~/headtrack-bakeoff/models/face_detection_yunet_2023mar.onnx")
BLAZE_MODEL = os.path.expanduser("~/headtrack-bakeoff/models/blaze_face_short_range.tflite")
SIXDREP_MODEL = os.path.join(REPO, "assets", "models", "head-pose-mobilenetv3-large.onnx")

# ── SCRFD constants — MUST mirror crates/headtrack-input-neuralnet/src/scrfd_detector.rs ──
SCRFD_INPUT = 320
STRIDES = (8, 16, 32)
NUM_ANCHORS = 2
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4
PIX_MEAN = 127.5
PIX_STD = 128.0
KPS = 5


def _anchor_centers(input_size, stride):
    """Anchor centres, anchor-major flattened — mirrors `build_anchor_centers`."""
    n = input_size // stride
    ys, xs = np.mgrid[0:n, 0:n]
    c = np.stack([xs.ravel() * stride, ys.ravel() * stride], axis=1).astype(np.float32)
    return np.repeat(c, NUM_ANCHORS, axis=0)


def _nms(boxes, scores, thresh):
    """NMS matching the Rust `nms()`, including its +1.0 in the area terms."""
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx0 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy0 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx1 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy1 = np.minimum(boxes[i, 3], boxes[rest, 3])
        iw = np.maximum(xx1 - xx0 + 1.0, 0.0)
        ih = np.maximum(yy1 - yy0 + 1.0, 0.0)
        inter = iw * ih
        area_i = (boxes[i, 2] - boxes[i, 0] + 1.0) * (boxes[i, 3] - boxes[i, 1] + 1.0)
        area_r = (boxes[rest, 2] - boxes[rest, 0] + 1.0) * (boxes[rest, 3] - boxes[rest, 1] + 1.0)
        iou = inter / (area_i + area_r - inter)
        order = rest[iou <= thresh]
    return keep


class Scrfd:
    """Faithful port of the production Rust decode. The reference track."""

    name = "scrfd"

    def __init__(self, model=SCRFD_MODEL):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self.sess = ort.InferenceSession(model, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.outs = [o.name for o in self.sess.get_outputs()]
        self.centers = [_anchor_centers(SCRFD_INPUT, s) for s in STRIDES]

    def detect(self, bgr):
        import cv2
        h, w = bgr.shape[:2]
        im_ratio = h / w
        # Letterbox into a square canvas, preserving aspect ratio.
        if im_ratio > 1.0:
            nh, nw = SCRFD_INPUT, int(round(SCRFD_INPUT / im_ratio))
        else:
            nw, nh = SCRFD_INPUT, int(round(SCRFD_INPUT * im_ratio))
        det_scale = nh / h

        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32)

        # Padding stays at 0.0 in *normalised* space, exactly as the Rust does
        # (it allocates a zeroed buffer and writes only the resized region).
        # That is equivalent to mid-grey, NOT to black — getting this wrong
        # shifts scores near the frame edge.
        blob = np.zeros((1, 3, SCRFD_INPUT, SCRFD_INPUT), dtype=np.float32)
        blob[0, :, :nh, :nw] = ((rgb - PIX_MEAN) / PIX_STD).transpose(2, 0, 1)

        o = self.sess.run(self.outs, {self.inp: blob})
        if len(o) != 9:
            raise RuntimeError(f"SCRFD: expected 9 outputs, got {len(o)}")

        boxes, scores, kpss = [], [], []
        for idx, stride in enumerate(STRIDES):
            c = self.centers[idx]
            sc = o[idx].reshape(-1)
            bd = o[idx + 3].reshape(-1, 4) * stride
            kd = o[idx + 6].reshape(-1, 10) * stride
            sel = sc >= CONF_THRESHOLD
            if not sel.any():
                continue
            cc = c[sel]
            b = np.stack([cc[:, 0] - bd[sel][:, 0], cc[:, 1] - bd[sel][:, 1],
                          cc[:, 0] + bd[sel][:, 2], cc[:, 1] + bd[sel][:, 3]], axis=1)
            k = kd[sel].reshape(-1, KPS, 2) + cc[:, None, :]
            boxes.append(b); scores.append(sc[sel]); kpss.append(k)

        if not boxes:
            return None
        boxes = np.concatenate(boxes); scores = np.concatenate(scores); kpss = np.concatenate(kpss)
        keep = _nms(boxes, scores, IOU_THRESHOLD)
        i = keep[0]
        inv = 1.0 / det_scale
        b = boxes[i] * inv
        k = kpss[i] * inv
        return dict(score=float(scores[i]),
                    bbox=(float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])),
                    kps=[(float(x), float(y)) for x, y in k])


class YuNet:
    """OpenCV Zoo YuNet — MIT, and natively emits our 5 keypoints in our order."""

    def __init__(self, model=YUNET_MODEL, input_size=SCRFD_INPUT, score=0.5, nms=0.3, top_k=500):
        import cv2
        self.name = f"yunet{input_size}"
        self.input_size = input_size
        self.det = cv2.FaceDetectorYN.create(model, "", (input_size, input_size),
                                             score, nms, top_k)

    def detect(self, bgr):
        import cv2
        h, w = bgr.shape[:2]
        # YuNet has no letterbox: it wants the frame scaled to its input size.
        # Scale isotropically to preserve face geometry, then pad, so keypoint
        # geometry stays comparable with SCRFD's.
        s = min(self.input_size / w, self.input_size / h)
        nw, nh = int(round(w * s)), int(round(h * s))
        canvas = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        self.det.setInputSize((self.input_size, self.input_size))
        _, faces = self.det.detect(canvas)
        if faces is None or len(faces) == 0:
            return None
        f = max(faces, key=lambda r: r[-1])
        inv = 1.0 / s
        # Layout: x, y, w, h, then 5 (x,y) pairs, then score.
        # YuNet's keypoint order is right eye, left eye, nose, right mouth,
        # left mouth — identical to SCRFD, so no remapping.
        return dict(score=float(f[-1]),
                    bbox=(float(f[0] * inv), float(f[1] * inv),
                          float(f[2] * inv), float(f[3] * inv)),
                    kps=[(float(f[4 + 2 * i] * inv), float(f[5 + 2 * i] * inv)) for i in range(KPS)])


class MediaPipe:
    """BlazeFace via the mediapipe Tasks API (1.0 dropped the legacy `solutions`).

    Only the **short-range** bundle is published for this API, and its own model
    card describes it as tuned for "selfie-like images from a smartphone camera
    or webcam" at close range. That is the most frontal-biased candidate here by
    design, so a poor showing at high yaw is a property of the model, not a
    setup error — worth stating plainly before reading its numbers.
    """

    name = "mediapipe"

    def __init__(self, model=BLAZE_MODEL, min_conf=0.5):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision
        self.mp = mp
        opts = vision.FaceDetectorOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            min_detection_confidence=min_conf,
        )
        self.det = vision.FaceDetector.create_from_options(opts)

    def detect(self, bgr):
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        res = self.det.detect(img)
        if not res.detections:
            return None
        d = max(res.detections, key=lambda x: x.categories[0].score)
        bb = d.bounding_box
        kp = d.keypoints
        if len(kp) < 4:
            return None
        # BlazeFace order: right eye, left eye, nose tip, mouth centre,
        # right ear, left ear. It has no mouth CORNERS, which SCRFD and YuNet
        # both provide. Index 3 (mouth centre) is duplicated into both corner
        # slots and flagged, rather than silently fabricating two points — the
        # PnP solve weights mouth corners, so a fake pair would quietly corrupt
        # any downstream comparison that used them.
        w, h = bgr.shape[1], bgr.shape[0]
        pts = [(kp[i].x * w, kp[i].y * h) for i in (0, 1, 2, 3, 3)]
        return dict(score=float(d.categories[0].score),
                    bbox=(float(bb.origin_x), float(bb.origin_y),
                          float(bb.width), float(bb.height)),
                    kps=pts,
                    mouth_is_synthetic=True)


# ── Data loading ────────────────────────────────────────────────────────────

def load_clip(clip_dir):
    """Yield (row, image_path) per frame, in frame order."""
    csv_path = os.path.join(clip_dir, "frames.csv")
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            idx = int(row["frame_idx"])
            img = os.path.join(clip_dir, f"frame_{idx:06d}.jpg")
            yield row, img


def rust_kps(row):
    """The Rust tracker's own detector keypoints for this frame, if it ran one."""
    if row["scrfd_ran"].strip() != "1" or row["source"].strip() != "scrfd":
        return None
    pts = []
    for i in range(KPS):
        x, y = row[f"kp{i}_x"].strip(), row[f"kp{i}_y"].strip()
        if not x or not y:
            return None
        pts.append((float(x), float(y)))
    return pts


# ── Modes ───────────────────────────────────────────────────────────────────

def cmd_validate(clips):
    """Check the Python SCRFD port against the Rust tracker's logged detections."""
    import cv2
    scrfd = Scrfd()
    deltas, missing, checked = [], 0, 0
    for clip_dir in clips:
        for row, img_path in load_clip(clip_dir):
            ref = rust_kps(row)
            if ref is None:
                continue
            img = cv2.imread(img_path)
            if img is None:
                continue
            got = scrfd.detect(img)
            checked += 1
            if got is None:
                missing += 1
                continue
            d = np.linalg.norm(np.array(got["kps"]) - np.array(ref), axis=1)
            deltas.append(d)

    if not checked:
        print("No Rust SCRFD rows found to validate against.", file=sys.stderr)
        return 1
    print(f"Validation: {checked} frames where the Rust tracker ran SCRFD")
    if missing:
        print(f"  !! Python port found no face on {missing} of them")
    if deltas:
        d = np.concatenate(deltas)
        print(f"  keypoint delta px: median {np.median(d):.3f}  p95 {np.percentile(d,95):.3f}  max {d.max():.3f}")
        ok = np.median(d) < 1.0 and d.max() < 5.0
        print(f"  => {'PASS — the port matches production' if ok else 'FAIL — port diverges, do not trust downstream numbers'}")
        return 0 if ok else 2
    return 2


def build_detectors(which):
    """Construct the requested detectors, skipping any that fail to initialise.

    A candidate that cannot even load is a result in itself, and must not stop
    the SCRFD-vs-YuNet comparison that the decision actually rests on.
    """
    factory = {
        "scrfd":     Scrfd,
        "yunet":     lambda: YuNet(input_size=320),
        "yunet640":  lambda: YuNet(input_size=640),
        "mediapipe": MediaPipe,
    }
    dets = []
    for w in which:
        if w not in factory:
            raise SystemExit(f"unknown detector {w}")
        try:
            dets.append(factory[w]())
        except Exception as e:
            print(f"  !! {w} unavailable, skipping: {e}", file=sys.stderr)
    if not dets:
        raise SystemExit("no detectors could be constructed")
    return dets


def cmd_detect(clips, which):
    import cv2
    dets = build_detectors(which)
    for clip_dir in clips:
        name = os.path.basename(clip_dir.rstrip("/"))
        out = os.path.join(clip_dir, "detections.csv")
        rows = list(load_clip(clip_dir))
        t0 = time.time()
        with open(out, "w", newline="") as fh:
            cols = ["frame_idx", "yaw", "pitch", "roll"]
            for d in dets:
                n = d.name
                cols += [f"{n}_found", f"{n}_score", f"{n}_bx", f"{n}_by", f"{n}_bw", f"{n}_bh"]
                cols += [f"{n}_kp{i}_{a}" for i in range(KPS) for a in ("x", "y")]
                cols += [f"{n}_ms"]
            wri = csv.writer(fh)
            wri.writerow(cols)
            last_pose = ("", "", "")
            for row, img_path in rows:
                img = cv2.imread(img_path)
                if img is None:
                    continue
                y, p, r = row["yaw"].strip(), row["pitch"].strip(), row["roll"].strip()
                if y:
                    last_pose = (y, p, r)
                rec = [row["frame_idx"], *last_pose]
                for d in dets:
                    t = time.perf_counter()
                    res = d.detect(img)
                    ms = (time.perf_counter() - t) * 1000.0
                    if res is None:
                        rec += [0, "", "", "", "", ""] + [""] * (KPS * 2) + [f"{ms:.2f}"]
                    else:
                        b = res["bbox"]
                        rec += [1, f"{res['score']:.4f}", f"{b[0]:.2f}", f"{b[1]:.2f}",
                                f"{b[2]:.2f}", f"{b[3]:.2f}"]
                        for (kx, ky) in res["kps"]:
                            rec += [f"{kx:.3f}", f"{ky:.3f}"]
                        rec += [f"{ms:.2f}"]
                wri.writerow(rec)
        print(f"{name:<15} {len(rows):>5} frames -> detections.csv  ({time.time()-t0:.0f}s)")



# ── Metrics ─────────────────────────────────────────────────────────────────

# Mirrors crates/headtrack-input-neuralnet/src/landmark_position.rs. Kept in
# sync by hand; `position_from_eyes` is 40 lines of arithmetic and porting it
# is cheaper than plumbing a Rust binary into the harness. The clamp below is
# YAW_COS_FLOOR — beyond ~60 degrees the far eye occludes and projected IPD
# stops meaning anything, so the correction stops rather than exploding.
IPD_MM = 63.0
YAW_COS_FLOOR = 0.5
FOV_DIAG_DEG = 84.0   # this camera; see ~/.config/headtrack-rs/camera.toml


def position_from_eyes(right_eye, left_eye, img_w, img_h, fov_diag_deg, yaw_deg):
    fov_diag = np.radians(fov_diag_deg)
    aspect = img_h / img_w
    fov_w = 2.0 * np.arctan(np.tan(fov_diag / 2.0) / np.sqrt(1.0 + aspect * aspect))
    fov_h = 2.0 * np.arctan(np.tan(fov_diag / 2.0) / np.sqrt(1.0 + 1.0 / (aspect * aspect)))
    fl_w = (img_w * 0.5) / np.tan(fov_w * 0.5)
    fl_h = (img_h * 0.5) / np.tan(fov_h * 0.5)
    dx = left_eye[0] - right_eye[0]
    dy = left_eye[1] - right_eye[1]
    ipd_px = max(np.hypot(dx, dy), 1.0)
    yaw_cos = max(abs(np.cos(np.radians(yaw_deg))), YAW_COS_FLOOR)
    z = fl_w * IPD_MM / (ipd_px / yaw_cos)
    cx = (right_eye[0] + left_eye[0]) * 0.5
    cy = (right_eye[1] + left_eye[1]) * 0.5
    x = ((cx - img_w * 0.5) / fl_w) * z
    y = ((cy - img_h * 0.5) / fl_h) * z
    return x, -y, -z, ipd_px


def read_detections(clip_dir):
    p = os.path.join(clip_dir, "detections.csv")
    if not os.path.exists(p):
        return None, None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    names = sorted({c.rsplit("_found", 1)[0] for c in rows[0] if c.endswith("_found")})
    return rows, names


def _f(v):
    v = (v or "").strip()
    return float(v) if v else None


def cmd_summary(clips, img_w=960, img_h=540):
    all_rows, names = [], None
    per_clip = {}
    for d in clips:
        rows, n = read_detections(d)
        if rows is None:
            continue
        names = names or n
        per_clip[os.path.basename(d)] = rows
        all_rows += rows
    if not all_rows:
        raise SystemExit("no detections.csv found — run --detect first")

    ref = "scrfd"
    cands = [n for n in names if n != ref]

    # ── 1. Recall vs yaw ────────────────────────────────────────────────
    print("=" * 78)
    print("1. RECALL vs |yaw|  — the hard gate")
    print("=" * 78)
    yaw = np.array([abs(_f(r["yaw"]) or 0.0) for r in all_rows])
    found = {n: np.array([r[f"{n}_found"] == "1" for r in all_rows]) for n in names}
    hdr = f"  {'bin':>8} {'n':>6}  " + "".join(f"{n:>12}" for n in names)
    print(hdr)
    for lo in range(0, 90, 10):
        sel = (yaw >= lo) & (yaw < lo + 10)
        n = int(sel.sum())
        if n == 0:
            continue
        cells = "".join(f"{100*found[d][sel].mean():>11.1f}%" for d in names)
        print(f"  {lo:>3}-{lo+10:<4} {n:>6}  {cells}")
    print(f"  {'ALL':>8} {len(yaw):>6}  " + "".join(f"{100*found[d].mean():>11.1f}%" for d in names))

    print()
    for c in cands:
        for label, m in (("<=50 deg (pass/fail)", yaw <= 50), ("50-90 deg (characterise)", yaw > 50)):
            if m.sum() == 0:
                continue
            gap = 100 * (found[ref][m].mean() - found[c][m].mean())
            verdict = "PASS" if gap <= 2.0 else "FAIL"
            note = verdict if "pass/fail" in label else f"{'ok' if gap <= 2.0 else 'degraded'}"
            print(f"  {c:<10} {label:<26} gap vs SCRFD {gap:>6.2f} pts   {note}")

    # ── 2 & 3. IPD stability and yaw-dependent drift ────────────────────
    print()
    print("=" * 78)
    print("2. IPD / Z NOISE  — variance, not offset (a constant offset is absorbed by recentre)")
    print("=" * 78)
    for clip in ("still", "z", "yaw-hold-45", "yaw-hold-max"):
        rows = per_clip.get(clip)
        if not rows:
            continue
        print(f"  {clip}:")
        for n in names:
            ipds, zs = [], []
            for r in rows:
                if r[f"{n}_found"] != "1":
                    continue
                re_ = (_f(r[f"{n}_kp0_x"]), _f(r[f"{n}_kp0_y"]))
                le_ = (_f(r[f"{n}_kp1_x"]), _f(r[f"{n}_kp1_y"]))
                if None in re_ or None in le_:
                    continue
                yv = _f(r["yaw"]) or 0.0
                _, _, z, ipd = position_from_eyes(re_, le_, img_w, img_h, FOV_DIAG_DEG, yv)
                ipds.append(ipd); zs.append(z)
            if len(ipds) < 10:
                print(f"    {n:<12} too few detections ({len(ipds)})")
                continue
            ipds = np.array(ipds); zs = np.array(zs)
            print(f"    {n:<12} IPD {np.mean(ipds):6.2f}px  std {np.std(ipds):5.2f}px"
                  f"   Z {np.mean(zs):8.1f}mm  std {np.std(zs):6.1f}mm  n={len(ipds)}")

    # ── 5. Re-anchor jump ───────────────────────────────────────────────
    print()
    print("=" * 78)
    print("5. RE-ANCHOR SPREAD — how much a re-detect DISTURBS the pose")
    print("=" * 78)
    print("  Measured against prev_kps: the optical-flow-carried points a re-detect")
    print("  would replace. Those points are SCRFD's own lineage, so raw distance")
    print("  flatters SCRFD and penalises any candidate that merely places a keypoint")
    print("  somewhere consistently different. A CONSTANT offset is harmless — the")
    print("  centre stage absorbs it, and once a candidate seeds the flow itself the")
    print("  offset disappears. So the fair number is the SPREAD after removing each")
    print("  detector's own median offset. Raw is shown only for context.")
    print(f"  {'clip':<15} " + "".join(f"{n:>22}" for n in names))
    print(f"  {'':<15} " + "".join(f"{'raw / debiased':>22}" for _ in names))
    for clip_dir in clips:
        clip = os.path.basename(clip_dir)
        rows = per_clip.get(clip)
        if not rows:
            continue
        prev = {}
        with open(os.path.join(clip_dir, "frames.csv")) as f:
            for fr in csv.DictReader(f):
                pts = []
                for i in range(KPS):
                    x, y = fr[f"prev_kp{i}_x"].strip(), fr[f"prev_kp{i}_y"].strip()
                    if x and y:
                        pts.append((float(x), float(y)))
                if len(pts) == KPS:
                    prev[fr["frame_idx"]] = np.array(pts)
        cells = []
        for n in names:
            deltas = []   # per-frame, per-keypoint offset vectors
            for r in rows:
                pv = prev.get(r["frame_idx"])
                if pv is None or r[f"{n}_found"] != "1":
                    continue
                k = [(_f(r[f"{n}_kp{i}_x"]), _f(r[f"{n}_kp{i}_y"])) for i in range(KPS)]
                if any(a is None or b is None for a, b in k):
                    continue
                deltas.append(np.array(k) - pv)
            if len(deltas) < 10:
                cells.append(f"{'--':>22}")
                continue
            d = np.stack(deltas)                       # [frames, KPS, 2]
            raw = np.median(np.linalg.norm(d, axis=2))
            bias = np.median(d, axis=0, keepdims=True)  # this detector's own placement offset
            deb = np.median(np.linalg.norm(d - bias, axis=2))
            cells.append(f"{raw:>10.2f} /{deb:>9.2f}")
        print(f"  {clip:<15} " + "".join(cells))

    # ── 6. Cost ─────────────────────────────────────────────────────────
    print()
    print("=" * 78)
    print("6. INFERENCE COST (per frame, this CPU)")
    print("=" * 78)
    for n in names:
        ms = np.array([_f(r[f"{n}_ms"]) or 0.0 for r in all_rows])
        print(f"  {n:<12} median {np.median(ms):6.2f} ms   p95 {np.percentile(ms,95):6.2f} ms")



# ── Gate 4: rotation through the crop ───────────────────────────────────────
# The detector's bbox becomes the 224x224 crop that 6DRepNet reads, so a
# different box means a different crop means different rotation. This measures
# the actual downstream effect rather than inferring it from bbox IoU.
# Mirrors pose_estimator_6drepnet.rs + image_proc.rs exactly.

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class SixDRepNet:
    def __init__(self, model=SIXDREP_MODEL):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.log_severity_level = 3
        self.sess = ort.InferenceSession(model, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        sh = self.sess.get_inputs()[0].shape
        self.h, self.w = int(sh[2]), int(sh[3])

    def run(self, bgr, bbox):
        """bbox = (x, y, w, h). Returns (yaw, pitch, roll) or None if the
        patch falls outside the frame — same rejection as the Rust."""
        import cv2
        x, y, bw, bh = bbox
        patch_size = int(max(bw, bh) * 1.05)      # Detection::patch_size
        cx, cy = x + bw * 0.5, y + bh * 0.5       # Detection::center
        half = patch_size * 0.5
        x0, y0 = int(round(cx - half)), int(round(cy - half))
        H, W = bgr.shape[:2]
        if x0 < 0 or y0 < 0 or x0 + patch_size > W or y0 + patch_size > H:
            return None
        patch = bgr[y0:y0 + patch_size, x0:x0 + patch_size]
        patch = cv2.resize(patch, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = ((rgb - IMAGENET_MEAN) / IMAGENET_STD).transpose(2, 0, 1)[None].astype(np.float32)
        r = self.sess.run(None, {self.inp: blob})[0].reshape(-1)
        return rotation_to_ypr(r)


def rotation_to_ypr(r):
    """Mirrors rotation_matrix_to_yaw_pitch_roll, including the roll flip."""
    r00, r10, r20, r21, r22 = r[0], r[3], r[6], r[7], r[8]
    r11, r12 = r[4], r[5]
    sy = float(np.hypot(r00, r10))
    if sy < 1e-6:
        pitch, roll = np.arctan2(-r12, r11), 0.0
    else:
        pitch, roll = np.arctan2(r21, r22), np.arctan2(r10, r00)
    yaw = np.arctan2(-r20, sy)
    d = 180.0 / np.pi
    return float(yaw * d), float(pitch * d), float(-roll * d)   # ROLL_SIGN = -1


def cmd_rotation(clips, which):
    import cv2
    dets = build_detectors(which)
    rot = SixDRepNet()
    ref = "scrfd"
    agg = {}
    print("=" * 78)
    print("4. ROTATION THROUGH THE CROP — the downstream effect on head angles")
    print("=" * 78)
    print("  Each detector's bbox -> 224x224 crop -> 6DRepNet. Delta vs SCRFD's crop")
    print("  on the SAME frame, in degrees. This is what a swap would actually change.")
    for clip_dir in clips:
        clip = os.path.basename(clip_dir)
        rows, _ = read_detections(clip_dir)
        if rows is None:
            continue
        per = {d.name: [] for d in dets if d.name != ref}
        # Subsample: 6DRepNet is ~3 ms and this is a comparison of medians,
        # not a per-frame trace. Every 5th frame is ample and keeps the pass
        # to a couple of minutes.
        for r in rows[::5]:
            img = cv2.imread(os.path.join(clip_dir, f"frame_{int(r['frame_idx']):06d}.jpg"))
            if img is None:
                continue
            base = None
            got = {}
            for d in dets:
                if r[f"{d.name}_found"] != "1":
                    continue
                bb = tuple(_f(r[f"{d.name}_b{k}"]) for k in ("x", "y", "w", "h"))
                if any(v is None for v in bb):
                    continue
                ypr = rot.run(img, bb)
                if ypr is None:
                    continue
                got[d.name] = ypr
            base = got.get(ref)
            if base is None:
                continue
            for n, v in got.items():
                if n == ref:
                    continue
                per[n].append([abs(a - b) for a, b in zip(v, base)])
        for n, v in per.items():
            if len(v) < 10:
                continue
            a = np.array(v)
            agg.setdefault(n, []).append(a)
            print(f"  {clip:<15} {n:<10} yaw {np.median(a[:,0]):5.2f}/{np.percentile(a[:,0],95):5.2f}"
                  f"  pitch {np.median(a[:,1]):5.2f}/{np.percentile(a[:,1],95):5.2f}"
                  f"  roll {np.median(a[:,2]):5.2f}/{np.percentile(a[:,2],95):5.2f}   (med/p95 deg, n={len(v)})")
    print()
    for n, v in agg.items():
        a = np.concatenate(v)
        med = a.mean(axis=0)
        p95 = np.percentile(a, 95, axis=0)
        ok = np.median(a) <= 1.0 and np.percentile(a, 95) <= 3.0
        print(f"  POOLED {n:<10} median {np.median(a):.2f} deg  p95 {np.percentile(a,95):.2f} deg"
              f"   => {'PASS' if ok else 'FAIL'} (gate: med<=1.0, p95<=3.0)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=os.path.expanduser("~/headtrack-bakeoff"))
    ap.add_argument("--clip", help="only this clip")
    ap.add_argument("--validate", action="store_true", help="check the SCRFD port vs production")
    ap.add_argument("--detect", action="store_true", help="run detectors over the frames")
    ap.add_argument("--summary", action="store_true", help="the decision metrics")
    ap.add_argument("--rotation", action="store_true", help="gate 4: rotation through the crop")
    ap.add_argument("--detectors", default="scrfd,yunet,yunet640,mediapipe")
    args = ap.parse_args()

    if args.clip:
        clips = [os.path.join(args.dir, args.clip)]
    else:
        clips = sorted(d.rstrip("/") for d in glob.glob(os.path.join(args.dir, "*/"))
                       if os.path.exists(os.path.join(d, "frames.csv")))
    if not clips:
        raise SystemExit(f"no clips with frames.csv under {args.dir}")

    if args.validate:
        return cmd_validate(clips)
    if args.detect:
        return cmd_detect(clips, args.detectors.split(","))
    if args.summary:
        return cmd_summary(clips)
    if args.rotation:
        return cmd_rotation(clips, args.detectors.split(","))
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
