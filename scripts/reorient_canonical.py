#!/usr/bin/env python3
"""Reorient a data-driven canonical into the convention frame (B2, step 2).

`build_canonical.py` recovers a metrically-correct, bilaterally-symmetric face,
but expressed in the legacy poses' gauge frame (carrying a fixed ~18 deg tilt).
Dense PnP only yields clean yaw/pitch/roll if the canonical sits in the
convention frame the rest of the pipeline assumes:

    +X = image-right   (subject's-right landmarks at -X)
    +Y = down          (chin at +Y)
    -Z = toward camera (nose tip most negative Z)   -> right-handed

We do NOT align to the old hand model: its per-point positions are wrong enough
that point-matching mis-assigns the depth axis (observed: Z collapsed to 13mm).
Instead the frame is built INTRINSICALLY from the recovered model itself:

  * left-right (X) axis = the bilateral-symmetry axis (mean of L/R mirror-pair
    difference vectors),
  * vertical (Y) = the dominant in-plane direction (PCA; the face is far taller
    than deep), so it is unambiguously chin->brow,
  * depth (Z) = X x Y  (forced right-handed, so a proper rotation -- preserves
    the triangulated chirality; no reflection),
  * signs pinned by anatomy: subject's-right eye centroid -> -X, chin -> +Y.

Scale is matched to the old model's RMS radius so downstream translation
magnitudes stay in the range the pipeline was tuned for (scale does not affect
recovered rotation). Old model is read only for that scalar + as a sanity ref.

Inputs : /tmp/canonical_106.csv, /tmp/old_canonical_106.csv
Outputs: /tmp/canonical_106_aligned.csv, /tmp/canonical_106_aligned.rs.txt
"""

import argparse
import csv
import sys
import numpy as np

N = 106
EYE_R = list(range(33, 43))   # subject's-right eye (InsightFace 2D106)
MOUTH = list(range(52, 72))
PAIRS = (list(zip(range(1, 17), range(17, 33)))
         + list(zip(range(33, 43), range(87, 97)))
         + list(zip(range(43, 52), range(97, 106))) + [(52, 61)])


def load(path):
    pts = {}
    for r in csv.DictReader(open(path)):
        if r.get("x", "") == "":
            continue
        pts[int(r["idx"])] = np.array([float(r["x"]), float(r["y"]), float(r["z"])])
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recovered", default="/tmp/canonical_106.csv")
    ap.add_argument("--old", default="/tmp/old_canonical_106.csv")
    args = ap.parse_args()

    rec = load(args.recovered)
    old = load(args.old)
    if len(rec) < 90:
        sys.exit(f"error: only {len(rec)} solved points in {args.recovered}")
    P = np.array([rec[i] for i in range(N) if i in rec])
    centroid = P.mean(0)

    # ── X axis: bilateral-symmetry axis from mirror-pair differences ────────
    diffs = []
    for a, b in PAIRS:
        if a in rec and b in rec:
            d = rec[a] - rec[b]
            if np.linalg.norm(d) > 5:
                diffs.append(d / np.linalg.norm(d))
    diffs = np.array(diffs)
    # consistent direction (flip any that oppose the first), then average
    ref = diffs[0]
    diffs = np.array([d if d @ ref > 0 else -d for d in diffs])
    ex = diffs.mean(0)
    ex /= np.linalg.norm(ex)
    # sign: subject's-right eye centroid must land at -X
    eye_r = np.mean([rec[i] for i in EYE_R if i in rec], 0) - centroid
    if eye_r @ ex > 0:
        ex = -ex

    # ── Y axis: dominant direction in the plane perpendicular to X ──────────
    Pc = P - centroid
    Pproj = Pc - np.outer(Pc @ ex, ex)          # remove X component
    _, _, Vt = np.linalg.svd(Pproj, full_matrices=False)
    ey = Vt[0] - (Vt[0] @ ex) * ex              # orthogonalize vs ex
    ey /= np.linalg.norm(ey)
    # sign: chin (idx 0) must land at +Y (most-down)
    if 0 in rec and (rec[0] - centroid) @ ey < 0:
        ey = -ey

    # ── Z axis: right-handed cross product (proper rotation, no reflection) ─
    ez = np.cross(ex, ey)

    R = np.vstack([ex, ey, ez])                 # rows map world->frame
    assert np.linalg.det(R) > 0, "frame is not a proper rotation"

    # scale to old model's RMS radius (keeps downstream magnitudes familiar)
    rec_rms = np.sqrt((Pc ** 2).sum(1).mean())
    Oc = np.array(list(old.values())) - np.array(list(old.values())).mean(0)
    old_rms = np.sqrt((Oc ** 2).sum(1).mean())
    scale = old_rms / rec_rms

    # origin at the eye/mouth midpoint to mirror the old convention
    A = {i: scale * (R @ (rec[i] - centroid)) for i in rec}
    eyes_mouth = [A[i] for i in (EYE_R + list(range(87, 97)) + MOUTH) if i in A]
    org = np.mean(eyes_mouth, 0)
    A = {i: v - org for i, v in A.items()}

    # ── Validation ──────────────────────────────────────────────────────────
    xs = [abs(A[a][0] + A[b][0]) for a, b in PAIRS if a in A and b in A]
    ys = [abs(A[a][1] - A[b][1]) for a, b in PAIRS if a in A and b in A]
    arr = np.array(list(A.values()))
    zmin = arr[:, 2].min()
    ymax = arr[:, 1].max()
    print(f"frame scale {scale:.3f}")
    print(f"bilateral symmetry: |x_a+x_b| mean {np.mean(xs):.1f}mm  "
          f"|y_a-y_b| mean {np.mean(ys):.1f}mm  (lower=better)")
    print(f"dims: X {arr[:,0].min():.0f}..{arr[:,0].max():.0f}  "
          f"Y {arr[:,1].min():.0f}..{arr[:,1].max():.0f}  "
          f"Z {arr[:,2].min():.0f}..{arr[:,2].max():.0f} mm  "
          f"(expect ~140 wide, ~150 tall, ~40 deep)")
    if 86 in A:
        ok = abs(A[86][2] - zmin) < 5
        print(f"nose tip idx86 z={A[86][2]:.1f}  most-forward z={zmin:.1f}  "
              f"[{'OK' if ok else 'CHECK: nose not most-forward'}]")
    if 0 in A:
        ok = abs(A[0][1] - ymax) < 6
        print(f"chin   idx 0 y={A[0][1]:.1f}  most-down y={ymax:.1f}  "
              f"[{'OK' if ok else 'CHECK: chin not most-down'}]")

    with open("/tmp/canonical_106_aligned.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["idx", "x", "y", "z"])
        for i in range(N):
            if i in A:
                w.writerow([i, f"{A[i][0]:.3f}", f"{A[i][1]:.3f}", f"{A[i][2]:.3f}"])
            else:
                w.writerow([i, "", "", ""])
    with open("/tmp/canonical_106_aligned.rs.txt", "w") as fh:
        fh.write("    // Data-driven canonical: triangulated from a 6-DOF sweep "
                 "(scripts/build_canonical.py),\n")
        fh.write("    // reoriented to the convention frame intrinsically "
                 "(scripts/reorient_canonical.py). mm.\n")
        fh.write("    let pts: [[f32; 3]; 106] = [\n")
        for i in range(N):
            p = A.get(i, np.zeros(3))
            fh.write(f"        [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}], // {i}\n")
        fh.write("    ];\n")
    print("\nwrote /tmp/canonical_106_aligned.csv and .rs.txt")


if __name__ == "__main__":
    main()
