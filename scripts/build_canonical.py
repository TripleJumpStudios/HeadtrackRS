#!/usr/bin/env python3
"""Rebuild the 106-point dense canonical from captured calibration data (B2).

Consumes /tmp/headtrack_calib.csv (written by compare mode: per frame, the
camera intrinsics, the trusted *legacy* head pose R|t, and the raw 2D106
landmark observations). With the pose known per frame, each canonical 3D point
is recovered independently by linear multi-view triangulation:

    for landmark i, across frames j:
        a = (u - cx)/fx ,  b = (v - cy)/fy
        (R0 - a*R2) . X = a*t2 - t0
        (R1 - b*R2) . X = b*t2 - t1
    stack -> least squares (3x3 normal equations) -> X_i   (millimetres)

A robust second pass drops per-frame outlier observations (bad legacy poses /
tracking glitches) and re-solves. Reports the post-rebuild reprojection
residual per landmark and overall, to compare against the hand-built
canonical's 8.9 px baseline (Path A) BEFORE re-testing in the sim.

Outputs:
  - /tmp/canonical_106.csv      idx,x,y,z,n_inliers,resid_px   (sorted by idx)
  - /tmp/canonical_106.rs.txt   ready-to-paste Rust array for face_model_106.rs

Stdlib only.
"""

import argparse
import csv
import math
import statistics
import sys

N = 106


def solve3(ata, atc):
    """Solve the 3x3 system ata * x = atc (Gaussian elimination, partial pivot)."""
    m = [list(ata[r]) + [atc[r]] for r in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None  # singular -> ill-conditioned (insufficient pose diversity)
        m[col], m[piv] = m[piv], m[col]
        for r in range(3):
            if r != col:
                f = m[r][col] / m[col][col]
                for c in range(col, 4):
                    m[r][c] -= f * m[col][c]
    return [m[r][3] / m[r][r] for r in range(3)]


def triangulate(views, drop_factor=None):
    """views: list of (R(9), t(3), fx,fy,cx,cy, u,v). Returns (X, resid_list)
    where resid_list is per-view reprojection error of the solution (px)."""
    ata = [[0.0] * 3 for _ in range(3)]
    atc = [0.0, 0.0, 0.0]

    def accumulate(eqs):
        for w, d in eqs:
            for r in range(3):
                for c in range(3):
                    ata[r][c] += w[r] * w[c]
                atc[r] += w[r] * d

    eqs_per_view = []
    for (R, t, fx, fy, cx, cy, u, v) in views:
        a = (u - cx) / fx
        b = (v - cy) / fy
        R0, R1, R2 = R[0:3], R[3:6], R[6:9]
        w1 = [R0[k] - a * R2[k] for k in range(3)]
        d1 = a * t[2] - t[0]
        w2 = [R1[k] - b * R2[k] for k in range(3)]
        d2 = b * t[2] - t[1]
        eqs_per_view.append((w1, d1, w2, d2))
    accumulate([(w1, d1) for (w1, d1, _, _) in eqs_per_view]
               + [(w2, d2) for (_, _, w2, d2) in eqs_per_view])
    X = solve3(ata, atc)
    if X is None:
        return None, []

    resids = [reproject_err(R, t, fx, fy, cx, cy, u, v, X)
              for (R, t, fx, fy, cx, cy, u, v) in views]

    if drop_factor is not None and len(views) > 8:
        med = statistics.median(resids)
        thr = max(3.0, drop_factor * med)
        keep = [vw for vw, e in zip(views, resids) if e <= thr]
        if 8 <= len(keep) < len(views):
            return triangulate(keep, drop_factor=None)  # one robust re-solve
    return X, resids


def reproject_err(R, t, fx, fy, cx, cy, u, v, X):
    xc = R[0] * X[0] + R[1] * X[1] + R[2] * X[2] + t[0]
    yc = R[3] * X[0] + R[4] * X[1] + R[5] * X[2] + t[1]
    zc = R[6] * X[0] + R[7] * X[1] + R[8] * X[2] + t[2]
    if zc < 1.0:
        return float("inf")
    pu = fx * xc / zc + cx
    pv = fy * yc / zc + cy
    return math.hypot(pu - u, pv - v)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default="/tmp/headtrack_calib.csv")
    ap.add_argument("--drop-factor", type=float, default=3.0,
                    help="drop a frame's obs if its residual > max(3px, f*median); 0 disables")
    args = ap.parse_args()

    with open(args.csv, newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        rows = [r for r in reader if r]
    if not rows:
        sys.exit(f"error: {args.csv} has no data rows")
    print(f"loaded {len(rows)} frames from {args.csv}")

    # Per-landmark view lists.
    views = [[] for _ in range(N)]
    for row in rows:
        vals = [float(x) for x in row]
        fx, fy, cx, cy = vals[0:4]
        R = vals[4:13]
        t = vals[13:16]
        pts = vals[16:]
        for i in range(N):
            u, v = pts[2 * i], pts[2 * i + 1]
            if math.isfinite(u) and math.isfinite(v):
                views[i].append((R, t, fx, fy, cx, cy, u, v))

    drop = args.drop_factor if args.drop_factor > 0 else None
    results = []  # (idx, X, n_inliers, rms_px)
    for i in range(N):
        if len(views[i]) < 8:
            results.append((i, None, len(views[i]), float("nan")))
            continue
        X, resids = triangulate(views[i], drop_factor=drop)
        if X is None:
            results.append((i, None, len(views[i]), float("nan")))
            continue
        finite = [e for e in resids if math.isfinite(e)]
        rms = math.sqrt(sum(e * e for e in finite) / len(finite)) if finite else float("nan")
        results.append((i, X, len(finite), rms))

    ok = [r for r in results if r[1] is not None and math.isfinite(r[3])]
    rmss = [r[3] for r in ok]
    print(f"\nsolved {len(ok)}/{N} landmarks")
    if rmss:
        print(f"post-rebuild reprojection residual: mean {statistics.mean(rmss):.2f}px  "
              f"median {statistics.median(rmss):.2f}px  max {max(rmss):.2f}px")
        print("  (Path A hand-built canonical baseline was 8.9px mean — lower is better)")
    worst = sorted(ok, key=lambda r: -r[3])[:8]
    print("worst-fit after rebuild:")
    for i, X, n, rms in worst:
        print(f"  idx {i:3}  resid {rms:5.2f}px  n={n}  X=({X[0]:.1f},{X[1]:.1f},{X[2]:.1f})")

    # CSV out.
    out_csv = "/tmp/canonical_106.csv"
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["idx", "x", "y", "z", "n_inliers", "resid_px"])
        for i, X, n, rms in results:
            if X is None:
                w.writerow([i, "", "", "", n, ""])
            else:
                w.writerow([i, f"{X[0]:.3f}", f"{X[1]:.3f}", f"{X[2]:.3f}", n, f"{rms:.3f}"])
    print(f"\nwrote {out_csv}")

    # Rust array out (for face_model_106.rs).
    out_rs = "/tmp/canonical_106.rs.txt"
    with open(out_rs, "w") as fh:
        fh.write("// Generated by scripts/build_canonical.py — data-driven 106-pt canonical (mm).\n")
        fh.write("// Indices that failed to solve (too few views / singular) are left at (0,0,0).\n")
        fh.write("let pts: [[f32; 3]; 106] = [\n")
        for i, X, n, rms in results:
            if X is None:
                fh.write(f"    [0.0, 0.0, 0.0], // idx {i}: UNSOLVED (n={n})\n")
            else:
                fh.write(f"    [{X[0]:.2f}, {X[1]:.2f}, {X[2]:.2f}], // idx {i}: {rms:.1f}px n={n}\n")
        fh.write("];\n")
    print(f"wrote {out_rs}")


if __name__ == "__main__":
    main()
