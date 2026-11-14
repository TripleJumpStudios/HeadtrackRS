#!/usr/bin/env python3
"""Bundle-adjust the 106-point canonical from captured 2D observations (B2++).

`build_canonical.py` triangulates against the *fixed* legacy poses. Those poses
are too rough (raw 5-pt PnP rotation is exaggerated/noisy), so the structure
collapses to a near-planar face. The captured data does contain real parallax
(confirmed: translation +-150mm, apparent face-width foreshortening to 0.23),
so the cure is to refine poses AND structure jointly to minimise reprojection.

Classic sparse bundle adjustment via scipy.least_squares (Levenberg-Marquardt /
trust-region) with a robust loss. Gauge is fixed by holding frame 0's pose at
its legacy value; perspective pins the scale. Frames are subsampled (plenty for
106 points). Initialised from legacy poses + the triangulated structure.

Input : /tmp/headtrack_calib.csv
Output: /tmp/canonical_106.csv   (same schema as build_canonical.py, so
        reorient_canonical.py consumes it unchanged)
"""

import argparse
import csv
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

N = 106


def load(path, stride):
    rows = list(csv.reader(open(path)))[1:]
    frames = []
    for r in rows[::stride]:
        v = np.array([float(x) for x in r])
        K = v[0:4]                       # fx,fy,cx,cy
        R = v[4:13].reshape(3, 3)
        t = v[13:16]
        pts = v[16:].reshape(N, 2)
        frames.append((K, R, t, pts))
    return frames


def project(R, t, X, fx, fy, cx, cy):
    """X: (N,3) -> (N,2) pixels under p_cam = R X + t."""
    pc = X @ R.T + t
    z = np.clip(pc[:, 2], 1e-3, None)
    u = fx * pc[:, 0] / z + cx
    v = fy * pc[:, 1] / z + cy
    return np.stack([u, v], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="/tmp/headtrack_calib.csv")
    ap.add_argument("--stride", type=int, default=12, help="use every Nth frame")
    ap.add_argument("--iters", type=int, default=2,
                    help="outer rounds (re-triangulate seed, then BA)")
    ap.add_argument("--fscale", type=float, default=3.0, help="robust loss scale (px)")
    args = ap.parse_args()

    frames = load(args.csv, args.stride)
    M = len(frames)
    print(f"{M} frames (stride {args.stride}), {N} points")

    fx, fy, cx, cy = frames[0][0]
    Rs = np.array([f[1] for f in frames])
    ts = np.array([f[2] for f in frames])
    obs = np.array([f[3] for f in frames])           # (M,N,2)

    # ── Seed structure: linear triangulation against legacy poses ───────────
    X = np.zeros((N, 3))
    for i in range(N):
        A = np.zeros((3, 3)); b = np.zeros(3)
        for m in range(M):
            R, t = Rs[m], ts[m]
            a0 = (obs[m, i, 0] - cx) / fx
            a1 = (obs[m, i, 1] - cy) / fy
            w0 = R[0] - a0 * R[2]; d0 = a0 * t[2] - t[0]
            w1 = R[1] - a1 * R[2]; d1 = a1 * t[2] - t[1]
            A += np.outer(w0, w0) + np.outer(w1, w1)
            b += w0 * d0 + w1 * d1
        X[i] = np.linalg.solve(A, b)
    print(f"seed depth (Z) span: {np.ptp(X[:,2]):.1f}mm")

    # ── Parameter packing. Frame 0 fixed (gauge). Optimise frames 1..M-1 ────
    rvecs = Rotation.from_matrix(Rs).as_rotvec()      # (M,3)
    free = list(range(1, M))
    n_pose = len(free)

    def pack(rv, tv, X):
        return np.concatenate([rv[free].ravel(), tv[free].ravel(), X.ravel()])

    def unpack(p):
        rv = rvecs.copy(); tv = ts.copy()
        rv[free] = p[:n_pose * 3].reshape(n_pose, 3)
        tv[free] = p[n_pose * 3:n_pose * 6].reshape(n_pose, 3)
        Xx = p[n_pose * 6:].reshape(N, 3)
        return rv, tv, Xx

    def residuals(p):
        rv, tv, Xx = unpack(p)
        Rm = Rotation.from_rotvec(rv).as_matrix()
        res = np.empty((M, N, 2))
        for m in range(M):
            res[m] = project(Rm[m], tv[m], Xx, fx, fy, cx, cy) - obs[m]
        return res.ravel()

    # Sparsity: residual (m,i,:) depends on pose m (6 params) and point i (3).
    from scipy.sparse import lil_matrix
    n_res = M * N * 2
    n_par = n_pose * 6 + N * 3
    S = lil_matrix((n_res, n_par), dtype=int)
    for m in range(M):
        for i in range(N):
            r0 = (m * N + i) * 2
            if m in free:
                k = free.index(m)
                for c in range(3):
                    S[r0, k * 3 + c] = 1; S[r0 + 1, k * 3 + c] = 1
                    S[r0, n_pose * 3 + k * 3 + c] = 1; S[r0 + 1, n_pose * 3 + k * 3 + c] = 1
            for c in range(3):
                S[r0, n_pose * 6 + i * 3 + c] = 1; S[r0 + 1, n_pose * 6 + i * 3 + c] = 1

    p0 = pack(rvecs, ts, X)
    r0 = residuals(p0)
    print(f"initial reprojection RMS: {np.sqrt((r0**2).mean()):.2f}px")
    sol = least_squares(residuals, p0, jac_sparsity=S, loss="soft_l1",
                        f_scale=args.fscale, max_nfev=60, verbose=2)
    rv, tv, X = unpack(sol.x)
    print(f"final reprojection RMS:   {np.sqrt((sol.fun**2).mean()):.2f}px")
    print(f"BA depth (Z) span: {np.ptp(X[:,2]):.1f}mm  (a real face is ~40mm)")

    with open("/tmp/canonical_106.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["idx", "x", "y", "z", "n_inliers", "resid_px"])
        for i in range(N):
            w.writerow([i, f"{X[i,0]:.3f}", f"{X[i,1]:.3f}", f"{X[i,2]:.3f}", M, ""])
    print("wrote /tmp/canonical_106.csv  (run reorient_canonical.py next)")


if __name__ == "__main__":
    main()
