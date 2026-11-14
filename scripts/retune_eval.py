#!/usr/bin/env python3
"""P13 re-tune evaluation: couplings, rest noise and travel on raw-pose captures.

    scripts/retune_eval.py couple  RAW.csv [RAW.csv ...]
    scripts/retune_eval.py score   RAW.csv FILTERED.csv

Pure numbers, no imagery: the inputs are HEADTRACK_DUMP_RAW pose CSVs (six floats
per frame) and the replay tool's output for the same file
(`cargo run -p headtrack-core --example replay`). See docs/P13_RETUNE.md.

couple
    Measures the pitch→(z, y, yaw) couplings that CrossAxisCompStage cancels, on
    pitch-dominant motion only: deltas over LAG_S where |Δpitch| is large and yaw
    and roll barely move. Slope = least squares through the origin after trimming
    the worst 10 % of residuals, plus the plain median ratio as a cross-check. The
    pipeline subtracts `pitch * coeff`, so the coefficient to ship IS the slope.
    The tracker's Z gate holds Z on some frames (Δz = 0); pairs touching a held
    frame are dropped when a sibling `.zgate.csv` exists, or the slope is biased
    toward zero.

score
    Rest  — 1 s windows whose RAW yaw and pitch ranges stay under 3°: pooled
            within-window sd of the output, per axis.
    Travel — 1 s windows where the raw signal (0.3 s moving average, to strip
            noise) genuinely moves: median ratio output p2p / raw p2p. For X/Y/Z
            the window must also have yaw and pitch within ROT_QUIET, or the
            ratio measures cross-axis compensation rather than lost travel.
    Z     — p50 / p95 of |Z output| over the whole session (the pipeline output
            is already centred).
"""
import sys
from pathlib import Path
import numpy as np

AXES = ["yaw", "pitch", "roll", "x", "y", "z"]
LAG_S = 0.20
MOVE = {"yaw": 8.0, "pitch": 6.0, "roll": 6.0, "x": 15.0, "y": 15.0, "z": 25.0}
ROT_QUIET = 4.0


def load(path):
    rows, cols = [], None
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split(",")
            if cols is None:
                cols = parts
                continue
            rows.append([float(p) for p in parts])
    a = np.array(rows)
    return {c: a[:, i] for i, c in enumerate(cols)}


def held_mask(raw_path, ts):
    """Per-frame Z-gate held flag aligned to the raw rows, or None."""
    p = Path(raw_path)
    name = p.name.replace("yunet-", "").replace("scrfd-", "").replace(".csv", ".zgate.csv")
    z = p.with_name(name)
    if not z.exists():
        return None
    g = load(z)
    idx = np.searchsorted(g["ts_us"], ts)
    idx = np.clip(idx, 0, len(g["ts_us"]) - 1)
    ok = np.abs(g["ts_us"][idx] - ts) < 2000
    held = np.where(ok, g["held"][idx] > 0, False)
    return held


def slope(dx, dy):
    if len(dx) < 30:
        return float("nan"), float("nan"), len(dx)
    k = np.sum(dx * dy) / np.sum(dx * dx)
    r = np.abs(dy - k * dx)
    keep = r <= np.quantile(r, 0.90)
    k = np.sum(dx[keep] * dy[keep]) / np.sum(dx[keep] ** 2)
    return float(k), float(np.median(dy / dx)), len(dx)


def couple(paths):
    print(f"{'file':34s} {'n':>5s} {'p->z':>14s} {'p->y':>14s} {'p->yaw':>14s} {'standoff':>9s}")
    for path in paths:
        d = load(path)
        ts = d["timestamp_us"]
        held = held_mask(path, ts)
        lag = max(1, int(round(LAG_S / np.median(np.diff(ts) / 1e6))))
        dp = d["pitch"][lag:] - d["pitch"][:-lag]
        dyaw = d["yaw"][lag:] - d["yaw"][:-lag]
        droll = d["roll"][lag:] - d["roll"][:-lag]
        sel = (np.abs(dp) > 3.0) & (np.abs(dyaw) < 1.5) & (np.abs(droll) < 1.5)
        selz = sel.copy()
        if held is not None:
            selz &= ~(held[lag:] | held[:-lag])
        out = []
        for ax, s in (("z", selz), ("y", sel), ("yaw", sel)):
            dv = d[ax][lag:] - d[ax][:-lag]
            k, med, n = slope(dp[s], dv[s])
            out.append(f"{k:+6.2f} ({med:+5.2f})")
        tag = Path(path).name[:34]
        print(f"{tag:34s} {int(sel.sum()):5d} {out[0]:>14s} {out[1]:>14s} {out[2]:>14s} "
              f"{np.median(d['z']):9.0f}{'' if held is not None else '  (no gate file)'}")


def windows(ts, width_s=1.0):
    t = (ts - ts[0]) / 1e6
    edges = np.arange(0, t[-1], width_s)
    idx = np.searchsorted(t, edges)
    return [(a, b) for a, b in zip(idx[:-1], idx[1:]) if b - a >= 10]


def movavg(v, n):
    if n <= 1:
        return v
    c = np.cumsum(np.insert(v, 0, 0.0))
    m = (c[n:] - c[:-n]) / n
    pad = n // 2
    return np.concatenate([np.full(pad, m[0]), m, np.full(len(v) - len(m) - pad, m[-1])])


def score(raw_path, out_path, quiet=False):
    raw, out = load(raw_path), load(out_path)
    n = min(len(raw["yaw"]), len(out["yaw"]))
    raw = {k: v[:n] for k, v in raw.items()}
    out = {k: v[:n] for k, v in out.items()}
    ts = raw["timestamp_us"]
    fr = 1e6 / np.median(np.diff(ts))
    smooth = {ax: movavg(raw[ax], max(1, int(0.3 * fr))) for ax in AXES}
    res = {}
    rest_w = [(a, b) for a, b in windows(ts)
              if np.ptp(raw["yaw"][a:b]) < 3.0 and np.ptp(raw["pitch"][a:b]) < 3.0]
    for ax in AXES:
        o = out[ax]
        r = raw[ax]
        rest_o = np.concatenate([o[a:b] - o[a:b].mean() for a, b in rest_w]) if rest_w else np.array([0.0])
        rest_r = np.concatenate([r[a:b] - r[a:b].mean() for a, b in rest_w]) if rest_w else np.array([0.0])
        ratios = []
        for a, b in windows(ts):
            p2p = np.ptp(smooth[ax][a:b])
            # A translation window only counts when the head is not also turning:
            # otherwise "travel" measures the cross-axis stage removing pivot motion.
            if ax in ("x", "y", "z") and (np.ptp(smooth["yaw"][a:b]) > ROT_QUIET
                                          or np.ptp(smooth["pitch"][a:b]) > ROT_QUIET):
                continue
            if p2p > MOVE[ax]:
                ratios.append(np.ptp(o[a:b]) / p2p)
        res[ax] = dict(rest_out=float(rest_o.std()), rest_raw=float(rest_r.std()),
                       travel=float(np.median(ratios)) if ratios else float("nan"),
                       n_move=len(ratios))
    az = np.abs(out["z"])
    res["z_p50"], res["z_p95"] = float(np.percentile(az, 50)), float(np.percentile(az, 95))
    res["rest_windows"] = len(rest_w)
    if not quiet:
        print(f"{Path(raw_path).name}: {len(rest_w)} rest windows")
        print(f"{'axis':5s} {'rest raw':>9s} {'rest out':>9s} {'travel':>7s} {'n':>4s}")
        for ax in AXES:
            r = res[ax]
            print(f"{ax:5s} {r['rest_raw']:9.3f} {r['rest_out']:9.3f} {r['travel']:7.2f} {r['n_move']:4d}")
        print(f"|Z out| p50 {res['z_p50']:.1f}  p95 {res['z_p95']:.1f}")
    return res


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "couple":
        couple(sys.argv[2:])
    elif len(sys.argv) == 4 and sys.argv[1] == "score":
        score(sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
        sys.exit(2)
