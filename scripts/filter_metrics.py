#!/usr/bin/env python3
"""Objective metrics for the offline pose workbench.

Two modes:

1. **Summary** (characterize a raw capture — no filtered file needed):
       scripts/filter_metrics.py --summary --raw clip.csv
       scripts/filter_metrics.py --dir ~/headtrack-dataset      # all clips
   Per-axis mean / std / peak-to-peak / max|dev-from-median|. For a head-still
   clip (still/blink/smile/talk/brows) every deviation IS jitter/expression
   bleed; for a single-axis motion clip (yaw/x/...) the *other* axes' std is the
   cross-axis bleed. See docs/CAPTURE_PROTOCOL.md.

2. **Compare** (raw vs the filtered CSV from `--example replay`):
       scripts/filter_metrics.py --raw raw.csv --filtered filtered.csv
   Per axis: jitter (std on still segments, raw vs filtered), drift (creep over
   the longest still run), latency (xcorr lag of filtered vs raw on motion).
"""
import argparse
import csv
import glob
import os

import numpy as np

AXES = ["yaw", "pitch", "roll", "x", "y", "z"]
STILL_CLIPS = {"still", "blink", "smile", "talk", "brows"}
# Per-axis "still" velocity thresholds: deg/s for rotation, mm/s for translation.
STILL_ROT = 6.0
STILL_TRANS = 25.0


def unit_of(axis):
    return "deg" if axis in ("yaw", "pitch", "roll") else "mm"


def load(path):
    with open(path, newline="") as f:
        # Skip `#` lines before DictReader sees them. Raw captures carry two
        # kinds: a `# detector=...` provenance stamp on the first line, and
        # `# ts_us=... <msg>` event markers written mid-file by the command
        # socket's `set marker tick 1`. Filtering here rather than after the
        # fact matters because DictReader treats the *first* line as the header,
        # so a leading comment would silently become the column names.
        rows = list(csv.DictReader(ln for ln in f if not ln.lstrip().startswith("#")))
    cols = {a: np.array([float(r[a]) for r in rows]) for a in AXES}
    ts = np.array([float(r.get("timestamp_us", i)) for i, r in enumerate(rows)])
    dt = np.array([float(r["dt_s"]) for r in rows]) if rows and "dt_s" in rows[0] \
        else np.full(len(rows), 1.0 / 30.0)
    return cols, ts, dt


# ── Summary mode ─────────────────────────────────────────────────────────────

def summarize_clip(path, label=None):
    raw, _, dt = load(path)
    n = len(dt)
    name = label or os.path.splitext(os.path.basename(path))[0]
    base = name.lower()
    mean_dt = float(np.mean(dt)) if n else float("nan")
    dur = n * mean_dt
    hz = 1.0 / mean_dt if mean_dt > 0 else float("nan")
    print(f"=== {name}  ({n} frames, {dur:.1f}s @ {hz:.0f}Hz) ===")
    if n == 0:
        print("  (empty)\n")
        return
    hdr = f"{'axis':<6}{'mean':>10}{'std':>9}{'p2p':>9}{'max|dev|':>10}  unit"
    print(hdr)
    print("-" * len(hdr))
    for a in AXES:
        x = raw[a]
        dev = float(np.max(np.abs(x - np.median(x))))
        print(f"{a:<6}{np.mean(x):>10.2f}{np.std(x):>9.3f}"
              f"{(x.max() - x.min()):>9.2f}{dev:>10.2f}  {unit_of(a)}")
    # Interpretation hint.
    if base in AXES:
        bleed = {a: float(np.std(raw[a])) for a in AXES if a != base}
        worst = max(bleed, key=bleed.get)
        print(f"→ intended axis: {base}; largest cross-axis bleed = {worst} "
              f"(std {bleed[worst]:.3f} {unit_of(worst)})")
    elif base in STILL_CLIPS:
        devs = {a: float(np.max(np.abs(raw[a] - np.median(raw[a])))) for a in AXES}
        worst = max(devs, key=devs.get)
        kind = "jitter" if base == "still" else "expression bleed"
        print(f"→ head-still clip: all motion is {kind}; "
              f"worst axis = {worst} (max|dev| {devs[worst]:.2f} {unit_of(worst)})")
    print()


# ── Compare mode ─────────────────────────────────────────────────────────────

def still_mask(raw, dt):
    """True where every axis velocity is below its still threshold."""
    mask = np.ones(len(dt), dtype=bool)
    safe_dt = np.clip(dt, 1e-3, None)
    for a in AXES:
        vel = np.abs(np.gradient(raw[a])) / safe_dt
        thr = STILL_ROT if a in ("yaw", "pitch", "roll") else STILL_TRANS
        mask &= vel < thr
    return mask


def longest_run(mask):
    """(start, end) of the longest contiguous True run in mask."""
    best_len = best_start = cur_start = 0
    cur_len = 0
    for i, m in enumerate(mask):
        if m:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_len, best_start = cur_len, cur_start
        else:
            cur_len = 0
    return best_start, best_start + best_len


def xcorr_lag_ms(raw_a, filt_a, motion, mean_dt, max_lag=20):
    """Lag (ms) maximising cross-correlation of filtered vs raw on motion rows."""
    r = raw_a[motion]
    fsig = filt_a[motion]
    if len(r) < 8 or np.std(r) < 1e-6 or np.std(fsig) < 1e-6:
        return float("nan")
    r = (r - r.mean()) / (np.std(r) + 1e-9)
    fsig = (fsig - fsig.mean()) / (np.std(fsig) + 1e-9)
    best_lag, best_val = 0, -1e18
    for lag in range(0, max_lag + 1):
        # filtered lags raw by `lag` samples (filtered is the delayed copy).
        a = r[: len(r) - lag] if lag else r
        b = fsig[lag:] if lag else fsig
        if len(a) < 8:
            break
        val = float(np.dot(a, b) / len(a))
        if val > best_val:
            best_val, best_lag = val, lag
    return best_lag * mean_dt * 1000.0


def compare(raw_path, filt_path, whole=False):
    raw, _, dt = load(raw_path)
    filt, _, _ = load(filt_path)
    n = min(len(dt), len(filt["yaw"]))
    if len(raw["yaw"]) != len(filt["yaw"]):
        print(f"note: raw has {len(raw['yaw'])} rows, filtered {len(filt['yaw'])}; "
              f"truncating to {n}")
        raw = {a: raw[a][:n] for a in AXES}
        filt = {a: filt[a][:n] for a in AXES}
        dt = dt[:n]

    # --whole: treat every frame as "still" — correct for a single-condition
    # head-still clip, where the velocity detector mis-classifies the axis noise
    # (esp. Z) as motion and computes jitter on almost nothing.
    still = np.ones(n, dtype=bool) if whole else still_mask(raw, dt)
    motion = np.ones(n, dtype=bool) if whole else ~still
    mean_dt = float(np.mean(dt))
    s0, s1 = longest_run(still)
    run_s = (s1 - s0) * mean_dt
    print(f"frames: {n}  still: {still.sum()} ({100*still.mean():.0f}%)  "
          f"motion: {motion.sum()}  mean dt: {mean_dt*1000:.1f} ms")
    print(f"longest still run: {s1-s0} frames ({run_s:.1f} s)\n")

    hdr = f"{'axis':<6}{'raw_jit':>9}{'flt_jit':>9}{'jit_drop':>9}{'drift':>9}{'lat_ms':>8}"
    print(hdr)
    print("-" * len(hdr))
    for a in AXES:
        if still.any():
            raw_jit = float(np.std(raw[a][still]))
            flt_jit = float(np.std(filt[a][still]))
        else:
            raw_jit = flt_jit = float("nan")
        drop = (1 - flt_jit / raw_jit) * 100 if raw_jit > 1e-9 else 0.0
        if s1 - s0 >= 4:
            seg = filt[a][s0:s1]
            t = np.arange(len(seg)) * mean_dt
            slope = np.polyfit(t, seg, 1)[0]
            drift = slope * run_s          # total creep across the run
        else:
            drift = float("nan")
        lat = xcorr_lag_ms(raw[a], filt[a], motion, mean_dt)
        print(f"{a:<6}{raw_jit:>9.3f}{flt_jit:>9.3f}{drop:>8.0f}%"
              f"{drift:>8.2f}{unit_of(a):<3}"[:43] + f"{lat:>8.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", help="raw pose CSV")
    ap.add_argument("--filtered", help="filtered CSV from `--example replay` (compare mode)")
    ap.add_argument("--summary", action="store_true",
                    help="characterize a single raw clip (no --filtered needed)")
    ap.add_argument("--dir", help="summarize every *.csv raw clip in a directory")
    ap.add_argument("--label", help="override clip label (default = filename stem)")
    ap.add_argument("--whole", action="store_true",
                    help="compare mode: treat the whole clip as still (head-still clips)")
    args = ap.parse_args()

    if args.dir:
        paths = sorted(glob.glob(os.path.join(args.dir, "*.csv")))
        if not paths:
            ap.error(f"no .csv files in {args.dir}")
        for p in paths:
            summarize_clip(p)
        return

    if args.summary or (args.raw and not args.filtered):
        if not args.raw:
            ap.error("--summary needs --raw")
        summarize_clip(args.raw, args.label)
        return

    if not (args.raw and args.filtered):
        ap.error("provide --raw + --filtered (compare), or --summary/--dir (characterize)")
    compare(args.raw, args.filtered, whole=args.whole)


if __name__ == "__main__":
    main()
