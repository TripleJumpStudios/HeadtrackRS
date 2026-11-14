#!/usr/bin/env python3
"""Analyze a HEADTRACK_DENSE_COMPARE run (legacy vs dense paired poses).

Consumes the CSV written by the compare harness (default
/tmp/headtrack_compare.csv, schema documented in docs/DENSE_COMPARE.md) and
prints a per-axis legacy-vs-dense table so the diagnostic decision is one
command instead of a spreadsheet session.

Two questions it answers, both from the same table:

  1. Whole recording (the docs/DENSE_COMPARE.md recipe): does dense's pose
     *shape* correlate with legacy's? High |r| with same sign => the dense
     path's residual is the placeholder canonical (biased-but-coherent),
     and teacher-guided calibration is the right next step. Low/negative r
     => a sign/axis-mapping bug downstream of PnP that calibration won't fix.

  2. Facial segment (head held still, blink/wink/mouth motion): pass
     --from <sec> to restrict the window to the facial tail. There, legacy
     bleeds (large range) while a good dense path stays flat (small range).
     The dense/legacy std ratio column quantifies that rejection.

Stdlib only; no numpy/pandas. dense_* rows that are NaN (dense produced
nothing usable) or collapsed are excluded from dense aggregates and counted
separately.
"""

import argparse
import csv
import math
import sys

AXES = ["yaw", "pitch", "roll", "x", "y", "z"]


def _to_float(s):
    try:
        v = float(s)
    except (ValueError, TypeError):
        return math.nan
    return v


def _to_bool(s):
    return str(s).strip().lower() in ("1", "true", "t", "yes")


def load(path):
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in ("timestamp_us", "legacy_yaw", "dense_yaw")
                   if c not in (reader.fieldnames or [])]
        if missing:
            sys.exit(f"error: {path} missing expected columns: {missing}\n"
                     f"got header: {reader.fieldnames}")
        for r in reader:
            rows.append(r)
    if not rows:
        sys.exit(f"error: {path} has no data rows")
    return rows


def stats(vals):
    """mean, std (population), min, max of the non-NaN values."""
    clean = [v for v in vals if not math.isnan(v)]
    n = len(clean)
    if n == 0:
        return dict(n=0, mean=math.nan, std=math.nan,
                    lo=math.nan, hi=math.nan, rng=math.nan)
    mean = sum(clean) / n
    var = sum((v - mean) ** 2 for v in clean) / n
    std = math.sqrt(var)
    lo, hi = min(clean), max(clean)
    return dict(n=n, mean=mean, std=std, lo=lo, hi=hi, rng=hi - lo)


def pearson(xs, ys):
    """Correlation over rows where BOTH values are non-NaN."""
    pairs = [(x, y) for x, y in zip(xs, ys)
             if not math.isnan(x) and not math.isnan(y)]
    n = len(pairs)
    if n < 2:
        return math.nan, n
    mx = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pairs)
    sxx = sum((p[0] - mx) ** 2 for p in pairs)
    syy = sum((p[1] - my) ** 2 for p in pairs)
    if sxx == 0 or syy == 0:
        return math.nan, n
    return sxy / math.sqrt(sxx * syy), n


def verdict(r, scale):
    if math.isnan(r):
        return "flat/no-var"
    if r >= 0.8:
        return "shape matches -> canonical/offset"
    if r <= -0.5:
        return "INVERTED -> sign bug"
    if r < 0.5:
        return "diverges -> mapping bug"
    return "weak corr"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default="/tmp/headtrack_compare.csv",
                    help="compare CSV (default: /tmp/headtrack_compare.csv)")
    ap.add_argument("--from", dest="t_from", type=float, default=None,
                    metavar="SEC",
                    help="restrict to rows >= SEC seconds from recording start "
                         "(e.g. the facial-segment tail)")
    ap.add_argument("--to", dest="t_to", type=float, default=None, metavar="SEC",
                    help="restrict to rows <= SEC seconds from recording start")
    args = ap.parse_args()

    rows = load(args.csv)
    t0 = _to_float(rows[0]["timestamp_us"])

    def t_sec(r):
        return (_to_float(r["timestamp_us"]) - t0) / 1e6

    sel = rows
    if args.t_from is not None:
        sel = [r for r in sel if t_sec(r) >= args.t_from]
    if args.t_to is not None:
        sel = [r for r in sel if t_sec(r) <= args.t_to]
    if not sel:
        sys.exit("error: time window selected zero rows")

    win_start, win_end = t_sec(sel[0]), t_sec(sel[-1])
    dur = win_end - win_start
    fps = (len(sel) - 1) / dur if dur > 0 else float("nan")
    collapsed = sum(1 for r in sel if _to_bool(r.get("dense_collapsed", "")))
    dense_nan = sum(1 for r in sel if math.isnan(_to_float(r["dense_yaw"])))
    rms = stats([_to_float(r.get("dense_rms_px", "nan")) for r in sel])

    win = f"{win_start:.1f}..{win_end:.1f}s"
    print(f"file        : {args.csv}")
    print(f"window      : {win}  ({len(sel)} rows, {dur:.1f}s, ~{fps:.0f} fps)")
    print(f"dense NaN   : {dense_nan}/{len(sel)} rows "
          f"({100*dense_nan/len(sel):.0f}%)   collapsed: {collapsed}")
    if rms["n"]:
        print(f"dense RMS px: mean {rms['mean']:.1f}  max {rms['hi']:.1f}")
    print()
    hdr = (f"{'axis':5} | {'leg.range':>9} {'leg.std':>8} | "
           f"{'den.range':>9} {'den.std':>8} | {'std d/l':>7} {'r':>6} | verdict")
    print(hdr)
    print("-" * len(hdr))
    for ax in AXES:
        leg = stats([_to_float(r[f"legacy_{ax}"]) for r in sel])
        den = stats([_to_float(r[f"dense_{ax}"]) for r in sel])
        r, _n = pearson([_to_float(rr[f"legacy_{ax}"]) for rr in sel],
                        [_to_float(rr[f"dense_{ax}"]) for rr in sel])
        ratio = (den["std"] / leg["std"]) if leg["std"] not in (0, math.nan) \
            and leg["std"] > 1e-9 else math.nan
        scale = (den["rng"] / leg["rng"]) if leg["rng"] > 1e-9 else math.nan
        rstr = "  n/a" if math.isnan(r) else f"{r:+.2f}"
        ratstr = "  n/a" if math.isnan(ratio) else f"{ratio:5.2f}"
        print(f"{ax:5} | {leg['rng']:9.2f} {leg['std']:8.2f} | "
              f"{den['rng']:9.2f} {den['std']:8.2f} | {ratstr:>7} {rstr:>6} | "
              f"{verdict(r, scale)}")

    print()
    print("legend: range/std in axis units (deg or mm). std d/l = dense_std /")
    print("legacy_std (<1 means dense moves less than legacy). r = Pearson over")
    print("paired non-NaN rows. See docs/DENSE_COMPARE.md for interpretation.")
    print("For the facial-bleed test, restrict with --from <facial_start_sec>:")
    print("low den.range + low std-ratio there = dense rejects the bleed.")


if __name__ == "__main__":
    main()
