#!/usr/bin/env python3
"""Test whether Z excursions are bounded by the detector re-detect interval.

    scripts/redetect_episodes.py <zgate-trace.csv> [...] [--redetect N]

Input is a `HEADTRACK_Z_GATE_TRACE` file:
    ts_us,z_in,z_out,held,conf,yaw,pitch

WHAT THIS IS FOR
----------------
`REDETECT_THRESHOLD` (tracker.rs) makes the detector re-seed the 5 landmarks
every N frames; optical flow carries them in between. If the Z excursions are
*flow drift*, they are terminated by that re-seed, and two signatures follow:

  1. No excursion episode outlives N samples.
  2. Large corrections recur with a gap clustering at N.

Measured on the 2026-08-21 MSFS sessions at N=300: zero of 51 episodes exceeded
300 samples (maxima 262 and 288), and the median gap between jump episodes was
exactly 301.

THE CONFOUND, AND HOW TO BREAK IT
---------------------------------
The yaw protocol used 5 s holds, and 300 frames at ~62 fps is ~4.8 s — movement
cadence and re-detect cadence nearly coincide, which contaminates signature 2.
Signature 1 is cleaner, because a 5 s holding rhythm does not explain a hard
ceiling.

Break the tie by *varying* the interval and re-running this:

    HEADTRACK_REDETECT_FRAMES=60 scripts/session.sh msfs yunet redetect60

If the ceiling tracks the knob, flow drift is confirmed. Nothing else in the
system has that period. See docs/DEPTH_CUE_PLAN.md.
"""
import argparse, math, statistics as st, sys
from pathlib import Path

EXCURSION_MM = 150.0   # "off baseline" threshold for an episode
JUMP_MM      = 50.0    # single-frame step counted as a correction
MERGE_GAP    = 60      # samples; jumps closer than this are one episode


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            if not line or line[0] == '#' or line.startswith('ts_us'):
                continue
            v = line.split(',')
            if len(v) != 7:
                continue
            try:
                rows.append((int(v[0]) / 1e6, float(v[1]), float(v[2]),
                             int(v[3]), float(v[4]), float(v[5])))
            except ValueError:
                continue
    if not rows:
        sys.exit(f"{path}: no data rows")
    t0 = rows[0][0]
    return [(t - t0, zi, zo, held, conf, yaw) for t, zi, zo, held, conf, yaw in rows]


def analyse(path, redetect):
    r = load(path)
    zi = [x[1] for x in r]
    base = st.median([x[1] for x in r if abs(x[5]) < 5] or zi)

    episodes, cur = [], None
    for i, x in enumerate(r):
        if abs(x[1] - base) > EXCURSION_MM:
            cur = [i, i] if cur is None else [cur[0], i]
        elif cur:
            episodes.append(tuple(cur)); cur = None
    if cur:
        episodes.append(tuple(cur))
    lens = [b - a + 1 for a, b in episodes]

    dz = [abs(zi[i] - zi[i - 1]) for i in range(1, len(zi))]
    starts = []
    for i, d in enumerate(dz):
        if d > JUMP_MM and (not starts or i - starts[-1] > MERGE_GAP):
            starts.append(i)
    gaps = [b - a for a, b in zip(starts, starts[1:])]

    print(f"\n{'=' * 74}\n{Path(path).name}   {len(r)} samples, {r[-1][0]:.0f}s"
          f"   (re-detect = {redetect})\n{'=' * 74}")
    if lens:
        over = [l for l in lens if l > redetect]
        print(f"  excursion episodes (>{EXCURSION_MM:.0f}mm off baseline): {len(lens)}")
        print(f"     length in samples: med={st.median(lens):.0f}  max={max(lens)}")
        print(f"     LONGER than the {redetect}-sample re-detect interval: "
              f"{len(over)}  ({len(over) / len(lens) * 100:.0f}%)")
        print("     -> consistent with flow drift terminated by the re-detect"
              if not over else
              "     -> some episodes SURVIVED a re-detect; flow drift is not the whole story")
    else:
        print(f"  no excursion episodes over {EXCURSION_MM:.0f}mm")

    if len(gaps) >= 3:
        lo, hi = redetect * 0.9, redetect * 1.1
        near = [g for g in gaps if lo <= g <= hi]
        chance = (hi - lo) / max(gaps) * 100 if max(gaps) else float('nan')
        print(f"\n  jump episodes (>{JUMP_MM:.0f}mm, merged within {MERGE_GAP}): {len(starts)}")
        print(f"     gap between starts: med={st.median(gaps):.0f}  mean={st.mean(gaps):.0f}")
        print(f"     within +/-10% of {redetect}: {len(near)}/{len(gaps)}"
              f" = {len(near) / len(gaps) * 100:.0f}%   (chance ~{chance:.0f}%)")
    else:
        print("\n  too few jump episodes for a gap distribution")
    return lens


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traces", nargs="+", help="HEADTRACK_Z_GATE_TRACE CSV files")
    ap.add_argument("--redetect", type=int, default=300,
                    help="re-detect interval in frames the capture ran at (default 300)")
    a = ap.parse_args()
    ceilings = {}
    for t in a.traces:
        lens = analyse(t, a.redetect)
        if lens:
            ceilings[Path(t).name] = max(lens)
    if len(ceilings) > 1:
        print(f"\n{'=' * 74}\nEPISODE CEILINGS (compare against the re-detect interval)\n{'=' * 74}")
        for k, v in ceilings.items():
            print(f"  {v:6d} samples   {k}")
    print()


if __name__ == "__main__":
    main()
