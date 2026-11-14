#!/usr/bin/env python3
"""Compare two HEADTRACK_Z_GATE_TRACE captures by joint (yaw, pitch) pose bin.

    scripts/gate_trace_compare.py     (edit AB/the two names for other pairs)

Joint (yaw, pitch) binning — the movement-mismatch-proof comparison.

Within one joint pose bin the head is in the same attitude, so:
  WITHIN-bin spread  = cue instability at that pose (plus genuine leaning)
  BETWEEN-bin drift  = rotation->Z cross-talk, the rubber-band
Dwell-weight both by the bins the pilot actually occupied, using a COMMON
weighting (the average of the two runs' dwell) so neither run's harder flying
biases the total.
"""
import csv, math
from pathlib import Path

AB = Path.home() / "headtrack-ab" / "msfs"
YB = [-90, -45, -30, -18, -8, 8, 18, 30, 45, 90]
PB = [-70, -32, -24, -18, -10, 0, 25]


def load(p):
    rows = [l for l in open(p) if not l.startswith("#")]
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(rows)]


def med(xs):
    s = sorted(xs); n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round((len(s) - 1) * p / 100.0))))]


def binof(v, edges):
    for i, (lo, hi) in enumerate(zip(edges, edges[1:])):
        if lo <= v < hi:
            return i
    return None


runs, cells = {}, {}
for cue in ("diagonals", "verticals"):
    rows = load(AB / f"c172-{cue}.zgate.csv")
    z0 = med([r["z_in"] for r in rows])
    pm = med([r["pitch"] for r in rows])
    c = {}
    for r in rows:
        r["zi"] = 100 * (r["z_in"] - z0) / abs(z0)
        r["zo"] = 100 * (r["z_out"] - z0) / abs(z0)
        # centre pitch on each run's own neutral so the two are comparable
        bi, bj = binof(r["yaw"], YB), binof(r["pitch"] - pm + (-13.3), PB)
        if bi is None or bj is None:
            continue
        c.setdefault((bi, bj), []).append(r)
    runs[cue] = (rows, z0, pm)
    cells[cue] = c

common = sorted(set(cells["diagonals"]) & set(cells["verticals"]),
                key=lambda k: -(len(cells["diagonals"][k]) + len(cells["verticals"][k])))
common = [k for k in common if len(cells["diagonals"][k]) >= 25 and len(cells["verticals"][k]) >= 25]

print(f"{len(common)} joint pose bins with >=25 frames in BOTH runs\n")
print(f"{'yaw bin':>12} {'pitch bin':>12} {'wt%':>5} |"
      f" {'diag spread':>11} {'diag bias':>9} {'held':>5} |"
      f" {'vert spread':>11} {'vert bias':>9} {'held':>5} | win")

tot_w = 0.0
acc = {c: dict(spread=0.0, bias=0.0, held=0.0, payback=0.0) for c in runs}
for k in common:
    bi, bj = k
    nd, nv = len(cells["diagonals"][k]), len(cells["verticals"][k])
    w = 0.5 * (nd / len(runs["diagonals"][0]) + nv / len(runs["verticals"][0]))
    tot_w += w
    line = (f"[{YB[bi]:+4.0f},{YB[bi+1]:+4.0f}) [{PB[bj]:+4.0f},{PB[bj+1]:+4.0f})"
            f" {100*w:5.1f} |")
    vals = {}
    for cue in ("diagonals", "verticals"):
        sel = cells[cue][k]
        zs = [r["zi"] for r in sel]
        bm = med(zs)
        spread = pct([abs(z - bm) for z in zs], 95)
        held = 100 * sum(r["held"] for r in sel) / len(sel)
        vals[cue] = (spread, bm, held)
        acc[cue]["spread"] += w * spread
        acc[cue]["bias"] += w * abs(bm)
        acc[cue]["held"] += w * held
        line += f" {spread:11.2f} {bm:+9.2f} {held:4.0f}% |"
    line += "  diag" if vals["diagonals"][0] < vals["verticals"][0] else "  vert"
    print(line)

print(f"\n{'='*104}")
print("DWELL-WEIGHTED TOTALS over the common envelope (equal weighting, so the harder-flown run "
      "gets no credit)")
print(f"{'='*104}")
print(f"{'':<28}{'diagonals':>12}{'verticals':>12}   ratio")
for m, lbl in (("spread", "within-pose spread p95 %"),
               ("bias",   "|pose bias| %"),
               ("held",   "gate held %")):
    a, b = acc["diagonals"][m] / tot_w, acc["verticals"][m] / tot_w
    print(f"{lbl:<28}{a:12.2f}{b:12.2f}   {a/b:5.2f}×  ->  "
          f"{'diagonals' if a < b else 'verticals'}")

# Between-bin cross-talk: how much does the pose bias itself move with pose?
print(f"\nBETWEEN-POSE cross-talk (spread of the per-bin biases — the rubber-band amplitude):")
for cue in ("diagonals", "verticals"):
    bs = [med([r["zi"] for r in cells[cue][k]]) for k in common]
    print(f"  {cue:<12} bias range {max(bs)-min(bs):6.2f}%   sd "
          f"{math.sqrt(sum((x-sum(bs)/len(bs))**2 for x in bs)/(len(bs)-1)):5.2f}%")
