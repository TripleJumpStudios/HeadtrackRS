#!/usr/bin/env python3
"""Rank depth cues across the whole attitude envelope — and the record of a fix that failed.

    scripts/cue_pitch_eval.py        (reads ~/headtrack-bakeoff/*/frames.csv, read-only)

This is the eval that decided `diagonals` for v0.2.0. Unlike an in-sim A/B it runs
EVERY cue over IDENTICAL frames, so movement mismatch between runs is impossible
by construction — which is why it outranked the flown C172 pair when the two
disagreed. See docs/DEPTH_CUE_PLAN.md §0.

FINDING BEING TESTED
--------------------
`Diagonals` is a strict SUBSET of `Verticals`:
    Verticals = [(0,3),(1,4),(0,4),(1,3)]   # 2 same-side + 2 diagonal
    Diagonals =                [(0,4),(1,3)] # the 2 diagonal only

The same-side pairs are near-vertical on the face, so PITCH foreshortens them:
at -60° they keep 46% of their length against the diagonals' 67%. Noise gain is
1/retention, so 2.17× vs 1.49×. That is why `verticals` collapses in pitch-down
(gate held 66-82%) and `diagonals` does not (28-36%) — measured in the C172 pair.

The conditioning guard that was supposed to handle this is DEAD CODE:
MIN_PROJECTED_MM = 6.0 mm, but the shortest projected baseline at 60° pitch is
36.9 mm. It has never fired, on any pair, at any attitude.

THE FIX UNDER TEST: drop a pair whose projected length falls below a FRACTION of
its own unrotated length. That sheds the vertical pairs in pitch-down and the
IPD pair in yaw, while keeping all four pairs in the bulk where the extra
averaging is what makes `verticals` 2× steadier.
"""
import csv, math, statistics as st
from pathlib import Path

BAKEOFF = Path.home() / "headtrack-bakeoff"      # READ ONLY
FACE = [(-31.5,-35.0,0.0),(31.5,-35.0,0.0),(0.0,-5.0,-25.0),
        (-22.0,45.0,-5.0),(22.0,45.0,-5.0)]
ALLP = [(i,j) for i in range(5) for j in range(i+1,5)]

CUES = {
    "ipd":            [(0,1)],
    "eye-mouth":      [(0,3),(1,4)],
    "diagonals":      [(0,4),(1,3)],
    "verticals":      [(0,3),(1,4),(0,4),(1,3)],
    "median-10":      ALLP,
}
FLOORS = [None, 0.90, 0.85, 0.80, 0.75, 0.70]


def rot(yaw, pitch, roll):
    y, p, r = math.radians(yaw), math.radians(-pitch), math.radians(-roll)
    cy,sy,cp,sp,cr,sr = math.cos(y),math.sin(y),math.cos(p),math.sin(p),math.cos(r),math.sin(r)
    Rx=[[1,0,0],[0,cp,-sp],[0,sp,cp]]
    Ry=[[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz=[[cr,-sr,0],[sr,cr,0],[0,0,1]]
    mul=lambda A,B:[[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(mul(Rz,Ry),Rx)


REST = {p: math.hypot(FACE[p[0]][0]-FACE[p[1]][0], FACE[p[0]][1]-FACE[p[1]][1]) for p in ALLP}


def depth(kp, R, pairs, fl, floor):
    Q = [[sum(R[a][b]*P[b] for b in range(3)) for a in range(3)] for P in FACE]
    s = []
    for i, j in pairs:
        Lp = math.hypot(Q[i][0]-Q[j][0], Q[i][1]-Q[j][1])
        if Lp < 6.0:
            continue
        if floor is not None and Lp < floor * REST[(i, j)]:
            continue                      # retention floor — the fix
        dpx = math.dist(kp[i], kp[j])
        if dpx < 8.0:
            continue
        s.append(dpx / Lp)
    if not s:
        return None, 0
    return fl / st.median(s), len(s)


def load(clip):
    rows = []
    f = BAKEOFF / clip / "frames.csv"
    if not f.exists():
        return rows
    with open(f) as fh:
        for r in csv.DictReader(fh):
            try:
                kp = [(float(r[f"kp{i}_x"]), float(r[f"kp{i}_y"])) for i in range(5)]
                rows.append((float(r["yaw"]), float(r["pitch"]), float(r["roll"]), kp))
            except (ValueError, KeyError):
                pass
    return rows


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s)-1, max(0, int(round((len(s)-1)*p/100.0))))]


FL = 640 / (2 * math.tan(math.radians(68.5) / 2))    # ~ diag-FOV focal, px

CLIPS = ["pitch-extreme", "yaw-hold-max", "yaw-hold-45", "yaw-sweep", "still", "combined"]

for clip in CLIPS:
    rows = load(clip)
    if len(rows) < 50:
        continue
    pitches = [r[1] for r in rows]
    yaws = [r[0] for r in rows]
    print(f"\n{'='*94}")
    print(f"{clip}   n={len(rows)}   pitch {min(pitches):+.0f}..{max(pitches):+.0f}°   "
          f"yaw {min(yaws):+.0f}..{max(yaws):+.0f}°")
    print(f"{'='*94}")
    print(f"{'cue':<12}{'floor':>7}" + "".join(f"{h:>11}" for h in
          ("med Z mm", "sd %", "p95dev %", "max %", "pairs")))
    for name, pairs in CUES.items():
        for floor in (FLOORS if name in ("verticals", "median-10") else [None]):
            zs, np_ = [], []
            for yaw, pitch, roll, kp in rows:
                z, k = depth(kp, rot(yaw, pitch, roll), pairs, FL, floor)
                if z is not None:
                    zs.append(z); np_.append(k)
            if len(zs) < 30:
                continue
            m = st.median(zs)
            dev = [abs(z-m)/m*100 for z in zs]
            print(f"{name:<12}{(f'{floor:.2f}' if floor else '—'):>7}"
                  f"{m:11.1f}{100*st.pstdev(zs)/m:11.2f}{pct(dev,95):11.2f}"
                  f"{max(dev):11.2f}{st.mean(np_):11.2f}")

# Which pairs does the floor actually shed, and when?
print(f"\n{'='*94}")
print("WHAT THE FLOOR SHEDS  (fraction of frames each pair is dropped at floor=0.80)")
print(f"{'='*94}")
NAMES = ["Reye","Leye","nose","Rmouth","Lmouth"]
for clip in ("pitch-extreme", "yaw-hold-max", "still"):
    rows = load(clip)
    if len(rows) < 50:
        continue
    drop = {p: 0 for p in ALLP}
    for yaw, pitch, roll, kp in rows:
        R = rot(yaw, pitch, roll)
        Q = [[sum(R[a][b]*P[b] for b in range(3)) for a in range(3)] for P in FACE]
        for p in ALLP:
            Lp = math.hypot(Q[p[0]][0]-Q[p[1]][0], Q[p[0]][1]-Q[p[1]][1])
            if Lp < 0.80 * REST[p]:
                drop[p] += 1
    shown = [(p, 100*c/len(rows)) for p, c in drop.items() if c]
    print(f"  {clip:<16}" + ("  ".join(f"{NAMES[i]}-{NAMES[j]} {v:.0f}%" for (i,j), v in
          sorted(shown, key=lambda t: -t[1])) or "nothing dropped"))
