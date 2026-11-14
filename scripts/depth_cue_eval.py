#!/usr/bin/env python3
"""Rank candidate depth cues on captured frames — offline, no sim, no rebuild.

    scripts/depth_cue_eval.py [clip-dir ...]      (default: ~/headtrack-bakeoff/yaw-sweep)

Reads `frames.csv` written by HEADTRACK_DUMP_FRAMES (frame_dump.rs): per frame the
5 keypoints in full-resolution pixels plus the tracker's yaw/pitch/roll.

WHY
---
Z is today derived from ONE landmark pair — interpupillary distance — which is the
most yaw-sensitive of the ten pairs the 5-point model offers, and the pair whose far
point occludes first. See docs/DEPTH_CUE_PLAN.md.

Each pair (i,j) gives an independent estimate of the same scalar:

    s_ij = |p_i - p_j|_px / |(R * P_i - R * P_j).xy|_mm      # px per mm

with R rebuilt from the logged rotation and P from FACE_MODEL_MM (pnp.rs). Depth is
focal_px / s. A perfect set of landmarks makes all ten agree; their spread is a
per-frame quality signal, which is exactly what z_gate.rs currently lacks.
"""
import csv, math, statistics as st, sys
from pathlib import Path

FACE_MODEL_MM = [(-31.5,-35.0,0.0),(31.5,-35.0,0.0),(0.0,-5.0,-25.0),
                 (-22.0,45.0,-5.0),(22.0,45.0,-5.0)]
NAME = ["R-eye","L-eye","nose","R-mouth","L-mouth"]
PAIRS = [(i,j) for i in range(5) for j in range(i+1,5)]

# Named candidate cues. Each is a subset of PAIRS aggregated by median.
CUES = {
    "ipd (today)":        [(0,1)],
    "V nose-mouth":       [(2,3),(2,4)],
    "V eye-nose":         [(0,2),(1,2)],
    "V eye-mouth (side)": [(0,3),(1,4)],
    "verticals (4)":      [(0,3),(1,4),(0,4),(1,3)],
    "median-10 (all)":    PAIRS,
}

def rot(yaw_deg, pitch_deg, roll_deg):
    """R = Rz(-roll)*Ry(+yaw)*Rx(-pitch) — the inverse of pnp.rs pose_to_canonical."""
    y, p, r = math.radians(yaw_deg), math.radians(-pitch_deg), math.radians(-roll_deg)
    cy,sy,cp,sp,cr,sr = math.cos(y),math.sin(y),math.cos(p),math.sin(p),math.cos(r),math.sin(r)
    Rx=[[1,0,0],[0,cp,-sp],[0,sp,cp]]
    Ry=[[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz=[[cr,-sr,0],[sr,cr,0],[0,0,1]]
    def mul(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(mul(Rz,Ry),Rx)

def scales(kp, R, floor_px=8.0):
    """Per-pair px-per-mm, skipping pairs whose projected model length is degenerate."""
    Q=[[sum(R[a][b]*P[b] for b in range(3)) for a in range(3)] for P in FACE_MODEL_MM]
    out={}
    for i,j in PAIRS:
        dx,dy = Q[i][0]-Q[j][0], Q[i][1]-Q[j][1]
        Lp = math.hypot(dx,dy)                      # projected model length, mm
        if Lp < 6.0: continue                       # conditioning guard
        dpx = math.dist(kp[i],kp[j])
        if dpx < floor_px: continue                 # occluded / collapsed
        out[(i,j)] = dpx/Lp
    return out

def load(clip):
    rows=[]
    with open(Path(clip)/"frames.csv") as f:
        for r in csv.DictReader(f):
            try:
                kp=[(float(r[f'kp{i}_x']),float(r[f'kp{i}_y'])) for i in range(5)]
                rows.append((float(r['yaw']),float(r['pitch']),float(r['roll']),kp))
            except (ValueError,KeyError): continue
    return rows

def med(v): return st.median(v) if v else float('nan')

def run(clip):
    rows=load(clip)
    if not rows: print(f"{clip}: no usable frames"); return
    print(f"\n{'='*78}\n{Path(clip).name}: {len(rows)} frames, "
          f"yaw {min(r[0] for r in rows):.0f}..{max(r[0] for r in rows):.0f}\n{'='*78}")

    per_frame=[]
    for yaw,pitch,roll,kp in rows:
        s=scales(kp,rot(yaw,pitch,roll))
        if len(s)>=6: per_frame.append((yaw,s))

    # --- is the rotation convention right? all pairs should agree ---
    spread=[ (max(s.values())-min(s.values()))/med(list(s.values()))*100 for _,s in per_frame ]
    print(f"  rotation-convention check — spread of the 10 per-pair scales:")
    print(f"     median {med(spread):.1f}%   (a wrong R would give a large, yaw-dependent spread)")

    # --- candidate cues: stability of the implied depth ---
    BINS=[(0,15),(15,30),(30,45),(45,90)]
    print(f"\n  RELATIVE FRAME-TO-FRAME NOISE OF THE IMPLIED DEPTH (lower is better)")
    hdr=f"  {'cue':22s}"+"".join(f"{f'|yaw|{a}-{b}':>12s}" for a,b in BINS)+f"{'worst/best':>12s}"
    print(hdr); print("  "+"-"*(len(hdr)-2))
    for nm,sel in CUES.items():
        vals=[]
        for a,b in BINS:
            seq=[(k,med([s[p] for p in sel if p in s]))
                 for k,(yaw,s) in enumerate(per_frame) if a<=abs(yaw)<b]
            seq=[(k,v) for k,v in seq if v==v]
            d=[abs(v2-v1)/((v1+v2)/2)*100 for (k1,v1),(k2,v2) in zip(seq,seq[1:]) if k2==k1+1]
            vals.append(med(d) if len(d)>=20 else float('nan'))
        good=[v for v in vals if v==v]
        line=f"  {nm:22s}"+"".join(f"{v:11.3f}%" if v==v else f"{'-':>12s}" for v in vals)
        line+=f"{max(good)/min(good):11.2f}x" if len(good)>1 else f"{'-':>12s}"
        print(line)

    # --- availability: how often is each pair usable at all ---
    print(f"\n  PAIR AVAILABILITY at |yaw|>45 (dropped = occluded or degenerate)")
    hi=[s for yaw,s in per_frame if abs(yaw)>45]
    if hi:
        for p in sorted(PAIRS, key=lambda p: -sum(1 for s in hi if p in s)):
            pct=sum(1 for s in hi if p in s)/len(hi)*100
            if pct<100: print(f"     {NAME[p[0]]:8s}-{NAME[p[1]]:8s} {pct:6.1f}%")
        if all(sum(1 for s in hi if p in s)==len(hi) for p in PAIRS):
            print("     all pairs available on every frame")

if __name__=="__main__":
    clips=sys.argv[1:] or [str(Path.home()/"headtrack-bakeoff"/"yaw-sweep")]
    for c in clips: run(c)
    print()
