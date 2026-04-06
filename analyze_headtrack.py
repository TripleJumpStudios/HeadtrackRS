#!/usr/bin/env python3
"""
headtrack-rs recording analyzer
--------------------------------
Ingests a headtrack-rs diagnostic CSV recording and produces a structured
analysis report covering signal quality, pipeline behaviour, and potential
bugs or regressions.

Usage:
    python analyze_headtrack.py <recording.csv> [--json] [--out <report.txt>]

Flags:
    --json          Also write a machine-readable JSON summary alongside the report
    --out FILE      Write the text report to FILE instead of stdout
"""

import sys
import csv
import math
import json
import argparse
import io
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field, asdict
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

MOTION_AXES    = ["yaw", "pitch", "roll", "x", "y", "z"]
ANGLE_AXES     = ["yaw", "pitch", "roll"]
TRANS_AXES     = ["x", "y", "z"]

# Parameter-change events are worth flagging loudly
INTERESTING_PARAMS = [
    "response-curve:yaw.exponent",  "response-curve:yaw.sensitivity",
    "response-curve:pitch.exponent","response-curve:pitch.sensitivity",
    "response-curve:roll.exponent", "response-curve:roll.sensitivity",
    "one-euro-filter:yaw.min_cutoff","one-euro-filter:yaw.beta",
    "one-euro-filter:pitch.min_cutoff","one-euro-filter:pitch.beta",
    "center:drift_rate","center:still_threshold",
    "prediction:predict_ms",
    "deadzone:yaw","deadzone:pitch","deadzone:roll",
    "deadzone:x","deadzone:y","deadzone:z",
]

FPS_DROP_WARN   = 0.80   # Warn if measured FPS falls below 80 % of stated target
GAP_WARN_MS     = 50     # Flag timestamp gaps > 50 ms (≈ 3 missed frames at 60 fps)
JITTER_WARN     = 2.0    # Warn if per-axis jitter (deg or mm RMS) exceeds this
SLEW_SAFETY     = 0.90   # Flag if actual velocity reaches 90 % of slew limit

# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class AxisStats:
    axis:     str
    count:    int   = 0
    min_val:  float = float("inf")
    max_val:  float = float("-inf")
    mean:     float = 0.0
    std:      float = 0.0
    rms_vel:  float = 0.0   # RMS frame-to-frame velocity
    max_vel:  float = 0.0   # peak frame-to-frame velocity
    jitter:   float = 0.0   # RMS of second derivative (acceleration noise)

@dataclass
class FpsStats:
    target_fps:   float = 0.0
    mean_fps:     float = 0.0
    min_fps:      float = float("inf")
    max_fps:      float = float("-inf")
    std_fps:      float = 0.0
    drop_count:   int   = 0   # frames where fps < target * FPS_DROP_WARN

@dataclass
class TimingStats:
    total_duration_s: float = 0.0
    frame_count:      int   = 0
    mean_dt_ms:       float = 0.0
    max_dt_ms:        float = 0.0
    gap_events:       list  = field(default_factory=list)  # (time_s, dt_ms)

@dataclass
class ParamChange:
    time_s: float
    param:  str
    old_val: str
    new_val: str

@dataclass
class Event:
    time_s:  float
    message: str

@dataclass
class SlewHit:
    time_s:   float
    axis:     str
    velocity: float
    limit:    float
    pct:      float

# ─────────────────────────────────────────────────────────────────────────────
# Parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_recording(path: Path):
    """
    Returns (header_meta, events, rows, param_columns).

    rows is a list of dicts with float values for numeric columns.
    Embedded comment lines are extracted as Event objects.
    """
    raw_lines  = path.read_text(encoding="utf-8").splitlines()
    header_meta = {}
    events      = []
    data_lines  = []
    col_names   = None

    for line in raw_lines:
        stripped = line.strip()
        if not stripped:
            continue

        if stripped.startswith("#"):
            # Could be version header or an embedded event
            body = stripped[1:].strip()
            if body.startswith("headtrack-rs"):
                header_meta["version_line"] = body
                continue
            # Try to parse as  "TIMESTAMP EVENT_TEXT"
            parts = body.split(None, 1)
            if len(parts) == 2:
                try:
                    t = float(parts[0])
                    events.append(Event(time_s=t, message=parts[1]))
                    continue
                except ValueError:
                    pass
            # Generic comment
            header_meta.setdefault("comments", []).append(body)
            continue

        if col_names is None:
            # Use csv.reader so quoted column names (e.g. "median-filter:yaw")
            # are unquoted correctly before we store them.
            col_names = next(csv.reader([stripped]))
            continue

        data_lines.append(stripped)

    if col_names is None:
        raise ValueError("No column header found in recording.")

    # Parse data rows
    rows = []
    for dl in data_lines:
        vals = dl.split(",")
        if len(vals) != len(col_names):
            continue  # malformed row – skip
        row = {}
        for k, v in zip(col_names, vals):
            try:
                row[k] = float(v)
            except ValueError:
                row[k] = v
        rows.append(row)

    param_cols = [c for c in col_names if c not in (["time_s"] + MOTION_AXES + ["fps"])]
    return header_meta, events, rows, param_cols


# ─────────────────────────────────────────────────────────────────────────────
# Analysis helpers
# ─────────────────────────────────────────────────────────────────────────────

def _mean(vals):
    return sum(vals) / len(vals) if vals else 0.0

def _std(vals):
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))

def _rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0


def compute_axis_stats(rows: list, axis: str) -> AxisStats:
    vals = [r[axis] for r in rows if axis in r]
    if not vals:
        return AxisStats(axis=axis)

    times = [r["time_s"] for r in rows if axis in r]
    dts   = [times[i+1] - times[i] for i in range(len(times)-1)]

    # Frame-to-frame velocities (units/s)
    velocities = []
    for i in range(len(vals)-1):
        dt = dts[i] if dts[i] > 0 else 1e-6
        velocities.append((vals[i+1] - vals[i]) / dt)

    # Acceleration (jitter = noise in velocity)
    accel = []
    for i in range(len(velocities)-1):
        dt = dts[i] if dts[i] > 0 else 1e-6
        accel.append((velocities[i+1] - velocities[i]) / dt)

    m  = _mean(vals)
    s  = AxisStats(
        axis    = axis,
        count   = len(vals),
        min_val = min(vals),
        max_val = max(vals),
        mean    = m,
        std     = _std(vals),
        rms_vel = _rms(velocities),
        max_vel = max(abs(v) for v in velocities) if velocities else 0.0,
        jitter  = _rms(accel),
    )
    return s


def compute_fps_stats(rows: list) -> FpsStats:
    fpss = [r["fps"] for r in rows if "fps" in r]
    if not fpss:
        return FpsStats()

    target = _mean(fpss[:10])   # assume first 10 frames reflect the target
    drops  = sum(1 for f in fpss if f < target * FPS_DROP_WARN)

    return FpsStats(
        target_fps = target,
        mean_fps   = _mean(fpss),
        min_fps    = min(fpss),
        max_fps    = max(fpss),
        std_fps    = _std(fpss),
        drop_count = drops,
    )


def compute_timing_stats(rows: list) -> TimingStats:
    times = [r["time_s"] for r in rows]
    if len(times) < 2:
        return TimingStats(frame_count=len(times))

    dts     = [(times[i+1] - times[i]) * 1000 for i in range(len(times)-1)]
    gaps    = [(times[i+1], dts[i]) for i, dt in enumerate(dts) if dt > GAP_WARN_MS]

    return TimingStats(
        total_duration_s = times[-1] - times[0],
        frame_count      = len(times),
        mean_dt_ms       = _mean(dts),
        max_dt_ms        = max(dts),
        gap_events       = gaps,
    )


def detect_param_changes(rows: list, param_cols: list) -> list[ParamChange]:
    changes = []
    if not rows:
        return changes

    prev = {k: rows[0].get(k) for k in param_cols}
    for row in rows[1:]:
        t = row["time_s"]
        for k in param_cols:
            v = row.get(k)
            if v != prev[k]:
                changes.append(ParamChange(
                    time_s  = t,
                    param   = k,
                    old_val = str(prev[k]),
                    new_val = str(v),
                ))
                prev[k] = v
    return changes


def compute_param_dwell(
    rows: list,
    param_cols: list,
    param_changes: list[ParamChange],
    total_duration: float,
) -> dict:
    """
    For every parameter that changed during the recording, compute per-value
    dwell statistics:
        total_s   – cumulative seconds spent at this value (across all visits)
        visits    – number of separate times the user landed on this value
        pct       – total_s / total_duration * 100

    Returns a dict keyed by param name → sorted list of
    { value, total_s, visits, pct } dicts, highest dwell first.

    Only parameters that actually changed are included (static params are
    uninteresting from a tuning perspective).
    """
    if not rows or total_duration <= 0:
        return {}

    # Build a timeline of (time_s, value) for each param that changed.
    # Start value comes from the first row; end sentinel is total_duration.
    changed_params = {pc.param for pc in param_changes}
    result = {}

    for param in changed_params:
        # Build ordered list of (time_s, value) transition points
        transitions = [(rows[0]["time_s"], rows[0].get(param))]
        for pc in param_changes:
            if pc.param == param:
                transitions.append((pc.time_s, float(pc.new_val)
                                     if pc.new_val not in ("None", "") else pc.new_val))
        transitions.append((rows[-1]["time_s"] + (rows[-1]["time_s"] - rows[-2]["time_s"]
                             if len(rows) > 1 else 0), None))  # sentinel

        # Accumulate dwell per value
        dwell: dict[str, dict] = {}  # str(value) → {total_s, visits, value}

        prev_t, prev_v = transitions[0]
        for (t, v) in transitions[1:]:
            dt   = t - prev_t
            key  = str(prev_v)
            if key not in dwell:
                dwell[key] = {"value": prev_v, "total_s": 0.0, "visits": 0}
            dwell[key]["total_s"] += dt
            dwell[key]["visits"]  += 1
            prev_t, prev_v = t, v

        # Sort by total dwell time descending, attach pct
        ranked = sorted(dwell.values(), key=lambda d: -d["total_s"])
        for entry in ranked:
            entry["pct"] = entry["total_s"] / total_duration * 100

        result[param] = ranked

    return result


def detect_slew_hits(rows: list, axis_stats: dict) -> list[SlewHit]:
    """
    Compare actual per-frame velocity against the slew-limit parameters.
    Flags when velocity reaches >= SLEW_SAFETY * limit.
    """
    hits = []
    for i in range(len(rows)-1):
        r    = rows[i]
        r2   = rows[i+1]
        dt   = r2["time_s"] - r["time_s"]
        if dt <= 0:
            continue
        for axis in MOTION_AXES:
            limit_key = f"slew-limit:{axis}.max_rate"
            if limit_key not in r:
                continue
            limit = r.get(limit_key, 0.0)
            if limit <= 0:
                continue
            vel = abs((r2[axis] - r[axis]) / dt)
            pct = vel / limit
            if pct >= SLEW_SAFETY:
                hits.append(SlewHit(
                    time_s   = r["time_s"],
                    axis     = axis,
                    velocity = vel,
                    limit    = limit,
                    pct      = pct,
                ))
    return hits


def detect_cross_axis_coupling(rows: list, threshold=0.3) -> list[dict]:
    """
    Pearson correlation between yaw velocity and each other axis velocity.
    Strong correlation that matches a configured cross-axis coefficient may
    indicate the coupling is working; unexpected correlation is a bug.
    """
    if len(rows) < 3:
        return []

    times = [r["time_s"] for r in rows]

    def velocities(axis):
        vals = [r[axis] for r in rows]
        dts  = [max(times[i+1]-times[i], 1e-6) for i in range(len(times)-1)]
        return [(vals[i+1]-vals[i])/dts[i] for i in range(len(vals)-1)]

    def pearson(a, b):
        n  = len(a)
        ma, mb = _mean(a), _mean(b)
        num = sum((a[i]-ma)*(b[i]-mb) for i in range(n))
        da  = math.sqrt(sum((v-ma)**2 for v in a))
        db  = math.sqrt(sum((v-mb)**2 for v in b))
        if da == 0 or db == 0:
            return 0.0
        return num / (da * db)

    vels   = {ax: velocities(ax) for ax in MOTION_AXES}
    pairs  = []
    axes   = MOTION_AXES
    for i, a in enumerate(axes):
        for b in axes[i+1:]:
            r = pearson(vels[a], vels[b])
            if abs(r) >= threshold:
                pairs.append({"axis_a": a, "axis_b": b, "correlation": round(r, 4)})
    return pairs


# ─────────────────────────────────────────────────────────────────────────────
# Report rendering
# ─────────────────────────────────────────────────────────────────────────────

def banner(title: str, width=70) -> str:
    bar = "─" * width
    return f"\n{bar}\n  {title}\n{bar}"

def flag(level: str) -> str:
    return {"OK": "✓", "WARN": "⚠", "ERR": "✗"}.get(level, "?")


def render_report(
    path:          Path,
    header_meta:   dict,
    events:        list[Event],
    rows:          list,
    param_cols:    list,
    axis_stats:    dict[str, AxisStats],
    fps_stats:     FpsStats,
    timing_stats:  TimingStats,
    param_changes: list[ParamChange],
    param_dwell:   dict,
    slew_hits:     list[SlewHit],
    cross_axis:    list[dict],
) -> str:
    out = io.StringIO()
    w   = lambda *a, **kw: print(*a, **kw, file=out)

    # ── Header ────────────────────────────────────────────────────────────
    w(banner("headtrack-rs Recording Analysis"))
    w(f"  File      : {path.name}")
    w(f"  Version   : {header_meta.get('version_line', 'unknown')}")
    w(f"  Frames    : {timing_stats.frame_count}")
    w(f"  Duration  : {timing_stats.total_duration_s:.3f} s")
    w(f"  Events    : {len(events)}")
    if events:
        for ev in events:
            w(f"    [{ev.time_s:8.4f}s]  {ev.message}")

    # ── Timing & FPS ──────────────────────────────────────────────────────
    w(banner("Timing & Frame Rate"))

    fps_ok = fps_stats.drop_count == 0 and fps_stats.std_fps < 2.0
    fps_lv = "OK" if fps_ok else ("WARN" if fps_stats.drop_count < 5 else "ERR")
    w(f"  {flag(fps_lv)} Target FPS      : {fps_stats.target_fps:.1f}")
    w(f"  {flag(fps_lv)} Mean FPS        : {fps_stats.mean_fps:.2f}  (σ={fps_stats.std_fps:.2f})")
    w(f"     Min FPS       : {fps_stats.min_fps:.1f}   Max FPS: {fps_stats.max_fps:.1f}")
    w(f"     FPS drops     : {fps_stats.drop_count} frames below {FPS_DROP_WARN*100:.0f}% of target")

    dt_ok = len(timing_stats.gap_events) == 0
    dt_lv = "OK" if dt_ok else "WARN"
    w(f"  {flag(dt_lv)} Mean frame dt   : {timing_stats.mean_dt_ms:.2f} ms")
    w(f"     Max frame dt  : {timing_stats.max_dt_ms:.2f} ms  (warn threshold: {GAP_WARN_MS} ms)")
    if timing_stats.gap_events:
        w(f"  ⚠  Timing gaps detected ({len(timing_stats.gap_events)} events):")
        for (t, dt) in timing_stats.gap_events[:10]:
            w(f"       t={t:.4f}s  gap={dt:.1f} ms")
        if len(timing_stats.gap_events) > 10:
            w(f"       … and {len(timing_stats.gap_events)-10} more")

    # ── Motion Axis Stats ─────────────────────────────────────────────────
    w(banner("Motion Axis Statistics"))
    hdr = f"  {'Axis':>6}  {'Min':>9}  {'Max':>9}  {'Mean':>9}  {'Std':>8}  {'RMS vel':>9}  {'Peak vel':>9}  Jitter"
    w(hdr)
    w("  " + "─" * (len(hdr) - 2))
    for ax in MOTION_AXES:
        unit = "°" if ax in ANGLE_AXES else "mm"
        s    = axis_stats[ax]
        rng  = s.max_val - s.min_val
        j_lv = "⚠ " if s.jitter > JITTER_WARN else "  "
        w(f"  {ax:>6}  {s.min_val:>8.3f}{unit}  {s.max_val:>8.3f}{unit}  "
          f"{s.mean:>8.3f}{unit}  {s.std:>7.3f}  "
          f"{s.rms_vel:>8.2f}/s  {s.max_vel:>8.2f}/s  {j_lv}{s.jitter:.3f}")

    w("")
    w("  Range of motion summary:")
    for ax in MOTION_AXES:
        unit = "°" if ax in ANGLE_AXES else "mm"
        s    = axis_stats[ax]
        rng  = s.max_val - s.min_val
        w(f"    {ax:>6}: {rng:.3f}{unit} span")

    # ── Parameter Changes ─────────────────────────────────────────────────
    w(banner("Pipeline Parameter Changes"))
    if not param_changes:
        w(f"  {flag('OK')} No parameter changes detected during recording.")
    else:
        by_param     = defaultdict(list)
        for pc in param_changes:
            by_param[pc.param].append(pc)

        interesting = [p for p in param_changes if p.param in INTERESTING_PARAMS]
        other       = [p for p in param_changes if p.param not in INTERESTING_PARAMS]

        w(f"  ⚠  {len(param_changes)} parameter change(s) detected across {len(by_param)} parameter(s).")

        if interesting:
            w(f"\n  Notable changes ({len(interesting)}):")
            for pc in interesting:
                w(f"    [{pc.time_s:8.4f}s]  {pc.param}:  {pc.old_val}  →  {pc.new_val}")

        if other:
            w(f"\n  Other changes ({len(other)}):")
            for pc in other:
                w(f"    [{pc.time_s:8.4f}s]  {pc.param}: {pc.old_val} → {pc.new_val}")

    # ── Parameter Dwell / Preferred Values ───────────────────────────────
    w(banner("Parameter Dwell — Preferred & Revisited Values"))
    if not param_dwell:
        w(f"  {flag('OK')} No parameter changes to analyse.")
    else:
        w("  Shows how long each value was held (cumulative across all visits).")
        w("  Values the user returned to multiple times are marked with ↩.")
        w("")
        for param, entries in sorted(param_dwell.items()):
            w(f"  {param}")
            # Table header
            w(f"    {'Value':>12}   {'Total time':>10}   {'Visits':>6}   {'% of session':>13}   Note")
            w(f"    {'─'*12}   {'─'*10}   {'─'*6}   {'─'*13}   {'─'*20}")
            for i, e in enumerate(entries):
                val_s   = str(e["value"])
                note    = ""
                if i == 0:
                    note = "◀ most time here"
                if e["visits"] > 1:
                    note = (note + "  ↩ revisited" if note else "↩ revisited") + f" ×{e['visits']}"
                w(f"    {val_s:>12}   {e['total_s']:>9.3f}s   {e['visits']:>6}   {e['pct']:>12.1f}%   {note}")
            w("")

    # ── Slew Limit Pressure ───────────────────────────────────────────────
    w(banner("Slew Limit Analysis"))
    if not slew_hits:
        w(f"  {flag('OK')} No axes approaching slew limits (< {SLEW_SAFETY*100:.0f}% threshold).")
    else:
        # Summarise by axis
        by_axis = defaultdict(list)
        for h in slew_hits:
            by_axis[h.axis].append(h)

        w(f"  ⚠  {len(slew_hits)} slew-limit proximity events on {len(by_axis)} axis/axes.")
        for ax, hits in by_axis.items():
            peak = max(h.pct for h in hits)
            w(f"    {ax:>6}:  {len(hits):4d} events  peak={peak*100:.1f}%  limit={hits[0].limit}/s")
            # Show worst 3
            worst = sorted(hits, key=lambda h: -h.pct)[:3]
            for h in worst:
                w(f"             t={h.time_s:.4f}s  vel={h.velocity:.2f}/s ({h.pct*100:.1f}% of limit)")

    # ── Cross-Axis Correlation ─────────────────────────────────────────────
    w(banner("Cross-Axis Velocity Correlation"))
    if not cross_axis:
        w(f"  {flag('OK')} No strong inter-axis velocity correlations (< {threshold*100:.0f}% threshold).")
    else:
        w(f"  The following axis pairs show correlation ≥ 0.3:")
        for p in cross_axis:
            r  = p["correlation"]
            lv = "⚠" if abs(r) > 0.6 else "ℹ"
            w(f"    {lv} {p['axis_a']:>6} ↔ {p['axis_b']:<6}  r = {r:+.4f}")
        w("")
        w("  Note: configured cross-axis coupling may intentionally produce some")
        w("  correlation. Compare against cross-axis:* parameter values.")

    # ── Potential Bug Flags ───────────────────────────────────────────────
    w(banner("Potential Bug / Regression Flags"))
    bugs_found = False

    # Stuck axes: an axis that never moves
    for ax in MOTION_AXES:
        s = axis_stats[ax]
        if s.std < 0.001 and s.count > 10:
            w(f"  ✗ STUCK AXIS: {ax} has essentially no variance (std={s.std:.6f}). "
              f"Check if the axis is masked, deadzoned, or the sensor is dead.")
            bugs_found = True

    # Axes that are masked but still moving
    for ax in MOTION_AXES:
        mask_key = f"axis-mask:{ax}"
        # Check if mask=0 appears in any row
        masked_rows    = [r for r in rows if r.get(mask_key, 1) == 0.0]
        moving_despite = [r for r in masked_rows if abs(r.get(ax, 0)) > 1.0]
        if moving_despite:
            w(f"  ⚠ MASKED+NONZERO: {ax} is axis-masked but shows non-zero output "
              f"({len(moving_despite)} frames). Possible mask bypass.")
            bugs_found = True

    # Large z-drift while user likely stationary early on
    z_early = [r["z"] for r in rows[:30]] if len(rows) > 30 else []
    if z_early:
        z_range_early = max(z_early) - min(z_early)
        if z_range_early > 10:
            w(f"  ⚠ EARLY Z DRIFT: z axis moved {z_range_early:.2f} mm in first 30 frames. "
              f"Could indicate cold-start convergence issue or tracking instability.")
            bugs_found = True

    # FPS jitter
    if fps_stats.std_fps > 5.0:
        w(f"  ⚠ FPS INSTABILITY: σ={fps_stats.std_fps:.2f} fps is high. "
          f"Irregular timing may cause filter artifacts.")
        bugs_found = True

    # Timing gaps
    if len(timing_stats.gap_events) > 0:
        w(f"  ⚠ TIMING GAPS: {len(timing_stats.gap_events)} gaps > {GAP_WARN_MS} ms detected. "
          f"Largest: {max(dt for _,dt in timing_stats.gap_events):.1f} ms. "
          f"These can cause velocity spikes after a gap.")
        bugs_found = True

    # High jitter axes
    for ax in MOTION_AXES:
        s = axis_stats[ax]
        if s.jitter > JITTER_WARN * 3:
            unit = "°/s²" if ax in ANGLE_AXES else "mm/s²"
            w(f"  ⚠ HIGH JITTER on {ax}: {s.jitter:.3f} {unit}. "
              f"One-Euro or median filter may need tuning.")
            bugs_found = True

    # Param changes happening during what appears to be motion
    motion_during_change = []
    for pc in param_changes:
        if pc.param in INTERESTING_PARAMS:
            # Find nearby rows
            near = [r for r in rows if abs(r["time_s"] - pc.time_s) < 0.1]
            for r in near:
                vel = sum(abs(r.get(ax, 0)) for ax in ["yaw", "pitch", "roll"])
                if vel > 5.0:
                    motion_during_change.append(pc)
                    break
    if motion_during_change:
        w(f"  ⚠ PARAM CHANGE DURING MOTION: {len(motion_during_change)} notable parameter(s) changed")
        w(f"    while the head was moving. This can cause transient artifacts.")
        for pc in motion_during_change:
            w(f"       [{pc.time_s:.4f}s] {pc.param}: {pc.old_val} → {pc.new_val}")
        bugs_found = True

    if not bugs_found:
        w(f"  {flag('OK')} No obvious bugs or regressions detected.")

    # ── Summary ───────────────────────────────────────────────────────────
    w(banner("Summary"))
    total_warnings = (
        fps_stats.drop_count +
        len(timing_stats.gap_events) +
        len(param_changes) +
        len(slew_hits) +
        len([p for p in cross_axis if abs(p["correlation"]) > 0.6]) +
        len(motion_during_change)
    )
    if total_warnings == 0:
        w("  ✓ Recording looks healthy. No significant anomalies found.")
    else:
        w(f"  {total_warnings} total warning condition(s) found across all checks.")
        w("  Review the flagged sections above for details.")

    w("\n" + "─" * 70)
    return out.getvalue()


def build_json_summary(
    path, header_meta, events, axis_stats, fps_stats,
    timing_stats, param_changes, param_dwell, slew_hits, cross_axis
):
    def _safe(obj):
        if isinstance(obj, float) and (math.isinf(obj) or math.isnan(obj)):
            return None
        return obj

    return {
        "file": str(path.name),
        "version": header_meta.get("version_line"),
        "events": [asdict(e) for e in events],
        "timing": {
            "duration_s":    _safe(timing_stats.total_duration_s),
            "frame_count":   timing_stats.frame_count,
            "mean_dt_ms":    _safe(timing_stats.mean_dt_ms),
            "max_dt_ms":     _safe(timing_stats.max_dt_ms),
            "gap_count":     len(timing_stats.gap_events),
        },
        "fps": {
            "target":     _safe(fps_stats.target_fps),
            "mean":       _safe(fps_stats.mean_fps),
            "min":        _safe(fps_stats.min_fps),
            "max":        _safe(fps_stats.max_fps),
            "std":        _safe(fps_stats.std_fps),
            "drop_count": fps_stats.drop_count,
        },
        "axes": {
            ax: {
                "min":      _safe(s.min_val),
                "max":      _safe(s.max_val),
                "mean":     _safe(s.mean),
                "std":      _safe(s.std),
                "rms_vel":  _safe(s.rms_vel),
                "max_vel":  _safe(s.max_vel),
                "jitter":   _safe(s.jitter),
            }
            for ax, s in axis_stats.items()
        },
        "param_changes": [asdict(pc) for pc in param_changes],
        "param_dwell": {
            param: [
                {"value": e["value"], "total_s": round(e["total_s"], 4),
                 "visits": e["visits"], "pct": round(e["pct"], 2)}
                for e in entries
            ]
            for param, entries in param_dwell.items()
        },
        "slew_hits_by_axis": {
            ax: len([h for h in slew_hits if h.axis == ax])
            for ax in MOTION_AXES
        },
        "cross_axis_correlations": cross_axis,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Analyze a headtrack-rs diagnostic recording.")
    ap.add_argument("csv", help="Path to the .csv recording file")
    ap.add_argument("--json", action="store_true", help="Also write a JSON summary")
    ap.add_argument("--out", metavar="FILE", help="Write report to FILE instead of stdout")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"Error: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {path.name} …", file=sys.stderr)
    header_meta, events, rows, param_cols = parse_recording(path)
    print(f"  {len(rows)} data rows, {len(events)} events, {len(param_cols)} parameter columns",
          file=sys.stderr)

    print("Running analysis …", file=sys.stderr)
    axis_stats    = {ax: compute_axis_stats(rows, ax) for ax in MOTION_AXES}
    fps_stats     = compute_fps_stats(rows)
    timing_stats  = compute_timing_stats(rows)
    param_changes = detect_param_changes(rows, param_cols)
    param_dwell   = compute_param_dwell(rows, param_cols, param_changes, timing_stats.total_duration_s)
    slew_hits     = detect_slew_hits(rows, axis_stats)
    cross_axis    = detect_cross_axis_coupling(rows, threshold=0.3)

    report = render_report(
        path, header_meta, events, rows, param_cols,
        axis_stats, fps_stats, timing_stats,
        param_changes, param_dwell, slew_hits, cross_axis,
    )

    if args.out:
        Path(args.out).write_text(report, encoding="utf-8")
        print(f"Report written to {args.out}", file=sys.stderr)
    else:
        print(report)

    if args.json:
        summary = build_json_summary(
            path, header_meta, events, axis_stats,
            fps_stats, timing_stats, param_changes, param_dwell, slew_hits, cross_axis,
        )
        json_path = path.with_suffix(".analysis.json")
        json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"JSON summary written to {json_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
