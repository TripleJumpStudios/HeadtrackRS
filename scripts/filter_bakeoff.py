#!/usr/bin/env python3
"""Offline filter bake-off — replay captured RAW pose clips through candidate
smoothers and score them.

This harness is what selected Accela over the old One-Euro + Kalman-prediction
stack; the in-sim FB2 A/B then confirmed it, and Snow Leopard 2 deleted the
legacy stack from the Rust tree entirely (see docs/MSFS_TICK_INVESTIGATION.md).

**The One-Euro and Kalman classes below are kept deliberately.** They are
faithful ports of implementations that no longer exist in Rust, and they are
what makes the comparison reproducible — they demonstrate the "escalation"
behaviour and let you A/B its causes (decel_boost, prediction) against Accela
and EMA candidates. Treat them as a frozen historical reference, not as a
description of what ships. `Accela` here still mirrors the production
`AccelaStage`, and that correspondence must be maintained.

For replaying a clip through the *actual* shipping pipeline rather than these
ports, use `cargo run -p headtrack-core --example replay`.

Usage:
    python3 scripts/filter_bakeoff.py ~/headtrack-dataset/*.csv
    python3 scripts/filter_bakeoff.py ~/headtrack-dataset/shaky.csv --axes yaw,pitch,z

Input CSV: header with timestamp_us,dt_s,yaw,pitch,roll,x,y,z (raw, pre-pipeline).
"""
import csv, sys, math, os, argparse

AXES = ["yaw", "pitch", "roll", "x", "y", "z"]

# ── candidate filters (per-axis, stateful) ──────────────────────────────────

class Passthrough:
    def reset(self): pass
    def step(self, x, dt): return x

class Ema:
    """Critically-damped exponential: alpha = dt/(dt+tau)."""
    def __init__(self, tau=0.12): self.tau = tau; self.y = None
    def reset(self): self.y = None
    def step(self, x, dt):
        if self.y is None: self.y = x; return x
        a = dt / (dt + self.tau); self.y += a * (x - self.y); return self.y

class OneEuro:
    """Port of one_euro.rs (cutoff = min_cutoff + beta*|dx_hat|)."""
    def __init__(self, min_cutoff=0.01, beta=0.03, d_cutoff=1.0):
        self.mc, self.beta, self.dc = min_cutoff, beta, d_cutoff
        self.x_prev = None; self.dx_prev = 0.0
    def reset(self): self.x_prev = None; self.dx_prev = 0.0
    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * max(cutoff, 0.001)); return dt / (dt + tau)
    def step(self, x, dt):
        if self.x_prev is None: self.x_prev = x; return x
        dx_raw = (x - self.x_prev) / max(dt, 1e-4)
        ad = self._alpha(self.dc, dt)
        dx_hat = ad * dx_raw + (1 - ad) * self.dx_prev
        cutoff = self.mc + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev = x_hat; self.dx_prev = dx_hat
        return x_hat

class Kalman:
    """Port of kalman.rs: constant-velocity + adaptive-Q decel_boost + predict."""
    def __init__(self, q=2.5, r=10.0, predict_ms=10.0, decel_boost_max=3.0):
        self.q, self.r = q, max(r, 1e-3); self.predict_s = predict_ms / 1000.0
        self.boost_max = decel_boost_max; self.reset()
    def reset(self):
        self.x = 0.0; self.v = 0.0; self.p00 = 1000.0; self.p01 = 0.0
        self.p11 = 1000.0; self.init = False
    def step(self, z, dt):
        if not self.init:
            self.x = z; self.v = 0.0; self.init = True; return z
        dt = max(dt, 1e-3)
        xpt = self.x + self.v * dt
        innov_mag = abs(z - xpt); speed = abs(self.v)
        boost = 0.0
        if speed > 0.1 and self.boost_max > 0:
            opposing = (self.v >= 0) != ((z - xpt) >= 0)
            if opposing:
                boost = min(innov_mag / (speed * dt + 0.01), self.boost_max)
        q = self.q * (1.0 + boost) * dt
        p00 = self.p00 + 2*dt*self.p01 + dt*dt*self.p11 + q
        p01 = self.p01 + dt*self.p11
        p11 = self.p11 + q
        innov = z - xpt; s = p00 + self.r
        k0 = p00 / s; k1 = p01 / s
        self.x = xpt + k0*innov; self.v = self.v + k1*innov
        self.p00 = (1-k0)*p00; self.p01 = (1-k0)*p01; self.p11 = p11 - k1*p01
        return self.x + self.v * self.predict_s

class Accela:
    """opentrack-style: deadzone, nonlinear gain on displacement, dt-integrated.
    No velocity state / prediction → structurally cannot escalate.
    gain(|d|) approximates the Accela spline as a power curve."""
    def __init__(self, deadzone=0.2, smoothing=1.5, exponent=2.0, max_gain=600.0):
        self.dz, self.sm, self.exp, self.maxg = deadzone, smoothing, exponent, max_gain
        self.y = None
    def reset(self): self.y = None
    def step(self, x, dt):
        if self.y is None: self.y = x; return x
        d = x - self.y
        ad = abs(d)
        if ad <= self.dz:
            return self.y
        d -= math.copysign(self.dz, d); ad = abs(d)
        xn = ad / self.sm                      # normalized displacement
        gain = min(xn ** self.exp, self.maxg)  # spline ≈ power curve (deg/s)
        step = math.copysign(gain, d) * dt
        if abs(step) > ad:                     # never overshoot the input
            step = d
        self.y += step
        return self.y

# Candidate set: name -> factory(per axis). Translation axes (x,y,z) get scaled
# variants (mm not deg) where the param is dimensional.
def candidates():
    return {
        "raw":            lambda rot: Passthrough(),
        "ema":            lambda rot: Ema(tau=0.12),
        "1euro":          lambda rot: OneEuro(0.01, 0.03) if rot else OneEuro(0.01, 0.03),
        "kalman_full":    lambda rot: Kalman(1.2, 0.7, 10.0, 3.0) if rot else Kalman(2.5, 10.0, 10.0, 3.0),
        "kalman_noboost": lambda rot: Kalman(1.2, 0.7, 10.0, 0.0) if rot else Kalman(2.5, 10.0, 10.0, 0.0),
        "kalman_nopred_noboost": lambda rot: Kalman(1.2, 0.7, 0.0, 0.0) if rot else Kalman(2.5, 10.0, 0.0, 0.0),
        "accela_heavy":   lambda rot: Accela(0.2, 1.5, 2.0) if rot else Accela(1.0, 8.0, 2.0),
        "accela_med":     lambda rot: Accela(0.15, 0.7, 1.5) if rot else Accela(0.6, 3.0, 1.5),
        "accela_light":   lambda rot: Accela(0.10, 0.35, 1.3) if rot else Accela(0.4, 1.5, 1.3),
    }

# ── metrics ─────────────────────────────────────────────────────────────────

def sd(v):
    n = len(v)
    if n < 2: return 0.0
    m = sum(v)/n; return (sum((x-m)**2 for x in v)/n) ** 0.5

def run_filter(make, raw, dts, rot):
    f = make(rot); return [f.step(x, dt) for x, dt in zip(raw, dts)]

def velocity(sig, dts):
    return [0.0] + [(sig[i]-sig[i-1])/max(dts[i],1e-4) for i in range(1, len(sig))]

def rest_jitter(filt, raw, dts):
    """sd of filtered output over the calmest 40% of frames (by raw speed)."""
    v = [abs(x) for x in velocity(raw, dts)]
    if len(v) < 10: return sd(filt)
    thr = sorted(v)[int(len(v)*0.4)]
    calm = [filt[i] for i in range(len(filt)) if v[i] <= thr]
    return sd(calm)

def jerk_rms(filt, dts):
    """RMS of acceleration (2nd difference) — a shakiness proxy."""
    acc = []
    for i in range(2, len(filt)):
        dt = max(dts[i], 1e-4)
        acc.append((filt[i] - 2*filt[i-1] + filt[i-2]) / (dt*dt))
    if not acc: return 0.0
    return (sum(a*a for a in acc)/len(acc)) ** 0.5

def lag_ms(filt, raw, dts):
    """Cross-correlation lag (filtered trails raw); ms. Crude integer-sample."""
    n = len(raw); mean_dt = sum(dts)/max(n,1)
    rm = sum(raw)/n; fm = sum(filt)/n
    rc = [x-rm for x in raw]; fc = [x-fm for x in filt]
    best_lag, best = 0, -1e18
    for lag in range(0, min(45, n//4)):
        s = sum(rc[i-lag]*fc[i] for i in range(lag, n))
        if s > best: best, best_lag = s, lag
    return best_lag * mean_dt * 1000.0

def track_rms(filt, raw):
    return (sum((f-r)**2 for f, r in zip(filt, raw))/len(raw)) ** 0.5

# ── main ────────────────────────────────────────────────────────────────────

def load(path):
    cols = {k: [] for k in ["t"] + AXES}
    with open(path) as fh:
        # The try/except below already drops mid-file `# ts_us=...` event
        # markers, but a comment on the FIRST line is different: DictReader
        # would take it as the column names. Raw captures now open with a
        # `# detector=...` provenance stamp, so filter before parsing.
        for row in csv.DictReader(ln for ln in fh if not ln.lstrip().startswith("#")):
            try:
                vals = {a: float(row[a]) for a in AXES}; t = float(row["timestamp_us"])/1e6
            except (ValueError, KeyError):
                continue
            cols["t"].append(t)
            for a in AXES: cols[a].append(vals[a])
    n = len(cols["t"])
    dts = [1/30.0] + [max(cols["t"][i]-cols["t"][i-1], 1e-4) for i in range(1, n)]
    # trim warm-up
    SKIP = 30
    for k in cols: cols[k] = cols[k][SKIP:]
    dts = dts[SKIP:]
    return cols, dts

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clips", nargs="+")
    ap.add_argument("--axes", default="yaw,pitch,z")
    args = ap.parse_args()
    axes = args.axes.split(",")
    cands = candidates()

    for clip in args.clips:
        name = os.path.basename(clip)
        try:
            cols, dts = load(clip)
        except FileNotFoundError:
            print(f"\n[{name}] MISSING"); continue
        n = len(cols["yaw"])
        if n < 30:
            print(f"\n[{name}] too short ({n})"); continue
        print(f"\n=== {name}  ({n} frames, {cols['t'][-1]-cols['t'][0]:.1f}s) ===")
        for ax in axes:
            rot = ax in ("yaw", "pitch", "roll")
            raw = cols[ax]
            print(f"  -- {ax} --   {'filter':22s} {'rest_jit':>9} {'jerk_rms':>10} {'lag_ms':>7} {'track_rms':>9}")
            for cname, make in cands.items():
                filt = run_filter(make, raw, dts, rot)
                print(f"  {'':10s}   {cname:22s} {rest_jitter(filt,raw,dts):9.3f} "
                      f"{jerk_rms(filt,dts):10.1f} {lag_ms(filt,raw,dts):7.0f} {track_rms(filt,raw):9.3f}")

if __name__ == "__main__":
    main()
