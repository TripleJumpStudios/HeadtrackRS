#!/usr/bin/env python3
"""Derive a raw (pre-pipeline) pose CSV from a legacy calibration capture.

Bootstrap for the offline filter workbench: the calib capture
(`TesterCSVData/headtrack_calib_*.csv`) stores per-frame camera intrinsics, the
legacy 5-point PnP rotation matrix R and translation t, plus the 106 landmarks.
This script extracts (R, t) and converts each frame to the canonical head pose
(yaw, pitch, roll, x, y, z) using the *exact* math of
`pose_to_canonical()` in crates/headtrack-input-neuralnet/src/pnp.rs, so the
resulting CSV is what the engine's pipeline would have received as RawPose.

The output feeds `cargo run -p headtrack-core --example replay`.

No new capture is needed to start tuning. Once the engine's HEADTRACK_DUMP_RAW
tap is used for a fresh session, that CSV can feed replay directly and this
bootstrap converter is no longer required.

Usage:
    scripts/raw_from_calib.py TesterCSVData/headtrack_calib_1780100357.csv \
        -o /tmp/headtrack_raw_pose.csv [--fps 30]
"""
import argparse
import csv
import math

TO_DEG = 180.0 / math.pi
PITCH_SIGN = -1.0
YAW_SIGN = 1.0
ROLL_SIGN = -1.0


def pose_to_canonical(r00, r01, r02, r10, r11, r12, r20, r21, r22, tx, ty, tz):
    """Port of pnp.rs::pose_to_canonical (ZYX Tait-Bryan + canonical signs)."""
    sy = math.sqrt(r00 * r00 + r10 * r10)
    singular = sy < 1e-6
    if singular:
        pitch_rad = math.atan2(-r12, r11)
        roll_rad = 0.0
    else:
        pitch_rad = math.atan2(r21, r22)
        roll_rad = math.atan2(r10, r00)
    yaw_rad = math.atan2(-r20, sy)

    yaw = YAW_SIGN * yaw_rad * TO_DEG
    pitch = PITCH_SIGN * pitch_rad * TO_DEG
    roll = ROLL_SIGN * roll_rad * TO_DEG
    # Camera frame (X right, Y down, Z into-scene) -> canonical (X right, Y up,
    # Z forward = toward camera).
    return yaw, pitch, roll, tx, -ty, -tz


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("calib_csv", help="input headtrack_calib_*.csv")
    ap.add_argument("-o", "--out", default="/tmp/headtrack_raw_pose.csv")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="nominal capture rate for synthesised timestamps/dt")
    args = ap.parse_args()

    dt_s = 1.0 / args.fps
    dt_us = int(round(1_000_000.0 / args.fps))

    with open(args.calib_csv, newline="") as fin, \
            open(args.out, "w", newline="") as fout:
        reader = csv.DictReader(fin)
        writer = csv.writer(fout)
        writer.writerow(["timestamp_us", "dt_s", "yaw", "pitch", "roll", "x", "y", "z"])

        n = 0
        for i, row in enumerate(reader):
            vals = pose_to_canonical(
                float(row["r00"]), float(row["r01"]), float(row["r02"]),
                float(row["r10"]), float(row["r11"]), float(row["r12"]),
                float(row["r20"]), float(row["r21"]), float(row["r22"]),
                float(row["tx"]), float(row["ty"]), float(row["tz"]),
            )
            ts = i * dt_us
            writer.writerow([ts, f"{dt_s:.6f}"] + [f"{v:.5f}" for v in vals])
            n += 1

    print(f"raw_from_calib: {n} frames -> {args.out} (@ {args.fps:g} Hz)")


if __name__ == "__main__":
    main()
