#!/usr/bin/env python3
"""Print what the Star Citizen / Wine bridge is actually being handed.

`/dev/shm/headtrack-pose` is the hand-off point between the engine's Wine
output adapter and `NPClient64.dll` running inside the game's Wine prefix. When
an axis looks dead in-game, this says which side of that boundary the problem
is on: if the numbers here move and the game does not, the bridge is doing its
job and the answer is in the game's own head-tracking settings.

Layout (`WineShmPose`, `#[repr(C, packed)]` — headtrack-output-wine/src/lib.rs,
mirrored by the DLL):

    u64 sequence   monotonic write counter; 0 = engine has written nothing yet
    f64 yaw, pitch, roll     degrees
    f64 x, y, z              mm  (x already has the adapter's SIGN_X applied)

The DLL then negates all six and scales to TrackIR headspace units
(+-16383 = +-180 deg, +-16383 = +-500 mm), so 50 mm of lean reaches the game as
about 1640 units.

Usage:
    scripts/shm_dump.py              # live table until Ctrl-C
    scripts/shm_dump.py --seconds 10 # sample for 10 s, then print per-axis range
"""

import argparse
import mmap
import os
import struct
import sys
import time

SHM_PATH = "/dev/shm/headtrack-pose"
LAYOUT = "<Qdddddd"  # sequence + yaw pitch roll x y z
SIZE = struct.calcsize(LAYOUT)
AXES = ("yaw", "pitch", "roll", "x", "y", "z")


def read_pose(mm):
    mm.seek(0)
    seq, *axes = struct.unpack(LAYOUT, mm.read(SIZE))
    return seq, axes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="sample for N seconds then print min/max per axis (0 = live)")
    ap.add_argument("--hz", type=float, default=20.0, help="sample rate (default 20)")
    args = ap.parse_args()

    if not os.path.exists(SHM_PATH):
        sys.exit(f"{SHM_PATH} does not exist — the engine is not running, "
                 "or the Wine/SC output is disabled")

    period = 1.0 / args.hz
    with open(SHM_PATH, "rb") as f, mmap.mmap(f.fileno(), SIZE, prot=mmap.PROT_READ) as mm:
        seq, axes = read_pose(mm)
        if seq == 0:
            print("sequence = 0: the engine has the segment open but has not "
                  "written a pose yet (no face detected?)")

        lo = list(axes)
        hi = list(axes)
        first_seq = seq
        start = time.monotonic()
        try:
            while True:
                seq, axes = read_pose(mm)
                for i, v in enumerate(axes):
                    lo[i] = min(lo[i], v)
                    hi[i] = max(hi[i], v)
                if args.seconds <= 0:
                    print("seq {:>10}  ".format(seq)
                          + "  ".join(f"{n}={v:8.2f}" for n, v in zip(AXES, axes)),
                          end="\r", flush=True)
                elif time.monotonic() - start >= args.seconds:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            print()

        written = seq - first_seq
        print(f"\nframes written during sample: {written}")
        print(f"{'axis':<6}{'min':>10}{'max':>10}{'range':>10}")
        for i, name in enumerate(AXES):
            unit = "deg" if i < 3 else "mm"
            print(f"{name:<6}{lo[i]:>10.2f}{hi[i]:>10.2f}{hi[i] - lo[i]:>9.2f} {unit}")
        if written == 0:
            print("\nnothing was written during the sample — the values above are "
                  "whatever the last engine run left in the segment, not live data")
        elif all(hi[i] - lo[i] < 0.5 for i in (3, 4, 5)):
            print("\ntranslation never moved more than 0.5 mm — the engine, not the "
                  "bridge, is the place to look")


if __name__ == "__main__":
    main()
