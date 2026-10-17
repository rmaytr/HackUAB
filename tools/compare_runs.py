"""Where along the course does run B gain or lose time against run A? (dev only)

  python ..\\tools\\compare_runs.py RUN_A RUN_B [--seg 20]

Resamples both runs' speed on a common progress grid and prints, per segment, each run's mean
speed and B's cumulative time delta (negative = B ahead).
"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("a")
ap.add_argument("b")
ap.add_argument("--seg", type=float, default=20.0)
args = ap.parse_args()


def load(run):
    d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
    p, t = d["progress"], d["t"]
    keep = np.r_[True, np.diff(p) > 1e-3]                 # progress must increase for interp
    return p[keep], t[keep], d["speed"][keep], d["cmd_brake"][keep], d["cmd_throttle"][keep]


pa, ta, va, ba, ha = load(args.a)
pb, tb, vb, bb, hb = load(args.b)
grid = np.arange(max(pa[0], pb[0]), min(pa[-1], pb[-1]), 0.5)
tA, tB = np.interp(grid, pa, ta), np.interp(grid, pb, tb)
dt0 = tB[0] - tA[0]
print(f"A: {os.path.basename(args.a)}\nB: {os.path.basename(args.b)}")
print(f"{'progress':>12} {'vA':>6} {'vB':>6} {'thrB':>5} {'brkB':>5} {'dT seg':>7} {'dT cum':>7}")
prev = 0.0
for lo in np.arange(grid[0], grid[-1], args.seg):
    m = (grid >= lo) & (grid < lo + args.seg)
    if not m.any():
        continue
    i = np.where(m)[0][-1]
    cum = (tB[i] - tA[i]) - dt0
    print(f"{lo:5.0f}-{lo + args.seg:<5.0f} {np.interp(grid[m], pa, va).mean():6.1f} "
          f"{np.interp(grid[m], pb, vb).mean():6.1f} {np.interp(grid[m], pb, hb).mean():5.2f} "
          f"{np.interp(grid[m], pb, bb).mean():5.2f} {cum - prev:+7.2f} {cum:+7.2f}")
    prev = cum
