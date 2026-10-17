"""List rear-slip episodes (|rear-axle slip| > 6 deg, the dynamics metric's threshold) in runs (dev only).

  python ..\\tools\\slip.py RUN_DIR [RUN_DIR ...]      RUN_DIR may be 'latest'
"""
import glob
import json
import os
import sys

import numpy as np

for run in sys.argv[1:]:
    if run == "latest":
        run = max(glob.glob(os.path.join("runs", "*")), key=os.path.getmtime)
    d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
    lr = json.load(open(os.path.join(run, "summary.json")))["vehicle"]["cg_to_rear"]
    rs = np.degrees(np.arctan2(d["vy"] - lr * d["yaw_rate"], np.maximum(np.abs(d["vx"]), 0.1)))
    moving = d["speed"] > 3.0
    m = moving & (np.abs(rs) > 6.0)
    print(f"{os.path.basename(run)}: {m.sum()} of {moving.sum()} frames over 6 deg")
    if not m.any():
        continue
    t = d["t"][m]
    br = np.where(np.diff(t) > 0.2)[0]
    for a, b in zip(np.r_[t[0], t[br + 1]], np.r_[t[br], t[-1]]):
        w = (d["t"] >= a - 0.5) & (d["t"] <= b)
        print(f"  {a:6.2f}-{b:6.2f}s  prog {d['progress'][w][0]:5.0f}  v {d['speed'][w].mean():5.1f}"
              f"  slip {np.abs(rs[w]).max():4.0f}  brake {d['cmd_brake'][w].max():.2f}"
              f"  thr {d['cmd_throttle'][w].max():.2f}  |steer| {np.abs(d['cmd_steer'][w]).max():.2f}")
