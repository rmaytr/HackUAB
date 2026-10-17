"""Print a window of a run's telemetry (dev only).

  python ..\\tools\\tele.py RUN_DIR [t0 t1] [col ...]          window in seconds
  python ..\\tools\\tele.py RUN_DIR p140 p215 [col ...]        window in progress (m)
  RUN_DIR may be 'latest'.
"""
import glob
import os
import sys

import numpy as np

DEFAULT = ["progress", "speed", "target_speed", "lateral", "path_err", "cmd_steer", "cmd_throttle",
           "cmd_brake", "yaw_rate"]

args = sys.argv[1:]
run = args.pop(0)
if run == "latest":
    run = max(glob.glob(os.path.join("runs", "*")), key=os.path.getmtime)
key, lo, hi = "t", 0.0, 1e9
if len(args) >= 2 and args[0][0] in "0123456789p":
    key = "progress" if args[0][0] == "p" else "t"
    lo, hi = float(args.pop(0).lstrip("p")), float(args.pop(0).lstrip("p"))
cols = args or DEFAULT
d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
m = (d[key] >= lo) & (d[key] <= hi) & (d["ctrl_step"] > 0)
rows = d[m][::2]
print(run)
print("t      " + " ".join(f"{c[:9]:>9}" for c in cols))
for r in rows:
    print(f"{r['t']:6.2f} " + " ".join(f"{r[c]:9.3f}" for c in cols))
