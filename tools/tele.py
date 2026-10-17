"""Print a time window of a run's telemetry (dev only).

  python ..\\tools\\tele.py RUN_DIR [t0 t1] [col ...]      RUN_DIR may be 'latest'
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
t0, t1 = (float(args.pop(0)), float(args.pop(0))) if len(args) >= 2 and args[0][0].isdigit() else (0.0, 1e9)
cols = args or DEFAULT
d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
m = (d["t"] >= t0) & (d["t"] <= t1) & (d["ctrl_step"] > 0)
rows = d[m][::2]
print(run)
print("t      " + " ".join(f"{c[:9]:>9}" for c in cols))
for r in rows:
    print(f"{r['t']:6.2f} " + " ".join(f"{r[c]:9.3f}" for c in cols))
