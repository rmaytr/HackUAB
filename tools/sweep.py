"""Tracker parameter sweep over the eval courses (dev only). Run from simulator/:

  .venv\\Scripts\\python.exe ..\\tools\\sweep.py brake_floor=0.15,0.3 fb_gain=0.7,1.0

Each combination runs the stand-in controller on practice_technical + seeds 42/7/123 and
prints mean technical points (technical course weighted 0.6 like the event), lap times,
mean path error and max rear slip. Config YAMLs go to the system temp dir.
"""
import itertools
import json
import os
import re
import subprocess
import sys
import tempfile

COURSES = ["tracks/practice_technical.yaml", "../tracks/seed42.yaml", "../tracks/seed7.yaml", "../tracks/seed123.yaml"]
PY = os.path.join(".venv", "Scripts", "python.exe")
CTRL = "../tools/tracker_dev_controller.py"

grid = {}
for arg in sys.argv[1:]:
    k, vals = arg.split("=")
    grid[k] = [float(v) for v in vals.split(",")]

tmp = tempfile.mkdtemp(prefix="tracker_sweep_")
print(f"{'params':40} {'score':>6} {'tech':>5} {'times':>24} {'perr':>5} {'slip':>5} {'cones':>5}")
for combo in itertools.product(*grid.values()):
    over = dict(zip(grid, combo))
    cfg = os.path.join(tmp, "cfg.yaml")
    with open(cfg, "w") as f:
        f.write("tracker:\n" + "".join(f"  {k}: {v}\n" for k, v in over.items()))
    pts, times, perr, slip, cones = [], [], [], [], 0
    for c in COURSES:
        out = subprocess.run([PY, "-m", "autocross", "run", "-c", CTRL, "-t", c, "--backend", "bicycle",
                              "--no-render", "--team", "sweep", "--controller-config", cfg],
                             capture_output=True, text=True).stdout
        run = re.search(r"^\s+-> (.+)$", out, re.M).group(1).strip()
        s = json.load(open(os.path.join(run, "summary.json")))
        m = s["metrics"]
        pts.append(s["score"]["technical_points"])
        times.append(m["completion"]["time_s"] or float("nan"))
        perr.append(m["deviation"]["path_err_mean_m"])
        slip.append(m["dynamics"]["rear_slip_max_deg"])
        cones += sum(1 for e in json.load(open(os.path.join(run, "events.json"))) if e["kind"] == "cone_hit"
                     and e["detail"].get("mapped", True))
    score = 0.6 * pts[0] + 0.4 * sum(pts[1:]) / 3
    label = " ".join(f"{k}={v:g}" for k, v in over.items())
    print(f"{label:40} {score:6.2f} {pts[0]:5.1f} {'/'.join(f'{t:.1f}' for t in times):>24} "
          f"{sum(perr) / 4:5.2f} {max(slip):5.1f} {cones:5d}", flush=True)
