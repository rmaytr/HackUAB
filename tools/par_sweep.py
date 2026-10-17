"""Parallel parameter sweep of the full controller on the bicycle backend (dev only). From simulator/:

  .venv\\Scripts\\python.exe ..\\tools\\par_sweep.py -j 12 ff_preview_t=0.1,0.2 fb_gain=1,1.5 [--courses a.yaml,b.yaml]

Runs every combination x course as separate processes (-j at a time) and prints, per combination:
total cones, off-course, mean technical points, worst boundary margin, mean path error, mean lap.
Compute scores are meaningless under parallel load, so the table reports points without them too.
"""
import argparse
import itertools
import json
import os
import re
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor

PY = os.path.join(".venv", "Scripts", "python.exe")
DEFAULT = ["tracks/practice_technical.yaml", "tracks/practice_obstacles.yaml", "tracks/hackathon_test.yaml",
           "../tracks/seed5.yaml", "../tracks/seed42.yaml", "../tracks/seed7.yaml", "../tracks/seed123.yaml",
           "../tracks/seed99.yaml"]

ap = argparse.ArgumentParser()
ap.add_argument("grid", nargs="*")
ap.add_argument("-j", type=int, default=12)
ap.add_argument("--courses", default=",".join(DEFAULT))
ap.add_argument("--backend", default="bicycle")
ap.add_argument("--serial-timing", action="store_true",
                help="keep the official enforce-mode deadline (only meaningful with -j 1)")
a = ap.parse_args()
# The harness pins every controller to the LAST logical core(s), so parallel runs share them and
# blow the 50 ms deadline (held commands -> cones, DNFs). measure mode waits for every answer, so
# driving is identical to a serial run; only the compute score is meaningless (see pts-cpu).
SETS = [] if a.serial_timing else ["--set", "compute_budget.mode=measure", "--set", "compute_budget.cpu_cores=8"]
grid = {k: [float(x) for x in v.split(",")] for k, v in (g.split("=") for g in a.grid)}
combos = [dict(zip(grid, c)) for c in itertools.product(*grid.values())] or [{}]
courses = a.courses.split(",")
tmp = tempfile.mkdtemp(prefix="par_sweep_")


def run(job):
    ci, over, course = job
    cfg = os.path.join(tmp, f"c{ci}.yaml")
    if not os.path.exists(cfg):
        with open(cfg, "w") as f:
            f.write("".join(f"{k}: {v}\n" for k, v in over.items()) or "{}\n")
    out = subprocess.run([PY, "-m", "autocross", "run", "-c", "../hackuab/controller.py", "-t", course,
                          "--backend", a.backend, "--no-render", "--team", f"ps{ci}", "--controller-config", cfg]
                         + SETS,
                         capture_output=True, text=True).stdout
    m = re.search(r"^\s+-> (.+)$", out, re.M)
    if not m:
        return ci, course, None
    s = json.load(open(os.path.join(m.group(1).strip(), "summary.json")))
    return ci, course, s


jobs = [(ci, over, c) for ci, over in enumerate(combos) for c in courses]
res = {}
with ThreadPoolExecutor(a.j) as ex:
    for ci, course, s in ex.map(run, jobs):
        res.setdefault(ci, []).append((course, s))

print(f"{'params':42} {'cones':>5} {'off':>4} {'fail':>4} {'pts':>6} {'pts-cpu':>7} {'minmarg':>7} {'perr':>5} {'lap':>6}")
for ci, over in enumerate(combos):
    rs = [s for _, s in res[ci] if s]
    fails = len(res[ci]) - len(rs)
    cones = sum(s["metrics"]["penalties"].get("cone_hit", 0) for s in rs)
    off = sum(s["metrics"]["penalties"].get("off_course", 0) for s in rs)
    pts = sum(s["score"]["technical_points"] for s in rs) / max(len(rs), 1)
    # points with the compute category at full marks: comparable across parallel runs
    pts_nc = sum(s["score"]["technical_points"] - s["score"]["category_points"]["compute"] + 15 for s in rs) / max(len(rs), 1)
    marg = min(s["metrics"]["deviation"]["min_boundary_margin_m"] for s in rs)
    perr = sum(s["metrics"]["deviation"]["path_err_mean_m"] for s in rs) / max(len(rs), 1)
    lap = sum(s["metrics"]["completion"]["time_s"] or 0 for s in rs) / max(len(rs), 1)
    label = " ".join(f"{k}={v:g}" for k, v in over.items()) or "(defaults)"
    print(f"{label:42} {cones:5d} {off:4d} {fails:4d} {pts:6.2f} {pts_nc:7.2f} {marg:7.2f} {perr:5.2f} {lap:6.1f}")
    bad = [(os.path.basename(c), s["metrics"]["penalties"].get("cone_hit", 0)) for c, s in res[ci] if s and s["metrics"]["penalties"].get("cone_hit", 0)]
    if bad:
        print(f"{'':42}   cones on: {bad}")
