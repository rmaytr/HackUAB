"""Parallel version of `autocross batch` (dev only). From simulator/:

  .venv\\Scripts\\python.exe ..\\tools\\par_robust.py [-j 12] [--backend bicycle] [--courses a.yaml,b.yaml]
                    [--controller-config cfg.yaml] [--only low_grip,short_sight]

Reads courses x perturbations x seeds from config/robustness.yaml and runs them as parallel processes
in compute_budget.mode=measure (driving identical to serial runs; see par_sweep.py), then prints one
row per perturbation: finished runs, cones, off-course, missed gates, DNFs, and points excluding the
compute category. Use -j 1 --serial-timing on CARLA (one server).
"""
import argparse
import json
import os
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor

import yaml

PY = os.path.join(".venv", "Scripts", "python.exe")
ap = argparse.ArgumentParser()
ap.add_argument("-j", type=int, default=12)
ap.add_argument("--backend", default="bicycle")
ap.add_argument("--courses")
ap.add_argument("--controller", default="../hackuab/controller.py")
ap.add_argument("--controller-config")
ap.add_argument("--only")
ap.add_argument("--serial-timing", action="store_true")
ap.add_argument("--robustness", default=os.path.join("config", "robustness.yaml"), help="sweep definition YAML")
a = ap.parse_args()

cfg = yaml.safe_load(open(a.robustness))
courses = a.courses.split(",") if a.courses else cfg["tracks"]
perts = cfg["perturbations"]
if a.only:
    perts = {k: v for k, v in perts.items() if k in a.only.split(",")}
seeds = cfg.get("seeds", [0])
SETS = [] if a.serial_timing else ["--set", "compute_budget.mode=measure", "--set", "compute_budget.cpu_cores=8"]


def run(job):
    name, pert, course, seed = job
    cmd = [PY, "-m", "autocross", "run", "-c", a.controller, "-t", course, "--backend", a.backend, "--no-render",
           # unique team per job: run folders are named by second + team + course, so two seeds of one
           # perturbation starting in the same second would otherwise write into the same folder
           "--team", f"rob_{name}_s{seed}", "--set", f"sim.seed={seed}"] + SETS
    for k, v in pert.items():
        cmd += ["--perturb", f"{k}={v}"]
    if a.controller_config:
        cmd += ["--controller-config", a.controller_config]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    m = re.search(r"^\s+-> (.+)$", out, re.M)
    s = json.load(open(os.path.join(m.group(1).strip(), "summary.json"))) if m else None
    return name, course, seed, s, (m.group(1).strip() if m else out[-300:])


jobs = [(n, p, c, sd) for n, p in perts.items() for c in courses for sd in seeds]
rows = {}
with ThreadPoolExecutor(a.j) as ex:
    for name, course, seed, s, where in ex.map(run, jobs):
        rows.setdefault(name, []).append((course, seed, s, where))

print(f"{'perturbation':22} {'done':>5} {'cones':>5} {'off':>4} {'gates':>5} {'dnf':>4} {'pts-cpu':>7} {'lap':>6}  problems")
for name in perts:
    rs = rows[name]
    ok = [r for r in rs if r[2]]
    pen = lambda r, k: r[2]["metrics"]["penalties"].get(k, 0)
    fin = sum(1 for r in ok if r[2]["metrics"]["completion"]["finished"])
    cones = sum(pen(r, "cone_hit") for r in ok)
    off = sum(pen(r, "off_course") for r in ok)
    gates = sum(pen(r, "missed_gate") for r in ok)
    dnf = len(rs) - fin
    pts = sum(r[2]["score"]["technical_points"] - r[2]["score"]["category_points"]["compute"] + 15 for r in ok) / max(len(ok), 1)
    laps = [r[2]["metrics"]["completion"]["time_s"] for r in ok if r[2]["metrics"]["completion"]["time_s"]]
    lap = sum(laps) / max(len(laps), 1)
    probs = [f"{os.path.basename(r[0]).split('.')[0]}/s{r[1]}"
             + ("" if r[2] is None else f"(c{pen(r, 'cone_hit')} o{pen(r, 'off_course')} g{pen(r, 'missed_gate')}"
                + ("" if r[2]["metrics"]["completion"]["finished"] else f" DNF:{r[2]['metrics']['completion']['dnf_reason']}") + ")")
             for r in rs if r[2] is None or not r[2]["metrics"]["completion"]["finished"]
             or pen(r, "cone_hit") or pen(r, "off_course") or pen(r, "missed_gate")]
    print(f"{name:22} {fin:>2}/{len(rs):<2} {cones:5d} {off:4d} {gates:5d} {dnf:4d} {pts:7.2f} {lap:6.1f}  {' '.join(probs)}")
