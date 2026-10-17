"""Export a static JSON dataset of simulator runs for the demo web app (dev only, NOT submitted).

    .venv\\Scripts\\python.exe ..\\tools\\export_viz.py [--jobs 4] [--quick] [--no-lat]     (from simulator/)

The web app (built on Base44) cannot run Python, so this tool runs a grid of bicycle-backend laps and writes
viz/data/: index.json (courses, runs, headline metrics), tracks/<course>.json (world-frame geometry) and
runs/<id>.json (10 Hz columnar time series + events). The app fetches them from
https://raw.githubusercontent.com/rmaytr/HackUAB/main/viz/data/. The schema is a contract the app is built
against; it is described in viz/data/README.md and checked by tests/test_viz_export.py.

Grid: practice_technical, hackathon_test, practice_obstacles, practice_basic and gen-track seeds 1/2/3
(--obstacles 3, generated into a temp folder, never committed). Per course: the baseline pure-pursuit
controller, ours at every eco_weight in ECO_WEIGHTS (lat_accel_frac at its default) and ours at each extra
lat_accel_frac at eco_weight 0. Strict mode is off (the scored configuration). A run that crashes is
recorded under index.json "failures" and skipped; it never stops the export.

Parallel runs: the harness pins every controller process to the LAST logical core, so parallel laps would
share one core and inflate step times. Each worker therefore runs in its own process with psutil.cpu_count
patched so that its controller gets a core of its own (cores N-1, N-2, ...). The simulator is used as-is.
Laps whose p99 step time exceeds --max-p99 (background load on a dev machine) or that fail are re-run alone,
up to --retries times, and the cleanest attempt is kept.

Energy (energy_wh_cum and the energy metrics) comes from tools/energy.py, whose regen/drivetrain numbers are
ASSUMED round values for a generic EV, not Blazer EV specifications.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import datetime as dt
import json
import math
import os
import queue
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import energy  # noqa: E402

OUT = ROOT / "viz" / "data"
CTRL = ROOT / "hackuab" / "controller.py"
SCHEMA_VERSION = 1
PUBLIC = ["practice_technical", "hackathon_test", "practice_obstacles", "practice_basic"]
SEEDS = [1, 2, 3]
ECO_WEIGHTS = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
LAT_FRACS = [0.6, 0.75, 0.85]          # 0.6 is params.py's default, so its run is the eco_weight 0 run
LAT_DEFAULT = 0.6
PRESETS = {"performance": {"eco_weight": 0.0}, "eco": {"eco_weight": 0.8}}
DT_OUT = 0.1                           # 10 Hz time series
SERIES = ["t", "x", "y", "yaw", "speed", "target_speed", "throttle", "brake", "steer", "ax", "ay", "progress",
          "path_err", "energy_wh_cum", "coast", "fallback", "t_horizon_ms", "t_plan_ms", "t_track_ms", "t_total_ms"]
# series key -> telemetry.csv column (steer is the applied, normalised [-1, 1] command)
TEL_COL = {"t": "t", "x": "x", "y": "y", "yaw": "yaw", "speed": "speed", "target_speed": "target_speed",
           "throttle": "app_throttle", "brake": "app_brake", "steer": "app_steer", "ax": "ax", "ay": "ay",
           "progress": "progress", "path_err": "path_err", "coast": "dbg_coast", "fallback": "dbg_fallback",
           "t_horizon_ms": "dbg_t_horizon_ms", "t_plan_ms": "dbg_t_plan_ms", "t_track_ms": "dbg_t_track_ms",
           "t_total_ms": "dbg_t_total_ms"}
BACKEND_NOTE = ("Bicycle backend only: the harness's dynamic bicycle model (AWD, 40/60 drive split, per-axle "
                "traction limit), 20 Hz control. CARLA is half of the official score but is not in this dataset. "
                "Energy is rebuilt from telemetry by tools/energy.py; the simulator itself has no battery or regen.")


# ----------------------------------------------------------------------------------------- helpers
def find_simulator(arg: str | None) -> Path:
    """The simulator checkout: --sim, this repo's submodule, or the main checkout's (from a git worktree)."""
    cands = [Path(arg)] if arg else []
    cands.append(ROOT / "simulator")
    try:
        common = subprocess.run(["git", "rev-parse", "--path-format=absolute", "--git-common-dir"], cwd=ROOT,
                                capture_output=True, text=True, check=True).stdout.strip()
        cands.append(Path(common).parent / "simulator")
    except (OSError, subprocess.CalledProcessError):
        pass
    for c in cands:
        if (c / "autocross" / "runner.py").exists():
            return c.resolve()
    raise SystemExit("simulator not found; pass --sim PATH (the folder holding autocross/)")


def sim_python(sim: Path) -> str:
    for p in (sim / ".venv" / "Scripts" / "python.exe", sim / ".venv" / "bin" / "python"):
        if p.exists():
            return str(p)
    return sys.executable


def r3(x):
    """Round to 3 decimals; non-finite -> None (JSON null)."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return round(x, 3) if math.isfinite(x) else None


def r2(x):
    x = float(x)
    return round(x, 2) if math.isfinite(x) else None


def pts(a, nd=2) -> list:
    return [[round(float(p[0]), nd), round(float(p[1]), nd)] for p in np.asarray(a, float).reshape(-1, 2)]


def git_sha() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def fmt(x: float) -> str:
    return f"{x:.2f}"


def run_id(course: str, mode: str, ew, laf) -> str:
    return f"{course}__baseline" if mode == "baseline" else f"{course}__{mode}__ew{fmt(ew)}__laf{fmt(laf)}"


def grid(courses: list[str], lat: bool) -> list[dict]:
    jobs = []
    for c in courses:
        jobs.append({"id": run_id(c, "baseline", None, None), "course": c, "mode": "baseline",
                     "params": {"eco_weight": None, "lat_accel_frac": None}})
        for w in ECO_WEIGHTS:
            mode = "performance" if w == 0.0 else "eco"
            jobs.append({"id": run_id(c, mode, w, LAT_DEFAULT), "course": c, "mode": mode,
                         "params": {"eco_weight": w, "lat_accel_frac": LAT_DEFAULT}})
        for f in LAT_FRACS if lat else []:
            if f != LAT_DEFAULT:
                jobs.append({"id": run_id(c, "performance", 0.0, f), "course": c, "mode": "performance",
                             "params": {"eco_weight": 0.0, "lat_accel_frac": f}})
    return jobs


# ----------------------------------------------------------------------------------------- tracks
def track_json(track, cid: str, gate_spacing: float) -> dict:
    """World-frame geometry of one course for the app (ENU, m, 0.01 m)."""
    from autocross import geometry as g
    closed = track.closed

    def line(a):
        r = g.resample(np.asarray(a, float), 1.0, closed)
        return pts(np.vstack((r, r[:1])) if closed else r)   # closed loops repeat the first point

    info = track.info(True, gate_spacing)
    pos, col, mapped = track.all_cones()
    xmin, ymin, xmax, ymax = track.bounds()
    if len(pos):
        xmin, ymin = min(xmin, float(pos[:, 0].min())), min(ymin, float(pos[:, 1].min()))
        xmax, ymax = max(xmax, float(pos[:, 0].max())), max(ymax, float(pos[:, 1].max()))
    return {"id": cid, "centerline": line(track.center), "left_boundary": line(track.left),
            "right_boundary": line(track.right),
            "cones": {"left": pts(pos[mapped & (col == 0)]), "right": pts(pos[mapped & (col == 1)]),
                      "obstacle": pts(pos[~mapped]),
                      "markers": pts(pos[mapped & (col != 0) & (col != 1)])},
            "start_line": pts(info.start_line), "finish_line": pts(info.finish_line),
            "gates": [pts(gt) for gt in info.gates],
            "bounds": {"xmin": r2(xmin), "xmax": r2(xmax), "ymin": r2(ymin), "ymax": r2(ymax)}}


# ----------------------------------------------------------------------------------------- one run
def read_csv_cols(path: Path) -> dict[str, np.ndarray]:
    import csv
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}

    def num(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return math.nan
    return {k: np.array([num(r.get(k)) for r in rows]) for k in rows[0].keys()}


def downsample_idx(t: np.ndarray, dt_out: float = DT_OUT) -> np.ndarray:
    """First row of every dt_out bin, plus the last row."""
    b = np.floor((t - t[0]) / dt_out + 1e-9).astype(int)
    idx = np.flatnonzero(np.r_[True, np.diff(b) > 0])
    return np.unique(np.r_[idx, len(t) - 1])


def energy_cum(tel_e: dict, idx: np.ndarray, veh: dict) -> np.ndarray:
    """Battery energy (Wh) from the start up to each kept row, by running energy.analyse on each prefix, so the
    last value is exactly the run's energy_wh."""
    out = np.zeros(len(idx))
    for j, i in enumerate(idx):
        if i >= 1:
            out[j] = energy.analyse({k: v[: i + 1] for k, v in tel_e.items()}, veh)["E_batt_Wh"]
    return out


def series(run_dir: Path, veh: dict) -> tuple[dict, dict]:
    cols = read_csv_cols(run_dir / "telemetry.csv")
    t = cols["t"]
    idx = downsample_idx(t)
    tel_e = energy.read_telemetry(str(run_dir / "telemetry.csv"))
    out = {}
    for k in SERIES:
        if k == "energy_wh_cum":
            out[k] = [r3(v) for v in energy_cum(tel_e, idx, veh)]
            continue
        c = cols.get(TEL_COL[k])
        if c is None or not np.isfinite(c).any():
            out[k] = [None] * len(idx)            # column missing (e.g. dbg_* for the baseline)
        else:
            out[k] = [r3(v) for v in c[idx]]
    try:
        ev = json.loads((run_dir / "events.json").read_text())
    except (OSError, ValueError):
        ev = []
    out["events"] = [{"t": r3(e.get("t")), "kind": str(e.get("kind")), "x": r3(e.get("x")), "y": r3(e.get("y"))}
                     for e in ev]
    full_e = energy.analyse(tel_e, veh, lap_time=None)
    return out, full_e


def metrics(summary: dict, e: dict) -> dict:
    m, sc = summary["metrics"], summary["score"]
    c, pen, cpu = m["completion"], m["penalties"], m["compute"]
    res = "finished" if c["finished"] else f"dnf: {c.get('dnf_reason')}"
    if summary.get("controller_error"):
        res += f" (controller error: {str(summary['controller_error'])[:80]})"
    return {"finished": bool(c["finished"]), "result": res,
            "lap_time_s": r3(c["time_s"]) if c["time_s"] is not None else None,
            "adjusted_time_s": r3(sc.get("adjusted_time_s")) if sc.get("adjusted_time_s") is not None else None,
            "technical_points": r3(sc["technical_points"]),
            "category_scores": {k: r3(sc["category_scores"][k])
                                for k in ("completion_time", "deviation", "compute", "dynamics")},
            "penalties": {"cone_hits": int(pen.get("cone_hit", 0)), "off_course": int(pen.get("off_course", 0)),
                          "missed_gates": int(pen.get("missed_gate", 0)), "controller_errors": int(cpu.get("errors", 0))},
            "p99_step_ms": r3(cpu["step_ms_p99"]),
            "energy_wh": r3(e["E_batt_Wh"]), "wh_per_km": r3(e["Wh_per_km"]), "peak_power_kw": r3(e["peak_trac_kW"]),
            "rms_jerk": r3(e["rms_jerk"]),
            "energy_breakdown_wh": {"drag": r3(e["E_drag_Wh"]), "rolling": r3(e["E_roll_Wh"]),
                                    "brake_dissipated": r3(e["E_brake_Wh"]), "regen_recovered": r3(e["E_regen_Wh"]),
                                    "tyre_scrub": r3(e["E_tyre_Wh"]), "drive_loss": r3(e["drive_loss_Wh"])},
            "energy_balance_residual_pct": r3(e["residual_pct"])}


def worker(spec_path: str) -> int:
    """Child process: one lap with the controller pinned to core N-1-slot, then write the run file and a
    metrics side-file. Runs with cwd = simulator/ and the simulator on sys.path."""
    spec = json.loads(Path(spec_path).read_text())
    sim = Path(spec["sim"])
    sys.path.insert(0, str(sim))
    os.chdir(sim)
    import psutil
    n = psutil.cpu_count(logical=True) or 1
    slot = int(spec["slot"]) % max(n - 1, 1)
    real = psutil.cpu_count
    psutil.cpu_count = lambda logical=True: (n - slot) if logical else real(logical=False)  # noqa: E731
    from autocross.config import load_sim_config
    from autocross.runner import run
    cfg = load_sim_config(None, None)
    cfg["output"]["plot"] = False
    cfg["carla"]["render"] = False
    ctrl = spec["controller"]
    ccfg = spec.get("controller_cfg")
    out_dir = Path(spec["out_dir"])
    s = run(cfg, ctrl, spec["track"], team=spec["id"].replace(".", "p"), backend="bicycle", controller_cfg=ccfg,
            out_dir=out_dir, verbose=False)
    veh = energy.load_vehicle(str(sim / "config" / "sim.yaml"))
    data, e = series(out_dir, veh)
    run_file = {"schema_version": SCHEMA_VERSION, "id": spec["id"], "course": spec["course"], "dt": DT_OUT, **data}
    Path(spec["run_json"]).write_text(json.dumps(run_file, separators=(",", ":"), allow_nan=False))
    side = {"metrics": metrics(s, e), "reference_time_s": s["metrics"]["completion"].get("reference_time_s"),
            "length_m": s["track"]["length_m"], "closed": s["track"]["closed"], "laps": s["track"]["laps"]}
    Path(spec["out_dir"], "export_metrics.json").write_text(json.dumps(side, default=float))
    return 0


# ----------------------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worker", help=argparse.SUPPRESS)
    ap.add_argument("--sim", default=None, help="simulator folder (default: this repo's, or the main checkout's)")
    ap.add_argument("--jobs", type=int, default=4, help="parallel laps, each controller on its own core")
    ap.add_argument("--no-lat", action="store_true", help="skip the lat_accel_frac axis")
    ap.add_argument("--quick", action="store_true", help="practice_technical only (smoke test)")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--timeout", type=float, default=600.0, help="seconds per lap before it counts as failed")
    ap.add_argument("--max-p99", type=float, default=25.0,
                    help="laps with a slower p99 step (machine load) or a failure are re-run one at a time")
    ap.add_argument("--retries", type=int, default=2, help="sequential re-runs per slow or failed lap")
    a = ap.parse_args()
    if a.worker:
        return worker(a.worker)

    sim = find_simulator(a.sim)
    py = sim_python(sim)
    out = Path(a.out)
    tmp = Path(tempfile.mkdtemp(prefix="export_viz_"))
    print(f"simulator: {sim}\npython: {py}\ntemp: {tmp}", flush=True)

    sys.path.insert(0, str(sim))
    from autocross.config import load_sim_config
    from autocross.track import load as load_track
    gate_spacing = float(load_sim_config(None, None)["rules"]["gate_spacing_m"])

    courses: dict[str, dict] = {}
    for c in (["practice_technical"] if a.quick else PUBLIC):
        courses[c] = {"path": str(sim / "tracks" / f"{c}.yaml"), "kind": "public", "name": c.replace("_", " ").title()}
    for seed in ([] if a.quick else SEEDS):
        p = tmp / f"seed{seed}.yaml"
        r = subprocess.run([py, "-m", "autocross", "gen-track", "--seed", str(seed), "--obstacles", "3", "--out", str(p)],
                           cwd=sim, capture_output=True, text=True)
        if r.returncode or not p.exists():
            print(f"gen-track seed {seed} failed: {r.stderr[-300:]}", flush=True)
            continue
        courses[f"seed{seed}"] = {"path": str(p), "kind": "random", "name": f"Random course (seed {seed})"}

    for d in ("runs", "tracks"):
        shutil.rmtree(out / d, ignore_errors=True)
        (out / d).mkdir(parents=True, exist_ok=True)
    tracks = {}
    for cid, c in list(courses.items()):
        try:
            tracks[cid] = load_track(c["path"])
            (out / "tracks" / f"{cid}.json").write_text(
                json.dumps(track_json(tracks[cid], cid, gate_spacing), separators=(",", ":"), allow_nan=False))
        except Exception as ex:  # a bad course is skipped, never fatal
            print(f"track {cid} failed: {ex!r}", flush=True)
            courses.pop(cid)

    jobs = grid(list(courses), lat=not a.no_lat)
    slots: queue.Queue = queue.Queue()
    for i in range(max(a.jobs, 1)):
        slots.put(i)

    def one(job: dict, attempt: int = 0) -> dict:
        slot = slots.get()
        try:
            rd = tmp / "runs" / f"{job['id']}__try{attempt}"
            spec = {**job, "sim": str(sim), "slot": slot, "track": courses[job["course"]]["path"], "out_dir": str(rd),
                    "run_json": str(rd / "run.json"),
                    "controller": str(sim / "controllers" / "baseline_pure_pursuit.py") if job["mode"] == "baseline"
                    else str(CTRL),
                    "controller_cfg": None if job["mode"] == "baseline" else {k: v for k, v in job["params"].items()}}
            sp = tmp / f"{job['id']}__try{attempt}.spec.json"
            sp.write_text(json.dumps(spec))
            try:
                r = subprocess.run([py, str(Path(__file__).resolve()), "--worker", str(sp)], cwd=sim,
                                   capture_output=True, text=True, timeout=a.timeout)
                ok, err = r.returncode == 0, (r.stderr or r.stdout)[-400:]
            except subprocess.TimeoutExpired:
                ok, err = False, f"timeout after {a.timeout:.0f} s"
            side = rd / "export_metrics.json"
            if ok and side.exists():
                return {**job, "side": json.loads(side.read_text()), "run_json": str(rd / "run.json")}
            return {**job, "error": err.strip() or "worker failed"}
        finally:
            slots.put(slot)

    def show(tag: str, res: dict) -> None:
        if "error" in res:
            print(f"[{tag}] {res['id']}: FAILED {res['error'][-200:]}", flush=True)
        else:
            m = res["side"]["metrics"]
            print(f"[{tag}] {res['id']}: {m['result']} {m['lap_time_s']} s, {m['wh_per_km']} Wh/km, "
                  f"{m['technical_points']} pts, p99 {m['p99_step_ms']} ms", flush=True)

    def badness(res: dict) -> tuple:
        """Lower is better: a run that worked, then fewer step-time overruns of the --max-p99 budget."""
        if "error" in res:
            return (1, math.inf)
        p99 = res["side"]["metrics"]["p99_step_ms"] or math.inf
        return (0, max(p99 - a.max_p99, 0.0))

    results = []
    with cf.ThreadPoolExecutor(max_workers=max(a.jobs, 1)) as ex:
        for k, res in enumerate(ex.map(one, jobs), 1):
            results.append(res)
            show(f"{k}/{len(jobs)}", res)
    # step times are measured on a shared machine: a lap slowed by background load (or a crash) is re-run
    # alone, and the attempt with the cleaner compute profile is kept
    for i, res in enumerate(results):
        for attempt in range(1, a.retries + 1):
            if badness(results[i]) == (0, 0.0):
                break
            again = one({k: res[k] for k in ("id", "course", "mode", "params")}, attempt)
            show(f"retry {attempt}", again)
            if badness(again) < badness(results[i]):
                results[i] = again

    runs, failures = [], []
    course_meta: dict[str, dict] = {}
    for res in results:
        if "error" in res:
            failures.append({"id": res["id"], "course": res["course"], "mode": res["mode"], "params": res["params"],
                             "error": res["error"]})
            continue
        shutil.copyfile(res["run_json"], out / "runs" / f"{res['id']}.json")
        sd = res["side"]
        course_meta.setdefault(res["course"], sd)
        runs.append({"id": res["id"], "course": res["course"], "mode": res["mode"], "backend": "bicycle",
                     "params": res["params"], "metrics": sd["metrics"], "file": f"runs/{res['id']}.json"})

    course_list = []
    for cid, c in courses.items():
        tr, sd = tracks[cid], course_meta.get(cid, {})
        course_list.append({"id": cid, "name": c["name"], "kind": c["kind"], "length_m": r2(tr.length),
                            "reference_time_s": r3(sd.get("reference_time_s")) if sd else None,
                            "closed": bool(tr.closed), "laps": int(tr.laps), "file": f"tracks/{cid}.json"})

    index = {"schema_version": SCHEMA_VERSION,
             "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
             "commit": git_sha(), "backend_note": BACKEND_NOTE,
             "assumptions": {"eta_drive": energy.ETA_DRIVE, "eta_regen": energy.ETA_REGEN,
                             "p_regen_max_kw": energy.P_REGEN_MAX / 1e3,
                             "note": "assumed values, not Blazer EV specifications"},
             "param_axes": {"eco_weight": ECO_WEIGHTS, "lat_accel_frac": LAT_FRACS if not a.no_lat else [LAT_DEFAULT]},
             "presets": PRESETS, "courses": course_list, "runs": runs, "failures": failures}
    (out / "index.json").write_text(json.dumps(index, indent=1, allow_nan=False))
    shutil.rmtree(tmp, ignore_errors=True)

    size = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    biggest = max((p.stat().st_size for p in (out / "runs").glob("*.json")), default=0)
    print(f"\n{len(runs)} runs, {len(failures)} failures, {len(course_list)} courses; "
          f"total {size / 1e6:.2f} MB, largest run file {biggest / 1e3:.0f} KB -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
