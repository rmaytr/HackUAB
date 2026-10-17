"""Eco-mode Pareto sweep: lap time vs energy for every eco_weight (dev only, not submitted). Run from simulator/:

  .venv\\Scripts\\python.exe ..\\tools\\eco_pareto.py [--weights 0,0.2,0.4,0.6,0.8,1] [--out DIR]
  .venv\\Scripts\\python.exe ..\\tools\\eco_pareto.py --plot-only --out DIR      # redraw from DIR/results.json

Courses: practice_technical, hackathon_test and gen-track seeds 1/2/3 (--obstacles 3, generated into
the temp output folder, never committed). Bicycle backend only, one run at a time (so step times are
clean). For every point: lap time, Wh/km (tools/energy.py, assumed eta/regen defaults), technical
points, penalties and p99 step time. The baseline pure-pursuit controller is run once per course as a
grey reference.

Recommendation: ONE eco_weight for all courses, the lowest mean energy among weights that keep every
course finished, penalty-free, error-free and within --max-loss (10 %) of the eco_weight 0 lap time.
Writes DIR/results.json, tools/figures/eco_pareto.png and tools/figures/eco_pareto.md.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import energy  # noqa: E402

PY = sys.executable
CTRL = os.path.join(HERE, "..", "hackuab", "controller.py")
BASELINE = os.path.join("controllers", "baseline_pure_pursuit.py")
FIG = os.path.join(HERE, "figures")


def run(ctrl: str, track: str, cfg: str | None, team: str) -> str:
    cmd = [PY, "-m", "autocross", "run", "-c", ctrl, "-t", track, "--backend", "bicycle", "--no-render",
           "--team", team]
    if cfg:
        cmd += ["--controller-config", cfg]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    return re.search(r"^\s+-> (.+)$", out, re.M).group(1).strip()


def measure(run_dir: str, veh: dict) -> dict:
    s = json.load(open(os.path.join(run_dir, "summary.json")))
    m = s["metrics"]
    e = energy.analyse_run(run_dir, veh)
    return {"run": run_dir, "finished": bool(m["completion"]["finished"]), "time_s": m["completion"]["time_s"],
            "points": s["score"]["technical_points"], "penalties": int(sum(m["penalties"].values())),
            "errors": int(m["compute"]["errors"]), "p99_ms": m["compute"]["step_ms_p99"],
            "Wh": e["E_batt_Wh"], "Wh_per_km": e["Wh_per_km"], "residual_pct": e["residual_pct"],
            "E_drag_Wh": e["E_drag_Wh"], "E_roll_Wh": e["E_roll_Wh"], "E_brake_Wh": e["E_brake_Wh"],
            "E_regen_Wh": e["E_regen_Wh"], "E_tyre_Wh": e["E_tyre_Wh"], "drive_loss_Wh": e["drive_loss_Wh"]}


def sweep(weights, out: str) -> dict:
    tracks = {"practice_technical": os.path.join("tracks", "practice_technical.yaml"),
              "hackathon_test": os.path.join("tracks", "hackathon_test.yaml")}
    for seed in (1, 2, 3):
        path = os.path.join(out, f"seed{seed}.yaml")
        if not os.path.exists(path):
            subprocess.run([PY, "-m", "autocross", "gen-track", "--seed", str(seed), "--obstacles", "3",
                            "--out", path], check=True, capture_output=True)
        tracks[f"seed{seed}"] = path
    veh = energy.load_vehicle(os.path.join("config", "sim.yaml"))
    res = {"weights": list(weights), "courses": {}}
    for name, track in tracks.items():
        rows = {}
        for w in weights:
            cfg = os.path.join(out, f"eco{w:g}.yaml")
            with open(cfg, "w") as f:
                f.write(f"strict: true\neco_weight: {w}\n")
            rows[f"{w:g}"] = measure(run(CTRL, track, cfg, f"eco{w:g}"), veh)
            r = rows[f"{w:g}"]
            print(f"{name:20} eco {w:4.2f}  {r['time_s'] or float('nan'):7.2f} s  {r['Wh_per_km']:6.1f} Wh/km  "
                  f"{r['points']:5.2f} pts  pen {r['penalties']}  err {r['errors']}  p99 {r['p99_ms']:.1f} ms  "
                  f"resid {r['residual_pct']:+.2f} %", flush=True)
        base = measure(run(BASELINE, track, None, "baseline"), veh)
        print(f"{name:20} baseline  {base['time_s'] or float('nan'):7.2f} s  {base['Wh_per_km']:6.1f} Wh/km", flush=True)
        res["courses"][name] = {"eco": rows, "baseline": base}
    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump(res, f, indent=1)
    return res


def recommend(res: dict, max_loss: float) -> tuple[float | None, dict]:
    """Lowest mean relative energy among weights that are clean and within max_loss everywhere."""
    table = {}
    for w in res["weights"]:
        k = f"{w:g}"
        ok, d_t, d_e = True, [], []
        for c in res["courses"].values():
            r, r0 = c["eco"][k], c["eco"]["0"]
            clean = r["finished"] and r["penalties"] == 0 and r["errors"] == 0
            loss = (r["time_s"] / r0["time_s"] - 1.0) if r["finished"] else float("inf")
            ok &= clean and loss <= max_loss
            d_t.append(loss)
            d_e.append(r["Wh_per_km"] / r0["Wh_per_km"] - 1.0)
        table[w] = {"ok": ok, "time_pct": 100 * float(np.mean(d_t)), "energy_pct": 100 * float(np.mean(d_e)),
                    "time_pct_max": 100 * float(np.max(d_t))}
    good = [w for w in res["weights"] if table[w]["ok"]]
    best = min(good, key=lambda w: table[w]["energy_pct"]) if good else None
    return best, table


def markdown(res: dict, best, table, max_loss: float) -> str:
    lines = ["| Course | eco_weight | Lap (s) | Lap vs 0 | Wh/km | Energy vs 0 | Points /75 | Penalties | p99 (ms) |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, c in res["courses"].items():
        r0 = c["eco"]["0"]
        for w in res["weights"]:
            r = c["eco"][f"{w:g}"]
            mark = " **(rec.)**" if w == best else ""
            lines.append(f"| {name} | {w:g}{mark} | {r['time_s']:.2f} | {100 * (r['time_s'] / r0['time_s'] - 1):+.1f} % "
                         f"| {r['Wh_per_km']:.0f} | {100 * (r['Wh_per_km'] / r0['Wh_per_km'] - 1):+.1f} % "
                         f"| {r['points']:.2f} | {r['penalties']} | {r['p99_ms']:.1f} |")
        b = c["baseline"]
        lines.append(f"| {name} | baseline PP | {b['time_s'] or float('nan'):.2f} | "
                     f"{100 * ((b['time_s'] or np.nan) / r0['time_s'] - 1):+.1f} % | {b['Wh_per_km']:.0f} | "
                     f"{100 * (b['Wh_per_km'] / r0['Wh_per_km'] - 1):+.1f} % | {b['points']:.2f} | {b['penalties']} "
                     f"| {b['p99_ms']:.1f} |")
    lines += ["", "| eco_weight | mean lap vs 0 | worst lap vs 0 | mean energy vs 0 | all clean and within "
              f"{100 * max_loss:.0f} % |", "|---:|---:|---:|---:|---|"]
    for w in res["weights"]:
        t = table[w]
        lines.append(f"| {w:g} | {t['time_pct']:+.1f} % | {t['time_pct_max']:+.1f} % | {t['energy_pct']:+.1f} % "
                     f"| {'yes' if t['ok'] else 'no'} |")
    lines.append("")
    lines.append(f"Recommended eco_weight: **{best:g}**" if best is not None else "No weight qualifies.")
    return "\n".join(lines)


def plot(res: dict, best, max_loss: float, path: str) -> None:
    import plot_run as pr
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8.8, 5.6))
    markers = ["o", "s", "D", "^", "v"]
    labels = []
    ws = res["weights"]
    for (name, c), mk in zip(res["courses"].items(), markers):
        r0 = c["eco"]["0"]
        x = [100 * (c["eco"][f"{w:g}"]["time_s"] / r0["time_s"] - 1) for w in ws]
        y = [100 * (c["eco"][f"{w:g}"]["Wh_per_km"] / r0["Wh_per_km"] - 1) for w in ws]
        ax.plot(x, y, color=pr.OURS, lw=2, marker=mk, ms=6, mfc=pr.SURFACE, mec=pr.OURS, mew=1.6, zorder=3)
        ax.annotate(name.replace("_", " ").replace("seed", "random seed "), (x[-1], y[-1]), xytext=(7, 0),
                    textcoords="offset points", fontsize=8, color=pr.INK2, va="center")
        labels.append((x[-1], y[-1]))
        if best is not None:
            i = ws.index(best)
            ax.plot(x[i], y[i], marker=mk, ms=11, color=pr.OURS, mec=pr.SURFACE, mew=2, zorder=4)
        b = c["baseline"]
        if b["time_s"]:
            bx, by = 100 * (b["time_s"] / r0["time_s"] - 1), 100 * (b["Wh_per_km"] / r0["Wh_per_km"] - 1)
            ax.plot(bx, by, marker=mk, ms=7, color=pr.BASE, ls="none", zorder=2)
    ax.axvline(100 * max_loss, color=pr.AXIS, lw=1, ls="--", zorder=1)
    ax.text(100 * max_loss, 1.5, f" {100 * max_loss:.0f} % lap-time budget", color=pr.MUTED, fontsize=8, va="bottom")
    ax.axhline(0, color=pr.AXIS, lw=0.8)
    ax.axvline(0, color=pr.AXIS, lw=0.8)
    ax.set_xlabel("Lap time vs Performance mode (eco_weight 0), %")
    ax.set_ylabel("Battery energy per km vs Performance mode, %")
    title = "Eco mode: one knob from fastest lap to lowest energy"
    ax.set_title(title, pad=30)
    sub = (f"eco_weight {ws[0]:g} to {ws[-1]:g} along each line; large marker = recommended {best:g}; "
           if best is not None else "")
    ax.text(0, 1.02, sub + "grey = baseline pure pursuit (same marker per course).\nBicycle backend; energy from "
            "tools/energy.py with assumed 90 % drivetrain efficiency and 65 % regen up to 60 kW.",
            transform=ax.transAxes, fontsize=7.5, color=pr.MUTED, va="bottom")
    from matplotlib.lines import Line2D
    ax.legend(handles=[Line2D([], [], color=pr.OURS, lw=2, marker="o", mfc=pr.SURFACE, label="ours, eco_weight sweep"),
                       Line2D([], [], color=pr.BASE, ls="none", marker="o", label="baseline pure pursuit")],
              loc="lower left", fontsize=8.5)
    ax.set_xlim(right=max(x for x, _ in labels) + 4.5)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="0,0.2,0.4,0.6,0.8,1")
    ap.add_argument("--out", default=None, help="folder for configs, generated tracks and results.json")
    ap.add_argument("--max-loss", type=float, default=0.10)
    ap.add_argument("--plot-only", action="store_true")
    a = ap.parse_args()
    out = a.out or tempfile.mkdtemp(prefix="eco_pareto_")
    os.makedirs(out, exist_ok=True)
    if a.plot_only:
        res = json.load(open(os.path.join(out, "results.json")))
    else:
        res = sweep([float(w) for w in a.weights.split(",")], out)
    best, table = recommend(res, a.max_loss)
    os.makedirs(FIG, exist_ok=True)
    plot(res, best, a.max_loss, os.path.join(FIG, "eco_pareto.png"))
    md = markdown(res, best, table, a.max_loss)
    with open(os.path.join(FIG, "eco_pareto.md"), "w") as f:
        f.write(md + "\n")
    print(md)
    print("results:", os.path.join(out, "results.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
