"""Eco-mode demo figures (dev only, not submitted). Run from simulator/ after tools/eco_pareto.py:

  .venv\\Scripts\\python.exe ..\\tools\\eco_figures.py --results DIR\\results.json --eco 0.6 [--course practice_technical]

Writes to tools/figures/:
  eco_energy_breakdown.png  where the battery energy goes: baseline / Performance / Eco (stacked bars)
  eco_speed_trace.png       Performance vs Eco speed over the course, Eco's coasting stretches shaded
and prints the headline sentence with the measured numbers.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import energy  # noqa: E402
import plot_run as pr  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

FIG = os.path.join(HERE, "figures")
PERF = "#104281"                      # Performance: the dark step of the accent ramp; Eco: the accent itself
PARTS = [("ke", "Kinetic energy left at the finish line", "#b7d3f6"), ("drag", "Aero drag", "#0d366b"), ("roll", "Rolling", "#1c5cab"), ("tyre", "Tyre scrub (cornering)", "#5598e7"),
         ("brake", "Brake heat (not recovered)", "#898781"), ("drive", "Drivetrain loss (assumed 10 %)", "#c3c2b7")]


def parts(run_dir: str, veh: dict) -> dict:
    e = energy.analyse_run(run_dir, veh)
    return {"ke": e["dKE_Wh"], "drag": e["E_drag_Wh"], "roll": e["E_roll_Wh"], "tyre": e["E_tyre_Wh"],
            "brake": e["E_brake_Wh"] - e["E_regen_Wh"], "drive": e["drive_loss_Wh"],
            "total": e["E_batt_Wh"], "lap": e["lap_time_s"], "Wh_km": e["Wh_per_km"]}


def fig_breakdown(runs: dict, veh: dict, course: str, path: str) -> dict:
    vals = {k: parts(r, veh) for k, r in runs.items()}
    fig, ax = plt.subplots(figsize=(9.6, 4.8))
    names = list(vals)
    x = np.arange(len(names))
    bottom = np.zeros(len(names))
    for key, label, col in PARTS:
        h = np.array([max(vals[n][key], 0.0) for n in names])
        ax.bar(x, h, 0.56, bottom=bottom, color=col, edgecolor=pr.SURFACE, linewidth=2, label=label, zorder=3)
        bottom += h
    for i, n in enumerate(names):
        v = vals[n]
        ax.text(i, bottom[i] + 4, f"{v['total']:.0f} Wh\n{v['lap']:.1f} s", ha="center", va="bottom",
                fontsize=9, color=pr.INK)
    ax.set_xticks(x, names)
    ax.set_ylabel("Battery energy for one run, Wh")
    ax.set_ylim(0, bottom.max() * 1.22)
    ax.set_title(f"Where the energy goes, {course.replace('_', ' ')}", pad=26)
    ax.text(0, 1.02, "Bicycle backend. Assumed: 90 % drivetrain efficiency, regen recovers 65 % of braking up to 60 kW\n"
            "(the simulator has no electric drivetrain; these are not vehicle specs)", transform=ax.transAxes,
            fontsize=7.5, color=pr.MUTED, va="bottom")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=8, ncol=1, reverse=True)
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return vals


def fig_speed(perf: str, eco: str, eco_w: float, course: str, path: str) -> float:
    d_p, s_p = pr.load(perf)
    d_e, s_e = pr.load(eco)
    fig, ax = plt.subplots(figsize=(10, 4.2))
    coast = (d_e["app_throttle"] == 0) & (d_e["app_brake"] == 0) & (d_e["vx"] > 3.0)
    prog = d_e["progress"]
    # shade each coasting stretch
    edges = np.flatnonzero(np.diff(np.concatenate(([0], coast.astype(int), [0]))))
    for a, b in zip(edges[::2], edges[1::2]):
        if b - a >= 4:                                   # >= 0.1 s
            ax.axvspan(prog[a], prog[b - 1], color=pr.PLAN, alpha=0.35, lw=0, zorder=1)
    ax.plot(d_p["progress"], d_p["vx"], color=PERF, lw=1.6, zorder=3,
            label=f"Performance (eco_weight 0): {s_p['metrics']['completion']['time_s']:.1f} s")
    ax.plot(prog, d_e["vx"], color=pr.OURS, lw=2, zorder=4,
            label=f"Eco (eco_weight {eco_w:g}): {s_e['metrics']['completion']['time_s']:.1f} s")
    ax.fill_between([], [], color=pr.PLAN, alpha=0.35, label="Eco coasting: no throttle, no brake")
    ax.set_xlabel("Distance along the course, m")
    ax.set_ylabel("Speed, m/s")
    ax.set_xlim(0, max(prog.max(), d_p["progress"].max()))
    ax.set_ylim(0, None)
    ax.set_title(f"Performance vs Eco speed, {course.replace('_', ' ')}")
    ax.legend(loc="lower center", fontsize=8.5, ncol=3, bbox_to_anchor=(0.5, -0.38))
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return float(coast.sum() * np.mean(np.diff(d_e["t"])))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", required=True)
    ap.add_argument("--eco", type=float, required=True)
    ap.add_argument("--course", default="practice_technical")
    a = ap.parse_args()
    res = json.load(open(a.results))
    c = res["courses"][a.course]
    veh = energy.load_vehicle(os.path.join("config", "sim.yaml"))
    runs = {"Baseline\npure pursuit": c["baseline"]["run"], "Performance\neco_weight 0": c["eco"]["0"]["run"],
            f"Eco\neco_weight {a.eco:g}": c["eco"][f"{a.eco:g}"]["run"]}
    os.makedirs(FIG, exist_ok=True)
    vals = fig_breakdown(runs, veh, a.course, os.path.join(FIG, "eco_energy_breakdown.png"))
    t_coast = fig_speed(c["eco"]["0"]["run"], c["eco"][f"{a.eco:g}"]["run"], a.eco, a.course,
                        os.path.join(FIG, "eco_speed_trace.png"))
    # headline over all courses of the sweep
    k = f"{a.eco:g}"
    de = [100 * (cc["eco"][k]["Wh_per_km"] / cc["eco"]["0"]["Wh_per_km"] - 1) for cc in res["courses"].values()]
    dt = [100 * (cc["eco"][k]["time_s"] / cc["eco"]["0"]["time_s"] - 1) for cc in res["courses"].values()]
    pen = sum(cc["eco"][k]["penalties"] for cc in res["courses"].values())
    print(f"{a.course}: " + ", ".join(f"{n.split(chr(10))[0]} {v['total']:.0f} Wh / {v['lap']:.2f} s"
                                       for n, v in vals.items()) + f"; Eco coasts {t_coast:.1f} s")
    print(f"Eco (eco_weight {a.eco:g}): {np.mean(de):+.0f} % energy for {np.mean(dt):+.1f} % lap time on average "
          f"over {len(de)} courses (range {min(de):+.0f}..{max(de):+.0f} % / {min(dt):+.1f}..{max(dt):+.1f} %), "
          f"{pen} penalties")
    return 0


if __name__ == "__main__":
    sys.exit(main())
