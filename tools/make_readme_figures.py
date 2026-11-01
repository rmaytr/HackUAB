"""Regenerate the README / docs figures from the committed visualization dataset (viz/data).

    python tools/make_readme_figures.py            # from the repository root; needs numpy + matplotlib

Writes PNGs to docs/img/. Every number comes from viz/data/index.json and its run files, so the
figures always match the published dataset. Final PERFORMANCE/ECO runs are the submitted controller;
the intermediate ECO weights in the Pareto plot come from an earlier sweep and are labelled as such.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "viz" / "data"
OUT = ROOT / "docs" / "img"

PERF, ECO, BASE, INK, GRID = "#1f6feb", "#1a9e55", "#9aa1aa", "#1f2328", "#e6e8eb"
PUBLIC = ["practice_basic", "practice_technical", "practice_obstacles", "hackathon_test", "practice_loop"]
LABEL = {"practice_basic": "Basic", "practice_technical": "Technical", "practice_obstacles": "Obstacles",
         "hackathon_test": "Hackathon", "practice_loop": "Loop", "seed1": "Random 1",
         "seed2": "Random 2", "seed3": "Random 3", "seed4": "Random 4", "seed5": "Random 5"}
G = 9.81

plt.rcParams.update({
    "figure.dpi": 160, "savefig.dpi": 160, "font.size": 10, "axes.titlesize": 12, "axes.titleweight": "bold",
    "axes.edgecolor": "#d0d7de", "axes.labelcolor": INK, "xtick.color": INK, "ytick.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "figure.facecolor": "white", "axes.facecolor": "white",
})

INDEX = json.loads((DATA / "index.json").read_text())


def run(course: str, mode: str, eco: float | None = None, lat: float | None = 0.75) -> dict | None:
    """Index entry for one run; PERFORMANCE/ECO default to the final submitted presets."""
    if eco is None and mode != "baseline":
        eco = 0.0 if mode == "performance" else 0.8
    for r in INDEX["runs"]:
        if r["course"] != course or r["mode"] != mode:
            continue
        if mode == "baseline" or (r["params"]["eco_weight"] == eco and r["params"]["lat_accel_frac"] == lat):
            return r
    return None


def series(entry: dict) -> dict:
    return json.loads((DATA / entry["file"]).read_text())


def save(fig, name: str) -> None:
    fig.savefig(OUT / name, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", (OUT / name).relative_to(ROOT))


def fig_scores() -> None:
    courses = [c["id"] for c in INDEX["courses"]]
    courses = [c for c in PUBLIC if c in courses] + sorted(c for c in courses if c.startswith("seed"))
    x = np.arange(len(courses))
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for i, (mode, color, name) in enumerate([("baseline", BASE, "Organizers' baseline"),
                                             ("performance", PERF, "HackUAB · PERFORMANCE"),
                                             ("eco", ECO, "HackUAB · ECO")]):
        vals = [(run(c, mode) or {}).get("metrics", {}).get("technical_points", np.nan) for c in courses]
        bars = ax.bar(x + (i - 1) * 0.27, vals, 0.27, color=color, label=name)
        if mode == "baseline":
            for b, c in zip(bars, courses):
                r = run(c, "baseline")
                cones = int(r["metrics"]["penalties"]["cone_hits"]) if r else 0
                if cones:
                    ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.6, f"{cones}\ncones",
                            ha="center", va="bottom", fontsize=7, color="#b42318", linespacing=0.9)
    for xi, c in zip(x, courses):
        if run(c, "baseline") is None:
            ax.text(xi - 0.27, 41, "n/a", ha="center", fontsize=7, color="#57606a")
    ax.set_xticks(x, [LABEL[c] for c in courses], rotation=0)
    ax.set_ylim(40, 80)
    ax.set_ylabel("Technical points (out of 75)")
    ax.set_title("Technical score per course (HackUAB: 0 cones, 0 off-course, 0 errors on every course)")
    ax.axvline(len([c for c in courses if not c.startswith("seed")]) - 0.5, color="#d0d7de", lw=1, ls="--")
    n_pub = len([c for c in courses if not c.startswith("seed")])
    ax.text((n_pub + len(courses) - 1) / 2, 77.5, "randomly generated courses", ha="center", fontsize=8,
            color="#57606a")
    ax.text((n_pub - 1) / 2, 77.5, "public practice courses", ha="center", fontsize=8, color="#57606a")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=3)
    save(fig, "results_by_course.png")


def fig_track() -> None:
    course = "practice_technical"
    trk = json.loads((DATA / "tracks" / f"{course}.json").read_text())
    perf, base = series(run(course, "performance")), series(run(course, "baseline"))
    fig, ax = plt.subplots(figsize=(11, 4.3))
    for side in ("left_boundary", "right_boundary"):
        b = np.asarray(trk[side])
        ax.plot(b[:, 0], b[:, 1], color="#c9d1d9", lw=1)
    for key, color, size in (("left", "#2f6fdb", 4), ("right", "#e3b505", 4), ("obstacle", "#f26b1d", 16)):
        pts = np.asarray(trk["cones"].get(key, []))
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=size, color=color, zorder=3,
                       label="obstacle cones" if key == "obstacle" else None)
    ax.plot(base["x"], base["y"], color=BASE, lw=1.4, ls="--", label="baseline path", zorder=2)
    pts = np.column_stack([perf["x"], perf["y"]])
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=2.6, zorder=4)
    lc.set_array(np.asarray(perf["speed"][:-1]) * 3.6)
    ax.add_collection(lc)
    hits = [e for e in base["events"] if e["kind"] == "cone_hit"]
    if hits:
        ax.scatter([e["x"] for e in hits], [e["y"] for e in hits], marker="x", s=60, color="#b42318", lw=2,
                   zorder=5, label=f"baseline cone hits ({len(hits)})")
    cb = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.01)
    cb.set_label("HackUAB speed (km/h)")
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    pm, bm = run(course, "performance")["metrics"], run(course, "baseline")["metrics"]
    ax.set_title(f"Technical course · HackUAB {pm['lap_time_s']:.2f} s, 0 cones   vs   "
                 f"baseline {bm['lap_time_s']:.2f} s, {int(bm['penalties']['cone_hits'])} cones")
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "racing_line_technical.png")


def fig_speed() -> None:
    course = "practice_technical"
    p, e, b = (series(run(course, m)) for m in ("performance", "eco", "baseline"))
    fig, ax = plt.subplots(figsize=(10, 3.8))
    prog = np.asarray(e["progress"])
    coast = np.asarray(e["coast"] or [0] * len(prog), dtype=float) > 0.5
    for i in np.flatnonzero(coast):
        ax.axvspan(prog[i], prog[min(i + 1, len(prog) - 1)], color=ECO, alpha=0.08, lw=0)
    ax.plot(b["progress"], np.asarray(b["speed"]) * 3.6, color=BASE, lw=1.4, label="baseline")
    ax.plot(p["progress"], np.asarray(p["speed"]) * 3.6, color=PERF, lw=1.8, label="PERFORMANCE")
    ax.plot(e["progress"], np.asarray(e["speed"]) * 3.6, color=ECO, lw=1.8, label="ECO (shaded: coasting)")
    ax.set_xlabel("Distance along the course (m)")
    ax.set_ylabel("Speed (km/h)")
    ax.set_title("Speed profile on the technical course")
    ax.legend(loc="upper right", ncol=3)
    save(fig, "speed_profile_technical.png")


def fig_gg() -> None:
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    for c in PUBLIC:
        r = run(c, "performance")
        if r:
            s = series(r)
            ax.scatter(np.asarray(s["ay"]) / G, np.asarray(s["ax"]) / G, s=3, color=PERF, alpha=0.25, lw=0)
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(0.95 * np.cos(th), 0.95 * np.sin(th), color="#b42318", lw=1.2, ls="--", label="tyre limit (μ ≈ 0.95)")
    ax.set_xlim(-1.15, 1.15)
    ax.set_ylim(-1.15, 1.15)
    ax.set_aspect("equal")
    ax.set_xlabel("Lateral acceleration (g)")
    ax.set_ylabel("Longitudinal acceleration (g)")
    ax.set_title("g-g diagram · all public courses")
    ax.legend(loc="lower right", fontsize=8)
    save(fig, "gg_diagram.png")


def fig_compute() -> None:
    times = []
    for c in [c["id"] for c in INDEX["courses"]]:
        r = run(c, "performance")
        if r:
            t = series(r)["t_total_ms"]
            times += [v for v in (t or []) if v is not None]
    times = np.asarray(times)
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.hist(times, bins=np.linspace(0, 52, 105), color=PERF)
    ax.axvline(50, color="#b42318", lw=1.5)
    ax.text(49.3, ax.get_ylim()[1] * 0.92, "50 ms deadline", ha="right", color="#b42318", fontsize=9)
    p50, p99 = np.percentile(times, [50, 99])
    ax.axvline(p99, color=INK, lw=1, ls=":")
    ax.text(p99 + 0.6, ax.get_ylim()[1] * 0.72, f"p99 {p99:.1f} ms", fontsize=9)
    ax.set_xlabel("Controller compute time per decision (ms)")
    ax.set_ylabel("Sampled decisions")
    ax.set_title(f"Compute time per decision · median {p50:.1f} ms (PERFORMANCE, all 10 courses)")
    ax.text(0.99, 0.55, f"{len(times):,} decisions sampled at 10 Hz", transform=ax.transAxes, ha="right",
            fontsize=8, color="#57606a")
    save(fig, "compute_time.png")


def fig_energy() -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [1, 1.25]})
    parts = [("drag", "Aerodynamic drag"), ("rolling", "Rolling resistance"), ("tyre_scrub", "Tyre scrub"),
             ("drive_loss", "Drivetrain loss"), ("brake_dissipated", "Friction braking")]
    colors = ["#8fb7f2", "#c7d9f5", "#f2c38f", "#d6a5e8", "#f08a8a"]
    totals = {}
    for i, mode in enumerate(("performance", "eco")):
        bd = {k: 0.0 for k, _ in parts}
        bd["regen_recovered"], total = 0.0, 0.0
        for c in PUBLIC:
            m = run(c, mode)["metrics"]
            total += m["energy_wh"]
            for k in bd:
                bd[k] += m["energy_breakdown_wh"].get(k, 0.0)
        totals[mode] = total
        bottom = 0.0
        for (k, name), col in zip(parts, colors):
            ax1.bar(i, bd[k], 0.55, bottom=bottom, color=col, label=name if i == 0 else None)
            bottom += bd[k]
        ax1.bar(i, -bd["regen_recovered"], 0.55, color="#7fd1a4", label="Regen recovered" if i == 0 else None)
        ax1.text(i, bottom + 25, f"{total:,.0f} Wh\nfrom battery", ha="center", fontsize=9, fontweight="bold")
    ax1.axhline(0, color=INK, lw=0.8)
    ax1.set_xticks([0, 1], ["PERFORMANCE", "ECO"])
    ax1.set_ylabel("Energy over the 5 public courses (Wh)")
    saving = 1 - totals["eco"] / totals["performance"]
    ax1.set_title(f"Where the energy goes · ECO uses {saving:.0%} less")
    ax1.legend(fontsize=7, loc="upper right")
    ax1.set_ylim(top=ax1.get_ylim()[1] * 1.12)

    course = "practice_technical"
    sweep = [(w, run(course, "performance" if w == 0 else "eco", eco=w)) for w in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)]
    sweep = [(w, r["metrics"]) for w, r in sweep if r]
    ax2.plot([m["lap_time_s"] for _, m in sweep], [m["wh_per_km"] for _, m in sweep], color="#c9d1d9", lw=1.5,
             zorder=1)
    for w, m in sweep:
        final = w in (0.0, 0.8)
        col = PERF if w == 0 else (ECO if w == 0.8 else "#9aa1aa")
        ax2.scatter(m["lap_time_s"], m["wh_per_km"], s=70 if final else 30, color=col, zorder=3,
                    edgecolor="white", lw=1)
        ax2.annotate(f"eco {w:g}" + (" · PERFORMANCE" if w == 0 else " · ECO preset" if w == 0.8 else ""),
                     (m["lap_time_s"], m["wh_per_km"]), xytext=(4, -14) if w == 0 else (6, 4),
                     textcoords="offset points", fontsize=8)
    ax2.set_xlabel("Lap time (s)")
    ax2.set_ylabel("Energy (Wh/km)")
    ax2.set_title("Lap time vs energy · one knob (technical course)")
    ax2.text(0.99, 0.02, "grey points: earlier controller version (parameter sweep)", transform=ax2.transAxes,
             ha="right", fontsize=7, color="#57606a")
    fig.text(0.01, -0.03, "Energy is estimated from the simulator's forces with assumed 90 % drive efficiency, "
             "65 % regeneration efficiency and a 60 kW regen cap (not vehicle specifications).",
             fontsize=7.5, color="#57606a")
    save(fig, "energy_eco.png")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for f in (fig_scores, fig_track, fig_speed, fig_gg, fig_compute, fig_energy):
        f()
