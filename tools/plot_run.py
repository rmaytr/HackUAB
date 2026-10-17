"""Demo figures for one course: ours vs the baseline (dev only, not submitted). Run from simulator/:

  python ..\\tools\\plot_run.py OURS_RUN BASELINE_RUN [--track tracks\\practice_technical.yaml] [--out ..\\figures]

Writes <course>_<backend>_{map,speed,gg,steptime}.png plus a 2x2 <course>_<backend>_slide.png.
Colours: ours = accent blue, baseline = grey (CLAUDE.md visualisation brief); speed on the map is
a single-hue blue ramp. Static PNGs for slides, so every value worth reading is labelled.
"""
from __future__ import annotations

import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

# reference palette (dataviz skill): slot-1 blue, its sequential ramp, chrome and ink
OURS = "#2a78d6"
PLAN = "#86b6ef"
BASE = "#898781"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
SPEED_CMAP = LinearSegmentedColormap.from_list(
    "speed", ["#b7d3f6", "#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"])
G = 9.81

plt.rcParams.update({
    "font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"], "font.size": 10,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.linewidth": 0.8,
    "axes.facecolor": SURFACE, "figure.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
})


def load(run):
    d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
    s = json.load(open(os.path.join(run, "summary.json")))
    return d, s


def label(s):
    c = s["metrics"]["completion"]
    t = f"{c['time_s']:.1f} s" if c["time_s"] else "DNF"
    return f"{t} · {s['score']['technical_points']:.1f}/75"


def fig_map(ax, d_o, d_b, track, s_o, s_b):
    if track is not None:
        ax.plot(track.left[:, 0], track.left[:, 1], color=AXIS, lw=0.8)
        ax.plot(track.right[:, 0], track.right[:, 1], color=AXIS, lw=0.8)
        ax.plot(track.center[:, 0], track.center[:, 1], color=GRID, lw=0.8)
        oc = np.asarray(track.obstacle_cones[0] if isinstance(track.obstacle_cones, tuple) else track.obstacle_cones)
        if oc.size:
            ax.scatter(oc[:, 0], oc[:, 1], s=10, color=INK2, zorder=4, label="obstacle cones")
    ax.plot(d_b["x"], d_b["y"], color=BASE, lw=1.0, zorder=2, label=f"baseline  {label(s_b)}")
    pts = np.column_stack([d_o["x"], d_o["y"]])
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=SPEED_CMAP, linewidths=2.0, zorder=3)
    lc.set_array(d_o["speed"][:-1])
    ax.add_collection(lc)
    ax.plot([], [], color=OURS, lw=2, label=f"ours  {label(s_o)}")
    cb = plt.colorbar(lc, ax=ax, fraction=0.035, pad=0.02, shrink=0.6)
    cb.set_label("our speed (m/s)", color=INK2)
    cb.outline.set_visible(False)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="best", fontsize=9)


def fig_speed(ax, d_o, d_b):
    ax.plot(d_b["progress"], d_b["speed"], color=BASE, lw=1.4, label="baseline")
    ax.plot(d_o["progress"], d_o["target_speed"], color=PLAN, lw=1.2, ls="--", label="our plan")
    ax.plot(d_o["progress"], d_o["speed"], color=OURS, lw=2.0, label="ours")
    ax.set_xlabel("distance along course (m)")
    ax.set_ylabel("speed (m/s)")
    ax.set_ylim(bottom=0)
    ax.legend(loc="lower right", ncol=3, fontsize=9)


def inside_ellipse(d, vp, frac):
    """Share of moving samples inside the ellipse drawn in fig_gg."""
    mov = d["speed"] > 3
    lon = np.where(d["ax"][mov] >= 0, vp["max_accel"], vp["max_decel"])
    r = (d["ay"][mov] / (frac * vp["mu"] * G)) ** 2 + (d["ax"][mov] / lon) ** 2
    return float((r <= 1.0).mean())


def launch_split(d_o, d_b, start, end, launch_m=60.0):
    """(gain over the first launch_m metres after the start line, total gain), seconds, over the
    timed stretch [start, end] of progress, so it matches the official lap-time difference."""
    def t_at(d, p):
        keep = np.r_[True, np.diff(d["progress"]) > 1e-3]
        return float(np.interp(p, d["progress"][keep], d["t"][keep]))
    end = min(end, d_o["progress"].max(), d_b["progress"].max())
    launch_m = start + launch_m
    g_launch = (t_at(d_b, launch_m) - t_at(d_b, start)) - (t_at(d_o, launch_m) - t_at(d_o, start))
    g_total = (t_at(d_b, end) - t_at(d_b, start)) - (t_at(d_o, end) - t_at(d_o, start))
    return g_launch, g_total


def fig_gg(ax, d_o, d_b, vp, frac):
    """Lateral vs longitudinal acceleration against the friction ellipse the tracker enforces:
    lateral semi-axis frac*mu*g, longitudinal max_accel (up) / max_decel (down)."""
    mov_o, mov_b = d_o["speed"] > 3, d_b["speed"] > 3
    ax.scatter(d_b["ay"][mov_b], d_b["ax"][mov_b], s=4, color=BASE, alpha=0.35, lw=0, label="baseline")
    ax.scatter(d_o["ay"][mov_o], d_o["ax"][mov_o], s=4, color=OURS, alpha=0.45, lw=0, label="ours")
    th = np.linspace(0, 2 * np.pi, 361)
    a_lat = frac * vp["mu"] * G
    lon = np.where(np.sin(th) >= 0, vp["max_accel"], vp["max_decel"])
    ax.plot(a_lat * np.cos(th), lon * np.sin(th), color=INK2, lw=1.2,
            label=f"grip limit: {frac:.2f} μg lateral,\n{vp['max_accel']:.1f} accel / {vp['max_decel']:.0f} brake m/s²")
    ax.axhline(0, color=AXIS, lw=0.8, zorder=0)
    ax.axvline(0, color=AXIS, lw=0.8, zorder=0)
    ax.set_xlabel("lateral acceleration (m/s²)")
    ax.set_ylabel("longitudinal acceleration (m/s²)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper right", fontsize=8, markerscale=3)


def fig_steptime(ax, d_o, s_o, deadline):
    ctrl = d_o["ctrl_step"] > 0
    ms = d_o["compute_ms"][ctrl]
    ms = ms[np.isfinite(ms)]
    p99 = s_o["metrics"]["compute"]["step_ms_p99"]
    ax.hist(ms, bins=np.linspace(0, max(10.0, p99 * 2.5), 40), color=OURS, rwidth=0.85)
    ax.axvline(p99, color=INK, lw=1.0)
    ax.text(p99, ax.get_ylim()[1] * 0.92, f"  p99 {p99:.1f} ms", color=INK, va="top", fontsize=9)
    ax.set_xlabel("controller step time (ms)")
    ax.set_ylabel("steps")
    ax.text(0.98, 0.92, f"deadline {deadline:.0f} ms\n(off the chart: {deadline / p99:.0f}× p99)",
            transform=ax.transAxes, ha="right", va="top", color=INK2, fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("base")
    ap.add_argument("--track", help="course YAML, for boundaries and obstacle cones on the map")
    ap.add_argument("--out", default=os.path.join("..", "figures"))
    ap.add_argument("--ellipse-frac", type=float, default=0.85, help="tracker's ellipse_lat_frac")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    d_o, s_o = load(a.ours)
    d_b, s_b = load(a.base)
    track = None
    if a.track:
        from autocross import track as trk
        track = trk.load(a.track)
    vp = s_o["vehicle"]
    deadline = s_o["metrics"]["compute"]["deadline_ms"]
    name = (track.name if track else os.path.basename(a.ours)) + "_" + s_o["backend"]
    t_o, t_b = s_o["metrics"]["completion"]["time_s"], s_b["metrics"]["completion"]["time_s"]
    where = f"{track.name if track else ''} on {s_o['backend'].upper() if s_o['backend'] == 'carla' else 'the bicycle model'}"
    map_title = (f"Ours {t_o:.1f} s vs baseline {t_b:.1f} s ({t_o - t_b:+.1f} s)" if (t_o and t_b)
                 else "Driven line, coloured by our speed")
    start, end = (track.start_offset, track.start_offset + track.length * track.laps) if track else (0.0, 1e9)
    g_launch, g_total = launch_split(d_o, d_b, start, end)
    speed_title = (f"{g_launch:.1f} s of the {g_total:.1f} s gain comes from the launch; the rest from corner exits"
                   if g_total > 0 else f"Speed along the course (net {g_total:+.1f} s)")
    pin_o, pin_b = inside_ellipse(d_o, vp, a.ellipse_frac), inside_ellipse(d_b, vp, a.ellipse_frac)
    gg_title = f"{pin_o:.1%} of our samples sit inside the grip limit (baseline {pin_b:.1%})"

    figs = {
        "map": (fig_map, (d_o, d_b, track, s_o, s_b), map_title),
        "speed": (fig_speed, (d_o, d_b), speed_title),
        "gg": (fig_gg, (d_o, d_b, vp, a.ellipse_frac), gg_title),
        "steptime": (fig_steptime, (d_o, s_o, deadline),
                     f"Step time p99 {s_o['metrics']['compute']['step_ms_p99']:.1f} ms against a {deadline:.0f} ms budget"),
    }
    sizes = {"map": (9, 7), "speed": (10, 4), "gg": (6, 6), "steptime": (7, 4)}
    for key, (fn, args, title) in figs.items():
        fig, ax = plt.subplots(figsize=sizes[key], constrained_layout=True)
        fn(ax, *args)
        ax.set_title(title)
        path = os.path.join(a.out, f"{name}_{key}.png")
        fig.savefig(path, dpi=160)
        plt.close(fig)
        print(path)

    fig, axs = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True,
                            gridspec_kw={"width_ratios": [1.3, 1], "height_ratios": [1.3, 1]})
    for ax, key in zip(axs.flat, ["map", "gg", "speed", "steptime"]):
        fn, args, title = figs[key]
        fn(ax, *args)
        ax.set_title(title, fontsize=11)
    fig.suptitle(f"HackUAB controller vs baseline · {where}", fontsize=14, fontweight="bold", x=0.01, ha="left")
    path = os.path.join(a.out, f"{name}_slide.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    main()
