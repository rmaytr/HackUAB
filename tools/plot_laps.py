"""Demo figure: lap time saved against the baseline on every selftest run (dev only). From simulator/:

  python ..\\tools\\plot_laps.py OURS_SELFTEST_DIR BASELINE_SELFTEST_DIR [--out ..\\figures]

One bar per course x backend (time saved, s), labelled with both lap times. Also writes a CSV
table view of the same numbers next to the PNG.
"""
from __future__ import annotations

import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import plot_run  # noqa: E402,F401  (shared rcParams and palette)
from plot_run import BASE, INK, INK2, OURS  # noqa: E402


def runs(folder):
    out = {}
    for name in os.listdir(folder):
        p = os.path.join(folder, name, "summary.json")
        if os.path.exists(p):
            s = json.load(open(p))
            out[name] = (s["metrics"]["completion"]["time_s"], s["score"]["technical_points"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("base")
    ap.add_argument("--out", default=os.path.join("..", "figures"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    o, b = runs(a.ours), runs(a.base)
    keys = sorted(set(o) & set(b), key=lambda k: (k.rsplit("_", 1)[1], k))     # group by backend
    rows = [(k, o[k][0], b[k][0], o[k][1], b[k][1]) for k in keys if o[k][0] and b[k][0]]
    saved = np.array([tb - to for _, to, tb, _, _ in rows])

    fig, ax = plt.subplots(figsize=(10, 0.5 * len(rows) + 1.6), constrained_layout=True)
    y = np.arange(len(rows))[::-1]
    ax.barh(y, saved, height=0.6, color=[OURS if v > 0 else BASE for v in saved])
    for yi, (k, to, tb, po, pb), v in zip(y, rows, saved):
        ax.text(max(v, 0) + 0.05, yi, f"{to:.1f} s vs {tb:.1f} s  ({v / tb:+.1%})".replace("+", "−"),
                va="center", color=INK2, fontsize=9)
    ax.set_yticks(y, [k.rsplit("_", 1)[0] + ("  · CARLA" if k.endswith("carla") else "  · bicycle") for k, *_ in rows])
    ax.axvline(0, color=INK, lw=0.8)
    ax.set_xlim(min(0, saved.min()) - 0.2, saved.max() * 1.9)
    ax.set_xlabel("lap time saved vs baseline (s)")
    ax.grid(axis="y", visible=False)
    ax.set_axisbelow(True)
    n_win = int((saved > 0).sum())
    ax.set_title(f"Faster than the baseline on {n_win} of {len(rows)} public runs "
                 f"(mean {saved.mean():.1f} s saved)")
    png = os.path.join(a.out, "lap_times.png")
    fig.savefig(png, dpi=160)
    with open(os.path.join(a.out, "lap_times.csv"), "w") as f:
        f.write("run,ours_s,baseline_s,saved_s,ours_points,baseline_points\n")
        for (k, to, tb, po, pb), v in zip(rows, saved):
            f.write(f"{k},{to:.2f},{tb:.2f},{v:.2f},{po:.1f},{pb:.1f}\n")
    print(png)


if __name__ == "__main__":
    main()
