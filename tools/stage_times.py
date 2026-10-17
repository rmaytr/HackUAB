"""p50 / p99 of each pipeline stage's time (from controller.py's dbg_t_*_ms columns) for runs (dev only).

  python ..\\tools\\stage_times.py RUN_DIR [...]
"""
import os
import sys

import numpy as np

cols = ["dbg_t_horizon_ms", "dbg_t_plan_ms", "dbg_t_track_ms", "dbg_t_total_ms", "compute_ms"]
print(f"{'run':44} " + " ".join(f"{c.replace('dbg_t_', '').replace('_ms', ''):>15}" for c in cols))
for run in sys.argv[1:]:
    d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
    ctrl = d["ctrl_step"] > 0
    cells = []
    for c in cols:
        x = d[c][ctrl] if c in d.dtype.names else np.array([np.nan])
        x = x[np.isfinite(x)]
        cells.append(f"{np.percentile(x, 50):6.2f} / {np.percentile(x, 99):6.2f}" if len(x) else f"{'-':>15}")
    print(f"{os.path.relpath(run)[-44:]:44} " + " ".join(f"{c:>15}" for c in cells))
