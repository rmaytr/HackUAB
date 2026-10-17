"""One line per run: time, technical points, path error, category scores (dev only).

  python ..\\tools\\runs_table.py RUN_DIR_OR_PARENT [...]
A directory without summary.json is treated as a parent (e.g. a compare/selftest folder).
"""
import json
import os
import sys

print(f"{'run':44} {'time':>7} {'tech':>5} {'perr':>5} {'time%':>6} {'dev%':>5} {'dyn%':>5} {'cones':>5}")
for arg in sys.argv[1:]:
    dirs = [arg] if os.path.exists(os.path.join(arg, "summary.json")) else \
        sorted(os.path.join(arg, d) for d in os.listdir(arg) if os.path.exists(os.path.join(arg, d, "summary.json")))
    for d in dirs:
        s = json.load(open(os.path.join(d, "summary.json")))
        m, sc = s["metrics"], s["score"]
        cs = sc["category_scores"]
        t = m["completion"]["time_s"]
        print(f"{os.path.relpath(d)[-44:]:44} {t if t else float('nan'):7.2f} {sc['technical_points']:5.1f} "
              f"{m['deviation']['path_err_mean_m']:5.2f} {cs['completion_time']:6.1f} {cs['deviation']:5.1f} "
              f"{cs['dynamics']:5.1f} {sc['deductions'].get('cone_hit', 0) // 2:5d}")
