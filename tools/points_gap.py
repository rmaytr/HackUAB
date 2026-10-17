"""Where are the points lost? Per run: category points vs the maximum, and the lap time needed for
full completion-time marks (dev only).  python ..\\tools\\points_gap.py RUN_DIR_OR_PARENT [...]"""
import json
import os
import sys

import yaml

sc = yaml.safe_load(open(os.path.join("config", "scoring.yaml")))
W = sc["weights"]
full_ratio, zero_ratio = sc["time"]["full_marks_ratio"], sc["time"]["zero_marks_ratio"]
print(f"{'run':40} {'lap':>6} {'ref':>6} {'need':>6} {'time':>5} {'dev':>5} {'cpu':>5} {'dyn':>5} {'pen':>5} "
      f"{'total':>5}  lost: time/dev/cpu/dyn/pen")
tot = {k: 0.0 for k in ("time", "dev", "cpu", "dyn", "pen", "total")}
n = 0
for arg in sys.argv[1:]:
    dirs = [arg] if os.path.exists(os.path.join(arg, "summary.json")) else sorted(
        os.path.join(arg, d) for d in os.listdir(arg) if os.path.exists(os.path.join(arg, d, "summary.json")))
    for d in dirs:
        s = json.load(open(os.path.join(d, "summary.json")))
        cp, c = s["score"]["category_points"], s["metrics"]["completion"]
        pen = sum(s["score"]["deductions"].values())
        ref = c["reference_time_s"]
        row = {"time": cp["completion_time"], "dev": cp["deviation"], "cpu": cp["compute"], "dyn": cp["dynamics"],
               "pen": -pen, "total": s["score"]["technical_points"]}
        for k in tot:
            tot[k] += row[k]
        n += 1
        lost = (W["completion_time"] - row["time"], W["deviation"] - row["dev"], W["compute"] - row["cpu"],
                W["dynamics"] - row["dyn"], pen)
        print(f"{os.path.basename(d)[-40:]:40} {c['time_s'] or 0:6.1f} {ref:6.1f} {full_ratio * ref:6.1f} "
              f"{row['time']:5.1f} {row['dev']:5.1f} {row['cpu']:5.1f} {row['dyn']:5.1f} {row['pen']:5.1f} "
              f"{row['total']:5.1f}  " + "/".join(f"{x:.1f}" for x in lost))
if n:
    print(f"{'MEAN':40} {'':6} {'':6} {'':6} {tot['time']/n:5.1f} {tot['dev']/n:5.1f} {tot['cpu']/n:5.1f} "
          f"{tot['dyn']/n:5.1f} {tot['pen']/n:5.1f} {tot['total']/n:5.1f}")
    print(f"max: time {W['completion_time']}, dev {W['deviation']}, cpu {W['compute']}, dyn {W['dynamics']}  "
          f"(time: full marks at <= {full_ratio} x ref, zero at {zero_ratio} x ref)")
