"""List a run's penalty events with where they happened (dev only).

  python ..\\tools\\events.py RUN_DIR [kind ...]       default kinds: cone_hit off_course missed_gate dnf
  RUN_DIR may be 'latest'.
"""
import glob
import json
import os
import sys

import numpy as np

args = sys.argv[1:]
run = args.pop(0)
if run == "latest":
    run = max(glob.glob(os.path.join("runs", "*")), key=os.path.getmtime)
kinds = set(args or ["cone_hit", "off_course", "missed_gate", "dnf"])
ev = json.load(open(os.path.join(run, "events.json"), encoding="utf-8-sig"))
d = np.genfromtxt(os.path.join(run, "telemetry.csv"), delimiter=",", names=True)
print(run)
for e in ev:
    if e["kind"] not in kinds:
        continue
    i = int(np.searchsorted(d["t"], e["t"]))
    i = min(i, len(d) - 1)
    det = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in e.get("detail", {}).items()}
    print(f"{e['kind']:12} t={e['t']:6.2f}  progress={d['progress'][i]:6.1f}  v={d['speed'][i]:5.1f}  "
          f"lateral={d['lateral'][i]:+5.2f}  {det}")
