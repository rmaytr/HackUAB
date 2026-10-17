"""Dev-only stand-in controller (NOT submitted): the simulator's baseline pure pursuit, but
steering toward the midline of the horizon.py corridor instead of the raw centerline.

Use it to test horizon.py in the simulator before hackuab/controller.py exists. From simulator/:
    .venv/bin/python -m autocross run -c ../tools/corridor_pp.py -t tracks/practice_technical.yaml --backend bicycle --no-render
(Windows: .venv\\Scripts\\python.exe and ..\\tools\\corridor_pp.py)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hackuab"))
sys.path.insert(0, str(ROOT / "simulator" / "controllers"))

from autocross.api import Command, Observation  # noqa: E402
from baseline_pure_pursuit import PurePursuitBaseline  # noqa: E402
from horizon import build_corridor  # noqa: E402


class CorridorPP(PurePursuitBaseline):
    def step(self, obs: Observation) -> Command:
        cmd = super().step(obs)                 # baseline speed control and fallback steering
        c = build_corridor(obs.road, self.vehicle, None)
        if not c.valid:
            return cmd
        off = 0.5 * (c.left - c.right)
        path = c.center + off[:, None] * c.normal
        v, vp = max(obs.speed, 0.0), self.vehicle
        ld = max(self.ld_min, self.k_ld * v + 2.0)
        tgt = np.array([np.interp(ld, c.s, path[:, 0]), np.interp(ld, c.s, path[:, 1])])
        alpha = math.atan2(tgt[1], max(tgt[0], 0.1))
        delta = math.atan2(2.0 * vp.wheelbase * math.sin(alpha), max(math.hypot(*tgt), 1.0))
        cmd.steer = float(np.clip(delta / vp.max_steer, -1.0, 1.0))
        cmd.path = path
        cmd.debug["corr_off_min"] = float(off.min())
        cmd.debug["corr_off_max"] = float(off.max())
        return cmd
