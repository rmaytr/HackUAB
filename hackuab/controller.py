"""HackUAB EcoCAR autocross controller: entry point the harness loads. Owner: Viraj.

    obs -> horizon.build_corridor -> planner.plan -> tracker.track -> Command

Every stage is timed into Command.debug (t_*_ms). If any stage raises or returns garbage, this step
falls back to a self-contained centerline pure pursuit (debug fallback=1), so a bug in one module
costs speed, never a DNF. Run with `--controller-config strict.yaml` containing `strict: true` to
re-raise instead, so bugs are visible while developing.
"""
from __future__ import annotations

import math
from time import perf_counter

import numpy as np

from autocross.api import Command, Controller, Observation, TrackInfo, VehicleParams
from horizon import build_corridor
from params import Params
from planner import plan
from tracker import track


class HackUABController(Controller):
    def on_start(self, track_info: TrackInfo, vehicle: VehicleParams) -> None:
        super().on_start(track_info, vehicle)
        self.strict = bool(self.config.get("strict", False))
        self.p = Params.from_vehicle(vehicle, {k: v for k, v in self.config.items() if k != "strict"})
        self.state: dict = {}
        self.n_fallback = 0
        self.n_over_budget = 0

    def step(self, obs: Observation) -> Command:
        road = obs.road
        if road is None or not road.valid or len(road.preview_s) < 2:
            return Command(brake=1.0, debug={"fallback": 1.0})

        t0 = perf_counter()
        dbg: dict[str, float] = {}
        try:
            corridor = build_corridor(road, self.vehicle, self.p)
            t1 = perf_counter()
            pl = plan(corridor, obs, self.vehicle, self.p)
            t2 = perf_counter()
            steer, throttle, brake = track(pl, obs, self.vehicle, self.p, self.state)
            t3 = perf_counter()
            path, v_target = pl.path, self.state.get("v_target")
            if not (np.all(np.isfinite(path)) and math.isfinite(steer + throttle + brake)):
                raise FloatingPointError("non-finite output")
            dbg.update({"t_horizon_ms": (t1 - t0) * 1e3, "t_plan_ms": (t2 - t1) * 1e3,
                        "t_track_ms": (t3 - t2) * 1e3, "fallback": 0.0})
            dbg.update(self.state.get("debug", {}))
        except Exception:
            if self.strict:
                raise
            self.n_fallback += 1
            steer, throttle, brake, path, v_target = self._fallback(obs)
            dbg["fallback"] = 1.0

        steer = float(np.clip(np.nan_to_num(steer), -1.0, 1.0))
        throttle = float(np.clip(np.nan_to_num(throttle), 0.0, 1.0))
        brake = float(np.clip(np.nan_to_num(brake), 0.0, 1.0))
        if brake > 0.0:
            throttle = 0.0                                  # pedals are mutually exclusive

        t_total = (perf_counter() - t0) * 1e3
        self.n_over_budget += t_total > self.p.budget_ms
        dbg.update({"t_total_ms": t_total, "n_fallback": float(self.n_fallback),
                    "n_over_budget": float(self.n_over_budget)})
        return Command(steer=steer, throttle=throttle, brake=brake, path=path, path_frame="vehicle",
                       target_speed=v_target, debug={k: float(v) for k, v in dbg.items()})

    def _fallback(self, obs: Observation):
        """Conservative centerline pure pursuit that depends on no other module."""
        road, vp = obs.road, self.vehicle
        cl, s = np.asarray(road.preview_centerline, float), np.asarray(road.preview_s, float)
        v = max(obs.speed, 0.0)
        ld = max(4.0, 0.5 * v + 2.0)
        tx, ty = float(np.interp(ld, s, cl[:, 0])), float(np.interp(ld, s, cl[:, 1]))
        alpha = math.atan2(ty, max(tx, 0.1))
        steer = math.atan2(2.0 * vp.wheelbase * math.sin(alpha), max(math.hypot(tx, ty), 1.0)) / vp.max_steer
        k_max = float(np.max(np.abs(road.preview_curvature))) if len(road.preview_curvature) else 0.0
        v_tgt = min(math.sqrt(0.5 * vp.mu * 9.81 / max(k_max, 1e-3)), 12.0)
        u = 0.5 * (v_tgt - v)
        throttle, brake = (min(u, 0.5), 0.0) if u >= 0 else (0.0, min(-0.6 * u, 1.0))
        self.state.pop("v_ref", None)                       # re-seed the tracker's ramp when it recovers
        return steer, throttle, brake, cl, v_tgt
