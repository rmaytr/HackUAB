"""HackUAB EcoCAR autocross controller: entry point the harness loads. Owner: Viraj.

    obs -> horizon.build_corridor -> planner.plan -> tracker.track -> Command

Every stage is timed into Command.debug (t_*_ms). Failures degrade one level at a time, so a bug
in one module costs speed, never a DNF (debug `fallback`):
  0  full pipeline
  1  horizon or planner raised, returned garbage, or the previous step ran over budget: the tracker
     drives a conservative centerline plan built here (no dependency on horizon.py / planner.py)
  2  the tracker itself failed: self-contained centerline pure pursuit
Run with `--controller-config strict.yaml` containing `strict: true` to re-raise instead, so bugs
are visible while developing.

The tracker's grip estimate (state["a_lat_scale"], from its slip adaptation) scales the planner's
cornering limit p.a_lat every step, so the speed plan backs off when the tyres start to slide.
"""
from __future__ import annotations

import math
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from autocross.api import Command, Controller, Observation, TrackInfo, VehicleParams
from horizon import build_corridor
from params import G, Params
from planner import plan
from tracker import track


class HackUABController(Controller):
    def on_start(self, track_info: TrackInfo, vehicle: VehicleParams) -> None:
        super().on_start(track_info, vehicle)
        self.strict = bool(self.config.get("strict", False))
        self.p = Params.from_vehicle(vehicle, {k: v for k, v in self.config.items() if k != "strict"})
        self.a_lat_base = self.p.a_lat
        self.state: dict = {}
        self.n_fallback = [0, 0, 0]          # steps at each fallback level
        self.n_over_budget = 0
        self.over_budget = False

    def step(self, obs: Observation) -> Command:
        road = obs.road
        if road is None or not road.valid or len(road.preview_s) < 2:
            return Command(brake=1.0, debug={"fallback": 2.0})

        t0 = perf_counter()
        dbg: dict[str, float] = {}
        level = 0
        scale = float(self.state.get("a_lat_scale", 1.0))
        self.p.a_lat = self.a_lat_base * scale

        # stages 1 + 2: corridor and plan, or the cheap centerline plan
        pl = None
        if not self.over_budget:
            try:
                corridor = build_corridor(road, self.vehicle, self.p)
                t1 = perf_counter()
                pl = plan(corridor, obs, self.vehicle, self.p)
                t2 = perf_counter()
                if not _plan_ok(pl):
                    raise FloatingPointError("invalid plan")
                dbg.update({"t_horizon_ms": (t1 - t0) * 1e3, "t_plan_ms": (t2 - t1) * 1e3})
                dbg.update(getattr(pl, "debug", None) or {})        # planner diagnostics (pred_*)
            except Exception:
                if self.strict:
                    raise
                pl = None
        if pl is None:
            level = 1
            t2 = perf_counter()
            pl = _centerline_plan(road, obs.speed, self.vehicle, scale)

        # stage 3: tracker, or the last-resort pure pursuit
        try:
            steer, throttle, brake = track(pl, obs, self.vehicle, self.p, self.state)
            if not math.isfinite(steer + throttle + brake):
                raise FloatingPointError("non-finite command")
            path, v_target = pl.path, self.state.get("v_target")
            dbg["t_track_ms"] = (perf_counter() - t2) * 1e3
            dbg.update(self.state.get("debug", {}))
        except Exception:
            if self.strict:
                raise
            level = 2
            steer, throttle, brake, path, v_target = self._fallback(obs)

        steer = float(np.clip(np.nan_to_num(steer), -1.0, 1.0))
        throttle = float(np.clip(np.nan_to_num(throttle), 0.0, 1.0))
        brake = float(np.clip(np.nan_to_num(brake), 0.0, 1.0))
        if brake > 0.0:
            throttle = 0.0                                  # pedals are mutually exclusive

        t_total = (perf_counter() - t0) * 1e3
        self.over_budget = t_total > self.p.budget_ms       # next step skips the planner if so
        self.n_over_budget += self.over_budget
        self.n_fallback[level] += 1
        dbg.update({"t_total_ms": t_total, "fallback": float(level), "n_fallback": float(sum(self.n_fallback[1:])),
                    "n_over_budget": float(self.n_over_budget), "a_lat_plan": self.p.a_lat,
                    "kappa_max": float(np.max(np.abs(pl.kappa))) if len(pl.kappa) else 0.0})
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
        v_tgt = min(math.sqrt(0.5 * vp.mu * G / max(k_max, 1e-3)), 12.0)
        u = 0.5 * (v_tgt - v)
        throttle, brake = (min(u, 0.5), 0.0) if u >= 0 else (0.0, min(-0.6 * u, 1.0))
        # keep the tracker's rate limiters continuous so handing back to it does not jerk the car
        self.state["steer_prev"] = float(np.clip(steer, -1.0, 1.0))
        self.state["a_prev"] = vp.max_accel * throttle if throttle > 0 else -vp.max_decel * brake
        self.state["i_err"] = 0.0
        return steer, throttle, brake, cl, v_tgt


def _plan_ok(pl) -> bool:
    n = len(pl.s)
    return (n >= 2 and pl.path.shape == (n, 2) and len(pl.kappa) == n and len(pl.v) == n
            and bool(np.all(np.isfinite(pl.path))) and bool(np.all(np.isfinite(pl.v)))
            and bool(np.all(np.diff(pl.s) > 0)))


def _centerline_plan(road, v_now: float, vehicle: VehicleParams, scale: float):
    """Level-1 plan: the perceived centerline with a conservative friction-limited speed profile
    (0.5 g x grip estimate, 0.5 x max braking, 6 m/s at the end of the horizon). Self-contained."""
    s = np.asarray(road.preview_s, float)
    k = np.nan_to_num(np.asarray(road.preview_curvature, float))
    a_lat, a_dec = 0.5 * vehicle.mu * G * scale, 0.5 * vehicle.max_decel
    v = np.minimum(np.sqrt(a_lat / np.maximum(np.abs(k), 1e-4)), min(vehicle.max_speed, 20.0))
    v[-1] = min(v[-1], 6.0)
    # backward pass, O(K): v_i^2 = min_{j>=i}(v_j^2 + 2 a s_j) - 2 a s_i
    v2 = np.minimum.accumulate((v * v + 2.0 * a_dec * s)[::-1])[::-1] - 2.0 * a_dec * s
    return SimpleNamespace(path=np.nan_to_num(np.asarray(road.preview_centerline, float)), s=s, kappa=k,
                           v=np.sqrt(np.maximum(v2, 0.0)))
