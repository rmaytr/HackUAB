"""Dev-only harness for hackuab/tracker.py (NOT submitted).

Drives tracker.track() with a stand-in Plan: the perceived centerline and a friction-limited
speed profile with the backward/forward passes and a horizon-end guard. It ignores obstacles
(that is horizon.py's job), so obstacle cone hits are expected, like the baseline.

From simulator/:
  .venv\\Scripts\\python.exe -m autocross run -c ..\\tools\\tracker_dev_controller.py -t tracks\\practice_technical.yaml --backend bicycle --no-render --team tracker_dev
"""
from __future__ import annotations

import os
import sys
import time
from types import SimpleNamespace

import numpy as np

from autocross.api import Command, Controller, Observation, TrackInfo, VehicleParams

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hackuab"))
from tracker import track  # noqa: E402

G = 9.81


def stand_in_plan(road, v_now: float, vehicle: VehicleParams, a_lat: float, v_horizon: float):
    s = np.asarray(road.preview_s, dtype=float)
    k = np.asarray(road.preview_curvature, dtype=float)
    v = np.minimum(np.sqrt(a_lat / np.maximum(np.abs(k), 1e-4)), vehicle.max_speed)
    v[-1] = min(v[-1], v_horizon)
    a_dec, a_acc = 0.8 * vehicle.max_decel, vehicle.max_accel
    # backward pass: v_i^2 = min_{j>=i}(v_j^2 + 2 a s_j) - 2 a s_i
    b = np.minimum.accumulate((v * v + 2 * a_dec * s)[::-1])[::-1] - 2 * a_dec * s
    # forward pass from the current speed
    f = np.minimum.accumulate(np.minimum(b - 2 * a_acc * s, v_now * v_now)) + 2 * a_acc * s
    v = np.sqrt(np.maximum(np.minimum(b, f), 0.0))
    return SimpleNamespace(path=np.asarray(road.preview_centerline, dtype=float), s=s, kappa=k, v=v)


class TrackerDev(Controller):
    def on_start(self, track_info: TrackInfo, vehicle: VehicleParams) -> None:
        super().on_start(track_info, vehicle)
        self.state: dict = {}
        # tracker overrides via --controller-config (keys under `tracker:`), for tuning sweeps
        self.params = SimpleNamespace(**self.config.get("tracker", {}))

    def step(self, obs: Observation) -> Command:
        road, vp = obs.road, self.vehicle
        if road is None or not road.valid:
            return Command(brake=1.0)
        t0 = time.perf_counter()
        a_lat = float(self.config.get("lat_frac", 0.70)) * vp.mu * G * self.state.get("a_lat_scale", 1.0)
        # horizon guard: be able to take a ~7 m radius corner just past the sight limit
        plan = stand_in_plan(road, obs.speed, vp, a_lat, v_horizon=float(np.sqrt(a_lat * 7.0)))
        steer, throttle, brake = track(plan, obs, vp, self.params, self.state)
        dbg = dict(self.state.get("debug", {}))
        dbg["t_track_ms"] = (time.perf_counter() - t0) * 1e3
        dbg["k_abs_max10"] = float(np.max(np.abs(plan.kappa[plan.s < 10.0])))
        dbg["plan_v0"] = float(plan.v[0])
        dbg["plan_vmin"] = float(np.min(plan.v))
        dbg["a_lat_plan"] = a_lat
        dbg["road_lat"] = road.lateral_offset
        dbg["road_hdg"] = road.heading_error
        dbg["road_k"] = road.curvature
        return Command(steer=steer, throttle=throttle, brake=brake, path=plan.path, path_frame="vehicle",
                       target_speed=dbg.get("v_target"), debug=dbg)
