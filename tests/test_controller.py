"""controller.py fallback levels and outputs against a synthetic road (no simulator run needed).

From simulator/:  .venv\\Scripts\\python.exe -m pytest ..\\tests -q
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hackuab"))
import controller as ctl  # noqa: E402
from autocross.api import Observation, RoadModel, TrackInfo, VehicleParams  # noqa: E402

VEHICLE = VehicleParams(mass=1845.0, wheelbase=3.0, track_width=1.67, length=4.79, width=2.16,
                        cg_to_front=1.45, cg_to_rear=1.55, max_steer=math.radians(35), max_steer_rate=2.0,
                        mu=0.95, max_accel=5.2, max_decel=9.0, max_speed=40.0, control_dt=0.05,
                        actuation_delay_steps=0)
TRACK = TrackInfo("course", False, 1, np.zeros((0, 2)), np.zeros(0), np.zeros(0), np.zeros((0, 2)),
                  np.zeros(0, int), np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((0, 2, 2)), False)


def road(kappa=1 / 40.0, half_width=2.75):
    s = np.arange(0.0, 40.0 + 1e-9, 1.0)
    th = kappa * s
    cl = np.column_stack((np.sin(th) / kappa, (1 - np.cos(th)) / kappa))
    w = np.full_like(s, half_width)
    return RoadModel(True, 40.0, 0.0, 0.0, kappa, half_width, half_width, half_width, half_width,
                     s, np.full_like(s, kappa), cl, w, w.copy(), cl, cl, [], [], float("nan"))


def obs(speed=10.0, rd=None):
    return Observation(t=0.0, step=0, x=0.0, y=0.0, yaw=0.0, vx=speed, vy=0.0, yaw_rate=0.0, ax=0.0, ay=0.0,
                       speed=speed, steer=0.0, throttle=0.0, brake=0.0, progress=0.0, lap=0, timing_started=True,
                       cones=None, road=road() if rd is None else rd)


def make(config=None):
    c = ctl.HackUABController(config or {})
    c.on_start(TRACK, VEHICLE)
    return c


def check_command(cmd):
    assert -1.0 <= cmd.steer <= 1.0 and 0.0 <= cmd.throttle <= 1.0 and 0.0 <= cmd.brake <= 1.0
    assert not (cmd.throttle > 0 and cmd.brake > 0)
    assert cmd.path is not None and np.all(np.isfinite(cmd.path)) and len(cmd.path) >= 2
    assert all(math.isfinite(v) for v in cmd.debug.values())


def test_full_pipeline_is_level_0_and_turns_left():
    cmd = make().step(obs())
    check_command(cmd)
    assert cmd.debug["fallback"] == 0.0 and cmd.steer > 0.0


def test_planner_crash_falls_back_to_tracker_on_centerline(monkeypatch):
    monkeypatch.setattr(ctl, "plan", lambda *a: 1 / 0)
    cmd = make().step(obs())
    check_command(cmd)
    assert cmd.debug["fallback"] == 1.0 and "kappa_cmd" in cmd.debug      # tracker still drove


def test_horizon_returning_nan_falls_back_to_level_1(monkeypatch):
    real = ctl.build_corridor

    def bad(*a):
        c = real(*a)
        c.center = c.center * np.nan
        return c
    monkeypatch.setattr(ctl, "build_corridor", bad)
    cmd = make().step(obs())
    check_command(cmd)
    assert cmd.debug["fallback"] == 1.0


def test_tracker_crash_falls_back_to_level_2_and_recovers(monkeypatch):
    c = make()
    real = ctl.track
    monkeypatch.setattr(ctl, "track", lambda *a: 1 / 0)
    cmd = c.step(obs())
    check_command(cmd)
    assert cmd.debug["fallback"] == 2.0
    monkeypatch.setattr(ctl, "track", real)
    cmd = c.step(obs())
    assert cmd.debug["fallback"] == 0.0
    assert abs(cmd.steer - c.state["steer_prev"]) < 1e-9                      # rate limiter continuous


def test_over_budget_step_skips_planner_next_step():
    c = make({"budget_ms": -1.0})                                            # every step is "over"
    assert c.step(obs()).debug["fallback"] == 0.0
    assert c.step(obs()).debug["fallback"] == 1.0


def test_strict_mode_reraises(monkeypatch):
    monkeypatch.setattr(ctl, "plan", lambda *a: 1 / 0)
    with pytest.raises(ZeroDivisionError):
        make({"strict": True}).step(obs())


def test_slip_estimate_scales_planner_grip():
    c = make()
    c.state["a_lat_scale"] = 0.8
    cmd = c.step(obs())
    assert cmd.debug["a_lat_plan"] == pytest.approx(0.8 * c.a_lat_base)


def test_invalid_road_brakes():
    rd = road()
    rd.valid = False
    cmd = make().step(obs(rd=rd))
    assert cmd.brake == 1.0 and cmd.throttle == 0.0
