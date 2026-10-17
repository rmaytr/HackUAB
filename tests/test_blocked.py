"""Blocked corridor: an obstacle that leaves no gap the car fits through.

Before the fix the planner turned an invalid corridor into a normal, accelerating centerline plan:
full throttle, target 18 m/s, fallback 0, straight at the obstacle. These tests check the final
Command (not just corridor.valid) in both driving modes.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hackuab"))
import controller as ctl  # noqa: E402
import horizon  # noqa: E402
import planner  # noqa: E402
from autocross.api import ObstacleInfo, Observation, RoadModel, TrackInfo, VehicleParams  # noqa: E402

VEHICLE = VehicleParams(mass=1845.0, wheelbase=3.0, track_width=1.67, length=4.79, width=2.16,
                        cg_to_front=1.45, cg_to_rear=1.55, max_steer=math.radians(35), max_steer_rate=2.0,
                        mu=0.95, max_accel=5.2, max_decel=9.0, max_speed=40.0, control_dt=0.05,
                        actuation_delay_steps=0)
TRACK = TrackInfo("course", False, 1, np.zeros((0, 2)), np.zeros(0), np.zeros(0), np.zeros((0, 2)),
                  np.zeros(0, int), np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((0, 2, 2)), False)
HALF_WIDTH = 2.75
MODES = pytest.mark.parametrize("config", [{}, {"mode": "eco"}], ids=["performance", "eco"])


def straight_road(obstacle_at: float | None, lat_min: float = -2.0, lat_max: float = 2.0) -> RoadModel:
    """Straight 5.5 m wide road; optionally an obstacle at `obstacle_at` m spanning [lat_min, lat_max]."""
    s = np.arange(0.0, 41.0)
    w = np.full_like(s, HALF_WIDTH)
    cl = np.column_stack((s, 0.0 * s))
    obs = []
    if obstacle_at is not None and 0.0 <= obstacle_at <= 40.0:
        mid = 0.5 * (lat_min + lat_max)
        obs = [ObstacleInfo(float(obstacle_at), lat_min, lat_max, np.array([obstacle_at, mid]),
                            np.array([[obstacle_at, lat_min], [obstacle_at, lat_max]]))]
    return RoadModel(True, 40.0, 0.0, 0.0, 0.0, HALF_WIDTH, HALF_WIDTH, HALF_WIDTH, HALF_WIDTH, s, 0.0 * s, cl,
                     w, w.copy(), np.column_stack((s, w)), np.column_stack((s, -w)), [], obs, float("nan"))


def observe(road: RoadModel, v: float, prog: float = 0.0, step: int = 0, thr=0.0, brk=0.0) -> Observation:
    return Observation(t=step * 0.05, step=step, x=prog, y=0.0, yaw=0.0, vx=v, vy=0.0, yaw_rate=0.0, ax=0.0,
                       ay=0.0, speed=v, steer=0.0, throttle=thr, brake=brk, progress=prog, lap=0,
                       timing_started=True, cones=None, road=road)


def make(config):
    planner._WARM.clear()
    planner._VCAP.clear()
    c = ctl.HackUABController(dict(config, strict=True))
    c.on_start(TRACK, VEHICLE)
    return c


@MODES
def test_blocked_corridor_brakes_instead_of_accelerating(config):
    road = straight_road(12.0)
    c = make(config)
    assert not horizon.build_corridor(road, VEHICLE, c.p).valid       # the precondition of the bug
    cmd = c.step(observe(road, 8.0))
    assert cmd.throttle == 0.0 and cmd.brake > 0.0
    assert cmd.target_speed is not None and cmd.target_speed <= 8.0 + 1e-9
    assert cmd.debug["blocked"] == 1.0 and cmd.debug["blocked_s"] <= 12.0
    assert cmd.debug["fallback"] == 0.0
    assert np.all(np.isfinite(cmd.path))


@MODES
def test_blocked_corridor_closed_loop_slows_before_the_obstacle(config):
    """Point-mass closed loop toward a fixed blockage 25 m ahead: the car never speeds up while blocked,
    and it is down to the creep speed before its nose reaches the obstacle."""
    c = make(config)
    creep = planner.TUNABLES["blocked_creep_speed"]
    v, prog, thr, brk = 10.0, 0.0, 0.0, 0.0
    obstacle = 25.0
    speeds = []
    for k in range(200):
        cmd = c.step(observe(straight_road(obstacle - prog), v, prog, k, thr, brk))
        thr, brk = cmd.throttle, cmd.brake
        assert not (thr > 0.0 and brk > 0.0)
        v = max(v + 0.05 * (thr * 5.2 - brk * 9.0), 0.0)
        prog += v * 0.05
        speeds.append(v)
        if prog + 0.5 * VEHICLE.length >= obstacle:
            break
    assert max(speeds) <= 10.0 + 1e-6                                   # never accelerated
    assert v <= creep + 0.5                                             # slowed down before reaching it


@MODES
def test_obstacle_with_a_gap_is_not_blocked(config):
    """Control case: a small obstacle on the left leaves room on the right, so nothing changes."""
    road = straight_road(15.0, lat_min=1.2, lat_max=2.5)
    c = make(config)
    assert horizon.build_corridor(road, VEHICLE, c.p).valid
    cmd = c.step(observe(road, 8.0))
    assert cmd.debug["blocked"] == 0.0
