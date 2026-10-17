"""Unit tests for hackuab/tracker.py against synthetic plans (no simulator run needed).

From simulator/:  .venv\\Scripts\\python.exe -m pytest ..\\tests -q
"""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hackuab"))
from params import Params  # noqa: E402
from tracker import track  # noqa: E402

VEHICLE = SimpleNamespace(
    wheelbase=3.0, max_steer=math.radians(35.0), max_steer_rate=2.0, mu=0.95, max_accel=5.2,
    max_decel=9.0, max_speed=40.0, control_dt=0.05, actuation_delay_steps=0, cg_to_rear=1.55,
)


def arc_plan(kappa=0.0, v=10.0, length=40.0, offset=0.0):
    s = np.arange(0.0, length + 1e-9, 1.0)
    if abs(kappa) < 1e-9:
        path = np.column_stack([s, np.full_like(s, offset)])
    else:
        r = 1.0 / kappa
        path = np.column_stack([r * np.sin(s * kappa), offset + r * (1.0 - np.cos(s * kappa))])
    return SimpleNamespace(path=path, s=s, kappa=np.full_like(s, kappa), v=np.full_like(s, v))


def obs(speed=10.0, steer=0.0, yaw_rate=0.0, vy=0.0):
    return SimpleNamespace(speed=speed, vx=speed, vy=vy, yaw_rate=yaw_rate, steer=steer)


def test_straight_on_path_gives_zero_steer():
    steer, _, _ = track(arc_plan(), obs(), VEHICLE, None, {})
    assert abs(steer) < 1e-6


@pytest.mark.parametrize("offset,sign", [(1.0, 1), (-1.0, -1)])
def test_path_offset_steers_toward_path(offset, sign):
    state = {"steer_prev": 0.0}
    steer, _, _ = track(arc_plan(offset=offset), obs(), VEHICLE, None, state)
    assert np.sign(steer) == sign


def test_steady_arc_steer_matches_kinematic_feedforward():
    kappa, v = 1 / 30.0, 8.0
    state = {}
    for _ in range(40):  # let the slew limit settle
        steer, _, _ = track(arc_plan(kappa, v), obs(v, yaw_rate=v * kappa), VEHICLE, None, state)
    expected = math.atan(VEHICLE.wheelbase * kappa) / VEHICLE.max_steer
    assert steer == pytest.approx(expected, abs=0.02)


def test_steer_rate_never_exceeds_actuator_limit():
    state = {"steer_prev": -1.0}
    steer, _, _ = track(arc_plan(1 / 10.0, 5.0), obs(5.0), VEHICLE, None, state)
    assert steer - (-1.0) <= VEHICLE.max_steer_rate * VEHICLE.control_dt + 1e-9


def test_grip_cap_limits_kinematic_demand_at_speed():
    v = 25.0
    steer, _, _ = track(arc_plan(1 / 10.0, v), obs(v), VEHICLE, None, {"steer_prev": 0.0})
    demand = v * v * abs(math.tan(steer * VEHICLE.max_steer)) / VEHICLE.wheelbase / (VEHICLE.mu * 9.81)
    assert demand <= 1.1


@pytest.mark.parametrize("v_now,v_plan", [(5.0, 15.0), (15.0, 5.0), (10.0, 10.0), (0.0, 0.0)])
def test_pedals_mutually_exclusive_and_in_range(v_now, v_plan):
    _, thr, brk = track(arc_plan(v=v_plan), obs(v_now), VEHICLE, None, {})
    assert 0.0 <= thr <= 1.0 and 0.0 <= brk <= 1.0
    assert not (thr > 0.0 and brk > 0.0)


def test_accelerates_when_slow_and_brakes_when_fast():
    _, thr, brk = track(arc_plan(v=15.0), obs(5.0), VEHICLE, None, {})
    assert thr > 0.0 and brk == 0.0
    _, thr, brk = track(arc_plan(v=5.0), obs(15.0), VEHICLE, None, {})
    assert brk > 0.0 and thr == 0.0


def test_falling_profile_ahead_does_not_hold_back_a_slow_car():
    plan = arc_plan(v=17.0)
    plan.v = np.linspace(17.0, 6.0, len(plan.s))     # horizon-guard ramp, car far below it
    _, thr, brk = track(plan, obs(12.0), VEHICLE, None, {})
    assert brk == 0.0 and thr > 0.9


def test_nan_inputs_give_finite_outputs():
    out = track(arc_plan(), obs(speed=float("nan"), yaw_rate=float("nan"), steer=float("nan")), VEHICLE, None, {})
    assert all(math.isfinite(x) for x in out)


def test_understeer_lowers_lateral_grip_estimate():
    state = {}
    for _ in range(30):  # commanded full-ish lock, car barely rotates
        track(arc_plan(1 / 30.0, 12.0), obs(12.0, steer=0.15, yaw_rate=0.05), VEHICLE, None, state)
    assert state["a_lat_scale"] < 1.0


def test_works_with_real_params_and_sets_v_target():
    p = Params.from_vehicle(SimpleNamespace(mu=0.95, max_decel=9.0, max_accel=5.2, width=2.16))
    state = {}
    out = track(arc_plan(1 / 30.0, 8.0), obs(8.0), VEHICLE, p, state)
    assert all(math.isfinite(x) for x in out)
    assert state["v_target"] == pytest.approx(8.0)


def test_params_object_overrides_defaults():
    p = SimpleNamespace(pp_lookahead_min=9.0)
    state = {}
    track(arc_plan(), obs(0.0), VEHICLE, p, state)
    assert state["debug"]["lookahead"] == pytest.approx(9.0)
