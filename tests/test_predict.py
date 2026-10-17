"""Unit tests for hackuab/predict.py (dynamic bicycle rollouts and plan verification).

Run from the repo root:  simulator\\.venv\\Scripts\\python.exe -m pytest tests -q
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hackuab"))

from autocross.api import VehicleParams  # noqa: E402
from params import G, Params  # noqa: E402
import predict  # noqa: E402

VP = VehicleParams(mass=1845.0, wheelbase=3.0, track_width=1.67, length=4.79, width=2.16, cg_to_front=1.45,
                   cg_to_rear=1.55, max_steer=math.radians(35.0), max_steer_rate=2.0, mu=0.95, max_accel=5.2,
                   max_decel=9.0, max_speed=40.0, control_dt=0.05, actuation_delay_steps=0)
P = Params.from_vehicle(VP)
C = predict.model_constants(VP, P)


def _state(vx=0.0, vy=0.0, r=0.0, delta=0.0):
    return np.array([0.0, 0.0, 0.0, vx, vy, r, delta])


def _best_ms(fn, reps=15):
    fn()  # warm-up
    best = math.inf
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        best = min(best, time.perf_counter() - t0)
    return best * 1e3


def _circle_plan(radius=25.0, v=10.0, length=40.0):
    s = np.arange(0.0, length + 1.0)
    th = s / radius
    path = np.column_stack((radius * np.sin(th), radius * (1.0 - np.cos(th))))
    return SimpleNamespace(path=path, s=s, kappa=np.full_like(s, 1.0 / radius), v=np.full_like(s, v))


def _obs(v, r=0.0, steer=0.0, ay=0.0):
    return SimpleNamespace(vx=v, vy=0.0, yaw_rate=r, steer=steer, ax=0.0, ay=ay, speed=v)


def test_shapes_and_initial_state():
    out = predict.rollout_dynamic(_state(10.0), np.zeros((4, 6)), np.zeros((4, 6)), VP, P)
    assert out.shape == (4, 7, 7)
    np.testing.assert_allclose(out[:, 0], np.broadcast_to(_state(10.0), (4, 7)))
    single = predict.rollout_dynamic(_state(10.0), np.zeros(6), np.zeros(6), VP, P)
    assert single.shape == (1, 7, 7)


def test_straight_line_acceleration_matches_force_over_mass():
    v0, t = 10.0, 10
    out = predict.rollout_dynamic(_state(v0), np.zeros(t), np.ones(t), VP, P)
    vx = out[0, :, 3]
    v_mid = 0.5 * (vx[0] + vx[-1])
    f_net = min(C.f_drive, C.power / v_mid) - C.k_drag * v_mid ** 2 - C.f_roll
    a_expected = f_net / C.m
    a_model = (vx[-1] - vx[0]) / (t * 0.05)
    assert a_model == pytest.approx(a_expected, rel=0.05)
    assert np.allclose(out[0, :, 1], 0.0) and np.allclose(out[0, :, 4:6], 0.0)  # stays straight


def test_power_limit_at_high_speed():
    v0 = 30.0  # P / v = 6.7 kN < 9.9 kN drive-force cap
    out = predict.rollout_dynamic(_state(v0), np.zeros(4), np.ones(4), VP, P)
    a_model = (out[0, -1, 3] - v0) / 0.2
    v_mid = 0.5 * (v0 + out[0, -1, 3])
    assert a_model == pytest.approx((C.power / v_mid - C.k_drag * v_mid ** 2 - C.f_roll) / C.m, rel=0.05)


def test_braking_and_pedal_inverse():
    out = predict.rollout_dynamic(_state(15.0), np.zeros(10), -np.ones(10), VP, P)
    a_brake = (out[0, -1, 3] - 15.0) / 0.5
    assert -C.lim * G * 1.05 < a_brake < -8.0          # ABS-limited, about mu*g
    for a_des in (2.0, -4.0):
        u = predict.pedal_for_accel(a_des, 12.0, VP, P)
        o = predict.rollout_dynamic(_state(12.0), np.zeros(4), np.full(4, u), VP, P)
        assert (o[0, -1, 3] - 12.0) / 0.2 == pytest.approx(a_des, rel=0.08)


@pytest.mark.parametrize("v", [2.0, 5.0])
def test_low_speed_circle_matches_kinematic_yaw_rate(v):
    delta = 0.15
    steer = delta / C.max_steer
    hold = (C.k_drag * v * v + C.f_roll) / C.f_drive   # pedal that balances the resistances
    out = predict.rollout_dynamic(_state(v, delta=delta), np.full(80, steer), np.full(80, hold), VP, P)
    vx, r = out[0, -1, 3], out[0, -1, 5]
    assert r == pytest.approx(vx * math.tan(delta) / VP.wheelbase, rel=0.05)


def test_friction_limited_cornering_stays_inside_mu_g():
    info = {}
    speeds = np.array([12.0, 18.0, 25.0])
    steer = np.array([[1.0], [0.6], [0.4]]) * np.ones((1, 40))
    pedal = np.array([[0.0], [0.3], [-0.3]]) * np.ones((1, 40))
    state = np.zeros((3, 7))
    state[:, 3] = speeds
    out = predict.rollout_dynamic(state, steer, pedal, VP, P, info=info)
    a_total = np.hypot(info["ax"], info["ay"])
    assert a_total.max() <= C.mu * G * 1.05
    assert a_total.max() > 0.8 * C.mu * G               # it actually reached the limit
    assert info["friction_use"].max() <= 1.0 + 1e-9
    assert np.isfinite(out).all()


def test_no_nans_from_standstill():
    rng = np.random.default_rng(1)
    steer, pedal = rng.uniform(-1, 1, (16, 30)), rng.uniform(-1, 1, (16, 30))
    info = {}
    out = predict.rollout_dynamic(_state(), steer, pedal, VP, P, info=info)
    assert np.isfinite(out).all() and all(np.isfinite(v).all() for v in info.values())
    assert (out[:, :, 3] >= 0.0).all()
    braked = predict.rollout_dynamic(_state(), np.zeros(20), -np.ones(20), VP, P)
    assert np.allclose(braked[0, :, :6], 0.0)           # brake at rest: does not move or roll back
    res = predict.verify_plan(_circle_plan(v=8.0), _obs(0.0), VP, P)
    assert np.isfinite(res["poses"]).all() and np.isfinite(res["v_scale"]).all()


def test_steering_slew_rate_and_lag():
    out = predict.rollout_dynamic(_state(10.0), np.ones(14), np.zeros(14), VP, P)
    delta = out[0, :, 6]
    applied_max = np.minimum(VP.max_steer_rate * 0.05 * np.arange(15), 1.0) * C.max_steer
    assert (delta <= applied_max + 1e-9).all()           # never ahead of the slew-limited command
    assert delta[10] < 0.92 * C.max_steer                 # lags it by ~ rate * tau at full slew
    assert delta[-1] > 0.95 * C.max_steer                 # and settles within ~3 tau after that


def test_substeps_converge():
    steer, pedal = np.full(30, 0.3), np.full(30, 0.2)
    coarse = predict.rollout_dynamic(_state(12.0), steer, pedal, VP, P)
    fine = predict.rollout_dynamic(_state(12.0), steer, pedal, VP, P, substeps=10)
    assert np.abs(coarse[0, :, 5] - fine[0, :, 5]).max() < 0.05
    assert np.hypot(*(coarse[0, -1, :2] - fine[0, -1, :2])) < 0.5


def test_batch_rollout_is_fast():
    rng = np.random.default_rng(0)
    steer, pedal = rng.uniform(-0.3, 0.3, (32, 30)), rng.uniform(-0.5, 0.5, (32, 30))
    ms = _best_ms(lambda: predict.rollout_dynamic(_state(12.0), steer, pedal, VP, P))
    print(f"\nrollout_dynamic 32 x 30 steps: {ms:.2f} ms")
    assert ms < 3.0


def test_verify_plan_tracks_a_gentle_circle():
    res = predict.verify_plan(_circle_plan(radius=25.0, v=8.0), _obs(8.0, r=8.0 / 25.0, steer=0.17), VP, P)
    assert res["poses"].shape == (31, 2)
    assert res["lat_dev_max"] < 0.3
    assert res["friction_use_max"] < 0.95 and res["ok"]
    np.testing.assert_allclose(res["v_scale"], 1.0)
    ms = _best_ms(lambda: predict.verify_plan(_circle_plan(), _obs(10.0), VP, P))
    print(f"\nverify_plan 1.5 s horizon: {ms:.3f} ms")
    assert ms < 2.0


def test_verify_plan_flags_and_scales_an_over_fast_corner():
    # entering a R = 20 m arc at 11 m/s with a 14 m/s plan: the car accelerates into over-demand
    plan = _circle_plan(radius=20.0, v=14.0)
    obs = _obs(11.0, r=11.0 / 20.0, steer=math.atan(3.0 / 20.0) / VP.max_steer)
    res = predict.verify_plan(plan, obs, VP, P)
    assert not res["ok"]
    assert max(res["friction_use_max"], res["grip_demand_max"]) > 0.95
    assert (res["v_scale"] <= 1.0).all() and res["v_scale"].min() < 0.95
    slower = SimpleNamespace(path=plan.path, s=plan.s, kappa=plan.kappa, v=plan.v * res["v_scale"])
    res2 = predict.verify_plan(slower, obs, VP, P)
    assert res2["ok"]                                    # one scale pass fixes it
    assert res2["lat_dev_max"] < res["lat_dev_max"]


def test_verify_plan_corridor_and_custom_tracker():
    plan = _circle_plan(radius=1e6, v=10.0)              # straight line along x
    s = plan.s
    corridor = SimpleNamespace(s=s, center=plan.path, normal=np.tile([0.0, 1.0], (len(s), 1)),
                               left=np.full(len(s), 1.0), right=np.full(len(s), 1.0), valid=True)
    res = predict.verify_plan(plan, _obs(10.0), VP, P, corridor=corridor)
    assert res["corridor_violation_max"] < 0.0 and res["ok"]
    # a "tracker" that steers hard left leaves the corridor and is caught
    res = predict.verify_plan(plan, _obs(10.0), VP, P, corridor=corridor, track_fn=lambda st, pl, k: (0.5, 0.0))
    assert res["corridor_violation_max"] > 0.0 and not res["ok"]
    assert res["lat_dev"][-1] > 1.0


def test_kinematic_helpers_still_work():
    out = predict.rollout_kinematic(np.array([0.0, 0.0, 0.0, 10.0]), np.zeros((2, 5)), np.zeros((2, 5)), 3.0, 0.05)
    assert out.shape == (2, 6, 4) and out[0, -1, 0] == pytest.approx(2.5)
    pose = predict.predict_pose(10.0, 0.1, 3.0, 0.5)
    assert pose[0] > 0 and pose[1] > 0
