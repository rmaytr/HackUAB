"""Eco mode (planner eco_weight) and its guarantee that eco_weight = 0 is today's controller, bit for bit.

The golden file tests/data/eco0_golden.npz was recorded from main at 8acab13 (before Eco mode existed)
by running this file as a script:  .venv\\Scripts\\python.exe ..\\tests\\test_eco.py --record
The sequence is closed loop on a synthetic course (straight, then a tight corner, then a straight),
so it covers acceleration, the braking zone, cornering and the warm-start / verifier-cap memories.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hackuab"))
import controller as ctl  # noqa: E402
import planner  # noqa: E402
from autocross.api import Observation, RoadModel, TrackInfo, VehicleParams  # noqa: E402

GOLDEN = os.path.join(os.path.dirname(__file__), "data", "eco0_golden.npz")
VEHICLE = VehicleParams(mass=1845.0, wheelbase=3.0, track_width=1.67, length=4.79, width=2.16,
                        cg_to_front=1.45, cg_to_rear=1.55, max_steer=math.radians(35), max_steer_rate=2.0,
                        mu=0.95, max_accel=5.2, max_decel=9.0, max_speed=40.0, control_dt=0.05,
                        actuation_delay_steps=0)
TRACK = TrackInfo("course", False, 1, np.zeros((0, 2)), np.zeros(0), np.zeros(0), np.zeros((0, 2)),
                  np.zeros(0, int), np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((0, 2, 2)), False)
CORNER = (70.0, 95.0, 1.0 / 14.0)          # course stations of a left corner and its curvature
N_STEPS = 140


def course_kappa(s):
    a, b, k = CORNER
    return np.where((s >= a) & (s <= b), k, 0.0)


def road_at(progress: float) -> RoadModel:
    s = np.arange(0.0, 40.0 + 1e-9, 1.0)
    kap = course_kappa(progress + s)
    th = np.concatenate(([0.0], np.cumsum(0.5 * (kap[1:] + kap[:-1]))))
    cl = np.column_stack((np.concatenate(([0.0], np.cumsum(np.cos(th[:-1])))),
                          np.concatenate(([0.0], np.cumsum(np.sin(th[:-1]))))))
    w = np.full_like(s, 3.0)
    return RoadModel(True, 40.0, 0.0, 0.0, float(kap[0]), 3.0, 3.0, 3.0, 3.0,
                     s, kap, cl, w, w.copy(), cl, cl, [], [], float("nan"))


def run_sequence(config=None) -> dict[str, np.ndarray]:
    """Closed loop: the car stays on the centerline and its speed follows the pedals (point mass)."""
    planner._WARM.clear()
    planner._VCAP.clear()
    c = ctl.HackUABController(dict(config or {}))
    c.on_start(TRACK, VEHICLE)
    v, prog, thr, brk, steer = 0.5, 0.0, 0.0, 0.0, 0.0
    out = {k: [] for k in ("steer", "throttle", "brake", "target_speed", "path", "speed")}
    for k in range(N_STEPS):
        ob = Observation(t=k * 0.05, step=k, x=prog, y=0.0, yaw=0.0, vx=v, vy=0.0, yaw_rate=0.0, ax=0.0, ay=0.0,
                         speed=v, steer=steer, throttle=thr, brake=brk, progress=prog, lap=0, timing_started=True,
                         cones=None, road=road_at(prog))
        cmd = c.step(ob)
        steer, thr, brk = cmd.steer, cmd.throttle, cmd.brake
        for key in ("steer", "throttle", "brake"):
            out[key].append(getattr(cmd, key))
        out["target_speed"].append(cmd.target_speed if cmd.target_speed is not None else np.nan)
        out["path"].append(np.asarray(cmd.path, float)[:40])
        out["speed"].append(v)
        drag = (0.5 * 1.2 * 0.55 * v * v + 0.012 * 1845 * 9.81) / 1845
        v = max(v + 0.05 * (thr * 5.2 - brk * 9.0 - drag), 0.0)
        prog += v * 0.05
    return {k: np.asarray(x, float) for k, x in out.items()}


def test_eco_weight_zero_reproduces_golden_commands():
    if not os.path.exists(GOLDEN):
        pytest.skip("golden file missing; record it with --record")
    gold = np.load(GOLDEN)
    for cfg in ({}, {"eco_weight": 0.0}):
        got = run_sequence(cfg)
        for key in gold.files:
            np.testing.assert_array_equal(got[key], gold[key], err_msg=f"{key} differs with config {cfg}")


def test_eco_mode_coasts_before_braking_and_uses_less_energy():
    perf, eco = run_sequence({}), run_sequence({"eco_weight": 1.0})
    for r in (perf, eco):
        assert not np.any((r["throttle"] > 0) & (r["brake"] > 0))
        assert np.all(np.isfinite(r["steer"]))
    # Eco drives slower on the straight and reaches the corner with less kinetic energy to brake away
    assert eco["speed"].max() < perf["speed"].max()
    assert np.sum(eco["brake"]) < np.sum(perf["brake"])
    assert np.sum(eco["throttle"] * eco["speed"]) < np.sum(perf["throttle"] * perf["speed"])
    # and it spends more time with neither pedal while moving (lift-and-coast)
    coast = lambda r: int(np.sum((r["throttle"] == 0) & (r["brake"] == 0) & (r["speed"] > 5)))
    assert coast(eco) > coast(perf)


def test_eco_debug_keys_and_plan_verified():
    planner._WARM.clear()
    planner._VCAP.clear()
    c = ctl.HackUABController({"eco_weight": 0.6})
    c.on_start(TRACK, VEHICLE)
    ob = Observation(t=0.0, step=0, x=0.0, y=0.0, yaw=0.0, vx=14.0, vy=0.0, yaw_rate=0.0, ax=0.0, ay=0.0,
                     speed=14.0, steer=0.0, throttle=0.0, brake=0.0, progress=40.0, lap=0, timing_started=True,
                     cones=None, road=road_at(40.0))
    cmd = c.step(ob)
    assert cmd.debug["eco_weight"] == pytest.approx(0.6)
    assert cmd.debug["coast"] in (0.0, 1.0)
    assert cmd.debug["regen_limited"] == 0.0
    assert cmd.debug["fallback"] == 0.0
    assert cmd.debug["pred_ok"] == 1.0


if __name__ == "__main__" and "--record" in sys.argv:
    os.makedirs(os.path.dirname(GOLDEN), exist_ok=True)
    np.savez(GOLDEN, **run_sequence({}))
    print("wrote", GOLDEN)
