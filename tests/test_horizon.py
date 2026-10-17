"""horizon.build_corridor against synthetic road models (no simulator run needed)."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

from autocross.api import ObstacleInfo, RoadModel, VehicleParams
from horizon import build_corridor

VEHICLE = VehicleParams(mass=1845.0, wheelbase=3.0, track_width=1.67, length=4.79, width=2.16,
                        cg_to_front=1.45, cg_to_rear=1.55, max_steer=np.radians(35), max_steer_rate=2.0,
                        mu=0.95, max_accel=5.2, max_decel=9.0, max_speed=40.0, control_dt=0.05,
                        actuation_delay_steps=0)
P = SimpleNamespace()                       # all defaults
BUF = VEHICLE.width / 2 + 0.3


def obstacle(d, lat_min, lat_max):
    return ObstacleInfo(d, lat_min, lat_max, np.array([d, 0.5 * (lat_min + lat_max)]), np.zeros((2, 2)))


def road(half_width=2.75, kappa=0.0, obstacles=(), lateral_offset=0.0, rng=40.0):
    s = np.arange(0.0, rng + 1e-9, 1.0)
    if kappa:
        th = kappa * s
        cl = np.column_stack((np.sin(th) / kappa, (1 - np.cos(th)) / kappa))
    else:
        cl = np.column_stack((s, np.zeros_like(s)))
    w = np.full_like(s, half_width)
    return RoadModel(True, rng, lateral_offset, 0.0, kappa, half_width, half_width, half_width, half_width,
                     s, np.full_like(s, kappa), cl, w, w.copy(), cl, cl, [], list(obstacles), float("nan"))


def at(c, d):
    i = int(np.argmin(np.abs(c.s - d)))
    return c.left[i], c.right[i]


def test_straight_road_shrinks_both_sides_by_buffer():
    c = build_corridor(road(), VEHICLE, P)
    assert c.valid
    np.testing.assert_allclose(c.left, 2.75 - BUF)
    np.testing.assert_allclose(c.right, 2.75 - BUF)
    np.testing.assert_allclose(c.normal, np.tile([0.0, 1.0], (len(c.s), 1)), atol=1e-12)


def test_normals_are_unit_and_point_left_on_a_curve():
    c = build_corridor(road(kappa=1 / 15), VEHICLE, P)
    np.testing.assert_allclose(np.hypot(*c.normal.T), 1.0)
    # left turn: the left normal points toward the turn centre at (0, 15)
    to_centre = np.array([0.0, 15.0]) - c.center
    assert np.all(np.sum(c.normal * to_centre, axis=1) > 0)


def test_obstacle_on_right_moves_band_left():
    # practice_technical s=65: cones at -1.7..-1.0 (the -2.4 cone was dropped next to the boundary)
    c = build_corridor(road(obstacles=[obstacle(20.0, -1.7, -1.0)]), VEHICLE, P)
    left, right = at(c, 20.0)
    assert c.valid
    assert -right == pytest.approx(-1.0 + BUF)          # lower bound pushed past the obstacle
    assert left == pytest.approx(2.75 - BUF)
    assert at(c, 0.0) == pytest.approx((2.75 - BUF, 2.75 - BUF))   # untouched far before it


def test_obstacle_on_left_moves_band_right():
    c = build_corridor(road(obstacles=[obstacle(20.0, 1.1, 2.4)]), VEHICLE, P)
    left, right = at(c, 20.0)
    assert left == pytest.approx(1.1 - BUF)
    assert right == pytest.approx(2.75 - BUF)


def test_cut_holds_over_car_length_and_tapers_monotonically():
    c = build_corridor(road(obstacles=[obstacle(20.0, -1.7, -1.0)]), VEHICLE, P)
    lo = -c.right
    full = -1.0 + BUF
    hold = VEHICLE.length / 2 + 0.5
    np.testing.assert_allclose(lo[np.abs(c.s - 20.0) <= hold], full)
    assert np.all(np.diff(lo[c.s <= 20.0]) >= -1e-12)    # ramps up into the obstacle
    assert np.all(np.diff(lo[c.s >= 20.0]) <= 1e-12)     # and back down after it


def test_obstacle_just_behind_car_still_cuts_at_s0():
    c = build_corridor(road(obstacles=[obstacle(-1.0, -1.7, -1.0)]), VEHICLE, P)
    assert -c.right[0] == pytest.approx(-1.0 + BUF)


def test_near_tie_follows_car_side():
    ob = obstacle(20.0, -0.3, 0.3)          # dead centre: both gaps equal
    c_left = build_corridor(road(half_width=3.5, obstacles=[ob], lateral_offset=0.5), VEHICLE, P)
    c_right = build_corridor(road(half_width=3.5, obstacles=[ob], lateral_offset=-0.5), VEHICLE, P)
    assert -at(c_left, 20.0)[1] == pytest.approx(0.3 + BUF)
    assert at(c_right, 20.0)[0] == pytest.approx(-0.3 - BUF)


def test_tight_gap_collapses_to_midline_but_stays_valid():
    # gap between obstacle and wall is 2.4 m: car (2.16) fits, but not with 0.3 m each side
    c = build_corridor(road(obstacles=[obstacle(20.0, -2.75, 0.35)]), VEHICLE, P)
    left, right = at(c, 20.0)
    assert c.valid
    assert left == pytest.approx(-right)                 # zero-width band
    assert -right == pytest.approx(0.5 * (0.35 + 2.75))  # middle of the gap


def test_blocked_road_is_invalid():
    c = build_corridor(road(obstacles=[obstacle(20.0, -2.0, 2.0)]), VEHICLE, P)
    assert not c.valid


@pytest.mark.parametrize("r", [None, RoadModel(False, 40.0, 0, 0, 0, 0, 0, 0, 0, np.zeros(0), np.zeros(0),
                                               np.zeros((0, 2)), np.zeros(0), np.zeros(0), np.zeros((0, 2)),
                                               np.zeros((0, 2)), [], [], float("nan"))])
def test_missing_road_is_invalid_and_empty(r):
    c = build_corridor(r, VEHICLE, P)
    assert not c.valid and len(c.s) == 0


def test_nan_inputs_give_finite_output():
    r = road(obstacles=[obstacle(float("nan"), -1.0, 1.0)])
    r.preview_width_left[5] = np.nan
    c = build_corridor(r, VEHICLE, P)
    assert np.all(np.isfinite(c.left)) and np.all(np.isfinite(c.right)) and np.all(np.isfinite(c.normal))


def test_params_override_margin():
    c = build_corridor(road(), VEHICLE, SimpleNamespace(cone_margin=0.5))
    np.testing.assert_allclose(c.left, 2.75 - VEHICLE.width / 2 - 0.5)


def test_team_params_object_works():
    from params import Params
    p = Params.from_vehicle(VEHICLE)
    c = build_corridor(road(obstacles=[obstacle(20.0, -1.7, -1.0)]), VEHICLE, p)
    assert c.valid and -at(c, 20.0)[1] == pytest.approx(-1.0 + VEHICLE.width / 2 + p.cone_margin)


def test_very_narrow_road_keeps_min_half_width():
    c = build_corridor(road(half_width=1.4), VEHICLE, P)
    np.testing.assert_allclose(c.left, 0.2)
    np.testing.assert_allclose(c.right, 0.2)


def test_fast_enough():
    r = road(kappa=1 / 20, obstacles=[obstacle(10.0, -1.7, -1.0), obstacle(30.0, 1.1, 2.4)])
    build_corridor(r, VEHICLE, P)
    t0 = time.perf_counter()
    for _ in range(200):
        build_corridor(r, VEHICLE, P)
    assert (time.perf_counter() - t0) / 200 < 1e-3
