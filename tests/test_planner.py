"""Planner speed reachability and finish visibility, independent of controller tuning."""
from types import SimpleNamespace

import numpy as np
import pytest

from autocross.api import VehicleParams
from horizon import Corridor
from params import Params
import planner


@pytest.fixture
def straight(monkeypatch):
    vp = VehicleParams(1845.0, 3.0, 1.67, 4.79, 2.16, 1.45, 1.55, np.radians(35),
                       2.0, 0.95, 5.2, 9.0, 40.0, 0.05, 0)
    p = Params.from_vehicle(vp)
    s = np.arange(41.0)
    path = np.column_stack((s, np.zeros_like(s)))
    c = Corridor(s, path, np.tile([0.0, 1.0], (len(s), 1)), np.ones_like(s), np.ones_like(s), True)
    road = SimpleNamespace(lateral_offset=0.0, heading_error=0.0, finish_distance=np.nan)
    obs = SimpleNamespace(road=road, step=0, progress=0.0, speed=20.0)
    monkeypatch.setattr(planner, "_WARM", {})
    monkeypatch.setattr(planner, "_VCAP", {})
    monkeypatch.setattr(planner, "TUNABLES", {**planner.TUNABLES, "verify_iters": -1})
    return c, obs, vp, p


@pytest.mark.parametrize("nodes", [1, 7, 41, 150])
def test_speed_passes_match_all_pair_reachability(nodes):
    rng = np.random.default_rng(nodes)
    s = np.cumsum(rng.uniform(0.05, 2.0, nodes))
    caps = rng.uniform(0.0, 22.0, nodes)
    a_dec = 3.6
    # Independent reference: for each node, enumerate every later braking constraint.
    expected = np.array([min(np.sqrt(caps[j] ** 2 + 2 * a_dec * (s[j] - s[i]))
                             for j in range(i, nodes)) for i in range(nodes)])
    np.testing.assert_allclose(planner.backward_pass(caps, s, a_dec), expected, atol=1e-12)
    tv = np.linspace(0.0, 24.0, 100)
    td = tv ** 2 / 8.0 + 0.003 * tv ** 3  # nonlinear, monotonic power-limited acceleration curve
    dist = np.interp(caps, tv, td)
    expected = np.array([min(np.interp(dist[j] + s[i] - s[j], td, tv)
                             for j in range(i + 1)) for i in range(nodes)])
    np.testing.assert_allclose(planner.forward_pass(caps, s, (tv, td)), expected, atol=1e-12)


@pytest.mark.parametrize("finish", [np.nan, np.inf, -1.0, 41.0])
def test_unknown_or_out_of_range_finish_preserves_visibility_guard(straight, finish):
    c, obs, vp, p = straight
    obs.road.finish_distance = finish
    pl = planner.plan(c, obs, vp, p)
    assert pl.v[-1] == pytest.approx(np.sqrt(p.a_lat * planner.TUNABLES["vis_hairpin_radius"]))
    assert pl.v[0] < p.v_max


def test_visible_finish_releases_only_the_unknown_corner_guard(straight):
    c, obs, vp, p = straight
    obs.road.finish_distance = 20.0
    pl = planner.plan(c, obs, vp, p)
    assert np.isfinite(pl.v).all()
    assert pl.v.max() <= p.v_max
    assert pl.v[-1] == pytest.approx(p.v_max)
    assert pl.v[0] > 20.0


def test_curvature_on_nonuniform_circle_is_rotation_invariant():
    s = np.linspace(0.0, np.sqrt(40.0), 81) ** 2
    path = np.column_stack((25 * np.sin(s / 25), 25 * (1 - np.cos(s / 25))))
    rot = np.array([[np.cos(0.7), -np.sin(0.7)], [np.sin(0.7), np.cos(0.7)]])
    kappa = planner.curvature(path, s)
    np.testing.assert_allclose(kappa[2:-2], 1 / 25, rtol=0.001)
    np.testing.assert_allclose(planner.curvature(path @ rot.T + [50.0, -30.0], s), kappa, atol=1e-9)


def test_remembered_speed_caps_use_centerline_progress_not_racing_line_length(straight, monkeypatch):
    c, obs, vp, p = straight
    theta = c.s / 20.0
    c.center = np.column_stack((20 * np.sin(theta), 20 * (1 - np.cos(theta))))
    c.normal = np.column_stack((-np.sin(theta), np.cos(theta)))
    obs.road.lateral_offset = 0.5
    obs.progress = 100.0
    monkeypatch.setitem(planner.TUNABLES, "verify_iters", 0)
    monkeypatch.setattr(planner, "_verify", lambda pl, cap, *args: (pl, cap * 0.9))
    pl = planner.plan(c, obs, vp, p)
    assert not np.allclose(pl.s, c.s)
    np.testing.assert_allclose(planner._VCAP["s"], obs.progress + c.s)

    # Exercise next-step lookup separately: a storage-only assertion misses a query in path s.
    saved_stations = planner._VCAP["s"].copy()
    saved_caps = np.linspace(3.0, 5.0, len(c.s))
    planner._VCAP["cap"] = saved_caps
    obs.step += 1
    obs.progress += 0.5
    incoming = []

    def capture(pl, cap, *args):
        incoming.append(cap.copy())
        return pl, cap

    monkeypatch.setattr(planner, "_verify", capture)
    planner.plan(c, obs, vp, p)
    expected = np.interp(obs.progress + c.s, saved_stations,
                         saved_caps * planner.TUNABLES["verify_relax"],
                         left=planner._NO_CAP, right=planner._NO_CAP)
    # The last node is beyond the old horizon and may instead hit the visibility cap.
    np.testing.assert_allclose(incoming[0][:-1], expected[:-1])
