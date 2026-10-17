"""Buffered corridor projection and its production speed-selection response."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from horizon import Corridor
from params import Params
import planner
import predict
from test_predict import VP, _circle_plan, _obs


def band(path, stations, normal, left, right):
    n = len(stations)
    return Corridor(np.asarray(stations), np.asarray(path), np.asarray(normal),
                    np.broadcast_to(left, (n,)).copy(), np.broadcast_to(right, (n,)).copy(), True)


@pytest.mark.parametrize("turn", [-1, 1])
def test_curved_projection_recovers_interpolated_cross_sections(turn):
    # Sparse, nonuniform, tight curve; offset band entirely LEFT of centerline. Nearest-node
    # lookup would apply the wrong variable width and wrong station between samples.
    s = np.array([0., 1., 3., 5., 8., 11., 14.])
    theta = s / 8
    center = np.column_stack((8 * np.sin(theta), turn * 8 * (1 - np.cos(theta))))
    normal = np.column_stack((-turn * np.sin(theta), np.cos(theta)))
    c = band(center, s, normal, 1.8 + 0.03 * s, -0.4 - 0.01 * s)
    j = np.arange(len(s) - 1)
    f = np.linspace(0.13, 0.87, len(j))
    ref = center[j] + f[:, None] * np.diff(center, axis=0)
    normals = normal[j] + f[:, None] * np.diff(normal, axis=0)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    offset = np.linspace(0.7, 1.5, len(j))
    points = ref + offset[:, None] * normals
    r = predict.corridor_clearance(points, c)
    np.testing.assert_allclose(r['corridor_station'], s[j] + f * np.diff(s), atol=1e-5)
    np.testing.assert_allclose(r['corridor_offset'], offset, atol=1e-6)
    assert r['corridor_covered'].all()
    assert not r['corridor_violation'].any()


@pytest.mark.parametrize("radius", [8., -8., 25.])
def test_projection_matches_analytic_circle_within_polyline_error(radius):
    s = np.arange(0., 21.)
    angle = s / radius
    center = np.column_stack((radius * np.sin(angle), radius * (1 - np.cos(angle))))
    normal = np.column_stack((-np.sin(angle), np.cos(angle)))
    c = band(center, s, normal, 2., 2.)
    q = np.arange(0.25, 19.75, 0.5)
    angle = q / radius
    points = np.column_stack((radius * np.sin(angle), radius * (1 - np.cos(angle))))
    points += 0.7 * np.column_stack((-np.sin(angle), np.cos(angle)))
    result = predict.corridor_clearance(points, c)
    np.testing.assert_allclose(result['corridor_station'], q, atol=0.005)
    np.testing.assert_allclose(result['corridor_offset'], 0.7, atol=0.017)
    assert result['corridor_covered'].all()


def test_projection_signed_bounds_buffers_and_rotation():
    s = np.array([0., 5., 10.])
    center = np.column_stack((s, s * 0))
    c = band(center, s, np.tile([0., 1.], (3, 1)), [1., 2., 3.], [2., 1., 0.])
    points = np.array([[2.5, 1.6], [7.5, -.7], [7.5, 2.4]])
    result = predict.corridor_clearance(points, c)
    np.testing.assert_allclose(result['corridor_violation'], [.1, .2, 0.], atol=1e-12)
    rot = np.array([[np.cos(.8), -np.sin(.8)], [np.sin(.8), np.cos(.8)]])
    rotated = replace(c, center=c.center @ rot.T + [10., -4.], normal=c.normal @ rot.T)
    other = predict.corridor_clearance(points @ rot.T + [10., -4.], rotated)
    np.testing.assert_allclose(other['corridor_violation'], result['corridor_violation'], atol=1e-12)
    # No vehicle half-width is subtracted a second time: the last point is already inside.
    assert result['corridor_violation'][-1] == 0


def test_initial_half_cell_checks_lateral_bounds_without_a_false_projection_breach():
    s = np.arange(4.)
    c = band(np.column_stack((s, s * 0)), s, np.tile([0., 1.], (4, 1)), 1., 1.)
    result = predict.corridor_clearance(np.array([[-.02, .5], [-.02, 1.1], [-.6, 0.]]), c)
    np.testing.assert_allclose(result['corridor_violation'][:2], [0., .1])
    assert result['corridor_covered'][:2].all()
    assert not result['corridor_covered'][2]


@pytest.mark.parametrize('offset', [-1., 1.])
def test_curved_vertex_does_not_create_an_unobserved_wedge(offset):
    s = np.arange(15.)
    angle = s / 8.
    center = np.column_stack((8 * np.sin(angle), 8 * (1 - np.cos(angle))))
    normal = np.column_stack((-np.sin(angle), np.cos(angle)))
    c = band(center, s, normal, 2., 2.)
    # The nearest centerline vertex is not the inverse normal coordinate near a joint.
    j = np.repeat(np.arange(len(s) - 1), 2)
    f = np.tile([.001, .999], len(s) - 1)
    norms = normal[j] + f[:, None] * np.diff(normal, axis=0)[j]
    norms /= np.linalg.norm(norms, axis=1, keepdims=True)
    points = center[j] + f[:, None] * np.diff(center, axis=0)[j] + offset * norms
    result = predict.corridor_clearance(points, c)
    assert result['corridor_covered'].all()
    np.testing.assert_allclose(result['corridor_station'], s[j] + f, atol=1e-6)


def test_local_start_projection_wins_over_a_remote_hairpin_normal():
    from horizon import left_normals
    s = np.arange(41.)
    theta = s / 8.
    center = np.column_stack((8 * np.sin(theta), 8 * (1 - np.cos(theta))))
    c = band(center, s, left_normals(center), 1., 1.)
    result = predict.corridor_clearance(np.array([[0., -.65]]), c)
    assert result['corridor_covered'][0]
    assert result['corridor_station'][0] < .1
    assert result['corridor_violation'][0] == 0


def test_folded_normals_fail_closed_without_nan_speeds():
    pl, c, obs = circle_case(3.)
    c.normal[1] *= -1
    result = predict.verify_plan(pl, obs, VP, Params.from_vehicle(VP), corridor=c)
    assert not result['ok']
    assert np.isfinite(result['v_scale']).all()


def test_single_node_plan_reports_failure_with_complete_breach_metadata():
    pl = planner.Plan(np.zeros((1, 2)), np.zeros(1), np.zeros(1), np.ones(1))
    result = predict.verify_plan(pl, _obs(1.), VP, Params.from_vehicle(VP))
    assert not result['ok'] and result['demand_max'] > 1
    assert result['corridor_first_step'] == 0


@pytest.mark.parametrize("kind", ['invalid', 'nan', 'closed', 'duplicate', 'unobserved'])
def test_bad_or_unobserved_corridor_does_not_pass(kind):
    s = np.arange(4.)
    c = band(np.column_stack((s, s * 0)), s, np.tile([0., 1.], (4, 1)), 1., 1.)
    point = np.array([[1.5, 0.]])
    if kind == 'invalid':
        c.valid = False
    elif kind == 'nan':
        c.left[1] = np.nan
    elif kind == 'closed':
        c.right[:] = -2.
    elif kind == 'duplicate':
        c.center[1] = c.center[0]
    else:
        point[0, 0] = 5.
    r = predict.corridor_clearance(point, c)
    assert r['corridor_violation'].max() > 0
    assert not r['corridor_covered'].all()


def circle_case(width):
    pl = _circle_plan(radius=25, v=8.)
    theta = pl.s / 25
    c = band(pl.path, pl.s, np.column_stack((-np.sin(theta), np.cos(theta))), width, width)
    return planner.Plan(pl.path, pl.s, pl.kappa, pl.v), c, _obs(8., r=8./25, steer=.17)


def test_breach_reports_per_step_first_location_and_local_scale():
    pl, c, obs = circle_case(.005)
    res = predict.verify_plan(pl, obs, VP, Params.from_vehicle(VP), corridor=c)
    assert res['friction_use_max'] < .95  # clearance alone must trigger correction
    assert not res['ok'] and not res['corridor_ok']
    assert res['corridor_violation'].shape == (31,)
    first = np.flatnonzero(res['corridor_violation'] > 1e-6)[0]
    assert res['corridor_first_step'] == first
    assert res['corridor_first_time'] == pytest.approx(first * .05)
    assert res['corridor_first_s'] >= 0
    assert res['demand_max'] > 1
    assert .65 <= res['v_scale'].min() <= .9
    assert (res['v_scale'][pl.s > res['corridor_first_path_s'] + 5] == 1).all()
    assert np.any(res['v_scale'][pl.s <= res['corridor_first_path_s']] < 1)


def run_verify(width, monkeypatch):
    pl, c, obs = circle_case(width)
    p = Params.from_vehicle(VP)
    monkeypatch.setitem(planner.TUNABLES, 'verify_budget_ms', 1000.)
    table = planner.accel_table(VP, p, 31.)
    calls = []

    def profile(cap):
        calls.append(cap.copy())
        return planner.forward_pass(planner.backward_pass(cap, pl.s, p.a_dec), pl.s, table)

    result, cap = planner._verify(pl, pl.v.copy(), profile, obs, VP, p, c)
    return pl, result, cap, calls


def test_narrow_corridor_changes_speed_plan_and_runs_reachability(monkeypatch):
    original, result, cap, calls = run_verify(.005, monkeypatch)
    assert calls  # not just a new diagnostic that cannot affect driving
    assert (result.v <= original.v).all()
    assert result.v.min() < original.v.min()
    assert cap.min() < original.v.min()
    assert result.debug['pred_corridor_max'] > 0 or result.debug['pred_corridor_ok']
    assert result.debug['pred_ok'] or result.debug['pred_unresolved']


def test_clearance_fallback_can_be_reverified_safe_in_the_dynamic_model(monkeypatch):
    original, result, _, calls = run_verify(.2, monkeypatch)
    assert calls and np.any(result.v < original.v)
    assert result.debug['pred_corridor_initial_max'] > 0
    assert result.debug['pred_verified'] == result.debug['pred_ok'] == 1
    assert result.debug['pred_corridor_ok'] == 1
    assert not result.debug['pred_unresolved']


def test_wide_corridor_preserves_speed_plan_bit_for_bit(monkeypatch):
    original, result, cap, calls = run_verify(3., monkeypatch)
    assert not calls
    np.testing.assert_array_equal(result.v, original.v)
    assert result.debug['pred_ok'] == 1 and result.debug['pred_corridor_ok'] == 1
    assert not result.debug['pred_fallback']


def test_legacy_grip_only_verification_has_finite_debug_without_certifying_clearance():
    pl, _, obs = circle_case(3.)
    result, _ = planner._verify(pl, pl.v.copy(), lambda cap: cap, obs, VP, Params.from_vehicle(VP))
    assert all(np.isfinite(value) for value in result.debug.values())
    assert result.debug['pred_corridor_checked'] == 0


def test_budget_exhaustion_reports_unverified_braking_fallback(monkeypatch):
    pl, c, obs = circle_case(.005)
    p = Params.from_vehicle(VP)
    monkeypatch.setitem(planner.TUNABLES, 'verify_budget_ms', 0.)
    result, _ = planner._verify(pl, pl.v.copy(), lambda cap: planner.backward_pass(cap, pl.s, p.a_dec),
                                obs, VP, p, c)
    assert result.debug['pred_budget_exhausted'] == 1
    assert result.debug['pred_fallback'] == result.debug['pred_unresolved'] == 1
    assert result.debug['pred_verified'] == result.debug['pred_ok'] == 0
    assert np.all(result.v <= obs.speed)


def test_grip_only_fallback_does_not_blanket_derate_a_wide_corridor(monkeypatch):
    pl, c, obs = circle_case(3.)
    p = Params.from_vehicle(VP)
    real = predict.verify_plan(pl, obs, VP, p, corridor=c)
    scale = np.ones_like(pl.v)
    scale[20] = .9
    failed = {**real, 'ok': False, 'demand_max': 1.1, 'v_scale': scale}
    monkeypatch.setattr(planner, 'verify_plan', lambda *a, **kw: failed)
    monkeypatch.setitem(planner.TUNABLES, 'verify_iters', 0)
    monkeypatch.setitem(planner.TUNABLES, 'verify_budget_ms', 0.)
    result, _ = planner._verify(pl, pl.v.copy(), lambda cap: cap, obs, VP, p, c)
    np.testing.assert_array_equal(result.v[:20], pl.v[:20])
    assert result.v[20] == pytest.approx(.9 * pl.v[20])
    assert result.debug['pred_fallback'] and not result.debug['pred_clearance_fallback']


@pytest.mark.parametrize('clearance', [False, True])
def test_only_clearance_stop_caps_are_excluded_from_next_step_memory(monkeypatch, clearance):
    _, c, obs = circle_case(3.)
    obs.road = SimpleNamespace(lateral_offset=0., heading_error=0., finish_distance=np.nan)
    obs.step, obs.progress = 0, 0.
    monkeypatch.setattr(planner, '_WARM', {})
    monkeypatch.setattr(planner, '_VCAP', {})

    def fallback(pl, cap, *args):
        pl.debug = {'pred_fallback': 1., 'pred_clearance_fallback': float(clearance)}
        return pl, cap * .9

    monkeypatch.setattr(planner, '_verify', fallback)
    planner.plan(c, obs, VP, Params.from_vehicle(VP))
    assert bool(planner._VCAP) != clearance


def test_production_plan_passes_original_corridor(monkeypatch):
    pl, c, obs = circle_case(3.)
    obs.road = SimpleNamespace(lateral_offset=0., heading_error=0., finish_distance=np.nan)
    obs.step, obs.progress = 0, 0.
    monkeypatch.setattr(planner, '_WARM', {})
    monkeypatch.setattr(planner, '_VCAP', {})
    seen = []
    real = planner.verify_plan

    def capture(*args, **kwargs):
        seen.append(kwargs['corridor'])
        return real(*args, **kwargs)

    monkeypatch.setattr(planner, 'verify_plan', capture)
    planner.plan(c, obs, VP, Params.from_vehicle(VP))
    assert seen and all(value is c for value in seen)


@pytest.mark.parametrize('width,changes', [(.005, True), (3., False)])
def test_production_speed_response_is_clearance_dependent(monkeypatch, width, changes):
    _, c, obs = circle_case(width)
    obs.road = SimpleNamespace(lateral_offset=0., heading_error=0., finish_distance=np.nan)
    obs.step, obs.progress = 0, 0.
    p = Params.from_vehicle(VP, {'v_max': 8.})
    monkeypatch.setattr(planner, '_WARM', {})
    monkeypatch.setattr(planner, '_VCAP', {})
    monkeypatch.setitem(planner.TUNABLES, 'verify_budget_ms', 1000.)
    monkeypatch.setitem(planner.TUNABLES, 'verify_iters', -1)
    reference = planner.plan(c, obs, VP, p)
    planner._WARM.clear()
    planner._VCAP.clear()
    monkeypatch.setitem(planner.TUNABLES, 'verify_iters', 1)
    result = planner.plan(c, obs, VP, p)
    np.testing.assert_array_equal(result.path, reference.path)
    if changes:
        assert np.any(result.v < reference.v - 1e-6)
    else:
        np.testing.assert_array_equal(result.v, reference.v)
        assert result.debug['pred_ok']
