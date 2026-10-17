"""Schema check of the committed web-app dataset in viz/data/ (written by tools/export_viz.py).

The Base44 demo app is built against these key names, so a renamed or missing key must fail here.
Fast: it only reads JSON, it runs no simulator lap.
"""
import json
import math
from pathlib import Path

import pytest

DATA = Path(__file__).resolve().parents[1] / "viz" / "data"
SERIES = ["t", "x", "y", "yaw", "speed", "target_speed", "throttle", "brake", "steer", "ax", "ay", "progress",
          "path_err", "energy_wh_cum", "coast", "fallback", "t_horizon_ms", "t_plan_ms", "t_track_ms", "t_total_ms"]
ALWAYS_NUMERIC = ["t", "x", "y", "yaw", "speed", "throttle", "brake", "steer", "ax", "ay", "progress", "energy_wh_cum"]
MODES = {"baseline", "performance", "eco"}

pytestmark = pytest.mark.skipif(not (DATA / "index.json").exists(), reason="viz/data not generated")


def finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def num_or_null(x) -> bool:
    return x is None or finite(x)


@pytest.fixture(scope="module")
def index():
    return json.loads((DATA / "index.json").read_text())


def test_index_top_level(index):
    for k in ("schema_version", "generated_at", "commit", "backend_note", "assumptions", "param_axes", "presets",
              "courses", "runs"):
        assert k in index, k
    assert index["schema_version"] == 1
    a = index["assumptions"]
    assert all(finite(a[k]) for k in ("eta_drive", "eta_regen", "p_regen_max_kw")) and isinstance(a["note"], str)
    assert set(index["param_axes"]) >= {"eco_weight", "lat_accel_frac"}
    assert index["presets"]["performance"]["eco_weight"] == 0.0 and index["presets"]["eco"]["eco_weight"] == 0.8
    assert index["runs"], "no runs exported"


def test_courses_and_tracks(index):
    for c in index["courses"]:
        for k in ("id", "name", "kind", "length_m", "reference_time_s", "closed", "laps"):
            assert k in c, (c.get("id"), k)
        assert c["kind"] in ("public", "random") and finite(c["length_m"])
        tr = json.loads((DATA / "tracks" / f"{c['id']}.json").read_text())
        for k in ("id", "centerline", "left_boundary", "right_boundary", "cones", "start_line", "finish_line",
                  "gates", "bounds"):
            assert k in tr, (c["id"], k)
        assert set(tr["cones"]) >= {"left", "right", "obstacle"}
        assert len(tr["centerline"]) > 10 and len(tr["start_line"]) == 2 and len(tr["finish_line"]) == 2
        for p in tr["centerline"] + tr["left_boundary"] + tr["right_boundary"]:
            assert len(p) == 2 and finite(p[0]) and finite(p[1])
        assert all(len(g) == 2 and len(g[0]) == 2 for g in tr["gates"])
        assert all(finite(tr["bounds"][k]) for k in ("xmin", "xmax", "ymin", "ymax"))


def test_runs_metrics(index):
    ids = {c["id"] for c in index["courses"]}
    for r in index["runs"]:
        assert r["course"] in ids and r["mode"] in MODES and r["backend"] == "bicycle"
        assert set(r["params"]) == {"eco_weight", "lat_accel_frac"}
        assert all(num_or_null(v) for v in r["params"].values())
        m = r["metrics"]
        for k in ("finished", "result", "lap_time_s", "adjusted_time_s", "technical_points", "category_scores",
                  "penalties", "p99_step_ms", "energy_wh", "wh_per_km", "peak_power_kw", "rms_jerk",
                  "energy_breakdown_wh", "energy_balance_residual_pct"):
            assert k in m, (r["id"], k)
        assert isinstance(m["finished"], bool) and isinstance(m["result"], str)
        assert set(m["category_scores"]) >= {"completion_time", "deviation", "compute", "dynamics"}
        assert set(m["penalties"]) >= {"cone_hits", "off_course", "missed_gates", "controller_errors"}
        assert set(m["energy_breakdown_wh"]) >= {"drag", "rolling", "brake_dissipated", "regen_recovered"}
        assert finite(m["energy_wh"]) and finite(m["wh_per_km"]) and finite(m["technical_points"])
        if m["finished"]:
            assert finite(m["lap_time_s"])
        assert (DATA / r["file"]).exists(), r["file"]


@pytest.mark.parametrize("which", ["performance", "baseline"])
def test_one_run_file(index, which):
    r = next(r for r in index["runs"] if r["mode"] == which)
    path = DATA / r["file"]
    assert path.stat().st_size < 300_000
    d = json.loads(path.read_text())
    n = len(d["t"])
    assert n > 10
    for k in SERIES:
        assert k in d and isinstance(d[k], list) and len(d[k]) == n, k
        assert all(num_or_null(v) for v in d[k]), k
    for k in ALWAYS_NUMERIC:
        assert all(finite(v) for v in d[k]), k
    assert all(b - a > 0 for a, b in zip(d["t"], d["t"][1:]))            # strictly increasing time
    assert abs(d["energy_wh_cum"][-1] - r["metrics"]["energy_wh"]) < 0.01  # same energy model as the metric
    for e in d["events"]:
        assert set(e) >= {"t", "kind", "x", "y"} and isinstance(e["kind"], str) and finite(e["t"])
    if which == "performance":   # ours logs its dbg_* timings on every control step (the final row has none)
        assert all(finite(v) for v in d["t_total_ms"][:-1])
