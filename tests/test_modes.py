"""Driving-mode selection: PERFORMANCE (fastest lap, the scored default) vs ECO (energy-optimal lap).

Each mode is a YAML in hackuab/ whose `mode:` key picks a profile from params.MODES. The two modes
are separate test groups:
    pytest -m performance        # PERFORMANCE mode only
    pytest -m eco                # ECO mode only
    pytest -m "eco and not sim"  # ECO mode, fast unit tests only (no full simulator laps)
"""
import json
import os
import re
import subprocess
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "hackuab"))
import params  # noqa: E402
from test_eco import (GOLDEN, MAIN_TECHNICAL_TIME_S, VEHICLE, assert_reference_lap,  # noqa: E402
                      budget_guard_trips, run_sequence)

HACKUAB = Path(__file__).resolve().parents[1] / "hackuab"
ECO_MAX_LAP_RATIO = 1.12          # one course; the all-course Eco budget is 10 % (see the eco lap test)


def mode_config(mode: str) -> dict:
    with open(HACKUAB / f"{mode}.yaml") as f:
        return yaml.safe_load(f) or {}


def field_values(p) -> dict:
    return {f.name: getattr(p, f.name) for f in fields(p)}


def run_lap(mode: str, tmp_path, track: str = "practice_technical") -> dict:
    """One full bicycle lap in the given mode (strict: module errors raise). Returns summary metrics."""
    try:
        import autocross
    except ImportError:
        pytest.skip("simulator not installed")
    sim = Path(autocross.__file__).resolve().parents[1]
    trk = sim / "tracks" / f"{track}.yaml"
    if not trk.exists():
        pytest.skip("simulator tracks not found")
    cfg = tmp_path / f"{mode}_strict.yaml"
    cfg.write_text((HACKUAB / f"{mode}.yaml").read_text() + "\nstrict: true\n")
    out = subprocess.run([sys.executable, "-m", "autocross", "run", "-c", str(HACKUAB / "controller.py"),
                          "-t", str(trk), "--backend", "bicycle", "--no-render", "--team", f"modetest_{mode}",
                          "--controller-config", str(cfg)], cwd=sim, capture_output=True, text=True, timeout=300).stdout
    run = re.search(r"^\s+-> (.+)$", out, re.M).group(1).strip()
    metrics = json.load(open(os.path.join(run, "summary.json")))["metrics"]
    metrics["budget_guard_trips"] = budget_guard_trips(run)
    return metrics


# ------------------------------------------------------------------------------------------ PERFORMANCE

@pytest.mark.performance
def test_no_config_means_performance_mode():
    p = params.Params.from_vehicle(VEHICLE, {})
    assert p.mode == "performance" and p.eco_weight == 0.0


@pytest.mark.performance
def test_performance_yaml_equals_no_config():
    """performance.yaml must not change a single parameter of the scored default."""
    cfg = mode_config("performance")
    assert cfg == {"mode": "performance"}
    assert field_values(params.Params.from_vehicle(VEHICLE, cfg)) == field_values(params.Params.from_vehicle(VEHICLE, {}))


@pytest.mark.performance
def test_performance_yaml_reproduces_golden_commands():
    if not os.path.exists(GOLDEN):
        pytest.skip("golden file missing")
    gold, got = np.load(GOLDEN), run_sequence(mode_config("performance"))
    for key in gold.files:
        np.testing.assert_array_equal(got[key], gold[key], err_msg=f"{key} differs in performance mode")


@pytest.mark.performance
@pytest.mark.sim
def test_performance_lap_practice_technical(tmp_path):
    m = run_lap("performance", tmp_path)
    assert m["completion"]["finished"]
    assert_reference_lap(m["completion"]["time_s"], m["budget_guard_trips"])
    assert m["compute"]["errors"] == 0 and sum(m["penalties"].values()) == 0


# ------------------------------------------------------------------------------------------ ECO

@pytest.mark.eco
def test_eco_yaml_selects_eco_profile():
    p = params.Params.from_vehicle(VEHICLE, mode_config("eco"))
    assert p.mode == "eco"
    assert p.eco_weight == pytest.approx(params.MODES["eco"]["eco_weight"]) and p.eco_weight > 0.0


@pytest.mark.eco
def test_explicit_key_overrides_mode():
    p = params.Params.from_vehicle(VEHICLE, {"mode": "eco", "eco_weight": 0.6})
    assert p.mode == "eco" and p.eco_weight == pytest.approx(0.6)


@pytest.mark.eco
@pytest.mark.performance
def test_unknown_mode_fails_loudly():
    with pytest.raises(ValueError, match="unknown mode"):
        params.Params.from_vehicle(VEHICLE, {"mode": "ecco"})


@pytest.mark.eco
def test_eco_yaml_drives_differently_and_saves_energy():
    perf, eco = run_sequence(mode_config("performance")), run_sequence(mode_config("eco"))
    for r in (perf, eco):
        assert not np.any((r["throttle"] > 0) & (r["brake"] > 0))
        assert np.all(np.isfinite(r["steer"]))
    assert not np.array_equal(perf["throttle"], eco["throttle"])
    assert np.sum(eco["throttle"] * eco["speed"]) < np.sum(perf["throttle"] * perf["speed"])
    assert eco["speed"].max() < perf["speed"].max()


@pytest.mark.eco
@pytest.mark.sim
def test_eco_lap_practice_technical_is_clean(tmp_path):
    m = run_lap("eco", tmp_path)
    assert m["completion"]["finished"]
    assert m["compute"]["errors"] == 0 and sum(m["penalties"].values()) == 0
    # Slower than performance, but bounded. The Eco knee point was picked for <= 10 % lap-time loss over all
    # courses together; with the PR #7 defaults practice_technical alone is +10.2 % (40.59 s vs 36.84 s) while
    # the 5 public courses together are +7.8 %, so a single course gets 12 % headroom.
    assert MAIN_TECHNICAL_TIME_S < m["completion"]["time_s"] <= ECO_MAX_LAP_RATIO * MAIN_TECHNICAL_TIME_S
