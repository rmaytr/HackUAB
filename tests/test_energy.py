"""tools/energy.py on a synthetic telemetry.csv with a known answer: accelerate, cruise, brake."""
import csv
import importlib.util
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.eco          # energy meter: part of the ECO mode test group

_spec = importlib.util.spec_from_file_location("energy", Path(__file__).resolve().parents[1] / "tools" / "energy.py")
energy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(energy)

DT = 0.025
THROTTLE, BRAKE = 0.3, 0.2


def _veh(**kw):
    v = dict(energy.VEHICLE)
    v.update(kw)
    return v


def _write(tmp_path, veh):
    """Rows like the harness writes them: state after each 25 ms tick, pedals applied during it."""
    m = veh["mass"]
    f_acc, f_brk = THROTTLE * veh["max_drive_force"], BRAKE * veh["max_brake_force"]
    rows, t, v = [], 0.0, 0.0
    phases = [(THROTTLE, 0.0, f_acc / m, 200), (0.0, 0.0, 0.0, 120), (0.0, BRAKE, -f_brk / m, None)]
    for thr, brk, a, n in phases:
        k = 0
        while (n is None and v > 0.0) or (n is not None and k < n):
            v_new = max(v + a * DT, 0.0)
            t += DT
            rows.append({"t": t, "vx": v_new, "vy": 0.0, "ax": (v_new - v) / DT, "yaw_rate": 0.0,
                         "steer_angle": 0.0, "app_throttle": thr, "app_brake": brk})
            v, k = v_new, k + 1
    path = tmp_path / "telemetry.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return tmp_path, f_acc, f_brk


def test_known_answer_without_resistances(tmp_path):
    veh = _veh(drag_area=0.0, rolling_resistance=0.0)
    run, f_acc, f_brk = _write(tmp_path, veh)
    m = veh["mass"]
    a = f_acc / m
    t_acc = 200 * DT
    v_top = a * t_acc
    d_acc = 0.5 * a * t_acc ** 2
    r = energy.analyse_run(str(run), veh, eta_drive=0.9, eta_regen=0.65, p_regen_max=60e3)
    wh = 1.0 / 3600.0
    assert r["E_trac_Wh"] == pytest.approx(f_acc * d_acc * wh, rel=1e-9)
    # the brake absorbs all the kinetic energy (last tick is clamped at v = 0, hence the tolerance)
    assert r["E_brake_Wh"] == pytest.approx(0.5 * m * v_top ** 2 * wh, rel=2e-3)
    assert f_brk * v_top < 60e3                               # never regen-capped in this test
    assert r["E_regen_Wh"] == pytest.approx(0.65 * r["E_brake_Wh"], rel=1e-9)
    assert r["E_batt_Wh"] == pytest.approx(r["E_trac_Wh"] / 0.9 - r["E_regen_Wh"], rel=1e-9)
    assert r["E_drag_Wh"] == 0.0 and r["E_roll_Wh"] == 0.0 and r["E_tyre_Wh"] == 0.0
    assert abs(r["residual_pct"]) < 0.5
    d_total = d_acc + v_top * 120 * DT + v_top ** 2 / (2 * f_brk / m)
    assert r["distance_m"] == pytest.approx(d_total, rel=2e-3)


def test_regen_cap_and_resistances(tmp_path):
    veh = _veh()
    run, _, _ = _write(tmp_path, veh)
    r = energy.analyse_run(str(run), veh, eta_regen=0.65, p_regen_max=10e3)
    assert r["E_regen_Wh"] < 0.65 * r["E_brake_Wh"]             # capped at 10 kW
    assert r["E_drag_Wh"] > 0.0 and r["E_roll_Wh"] > 0.0
    # rows were built without resistances, so the meter must report exactly that mismatch
    gap = r["E_trac_Wh"] - r["E_brake_Wh"] - r["E_drag_Wh"] - r["E_roll_Wh"] - r["dKE_Wh"]
    assert gap == pytest.approx(r["raw_residual_pct"] / 100 * r["E_trac_Wh"], rel=1e-9)


def test_traction_control_clip_uses_axle_limit():
    veh = _veh(mu=0.3)                                          # low grip: the AWD axles saturate
    vx = np.array([5.0])
    fxf, fxr, _, _ = energy.wheel_forces(vx, np.array([0.0]), np.array([1.0]), np.array([0.0]), veh)
    m, L, lf = veh["mass"], veh["wheelbase"], veh["cg_to_front"]
    fzf = m * energy.G * (L - lf) / L
    lim = veh["traction_limit"] * veh["mu"]
    assert fxf[0] == pytest.approx(lim * fzf)
    assert fxr[0] == pytest.approx(lim * (m * energy.G - fzf))
    assert fxf[0] + fxr[0] < veh["max_drive_force"]
