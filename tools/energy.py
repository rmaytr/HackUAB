"""Energy meter for bicycle-backend runs (dev only, NOT submitted). Owner: Rodion (added to tools/).

    .venv\\Scripts\\python.exe ..\\tools\\energy.py runs\\<run_dir> [<run_dir> ...] [--json]

Reads each run's telemetry.csv (+ summary.json for lap time) and rebuilds the wheel forces exactly
like simulator/autocross/backends/bicycle.py `substep()`:
    f_trac = throttle * min(F_drive, P / max(vx, 1))       f_brk = brake * F_brake
    per-axle split (AWD drive split, brake split), then traction control / ABS clip
    |Fx_axle| <= traction_limit * mu * Fz_axle, with load transfer from the previous row's ax
    drag = 0.5 * 1.2 * CdA * vx|vx|,  rolling = Crr * m * g only while vx > 0.05
The CLIPPED axle forces are integrated (rows come at an uneven rate, so dt comes from column t):
    E_trac  = sum(max(Fx_axle, 0)) * vx * dt          (positive tractive work at the wheels)
    E_brake = sum(max(-Fx_axle, 0)) * vx * dt         (work absorbed by the brakes)
    E_drag, E_roll likewise.
Energy balance check: E_trac - E_brake - E_drag - E_roll should equal the change in kinetic energy
plus the cornering loss. That remainder (raw residual, ~7-12 % of E_trac on our runs) is the work of
the lateral tyre forces: the slip-angle 'scrub' that also slows the car in every corner. We rebuild it
with bicycle._lat() from vy, yaw_rate and steer_angle (E_tyre). The residual after it (in % of E_trac)
has to stay < 5 % before any number is trusted; what is left is the 40 Hz sampling of a 400 Hz
physics loop, the kinematic low-speed blend and yaw kinetic energy.

Battery energy is an ESTIMATE on top of the simulator, which has no electric drivetrain and no regen:
    E_batt = E_trac / eta_drive - eta_regen * sum(min(P_brake, P_regen_max) * dt)
eta_drive, eta_regen and P_regen_max are ASSUMED round numbers for a generic EV (CLI flags below),
NOT Chevy Blazer EV specifications.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

G = 9.81
RHO = 1.2                      # kg/m^3, same constant as bicycle.py

# vehicle.bicycle defaults (simulator/config/sim.yaml, with bicycle.py DEFAULTS for missing keys)
VEHICLE = {"mass": 1845.0, "max_drive_force": 9900.0, "max_power_w": 200000.0, "max_brake_force": 17500.0,
           "drag_area": 0.55, "rolling_resistance": 0.012, "mu": 0.97, "traction_limit": 0.97,
           "front_drive_fraction": 0.4, "front_brake_fraction": 0.6, "cg_height": 0.5, "cg_to_front": 1.45,
           "wheelbase": 3.0, "cornering_stiffness_per_load": 14.0}

# ASSUMPTIONS (not vehicle specs): generic EV drive efficiency, regen round-trip, regen power cap
ETA_DRIVE, ETA_REGEN, P_REGEN_MAX = 0.90, 0.65, 60e3


def load_vehicle(sim_yaml: str | None = None) -> dict:
    """VEHICLE constants, overridden by simulator/config/sim.yaml vehicle.bicycle when readable."""
    v = dict(VEHICLE)
    path = sim_yaml or os.path.join(os.path.dirname(__file__), "..", "simulator", "config", "sim.yaml")
    try:
        import yaml
        with open(path) as f:
            bic = (yaml.safe_load(f) or {}).get("vehicle", {}).get("bicycle", {}) or {}
        v.update({k: float(bic[k]) for k in v if k in bic})
    except (OSError, ImportError):
        pass
    return v


def read_telemetry(path: str) -> dict[str, np.ndarray]:
    cols = ("t", "vx", "vy", "ax", "yaw_rate", "steer_angle", "app_throttle", "app_brake")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    def num(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return math.nan

    out = {c: np.array([num(r.get(c)) for r in rows]) for c in cols}
    for c in cols:
        out[c] = np.nan_to_num(out[c])
    return out


def wheel_forces(vx, ax_prev, throttle, brake, veh: dict):
    """Clipped front/rear longitudinal tyre forces + resistances, as in bicycle.substep()."""
    m, L, lf = veh["mass"], veh["wheelbase"], veh["cg_to_front"]
    lr = L - lf
    f_trac = throttle * np.minimum(veh["max_drive_force"], veh["max_power_w"] / np.maximum(vx, 1.0))
    f_brk = brake * veh["max_brake_force"]
    f_drag = 0.5 * RHO * veh["drag_area"] * vx * np.abs(vx)
    f_roll = veh["rolling_resistance"] * m * G * (vx > 0.05)
    fzf = m * (G * lr - ax_prev * veh["cg_height"]) / L
    fzr = m * G - fzf
    fd, fb = veh["front_drive_fraction"], veh["front_brake_fraction"]
    lim = veh["traction_limit"] * veh["mu"]
    fxf = np.clip(fd * f_trac - fb * f_brk, -lim * fzf, lim * fzf)
    fxr = np.clip((1.0 - fd) * f_trac - (1.0 - fb) * f_brk, -lim * fzr, lim * fzr)
    return fxf, fxr, f_drag, f_roll


def tyre_slip_power(vx, vy, r, delta, fxf, fxr, ax_prev, veh: dict) -> np.ndarray:
    """Power dissipated by the lateral tyre forces (>= 0): -(Fy_f * v_lat_f + Fy_r * v_lat_r), with the
    tyre law of bicycle._lat() (saturating, friction circle shared with Fx). This is the cornering
    'scrub' loss: the same forces that turn the car also slow it down."""
    m, L, lf = veh["mass"], veh["wheelbase"], veh["cg_to_front"]
    lr = L - lf
    fzf = m * (G * lr - ax_prev * veh["cg_height"]) / L
    fzr = m * G - fzf
    vxs = np.maximum(vx, 1.0)
    af = np.arctan2(vy + lf * r, vxs) - delta
    ar = np.arctan2(vy - lr * r, vxs)

    def lat(alpha, fz, fx):
        avail = np.sqrt(np.maximum((veh["mu"] * fz) ** 2 - fx ** 2, 0.0))
        return -avail * np.tanh(veh["cornering_stiffness_per_load"] * fz * alpha / np.maximum(avail, 1.0))

    fyf, fyr = lat(af, fzf, fxf), lat(ar, fzr, fxr)
    v_lat_f = -vx * np.sin(delta) + (vy + lf * r) * np.cos(delta)     # front contact patch, wheel frame
    v_lat_r = vy - lr * r
    return np.maximum(-(fyf * v_lat_f + fyr * v_lat_r), 0.0) * (vx > 1.0)


def analyse(tel: dict[str, np.ndarray], veh: dict, eta_drive=ETA_DRIVE, eta_regen=ETA_REGEN,
            p_regen_max=P_REGEN_MAX, lap_time: float | None = None) -> dict:
    """Energy figures of one telemetry trace. Row k's applied pedals act over (t[k-1], t[k]]."""
    t, vx, vy, ax = tel["t"], tel["vx"], tel["vy"], tel["ax"]
    thr, brk = np.clip(tel["app_throttle"], 0, 1), np.clip(tel["app_brake"], 0, 1)
    m = veh["mass"]
    n = len(t)
    if n < 2:
        raise ValueError("telemetry has fewer than 2 rows")
    t0 = np.concatenate(([0.0], t[:-1]))
    dt = np.maximum(t - t0, 0.0)
    v_start = np.concatenate(([0.0], vx[:-1]))
    v_mid = 0.5 * (v_start + vx)                      # speed over the interval (trapezoid)
    ax_prev = np.concatenate(([0.0], ax[:-1]))        # load transfer uses last tick's accel
    # the power limit sees the speed inside the interval: evaluate at the interval mean
    fxf, fxr, f_drag, f_roll = wheel_forces(v_mid, ax_prev, thr, brk, veh)
    p_f, p_r = fxf * v_mid, fxr * v_mid
    p_trac = np.maximum(p_f, 0.0) + np.maximum(p_r, 0.0)
    p_brake = np.maximum(-p_f, 0.0) + np.maximum(-p_r, 0.0)
    e_trac = float(np.sum(p_trac * dt))
    e_brake = float(np.sum(p_brake * dt))
    e_drag = float(np.sum(f_drag * v_mid * dt))
    e_roll = float(np.sum(f_roll * v_mid * dt))
    mid = lambda a: 0.5 * (np.concatenate(([a[0]], a[:-1])) + a)
    e_tyre = float(np.sum(tyre_slip_power(v_mid, mid(vy), mid(tel["yaw_rate"]), mid(tel["steer_angle"]),
                                          fxf, fxr, ax_prev, veh) * dt))
    ke = 0.5 * m * (vx * vx + vy * vy)
    d_ke = float(ke[-1] - 0.5 * m * 0.0)              # cars start at rest
    raw = e_trac - e_brake - e_drag - e_roll - d_ke          # the plain longitudinal balance
    residual = raw - e_tyre                                    # after the modelled cornering loss
    recovered = float(eta_regen * np.sum(np.minimum(p_brake, p_regen_max) * dt))
    e_batt = e_trac / eta_drive - recovered
    dist = float(np.sum(v_mid * dt))
    jerk = np.diff(ax) / np.maximum(np.diff(t), 1e-6)
    wh = 1.0 / 3600.0
    return {
        "lap_time_s": lap_time if lap_time is not None else float(t[-1]),
        "distance_m": dist,
        "E_batt_Wh": e_batt * wh,
        "Wh_per_km": e_batt * wh / max(dist / 1000.0, 1e-9),
        "E_trac_Wh": e_trac * wh, "E_drag_Wh": e_drag * wh, "E_roll_Wh": e_roll * wh,
        "E_brake_Wh": e_brake * wh, "E_regen_Wh": recovered * wh,
        "drive_loss_Wh": e_trac * (1.0 / eta_drive - 1.0) * wh,
        "dKE_Wh": d_ke * wh,
        "E_tyre_Wh": e_tyre * wh,
        "raw_residual_pct": 100.0 * raw / max(e_trac, 1e-9),
        "residual_pct": 100.0 * residual / max(e_trac, 1e-9),
        "peak_trac_kW": float(np.max(p_trac)) / 1e3,
        "rms_jerk": float(np.sqrt(np.mean(jerk * jerk))) if len(jerk) else 0.0,
        "assumptions": {"eta_drive": eta_drive, "eta_regen": eta_regen, "P_regen_max_kW": p_regen_max / 1e3},
    }


def analyse_run(run_dir: str, veh: dict | None = None, **kw) -> dict:
    veh = veh or load_vehicle()
    tel = read_telemetry(os.path.join(run_dir, "telemetry.csv"))
    lap = None
    try:
        with open(os.path.join(run_dir, "summary.json")) as f:
            lap = json.load(f)["metrics"]["completion"]["time_s"]
    except (OSError, KeyError, ValueError):
        pass
    out = analyse(tel, veh, lap_time=lap, **kw)
    out["run"] = os.path.basename(os.path.normpath(run_dir))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+", help="run folders containing telemetry.csv")
    ap.add_argument("--eta-drive", type=float, default=ETA_DRIVE, help="ASSUMED battery-to-wheel efficiency")
    ap.add_argument("--eta-regen", type=float, default=ETA_REGEN, help="ASSUMED wheel-to-battery regen efficiency")
    ap.add_argument("--p-regen-max", type=float, default=P_REGEN_MAX / 1e3, help="ASSUMED regen power cap, kW")
    ap.add_argument("--sim-yaml", default=None, help="sim.yaml to read vehicle.bicycle from")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    veh = load_vehicle(a.sim_yaml)
    res = [analyse_run(r, veh, eta_drive=a.eta_drive, eta_regen=a.eta_regen, p_regen_max=a.p_regen_max * 1e3)
           for r in a.runs]
    if a.json:
        print(json.dumps(res, indent=1))
        return 0
    print(f"assumed: eta_drive {a.eta_drive}, eta_regen {a.eta_regen}, P_regen_max {a.p_regen_max:g} kW "
          "(simulator has no regen; these are not vehicle specs)")
    hdr = (f"{'run':48} {'lap s':>7} {'Wh':>7} {'Wh/km':>7} {'drag':>6} {'roll':>6} {'brake':>6} {'regen':>6} "
           f"{'tyre':>6} {'peakkW':>7} {'jerk':>6} {'raw%':>6} {'resid%':>7}")
    print(hdr)
    for r in res:
        print(f"{r['run'][-48:]:48} {r['lap_time_s'] or float('nan'):7.2f} {r['E_batt_Wh']:7.1f} "
              f"{r['Wh_per_km']:7.1f} {r['E_drag_Wh']:6.1f} {r['E_roll_Wh']:6.1f} {r['E_brake_Wh']:6.1f} "
              f"{r['E_regen_Wh']:6.1f} {r['E_tyre_Wh']:6.1f} {r['peak_trac_kW']:7.1f} {r['rms_jerk']:6.2f} "
              f"{r['raw_residual_pct']:6.2f} {r['residual_pct']:7.2f}")
        if abs(r["residual_pct"]) >= 5.0:
            print(f"  WARNING: energy balance residual {r['residual_pct']:.1f} % >= 5 %, do not trust this run")
    return 0


if __name__ == "__main__":
    sys.exit(main())
