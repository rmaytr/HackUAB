"""Cross-check hackuab/predict.py against harness telemetry (dev only, not submitted).

Replays 1.5 s windows of a run: each window starts from the logged state and is driven by the
logged commands (steer command through our slew/lag model, applied pedals). The predicted yaw
rate, lateral velocity, heading and position are compared with what the simulator did.
All windows are evaluated in one batched rollout_dynamic() call.

    simulator\\.venv\\Scripts\\python.exe tools\\predict_replay.py [RUN_DIR] [--sim PATH] [--horizon 1.5]

RUN_DIR defaults to the newest *practice_technical* run in <sim>/runs.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hackuab"))

from autocross.api import VehicleParams  # noqa: E402
from params import Params  # noqa: E402
import predict  # noqa: E402

COLS = ("t", "x", "y", "yaw", "vx", "vy", "yaw_rate", "ax", "steer_angle", "cmd_steer", "app_steer",
        "app_throttle", "app_brake", "ctrl_step")


def load_vehicle(sim: Path) -> tuple[VehicleParams, Params, float]:
    cfg = yaml.safe_load((sim / "config" / "sim.yaml").read_text())
    v, b = cfg["vehicle"], cfg["vehicle"]["bicycle"]
    vp = VehicleParams(mass=b["mass"], wheelbase=b["wheelbase"], track_width=b["track_width"], length=b["length"],
                       width=b["width"], cg_to_front=b["cg_to_front"], cg_to_rear=b["wheelbase"] - b["cg_to_front"],
                       max_steer=math.radians(v["max_steer_deg"]), max_steer_rate=v["steer_rate_limit"], mu=v["mu"],
                       max_accel=v["max_accel"], max_decel=v["max_decel"], max_speed=v["max_speed"],
                       control_dt=1.0 / cfg["sim"]["control_hz"], actuation_delay_steps=cfg["actuators"]["delay_steps"])
    overrides = {k: b[k] for k in ("cg_height", "cornering_stiffness_per_load", "max_drive_force", "max_power_w",
                                   "max_brake_force", "drag_area", "rolling_resistance", "steer_time_constant",
                                   "front_drive_fraction", "traction_limit") if k in b}
    overrides["mu_model"] = b["mu"]
    p = Params.from_vehicle(vp)
    for k, val in overrides.items():   # extra keys (traction_limit, ...) are read by predict via getattr
        setattr(p, k, float(val))
    return vp, p, float(cfg["sim"]["physics_dt"])


def load_log(run: Path) -> dict[str, np.ndarray]:
    with open(run / "telemetry.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    return {c: np.array([float(r[c]) if r[c] else np.nan for r in rows]) for c in COLS}


def rms(a: np.ndarray) -> float:
    return float(np.sqrt(np.nanmean(a ** 2)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?", type=Path)
    ap.add_argument("--sim", type=Path, default=ROOT / "simulator")
    ap.add_argument("--horizon", type=float, default=1.5)
    ap.add_argument("--stride", type=float, default=0.25, help="s between window starts")
    a = ap.parse_args()
    run = a.run or max((a.sim / "runs").glob("*practice_technical*"), key=lambda p: p.stat().st_mtime)
    vp, p, phys_dt = load_vehicle(a.sim)
    log = load_log(run)
    dt = vp.control_dt
    tpc = int(round(dt / phys_dt))                      # physics ticks per control step
    steps = int(round(a.horizon / dt))

    # row k (ctrl_step == 1) = state after the tick that first applied the command decided from row k-1
    ctrl = np.flatnonzero(log["ctrl_step"] > 0.5)
    stride = max(int(round(a.stride / dt)), 1)
    starts = ctrl[(ctrl >= 1) & (ctrl + tpc * steps - 1 < len(log["t"]))][::stride]
    starts = starts[log["vx"][starts - 1] > 1.0]          # skip the standing start
    i0 = starts - 1
    cmd_idx = starts[:, None] + tpc * np.arange(steps)[None, :]      # (B,T) rows where each command applies
    tgt_idx = cmd_idx + tpc - 1                                       # (B,T) row after the step

    # initial states in each window's own start frame (x, y, yaw = 0)
    state = np.zeros((len(starts), 7))
    state[:, 3], state[:, 4], state[:, 5] = log["vx"][i0], log["vy"][i0], log["yaw_rate"][i0]
    state[:, 6] = log["steer_angle"][i0]
    steer = log["cmd_steer"][cmd_idx]
    pedal = log["app_throttle"][cmd_idx] - log["app_brake"][cmd_idx]
    c0, s0 = np.cos(log["yaw"][i0])[:, None], np.sin(log["yaw"][i0])[:, None]
    dx, dy = log["x"][tgt_idx] - log["x"][i0][:, None], log["y"][tgt_idx] - log["y"][i0][:, None]
    true_x, true_y = c0 * dx + s0 * dy, c0 * dy - s0 * dx
    true_yaw = np.angle(np.exp(1j * (log["yaw"][tgt_idx] - log["yaw"][i0][:, None])))
    true_r, true_vy = log["yaw_rate"][tgt_idx], log["vy"][tgt_idx]

    print(f"run: {run.name}\nwindows: {len(starts)} x {a.horizon:.2f} s ({steps} steps), stride {a.stride} s\n")
    print(f"{'model':<26}{'yaw-rate RMS':>13}{'vy RMS':>9}{'yaw RMS':>9}{'pos@0.5s':>10}{'pos@1.0s':>10}"
          f"{'pos@end':>9}{'pos@end p95':>12}{'ms':>8}")
    print(f"{'':<26}{'rad/s':>13}{'m/s':>9}{'deg':>9}{'m':>10}{'m':>10}{'m':>9}{'m':>12}{'':>8}")

    def report(name: str, x, y, yaw, r, vy, ms):
        perr = np.hypot(x - true_x, y - true_y)
        yerr = np.degrees(np.angle(np.exp(1j * (yaw - true_yaw))))
        at = lambda s: float(np.mean(perr[:, min(int(round(s / dt)), steps) - 1]))  # noqa: E731
        print(f"{name:<26}{rms(r - true_r):>13.4f}{rms(vy - true_vy):>9.4f}{rms(yerr):>9.3f}{at(0.5):>10.3f}"
              f"{at(1.0):>10.3f}{at(a.horizon):>9.3f}{np.percentile(perr[:, -1], 95):>12.3f}{ms:>8.2f}")

    for sub in (1, 2):
        t0 = time.perf_counter()
        out = predict.rollout_dynamic(state, steer, pedal, vp, p, dt, steer_applied=log["app_steer"][i0],
                                      ax0=log["ax"][i0], substeps=sub)
        ms = (time.perf_counter() - t0) * 1e3
        o = out[:, 1:]
        report(f"dynamic (substeps={sub})", o[..., 0], o[..., 1], o[..., 2], o[..., 5], o[..., 4], ms)

    # reference: kinematic bicycle fed the logged road-wheel angle and logged longitudinal accel
    t0 = time.perf_counter()
    kin = predict.rollout_kinematic(np.column_stack((np.zeros((len(starts), 3)), log["vx"][i0])),
                                    log["steer_angle"][cmd_idx], log["ax"][cmd_idx], vp.wheelbase, dt)
    ms = (time.perf_counter() - t0) * 1e3
    k = kin[:, 1:]
    r_kin = k[..., 3] * np.tan(log["steer_angle"][tgt_idx]) / vp.wheelbase
    report("kinematic (logged steer)", k[..., 0], k[..., 1], k[..., 2], r_kin, r_kin * vp.cg_to_rear, ms)
    print(f"\nlogged |yaw rate| RMS {rms(true_r):.3f} rad/s, |vy| RMS {rms(true_vy):.3f} m/s (scale of the signals)")


if __name__ == "__main__":
    main()
