"""Stage 3: plan -> steer / throttle / brake. Owner: Viraj.

Phase 0 stub: the baseline's pure pursuit + PI speed control + friction-circle throttle cap.
Roadmap V2 adds delay compensation, feedforward, the full friction ellipse and slip adaptation.
"""
from __future__ import annotations

import math

import numpy as np

from autocross.api import Observation, VehicleParams
from params import G, Params
from planner import Plan


def track(plan: Plan, obs: Observation, vehicle: VehicleParams, p: Params, state: dict) -> tuple[float, float, float]:
    """Return (steer, throttle, brake), already clipped, with throttle and brake never both > 0.
    Writes scalar diagnostics into state["debug"] and the commanded speed into state["v_target"]."""
    v = max(obs.speed, 0.0)

    # lateral: pure pursuit to the plan point `ld` metres ahead along the path
    ld = max(p.lookahead_min, p.lookahead_gain * v + p.lookahead_offset)
    tx = float(np.interp(ld, plan.s, plan.path[:, 0]))
    ty = float(np.interp(ld, plan.s, plan.path[:, 1]))
    alpha = math.atan2(ty, max(tx, 0.1))
    delta = math.atan2(2.0 * vehicle.wheelbase * math.sin(alpha), max(math.hypot(tx, ty), 1.0))
    steer = float(np.clip(delta / vehicle.max_steer, -1.0, 1.0))

    # longitudinal: ramp the target up gently (it jumps when a corner leaves the preview), cut it at once
    v_tgt = min(float(plan.v[0]), state.get("v_ref", v) + p.a_acc * vehicle.control_dt)
    state["v_ref"] = v_tgt
    err = v_tgt - v
    state["i_err"] = float(np.clip(state.get("i_err", 0.0) + err * vehicle.control_dt, -p.i_limit, p.i_limit))
    u = p.kp * err + p.ki * state["i_err"]

    # friction circle: leave grip for cornering by capping throttle at high lateral acceleration
    lat_use = min(abs(obs.ay) / (vehicle.mu * G), 1.0)
    throttle_cap = max(1.0 - lat_use ** 2, p.min_throttle_cap)
    throttle, brake = (min(u, throttle_cap), 0.0) if u >= 0 else (0.0, min(-u * p.brake_gain, 1.0))

    state["v_target"] = v_tgt
    state["debug"] = {"lookahead": ld, "alpha": alpha, "v_target": v_tgt}
    return steer, float(throttle), float(brake)
