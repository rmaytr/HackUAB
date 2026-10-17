"""Tracker: turns a Plan into (steer, throttle, brake).

Contract (CLAUDE.md, "Data contracts"):
    track(plan, obs, vehicle, p, state) -> (steer, throttle, brake)
    clipped, NaN-free, throttle and brake never both > 0.

What it does each step (all O(N) numpy, no loops over the path):
  1. Latency compensation: predict where the car will be when this command lands
     (one control period plus any actuation delay) and track from that pose.
  2. Lateral: curvature feedforward from plan.kappa at the car's station, plus feedback from
     pure pursuit on plan.path (speed-scaled lookahead along the path) with the arc a car
     already on the path would need subtracted, so only tracking error drives the correction
     and the car does not cut curvature changes.
  3. Steering guards: grip cap (keeps commanded lateral demand under the dynamics metric's
     110% threshold) and a slew limit just below the actuator's rate limit.
  4. Longitudinal: feedforward accel from the plan's v(s) + PI on speed error.
  5. Friction ellipse: longitudinal accel capped by the lateral grip already in use.
  6. Online slip adaptation: compares expected yaw rate v/L*tan(delta) with the measured one
     and rear-axle slip; shrinks the usable lateral grip when the tyres saturate and recovers
     it slowly. The result goes in state["a_lat_scale"] for the planner to read.

`plan` is duck-typed (needs .path, .s, .kappa, .v). Tunables are the tracking section of
params.Params; `p` is read with getattr, falling back to the Params defaults, so a partial
namespace (tests, tools/) works too. Debug values for Command.debug are left in state["debug"],
and the speed target in state["v_target"].
"""
from __future__ import annotations

import math
from dataclasses import fields

import numpy as np

from params import G, Params

DEFAULTS = {f.name: f.default for f in fields(Params)}


def _p(p, name):
    return float(getattr(p, name, DEFAULTS[name])) if p is not None else DEFAULTS[name]


def _finite(x, fallback=0.0):
    return float(x) if math.isfinite(x) else fallback


def track(plan, obs, vehicle, p, state: dict) -> tuple[float, float, float]:
    """Return (steer, throttle, brake), already clipped, NaN-free, and with throttle and brake never both > 0."""
    dt = vehicle.control_dt
    L = vehicle.wheelbase
    v = max(_finite(obs.speed), 0.0)
    mu_g = vehicle.mu * G

    # ------------------------------------------------------------------ slip adaptation
    scale = _update_slip(obs, vehicle, p, state, v)
    a_lat_max = _p(p, "ellipse_lat_frac") * mu_g * scale

    # ------------------------------------------------------------------ latency compensation
    t_pred = dt * (1 + int(getattr(vehicle, "actuation_delay_steps", 0)))
    r = _finite(obs.yaw_rate)
    half = 0.5 * r * t_pred
    d = v * t_pred
    xp, yp, yawp = d * math.cos(half), d * math.sin(half), r * t_pred

    path = np.asarray(plan.path, dtype=float)
    s = np.asarray(plan.s, dtype=float)
    c, sn = math.cos(yawp), math.sin(yawp)
    dx, dy = path[:, 0] - xp, path[:, 1] - yp
    px = c * dx + sn * dy          # path in the predicted vehicle frame
    py = -sn * dx + c * dy

    # nearest point, then lookahead measured along the path
    i_near = int(np.argmin(px * px + py * py))
    ld = float(np.clip(_p(p, "pp_lookahead_min") + _p(p, "pp_lookahead_gain") * v,
                       _p(p, "pp_lookahead_min"), _p(p, "pp_lookahead_max")))
    s_tgt = s[i_near] + ld
    if s_tgt <= s[-1]:
        tx, ty = float(np.interp(s_tgt, s, px)), float(np.interp(s_tgt, s, py))
    else:  # path shorter than the lookahead: extend along its last segment
        ex, ey = px[-1] - px[-2], py[-1] - py[-2]
        n = math.hypot(ex, ey) or 1.0
        extra = s_tgt - s[-1]
        tx, ty = px[-1] + ex / n * extra, py[-1] + ey / n * extra
    dist2 = max(tx * tx + ty * ty, 1.0)
    kappa_pp = 2.0 * ty / dist2    # pure-pursuit arc through the target

    # Pure pursuit alone cuts curvature changes (it steers for the target, not the path under
    # the car). Split it: feedforward = the plan's curvature where the car is, feedback = the
    # part of the pure-pursuit arc caused by tracking error, i.e. minus the arc a car sitting
    # exactly on the path at i_near would compute for the same target.
    hx, hy = _tangent(px, py, i_near)
    rx, ry = tx - px[i_near], ty - py[i_near]
    ty_ideal = -hy * rx + hx * ry
    kappa_ideal = 2.0 * ty_ideal / max(rx * rx + ry * ry, 1.0)
    # read the curvature slightly ahead: the car's yaw response lags the steering wheel
    kappa_ff = float(np.interp(s[i_near] + v * _p(p, "ff_preview_t"), s, np.asarray(plan.kappa, dtype=float)))
    kappa_cmd = kappa_ff + _p(p, "fb_gain") * (kappa_pp - kappa_ideal)

    # ------------------------------------------------------------------ steering + guards
    tan_cap = _p(p, "grip_cap") * mu_g * L / max(v * v, 1e-3)
    tan_d = float(np.clip(L * kappa_cmd, -tan_cap, tan_cap))
    steer_raw = math.atan(tan_d) / vehicle.max_steer
    prev = state.get("steer_prev", _finite(obs.steer))
    max_step = _p(p, "steer_rate_margin") * vehicle.max_steer_rate * dt
    steer = float(np.clip(np.clip(steer_raw, prev - max_step, prev + max_step), -1.0, 1.0))
    state["steer_prev"] = steer

    # ------------------------------------------------------------------ longitudinal
    vp = np.asarray(plan.v, dtype=float)
    s0 = s[i_near]
    s1 = s0 + max(v * _p(p, "speed_preview_t"), 1.0)
    v0, v1 = float(np.interp(s0, s, vp)), float(np.interp(s1, s, vp))
    a_ff = (v1 * v1 - v0 * v0) / (2.0 * (s1 - s0))
    v_ref = v1
    err = v_ref - v
    if err > 0.0:
        # Below the profile its downward slope is not a demand: the profile always falls toward
        # the horizon-end guard, and following that slope here caps speed short of the target
        # (on CARLA's weaker drivetrain the car stalled at 13 m/s against a 17 m/s target).
        a_ff = max(a_ff, 0.0)
    i_lim = _p(p, "i_limit")
    state["i_err"] = float(np.clip(state.get("i_err", 0.0) + _p(p, "ki_speed") * err * dt, -i_lim, i_lim))
    a_cmd = a_ff + _p(p, "kp_speed") * err + state["i_err"]

    # friction ellipse: grip used laterally is not available longitudinally. Lateral load is the
    # larger of what the steering asks now and what the path asks just ahead (corner entry).
    # lon_frac = 1 - (a_lat/a_lat_max)^2 sits inside the true ellipse sqrt(1 - r^2): braking
    # scales against max_decel, which is above the lateral limit, and the rear axle of this car
    # lets go first under combined braking and cornering.
    delta = steer * vehicle.max_steer
    a_lat = max(v * v * abs(math.tan(delta)) / L, v * v * abs(kappa_ff))
    lon_frac = max(0.0, 1.0 - min(a_lat / max(a_lat_max, 1e-3), 1.0) ** 2)
    a_up = vehicle.max_accel * max(lon_frac, _p(p, "accel_floor"))
    a_dn = vehicle.max_decel * max(lon_frac, _p(p, "brake_floor"))
    a_cmd = float(np.clip(a_cmd, -a_dn, a_up))

    # jerk limit: ramp the pedals so load transfer does not unsettle the rear
    a_prev = state.get("a_prev", a_cmd)
    j = _p(p, "jerk_max") * dt
    a_cmd = float(np.clip(a_cmd, a_prev - j, a_prev + j))
    state["a_prev"] = a_cmd

    # pedals, mutually exclusive by construction
    if a_cmd >= 0.0:
        throttle, brake = a_cmd / vehicle.max_accel, 0.0
    elif a_cmd > -_p(p, "coast_band"):
        throttle, brake = 0.0, 0.0
    else:
        throttle, brake = 0.0, -a_cmd / vehicle.max_decel
    throttle = float(np.clip(_finite(throttle), 0.0, 1.0))
    brake = float(np.clip(_finite(brake), 0.0, 1.0))
    if throttle > 0.0 and brake > 0.0:      # unreachable, kept as a hard guarantee
        brake = 0.0
    if throttle == 0.0 and brake == 0.0 and v < 0.5 and v_ref < 0.5:
        brake = 0.3                          # hold still when the plan says stop

    state["v_target"] = v_ref
    state["debug"] = {
        "v_target": v_ref, "a_cmd": a_cmd, "a_lat": a_lat, "lon_frac": lon_frac,
        "lookahead": ld, "kappa_cmd": kappa_cmd, "kappa_ff": kappa_ff, "kappa_pp": kappa_pp,
        "kappa_ideal": kappa_ideal, "steer_raw": steer_raw,
        "a_lat_max_est": a_lat_max, "slip": state.get("slip", 0.0),
    }
    return _finite(steer), throttle, brake


def _tangent(px, py, i):
    """Unit tangent at point i. Segment headings are averaged around i, and extrapolated at the
    ends, so a constant-curvature path gives the exact tangent at every point."""
    n = len(px)
    if n < 3:
        th = math.atan2(py[-1] - py[0], px[-1] - px[0])
        return math.cos(th), math.sin(th)

    def seg(j):  # heading of segment j -> j+1
        return math.atan2(py[j + 1] - py[j], px[j + 1] - px[j])

    if i <= 0:
        a, b = seg(0), seg(1)
        th = a - 0.5 * _wrap(b - a)
    elif i >= n - 1:
        a, b = seg(n - 3), seg(n - 2)
        th = b + 0.5 * _wrap(b - a)
    else:
        a, b = seg(i - 1), seg(i)
        th = a + 0.5 * _wrap(b - a)
    return math.cos(th), math.sin(th)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _update_slip(obs, vehicle, p, state: dict, v: float) -> float:
    """Shrink usable lateral grip when the car understeers or the rear slides; recover slowly."""
    scale = state.get("a_lat_scale", 1.0)
    slip = state.get("slip", 0.0)       # low-passed understeer indicator, 0 = tracks kinematics
    rear_ok = True
    if v > 5.0:
        delta = _finite(obs.steer) * vehicle.max_steer
        r_exp = v / vehicle.wheelbase * math.tan(delta)
        r_meas = _finite(obs.yaw_rate)
        if abs(r_exp) > 0.15:
            raw = float(np.clip(1.0 - r_meas / r_exp, -1.0, 1.0))
            a = _p(p, "slip_ema_alpha")
            slip = (1.0 - a) * slip + a * raw
        vx = max(abs(_finite(obs.vx)), 0.1)
        rear = math.degrees(math.atan2(_finite(obs.vy) - vehicle.cg_to_rear * r_meas, vx))
        rear_ok = abs(rear) <= _p(p, "rear_slip_max_deg")
    else:
        slip *= 0.9
    if slip > _p(p, "slip_threshold") or not rear_ok:
        scale = max(scale - _p(p, "slip_drop"), _p(p, "slip_scale_min"))
    else:
        scale = min(scale + _p(p, "slip_recover"), 1.0)
    state["a_lat_scale"] = scale
    state["slip"] = slip
    return scale
