"""Physics-based prediction. Owner: Rodion.

Three tools, all pure functions (no module state), numpy + stdlib only:

* rollout_kinematic / predict_pose: the Phase 0 kinematic helpers (delay compensation, V2).
* rollout_dynamic: a batched dynamic single-track model with the same equations as the harness's
  bicycle backend (simulator/autocross/backends/bicycle.py): linear tyres (stiffness per unit load)
  saturating at mu*Fz via tanh, a friction circle per axle, static load split plus longitudinal load
  transfer, AWD drive / brake split with traction control, drive force capped by power, drag and
  rolling resistance, the actuator's steer slew-rate limit and first-order steering lag, and the
  harness's kinematic blend below 3 m/s. The stiff tyre modes (vy, yaw rate) use a linearly
  implicit (Rosenbrock-Euler) update, so the model stays stable at the 50 ms control step where
  the harness needs 2.5 ms sub-steps.
* verify_plan: rolls the car out along a Plan with the tracker's control law and reports predicted
  poses, friction use, path deviation and a per-node speed scale that keeps friction use < 0.95.

The same physics code (_step) runs on numpy arrays (batched rollouts, sampling MPC) and on plain
floats (one verification, ~10x faster than 1-element numpy arrays): `xp` selects the math backend.
"""
from __future__ import annotations

import math
from typing import Callable, NamedTuple

import numpy as np

G = 9.81

# Bicycle-backend constants that params.py does not carry yet (simulator/config/sim.yaml
# vehicle.bicycle and DEFAULTS in backends/bicycle.py). A Params field of the same name wins.
_BICYCLE_EXTRA = {"front_drive_fraction": 0.4, "front_brake_fraction": 0.6, "traction_limit": 0.97,
                  "yaw_inertia_factor": 1.0}
# Bicycle-backend defaults for the fields params.py does carry, used when p lacks them.
_BICYCLE_BASE = {"cg_height": 0.5, "cornering_stiffness_per_load": 14.0, "mu_model": 0.97,
                 "max_drive_force": 9900.0, "max_power_w": 200000.0, "max_brake_force": 17500.0,
                 "drag_area": 0.55, "rolling_resistance": 0.012, "steer_time_constant": 0.06}


def _get(p, name: str) -> float:
    v = getattr(p, name, None)
    return float(v if v is not None else {**_BICYCLE_BASE, **_BICYCLE_EXTRA}[name])


# ----------------------------------------------------------------------------- kinematic (Phase 0)

def rollout_kinematic(state: np.ndarray, steer_rad: np.ndarray, accel: np.ndarray,
                      wheelbase: float, dt: float) -> np.ndarray:
    """Roll out B candidate control sequences at once.

    state:     (4,) or (B,4) [x, y, yaw, v] in any frame
    steer_rad: (B,T) road-wheel angle per step
    accel:     (B,T) longitudinal acceleration per step
    returns    (B,T+1,4) predicted states (vectorised over the batch; the loop is over time only)
    """
    steer_rad, accel = np.atleast_2d(steer_rad), np.atleast_2d(accel)
    b, t = steer_rad.shape
    out = np.empty((b, t + 1, 4))
    out[:, 0] = np.broadcast_to(state, (b, 4))
    for k in range(t):
        x, y, yaw, v = out[:, k].T
        out[:, k + 1, 0] = x + v * np.cos(yaw) * dt
        out[:, k + 1, 1] = y + v * np.sin(yaw) * dt
        out[:, k + 1, 2] = yaw + v / wheelbase * np.tan(steer_rad[:, k]) * dt
        out[:, k + 1, 3] = np.maximum(v + accel[:, k] * dt, 0.0)
    return out


def predict_pose(speed: float, steer_rad: float, wheelbase: float, horizon_s: float) -> np.ndarray:
    """Where the car will be after horizon_s (constant speed and steer), in the current vehicle
    frame: returns [x, y, yaw]. Closed form for an arc, so it costs a few microseconds."""
    yaw = speed / wheelbase * np.tan(steer_rad) * horizon_s
    dist = speed * horizon_s
    if abs(yaw) < 1e-6:
        return np.array([dist, 0.0, 0.0])
    r = dist / yaw
    return np.array([r * np.sin(yaw), r * (1.0 - np.cos(yaw)), yaw])


# ----------------------------------------------------------------------------- dynamic model

class Model(NamedTuple):
    """Constants of the dynamic single-track model, derived once per call from (vp, p)."""
    m: float
    wheelbase: float
    lf: float
    lr: float
    iz: float
    mu: float              # tyre-road friction of the model (p.mu_model)
    c_per_load: float      # cornering stiffness per unit load, 1/rad
    f_drive: float         # N
    power: float           # W
    k_drag: float          # 0.5 * rho * Cd*A
    f_roll: float          # rolling resistance force, N
    fzf0: float            # static front axle load, N
    k_transfer: float      # front load lost per m/s^2 of forward accel: m * h_cg / L
    mg: float
    drive_f: float         # front share of drive force
    lim: float             # traction control / ABS: |Fx| <= lim * Fz
    max_steer: float       # rad at |steer| = 1
    f_brake: float         # N at brake = 1
    brake_f: float         # front share of brake force
    tau_steer: float       # s, steering lag
    steer_rate: float      # normalised units / s
    mu_score: float        # published mu (VehicleParams.mu) used by the scorer's grip metric


def model_constants(vp, p) -> Model:
    """Model constants from VehicleParams (geometry, steering) and Params (tyres, powertrain)."""
    m, L = float(vp.mass), float(vp.wheelbase)
    lf = float(vp.cg_to_front)
    lr = L - lf
    lim = _get(p, "traction_limit") * _get(p, "mu_model")
    return Model(m=m, wheelbase=L, lf=lf, lr=lr, iz=_get(p, "yaw_inertia_factor") * m * lf * lr,
                 mu=_get(p, "mu_model"), c_per_load=_get(p, "cornering_stiffness_per_load"),
                 f_drive=_get(p, "max_drive_force"), power=_get(p, "max_power_w"),
                 k_drag=0.5 * 1.2 * _get(p, "drag_area"), f_roll=_get(p, "rolling_resistance") * m * G,
                 fzf0=m * G * lr / L, k_transfer=m * _get(p, "cg_height") / L, mg=m * G,
                 drive_f=_get(p, "front_drive_fraction"), lim=lim, max_steer=float(vp.max_steer),
                 f_brake=_get(p, "max_brake_force"), brake_f=_get(p, "front_brake_fraction"),
                 tau_steer=max(_get(p, "steer_time_constant"), 1e-3),
                 steer_rate=float(getattr(vp, "max_steer_rate", 2.0)), mu_score=float(vp.mu))


def _lag_coeffs(c: Model, h: float) -> tuple:
    """Exact first-order lag over h for a target ramping linearly by `step` (normalised):
    d' = d + (u_end - d)(1 - E) + step * max_steer * (E - tau/h (1 - E)),  E = exp(-h / tau)."""
    e = math.exp(-h / c.tau_steer)
    return 1.0 - e, c.max_steer * (e - c.tau_steer / h * (1.0 - e))


class _Scalar:
    """The subset of the numpy API that _step uses, on Python floats."""
    minimum = staticmethod(min)
    maximum = staticmethod(max)
    sqrt = staticmethod(math.sqrt)
    tanh = staticmethod(math.tanh)
    arctan2 = staticmethod(math.atan2)
    cos = staticmethod(math.cos)
    sin = staticmethod(math.sin)
    tan = staticmethod(math.tan)

    @staticmethod
    def where(c, a, b):
        return a if c else b

    @staticmethod
    def amin(a):
        return a


class _Numpy:
    """numpy namespace for _step (amin via the array method, which is faster than np.min)."""
    minimum, maximum, sqrt, tanh, arctan2 = np.minimum, np.maximum, np.sqrt, np.tanh, np.arctan2
    cos, sin, tan, where = np.cos, np.sin, np.tan, np.where

    @staticmethod
    def amin(a):
        return a.min()


def _step(xp, s, steer_cmd, thr, brk, c: Model, h: float, lag: tuple, want_use: bool):
    """Advance s = (x, y, yaw, vx, vy, r, delta, steer_applied, ax_prev) by h seconds under the
    normalised steer command and pedals (thr, brk in [0,1]); lag = _lag_coeffs(c, h).
    Mirrors bicycle.substep();
    returns (new state tuple, per-axle max tyre utilisation or None)."""
    x, y, yaw, vx, vy, r, d, app, ax_prev = s
    m, lf, lr = c.m, c.lf, c.lr
    # actuator: slew-rate limit on the normalised command, then first-order steering lag, solved
    # exactly for the target ramping from the old to the new applied steer over the step
    rate = c.steer_rate * h
    step = xp.minimum(xp.maximum(steer_cmd - app, -rate), rate)
    app = app + step
    d = d + (app * c.max_steer - d) * lag[0] + step * lag[1]
    # longitudinal forces: drive (force and power limited), brake, drag + rolling resistance
    vxs = xp.maximum(vx, 1.0)
    f_trac = thr * xp.minimum(c.f_drive, c.power / vxs)
    f_brk = brk * c.f_brake
    f_res = c.k_drag * vx * abs(vx) + c.f_roll * (vx > 0.05)
    # axle loads with longitudinal load transfer from the previous step's accel
    fzf = c.fzf0 - c.k_transfer * ax_prev
    fzr = c.mg - fzf
    limf, limr = c.lim * fzf, c.lim * fzr
    fxf = xp.minimum(xp.maximum(c.drive_f * f_trac - c.brake_f * f_brk, -limf), limf)
    fxr = xp.minimum(xp.maximum((1.0 - c.drive_f) * f_trac - (1.0 - c.brake_f) * f_brk, -limr), limr)
    # lateral tyre forces: linear in slip angle, saturating at what the friction circle leaves
    af = xp.arctan2(vy + lf * r, vxs) - d
    ar = xp.arctan2(vy - lr * r, vxs)
    muf, mur = c.mu * fzf, c.mu * fzr
    availf = xp.sqrt(xp.maximum(muf * muf - fxf * fxf, 0.0))
    availr = xp.sqrt(xp.maximum(mur * mur - fxr * fxr, 0.0))
    cf, cr = c.c_per_load * fzf, c.c_per_load * fzr
    thf = xp.tanh(cf * af / xp.maximum(availf, 1.0))
    thr_ = xp.tanh(cr * ar / xp.maximum(availr, 1.0))
    fyf, fyr = -availf * thf, -availr * thr_
    cd, sd = xp.cos(d), xp.sin(d)
    fyf_b = fyf * cd + fxf * sd                       # front axle force, body-lateral
    vx_dot = (fxr + fxf * cd - fyf * sd - f_res) / m + r * vy
    vy_dot = (fyr + fyf_b) / m - r * vx
    r_dot = (lf * fyf_b - lr * fyr) / c.iz
    # linearly implicit Euler on (vy, r): (I - hJ) dz = h f with the tyres' local stiffness
    kf = cf * (1.0 - thf * thf) / vxs                 # -dFyf/dvy
    kr = cr * (1.0 - thr_ * thr_) / vxs
    q = lr * kr - lf * kf
    a11 = 1.0 + h * (kf + kr) / m
    a22 = 1.0 + h * (lf * lf * kf + lr * lr * kr) / c.iz
    a12 = h * (vx - q / m)
    a21 = -h * q / c.iz
    det = xp.maximum(a11 * a22 - a12 * a21, 0.05)
    b1, b2 = h * vy_dot, h * r_dot
    vx_d = vx + vx_dot * h
    vy_d = vy + (a22 * b1 - a12 * b2) / det
    r_d = r + (a11 * b2 - a21 * b1) / det
    if xp.amin(vx_d) >= 3.0 and xp.amin(vx) > 1.0:
        vx_n, vy_n, r_n = vx_d, vy_d, r_d
    else:
        # low speed: the harness's kinematic model, blended in over 1..3 m/s
        dyn = vx > 1.0
        vx_d = xp.where(dyn, vx_d, vx)
        vy_d = xp.where(dyn, vy_d, vy)
        r_d = xp.where(dyn, r_d, r)
        w = xp.minimum(xp.maximum((3.0 - vx_d) * 0.5, 0.0), 1.0)
        vk = xp.where(vx_d <= 1.0, vx_d + (fxr + fxf - f_res) / m * h, vx_d)
        vk = xp.where((vk < 0.0) & (thr <= 0.0), 0.0, vk)
        rk = vk * xp.tan(d) / c.wheelbase
        vx_n = w * vk + (1.0 - w) * vx_d
        vy_n = w * rk * lr + (1.0 - w) * vy_d
        r_n = w * rk + (1.0 - w) * r_d
    vx_n = xp.maximum(vx_n, 0.0)
    cy, sy = xp.cos(yaw), xp.sin(yaw)
    x = x + (vx_n * cy - vy_n * sy) * h
    y = y + (vx_n * sy + vy_n * cy) * h
    yaw = yaw + r_n * h
    ax = (vx_n - vx) / h - r_n * vy_n
    use = None
    if want_use:  # tyre utilisation |F| / (mu Fz) of the busier axle: 1 = sliding
        use = xp.maximum(xp.sqrt(fxf * fxf + fyf * fyf) / xp.maximum(muf, 1.0),
                         xp.sqrt(fxr * fxr + fyr * fyr) / xp.maximum(mur, 1.0))
    return (x, y, yaw, vx_n, vy_n, r_n, d, app, ax), use


def rollout_dynamic(state: np.ndarray, steer_cmd: np.ndarray, accel_cmd: np.ndarray, vp, p,
                    dt: float = 0.05, *, steer_applied=None, ax0=0.0, substeps: int = 1,
                    info: dict | None = None) -> np.ndarray:
    """Roll out B control sequences through the dynamic bicycle model at once.

    state:         (7,) or (B,7) [x, y, yaw, vx, vy, yaw_rate, steer_actual] in any frame
                   (vx, vy body frame; steer_actual = road-wheel angle in rad, after the lag)
    steer_cmd:     (B,T) or (T,) normalised steer command in [-1, 1] (what Command.steer would be)
    accel_cmd:     (B,T) or (T,) signed pedal in [-1, 1]: > 0 throttle, < 0 brake (never both;
                   pedal_for_accel() converts a desired acceleration into this)
    steer_applied: normalised steer after the slew limiter (obs.steer); default steer_actual/max_steer
    ax0:           longitudinal accel of the previous tick (obs.ax), for load transfer
    substeps:      integration sub-steps per dt (1 is stable; 2 is closer to the harness)
    info:          if a dict is given it is filled with (B,T) arrays 'friction_use' (per-axle tyre
                   utilisation |F|/(mu Fz), max over axles), 'ax', 'ay' (body-frame accel, m/s^2)
    returns        (B,T+1,7) states; the loop is over time only
    """
    c = model_constants(vp, p)
    steer_cmd, accel_cmd = np.atleast_2d(np.asarray(steer_cmd, float)), np.atleast_2d(np.asarray(accel_cmd, float))
    b, t = np.broadcast_shapes(steer_cmd.shape, accel_cmd.shape)
    steer_cmd = np.clip(np.nan_to_num(np.broadcast_to(steer_cmd, (b, t))), -1.0, 1.0)
    accel_cmd = np.clip(np.nan_to_num(np.broadcast_to(accel_cmd, (b, t))), -1.0, 1.0)
    sc_all = steer_cmd.T.copy()                                  # (T,B): contiguous per step
    thr_all, brk_all = np.maximum(accel_cmd, 0.0).T.copy(), np.maximum(-accel_cmd, 0.0).T.copy()
    s0 = np.broadcast_to(np.nan_to_num(np.asarray(state, float)), (b, 7))
    n = max(int(substeps), 1)
    h = dt / n
    lag = _lag_coeffs(c, h)
    app0 = s0[:, 6] / c.max_steer if steer_applied is None else np.broadcast_to(steer_applied, (b,))
    s = (*s0.T.copy(), np.array(app0, float), np.array(np.broadcast_to(np.asarray(ax0, float), (b,))))
    states = [s[:7]]
    want = info is not None
    uses, axs, ays = [], [], []
    for k in range(t):
        sc, th, br = sc_all[k], thr_all[k], brk_all[k]
        vx0, vy0 = s[3], s[4]
        use = None
        for _ in range(n):
            s, u = _step(_Numpy, s, sc, th, br, c, h, lag, want)
            use = u if use is None else np.maximum(use, u)
        states.append(s[:7])
        if want:
            uses.append(use)
            axs.append((s[3] - vx0) / dt - s[5] * s[4])
            ays.append((s[4] - vy0) / dt + s[5] * s[3])
    out = np.array(states).transpose(2, 0, 1)                   # (T+1,7,B) -> (B,T+1,7)
    if want:
        info.update(friction_use=np.array(uses).T, ax=np.array(axs).T, ay=np.array(ays).T)
    return out


def pedal_for_accel(a_des, vx, vp, p):
    """Signed pedal in [-1, 1] that gives longitudinal accel a_des (m/s^2) at speed vx on a straight,
    by inverting the model's force balance (drive force/power limit, brake force, drag, rolling)."""
    c = model_constants(vp, p)
    vx = np.maximum(np.asarray(vx, float), 0.0)
    f_need = c.m * np.asarray(a_des, float) + c.k_drag * vx * vx + c.f_roll * (vx > 0.05)
    f_drive = np.minimum(c.f_drive, c.power / np.maximum(vx, 1.0))
    return np.clip(np.where(f_need >= 0.0, f_need / f_drive, f_need / c.f_brake), -1.0, 1.0)


# ----------------------------------------------------------------------------- plan verification

def verify_plan(plan, obs, vp, p, track_fn: Callable | None = None, horizon_s: float = 1.5,
                dt: float = 0.05, corridor=None, use_limit: float = 0.95) -> dict:
    """Simulate the car following `plan` (vehicle frame of `obs`) for horizon_s seconds.

    The default control law is the tracker's: pure pursuit at the speed-scaled lookahead, the
    ramped speed target with P control and the friction-circle throttle cap. track_fn, if given, is
    called as track_fn((x, y, yaw, vx, vy, yaw_rate, steer_actual), plan, k) -> (steer, pedal).

    Returns a dict with
      poses (T+1,2), yaw/speed/yaw_rate (T+1,), lat_dev (T+1,) signed offset from plan.path (+ left),
      friction_use (T,) tyre utilisation |F|/(mu Fz) of the busier axle, grip_demand (T,) the scorer's v^2 tan(steer)/(L mu g),
      friction_use_max, grip_demand_max, lat_dev_max (floats),
      v_scale (N,) per-node factor (<= 1) on plan.v that brings both demands under use_limit,
      corridor_violation_max (m outside corridor.left/right; NaN without a corridor), ok (bool).
    """
    c = model_constants(vp, p)
    path = np.nan_to_num(np.asarray(plan.path, float))
    n_nodes = len(path)
    t_steps = max(int(round(horizon_s / dt)), 1)
    if n_nodes < 2:
        return {"poses": np.zeros((1, 2)), "friction_use_max": 0.0, "grip_demand_max": 0.0, "lat_dev_max": 0.0,
                "v_scale": np.ones(n_nodes), "corridor_violation_max": float("nan"), "ok": False}
    seg = np.diff(path, axis=0)
    seg_len = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-6)
    ux_l, uy_l = (seg[:, 0] / seg_len).tolist(), (seg[:, 1] / seg_len).tolist()
    px_l, py_l, sl_l = path[:, 0].tolist(), path[:, 1].tolist(), seg_len.tolist()
    s_l = np.concatenate(([0.0], np.cumsum(seg_len))).tolist()
    v_l = np.nan_to_num(np.asarray(plan.v, float)).tolist()
    last = n_nodes - 2

    # tracker gains (params.py names), defaults = Phase 0 tracker
    ld_min, ld_gain, ld_off = (float(getattr(p, k, d)) for k, d in
                               (("lookahead_min", 4.0), ("lookahead_gain", 0.5), ("lookahead_offset", 2.0)))
    kp, brake_gain = float(getattr(p, "kp", 0.5)), float(getattr(p, "brake_gain", 0.6))
    min_cap = float(getattr(p, "min_throttle_cap", 0.15))
    a_acc = float(getattr(p, "a_acc", 0.0)) or 0.5 * float(getattr(vp, "max_accel", 5.2))
    L, max_steer, mu_g = c.wheelbase, c.max_steer, c.mu_score * G

    lag = _lag_coeffs(c, dt)
    app = float(np.nan_to_num(obs.steer))
    s = (0.0, 0.0, 0.0, max(float(np.nan_to_num(obs.vx)), 0.0), float(np.nan_to_num(obs.vy)),
         float(np.nan_to_num(obs.yaw_rate)), app * max_steer, app, float(np.nan_to_num(obs.ax)))
    ay = float(np.nan_to_num(obs.ay))
    v_ref = max(float(np.nan_to_num(obs.speed)), 0.0)
    i = 0
    xs, ys, yaws, spd, rates, devs, nodes = [], [], [], [], [], [], []
    uses, demands = [], []
    for k in range(t_steps + 1):
        x, y, yaw, vx, vy, r = s[0], s[1], s[2], s[3], s[4], s[5]
        # project the car onto the path (monotonic walk: the car only moves forward along it)
        while True:
            along = (x - px_l[i]) * ux_l[i] + (y - py_l[i]) * uy_l[i]
            if along <= sl_l[i] or i == last:
                break
            i += 1
        along_c = min(max(along, 0.0), sl_l[i])
        xs.append(x), ys.append(y), yaws.append(yaw), rates.append(r)
        speed = math.hypot(vx, vy)
        spd.append(speed)
        devs.append(ux_l[i] * (y - py_l[i]) - uy_l[i] * (x - px_l[i]))
        nodes.append(i if along_c < 0.5 * sl_l[i] else i + 1)
        if k == t_steps:
            break
        s_car = s_l[i] + along_c
        if track_fn is not None:
            steer, pedal = track_fn(tuple(s[:7]), plan, k)
            steer, pedal = min(max(float(steer), -1.0), 1.0), min(max(float(pedal), -1.0), 1.0)
        else:
            # pure pursuit to the path point ld ahead of the car (clamped to the path end like np.interp)
            s_t = s_car + max(ld_min, ld_gain * speed + ld_off)
            j = i
            while j < last and s_l[j + 1] < s_t:
                j += 1
            f = min(max((s_t - s_l[j]) / sl_l[j], 0.0), 1.0)
            dx = px_l[j] + f * (px_l[j + 1] - px_l[j]) - x
            dy = py_l[j] + f * (py_l[j + 1] - py_l[j]) - y
            cy, sy = math.cos(yaw), math.sin(yaw)
            lx, ly = cy * dx + sy * dy, cy * dy - sy * dx
            alpha = math.atan2(ly, max(lx, 0.1))
            delta = math.atan2(2.0 * L * math.sin(alpha), max(math.hypot(lx, ly), 1.0))
            steer = min(max(delta / max_steer, -1.0), 1.0)
            # speed: ramped target, P control, friction-circle throttle cap (tracker.py)
            fv = along_c / sl_l[i]
            v_ref = min(v_l[i] + fv * (v_l[i + 1] - v_l[i]), v_ref + a_acc * dt)
            u = kp * (v_ref - speed)
            lat_use = min(abs(ay) / mu_g, 1.0)
            pedal = min(u, max(1.0 - lat_use * lat_use, min_cap)) if u >= 0.0 else -min(-u * brake_gain, 1.0)
        thr, brk = (pedal, 0.0) if pedal >= 0.0 else (0.0, -pedal)
        demands.append(speed * speed * abs(math.tan(steer * max_steer)) / L / mu_g)
        s_new, use = _step(_Scalar, s, steer, thr, brk, c, dt, lag, True)
        ay = (s_new[4] - vy) / dt + s_new[5] * s_new[3]
        uses.append(use)
        s = s_new

    poses = np.column_stack((xs, ys))
    use_t, dem_t = np.array(uses), np.array(demands)
    # where a demand is over the limit, the plan speed at the car's node should be the predicted
    # speed scaled by sqrt(limit / demand) (lateral demand ~ v^2)
    worst = np.maximum(use_t, dem_t)
    node_t = np.array(nodes[:-1])
    v_ok = np.array(spd[:-1]) * np.sqrt(use_limit / np.maximum(worst, 1e-9))
    scale_t = np.where(worst > use_limit, v_ok / np.maximum(np.asarray(v_l)[node_t], 0.1), 1.0)
    v_scale = np.ones(n_nodes)
    np.minimum.at(v_scale, node_t, np.clip(scale_t, 0.0, 1.0))
    v_scale[:-1] = np.minimum(v_scale[:-1], v_scale[1:])   # start slowing one node early
    dev = np.array(devs)
    viol = float("nan")
    if corridor is not None and getattr(corridor, "valid", True) and len(corridor.center) > 1:
        d2 = ((poses[:, None, :] - corridor.center[None, :, :]) ** 2).sum(axis=2)
        jn = d2.argmin(axis=1)
        off = ((poses - corridor.center[jn]) * corridor.normal[jn]).sum(axis=1)
        viol = float(np.max(np.maximum(off - corridor.left[jn], -off - corridor.right[jn])))
    f_max, g_max = float(use_t.max()), float(dem_t.max())
    return {"poses": poses, "yaw": np.array(yaws), "speed": np.array(spd), "yaw_rate": np.array(rates),
            "lat_dev": dev, "friction_use": use_t, "grip_demand": dem_t,
            "friction_use_max": f_max, "grip_demand_max": g_max, "lat_dev_max": float(np.abs(dev).max()),
            "v_scale": v_scale, "corridor_violation_max": viol,
            "ok": bool(f_max <= use_limit and g_max <= use_limit and not viol > 0.0)}
