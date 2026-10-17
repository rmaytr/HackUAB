"""Physics-based prediction. Owner: Rodion.

Three tools, all pure functions (with bounded memoization), numpy + stdlib only:

* rollout_kinematic / predict_pose: the Phase 0 kinematic helpers (delay compensation, V2).
* rollout_dynamic: a batched dynamic single-track model with the same equations as the harness's
  bicycle backend (simulator/autocross/backends/bicycle.py): linear tyres (stiffness per unit load)
  saturating at mu*Fz via tanh, a friction circle per axle, static load split plus longitudinal load
  transfer, AWD drive / brake split with traction control, drive force capped by power, drag and
  rolling resistance, the actuator's steer slew-rate limit and first-order steering lag, and the
  harness's kinematic blend below 3 m/s. The stiff tyre modes (vy, yaw rate) use a linearly
  implicit (Rosenbrock-Euler) update, so the model stays stable at the 50 ms control step where
  the harness needs 2.5 ms sub-steps.
* verify_plan: rolls the car out along a Plan with a scalar version of tracker.py's control law and
  reports predicted poses, friction use, grip demand, rear slip, path deviation and a per-node
  speed scale for grip/slip and buffered corridor breaches. planner.py applies caps and
  re-verifies; a proposed speed scale alone is not a safety certificate.

The same physics code (_step) runs on numpy arrays (batched rollouts, sampling MPC) and on plain
floats (one verification, ~10x faster than 1-element numpy arrays): `xp` selects the math backend.
"""
from __future__ import annotations

import math
from bisect import bisect_right
from functools import lru_cache
from typing import Callable, NamedTuple
from types import SimpleNamespace

import numpy as np

from tracker import _finite, _p, _tangent, _update_slip, lon_envelope

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
    state = np.atleast_2d(np.nan_to_num(np.asarray(state, float)))
    b, t = np.broadcast_shapes(steer_cmd.shape, accel_cmd.shape, (state.shape[0], 1))
    steer_cmd = np.clip(np.nan_to_num(np.broadcast_to(steer_cmd, (b, t))), -1.0, 1.0)
    accel_cmd = np.clip(np.nan_to_num(np.broadcast_to(accel_cmd, (b, t))), -1.0, 1.0)
    sc_all = steer_cmd.T.copy()                                  # (T,B): contiguous per step
    thr_all, brk_all = np.maximum(accel_cmd, 0.0).T.copy(), np.maximum(-accel_cmd, 0.0).T.copy()
    s0 = np.broadcast_to(state, (b, 7))
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

class _GripVehicle(NamedTuple):
    mass: float
    wheelbase: float
    cg_to_front: float
    mu: float
    max_accel: float
    max_decel: float


class _GripParams(NamedTuple):
    cg_height: float
    grip_frac: float
    traction_limit: float
    front_drive_fraction: float
    front_brake_fraction: float


def _grip_constants(vp, p) -> tuple[_GripVehicle, _GripParams]:
    """Immutable keys containing every input read by tracker.lon_envelope()."""
    return (_GripVehicle(float(vp.mass), float(vp.wheelbase), float(vp.cg_to_front),
                         float(vp.mu), float(vp.max_accel), float(vp.max_decel)),
            _GripParams(*(_p(p, name) for name in _GripParams._fields)))


@lru_cache(maxsize=2048)
def _grip_grid(vp: _GripVehicle, p: _GripParams, a_lat: float, scale: float) -> tuple[float, float]:
    return lon_envelope(a_lat, vp, p, scale)


def _grip_bound(vp: _GripVehicle, p: _GripParams, a_lat: float, scale: float, down: bool) -> float:
    """Exact shared-envelope answer, memoizing cells wholly inside one grid plateau.

    Capacity decreases with lateral demand and increases with grip. If both extreme corners
    of a small (a_lat, scale) cell give the same discrete capacity, every point inside gives
    that value. At any boundary we evaluate the original helper at the actual input. There
    is no interpolation, rounding of the returned capacity, or second copy of the AWD model.
    Binary cell sizes keep endpoint arithmetic exact for ordinary finite inputs.
    """
    if not (math.isfinite(a_lat) and math.isfinite(scale)
            and 0.0 <= a_lat < 1e6 and 0.0 <= scale < 1e6):
        return lon_envelope(a_lat, vp, p, scale)[down]
    low_lat = math.floor(a_lat * 32.0) / 32.0
    low_mu = math.floor(scale * 256.0) / 256.0
    high = _grip_grid(vp, p, low_lat, low_mu + 1.0 / 256.0)[down]
    low = _grip_grid(vp, p, low_lat + 1.0 / 32.0, low_mu)[down]
    return low if low == high else lon_envelope(a_lat, vp, p, scale)[down]


def corridor_clearance(poses: np.ndarray, corridor) -> dict:
    """Project reference points into the buffered, piecewise-linear corridor.

    Interpolate BOTH the center and its normal/bounds, not the nearest node's band. Three
    bounded Newton steps invert the ruled strip C(u) + d*N(u); this avoids the tangential
    station error of nearest-node/segment-normal projection on curved, asymmetric bands.
    Width/obstacle/vehicle buffers already belong to horizon.py: do not subtract them again.
    This checks reference-point clearance, not a new yaw-dependent vehicle footprint model.
    """
    poses = np.asarray(poses, float)
    count = len(poses)
    bad = {"corridor_station": np.zeros(count), "corridor_offset": np.zeros(count),
           "corridor_violation": np.full(count, 1e3), "corridor_covered": np.zeros(count, bool),
           "corridor_signed_violation": np.full(count, 1e3), "corridor_valid": False}
    if not getattr(corridor, "valid", False):
        return bad
    center, normal = np.asarray(corridor.center, float), np.asarray(corridor.normal, float)
    station = np.asarray(corridor.s, float)
    left, right = np.asarray(corridor.left, float), np.asarray(corridor.right, float)
    n = len(station)
    if (n < 2 or center.shape != (n, 2) or normal.shape != (n, 2)
            or left.shape != (n,) or right.shape != (n,)
            or not all(np.isfinite(a).all() for a in (poses, center, normal, station, left, right))
            or np.any(np.diff(station) <= 0) or np.any(left + right < 0)):
        return bad
    seg = np.diff(center, axis=0)
    seg2 = np.sum(seg * seg, axis=1)
    if np.any(seg2 < 1e-12) or np.any(np.linalg.norm(normal, axis=1) < 0.5):
        return bad
    normal = normal / np.linalg.norm(normal, axis=1, keepdims=True)
    if np.any(np.sum(normal[:-1] * normal[1:], axis=1) <= 0.0):
        return bad  # folded/degenerate cross-sections are not certifiable free space
    delta = poses[:, None, :] - center[None, :-1, :]
    dn = np.diff(normal, axis=0)

    def cross(a, b):
        return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]

    u = np.clip(np.sum(delta * seg, axis=2) / seg2, 0.0, 1.0)
    a = cross(seg, dn)
    b = cross(delta, dn) - cross(seg, normal[:-1])
    c0 = cross(delta, normal[:-1])
    for _ in range(3):  # fixed iterations, vectorised over poses AND spatial segments
        deriv = b - 2.0 * a * u
        step = np.divide(c0 + b * u - a * u * u, deriv,
                         out=np.zeros_like(u), where=np.abs(deriv) > 1e-9)
        u = np.clip(u - step, 0.0, 1.0)
    residual = delta - u[..., None] * seg
    # A vertex can be Euclidean-nearest while its normal does not pass through the point.
    # Prefer a genuine cross-section inverse on either adjacent segment; otherwise every
    # curved vertex creates a false "unobserved" wedge for nonzero lateral offsets.
    normals = normal[:-1] + u[..., None] * dn
    normal_lengths = np.linalg.norm(normals, axis=2)
    ray_error = np.abs(cross(residual, normals)) / np.maximum(normal_lengths, 1e-9)
    inverse = (ray_error <= 1e-5) & (normal_lengths > 1e-9)
    distance = np.sum(residual * residual, axis=2)
    start_along = cross(residual[:, 0], normals[:, 0]) / np.maximum(normal_lengths[:, 0], 1e-9)
    inverse[:, 0] |= ((u[:, 0] <= 1e-9) & (start_along <= 0.0) &
                      (-start_along <= min(0.5 * float(np.sqrt(seg2[0])), 0.5)))
    # A remote normal ray across a hairpin may also intersect this point. It must not
    # displace the local projection just because a clamped endpoint has a tiny residual.
    inverse &= distance <= distance.min(axis=1)[:, None] + 0.25
    distance = np.where(inverse.any(axis=1)[:, None], np.where(inverse, distance, np.inf), distance)
    j = np.argmin(distance, axis=1)
    rows = np.arange(count)
    f = u[rows, j]
    ref = center[j] + f[:, None] * seg[j]
    norm = normal[j] + f[:, None] * dn[j]
    norm /= np.linalg.norm(norm, axis=1, keepdims=True)
    diff = poses - ref
    off = np.sum(diff * norm, axis=1)
    along_error = cross(diff, norm)
    tangent_error = np.abs(along_error)
    # No extrapolated end band is certified as observed free space.
    # The first normal is a forward-difference tangent in horizon.py. On a curved road
    # an offset car at station zero can lie just behind that discretised cross-section.
    # Use the SAME buffered lateral band across its half-cell; never extrapolate the
    # far preview end. This is not an extra lateral tolerance or a rejoin exemption.
    start_cell = (j == 0) & (f <= 1e-9) & (along_error <= 0.0) & (
        tangent_error <= min(0.5 * float(np.sqrt(seg2[0])), 0.5))
    covered = (tangent_error <= 1e-5) | start_cell
    hi = left[j] + f * (left[j + 1] - left[j])
    lo = -(right[j] + f * (right[j + 1] - right[j]))
    signed_violation = np.maximum(off - hi, lo - off)
    violation = np.maximum(signed_violation, 0.0)
    # Unobserved space is a failed clearance check, not a silent pass at the last node.
    violation = np.where(covered, violation, np.maximum(violation, tangent_error))
    signed_violation = np.where(covered, signed_violation, violation)
    return {"corridor_station": station[j] + f * (station[j + 1] - station[j]),
            "corridor_offset": off, "corridor_violation": violation,
            "corridor_signed_violation": signed_violation,
            "corridor_covered": covered, "corridor_valid": True}


def verify_plan(plan, obs, vp, p, track_fn: Callable | None = None, horizon_s: float = 1.5,
                dt: float = 0.05, corridor=None, use_limit: float = 0.95, grip_limit: float = 1.0,
                rear_slip_limit_deg: float = 5.0) -> dict:
    """Simulate the car following `plan` (vehicle frame of `obs`) for horizon_s seconds.

    The default control law mirrors tracker.track() (same Params fields): latency-compensated pose,
    curvature feedforward plus pure-pursuit error feedback, grip cap and steer slew limit, speed
    feedforward + PI, AWD per-axle friction limits, jerk limit, acceleration -> pedal mapping.
    Grip limits, slip adaptation, defaults and tangents use the tracker's own helpers. The scalar
    geometry/PI avoids calling the full numpy tracker 30 times per plan. Its integrator starts at
    0 and its initial grip scale comes from p.a_lat (the planner cannot see the tracker's remaining
    state). track_fn, if given, replaces it:
    track_fn((x, y, yaw, vx, vy, yaw_rate, steer_actual), plan, k) -> (steer, pedal).

    Returns a dict with
      poses (T+1,2), yaw/speed/yaw_rate/rear_slip_deg (T+1,), lat_dev (T+1,) signed offset from
      plan.path (+ left), friction_use (T,) tyre utilisation |F|/(mu Fz) of the busier axle,
      grip_demand (T,) the scorer's v^2 tan(steer_cmd)/(L mu g),
      friction_use_max, grip_demand_max, rear_slip_max_deg, lat_dev_max (floats),
      v_scale (N,) per-node factor (<= 1) on plan.v that brings friction use under use_limit, grip
      demand under grip_limit (the tracker's grip cap allows 1.05, the scorer flags > 1.1) and rear
      slip under rear_slip_limit_deg (the scorer flags > 6 deg),
      demand_max (the worst of the three, normalised by its limit: > 1 = over),
      corridor_violation (T+1,) nonnegative metres outside the buffered corridor, corridor_station,
      corridor_offset, corridor_covered, corridor_ok, corridor_first_step/time/s/path_s (-1 if none).
      Unobserved end-of-preview space and invalid corridors fail closed. No second width buffer.
      v_scale and demand_max include clearance failures as well as grip/slip; ok requires both.
      corridor_violation_max retains signed-margin compatibility (negative = inside everywhere);
      corridor_violation is nonnegative. The first half-cell covers the current offset car pose.
    """
    c = model_constants(vp, p)
    grip_vp, grip_p = _grip_constants(vp, p)
    path = np.nan_to_num(np.asarray(plan.path, float))
    n_nodes = len(path)
    t_steps = max(int(round(horizon_s / dt)), 1)
    if n_nodes < 2:
        return {"poses": np.zeros((1, 2)), "friction_use_max": 0.0, "grip_demand_max": 0.0,
                "rear_slip_max_deg": 0.0, "lat_dev_max": 0.0, "v_scale": np.zeros(n_nodes),
                "demand_max": 1e3, "corridor_ok": False, "corridor_valid": False,
                "corridor_violation": np.full(1, 1e3), "corridor_covered": np.zeros(1, bool),
                "corridor_station": np.zeros(1), "corridor_offset": np.zeros(1),
                "corridor_first_step": 0, "corridor_first_time": 0.0, "corridor_first_s": 0.0,
                "corridor_first_path_s": 0.0, "corridor_violation_max": 1e3, "ok": False}
    seg = np.diff(path, axis=0)
    seg_len = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-6)
    ux_l, uy_l = (seg[:, 0] / seg_len).tolist(), (seg[:, 1] / seg_len).tolist()
    px_l, py_l, sl_l = path[:, 0].tolist(), path[:, 1].tolist(), seg_len.tolist()
    stations = np.asarray(plan.s, float)
    speeds = np.nan_to_num(np.asarray(plan.v, float))
    kappas = np.nan_to_num(np.asarray(plan.kappa, float))
    s_l, v_l, k_l = stations.tolist(), speeds.tolist(), kappas.tolist()
    path_x, path_y = path[:, 0].copy(), path[:, 1].copy()
    last, s_end = n_nodes - 2, s_l[-1]
    L, max_steer, mu_g = c.wheelbase, c.max_steer, c.mu_score * G

    def g(name):
        return _p(p, name)

    # tracker.py gains (Params defaults if absent)
    ld_min, ld_gain, ld_max = g("pp_lookahead_min"), g("pp_lookahead_gain"), g("pp_lookahead_max")
    ff_t, fb_gain, grip_cap = g("ff_preview_t"), g("fb_gain"), g("grip_cap")
    max_step = g("steer_rate_margin") * c.steer_rate * vp.control_dt
    spd_t, kp, ki, i_lim = g("speed_preview_t"), g("kp_speed"), g("ki_speed"), g("i_limit")
    overspeed_band = g("overspeed_band")
    coast, acc_floor, brk_floor = g("coast_band"), g("accel_floor"), g("brake_floor")
    jerk = g("jerk_max") * vp.control_dt
    max_acc, max_dec = float(vp.max_accel), float(vp.max_decel)
    # grip estimate the controller applied to p.a_lat (= a_lat_base * a_lat_scale)
    base = g("lat_accel_frac") * mu_g
    scale = min(max(g("a_lat") / base, g("slip_scale_min")), 1.0) if base > 0.0 else 1.0
    slip_state = {"a_lat_scale": scale}
    predicted_obs = SimpleNamespace()
    t_pred = vp.control_dt * (1 + int(getattr(vp, "actuation_delay_steps", 0) or 0))

    def interp(values, station):
        """Scalar np.interp equivalent without per-call numpy conversion/dispatch."""
        j = min(max(bisect_right(s_l, station) - 1, 0), last)
        f = min(max((station - s_l[j]) / (s_l[j + 1] - s_l[j]), 0.0), 1.0)
        return values[j] + f * (values[j + 1] - values[j])

    def project(x, y, j):
        """Monotonic projection onto the path: (segment, distance along it clamped to the segment)."""
        while True:
            along = (x - px_l[j]) * ux_l[j] + (y - py_l[j]) * uy_l[j]
            if along <= sl_l[j] or j == last:
                return j, min(max(along, 0.0), sl_l[j])
            j += 1

    lag = _lag_coeffs(c, dt)
    app = _finite(obs.steer)
    s = (0.0, 0.0, 0.0, max(_finite(obs.vx), 0.0), _finite(obs.vy),
         _finite(obs.yaw_rate), app * max_steer, app, _finite(obs.ax))
    steer_prev, i_err, a_prev = app, 0.0, None
    i = 0
    xs, ys, yaws, spd, rates, devs, nodes, rears, path_stations = [], [], [], [], [], [], [], [], []
    uses, demands = [], []
    for k in range(t_steps + 1):
        x, y, yaw, vx, vy, r = s[0], s[1], s[2], s[3], s[4], s[5]
        i, along_c = project(x, y, i)
        path_stations.append(s_l[i] + along_c / sl_l[i] * (s_l[i + 1] - s_l[i]))
        xs.append(x), ys.append(y), yaws.append(yaw), rates.append(r)
        speed = math.hypot(vx, vy)
        spd.append(speed)
        rears.append(math.degrees(math.atan2(vy - c.lr * r, max(abs(vx), 0.1))))
        devs.append(ux_l[i] * (y - py_l[i]) - uy_l[i] * (x - px_l[i]))
        nodes.append(i if along_c < 0.5 * sl_l[i] else i + 1)
        if k == t_steps:
            break
        if track_fn is not None:
            steer, pedal = track_fn(tuple(s[:7]), plan, k)
            steer, pedal = min(max(float(steer), -1.0), 1.0), min(max(float(pedal), -1.0), 1.0)
        else:
            predicted_obs.vx, predicted_obs.vy = vx, vy
            predicted_obs.yaw_rate, predicted_obs.steer = r, s[7]
            scale = _update_slip(predicted_obs, vp, p, slip_state, speed)
            # latency compensation: track from where the car is when the command lands
            half, d = 0.5 * r * t_pred, speed * t_pred
            xq, yq, yawq = x + d * math.cos(yaw + half), y + d * math.sin(yaw + half), yaw + r * t_pred
            # Use the tracker's nearest-node rule, including on nonuniform/curved paths.
            m = int(np.argmin((path_x - xq) ** 2 + (path_y - yq) ** 2))
            sq, nx, ny = s_l[m], px_l[m], py_l[m]
            # pure-pursuit target ld ahead along the path (extended past its end)
            s_t = sq + min(max(ld_min + ld_gain * speed, ld_min), ld_max)
            if s_t <= s_end:
                tx, ty = interp(px_l, s_t), interp(py_l, s_t)
            else:
                tx, ty = px_l[-1] + ux_l[last] * (s_t - s_end), py_l[-1] + uy_l[last] * (s_t - s_end)
            cq, sn = math.cos(yawq), math.sin(yawq)
            dx, dy = tx - xq, ty - yq
            lx, ly = cq * dx + sn * dy, cq * dy - sn * dx
            kappa_pp = 2.0 * ly / max(lx * lx + ly * ly, 1.0)
            rx, ry = tx - nx, ty - ny     # arc a car sitting on the path at the projection would need
            hx, hy = _tangent(px_l, py_l, m)
            kappa_ideal = 2.0 * (hx * ry - hy * rx) / max(rx * rx + ry * ry, 1.0)
            kappa_ff = interp(k_l, sq + speed * ff_t)
            kappa_cmd = kappa_ff + fb_gain * (kappa_pp - kappa_ideal)
            tan_cap = grip_cap * mu_g * L / max(speed * speed, 1e-3)
            steer_raw = math.atan(min(max(L * kappa_cmd, -tan_cap), tan_cap)) / max_steer
            steer = min(max(min(max(steer_raw, steer_prev - max_step), steer_prev + max_step), -1.0), 1.0)
            steer_prev = steer
            # Speed: same PI and pedal mapping, with the shared AWD friction envelope.
            v0 = v_l[m]
            s1 = sq + max(speed * spd_t, 1.0)
            v1 = interp(v_l, s1)
            a_ff = (v1 * v1 - v0 * v0) / (2.0 * (s1 - sq))
            err = v0 - speed
            if err > 0.0:
                a_ff = max(a_ff, 0.0)
            i_err = min(max(i_err + ki * err * vp.control_dt, -i_lim), i_lim)
            i_term = min(i_err, 0.0) if err < -overspeed_band else i_err
            a_cmd = a_ff + kp * err + i_term
            a_lat_now = speed * speed * abs(math.tan(steer * max_steer)) / L
            a_lat = max(a_lat_now, speed * speed * abs(kappa_ff))
            # Only the bound on the command's side of zero can clip it. Commands already
            # inside the safety floors cannot be clipped by either AWD envelope at all.
            if a_cmd > max_acc * acc_floor:
                env_up = _grip_bound(grip_vp, grip_p, a_lat, scale, False)
                a_cmd = min(a_cmd, max(env_up, max_acc * acc_floor))
            elif a_cmd < -max_dec * brk_floor:
                env_dn = _grip_bound(grip_vp, grip_p, a_lat_now, scale, True)
                a_cmd = max(a_cmd, -max(env_dn, max_dec * brk_floor))
            if a_prev is not None:
                a_cmd = min(max(a_cmd, a_prev - jerk), a_prev + jerk)
            a_prev = a_cmd
            pedal = (min(a_cmd / max_acc, 1.0) if a_cmd >= 0.0 else
                     0.0 if a_cmd > -coast else max(a_cmd / max_dec, -1.0))
            if pedal == 0.0 and speed < 0.5 and v0 < 0.5:
                pedal = -0.3  # tracker holds the car when the plan says stop
        thr, brk = (pedal, 0.0) if pedal >= 0.0 else (0.0, -pedal)
        demands.append(speed * speed * abs(math.tan(steer * max_steer)) / L / mu_g)
        s, use = _step(_Scalar, s, steer, thr, brk, c, dt, lag, True)
        uses.append(use)

    poses = np.column_stack((xs, ys))
    use_t, dem_t, rear = np.array(uses), np.array(demands), np.array(rears)
    # worst normalised demand per step (> 1 = over a limit). Lateral demand grows like v^2 and faster
    # near saturation (understeer), so the plan speed at the car's node becomes predicted speed / worst,
    # and at least 1/worst below the plan (when the car lags the plan, scaling only its speed would
    # change nothing). Linear rather than sqrt: it converges in 1-3 passes instead of stalling.
    worst = np.maximum(np.maximum(use_t / use_limit, dem_t / grip_limit), np.abs(rear[1:]) / rear_slip_limit_deg)
    node_t = np.array(nodes[:-1])
    inv = 1.0 / np.maximum(worst, 1e-9)
    scale_t = np.where(worst > 1.0, np.minimum(np.array(spd[:-1]) * inv / np.maximum(np.asarray(v_l)[node_t], 0.1),
                                               inv), 1.0)
    v_scale = np.ones(n_nodes)
    np.minimum.at(v_scale, node_t, np.clip(scale_t, 0.0, 1.0))
    v_scale[:-1] = np.minimum(v_scale[:-1], v_scale[1:])   # start slowing one node early
    dev = np.array(devs)
    clearance = {}
    viol, first, corridor_ok = float("nan"), -1, True
    demand = float(worst.max())
    if corridor is not None:
        clearance = corridor_clearance(poses, corridor)
        violation = clearance["corridor_violation"]
        failed = (violation > 1e-6) | ~clearance["corridor_covered"]
        hits = np.flatnonzero(failed)
        viol = float(clearance["corridor_signed_violation"].max())
        corridor_ok = not len(hits)
        if len(hits):
            first = int(hits[0])
            demand = max(demand, 1.0 + max(viol, 1e-6) / 0.3)
            # A bounded local cap ahead of the first breach. Braking/acceleration reachability
            # is re-run by the planner, so the braking influence extends upstream as needed.
            breach = path_stations[first]
            lead = max(float(spd[first]) * vp.control_dt * (1 + vp.actuation_delay_steps), 1.0)
            anchor = max(breach - lead, 0.0)
            start = max(int(np.searchsorted(stations, anchor, side="right")) - 1, 0)
            end = min(int(np.searchsorted(stations, breach + 2.0)) + 1, n_nodes)
            factor = min(max(1.0 / (1.0 + viol / 0.3), 0.65), 0.9)
            v_scale[start:end] = np.minimum(v_scale[start:end], factor)
    clearance.update(corridor_ok=corridor_ok, corridor_first_step=first,
                     corridor_first_time=first * dt if first >= 0 else -1.0,
                     corridor_first_s=float(clearance["corridor_station"][first]) if first >= 0 else -1.0,
                     corridor_first_path_s=float(path_stations[first]) if first >= 0 else -1.0,
                     corridor_violation_max=viol)
    return {"poses": poses, "yaw": np.array(yaws), "speed": np.array(spd), "yaw_rate": np.array(rates),
            "rear_slip_deg": rear, "lat_dev": dev, "friction_use": use_t, "grip_demand": dem_t,
            "friction_use_max": float(use_t.max()), "grip_demand_max": float(dem_t.max()),
            "rear_slip_max_deg": float(np.abs(rear).max()), "lat_dev_max": float(np.abs(dev).max()),
            "v_scale": v_scale, "demand_max": demand, **clearance,
            "ok": bool(worst.max() <= 1.0 and corridor_ok)}
