"""Stage 2: corridor -> path + speed profile. Owner: Rodion.

R2, racing line: every step we re-solve a minimum-curvature line inside the 40 m corridor. The unknowns
are lateral offsets n_i along the corridor normals (bounded by [-right_i, +left_i]). Curvature is
linear in n for small offsets, kappa ~ kappa_c + (D2/ds^2 + diag(kappa_c^2)) n, so the line is one
small regularised least-squares solve (41 x 41, ~50 us), repeated 2-3 times with the violated bounds
pinned (a cheap projected active-set solve). Extra terms tie n_0 and its slope to the car's pose,
keep the line close to last step's solution (shifted by the distance driven) so the published path
does not jump, and fade n back to 0 over the last few metres so we never commit to an unseen apex.

R3 steps 1-2, speed profile on the new line: v_cap = sqrt(a_lat / |kappa|), a visibility guard at
the horizon end (a hairpin of radius vis_hairpin_radius may start just past it), a backward braking
pass and a forward acceleration pass that is power-limited (F = min(F_drive, P/v)) and includes drag
and rolling resistance. Both passes are exact O(K) prefix/suffix reachability minimisations.
Once the finish is visible, only the speculative unseen-corner cap is released.

R3 steps 4 + 6, predictive verification: predict.verify_plan() rolls the car out along the plan for
1.5 s with the dynamic bicycle model driven by (a mirror of) the tracker's control law. Where it
predicts tyre utilisation over 0.95, commanded grip demand over 1.0 or rear slip over 5 deg, it
returns a per-node speed factor; we cap v there and redo the backward/forward passes, at most
verify_iters times, keeping the first clean plan (or the least over-demanding one). The caps it
imposed are remembered by course progress and applied up front next step (relaxed by verify_relax per
step), so a corner is usually found once and later steps verify in a single pass. Results go to
Plan.debug (pred_* keys) for telemetry.

Eco mode (Params.eco_weight in [0, 1], default 0 = everything above, unchanged): one knob trades lap
time for energy inside the speed profile, see _eco_setup / eco_backward_pass. (a) Lift-and-coast: before
each braking zone the profile follows the car's own coast-down (drag + rolling, ~0.2 m/s^2) and only
brakes where coasting can no longer make the corner; the coast length grows with eco_weight and is
limited to what fits in the visible horizon, so the car never has to brake to get onto a coast line.
(c) Gentler acceleration and a lower top speed (drag power grows with v^3). Every eco plan still goes
through predict.verify_plan().

If the solver fails or produces non-finite values we return the Phase 0 centerline plan.

New tunables (TUNABLES below; a Params field of the same name wins, so add them to params.py to tune):
    rl_ds                1.0    m, resampling step (keep it at the 1 m preview spacing: a finer linear
                                resample of the preview polyline makes its curvature alternate)
    rl_w_smooth          4.0    weight on the curvature rate (third difference of n), m^2
    rl_w_start           1e4    weight tying n_0 to the car's lateral offset
    rl_w_heading         20.0   weight tying dn/ds at s=0 to tan(heading_error)
    rl_w_warm            0.05   weight (per m) pulling n toward last step's (shifted) solution; it keeps
                                apexes already committed to (0.002 -> 0.05 cut 1-3 s per course)
    rl_fade_m            4.0    m, length over which n is pulled back to 0 at the horizon end
    rl_w_fade            0.5    weight (per m) of that pull at the very end (grows quadratically)
    rl_iters             3      projected re-solves with the violated bounds pinned
    rl_start_free_m      1.0    m, no corridor bounds before this station (the car may be outside)
    rl_rejoin_m          6.0    m, minimum length over which a car outside the corridor is led back in
    rl_rejoin_time       1.0    s, ... or this many seconds of travel, whichever is longer
    rl_margin            0.5    m, keep the line this far inside the corridor (pure pursuit cuts corners)
    vis_hairpin_radius   8.0    m, worst-case corner radius assumed just past the horizon
    vis_margin_m         0.0    m, braking distance granted between the horizon end and that corner
    fwd_accel_frac       1.0    fraction of vehicle.max_accel the forward pass may use
    fwd_seed_time        1.0    s, the forward pass starts from speed + a(speed) * this
    air_density          1.2    kg/m^3, for the drag term of the forward pass
    verify_iters         1      re-plans allowed after the first verification (0 = verify and log only;
                                negative = skip verification)
    verify_horizon_s     1.5    s, prediction horizon of the verifier
    verify_use_limit     0.95   max predicted tyre utilisation |F| / (mu Fz)
    verify_grip_limit    1.0    max predicted commanded grip demand (tracker caps at 1.05, scorer flags 1.1)
    verify_rear_slip_deg 5.0    max predicted rear-axle slip angle (scorer flags 6 deg)
    verify_relax         1.01   remembered verifier caps grow by this factor per step until re-confirmed
    blocked_stop_margin  1.0    m, stop this far before the car's nose reaches the first blocked station
    blocked_decel_frac   0.8    fraction of vehicle.max_decel used to plan the stop before a blockage
    blocked_creep_speed  2.0    m/s, speed held at the stop point instead of 0, so a blockage that is only
                                transient perception, or a cone wall, never ends in the 8 s "stuck" DNF
                                (set 0 for a true stop)

Blocked corridor (horizon.build_corridor returns valid=False: no gap the car fits through): the plan
is the centerline with a speed profile that brakes to blocked_creep_speed at the stop point in front of
the first blocked station and NEVER exceeds the current speed. It used to be a normal, accelerating
centerline plan (full throttle at the blockage). debug["blocked"] = 1 and debug["blocked_s"] = station.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

from autocross.api import Observation, VehicleParams
from horizon import Corridor
from params import G, Params
from predict import verify_plan


@dataclass
class Plan:
    path: np.ndarray         # (N,2) driven line, vehicle frame; goes into Command.path
    s: np.ndarray            # (N,) arc length along the path
    kappa: np.ndarray        # (N,) signed curvature, + left
    v: np.ndarray            # (N,) target speed after the braking pass
    debug: dict = field(default_factory=dict)   # scalar diagnostics for Command.debug (optional)


TUNABLES = {
    "rl_ds": 1.0, "rl_w_smooth": 4.0, "rl_w_start": 1e4, "rl_w_heading": 20.0, "rl_w_warm": 0.05,
    "rl_fade_m": 4.0, "rl_w_fade": 0.5, "rl_iters": 3, "rl_start_free_m": 1.0,
    "rl_rejoin_m": 6.0, "rl_rejoin_time": 1.0, "rl_margin": 0.5,
    "vis_hairpin_radius": 8.0, "vis_margin_m": 0.0, "fwd_accel_frac": 1.0, "fwd_seed_time": 1.0,
    "air_density": 1.2, "verify_iters": -1, "verify_horizon_s": 1.5, "verify_use_limit": 0.95,
    "verify_grip_limit": 1.0, "verify_rear_slip_deg": 5.0, "verify_relax": 1.01,
    # Eco mode; the Params fields of the same name (params.py) are the ones to tune
    "eco_coast_m": 30.0, "eco_coast_margin": 0.03, "eco_vmax_drop": 0.35, "eco_accel_drop": 0.5,
    "eco_seed_drop": 0.5,
    # blocked corridor
    "blocked_stop_margin": 1.0, "blocked_decel_frac": 0.8, "blocked_creep_speed": 2.0,
}


def _t(p: Params, name: str) -> float:
    """A planner tunable: the Params field if params.py has it, else the default above."""
    return float(getattr(p, name, TUNABLES[name]))


# warm start carried between steps: offsets on the previous centerline grid + where we were
_WARM: dict = {}
# speed caps the verifier imposed last step, by course progress (no cap = _NO_CAP)
_VCAP: dict = {}
_NO_CAP = 1e3


def curvature(path: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Signed curvature of a polyline: (x'y'' - y'x'') / (x'^2 + y'^2)^1.5, derivatives w.r.t. s."""
    d = np.gradient(path, s, axis=0)
    dd = np.gradient(d, s, axis=0)
    dx, dy, ddx, ddy = d[:, 0], d[:, 1], dd[:, 0], dd[:, 1]
    return (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)


def backward_pass(v_cap: np.ndarray, s: np.ndarray, a_dec: float) -> np.ndarray:
    """Fastest speed at each node from which every later cap is reachable at deceleration a_dec:
    v_i = min_{j >= i} sqrt(v_cap_j^2 + 2 a_dec (s_j - s_i)). An O(K) suffix minimum."""
    energy = v_cap * v_cap + 2.0 * a_dec * s
    return np.sqrt(np.maximum(np.minimum.accumulate(energy[::-1])[::-1] - 2.0 * a_dec * s, 0.0))


def accel_table(vehicle: VehicleParams, p: Params, v_top: float,
                accel_scale: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Distance needed to accelerate from 0 to each speed in a table, flat out:
    a(v) = min(a_max, F_drive/m, P/(m v), a_traction) - (drag + rolling)/m and S(v) = int v / a(v) dv.
    The car is all-wheel drive, so the traction limit is set by the load on all four tyres:
    a_traction = traction_limit * mu * g (rear-wheel drive would only get the rear axle's share)."""
    m = vehicle.mass
    v = np.linspace(0.0, v_top, 64)
    f_drive = np.minimum(p.max_drive_force, p.max_power_w / np.maximum(v, 0.5))
    a_traction = float(getattr(p, "traction_limit", 0.97)) * float(getattr(p, "mu_model", vehicle.mu)) * G
    a = np.minimum(np.minimum(_t(p, "fwd_accel_frac") * accel_scale * vehicle.max_accel, f_drive / m), a_traction)
    a -= (0.5 * _t(p, "air_density") * p.drag_area * v * v) / m + p.rolling_resistance * G
    a = np.maximum(a, 0.05)
    integrand = v / a
    dist = np.concatenate(([0.0], np.cumsum(0.5 * (integrand[1:] + integrand[:-1]) * np.diff(v))))
    return v, dist


def forward_pass(v_cap: np.ndarray, s: np.ndarray, table: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """Slowest of all flat-out acceleration curves started from an earlier node's cap:
    v_i = min_{j <= i} V(S(v_cap_j) + s_i - s_j), with S/V the accel_table and its inverse.
    Monotonic V lets us take the prefix minimum in distance space before one interpolation."""
    tv, td = table
    d0 = np.interp(v_cap, tv, td)                                       # (K,) "distance from standstill"
    reachable_distance = np.minimum.accumulate(d0 - s) + s
    return np.interp(reachable_distance, td, tv, right=tv[-1])


def _centerline_plan(c: Corridor, obs: Observation, p: Params) -> Plan:
    """Phase 0 plan: drive the centerline, curvature-limited speed with a slow horizon end."""
    path, s = c.center, c.s
    road_kappa = obs.road.preview_curvature if obs.road is not None else None
    kappa = road_kappa if road_kappa is not None and len(road_kappa) == len(s) else curvature(path, s)
    v_cap = np.minimum(np.sqrt(p.a_lat / np.maximum(np.abs(kappa), 1e-4)), p.v_max)
    v_cap[-1] = min(v_cap[-1], p.v_horizon)
    return Plan(path=path, s=s, kappa=np.asarray(kappa, dtype=float), v=backward_pass(v_cap, s, p.a_dec))


def _diff_matrix(k: int, order: int) -> np.ndarray:
    """(k - order, k) forward-difference matrix of the given order."""
    d = np.eye(k)
    for _ in range(order):
        d = d[1:] - d[:-1]
    return d


@lru_cache(maxsize=8)
def _line_operators(k: int, ds: float) -> tuple[np.ndarray, np.ndarray]:
    """Bounded cache of geometry-independent derivative operators; callers never modify them."""
    d2 = _diff_matrix(k, 2) / ds ** 2
    d3 = _diff_matrix(k, 3) / ds ** 3
    out = d2, d3.T @ d3
    for matrix in out:
        matrix.setflags(write=False)
    return out


def racing_offsets(s: np.ndarray, kappa_c: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                   n0: float, slope0: float, n_warm: np.ndarray, w_warm: np.ndarray,
                   p: Params) -> tuple[np.ndarray, int]:
    """Min-curvature lateral offsets n (K,) with lo <= n <= hi. Returns (n, solves used)."""
    k = len(s)
    ds = float(s[1] - s[0])
    d2, smooth_gram = _line_operators(k, ds)
    a = d2.copy()
    rows = np.arange(k - 2)
    a[rows, rows + 1] += kappa_c[1:-1] ** 2                          # d kappa / d n
    h = ds * (a.T @ a) + _t(p, "rl_w_smooth") * ds * smooth_gram
    g = -ds * (a.T @ kappa_c[1:-1])

    # pose: n_0 = car offset, initial slope = car heading relative to the centerline
    w0, wh = _t(p, "rl_w_start"), _t(p, "rl_w_heading")
    h[0, 0] += w0
    g[0] += w0 * n0
    h[0, 0] += wh / ds ** 2
    h[1, 1] += wh / ds ** 2
    h[0, 1] -= wh / ds ** 2
    h[1, 0] -= wh / ds ** 2
    g[0] -= wh * slope0 / ds
    g[1] += wh * slope0 / ds

    # warm start + fade to the centerline at the horizon end
    diag = w_warm * _t(p, "rl_w_warm") * ds
    g += diag * n_warm
    fade_m = _t(p, "rl_fade_m")
    ramp = np.clip((s - (s[-1] - fade_m)) / max(fade_m, 1e-6), 0.0, 1.0)
    diag = diag + _t(p, "rl_w_fade") * ds * ramp ** 2
    h[np.diag_indices(k)] += diag

    # projected active-set: pin violated bounds with a stiff spring, re-solve
    bounded = s >= _t(p, "rl_start_free_m")
    pin_w = 1e5
    pinned = np.zeros(k)
    target = np.zeros(k)
    n = np.linalg.solve(h, g)
    iters = 1
    for _ in range(int(_t(p, "rl_iters"))):
        over, under = bounded & (n > hi), bounded & (n < lo)
        new = (over | under) & (pinned == 0.0)
        if not new.any():
            break
        target = np.where(over, hi, np.where(under, lo, target))
        pinned = np.where(new, pin_w, pinned)
        n = np.linalg.solve(h + np.diag(pinned), g + pinned * target)
        iters += 1
    n = np.where(bounded, np.clip(n, lo, hi), n)
    return n, iters


def plan(c: Corridor, obs: Observation, vehicle: VehicleParams, p: Params) -> Plan:
    """Racing-line plan, or a braking plan when the corridor is blocked (see the module docstring)."""
    if not c.valid and len(c.s) >= 2:
        return _blocked_plan(c, obs, vehicle, p)
    pl = _plan(c, obs, vehicle, p)
    pl.debug = {**(pl.debug or {}), "blocked": 0.0}
    return pl


def _blocked_plan(c: Corridor, obs: Observation, vehicle: VehicleParams, p: Params) -> Plan:
    """Centerline plan that brakes to a creep (or a stop) before the first blocked station and never
    accelerates. A blocked station is one where horizon.py had to pinch the band to zero width."""
    pl = _centerline_plan(c, obs, p)
    s = pl.s
    width = np.asarray(c.left, float) + np.asarray(c.right, float)
    pinched = np.flatnonzero(width <= 1e-6)
    s_block = float(s[pinched[0]]) if len(pinched) else 0.0          # no pinch found: treat it as here
    s_stop = max(s_block - 0.5 * vehicle.length - _t(p, "blocked_stop_margin"), 0.0)
    creep = max(_t(p, "blocked_creep_speed"), 0.0)
    a_stop = max(_t(p, "blocked_decel_frac") * vehicle.max_decel, 0.1)
    v_stop = np.sqrt(creep * creep + 2.0 * a_stop * np.maximum(s_stop - s, 0.0))
    v_now = max(float(obs.speed) if np.isfinite(obs.speed) else 0.0, creep)
    pl.v = np.minimum(np.minimum(pl.v, v_stop), v_now)                  # brake, never accelerate
    pl.debug = {"blocked": 1.0, "blocked_s": s_block}
    return pl


def _plan(c: Corridor, obs: Observation, vehicle: VehicleParams, p: Params) -> Plan:
    road = obs.road
    if not c.valid or road is None or len(c.s) < 6:
        return _centerline_plan(c, obs, p)

    # 1. resample the corridor uniformly
    ds = float(_t(p, "rl_ds"))
    s = np.arange(0.0, float(c.s[-1]) + 1e-9, ds)
    if len(s) < 6:
        return _centerline_plan(c, obs, p)
    center = np.column_stack([np.interp(s, c.s, c.center[:, 0]), np.interp(s, c.s, c.center[:, 1])])
    normal = np.column_stack([np.interp(s, c.s, c.normal[:, 0]), np.interp(s, c.s, c.normal[:, 1])])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-9)
    # extra margin on top of the corridor buffer: the tracker cuts corners by a few tenths of a metre
    margin = _t(p, "rl_margin")
    hi = np.interp(s, c.s, c.left) - margin
    lo = margin - np.interp(s, c.s, c.right)
    mid = 0.5 * (hi + lo)
    hi, lo = np.maximum(hi, mid), np.minimum(lo, mid)
    kappa_c = curvature(center, s)

    # if the car is outside the shrunk corridor, widen the bounds near it and taper them back, so the
    # line rejoins the corridor smoothly instead of kinking (a kink's curvature would stall the speed)
    n0 = float(road.lateral_offset)
    rejoin = max(_t(p, "rl_rejoin_m"), _t(p, "rl_rejoin_time") * max(float(obs.speed), 0.0))
    taper = np.clip(1.0 - s / rejoin, 0.0, 1.0)
    hi = hi + max(n0 - hi[0], 0.0) * taper
    lo = lo - max(lo[0] - n0, 0.0) * taper

    # 2. warm start: last step's offsets shifted by the centerline distance driven since then
    n_warm, w_warm = np.zeros_like(s), np.zeros_like(s)
    prev = _WARM.get("n")
    if prev is not None and obs.step == _WARM.get("step", -9) + 1:
        shift = obs.progress - _WARM["progress"]
        q = s + shift
        if 0.0 <= shift < 5.0:
            n_warm = np.interp(q, _WARM["s"], prev)
            w_warm = (q <= _WARM["s"][-1]).astype(float)

    try:
        n, _ = racing_offsets(s, kappa_c, lo, hi, n0, float(np.tan(road.heading_error)),
                              n_warm, w_warm, p)
    except np.linalg.LinAlgError:
        _WARM.clear()
        return _centerline_plan(c, obs, p)
    path = center + n[:, None] * normal
    seg = np.hypot(*np.diff(path, axis=0).T)
    sp = np.concatenate(([0.0], np.cumsum(seg)))
    if not (np.all(np.isfinite(path)) and np.all(seg > 1e-6)):
        _WARM.clear()
        return _centerline_plan(c, obs, p)
    _WARM.update(n=n, s=s, step=obs.step, progress=obs.progress)

    # 3. speed profile on the new line
    kappa = curvature(path, sp)
    v_cap = np.minimum(np.sqrt(p.a_lat / np.maximum(np.abs(kappa), 1e-4)), p.v_max)
    r_min = _t(p, "vis_hairpin_radius")
    finish = float(getattr(road, "finish_distance", float("nan")))
    if not (np.isfinite(finish) and 0.0 <= finish <= float(c.s[-1])):
        v_cap[-1] = min(v_cap[-1], np.sqrt(p.a_lat * r_min + 2.0 * p.a_dec * _t(p, "vis_margin_m")))
    eco = float(getattr(p, "eco_weight", 0.0))
    eco_cfg = None
    if eco > 0.0:
        v_cap, eco_cfg = _eco_setup(v_cap, sp, vehicle, p, eco)
    table = accel_table(vehicle, p, max(p.v_max, float(v_cap.max())) + 1.0,
                        1.0 if eco_cfg is None else eco_cfg["accel_scale"])
    v_now = max(float(obs.speed), 0.0)
    a_now = float(np.interp(v_now + 0.1, table[0], table[1]) - np.interp(v_now, table[0], table[1]))
    a_now = 0.1 * (v_now + 0.05) / max(a_now, 1e-6)                      # a = v dv/ds
    v_seed = v_now + a_now * _t(p, "fwd_seed_time") * (1.0 if eco_cfg is None else eco_cfg["seed_scale"])

    def profile(cap: np.ndarray) -> np.ndarray:
        if eco_cfg is None:
            v_back = backward_pass(cap, sp, p.a_dec)
        else:
            v_back, eco_cfg["coast"] = eco_backward_pass(cap, sp, p.a_dec, eco_cfg)
            eco_cfg["v_back"] = v_back.copy()
        v_back[0] = min(v_back[0], v_seed)
        return forward_pass(v_back, sp, table)

    verify = _t(p, "verify_iters") >= 0
    base_cap = v_cap
    if verify and _VCAP and obs.step == _VCAP["step"] + 1:
        v_cap = np.minimum(v_cap, np.interp(obs.progress + s, _VCAP["s"], _VCAP["cap"] * _t(p, "verify_relax"),
                                            left=_NO_CAP, right=_NO_CAP))
    v = profile(v_cap)
    if not (np.all(np.isfinite(v)) and np.all(np.isfinite(kappa))):
        _WARM.clear()
        _VCAP.clear()
        return _centerline_plan(c, obs, p)
    out = Plan(path=path, s=sp, kappa=kappa, v=v)
    if verify:
        out, cap = _verify(out, v_cap, profile, obs, vehicle, p)
        # Progress is measured along the centerline, not along this step's offset racing line.
        _VCAP.update(s=obs.progress + s, cap=np.where(cap < base_cap - 1e-6, cap, _NO_CAP), step=obs.step)
    if eco_cfg is not None:
        out.debug = dict(out.debug or {})
        out.debug.update(_eco_debug(out, eco_cfg, eco, v_now, p))
    return out


def _eco_setup(v_cap: np.ndarray, s: np.ndarray, vehicle: VehicleParams, p: Params,
               eco: float) -> tuple[np.ndarray, dict]:
    """Eco caps + settings. (c) top speed: a fraction of the fastest speed the horizon guard allows
    (so even small eco_weights bite, the 22 m/s v_max is rarely reached); gentler forward pass.
    (a) the coast-down deceleration the car really has at that speed (+ a small margin)."""
    eco = min(max(eco, 0.0), 1.0)
    h = float(s[-1])
    v_guard = float(np.sqrt(v_cap[-1] ** 2 + 2.0 * p.a_dec * h))
    v_cruise = min(p.v_max, v_guard) * (1.0 - _t(p, "eco_vmax_drop") * eco)
    cap = np.minimum(v_cap, v_cruise)
    m = vehicle.mass
    a_coast = ((0.5 * _t(p, "air_density") * p.drag_area * v_cruise ** 2) / m + p.rolling_resistance * G
               + _t(p, "eco_coast_margin"))
    return cap, {"v_cruise": v_cruise, "a_coast": min(a_coast, 0.9 * p.a_dec), "horizon": h,
                 "coast_m": eco * _t(p, "eco_coast_m"), "accel_scale": 1.0 - _t(p, "eco_accel_drop") * eco,
                 "seed_scale": 1.0 - _t(p, "eco_seed_drop") * eco}


def eco_backward_pass(v_cap: np.ndarray, s: np.ndarray, a_dec: float, cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    """Coast-then-brake reachability (vectorised K x K like backward_pass). For every capped node j
    (a corner) the car may arrive along one of two lines in v^2: braking at a_dec, or, further out,
    coasting at a_coast that hands over to braking d_x before j:
        brake_j(d) = v_j^2 + 2 a_dec d
        coast_j(d) = v_j^2 + 2 a_dec d_x + 2 a_coast (d - d_x),  d_x = (V^2 - 2 a_coast L_j - v_j^2) / (2 a_dec)
    so coasting covers L_j metres down from the cruise speed V. L_j is the eco coast length, cut so the
    whole coast + brake fits inside the horizon (else the car would brake to get onto the coast line).
    The horizon-end visibility guard is not a corner and never gets a coast line.
    Returns (v, coast) where coast[i] is True when a coast line is the binding limit at node i."""
    v_cr, a_c, h, coast_m = cfg["v_cruise"], cfg["a_coast"], cfg["horizon"], cfg["coast_m"]
    ds = s[None, :] - s[:, None]
    d = np.maximum(ds, 0.0)
    cap2 = v_cap * v_cap
    d_brake = np.maximum(v_cr * v_cr - cap2, 0.0) / (2.0 * a_dec)
    room = np.clip((h - d_brake) / max(1.0 - a_c / a_dec, 1e-6), 0.0, coast_m)
    d_x = np.maximum(v_cr * v_cr - 2.0 * a_c * room - cap2, 0.0) / (2.0 * a_dec)
    brake = cap2[None, :] + 2.0 * a_dec * d
    coast = cap2[None, :] + 2.0 * a_dec * d_x[None, :] + 2.0 * a_c * (d - d_x[None, :])
    real = (room > 0.0) & (cap2 < v_cr * v_cr - 1e-6)
    real[-1] = False
    coast = np.where(real[None, :], coast, np.inf)
    valid = ds >= 0.0
    b_min = np.where(valid, brake, np.inf).min(axis=1)
    c_min = np.where(valid, coast, np.inf).min(axis=1)
    return np.sqrt(np.maximum(np.minimum(b_min, c_min), 0.0)), c_min < b_min - 1e-9


def _eco_debug(pl: Plan, cfg: dict, eco: float, v_now: float, p: Params) -> dict:
    """dbg_coast = 1 when the speed the tracker follows right now comes from a coast line."""
    s_look = max(v_now * float(getattr(p, "speed_preview_t", 0.25)), 1.0)
    i = int(np.clip(np.searchsorted(pl.s, s_look), 0, len(pl.s) - 1))
    vb = cfg.get("v_back")
    coast = bool(cfg.get("coast") is not None and cfg["coast"][i] and vb is not None
                 and abs(float(pl.v[i]) - float(vb[i])) < 1e-3)
    return {"eco_weight": float(eco), "coast": float(coast), "regen_limited": 0.0,
            "eco_v_cruise": float(cfg["v_cruise"]), "eco_coast_m": float(cfg["coast_m"])}


def _verify(pl: Plan, v_cap: np.ndarray, profile, obs: Observation, vehicle: VehicleParams,
            p: Params) -> tuple[Plan, np.ndarray]:
    """R3.4: predict the car along the plan; where it over-demands the tyres, cap v and redo the passes.
    Returns (the first clean plan, else the least over-demanding one, with pred_* diagnostics; its cap)."""
    from time import perf_counter
    t0 = perf_counter()
    kw = dict(horizon_s=_t(p, "verify_horizon_s"), use_limit=_t(p, "verify_use_limit"),
              grip_limit=_t(p, "verify_grip_limit"), rear_slip_limit_deg=_t(p, "verify_rear_slip_deg"))
    v0 = pl.v
    best, best_res, best_cap, n = pl, None, v_cap, 0
    for it in range(int(_t(p, "verify_iters")) + 1):
        res = verify_plan(pl, obs, vehicle, p, **kw)
        n += 1
        if best_res is None or res["ok"] or res["demand_max"] < best_res["demand_max"]:
            best, best_res, best_cap = pl, res, v_cap
        if res["ok"] or it == int(_t(p, "verify_iters")):
            break
        v_cap = np.minimum(v_cap, pl.v * res["v_scale"])
        v = profile(v_cap)
        if not np.all(np.isfinite(v)):
            break
        pl = Plan(path=pl.path, s=pl.s, kappa=pl.kappa, v=v)
    r = best_res
    best.debug = {"pred_friction_max": r["friction_use_max"], "pred_grip_max": r["grip_demand_max"],
                  "pred_rear_slip_max": r["rear_slip_max_deg"], "pred_offset_max": r["lat_dev_max"],
                  "pred_demand_max": r["demand_max"], "pred_ok": float(r["ok"]), "pred_iters": float(n),
                  "v_scale_min": float(np.min(best.v / np.maximum(v0, 0.1))),
                  "t_verify_ms": (perf_counter() - t0) * 1e3}
    return best, best_cap
