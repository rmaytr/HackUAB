"""Tunables and vehicle-model constants. Owner: Viraj.

Phase 0 values reproduce simulator/controllers/baseline_pure_pursuit.py exactly. Every field can be
overridden from a --controller-config YAML (keys = field names), e.g. `lat_accel_frac: 0.8`.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

from autocross.api import VehicleParams

G = 9.81


@dataclass
class Params:
    # --- corridor (horizon.py) ---
    cone_margin: float = 0.3            # m, extra clearance on top of half the car width (Phase 0 stub)
    min_half_width: float = 0.2         # m, never shrink the usable corridor below this per side (Phase 0 stub)
    corridor_margin: float = 0.3        # m, clearance added to the vehicle half-width on every side
    obstacle_hold_m: float = 0.5        # m, extra full-cut length before/after an obstacle, past the car half-length
    obstacle_taper_m: float = 20.0      # m, ramp length in front of/behind the full cut, so the path shifts smoothly
                                        # (8 -> 20: the racing line no longer pins to the ramp's corners;
                                        #  practice_technical CARLA 42.4 -> 40.1 s, still 0 cones)
    side_hysteresis_m: float = 0.3      # m, gaps closer than this in width are decided by where the car already is

    # --- speed profile (planner.py) ---
    lat_accel_frac: float = 0.6         # fraction of mu*g used for cornering
    decel_frac: float = 0.4             # fraction of max_decel used when planning braking
    accel_frac: float = 0.5             # fraction of max_accel used when ramping the speed target up
    v_max: float = 22.0                 # m/s
    v_horizon: float = 6.0              # m/s, speed cap at the end of the visible horizon

    # --- Eco mode (planner.py, added by Rodion): ONE knob, 0 = today's performance controller ---
    eco_weight: float = 0.0             # 0 = fastest lap (default, unchanged), 1 = most energy-saving
    eco_coast_m: float = 30.0           # m, longest lift-and-coast before a braking zone (at eco_weight 1)
    eco_coast_margin: float = 0.03      # m/s^2, planned coast decel = (drag + rolling) / m + this
    eco_vmax_drop: float = 0.35         # top speed cut at eco_weight 1, fraction of the horizon-guard top speed
    eco_accel_drop: float = 0.5         # forward-pass acceleration cut at eco_weight 1 (fraction)
    eco_seed_drop: float = 0.5          # cut of the forward-pass seed time at eco_weight 1 (less speed-PI wind-up,
                                        #  so less overshoot braking; practice_technical eco 0.6: -3.6 % Wh, -0.4 s)

    # --- tracking (tracker.py) ---
    pp_lookahead_min: float = 4.0       # m
    pp_lookahead_gain: float = 0.32     # s, lookahead grows with speed
    pp_lookahead_max: float = 16.0      # m
    ff_preview_t: float = 0.2           # s, curvature feedforward is read this far ahead of the car
    fb_gain: float = 1.0                # weight of the pure-pursuit error feedback on top of the plan's curvature
    grip_frac: float = 0.85             # fraction of mu each axle's friction circle may use (safety margin)
    grip_cap: float = 1.05              # max commanded v^2*tan(delta)/L as a fraction of mu*g (metric flags > 1.1)
    steer_rate_margin: float = 0.95     # fraction of max_steer_rate the command may use
    speed_preview_t: float = 0.25       # s, how far ahead on the profile the speed target is read
    kp_speed: float = 1.2               # (m/s^2) per (m/s)
    ki_speed: float = 0.5
    i_limit: float = 4.0                # m/s^2, integrator clamp (CARLA needs ~3 to hold speed against drag)
    overspeed_band: float = 1.0         # m/s over the target beyond which a positive integral is ignored
    coast_band: float = 0.25            # m/s^2, small decel requests coast instead of braking
    accel_floor: float = 0.15           # min fraction of max_accel allowed even at full lateral load
    brake_floor: float = 0.3            # min fraction of max_decel ever allowed (safety over realism)
    jerk_max: float = 40.0              # m/s^3, max change of the acceleration command
    slip_ema_alpha: float = 0.2         # low-pass on the yaw-rate understeer indicator
    slip_threshold: float = 0.15        # filtered 1 - r_meas/r_expected above this counts as understeer
    rear_slip_max_deg: float = 4.0      # rear-axle slip above this counts as oversteer
    slip_drop: float = 0.01             # a_lat_scale drop per step while slipping
    slip_recover: float = 0.002         # a_lat_scale recovery per step otherwise
    slip_scale_min: float = 0.75

    # --- compute budget (controller.py) ---
    budget_ms: float = 35.0             # soft per-step budget; over it is logged in debug

    # --- dynamic bicycle model (predict.py), copied from simulator/config/sim.yaml -> vehicle.bicycle ---
    # These are vehicle parameters, not course data. Re-copy if the organisers refit the model.
    cg_to_front: float = 1.45
    cg_height: float = 0.5
    cornering_stiffness_per_load: float = 14.0   # 1/rad, C_alpha / Fz
    mu_model: float = 0.97
    max_drive_force: float = 9900.0              # N
    max_power_w: float = 200000.0                # W
    max_brake_force: float = 17500.0             # N
    drag_area: float = 0.55                      # m^2, Cd*A
    rolling_resistance: float = 0.012
    steer_time_constant: float = 0.06            # s, first-order steering lag
    # Drivetrain: the car is ALL-WHEEL DRIVE (EIC vehicle: Chevy Blazer EV, AWD; matches the bicycle backend).
    # Traction is limited by the load on all four tyres, and drive force also uses front-tyre grip in
    # corners (power-on understeer). Used by the tracker's per-axle friction model: each axle's
    # drive/brake force and its share of the cornering force must fit inside that axle's friction
    # circle, with load transfer from cg_height. Don't model it as rear- or front-wheel drive anywhere.
    front_drive_fraction: float = 0.4            # share of drive force on the front axle (rear gets 0.6)
    front_brake_fraction: float = 0.6            # share of brake force on the front axle
    traction_limit: float = 0.97                 # TC/ABS: max |Fx| per axle as a fraction of mu*Fz

    # --- derived from VehicleParams in from_vehicle() ---
    a_lat: float = 0.0
    a_dec: float = 0.0
    a_acc: float = 0.0
    half_width: float = 0.0

    @classmethod
    def from_vehicle(cls, vehicle: VehicleParams, config: dict[str, Any] | None = None) -> "Params":
        names = {f.name for f in fields(cls)}
        p = cls(**{k: float(v) for k, v in (config or {}).items() if k in names})
        p.a_lat = p.lat_accel_frac * vehicle.mu * G
        p.a_dec = p.decel_frac * vehicle.max_decel
        p.a_acc = p.accel_frac * vehicle.max_accel
        p.half_width = 0.5 * vehicle.width
        return p
