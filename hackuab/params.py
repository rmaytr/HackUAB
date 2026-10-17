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
    cone_margin: float = 0.3            # m, extra clearance on top of half the car width
    min_half_width: float = 0.2         # m, never shrink the usable corridor below this per side

    # --- speed profile (planner.py) ---
    lat_accel_frac: float = 0.6         # fraction of mu*g used for cornering
    decel_frac: float = 0.4             # fraction of max_decel used when planning braking
    accel_frac: float = 0.5             # fraction of max_accel used when ramping the speed target up
    v_max: float = 22.0                 # m/s
    v_horizon: float = 6.0              # m/s, speed cap at the end of the visible horizon

    # --- tracking (tracker.py) ---
    lookahead_gain: float = 0.5         # s, Ld = max(lookahead_min, gain*v + lookahead_offset)
    lookahead_min: float = 4.0          # m
    lookahead_offset: float = 2.0       # m
    kp: float = 0.5
    ki: float = 0.1
    i_limit: float = 5.0
    min_throttle_cap: float = 0.15      # friction-circle throttle cap never goes below this
    brake_gain: float = 0.6

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
