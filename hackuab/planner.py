"""Stage 2: corridor -> path + speed profile. Owner: Rodion.

Phase 0 stub: drive the centerline. Speed = curvature limit, with the end of the horizon treated
as a possible slow corner, then a backward braking pass (same result as the baseline at s = 0).
Roadmap R2/R3 replace this with the min-curvature line, forward pass and predictive verifier.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from autocross.api import Observation, VehicleParams
from horizon import Corridor
from params import Params


@dataclass
class Plan:
    path: np.ndarray         # (N,2) driven line, vehicle frame; goes into Command.path
    s: np.ndarray            # (N,) arc length along the path
    kappa: np.ndarray        # (N,) signed curvature, + left
    v: np.ndarray            # (N,) target speed after the braking pass


def curvature(path: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Signed curvature of a polyline: (x'y'' - y'x'') / (x'^2 + y'^2)^1.5, derivatives w.r.t. s."""
    dx, dy = np.gradient(path[:, 0], s), np.gradient(path[:, 1], s)
    ddx, ddy = np.gradient(dx, s), np.gradient(dy, s)
    return (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)


def backward_pass(v_cap: np.ndarray, s: np.ndarray, a_dec: float) -> np.ndarray:
    """Fastest speed at each node from which every later cap is reachable at deceleration a_dec:
    v_i = min_{j >= i} sqrt(v_cap_j^2 + 2 a_dec (s_j - s_i)). Vectorised (K x K, K ~ 40)."""
    ds = s[None, :] - s[:, None]
    reach = np.sqrt(v_cap[None, :] ** 2 + 2.0 * a_dec * np.maximum(ds, 0.0))
    return np.where(ds >= 0.0, reach, np.inf).min(axis=1)


def plan(c: Corridor, obs: Observation, vehicle: VehicleParams, p: Params) -> Plan:
    path, s = c.center, c.s
    road_kappa = obs.road.preview_curvature if obs.road is not None else None
    kappa = road_kappa if road_kappa is not None and len(road_kappa) == len(s) else curvature(path, s)
    v_cap = np.minimum(np.sqrt(p.a_lat / np.maximum(np.abs(kappa), 1e-4)), p.v_max)
    v_cap[-1] = min(v_cap[-1], p.v_horizon)
    return Plan(path=path, s=s, kappa=np.asarray(kappa, dtype=float), v=backward_pass(v_cap, s, p.a_dec))
