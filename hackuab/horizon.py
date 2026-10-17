"""Stage 1: road model -> safe corridor. Owner: Saisi.

Everything is in the vehicle frame (x forward, y left). Offsets are measured along the
centerline's left normal, so a path point is ``center + offset * normal`` with
``-right <= offset <= left``.

After an obstacle cut the band can sit entirely on one side of the centerline, so ``left``
or ``right`` may be negative there. Only ``-right <= left`` is guaranteed.

Tunables (read from ``p``; the obstacle ones fall back to these defaults until params.py has them):
  corridor_margin     0.3  m  clearance added to the vehicle half-width on every side
  min_half_width      0.2  m  the wall-only band never shrinks below this per side
  obstacle_hold_m     0.5  m  extra full-cut length before/after an obstacle, past the car half-length
  obstacle_taper_m    8.0  m  ramp length in front of/behind the full cut, so the path shifts smoothly
  side_hysteresis_m   0.3  m  gaps closer than this in width are decided by where the car already is
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from autocross.api import RoadModel, VehicleParams
from params import Params


@dataclass
class Corridor:
    s: np.ndarray            # (K,) station along centerline, 0..range
    center: np.ndarray       # (K,2) centerline points, vehicle frame
    normal: np.ndarray       # (K,2) unit left-normal at each point
    left: np.ndarray         # (K,) usable offset to the left  (+, buffer already removed)
    right: np.ndarray        # (K,) usable offset to the right (+, buffer already removed)
    valid: bool


def left_normals(points: np.ndarray) -> np.ndarray:
    """Unit left-normals of a polyline, (K,2) -> (K,2)."""
    t = np.gradient(points, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    return np.column_stack([-t[:, 1], t[:, 0]])


def _empty() -> Corridor:
    z = np.zeros(0)
    return Corridor(z, np.zeros((0, 2)), np.zeros((0, 2)), z, z.copy(), False)


def build_corridor(road: RoadModel | None, vehicle: VehicleParams, p: Params) -> Corridor:
    if road is None or not road.valid or len(road.preview_s) < 2:
        return _empty()
    margin = float(getattr(p, "corridor_margin", 0.3))
    min_half = float(getattr(p, "min_half_width", 0.2))
    hold = 0.5 * vehicle.length + float(getattr(p, "obstacle_hold_m", 0.5))
    taper = max(float(getattr(p, "obstacle_taper_m", 8.0)), 1e-3)
    hyst = float(getattr(p, "side_hysteresis_m", 0.3))
    buf = 0.5 * vehicle.width + margin

    s = np.nan_to_num(np.asarray(road.preview_s, float))
    center = np.nan_to_num(np.asarray(road.preview_centerline, float))
    normal = left_normals(center)

    # lateral band as [lo, hi] offsets; the cone rows are at -width_right / +width_left
    wall_lo = -np.nan_to_num(np.asarray(road.preview_width_right, float))
    wall_hi = np.nan_to_num(np.asarray(road.preview_width_left, float))
    lo, hi = np.minimum(wall_lo + buf, -min_half), np.maximum(wall_hi - buf, min_half)

    for ob in road.obstacles:
        d, o_min, o_max = float(ob.distance), float(ob.lateral_min), float(ob.lateral_max)
        if not np.isfinite(d + o_min + o_max):
            continue
        # free space either side of the obstacle at its station, measured to the cone rows
        # (cones next to a boundary are dropped by perception, so never assume the outside is open)
        gap_right = (o_min - buf) - np.interp(d, s, lo)
        gap_left = np.interp(d, s, hi) - (o_max + buf)
        if abs(gap_left - gap_right) < hyst:
            pass_left = road.lateral_offset > 0.5 * (o_min + o_max)
        else:
            pass_left = gap_left > gap_right
        # 1 inside [d-hold, d+hold], ramping linearly to 0 over `taper` either side
        w = np.clip((hold + taper - np.abs(s - d)) / taper, 0.0, 1.0)
        if pass_left:
            lo = lo + w * np.maximum(o_max + buf - lo, 0.0)
        else:
            hi = hi - w * np.maximum(hi - (o_min - buf), 0.0)

    # where the band closed up, the car fits only by eating into the margin: drive the middle
    # of the gap, and give up (valid=False) only if it would not fit even with zero margin
    pinch = lo > hi
    valid = bool(np.all(hi - lo > -2.0 * margin))
    if pinch.any():
        mid = 0.5 * (lo + hi)
        lo, hi = np.where(pinch, mid, lo), np.where(pinch, mid, hi)
    return Corridor(s, center, normal, hi, -lo, valid)
