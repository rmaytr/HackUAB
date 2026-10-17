"""Stage 1: road model -> safe corridor. Owner: Saisi.

Phase 0 stub: centerline corridor shrunk by the car's half width + margin. Obstacles are ignored
(roadmap S1 adds them).
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


def build_corridor(road: RoadModel, vehicle: VehicleParams, p: Params) -> Corridor:
    center = np.asarray(road.preview_centerline, dtype=float)
    s = np.asarray(road.preview_s, dtype=float)
    if center.shape[0] < 2:
        z = np.zeros(center.shape[0])
        return Corridor(s, center, np.zeros_like(center), z, z, valid=False)
    buffer = p.half_width + p.cone_margin
    left = np.maximum(np.asarray(road.preview_width_left) - buffer, p.min_half_width)
    right = np.maximum(np.asarray(road.preview_width_right) - buffer, p.min_half_width)
    return Corridor(s, center, left_normals(center), left, right, valid=True)
