"""Physics-based prediction. Owner: Rodion.

Phase 0: a batched kinematic bicycle rollout plus a delay-compensation helper. Not wired into the
loop yet. Roadmap R3 replaces the kinematic model with the dynamic one (tyres, steering lag, drag)
and uses it to verify plans; roadmap V2 uses predict_pose() for actuation-delay compensation.
"""
from __future__ import annotations

import numpy as np


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
