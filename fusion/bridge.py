"""Adapter from fused objects to the existing HackUAB autonomy stack.

Standalone/offline demo only. Do NOT import this into hackuab/controller.py:
the official controller receives a processed RoadModel and ConeDetections;
raw CARLA sensors are not available there and the controller is sandboxed.
"""
from __future__ import annotations
from dataclasses import replace
import numpy as np
from .schema import FusionResult


def to_cone_detections(fused: FusionResult, range_m: float = 30.,
                       fov_rad: float = np.deg2rad(140.)):
    """Convert classified cone-like clusters into organizer ConeDetections."""
    from autocross.api import ConeDetections
    known = [o for o in fused.objects
             if (o.color in (0, 1, 2) and o.confidence >= .5
                 and float(np.linalg.norm(o.xy)) <= range_m
                 and abs(float(np.arctan2(o.xy[1], o.xy[0]))) <= fov_rad/2.)]
    positions = np.array([o.xy for o in known], dtype=float).reshape(-1, 2)
    colors = np.array([o.color for o in known], dtype=np.int32)
    return ConeDetections(positions=positions, colors=colors,
                          range_m=float(range_m), fov_rad=float(fov_rad))


def _project_to_centerline(point: np.ndarray, road) -> tuple[float, float]:
    """Continuous nearest segment: (station in metres, signed lateral offset +left)."""
    center = np.asarray(road.preview_centerline, dtype=float)
    s = np.asarray(road.preview_s, dtype=float)
    if len(center) < 2 or len(s) != len(center):
        raise ValueError("road must have at least 2 preview_centerline points")
    a, segment = center[:-1], center[1:] - center[:-1]
    length2 = np.maximum((segment**2).sum(axis=1), 1e-12)
    t = np.clip(np.sum((point[None, :] - a)*segment, axis=1) / length2, 0., 1.)
    proj = a + t[:, None] * segment
    j = int(np.argmin(np.sum((proj-point[None, :])**2, axis=1)))
    station = float(s[j] + t[j]*(s[j+1]-s[j]))
    tangent = segment[j] / np.sqrt(length2[j])
    normal = np.array([-tangent[1], tangent[0]])
    offset = float((point-proj[j]) @ normal)
    return station, offset


def to_obstacles(fused: FusionResult, road, group_distance_m: float = 1.15,
                 cone_radius_m: float = .17):
    """Build organizer ObstacleInfo objects from fused orange cone clusters.

    Boundary cones (blue/yellow) are NOT obstacles. Nearby orange points are
    grouped into an obstacle. Distances/offsets are along the centerline, NOT
    ego-x and ego-y in curves. Unknown-color clusters are not silently orange.
    """
    from autocross.api import ObstacleInfo
    orange = [np.asarray(o.xy, float) for o in fused.objects
              if o.color == 2 and o.confidence >= .5]
    if not orange:
        return []
    xy = np.stack(orange)
    used = np.zeros(len(xy), dtype=bool)
    groups = []
    for i in range(len(xy)):
        if used[i]:
            continue
        queue = [i]
        used[i] = True
        inds = []
        while queue:
            idx = queue.pop()
            inds.append(idx)
            close = np.where(~used & (((xy-xy[idx])**2).sum(axis=1) <= group_distance_m**2))[0]
            used[close] = True
            queue.extend(close.tolist())
        groups.append(xy[inds])
    obstacles = []
    for points in groups:
        projections = [_project_to_centerline(pt, road) for pt in points]
        stations = np.array([p[0] for p in projections])
        offsets = np.array([p[1] for p in projections])
        station = float(np.median(stations))
        if not np.isfinite(station) or station < -1. or station > float(road.range_m) + 1.:
            continue
        obstacles.append(ObstacleInfo(distance=station,
                                      lateral_min=float(offsets.min()-cone_radius_m),
                                      lateral_max=float(offsets.max()+cone_radius_m),
                                      center=np.mean(points, axis=0), points=points.copy()))
    return sorted(obstacles, key=lambda o: o.distance)


def inject_fusion(obs, fused: FusionResult, replace_existing_obstacles: bool = True):
    """Return a NEW Observation (and RoadModel) with fused cones and obstacles.

    Default replaces the processed RoadModel's obstacle list to create a fair
    *offline* fusion-input comparison. It is not a modification of the scored
    simulator or controller. Remaining road geometry is the harness road model.
    """
    if obs.road is None:
        raise ValueError("Need an existing RoadModel; this adapter does not infer full road geometry")
    cones = to_cone_detections(fused)
    obstacles = to_obstacles(fused, obs.road)
    if not replace_existing_obstacles:
        obstacles = list(obs.road.obstacles) + obstacles
    return replace(obs, road=replace(obs.road, obstacles=obstacles), cones=cones)


def label_from_road_prior(fused: FusionResult, road, boundary_tol_m: float = .9,
                          interior_margin_m: float = 1.15) -> FusionResult:
    """OFFLINE optional semantic inference using harness-provided road boundaries.

    CARLA's same-looking construction-cone meshes do not encode which row is
    left/right or which cone is a surprise obstacle. This prior uses the *known
    RoadModel* to classify geometry: near left/right cone rows => boundary,
    safely inside the corridor => orange. It is NOT raw-sensor-only fusion, and
    it can be wrong near corners or narrow passages. Keep this outside scored
    hackuab/ code and disclose its use in any demonstration.
    """
    from dataclasses import replace as _replace
    labeled = []
    s = np.asarray(road.preview_s, dtype=float)
    wl = np.asarray(road.preview_width_left, dtype=float)
    wr = np.asarray(road.preview_width_right, dtype=float)
    for obj in fused.objects:
        if obj.color >= 0:
            labeled.append(obj); continue
        station, offset = _project_to_centerline(obj.xy, road)
        if station < 0 or station > float(road.range_m):
            labeled.append(obj); continue
        left = float(np.interp(station, s, wl))
        right = float(np.interp(station, s, wr))
        dl, dr = abs(offset-left), abs(offset+right)
        if dl < boundary_tol_m and dl < dr:
            color = 0
        elif dr < boundary_tol_m:
            color = 1
        elif -right+interior_margin_m <= offset <= left-interior_margin_m:
            color = 2
        else:
            color = -1
        labeled.append(_replace(obj, color=color,
                                sources=obj.sources + (("road_prior",) if color >= 0 else ())))
    return _replace(fused, objects=labeled)
