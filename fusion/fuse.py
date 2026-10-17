"""Fast, transparent object-level sensor fusion. Pure NumPy; no model training/GPU.

Pipeline: z-filter LiDAR -> 2D grid connected components -> image color patch
association -> radar nearest-neighbor association. A cluster without a reliable
camera color remains UNKNOWN, rather than being misreported as an obstacle.

Assumptions:
  * standardized LiDAR points are vehicle x-forward, y-left, z-up;
  * camera is pointed along vehicle +x with negligible roll/pitch/yaw;
  * radar polar azimuth is LEFT-positive in the vehicle frame;
  * cone-scale objects only. Real-world generic object perception needs more.
"""
from __future__ import annotations
from time import perf_counter
import math
import numpy as np
from .schema import FrameBundle, FusedObject, FusionResult


# Scene sizes, clustering and color thresholds can be tuned on CARLA recordings.
CELL_M = 0.25
MIN_POINTS = 5
MAX_OBJECT_WIDTH_M = 1.0
RADAR_GATE_M = 1.0


def project_to_image(xyz: np.ndarray, frame: FrameBundle) -> np.ndarray:
    """Project vehicle xyz to pixel (u,v) for a forward-facing level camera.

    CAUTION: This only supports an aligned camera. A camera rotated relative
    to ego needs calibrated extrinsics before using this projection.
    """
    q = np.atleast_2d(np.asarray(xyz, dtype=float)) - frame.camera_xyz
    depth = np.maximum(q[:, 0], 1e-6)
    K = frame.K
    return np.column_stack((K[0, 2] - K[0, 0] * q[:, 1] / depth,
                            K[1, 2] - K[1, 1] * q[:, 2] / depth))


def _connected_components(cells: np.ndarray) -> list[list[int]]:
    """8-connected components over occupied BEV grid CELLS, NOT LiDAR points."""
    indices = {tuple(v): i for i, v in enumerate(cells)}
    visited = np.zeros(len(cells), dtype=bool)
    components: list[list[int]] = []
    for start in range(len(cells)):
        if visited[start]:
            continue
        visited[start] = True
        stack = [start]
        comp = []
        while stack:
            idx = stack.pop()
            comp.append(idx)
            x, y = cells[idx]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    j = indices.get((int(x + dx), int(y + dy)))
                    if j is not None and not visited[j]:
                        visited[j] = True
                        stack.append(j)
        components.append(comp)
    return components


def _color_from_image(frame: FrameBundle, xy: np.ndarray, z: float) -> tuple[int, float]:
    """Classify a small projected RGB patch. Returns -1 when colors are ambiguous.

    Synthetic blue/yellow/orange cones are easy; many CARLA course props use the
    *same* orange construction-cone mesh for all boundary and obstacle cones.
    In that case RGB alone cannot infer the organizer's semantic cone class.
    """
    uv = project_to_image(np.array([[xy[0], xy[1], z]]), frame)[0]
    if not np.isfinite(uv).all() or xy[0] <= frame.camera_xyz[0] + .1:
        return -1, 0.0
    u, v = np.rint(uv).astype(int)
    h, w = frame.image.shape[:2]
    radius = 7
    if u < -radius or u >= w + radius or v < -radius or v >= h + radius:
        return -1, 0.0
    patch = frame.image[max(0, v-radius):min(h, v+radius+1),
                        max(0, u-radius):min(w, u+radius+1)].astype(np.float32)
    if not patch.size:
        return -1, 0.0
    r, g, b = patch[:, :, 0], patch[:, :, 1], patch[:, :, 2]
    masks = [
        (b > 110) & (b > 1.28 * r) & (b > 1.12 * g),  # blue
        (r > 125) & (g > 105) & (b < .72 * r) & (g > .65 * r), # yellow
        (r > 135) & (g > 50) & (g < .72 * r) & (b < .6 * r), # orange
    ]
    votes = np.array([np.count_nonzero(m) for m in masks])
    best = int(votes.argmax())
    # Small colorful window means no trustworthy visual association.
    if votes[best] < 5 or votes[best] < 1.5 * np.partition(votes, -2)[-2]:
        return -1, 0.0
    # A heuristic quality measure, not a detection probability.
    return best, min(1.0, float(votes[best]) / 35.0)


def fuse_frame(frame: FrameBundle, trust_carla_rgb: bool = False) -> FusionResult:
    """Return colored 2D object detections, radar velocity associations and timings.

    The timing covers only fusion stages, not image generation, disk, or plotting.
    """
    t0 = perf_counter()
    cloud = np.asarray(frame.lidar, dtype=np.float32)
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        raise ValueError("lidar must have shape (N,4) with xyz in vehicle frame")
    good = (np.isfinite(cloud[:, :3]).all(axis=1)
            & (cloud[:, 0] > 1.0) & (cloud[:, 0] < 40.0)
            & (np.abs(cloud[:, 1]) < 12.0)
            & (cloud[:, 2] > 0.08) & (cloud[:, 2] < 1.5))
    pts = cloud[good, :3]
    t1 = perf_counter()
    if len(pts) == 0:
        return FusionResult([], {"filter": 1000*(t1-t0), "cluster": 0.,
                                 "camera": 0., "radar": 0., "total": 1000*(t1-t0)}, 0, 0)
    cells, inverse = np.unique(np.floor(pts[:, :2] / CELL_M).astype(np.int32),
                               axis=0, return_inverse=True)
    comps = _connected_components(cells)
    # Convert the connected components into per-point labels in O(N).
    grid_labels = np.zeros(len(cells), dtype=np.int32)
    for label, comp in enumerate(comps, start=1):
        grid_labels[comp] = label
    point_labels = grid_labels[inverse]
    clusters = []
    for label in range(1, len(comps)+1):
        # One vectorized pass per candidate component. For cone-scale scenes this
        # keeps the code simple while avoiding a Python loop over LiDAR points.
        subset = pts[point_labels == label]
        if len(subset) < MIN_POINTS:
            continue
        xy_span = np.ptp(subset[:, :2], axis=0)
        if np.max(xy_span) > MAX_OBJECT_WIDTH_M:
            continue  # likely a wall or a merged group, not a cone-scale object
        clusters.append((np.median(subset[:, :2], axis=0),
                         float(np.median(subset[:, 2])), len(subset)))
    t2 = perf_counter()
    classified = [(_color_from_image(frame, xy, z), xy, n) for xy, z, n in clusters]
    # The organizer's CARLA config uses the same constructioncone blueprint for
    # blue/yellow/orange roles. Without independently marked colored materials,
    # RGB cannot determine their semantic role. Keep colors unknown by default;
    # the OFFLINE bridge may use the existing road geometry as an explicit prior.
    if frame.source.lower() == "carla" and not trust_carla_rgb:
        classified = [((-1, 0.0), xy, n) for (_, xy, n) in classified]
    t3 = perf_counter()

    radar = np.asarray(frame.radar, dtype=float)
    if radar.size == 0:
        radar_xy = np.empty((0, 2))
    else:
        if radar.ndim != 2 or radar.shape[1] != 4:
            raise ValueError("radar must be (M,4) [range,azimuth_left,altitude,radial_velocity]")
        keep = np.isfinite(radar).all(axis=1) & (radar[:, 0] > 0)
        radar = radar[keep]
        rho = radar[:, 0] * np.cos(radar[:, 2])
        radar_xy = np.column_stack((rho*np.cos(radar[:, 1]), rho*np.sin(radar[:, 1])))
    result = []
    for (color, quality), xy, n in classified:
        velocity = math.nan
        sources = ["lidar"]
        if color >= 0:
            sources.append("camera")
        if len(radar_xy):
            d2 = ((radar_xy-xy[None, :])**2).sum(axis=1)
            j = int(np.argmin(d2))
            if d2[j] < RADAR_GATE_M**2:
                velocity = float(radar[j, 3]); sources.append("radar")
        # Defined heuristic confidence; not a calibrated posterior.
        confidence = min(1., .45 + .35*quality + (.15 if np.isfinite(velocity) else 0.))
        result.append(FusedObject(np.asarray(xy, dtype=float), color, velocity,
                                  confidence, int(n), tuple(sources)))
    t4 = perf_counter()
    return FusionResult(result, {"filter": (t1-t0)*1000, "cluster": (t2-t1)*1000,
                                  "camera": (t3-t2)*1000, "radar": (t4-t3)*1000,
                                  "total": (t4-t0)*1000}, len(pts), len(comps))
