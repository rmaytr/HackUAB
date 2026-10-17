"""Synthetic sensor frames for a straight sample autocross segment.

Ground truth is used ONLY to generate and evaluate these frames, never inside
fuse_frame(). The scored controller does not use this source.
"""
from __future__ import annotations
import numpy as np
from .schema import FrameBundle
from .fuse import project_to_image

# World coordinates for the demonstration track: +x forward, +y left.
_COLORS = np.array([(45, 115, 248), (241, 218, 48), (250, 119, 38)], dtype=np.uint8)


def course_cones() -> tuple[np.ndarray, np.ndarray]:
    x = np.arange(6., 199., 6.)
    left = np.column_stack((x, np.full_like(x, 3.6)))
    right = np.column_stack((x, np.full_like(x, -3.6)))
    # Orange cone is NOT part of either boundary. Second appears later in replay.
    obstacles = np.array([[20., -.4], [49., .65], [81., -.45],
                          [113., .35], [151., -.6], [183., .2]])
    return (np.concatenate((left, right, obstacles), axis=0),
            np.array([0]*len(left) + [1]*len(right) + [2]*len(obstacles), dtype=np.int32))


def _disk(image: np.ndarray, u: int, v: int, radius: int, color: np.ndarray) -> None:
    h, w = image.shape[:2]
    if u + radius < 0 or v + radius < 0 or u - radius >= w or v - radius >= h:
        return
    yy, xx = np.ogrid[max(0, v-radius):min(h, v+radius+1),
                       max(0, u-radius):min(w, u+radius+1)]
    image[max(0, v-radius):min(h, v+radius+1),
          max(0, u-radius):min(w, u+radius+1)][(xx-u)**2+(yy-v)**2 <= radius**2] = color


def make_frame(index: int = 0, seed: int = 7, speed_mps: float = 6.0,
               dt: float = .05) -> tuple[FrameBundle, np.ndarray, np.ndarray]:
    """Generate RGB + LiDAR + radar and return (frame, visible_truth_xy, truth_colors)."""
    rng = np.random.default_rng(seed + index * 101)
    progress = index * dt * speed_mps
    xy_world, colors = course_cones()
    xy = xy_world - np.array([progress, 0.0])
    vis = ((xy[:, 0] > 2.0) & (xy[:, 0] < 35.0)
           & (np.abs(np.arctan2(xy[:, 1], xy[:, 0])) < np.deg2rad(64)))
    xy, colors = xy[vis], colors[vis]
    image = np.empty((180, 320, 3), dtype=np.uint8)
    image[:] = [34, 43, 53]
    # Simple road stripe; not used in classification.
    image[141:143, :, :] = [75, 80, 85]
    K = np.array([[160., 0., 160.], [0., 160., 90.], [0., 0., 1.]])
    cam = np.array([1.5, 0., 1.2])
    tmp = FrameBundle(index*dt, np.empty((0, 4)), np.empty((0, 4)),
                      image, K, cam, np.array([progress, 0., 0.]), speed_mps)

    cones_lidar = []
    radar = []
    for p, col in zip(xy, colors):
        # Colored patch in the pinhole image; radius decreases with distance.
        uv = project_to_image(np.array([[p[0], p[1], .43]]), tmp)[0]
        radius = max(3, min(9, round(65 / max(p[0], 5.))))
        _disk(image, int(round(uv[0])), int(round(uv[1])), radius, _COLORS[col])
        # 36 returns concentrated around the cone body + 3D height.
        n = 36
        spread = rng.normal(0, .065, size=(n, 2))
        zs = rng.uniform(.13, .75, size=(n, 1))
        intensity = rng.uniform(.35, .9, size=(n, 1))
        cones_lidar.append(np.column_stack((p+spread, zs, intensity)))
        # Radar returns are optional; for stationary cones apparent velocity is
        # approximately -ego_speed*cos(bearing) in sensor polar convention.
        if col == 2:
            rho = float(np.linalg.norm(p))
            az = float(np.arctan2(p[1], p[0]))
            radar.append([rho+rng.normal(0, .13), az+rng.normal(0, .005),
                          0., -speed_mps*np.cos(az)+rng.normal(0, .08)])
    # Ground returns are removed by the z threshold. No artificial noise remains
    # above that threshold; real CARLA will have more clutter and occlusions.
    ng = 240
    ground = np.column_stack((rng.uniform(0, 35, ng), rng.uniform(-5, 5, ng),
                              rng.uniform(-.03, .065, ng), rng.uniform(.1, .4, ng)))
    lidar = np.concatenate(cones_lidar + [ground], axis=0).astype(np.float32)
    frame = FrameBundle(t=index*dt, lidar=lidar,
                        radar=np.asarray(radar, dtype=np.float32).reshape(-1, 4),
                        image=image, K=K, camera_xyz=cam,
                        ego_pose=np.array([progress, 0., 0.]),
                        ego_speed_mps=speed_mps, source="synthetic")
    return frame, xy, colors
