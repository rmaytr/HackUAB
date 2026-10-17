"""Explicit frame format shared by synthetic and (optional) CARLA capture sources."""
from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np


@dataclass
class FrameBundle:
    # Standardized into the VEHICLE frame before fusion, not raw CARLA sensor axes.
    t: float
    lidar: np.ndarray             # (N,4): x forward, y left, z up, intensity
    radar: np.ndarray             # (M,4): range, azimuth LEFT positive, altitude, radial velocity
    image: np.ndarray             # (H,W,3) RGB uint8, optical axis roughly forward
    K: np.ndarray                 # (3,3) standard pinhole intrinsics
    camera_xyz: np.ndarray        # (3,) location in vehicle coordinates
    ego_pose: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world x,y,yaw
    ego_speed_mps: float = 0.0
    source: str = "synthetic"

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, t=float(self.t), lidar=self.lidar, radar=self.radar,
                            image=self.image, K=self.K, camera_xyz=self.camera_xyz,
                            ego_pose=self.ego_pose, ego_speed_mps=self.ego_speed_mps,
                            source=np.asarray(self.source))

    @classmethod
    def load(cls, path: str | Path) -> "FrameBundle":
        with np.load(path, allow_pickle=False) as f:
            return cls(t=float(f["t"]), lidar=f["lidar"], radar=f["radar"],
                       image=f["image"], K=f["K"], camera_xyz=f["camera_xyz"],
                       ego_pose=f["ego_pose"], ego_speed_mps=float(f["ego_speed_mps"]),
                       source=str(f["source"]))


@dataclass
class FusedObject:
    xy: np.ndarray              # (2,) position in vehicle frame
    color: int                  # 0 blue/left, 1 yellow/right, 2 orange/obstacle, -1 unknown
    radial_velocity: float      # m/s; NaN when there was no radar match
    confidence: float           # heuristic 0..1, NOT calibrated probability
    lidar_points: int
    sources: tuple[str, ...]


@dataclass
class FusionResult:
    objects: list[FusedObject]
    stage_ms: dict[str, float]
    lidar_points_used: int
    candidate_clusters: int
