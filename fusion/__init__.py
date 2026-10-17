"""Offline, CPU-only LiDAR/camera/radar fusion demo for HackUAB.

This is not loaded by the scored controller. Sensor data is in vehicle axes:
+x forward, +y left, +z up, all distances in meters.
"""
from .schema import FrameBundle, FusedObject, FusionResult
from .fuse import fuse_frame

__all__ = ["FrameBundle", "FusedObject", "FusionResult", "fuse_frame"]
