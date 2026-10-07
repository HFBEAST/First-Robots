"""Deterministic pixel-to-plane geometry for the virtual coordinate experiment only."""
from __future__ import annotations

import math

import numpy as np


def pixel_centroid_to_world_plane(centroid_xy: list[float], width: int, height: int, fovy_degrees: float, camera_position_m: np.ndarray, world_from_camera: np.ndarray, plane_z_m: float) -> tuple[np.ndarray, float]:
    u, v = centroid_xy; vertical = math.tan(math.radians(fovy_degrees) / 2); horizontal = vertical * width / height
    x = ((u + 0.5) / width * 2 - 1) * horizontal; y = (1 - (v + 0.5) / height * 2) * vertical
    direction_camera = np.array([x, y, -1.0]); direction_camera /= np.linalg.norm(direction_camera); direction_world = world_from_camera @ direction_camera
    if abs(direction_world[2]) < 1e-15: raise ValueError("ray is parallel to the plane")
    forward_distance = (plane_z_m - camera_position_m[2]) / direction_world[2]
    if forward_distance <= 0: raise ValueError("plane intersection is behind the camera")
    return camera_position_m + forward_distance * direction_world, float(forward_distance)
