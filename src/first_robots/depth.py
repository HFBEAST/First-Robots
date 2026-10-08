"""Metric optical-frame contracts and nominal virtual fixed-depth capture; no control."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import mujoco
import numpy as np


@dataclass(frozen=True)
class DepthFrame:
    frame_id: str
    scene_version: str
    source_kind: str
    depth_m: np.ndarray
    encoding: str = "planar_depth_float32_m"


@dataclass(frozen=True)
class CameraCalibration:
    frame_id: str
    scene_version: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    position_world_m: np.ndarray
    world_from_optical: np.ndarray

    def validate(self) -> None:
        values = np.r_[self.fx, self.fy, self.cx, self.cy, self.position_world_m.ravel(), self.world_from_optical.ravel()]
        if not self.frame_id or not self.scene_version or self.width <= 0 or self.height <= 0 or not np.isfinite(values).all() or self.fx <= 0 or self.fy <= 0:
            raise ValueError("Invalid camera calibration")
        if self.position_world_m.shape != (3,) or self.world_from_optical.shape != (3, 3):
            raise ValueError("Invalid camera transform dimensions")
        if not np.allclose(self.world_from_optical.T @ self.world_from_optical, np.eye(3), rtol=0, atol=1e-12) or not np.isclose(np.linalg.det(self.world_from_optical), 1, rtol=0, atol=1e-12):
            raise ValueError("Camera transform must be a proper rotation")


class DepthFrameSource(Protocol):
    def capture(self, camera_name: str) -> DepthFrame: ...


def add_fixed_cameras(spec: mujoco.MjSpec, configuration: dict) -> None:
    width, height, fovy = configuration["width"], configuration["height"], configuration["fovy_degrees"]
    if not isinstance(width, int) or not isinstance(height, int) or not (1 <= width <= 4096 and 1 <= height <= 4096 and 0 < fovy < 180):
        raise ValueError("Invalid bounded camera resolution/FOV")
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, width)
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, height)
    if configuration["offsamples"] != 0:
        raise ValueError("Pixel-center depth contract requires offsamples=0")
    spec.visual.quality.offsamples = 0
    for name, settings in configuration["cameras"].items():
        position = np.asarray(settings["position_world_m"], dtype=float)
        target = np.asarray(settings["target_world_m"], dtype=float)
        if position.shape != (3,) or target.shape != (3,) or not np.isfinite(np.r_[position, target]).all():
            raise ValueError("Camera position/target must be finite XYZ")
        backward = position - target
        if np.linalg.norm(backward) == 0:
            raise ValueError("Camera target cannot equal position")
        backward /= np.linalg.norm(backward)
        right = np.cross([0, 0, 1], backward)
        if np.linalg.norm(right) == 0:
            raise ValueError("Vertical look-at needs an explicit up convention")
        right /= np.linalg.norm(right)
        up = np.cross(backward, right)
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(quaternion, np.column_stack((right, up, backward)).ravel())
        camera = spec.worldbody.add_camera()
        camera.name, camera.pos, camera.quat = name, position, quaternion
        camera.fovy, camera.resolution = fovy, [width, height]


def calibration_from_model(model: mujoco.MjModel, data: mujoco.MjData, camera_name: str, configuration: dict) -> CameraCalibration:
    camera = model.camera(camera_name).id
    width, height = configuration["width"], configuration["height"]
    focal = height / (2 * np.tan(np.deg2rad(model.cam_fovy[camera]) / 2))
    calibration = CameraCalibration(camera_name, configuration["scene_version"], width, height, float(focal), float(focal), (width - 1) / 2, (height - 1) / 2, data.cam_xpos[camera].copy(), data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1]))
    calibration.validate()
    return calibration


class MujocoDepthFrameSource:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, configuration: dict):
        if model.vis.quality.offsamples != 0:
            raise ValueError("Multisampled depth is outside this pixel-center contract")
        self.model, self.data, self.configuration = model, data, configuration

    def capture(self, camera_name: str) -> DepthFrame:
        if camera_name not in self.configuration["cameras"]:
            raise ValueError("Unregistered depth camera")
        with mujoco.Renderer(self.model, height=self.configuration["height"], width=self.configuration["width"]) as renderer:
            renderer.enable_depth_rendering()
            renderer.update_scene(self.data, camera=camera_name)
            depth = renderer.render().copy()
        return DepthFrame(camera_name, self.configuration["scene_version"], "mujoco_depth_render", depth)


def unproject_pixels(frame: DepthFrame, calibration: CameraCalibration, pixels_xy: np.ndarray) -> np.ndarray:
    """Project selected valid planar-depth samples; do not normalize optical rays."""
    calibration.validate()
    if frame.frame_id != calibration.frame_id or frame.scene_version != calibration.scene_version or frame.encoding != "planar_depth_float32_m":
        raise ValueError("Frame/calibration identity, version or depth units mismatch")
    if frame.depth_m.shape != (calibration.height, calibration.width) or frame.depth_m.dtype != np.float32:
        raise ValueError("Depth dimensions or dtype mismatch")
    pixels = np.asarray(pixels_xy)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.issubdtype(pixels.dtype, np.integer):
        raise ValueError("Pixels must be an Nx2 integer array")
    if np.any(pixels < 0) or np.any(pixels[:, 0] >= calibration.width) or np.any(pixels[:, 1] >= calibration.height):
        raise ValueError("Pixel outside declared frame")
    depth = frame.depth_m[pixels[:, 1], pixels[:, 0]].astype(float)
    if not np.isfinite(depth).all() or np.any(depth <= 0):
        raise ValueError("Selected depth samples must be finite and positive")
    local = np.column_stack(((pixels[:, 0] - calibration.cx) / calibration.fx * depth, (pixels[:, 1] - calibration.cy) / calibration.fy * depth, depth))
    return calibration.position_world_m + local @ calibration.world_from_optical.T


def project_world_points(points_world_m: np.ndarray, calibration: CameraCalibration) -> tuple[np.ndarray, np.ndarray]:
    calibration.validate()
    points = np.asarray(points_world_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("World points must be finite Nx3")
    local = (points - calibration.position_world_m) @ calibration.world_from_optical
    if np.any(local[:, 2] <= 0):
        raise ValueError("Point is not in front of camera")
    pixels = np.column_stack((calibration.fx * local[:, 0] / local[:, 2] + calibration.cx, calibration.fy * local[:, 1] / local[:, 2] + calibration.cy))
    return pixels, local[:, 2]
