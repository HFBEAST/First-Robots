"""Corrected deterministic research-only coordinate-message contract.

No perception, actuator, planner, robot, or hardware API is present here.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

NUMERICAL_ATOL = 1e-12


@dataclass(frozen=True)
class CameraMessage:
    agent: str
    source: str
    frame_id: str
    scenario_version: str
    point_camera_m: tuple[float, float, float]


def world_to_camera(point_world_m: np.ndarray, camera_position_m: np.ndarray, world_from_camera: np.ndarray) -> np.ndarray:
    return world_from_camera.T @ (point_world_m - camera_position_m)


def camera_to_world(message: CameraMessage, camera_position_m: np.ndarray, world_from_camera: np.ndarray) -> dict[str, Any]:
    point_world = camera_position_m + world_from_camera @ np.asarray(message.point_camera_m, dtype=float)
    return {"agent": "coordinate", "source_message_agent": message.agent, "frame_id": message.frame_id, "scenario_version": message.scenario_version, "point_world_m": point_world.tolist()}


def coordinate_agent_accepts(messages: list[dict[str, Any]], registered_frames: set[str], scenario_version: str) -> tuple[bool, str, np.ndarray | None]:
    if not messages:
        return False, "no_coordinate_messages", None
    if any(message["frame_id"] not in registered_frames for message in messages):
        return False, "unregistered_frame", None
    if any(message["scenario_version"] != scenario_version for message in messages):
        return False, "scenario_version_mismatch", None
    points = [np.asarray(message["point_world_m"], dtype=float) for message in messages]
    if not all(np.allclose(points[0], point, rtol=0, atol=NUMERICAL_ATOL) for point in points[1:]):
        return False, "camera_world_coordinate_disagreement", None
    return True, "accepted", points[0]


def table_contains_cup(center_m: np.ndarray, table_center_m: np.ndarray, table_half_extents_m: np.ndarray, cup_radius_m: float, cup_half_height_m: float) -> bool:
    tabletop_z = table_center_m[2] + table_half_extents_m[2]
    xy_lower = table_center_m[:2] - table_half_extents_m[:2] + cup_radius_m
    xy_upper = table_center_m[:2] + table_half_extents_m[:2] - cup_radius_m
    return bool(np.all(center_m[:2] >= xy_lower) and np.all(center_m[:2] <= xy_upper) and np.isclose(center_m[2], tabletop_z + cup_half_height_m, rtol=0, atol=NUMERICAL_ATOL))


def mock_execute(state: dict[str, list[float]], approved: bool, target_m: np.ndarray) -> tuple[bool, str]:
    if not approved:
        return False, "not_approved_no_state_transition"
    state["cup_center_m"] = target_m.tolist()
    return True, "mock_state_transition_applied"
