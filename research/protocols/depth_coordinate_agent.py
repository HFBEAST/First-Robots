"""Deterministic structural gate for fused virtual depth coordinate messages."""
from __future__ import annotations

from typing import Any

import numpy as np


def coordinate_message(center_world_m: list[float], source_frame_ids: list[str], scenario_version: str, object_model_version: str) -> dict[str, Any]:
    return {"agent":"coordinate","source":"dual_depth_known_cylinder_fit","center_world_m":center_world_m,"source_frame_ids":source_frame_ids,"scenario_version":scenario_version,"object_model_version":object_model_version}


def coordinator_accepts(message:dict[str,Any], required_frames:set[str], scenario_version:str, object_model_version:str)->tuple[bool,str,np.ndarray|None]:
    if set(message.get("source_frame_ids",[]))!=required_frames:return False,"required_depth_frames_incomplete",None
    if message.get("scenario_version")!=scenario_version:return False,"scenario_version_mismatch",None
    if message.get("object_model_version")!=object_model_version:return False,"object_model_version_mismatch",None
    point=np.asarray(message.get("center_world_m",[]),dtype=float)
    if point.shape!=(3,) or not np.isfinite(point).all():return False,"invalid_center_coordinate",None
    return True,"accepted",point
