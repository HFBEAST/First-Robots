"""Research-only RGB frame contract shared by virtual and future real frame adapters."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class RgbFrame:
    source_kind: str
    frame_id: str
    scenario_version: str
    pixel_encoding: str
    rgb: np.ndarray

    def metadata(self) -> dict[str, object]:
        return {"source_kind": self.source_kind, "frame_id": self.frame_id, "scenario_version": self.scenario_version, "pixel_encoding": self.pixel_encoding, "width": int(self.rgb.shape[1]), "height": int(self.rgb.shape[0])}


class RgbFrameSource(Protocol):
    """Future real adapters implement this method without changing the detector contract."""

    def capture(self, camera_name: str) -> RgbFrame: ...


def candidate_magenta_detection(frame: RgbFrame, red_min: int, green_max: int, blue_min: int) -> tuple[np.ndarray, dict[str, object]]:
    """Return a fixture-specific mask from RGB pixels only; no geometry or world truth is accepted."""
    if frame.pixel_encoding != "rgb_uint8" or frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3:
        raise ValueError("RGB frame contract violated")
    red, green, blue = frame.rgb[..., 0], frame.rgb[..., 1], frame.rgb[..., 2]
    mask = (red >= red_min) & (green <= green_max) & (blue >= blue_min)
    rows, columns = np.nonzero(mask)
    centroid = None if len(rows) == 0 else [float(columns.mean()), float(rows.mean())]
    return mask, {"agent": "detection", "source_kind": frame.source_kind, "frame_id": frame.frame_id, "scenario_version": frame.scenario_version, "pixel_count": int(mask.sum()), "centroid_pixel_xy": centroid}
