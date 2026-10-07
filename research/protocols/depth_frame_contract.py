"""Research-only depth-frame contract for virtual and future real fixed-depth adapters."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class DepthFrame:
    source_kind: str
    frame_id: str
    scenario_version: str
    pixel_encoding: str
    depth: np.ndarray

    def metadata(self) -> dict[str, object]:
        return {"source_kind":self.source_kind,"frame_id":self.frame_id,"scenario_version":self.scenario_version,"pixel_encoding":self.pixel_encoding,"width":int(self.depth.shape[1]),"height":int(self.depth.shape[0]),"dtype":str(self.depth.dtype)}


class DepthFrameSource(Protocol):
    def capture(self, camera_name: str) -> DepthFrame: ...
