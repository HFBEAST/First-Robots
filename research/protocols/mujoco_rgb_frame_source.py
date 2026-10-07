"""Virtual-only implementation of the RGB frame-source contract."""
from __future__ import annotations

import mujoco

from research.protocols.rgb_frame_contract import RgbFrame


class MujocoRgbFrameSource:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, width: int, height: int, scenario_version: str):
        self._model, self._data, self._width, self._height, self._scenario_version = model, data, width, height, scenario_version

    def capture(self, camera_name: str) -> RgbFrame:
        renderer = mujoco.Renderer(self._model, height=self._height, width=self._width)
        try:
            renderer.update_scene(self._data, camera=camera_name)
            rgb = renderer.render().copy()
        finally:
            renderer.close()
        return RgbFrame(source_kind="mujoco_rgb_render", frame_id=camera_name, scenario_version=self._scenario_version, pixel_encoding="rgb_uint8", rgb=rgb)
