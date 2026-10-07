"""Virtual-only implementation of the depth-frame source contract."""
from __future__ import annotations

import mujoco

from research.protocols.depth_frame_contract import DepthFrame


class MujocoDepthFrameSource:
    def __init__(self, model:mujoco.MjModel, data:mujoco.MjData, width:int, height:int, scenario_version:str): self._model,self._data,self._width,self._height,self._scenario_version=model,data,width,height,scenario_version

    def capture(self,camera_name:str)->DepthFrame:
        renderer=mujoco.Renderer(self._model,height=self._height,width=self._width)
        try:
            renderer.enable_depth_rendering();renderer.update_scene(self._data,camera=camera_name);depth=renderer.render().copy()
        finally: renderer.close()
        return DepthFrame(source_kind="mujoco_depth_render",frame_id=camera_name,scenario_version=self._scenario_version,pixel_encoding="depth_float32_m",depth=depth)
