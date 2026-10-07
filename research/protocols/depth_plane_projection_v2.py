"""MuJoCo virtual depth-plane projection; output remains only a visible surface candidate."""
from __future__ import annotations

import math

import numpy as np


def depth_pixel_to_world_surface_from_camera_plane(centroid_xy:list[float], planar_depth_m:float, width:int, height:int, fovy_degrees:float, camera_position_m:np.ndarray, world_from_camera:np.ndarray)->tuple[np.ndarray,np.ndarray]:
    u,v=centroid_xy;vertical=math.tan(math.radians(fovy_degrees)/2);horizontal=vertical*width/height;x=((u+0.5)/width*2-1)*horizontal;y=(1-(v+0.5)/height*2)*vertical
    if planar_depth_m<=0 or not np.isfinite(planar_depth_m):raise ValueError("planar depth must be finite and positive")
    camera_local=np.array([x*planar_depth_m,y*planar_depth_m,-planar_depth_m]);return camera_position_m+world_from_camera@camera_local,camera_local
