"""Virtual depth-ray surface-point projection; deliberately does not estimate object centers."""
from __future__ import annotations

import math

import numpy as np


def depth_pixel_to_world_surface(centroid_xy:list[float], depth_m:float, width:int, height:int, fovy_degrees:float, camera_position_m:np.ndarray, world_from_camera:np.ndarray)->tuple[np.ndarray,np.ndarray]:
    u,v=centroid_xy;vertical=math.tan(math.radians(fovy_degrees)/2);horizontal=vertical*width/height;x=((u+0.5)/width*2-1)*horizontal;y=(1-(v+0.5)/height*2)*vertical;direction_camera=np.array([x,y,-1.0]);direction_camera/=np.linalg.norm(direction_camera);direction_world=world_from_camera@direction_camera
    if depth_m<=0 or not np.isfinite(depth_m):raise ValueError("depth must be finite and positive")
    return camera_position_m+depth_m*direction_world,direction_world
