"""Known upright-cylinder center fit for one virtual depth golden case only."""
from __future__ import annotations

import math

import numpy as np


def foreground_pixels_to_world_points(depth:np.ndarray, mask:np.ndarray, width:int, height:int, fovy_degrees:float, camera_position_m:np.ndarray, world_from_camera:np.ndarray)->np.ndarray:
    rows,columns=np.nonzero(mask);vertical=math.tan(math.radians(fovy_degrees)/2);horizontal=vertical*width/height;x=((columns+0.5)/width*2-1)*horizontal;y=(1-(rows+0.5)/height*2)*vertical;d=depth[rows,columns];local=np.column_stack((x*d,y*d,-d));return camera_position_m+local@world_from_camera.T


def fit_xy_circle(points:np.ndarray)->tuple[np.ndarray,float,float]:
    xy=points[:,:2];matrix=np.column_stack((2*xy[:,0],2*xy[:,1],np.ones(len(xy))));target=np.sum(xy**2,axis=1);cx,cy,constant=np.linalg.lstsq(matrix,target,rcond=None)[0];radius=float(math.sqrt(max(0.0,constant+cx*cx+cy*cy)));residual=float(np.sqrt(np.mean((np.linalg.norm(xy-np.array([cx,cy]),axis=1)-radius)**2)));return np.array([cx,cy]),radius,residual
