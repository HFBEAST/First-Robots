"""Virtual depth-background foreground components; no calibration, pose, or hardware policy."""
from __future__ import annotations

from collections import deque

import numpy as np

from research.protocols.depth_frame_contract import DepthFrame


def positive_depth_foreground_components(reference: DepthFrame, current: DepthFrame, epsilon_m: float) -> tuple[np.ndarray, list[dict[str, object]]]:
    if reference.metadata() != current.metadata(): raise ValueError("DepthFrame metadata mismatch")
    mask = (reference.depth - current.depth) > epsilon_m; height, width = mask.shape; seen=np.zeros_like(mask,dtype=bool); components=[]
    for row,column in zip(*np.nonzero(mask)):
        if seen[row,column]: continue
        queue=deque([(int(row),int(column))]);seen[row,column]=True;pixels=[]
        while queue:
            y,x=queue.popleft();pixels.append((y,x))
            for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
                ny,nx=y+dy,x+dx
                if 0<=ny<height and 0<=nx<width and mask[ny,nx] and not seen[ny,nx]:seen[ny,nx]=True;queue.append((ny,nx))
        rows=[p[0] for p in pixels];cols=[p[1] for p in pixels];components.append({"pixel_count":len(pixels),"bbox_xywh":[min(cols),min(rows),max(cols)-min(cols)+1,max(rows)-min(rows)+1],"centroid_pixel_xy":[float(sum(cols)/len(cols)),float(sum(rows)/len(rows))]})
    return mask,sorted(components,key=lambda item:int(item["pixel_count"]),reverse=True)
