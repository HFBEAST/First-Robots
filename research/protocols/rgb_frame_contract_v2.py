"""Version-two virtual RGB fixture detector with explicit channel dominance and components."""
from __future__ import annotations

from collections import deque

import numpy as np

from research.protocols.rgb_frame_contract import RgbFrame


def candidate_magenta_components(frame: RgbFrame, detector: dict[str, int]) -> tuple[np.ndarray, list[dict[str, object]]]:
    if frame.pixel_encoding != "rgb_uint8" or frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3:
        raise ValueError("RGB frame contract violated")
    rgb = frame.rgb.astype(np.int16); red, green, blue = rgb[...,0], rgb[...,1], rgb[...,2]
    mask = (red >= detector["red_min"]) & (green <= detector["green_max"]) & (blue >= detector["blue_min"]) & ((red-green) >= detector["red_minus_green_min"]) & ((blue-green) >= detector["blue_minus_green_min"])
    height, width = mask.shape; seen = np.zeros_like(mask, dtype=bool); components = []
    for row, column in zip(*np.nonzero(mask)):
        if seen[row, column]: continue
        queue = deque([(int(row), int(column))]); seen[row, column] = True; pixels = []
        while queue:
            y, x = queue.popleft(); pixels.append((y, x))
            for dy, dx in ((1,0),(-1,0),(0,1),(0,-1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True; queue.append((ny, nx))
        rows = [item[0] for item in pixels]; columns = [item[1] for item in pixels]
        components.append({"pixel_count":len(pixels),"bbox_xywh":[min(columns),min(rows),max(columns)-min(columns)+1,max(rows)-min(rows)+1],"centroid_pixel_xy":[float(sum(columns)/len(columns)),float(sum(rows)/len(rows))]})
    return mask, sorted(components, key=lambda item: int(item["pixel_count"]), reverse=True)
