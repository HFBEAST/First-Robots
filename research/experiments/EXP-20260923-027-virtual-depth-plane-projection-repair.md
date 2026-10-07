# EXP-20260923-027-virtual-depth-plane-projection-repair: Experiment Plan

026 used the correct foreground data but an incorrect projection premise. MuJoCo’s official camera XML reference
defines `depth` as distance from the camera plane, whereas `distance` is distance from camera origin. This run retains
the same 025 arrays, components, camera contract and B, but maps a pixel normalized direction `[x,y,-1]` with planar
depth `d` to camera-local `[x*d,y*d,-d]`, rather than multiplying a normalized ray by `d`.

The output remains a visible surface-point candidate. It is not fitted or named as cup center; no tolerance, fusion,
Coordinator or execution follows from this correction.
