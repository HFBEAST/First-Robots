# EXP-20261008-001: Dual-depth coordinate prerequisites in the contact scene

## Question and prerequisite

Before replacing known coordinates, can two fixed virtual depth sources observe the frozen 005/006 contact
fixture with an explicit metric-depth and optical-frame calibration contract? Formal 006 passed and its fresh
process trace equals 005. The corrected 027/028 work is methodological reference, not an inherited precision
claim: those runs used a different static cylinder location and scene. No hardware, control, detection or
center fitting is performed in this stage.

## Frozen conditions

Load config/sim_pick_place.json unchanged. Add only two fixed cameras at [0.45,-0.45,0.42] and
[0.45,0.45,0.42] m, directed towards the nominal task region [0.30,0,0.055] m, with 640x480 pixels and
60 degree vertical FOV. These are nominal virtual placements, not measured Astra extrinsics. Wrist RGB remains
in the model but is not used or evaluated. Let the cylinder settle for the existing two seconds under actuators;
compare the complete resulting qpos/qvel against the no-added-camera fixture to detect physics drift.

Development plane validation with the inherited offsamples=4 failed: fixed_depth_01 reconstructed five Z values
[0.000492684,0.000490361,0.000490906,0.000791150,0.000357105] m; maximum 0.000791150 m exceeded
the unchanged 0.00000420716 m numerical budget. A bounded isolation changed only offsamples to 0; Z became
[0.0000000875,-0.00000219894,-0.00000165455,0.00000267897,-0.00000108469] m, maximum
0.00000267897 m within the original budget rule. Freeze offsamples=0 for pixel-center depth acquisition and
reject other sampling configurations. This is a local renderer-setting observation, not proof of a universal
GPU fault or physical precision. Do not compensate with a fitted pixel offset or widen the numerical budget.
Formal evidence will be rerun from committed code; the console preflight itself is not a formal run.

MuJoCo's OpenGL camera uses right/up/backward axes. Export a conventional optical frame with right/down/forward
axes and a corresponding world transform, and depth as forward distance in metres. Camera intrinsics use
pixel-center coordinates, fx=fy=height/(2*tan(fovy/2)), cx=(width-1)/2, cy=(height-1)/2.
Separate independent tests from internal projection round trips: render a plane-only calibration fixture using
the same cameras and compare reconstructed world Z against its known Z=0 surface at five fixed non-boundary
pixels per camera. This can detect a normalized-ray/planar-depth confusion that a self-consistent inverse cannot.

## Criteria and stop conditions

Both registered camera frames must be float32 metric arrays of the declared shape, finite, positive and
nonuniform. Compiled camera poses must match their declared extrinsics; transforms must be finite proper
rotations. Settled physics arrays must equal the no-added-camera fixture exactly. A frame/calibration ID,
version, dimensions or units mismatch must reject projection, as must nonpositive/invalid selected samples.

The plane-only reconstruction residual must fit the predeclared numerical renderer budget
32 * float32 epsilon * max(1 m, sampled depth); this is an engineering round-off check, not a physical sensing
accuracy threshold. Record each pixel, depth, world point, height residual and budget. Pixel→world→pixel/depth
round trips use double-precision numerical checks. Any violated premise stops center-estimation and execution
work; preserve failure, isolate only its cause, do not widen the numerical budget after viewing results.

## Evidence and scope

Commit plan/config/pre-run record and implementation before a clean-checkout formal run. Save two raw depth
frames and context RGB images, plane fixture frames, calibration JSON, settled state, per-test observations,
complete input/model/code hashes and actual Python/package versions captured at execution start.
Null seed is the explicit deterministic exception: no random sampling is used. Run relevant behavior tests;
do not re-run retired branches or rewrite their manifests.

Primary reference checked 2026-10-08: [MuJoCo camera/output definition](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera).
The same reference documents offsamples=4 as the default and 0 as disabling offscreen multisampling.
Successful unit/coordinate evidence does not prove the cylinder is detectable, its center is recovered, a
camera drives grasping, RGB fusion helps, physical calibration is valid or the real task is safe.
