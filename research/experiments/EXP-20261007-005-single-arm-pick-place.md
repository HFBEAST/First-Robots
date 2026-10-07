# EXP-20261007-005: Known-coordinate contact pick-and-place

## Question and prerequisites

Can the fixed synthetic cylinder move from A to region B through actual MuJoCo position-actuator motion and
contact, without assigning its free-joint state after initialization? Motion baseline 003 and free-cylinder /
side-pose prerequisite 004 have passed their technical and provenance checks. Human capability adoption remains
pending; neither is evidence for hardware control. No camera, language model, obstacle or multi-robot input is added.

## Controls and development comparison

Retain the 004 model, cylinder dimensions/mass, world origin, A, solver constraints and high-to-low approach.
The high transport site is [0.30,0.115,0.160] m; the low release site is [0.30,0.115,0.070] m. B is a declared
synthetic circular placement region centered at [0.30,0.10] m with radius 0.07 m. The cylinder's full nominal
horizontal radius must fit inside it. This region is task geometry, not an adopted physical precision threshold.
Report the center error separately; being inside B does not mean accurate point placement.

An unrecorded console development preflight at closed command 0.5 rad completed the phases and observed about
43 N jaw normal force and 24.84 mm center error. It is not the formal result and has no stored trajectory.
Before freezing the formal configuration, perform one bounded development comparison at closed commands
[0.5,0.6,0.7,0.8] rad, varying only this command. Preserve every configuration, phase result and trajectory in
DEV-20261007-005-gripper-command-tuning, including failures. Prefer the largest tested command that still meets
the task postconditions, recording force and error rather than claiming a safe hardware force.
No further parameter search is authorized by this plan if all four fail: isolate the first failed phase instead.

The recorded development comparison completed at 0.5, 0.6 and 0.7 rad; 0.8 rad stopped after close because the
moving jaw had no contact. No lift or downstream phase was attempted for that failed condition. Freeze 0.7 rad
as the largest successful tested command, not an optimum. Its center error was 13.61 mm versus 24.84 mm at 0.5.
Maximum jaw normal forces were still 55.55/60.09 N; lower commanded closure is not proof of safe physical force.
The exploratory manifest explicitly records a dirty development checkout and is not a formal provenance pass.

## Criteria and stop conditions

Every actual step must be finite and within model/actuator joint limits. Sampled paths and actual states must
have no negative contacts except intended cylinder-floor support and cylinder-jaw contacts; mounting contacts
between bodies welded to world are excluded. Intended contacts use MuJoCo's compliant contact, not a claim of
zero interpenetration. There is no weld, attachment, teleport or object-pose correction during execution.

After close, lift and transfer, both jaw groups must have positive normal contact force. Lift and transfer
checkpoints must show the cylinder's geometric bottom above the support plane and zero floor support force.
Only after these prerequisites pass may the next phase execute. After release/retreat/settling, B must contain
the nominal cylinder radius; floor support must be positive and both jaw forces zero. Any failed numerical,
collision or postcondition gate stops downstream phases and is retained. Endpoint grip checks are not a proof
that continuous slip never occurred; record the complete force and pose trace for inspection.

## Evidence and limitations

Freeze the selected config and commit code, plan and pre-run record before formal execution in a clean checkout.
Save input hashes, complete qpos/qvel/ctrl, cylinder/site pose and force traces, phase checkpoints, GIF replay,
phase snapshots, result/record pair and run manifest. Repeat in a fresh process with an independent run ID;
compare numeric traces before extending to perception. No stochastic inputs are used; null seed is an explicit
deterministic-protocol exception. Judge failure visibly, not by the process exit alone.

The unchanged upstream gripper collision geometry, friction, contact compliance, position actuators and synthetic
mass are not calibrated hardware properties. Successful completion supports only this fixed virtual golden case.
Camera grounding, natural language, broad workspace robustness and real-arm feasibility remain untested.
