# EXP-20261007-001: Fixed-target actuator-driven SO-101 motion

## Question

Can the current virtual SO-101 move its gripperframe from a declared initial joint state toward the fixed world target
`[0.25, 0, 0.20] m` using bounded deterministic inverse kinematics and MuJoCo position actuators?
This is the first motion prerequisite for a later fixed-cylinder pick-and-place experiment.

## Inputs and controls

- Model: unchanged Menagerie `robotstudio_so101`, revision `da76818e269b82289eba39808e2fb91d679d6994`, Apache-2.0,
  independently vendored under `assets/so101`; each input file is hashed in the run.
- Runtime configuration: `config/sim_reach.json`; known target is an explicit control-baseline input, not perception.
- Fixed world floor, gravity, model joint limits, position actuators, initial state, and gripper opening.
- Position-only Jacobian IK with bounded iterations and step sizes; no orientation or grasp constraint yet.
- Check the sampled joint path before execution and actual simulated state after every physics step for penetrating
  contacts between moving bodies and joint limit violations. These discrete checks do not prove continuous clearance.

## Criteria and stop conditions

The solver must converge to its declared numerical precision, the position-actuator simulation must complete, all
recorded states must be finite and within model limits, no checked moving-body penetration may occur, and final
position error must be lower than initial error. Record the final error without treating it as an accepted robot
accuracy threshold. Numerical solver settings are provisional engineering settings, not adopted hardware limits.
Any IK exhaustion, sampled collision, actual collision, limit violation, or nonfinite state is an explicit failure.

## Artifacts and scope

Save configuration, metrics, every-step joint/control/site trajectory, rendered GIF replay, run manifest, result,
and execution record. Prebuilt plan records and execution records have separate paths. Failed runs are retained;
new formal configurations use a new run ID. No object grasp, language understanding, perception, hardware control,
or robot-capability adoption is tested by this motion experiment.

An independent process repeats this exact condition as `EXP-20261007-002-single-arm-reach-process-repeat`.
It hashes the predecessor result and trajectory, reruns the physics, and requires every recorded trajectory array
to equal its predecessor on this frozen host/software configuration. This is reproducibility evidence, not generalization.

Formal validation rejected 001/002 because the worktree was dirty at their start. Their observations remain unchanged.
`EXP-20261007-003-single-arm-reach-committed-baseline` reruns the same condition from committed code in a clean checkout,
and is the candidate formal baseline. No random sampling or randomized simulator inputs occur; the null seed is an
explicit deterministic-protocol exception. Code changes and numerical adjustments are recorded before each formal run.
