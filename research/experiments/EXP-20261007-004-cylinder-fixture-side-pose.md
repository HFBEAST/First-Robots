# EXP-20261007-004: Free-cylinder fixture and side-grasp poses

## Question and prerequisite

After the formally validated actuator-motion baseline 003, can an idealized free cylinder settle on the floor under
gravity, and can the five arm axes satisfy the planned side-grasp positions with a horizontal approach direction?
This bundles the physical fixture and side-pose prerequisites for the first actual grasp attempt.

## Controls and engineering adjustment

Use unchanged runtime model assets, a synthetic 50 g upright cylinder (radius 35 mm, height 110 mm), collision and
gravity enabled, and fixed known world positions. The cylinder begins 5 mm above its nominal support height.
Hold the six robot joints with the original position actuators during a two-second fixture simulation.
Fix wrist roll at -pi/2 to orient the closing direction sideways and solve XYZ plus horizontal approach using the
four other arm axes; do not impose an arbitrary six-DOF end-effector pose on a five-axis arm.

Bounded development preflight at site `[0.25,0.015,0.070] m` exhausted improving limited steps after 23 iterations:
position residual 0.00959968 m, approach Z component -0.108804, and wrist-flex at its lower limit. The grasp site
`[0.30,0.015,0.070] m` converged. Retain that failed preflight as evidence of this solver/seed/configuration;
it does not prove global infeasibility. A 30 mm retreat at X=0.27 m also failed at the wrist-flex limit after
22 iterations (position residual 0.00499725 m). A 10 mm retreat at X=0.29 m failed after 21 iterations
(position residual 0.00148265 m); an alternate seed at X=0.25 m failed at other limits. These bounded exploratory
checks motivate a path adjustment: keep horizontal gripper approach/sideways closure, but descend from an elevated
site `[0.30,0.015,0.160] m` into `[0.30,0.015,0.070] m`. This retains side clamping while removing the failed
low horizontal-retreat waypoint. Freeze this revised pair for the formal run; trajectory clearance is still untested.

## Criteria

The declared free cylinder must exist with positive mass and contact enabled. All simulated states must be finite;
the cylinder must move downward from its initial elevated position and have positive normal support force at the
end. Record final position and velocity, rather than inventing a physical stability/accuracy threshold.
Both named side poses must converge within the declared numerical solver precision and model/actuator limits.
Failure blocks grasp execution; preserve all outputs and use an isolation test if needed. This stage does not
close the gripper, lift or transport the object, or establish hardware feasibility.

## Artifacts

Record fixture trajectory, per-pose numerical residuals, a rendered final scene, complete input hashes, runtime
versions and immutable manifest/result/record. Run from committed code in a clean checkout. No stochastic input
is used; null seed is the same explicit deterministic-protocol exception as 003.
