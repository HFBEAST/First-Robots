# EXP-20260831-003-so101-mujoco-observation-preflight: Validation Failure

`EXP-20260831-003-so101-mujoco-observation-preflight.json` generated RGB/depth artifacts and a paired result,
but is not a valid formal run. `experiment-management validate` reported:

```text
formal run started from a dirty Git worktree
```

Cause: the runner created `research/artifacts/EXP-20260831-003-so101-mujoco-observation-preflight/` before it
instantiated `RunRecorder`; the recorder consequently captured the run's own new artifacts as pre-existing dirty
working-tree content. The manifest, result and artifacts are retained unchanged for audit. A corrected runner uses
a new run ID and captures Git metadata before it creates output evidence.
