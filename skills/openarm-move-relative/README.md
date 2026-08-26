# OpenArm Bounded Relative Motion

## What it does

Moves one selected OpenArm end effector by a bounded Cartesian translation in
the `world` frame through the official ROSClaw Runtime boundary.

## Supported robots

- `openarm_dual_mujoco_01`

## Required sensors

- `robot_state`

## Required providers

- None. Parameter normalization is deterministic and the MuJoCo executor uses
  a programmed damped-least-squares Jacobian controller.
- `providers.yaml` is retained because it is required by the official Skill
  package schema.

## Safety constraints

- See `safety.yaml`
- Default runtime mode: `sandbox_first`

## How to run

```bash
rosclaw skill validate . --json
rosclaw skill eval . --mode sandbox --json
```

## Evaluation evidence

The official VM acceptance passed with real MuJoCo physics, a `COMPLETED`
receipt, `TASK_VERIFIED` evidence, integrity verification, and final error below
0.5 mm. See `evidence/reports/` for the recorded acceptance summary.

## Version history

## 0.1.0

- Migrated from the OpenArm integration into the official ROSClaw Skill package format.

## Known limitations

- Simulation only; never use the package as REAL-robot evidence.
- The package remains at draft stage until a fresh `rosclaw skill eval` report
  and package hash lock are generated after installation in the official VM.
