# Dexterous-Hand Body Initialization

This guide covers initializing and operating dexterous hands and grippers that
come from the upgraded `e-urdf-zoo` asset library.

## Quick start

```bash
# Search available hands
rosclaw eurdf search hand

# Pull an asset into the local cache (optional)
rosclaw eurdf pull dexhands/inspire_hand/right \
  --zoo-path /home/ubuntu/rosclaw/rosclaw/e-urdf-zoo/robots

# Initialize a body instance
rosclaw body init --robot dexhands/inspire_hand/right --name inspire-right-test

# Inspect the generated body manual
rosclaw body show
```

## What is different about imported hands

All dex-urdf imported assets are loaded with a conservative safety envelope:

| Policy | Default value |
|--------|---------------|
| `status` | `experimental` |
| `real_robot_execution_allowed` | `false` |
| `sandbox_required` | `true` |
| `current_limit_required` | `true` |
| `calibration_required` | `true` |

The effective body compiler enforces these defaults automatically:

- Manipulation and gesture capabilities are **degraded** to simulation-only
  until `real_robot_execution_allowed` is explicitly set to `true`.
- `ok_gesture` and `countdown_gesture` are **degraded** until calibration is
  `validated`.
- `fast_full_close` and `forceful_grasp_without_current_limit` are **blocked**
  by the safety invariant engine.

## Agent queries

You can ask the body natural-language safety questions:

```bash
rosclaw body query "Can this hand close fast?"
rosclaw body query "Can this hand do OK gesture on real hardware?"
rosclaw body query "Can this hand perform countdown gesture 5-4-3-2-1?"
rosclaw body query "Can I forcefully grasp an object?"
```

Expected answers for an unvalidated experimental hand:

| Question | Answer |
|----------|--------|
| Close fast? | **Blocked** — `fast_full_close` is a critical forbidden capability. |
| OK gesture on real hardware? | **Blocked** — real-robot execution is not allowed and calibration is not validated. |
| Countdown gesture? | **Sandbox first** — run in simulation and validate each pose before real hardware. |
| Forceful grasp? | **Blocked** — active current/torque limits are required. |

## Promoting a hand to real-robot use

Do not change `real_robot_execution_allowed` until the following steps are
completed and recorded in `maintenance.log`:

1. Complete sandbox-only validation.
2. Upload clearance calibration to `calibration.yaml`.
3. Run a low-speed range-of-motion check.
4. Enable current/torque monitors.
5. Obtain human-in-the-loop confirmation per pose.

After the steps are recorded, update the body policy and re-render:

```bash
rosclaw body safety-override --set real_robot_execution_allowed=true  # example
rosclaw body render
rosclaw body validate
```

## End-effector only

Imported dexterous hands are end-effectors. They have no arm, torso, or base.
The rendered `EMBODIMENT.md` explicitly warns that real-robot execution is
disabled for the asset.
