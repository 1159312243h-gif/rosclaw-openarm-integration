# e-URDF-Zoo Integration

ROSClaw can now consume manifest-driven assets from the upgraded
`e-urdf-zoo` library. This lets you initialize a body directly from a
structured asset bundle such as `dexhands/inspire_hand/right`.

## What changed

- New `rosclaw.eurdf.zoo_client.EurdfZooClient` loads manifest-driven
  assets and converts them into the `RobotCompleteProfile` /
  `EurdfProfile` objects the body module already understands.
- New `rosclaw eurdf` CLI commands:
  - `rosclaw eurdf search <query>`
  - `rosclaw eurdf info <asset_id>`
  - `rosclaw eurdf validate <asset_id>`
  - `rosclaw eurdf pull <asset_id>`
  - `rosclaw eurdf cache list`
- `rosclaw body init --robot <asset_id>` automatically resolves
  slash-containing IDs through the zoo client.

## Resolution order

When a zoo asset ID is requested, `EurdfZooClient` resolves it in this
order:

1. Explicit `--source` path if given.
2. `~/.rosclaw/cache/e-urdf-zoo/<asset_id>/`
3. Project-root `e-urdf-zoo` checkout (or pip-installed package data).

## Initialize a body from a zoo asset

```bash
# Optional: pull the asset into the local cache first
rosclaw eurdf pull dexhands/inspire_hand/right \
  --zoo-path /home/ubuntu/rosclaw/rosclaw/e-urdf-zoo/robots

# Initialize the body
rosclaw body init --robot dexhands/inspire_hand/right \
  --name inspire-right-test
```

The resulting lock file records `source: zoo` and the body artifacts are
rendered as usual.

## Safety contract

All imported dexterous-hand and gripper assets are **experimental**:

- `real_robot_execution_allowed: false`
- `sandbox_required: true`
- Forbidden capabilities include `fast_full_close` and
  `forceful_grasp_without_current_limit`.

Do not promote an asset to `validated` until it passes the full sandbox
and calibration protocol.

See [DEX_HAND_BODY_INIT.md](DEX_HAND_BODY_INIT.md) for the dexterous-hand
operating guide and Agent-query examples.

## Adding a new asset

1. Import or author the asset under `e-urdf-zoo/robots/<category>/...`.
2. Run `e-urdf-zoo validate <asset_id>` until it passes.
3. Run `rosclaw eurdf info <asset_id>` to verify the conversion.
4. Run `rosclaw body init --robot <asset_id> --name <instance>` to test
   the body lifecycle.

## Backward compatibility

Legacy flat `e_urdf.json` assets continue to work through the existing
`RobotRegistry` path. Only IDs containing `/` (or explicit zoo flags)
trigger the new manifest-driven flow.
