# e-URDF-Zoo / dex-urdf Upgrade Implementation Report

## Summary

This report documents the work to upgrade `e-urdf-zoo` from a flat legacy asset
layout into a structured, manifest-driven asset library and to wire the new
assets into ROSClaw.

## Repositories and scope

- `/home/ubuntu/rosclaw/rosclaw/e-urdf-zoo` — asset library and importer.
- `/home/ubuntu/rosclaw/rosclaw/rosclaw-v1.0` — ROSClaw runtime integration.
- `/home/ubuntu/rosclaw/rosclaw/dex-urdf` — source URDFs for bulk import.

## Delivered phases

### Phase 1 — Asset spec & schemas

Defined Pydantic schemas for:

- `manifest.yaml`
- `semantic.yaml`
- `capabilities.yaml`
- `safety.yaml`
- `providers.yaml`
- `sandbox.yaml`

Added schema-level validation tests and documentation.

### Phase 2 — Loader, API, CLI, index

Implemented:

- `AssetLoader` with backward-compatible legacy fallback.
- `EmbodimentAsset` exposing both new and legacy fields.
- `AssetIndex` and `AssetValidator`.
- `e-urdf-zoo` CLI: `list`, `info`, `validate`, `index build`, `import dex-urdf`.

### Phase 3 & 4 — dex-urdf importer and bulk import

Built the `dex_urdf.py` importer and generated assets under:

- `robots/dexhands/...`
- `robots/grippers/...`

Imported assets include:

- `dexhands/inspire_hand/right`
- `dexhands/ability_hand/left`
- `grippers/panda/default`

All imported assets default to `experimental`,
`real_robot_execution_allowed=false`, `sandbox_required=true`, and include
mandatory `safety.yaml` plus `forbidden_capabilities`.

### Phase 5 — ROSClaw `rosclaw eurdf` integration

Added to ROSClaw:

- `EurdfZooClient` — resolves, pulls, caches, and converts manifest assets.
- `rosclaw eurdf {search,info,validate,pull,cache list}` commands.
- `rosclaw body init --robot <asset_id>` auto-detects slash-containing IDs and
  resolves them through the zoo client.
- Local cache layout:
  `~/.rosclaw/cache/e-urdf-zoo/<asset_id>/`.

Tests:

- `tests/eurdf/test_zoo_client.py`
- `tests/eurdf/test_eurdf_cli.py`
- `tests/body/test_body_init_from_zoo.py`

### Phase 6 — Dexterous-hand safety & Agent queries

Hardened ROSClaw body safety for imported hands:

- `EurdfProfile.from_robot_complete_profile` now propagates
  `forbidden_capabilities`.
- `BodyInstanceService.create_or_init` records forbidden capabilities in
  `body.yaml` and sets `agent_policy.direct_real_robot_execution_allowed`.
- `EffectiveBodyCompiler._derive_capabilities`:
  - Degrades `ok_gesture` / `countdown_gesture` when calibration is not
    validated.
  - Degrades all manipulation capabilities to simulation-only when
    `real_robot_execution_allowed=false`.
- `SafetyInvariantEngine` now disables critical forbidden capabilities such as
  `fast_full_close` and `forceful_grasp_without_current_limit`.
- `BodyQueryEngine` answers hand-specific safety questions:
  - "Can this hand close fast?" → blocked.
  - "Can this hand do OK gesture on real hardware?" → blocked until clearance.
  - "Can this hand perform countdown gesture 5-4-3-2-1?" → sandbox first.
  - "Can I forcefully grasp an object?" → blocked without current limit.
- `EmbodimentRenderer` emits a prominent warning when real-robot execution is
  disabled.

Tests:

- `tests/body/test_dexhand_agent_safety.py`

## Verification results

```bash
pytest tests/eurdf tests/body -q
# 163 passed
```

## Backward compatibility

Legacy flat `e_urdf.json` assets continue to load through `RobotRegistry`.
Only asset IDs containing `/` (or explicit zoo flags) trigger the new
manifest-driven flow.

## Next recommended work

- Add CI job running the full eurdf + body suite.
- Expand dex-urdf importer coverage for additional hand families.
- Add ROS2 live smoke tests with sandbox-only validation.
- Promote the first validated hand asset from `experimental` to `validated`.
