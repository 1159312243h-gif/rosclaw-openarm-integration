# `rosclaw.body` — Physical AI Body Runtime layer

This module implements ROSClaw's three-layer body model:

| Layer | Concept | Responsibility |
|-------|---------|----------------|
| **e-URDF** | `EurdfProfile` | Physical DNA — model-level joints, sensors, actuators, capability hints, safety limits. |
| **body.yaml** | `BodyYaml` | Body Instance Ledger — per-robot installation state, overrides, notes, identity. |
| **EMBODIMENT.md** | rendered output | Agent-readable compiled manual from the Effective Body Model. |

The module is intentionally self-contained under `src/rosclaw/body/`. Other
parts of the codebase should interact with it only through the public API
or `rosclaw://` URIs.

## Public API

```python
from rosclaw.body import BodyResolver, EffectiveBodyCompiler, SkillCompatibilityChecker
from rosclaw.body.resolver import BodyNotLinkedError

resolver = BodyResolver()
if resolver.is_linked():
    body = resolver.get_effective_body()
    print(body.effective_body_hash)
```

Main entry points:

- `BodyResolver` — load body state, compile effective body, resolve URIs,
  discover skill manifests, render artifacts.
- `EffectiveBodyCompiler` — merge e-URDF + body.yaml + calibration + maintenance
  into an `EffectiveBody`.
- `SkillCompatibilityChecker` — classify skills as `compatible`, `degraded`,
  `blocked`, or `unknown` against the current effective body.
- `EmbodimentRenderer` — render `EMBODIMENT.md` and preserve human notes.
- `BodyDiffer` — detect changes between body states and decide whether skills
  need rechecking.

## CLI

```bash
# Link a robot model to the current workspace
rosclaw body link-eurdf unitree-g1 [--version 1.0.0]

# Inspect the compiled body
rosclaw body inspect [--json --agent --capabilities --components --skills]

# Diff against the e-URDF base or a snapshot
rosclaw body diff [--against eurdf|snapshot:PATH] [--format json]

# Update instance state with an audit trail
rosclaw body update-state --set installed_components.sensors.head_rgb_camera.status=unavailable --reason "cable unplugged"

# Append a maintenance/incident note
rosclaw body note "Right arm overheated" --type incident --severity warning --affects right_arm_actuator_group
```

## Cross-module rules

- Read body state through `BodyResolver`, never by opening `~/.rosclaw/body/body.yaml`
  directly.
- Use `rosclaw://body/current/effective` or `rosclaw://eurdf/{id}@{version}` URIs
  when passing body references across module boundaries.
- Body capabilities can only be **restricted** by instance state, never expanded
  beyond what the e-URDF permits.
- `SkillExecutor` enforces fail-closed compatibility checks: `unknown` status or
  resolver errors block execution.

## Files in the workspace

```
~/.rosclaw/body/
├── body.yaml                  # instance ledger
├── calibration.yaml           # offsets/extrinsics
├── maintenance.log            # JSONL audit trail
├── EMBODIMENT.md              # rendered body manual
├── skill_compatibility.yaml   # cached compatibility report
├── refs/
│   ├── eurdf.lock             # pinned e-URDF reference
│   ├── eurdf.profile.yaml     # normalized profile
│   └── effective_body.json    # compiled effective body
├── snapshots/                 # historical body states
└── generated/                 # machine-readable summaries
```

## Tests

Run body module tests:

```bash
pytest tests/body -q
```

Run skill-manager + CLI regression tests:

```bash
pytest tests/test_skill_manager.py tests/test_cli.py tests/test_cli_coverage.py -q
```
