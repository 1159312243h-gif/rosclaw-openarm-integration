# Skill compatibility with the Effective Body

The body module answers: *"Given the robot's current physical state, can this
skill run safely and correctly?"* Each skill declares its hardware
requirements in a `.skill.yaml` manifest. The compatibility checker compares
the manifest against the `EffectiveBody` and produces one of four statuses.

## Compatibility statuses

| Status | Meaning | Execution behavior |
|--------|---------|--------------------|
| `compatible` | All requirements satisfied. | Allowed. |
| `degraded` | Requirements satisfied but with warnings (e.g. degraded sensor, uncalibrated camera). | Allowed only if the skill manifest explicitly accepts the degradation via `degradation_policy`. Otherwise treated as `blocked`. |
| `blocked` | A hard requirement is missing or a capability is disabled. | Refused by `SkillExecutor`. |
| `unknown` | No skill manifest was found or the check could not be completed. | **Fail-closed**: refused by `SkillExecutor`. |

## Manifest requirements

A skill manifest specifies what the skill needs from the body:

```yaml
skill_id: dual_arm_lift
skill_version: "1.0.0"
requires:
  robot_class: humanoid
  eurdf:
    compatible_profiles: [unitree-g1]
  capabilities:
    all_of: [dual_arm_coordination, lift_object]
  actuators:
    all_of:
      - group: left_arm_actuator_group
        status: available
      - group: right_arm_actuator_group
        status: available
  sensors:
    all_of:
      - id: head_rgb_camera
        status: available
  calibration:
    required_status: [validated]
  safety:
    max_base_speed_mps_at_least: 0.5
degradation_policy:
  allow_uncalibrated_camera: false
  allow_lower_speed: false
```

The checker validates, in order:

1. Robot class and e-URDF profile compatibility.
2. Required capabilities are not `blocked` (degraded capabilities emit a warning).
3. Required sensors and actuators exist and are in the requested status.
4. Required frames exist.
5. Calibration status meets the skill's threshold.
6. Safety limits (e.g. max base speed) are sufficient.

## Impact-aware incremental recheck

After a small body change (e.g. one sensor becomes unavailable), re-checking
every skill is wasteful. `SkillCompatibilityChecker.check_incremental()` only
re-checks skills whose `requirement_ids()` intersect the set of IDs affected by
the change. Unaffected skills are copied from the previous compatibility
report, with their `checked_against` metadata updated to the new body hash.

Affected IDs are produced by `BodyDiffer` categories such as `sensor_status`,
`actuator_status`, `capability`, `structural`, `safety`, and `incident`.

## Fail-closed enforcement in `SkillExecutor`

`SkillExecutor._check_body_compatibility()` runs before any skill execution:

- If no body is linked, execution proceeds for backward compatibility.
- If the body is linked and the skill is `blocked` or `unknown`, execution is
  blocked.
- If `BodyResolver` raises any exception, execution is blocked.

This is a safety invariant: do not swallow resolver errors and return `"ok"`.

## Generating a compatibility report

CLI:

```bash
rosclaw body inspect --skills
```

Programmatically:

```python
from rosclaw.body.resolver import BodyResolver

resolver = BodyResolver()
report = resolver.get_skill_compatibility()
for key, result in report.skills.items():
    print(key, result.status, result.reason)
```

## Files

- `src/rosclaw/body/compatibility.py` — checker and store.
- `src/rosclaw/body/schema.py` — `SkillManifest`, `SkillCompatibilityResult`,
  `SkillCompatibilityReport`.
- `src/rosclaw/skill_manager/executor.py` — runtime enforcement.
