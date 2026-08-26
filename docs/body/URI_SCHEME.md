# ROSClaw URI scheme for body references

`rosclaw://` URIs provide stable, cross-module references to body and e-URDF
resources. They are parsed by `rosclaw.body.references.RosclawURI` and resolved
by `rosclaw.body.resolver.BodyResolver.resolve()`.

## Grammar

```
rosclaw://{resource_type}/{path}[@{version}][/{qualifier}]
```

- `resource_type` — `body` or `eurdf` in the current implementation.
- `path` — identifier or `current` for the linked body instance.
- `version` — optional version tag, attached to the path segment (e.g. `unitree-g1@1.0.0`).
- `qualifier` — optional sub-resource selector.

## Body URIs

| URI | Resolved value | Notes |
|-----|----------------|-------|
| `rosclaw://body/current` | `BodyYaml` of the current/selected body instance. | Raises `BodyNotLinkedError` if no body is linked. |
| `rosclaw://body/current/effective` | `EffectiveBody` compiled from all sources. | Recompiles if stale. |
| `rosclaw://body/current/calibration` | `CalibrationYaml` or an empty one. | Returns default if file is absent. |
| `rosclaw://body/current/maintenance` | `list[MaintenanceEvent]` | Empty list if no log exists. |
| `rosclaw://body/current/capabilities` | `dict[str, list[str]]` with `enabled`, `degraded`, `blocked`. | Shortcut to `effective.capabilities`. |
| `rosclaw://body/{instance_id}` | `BodyYaml` for the named instance. | P0 primarily supports `current`. |

## e-URDF URIs

| URI | Resolved value | Notes |
|-----|----------------|-------|
| `rosclaw://eurdf/{profile_id}` | `EurdfProfile` for the profile. | Version inferred from lock/registry if omitted. |
| `rosclaw://eurdf/{profile_id}@{version}` | `EurdfProfile` pinned to version. | Used in `model_ref.eurdf_uri` of `body.yaml`. |

## Examples

```python
from rosclaw.body.resolver import BodyResolver

resolver = BodyResolver()
effective = resolver.resolve("rosclaw://body/current/effective")
caps = resolver.resolve("rosclaw://body/current/capabilities")
profile = resolver.resolve("rosclaw://eurdf/unitree-g1@1.0.0")
```

## Extending the scheme

New resource types or qualifiers should be added in `references.py` and
`resolver.resolve()`. Keep the rule that URIs are read-only references:
mutating body state goes through `BodyResolver.update_body_yaml()` or the CLI,
never by writing to a URI.
