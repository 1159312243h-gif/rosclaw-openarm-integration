# ROSClaw Event Topic Standardization

## Overview

ROSClaw v1.0 introduces a standardized event topic namespace: all topics are prefixed with `rosclaw.` and follow the hierarchical pattern:

```
rosclaw.<module>.<event>
```

This ensures:
- Clear ownership: every topic belongs to a specific module
- No collisions: third-party extensions cannot clash with core topics
- IDE autocomplete: `EventTopics.AGENT_COMMAND` instead of raw strings
- Easy refactoring: change in one place, propagate everywhere

## Canonical Topic Reference

| Constant | Canonical Topic | Description |
|----------|----------------|-------------|
| `EventTopics.RUNTIME_STARTED` | `rosclaw.runtime.started` | Runtime initialization complete |
| `EventTopics.RUNTIME_STOPPED` | `rosclaw.runtime.stopped` | Runtime shutdown initiated |
| `EventTopics.RUNTIME_STATUS` | `rosclaw.runtime.status` | Periodic heartbeat/status |
| `EventTopics.AGENT_COMMAND` | `rosclaw.agent.command` | Agent instruction (natural language or structured) |
| `EventTopics.AGENT_RESPONSE` | `rosclaw.agent.response` | Agent reply / action plan |
| `EventTopics.AGENT_CAPABILITY_REQUEST` | `rosclaw.agent.capability.request` | Request a capability from MCPHub |
| `EventTopics.SKILL_EXECUTION_START` | `rosclaw.skill.execution.start` | Skill execution begins |
| `EventTopics.SKILL_EXECUTION_COMPLETE` | `rosclaw.skill.execution.complete` | Skill execution finished |
| `EventTopics.PROVIDER_INFERENCE_REQUESTED` | `rosclaw.provider.inference.requested` | LLM/provider inference call |
| `EventTopics.PROVIDER_INFERENCE_COMPLETED` | `rosclaw.provider.inference.completed` | LLM/provider result ready |
| `EventTopics.SANDBOX_EPISODE_STARTED` | `rosclaw.sandbox.episode.started` | Physics sandbox episode begins |
| `EventTopics.SANDBOX_EPISODE_FINISHED` | `rosclaw.sandbox.episode.finished` | Physics sandbox episode ends |
| `EventTopics.SANDBOX_ACTION_ALLOWED` | `rosclaw.sandbox.action.allowed` | Firewall approved action |
| `EventTopics.SANDBOX_ACTION_BLOCKED` | `rosclaw.sandbox.action.blocked` | Firewall rejected action |
| `EventTopics.PRAXIS_COMPLETED` | `rosclaw.praxis.completed` | Successful practice episode |
| `EventTopics.PRAXIS_FAILED` | `rosclaw.praxis.failed` | Failed practice episode |
| `EventTopics.PRAXIS_RECORDED` | `rosclaw.praxis.recorded` | Episode persisted to storage |
| `EventTopics.PRACTICE_EVENT_CREATED` | `rosclaw.practice.event.created` | Generic practice event |
| `EventTopics.SAFETY_VIOLATION` | `rosclaw.safety.violation` | Joint limit / collision detected |
| `EventTopics.ROBOT_EMERGENCY_STOP` | `rosclaw.robot.emergency_stop` | Hard emergency stop triggered |
| `EventTopics.MEMORY_EXPERIENCE_STORED` | `rosclaw.memory.experience.stored` | Experience written to memory |
| `EventTopics.MEMORY_WRITE_COMPLETED` | `rosclaw.memory.write.completed` | Async write acknowledged |
| `EventTopics.HOW_RECOVERY_HINT_GENERATED` | `rosclaw.how.recovery_hint.generated` | Heuristic recovery suggestion |
| `EventTopics.HOW_RECOVERY_EXECUTED` | `rosclaw.how.recovery_executed` | Recovery action performed |
| `EventTopics.CRITIC_SUCCESS_DETECTED` | `rosclaw.critic.success.detected` | Critic detected success state |
| `EventTopics.CRITIC_JUDGMENT` | `rosclaw.critic.judgment` | Critic evaluation result |
| `EventTopics.DASHBOARD_TRACE_UPDATED` | `rosclaw.dashboard.trace.updated` | Trace visualization update |
| `EventTopics.ROBOT_TELEMETRY` | `rosclaw.robot.telemetry` | Periodic sensor telemetry |
| `EventTopics.ROBOT_JOINT_STATES` | `rosclaw.robot.joint_states` | Joint position/velocity/effort |

## Backward Compatibility

Old topic names continue to work. The EventBus automatically normalizes them at subscription and publication time.

```python
from rosclaw.core.event_bus import EventBus, Event
from rosclaw.core.event_topics import EventTopics

bus = EventBus()

# All three of these subscribe to the SAME topic:
bus.subscribe("agent.command", handler)                    # old name → normalized
bus.subscribe("rosclaw.agent.command", handler)             # already canonical
bus.subscribe(EventTopics.AGENT_COMMAND, handler)           # constant

# Publication also normalizes internally for matching:
bus.publish(Event(topic="agent.command", payload={...}))    # matches all above
bus.publish(Event(topic="rosclaw.agent.command", payload={...}))  # same
```

The `event.topic` field is **never mutated** — the original string is preserved for consumers that inspect it. Only internal subscriber matching uses the normalized form.

### Legacy Mapping Table

| Old Topic | Canonical Topic |
|-----------|----------------|
| `agent.command` | `rosclaw.agent.command` |
| `skill.execution.start` | `rosclaw.skill.execution.start` |
| `skill.execution.complete` | `rosclaw.skill.execution.complete` |
| `praxis.completed` | `rosclaw.praxis.completed` |
| `praxis.failed` | `rosclaw.praxis.failed` |
| `praxis.recorded` | `rosclaw.praxis.recorded` |
| `firewall.action_blocked` | `rosclaw.sandbox.action.blocked` |
| `safety.violation` | `rosclaw.safety.violation` |
| `agent.response` | `rosclaw.agent.response` |
| `agent.capability.request` | `rosclaw.agent.capability.request` |
| `robot.emergency_stop` | `rosclaw.robot.emergency_stop` |
| `memory.experience.stored` | `rosclaw.memory.experience.stored` |
| `rosclaw.memory.write.completed` | `rosclaw.memory.write.completed` |
| `heuristic.recovery_suggested` | `rosclaw.how.recovery_hint.generated` |
| `heuristic.recovery_executed` | `rosclaw.how.recovery_executed` |
| `runtime.status` | `rosclaw.runtime.status` |

## Wildcard Subscriptions

The EventBus supports MQTT-style `#` (match all) and glob-style `*` wildcards:

```python
# Subscribe to ALL safety-related events
bus.subscribe("rosclaw.safety.*", on_safety_event)

# Subscribe to EVERYTHING
bus.subscribe("#", on_any_event)

# Subscribe to all praxis outcomes
bus.subscribe("rosclaw.praxis.*", on_praxis)
```

## Trace IDs and Distributed Tracing

Every Event carries a `trace_id` for correlation across the pipeline. If not provided, the EventBus auto-injects one derived from `request_id`, `correlation_id`, or `episode_id` in the payload, falling back to a generated UUID.

```python
event = Event(
    topic=EventTopics.SKILL_EXECUTION_START,
    payload={"episode_id": "ep_001", "skill": "reach"},
)
# event.trace_id == "ep_001"  (inherited from payload)

derived = event.derive(topic=EventTopics.PRAXIS_COMPLETED)
# derived.trace_id == "ep_001"  (preserved across pipeline stages)
```

## Best Practices

1. **Use constants**: `EventTopics.AGENT_COMMAND` instead of raw strings
2. **Derive events**: use `event.derive(topic=...)` to preserve `trace_id`
3. **Subscribe with wildcards**: group related topics with `rosclaw.sandbox.*`
4. **Never rely on `event.topic` for logic**: the original string may be legacy or canonical — use payload fields instead

## Implementation Details

- `EventBus._normalize_topics` defaults to `True`; set to `False` for strict legacy mode
- `normalize_topic()` is defined in `rosclaw.core.event_topics` with `_TOPIC_COMPAT` mapping
- Unknown topics (not in `_TOPIC_COMPAT` and not starting with `rosclaw.`) pass through unchanged
