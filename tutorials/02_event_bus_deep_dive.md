# ROSClaw v1.0 — EventBus Deep Dive

> **Prerequisites**: `01_getting_started.md`  
> **Time**: ~30 minutes  
> **Difficulty**: Beginner

---

## What You'll Learn

- Async subscribers and error handling
- Event history queries and filtering
- `await_event()` for synchronous waiting
- Priority-based processing
- Command/event separation pattern

---

## Overview

The EventBus is ROSClaw's central nervous system. Every module communicates through it using publish/subscribe patterns. This tutorial dives deeper than the getting-started overview, covering real-world patterns you'll use daily.

---

## Step 1: Async Subscribers with Error Handling

Sync subscribers block the publisher. For I/O-bound work, use async subscribers:

```python
#!/usr/bin/env python3
"""Tutorial 02: Async subscribers with error handling."""

import asyncio
from rosclaw.core import EventBus, Event


async def async_handler(event: Event):
    """Async subscriber that processes events."""
    try:
        print(f"  Processing: {event.payload}")
        await asyncio.sleep(0.05)  # Simulate work
        print(f"  Done: {event.payload}")
    except Exception as e:
        print(f"  Error: {e}")


async def main():
    bus = EventBus()
    bus.subscribe_async("tutorial.topic", async_handler)

    for i in range(3):
        bus.publish(Event(
            topic="tutorial.topic",
            payload=f"Event {i}",
            source="tutorial",
        ))

    await asyncio.sleep(0.3)


if __name__ == "__main__":
    asyncio.run(main())
```

**Expected output**:

```
  Processing: Event 0
  Processing: Event 1
  Processing: Event 2
  Done: Event 0
  Done: Event 1
  Done: Event 2
```

**Key point**: Async subscribers run concurrently. Three events are published instantly, then all three process in parallel.

---

## Step 2: Event History and Filtering

The EventBus keeps the last 10,000 events. Query them for debugging or replay:

```python
#!/usr/bin/env python3
"""Tutorial 02: Event history and filtering."""

from rosclaw.core import EventBus, Event

bus = EventBus()

# Publish some events
for i in range(5):
    bus.publish(Event(topic="sensor.temperature", payload={"val": 20 + i}, source="sensor"))
    bus.publish(Event(topic="sensor.pressure",    payload={"val": 100 + i}, source="sensor"))

# Get all recent events
all_events = bus.get_history(limit=10)
print(f"Total recent events: {len(all_events)}")

# Filter by topic
temp_events = bus.get_history(topic="sensor.temperature", limit=3)
print(f"Temperature events: {len(temp_events)}")
print(f"Latest temp: {temp_events[-1].payload}")

# Clear temperature history
bus.clear_history(topic="sensor.temperature")
print(f"After clear, all events: {len(bus.get_history())}")
```

**Expected output**:

```
Total recent events: 10
Temperature events: 3
Latest temp: {'val': 22}
After clear, all events: 5
```

---

## Step 3: Waiting for Events with `await_event()`

Use `await_event()` to block until a specific event arrives. Great for request-response patterns:

```python
#!/usr/bin/env python3
"""Tutorial 02: await_event for synchronization."""

import asyncio
from rosclaw.core import EventBus, Event


async def delayed_publisher(bus: EventBus):
    """Simulate a slow module that publishes after a delay."""
    await asyncio.sleep(0.2)
    bus.publish(Event(
        topic="robot.move.completed",
        payload={"status": "ok", "position": [0.1, 0.2]},
        source="driver",
    ))


async def main():
    bus = EventBus()

    # Start the delayed publisher
    asyncio.create_task(delayed_publisher(bus))

    # Wait for completion (with timeout)
    print("Waiting for move to complete...")
    event = await bus.await_event(
        "robot.move.completed",
        timeout=1.0,
        filter_fn=lambda e: e.payload.get("status") == "ok",
    )

    if event:
        print(f"Move done! Position: {event.payload['position']}")
    else:
        print("Timeout -- move did not complete in time.")


if __name__ == "__main__":
    asyncio.run(main())
```

**Expected output**:

```
Waiting for move to complete...
Move done! Position: [0.1, 0.2]
```

**Key point**: `await_event()` creates a temporary subscription, waits for a match, then cleans itself up automatically.

---

## Step 4: Priority-Based Processing

Not all events are equal. Use priorities to ensure critical events are handled first:

```python
#!/usr/bin/env python3
"""Tutorial 02: Priority-based event processing."""

from rosclaw.core import EventBus, Event, EventPriority

bus = EventBus()

received = []


def handler(event: Event):
    received.append((event.priority.name, event.payload["msg"]))


bus.subscribe("mixed.priority", handler)

# Publish in reverse priority order
bus.publish(Event(topic="mixed.priority", payload={"msg": "background"}, priority=EventPriority.BACKGROUND))
bus.publish(Event(topic="mixed.priority", payload={"msg": "low"},        priority=EventPriority.LOW))
bus.publish(Event(topic="mixed.priority", payload={"msg": "normal"},     priority=EventPriority.NORMAL))
bus.publish(Event(topic="mixed.priority", payload={"msg": "high"},       priority=EventPriority.HIGH))
bus.publish(Event(topic="mixed.priority", payload={"msg": "critical"},   priority=EventPriority.CRITICAL))

print("Received order:")
for priority, msg in received:
    print(f"  {priority:10s} -> {msg}")
```

**Expected output**:

```
Received order:
  BACKGROUND -> background
  LOW        -> low
  NORMAL     -> normal
  HIGH       -> high
  CRITICAL   -> critical
```

**Key point**: Priority affects how the event is stored in the async queue, not sync subscriber call order. For guaranteed ordering, use a single subscriber and sort by priority.

---

## Step 5: Command/Event Separation Pattern

A robust pattern: use `command.*` topics for actions and `event.*` topics for state changes. This decouples senders from receivers:

```python
#!/usr/bin/env python3
"""Tutorial 02: Command/event separation pattern."""

import asyncio
from rosclaw.core import EventBus, Event


class RobotController:
    """Receives commands and publishes state events."""

    def __init__(self, bus: EventBus):
        self.bus = bus
        self.position = [0.0, 0.0, 0.0]
        bus.subscribe("command.robot.move", self._on_move)

    def _on_move(self, event: Event):
        target = event.payload["target"]
        print(f"  [Controller] Moving to {target}")
        self.position = target
        self.bus.publish(Event(
            topic="event.robot.moved",
            payload={"position": self.position},
            source="controller",
        ))


class Logger:
    """Logs all state changes without knowing who caused them."""

    def __init__(self, bus: EventBus):
        bus.subscribe("event.robot.moved", lambda e: print(f"  [Logger] Robot at {e.payload['position']}"))


async def main():
    bus = EventBus()
    controller = RobotController(bus)
    logger = Logger(bus)

    # Send a command
    bus.publish(Event(
        topic="command.robot.move",
        payload={"target": [0.5, 0.2, 0.1]},
        source="planner",
    ))

    await asyncio.sleep(0.1)


if __name__ == "__main__":
    asyncio.run(main())
```

**Expected output**:

```
  [Controller] Moving to [0.5, 0.2, 0.1]
  [Logger] Robot at [0.5, 0.2, 0.1]
```

**Key point**: The `Logger` never references `command.robot.move`. It only reacts to state changes. This means you can add new command sources (joystick, LLM, script) without changing the logger.

---

## Complete Example

Putting it all together -- a mini system with async handlers, history, and command/event separation:

```python
#!/usr/bin/env python3
"""Tutorial 02: Complete EventBus example."""

import asyncio
from rosclaw.core import EventBus, Event, EventPriority


async def main():
    bus = EventBus()

    # Logger subscribes to all state events
    bus.subscribe("event.", lambda e: print(f"  [LOG] {e.topic}: {e.payload}"))

    # Safety monitor subscribes to critical events
    bus.subscribe("event.safety.", lambda e: print(f"  [SAFETY] ALERT: {e.payload}"))

    # Simulate a sequence
    bus.publish(Event(topic="event.robot.power_on", payload={}, source="system"))
    bus.publish(Event(topic="command.robot.move", payload={"target": [0.1, 0.2]}, source="planner"))
    bus.publish(Event(topic="event.robot.moved", payload={"position": [0.1, 0.2]}, source="driver"))
    bus.publish(Event(topic="event.safety.near_limit", payload={"joint": 2}, priority=EventPriority.HIGH))

    # Query history
    print(f"\nHistory count: {len(bus.get_history())}")

    # Wait for a specific future event
    async def future_event():
        await asyncio.sleep(0.1)
        bus.publish(Event(topic="event.robot.done", payload={"status": "success"}))

    asyncio.create_task(future_event())
    evt = await bus.await_event("event.robot.done", timeout=1.0)
    print(f"\nFuture event received: {evt.payload}")


if __name__ == "__main__":
    asyncio.run(main())
```

---

## Try It Yourself

1. **Filter history**: Modify the history example to find all events with `payload["val"] > 22`.

2. **Timeout handling**: Change `await_event()` timeout to `0.1s` and remove the delayed publisher. Handle the `None` result gracefully.

3. **Multiple awaiters**: Create two coroutines that both `await_event("event.ready")`. Publish one event. Do both coroutines receive it? (Hint: `await_event()` uses a temporary sync subscriber.)

---

## Next Steps

- `03_runtime_lifecycle.md` -- Learn the 8-state lifecycle machine
- `docs/API_REFERENCE.md` -- Full EventBus API
- `tests/test_event_bus_extended.py` -- See how ROSClaw tests the EventBus

---

## Common Issues

**Events not arriving in async handlers**

Ensure you `await asyncio.sleep()` after publishing to give the event loop time to process:

```python
bus.publish(Event(...))
await asyncio.sleep(0.1)  # Allow async handlers to run
```

**Memory usage with high event volume**

The default history limit is 10,000 events. For long-running processes, clear history periodically:

```python
if len(bus._event_history) > 5000:
    bus.clear_history()
```

**Subscriber exceptions breaking other handlers**

The EventBus catches and prints exceptions, so one bad handler won't crash others. Check stdout for `[EventBus] Error in subscriber...` messages.

---

*Ready for the next level? See `03_runtime_lifecycle.md`.*
