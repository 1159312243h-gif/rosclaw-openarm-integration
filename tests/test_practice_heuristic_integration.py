"""Tests for heuristic.recovery_executed event publishing.

Task 4: Practice publishes heuristic.recovery_executed after recovery.
"""

from rosclaw.core.event_bus import EventBus
from rosclaw.practice.recorder import PracticeRecorder


def test_heuristic_recovery_executed_published():
    """record_recovery_outcome publishes heuristic.recovery_executed event."""
    bus = EventBus()
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=bus)
    recorder.initialize()

    captured = []
    bus.subscribe("heuristic.recovery_executed", lambda e: captured.append(e))

    recorder.record_recovery_outcome(
        rule_id="gripper_force_increase",
        success=True,
        duration=0.45,
        correlation_id="prac_123",
    )

    assert len(captured) == 1
    evt = captured[0]
    assert evt.topic == "heuristic.recovery_executed"
    assert evt.payload["rule_id"] == "gripper_force_increase"
    assert evt.payload["success"] is True
    assert evt.payload["duration"] == 0.45
    assert evt.payload["robot_id"] == "ur5e_01"
    assert evt.payload["correlation_id"] == "prac_123"
    assert "timestamp" in evt.payload

    recorder.stop()


def test_heuristic_recovery_failure_published():
    """Failed recovery also publishes event."""
    bus = EventBus()
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=bus)
    recorder.initialize()

    captured = []
    bus.subscribe("heuristic.recovery_executed", lambda e: captured.append(e))

    recorder.record_recovery_outcome(
        rule_id="reposition_gripper",
        success=False,
        duration=0.12,
        correlation_id="prac_456",
    )

    assert len(captured) == 1
    assert captured[0].payload["success"] is False
    assert captured[0].payload["rule_id"] == "reposition_gripper"

    recorder.stop()


def test_heuristic_recovery_without_eventbus():
    """record_recovery_outcome without EventBus does not crash."""
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=None)
    recorder.initialize()

    recorder.record_recovery_outcome(
        rule_id="test_rule",
        success=True,
        duration=0.1,
    )

    recorder.stop()


def test_heuristic_recovery_multiple_rules():
    """Multiple recovery attempts publish separate events."""
    bus = EventBus()
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=bus)
    recorder.initialize()

    captured = []
    bus.subscribe("heuristic.recovery_executed", lambda e: captured.append(e.payload))

    rules = [
        ("rule_1", True, 0.1),
        ("rule_2", False, 0.2),
        ("rule_3", True, 0.3),
    ]

    for rule_id, success, duration in rules:
        recorder.record_recovery_outcome(rule_id, success, duration)

    assert len(captured) == 3
    assert captured[0]["rule_id"] == "rule_1"
    assert captured[1]["rule_id"] == "rule_2"
    assert captured[2]["rule_id"] == "rule_3"

    recorder.stop()


def test_heuristic_recovery_payload_structure():
    """Payload has all required fields for HOW subscriber."""
    bus = EventBus()
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=bus)
    recorder.initialize()

    captured = []
    bus.subscribe("heuristic.recovery_executed", lambda e: captured.append(e.payload))

    recorder.record_recovery_outcome(
        rule_id="angle_adjust_15deg",
        success=True,
        duration=0.67,
        correlation_id="prac_789",
    )

    p = captured[0]
    assert "rule_id" in p
    assert "success" in p
    assert "duration" in p
    assert "robot_id" in p
    assert "timestamp" in p
    assert "correlation_id" in p

    recorder.stop()


def test_heuristic_recovery_default_correlation_id():
    """correlation_id defaults to empty string."""
    bus = EventBus()
    recorder = PracticeRecorder("ur5e_01", joint_dof=6, event_bus=bus)
    recorder.initialize()

    captured = []
    bus.subscribe("heuristic.recovery_executed", lambda e: captured.append(e.payload))

    recorder.record_recovery_outcome(rule_id="test", success=True, duration=0.1)

    assert captured[0]["correlation_id"] == ""

    recorder.stop()
