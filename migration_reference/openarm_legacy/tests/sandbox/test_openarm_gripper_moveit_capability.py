"""Fail-closed tests for the official OpenArm gripper capability."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from rosclaw.core.event_bus import EventBus
from rosclaw.kernel import ActionEnvelope, ActionState, ExecutionMode
from rosclaw.sandbox.openarm import ROBOT_ID
from rosclaw.sandbox.openarm_gripper_moveit import (
    CAPABILITY_ID,
    run_openarm_gripper_move_via_moveit,
    validate_openarm_gripper_move,
)
from rosclaw.sandbox.runtime_adapter import SandboxRuntimeAdapter


def _action(**arguments: object) -> ActionEnvelope:
    payload: dict[str, object] = {
        "side": "right",
        "target_m": 0.022,
        "duration_s": 2.0,
        "hold_s": 1.0,
    }
    payload.update(arguments)
    return ActionEnvelope(
        action_id="action_gripper_test_001",
        actor_id="pytest",
        agent_framework="pytest",
        session_id="session-test",
        body_id=ROBOT_ID,
        capability_id=CAPABILITY_ID,
        arguments=payload,
        execution_mode=ExecutionMode.SIMULATION,
    )


def _payload(**override: Any) -> dict[str, Any]:
    payload = {
        "schema_version": "openarm.gripper.moveit.worker.v1",
        "request_id": "action_gripper_test_001",
        "server_available": True,
        "accepted": True,
        "timed_out": False,
        "status": 4,
        "success": True,
        "error_code": 0,
        "message": "Gripper motion verified",
        "goal_id": "001122",
        "feedback": [
            {"phase": 2, "progress": 0.2, "message": "planning"},
            {"phase": 3, "progress": 0.5, "message": "executing"},
            {"phase": 4, "progress": 0.9, "message": "verifying"},
        ],
        "final_driver_joint_m": 0.022,
        "final_follower_joint_m": 0.02201,
        "target_error_m": 0.0,
        "mimic_error_m": 0.00001,
        "driver_hold_drift_m": 0.00001,
        "follower_hold_drift_m": 0.00001,
        "maximum_stationary_joint_delta": 0.00001,
    }
    payload.update(override)
    return payload


def test_gripper_contract_normalizes_request() -> None:
    assert validate_openarm_gripper_move(_action()) == {
        "side": "right",
        "target_m": 0.022,
        "duration_s": 2.0,
        "hold_s": 1.0,
    }


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"side": "center"}, "unsupported side"),
        ({"target_m": -0.001}, "target_m must be between"),
        ({"target_m": 0.045}, "target_m must be between"),
        ({"duration_s": 0.1}, "duration_s must be between"),
        ({"hold_s": 11.0}, "hold_s must be between"),
        ({"unexpected": True}, "unknown arguments"),
    ],
)
def test_gripper_contract_fails_closed(override: dict[str, object], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        validate_openarm_gripper_move(_action(**override))


def test_invalid_target_is_blocked_before_worker_dispatch(tmp_path: Path) -> None:
    called = False

    def transport(_: dict[str, Any]) -> dict[str, Any]:
        nonlocal called
        called = True
        return _payload()

    result = run_openarm_gripper_move_via_moveit(
        _action(target_m=0.1),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )

    assert result.final_state is ActionState.BLOCKED
    assert result.errors[0]["code"] == "OPENARM_GRIPPER_ACTION_REJECTED"
    assert called is False
    assert list(tmp_path.iterdir()) == []


def test_non_simulation_mode_is_blocked_before_worker_dispatch(tmp_path: Path) -> None:
    action = ActionEnvelope(
        action_id="action_gripper_real_test",
        actor_id="pytest",
        agent_framework="pytest",
        session_id="session-test",
        body_id=ROBOT_ID,
        capability_id=CAPABILITY_ID,
        arguments={"side": "right", "target_m": 0.022},
        execution_mode=ExecutionMode.REAL,
    )

    result = run_openarm_gripper_move_via_moveit(
        action,
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _: pytest.fail("worker must not be called"),
    )

    assert result.final_state is ActionState.BLOCKED
    assert "only in SIMULATION" in result.errors[0]["message"]
    assert list(tmp_path.iterdir()) == []


def test_worker_receives_normalized_arguments(tmp_path: Path) -> None:
    captured: dict[str, Any] = {}

    def transport(request: dict[str, Any]) -> dict[str, Any]:
        captured.update(request)
        return _payload()

    run_openarm_gripper_move_via_moveit(
        _action(side=" RIGHT "),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )

    assert captured["schema_version"] == "openarm.gripper.moveit.worker.v1"
    assert captured["side"] == "right"
    assert captured["target_m"] == 0.022
    assert captured["duration_s"] == 2.0
    assert captured["hold_s"] == 1.0


def test_complete_measured_evidence_is_task_verified(tmp_path: Path) -> None:
    result = run_openarm_gripper_move_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _: _payload(),
    )

    assert result.final_state is ActionState.COMPLETED
    assert result.evidence_level.value == "TASK_VERIFIED"
    assert result.verification_result["passed"] is True
    assert result.normalized_arguments == {
        "side": "right",
        "target_m": 0.022,
        "duration_s": 2.0,
        "hold_s": 1.0,
    }


def test_success_without_mimic_evidence_is_rejected(tmp_path: Path) -> None:
    result = run_openarm_gripper_move_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _: _payload(mimic_error_m=None),
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "MOVEIT_GRIPPER_EVIDENCE_INCOMPLETE"


def test_runtime_registers_arm_and_gripper_capabilities() -> None:
    adapter = SandboxRuntimeAdapter(
        {
            "engine": "fixture",
            "world_id": "empty",
            "robot_id": ROBOT_ID,
            "openarm_moveit_required": True,
        },
        event_bus=EventBus(),
    )

    assert adapter.supported_capabilities == (
        "openarm.arm.move_relative",
        CAPABILITY_ID,
        "openarm.pick_demo",
        "sandbox.reach",
    )
