"""MoveIt-backed OpenArm gripper executor for the official sandbox boundary."""

from __future__ import annotations

import math
import subprocess
from collections.abc import Callable
from pathlib import Path
from typing import Any

from rosclaw.kernel import (
    AcknowledgementStage,
    ActionEnvelope,
    ActionExecutionResult,
    ActionState,
    EvidenceDomain,
    EvidenceLevel,
    ExecutionMode,
)
from rosclaw.sandbox.openarm import ROBOT_ID
from rosclaw.sandbox.openarm_moveit import (
    _artifact_directory,
    _canonical_arguments_sha256,
    _invoke_worker,
    _write_worker_artifact,
)

CAPABILITY_ID = "openarm.gripper.move"
MIN_TARGET_M = 0.0
MAX_TARGET_M = 0.044
MIN_DURATION_S = 0.25
MAX_DURATION_S = 10.0
MIN_HOLD_S = 0.0
MAX_HOLD_S = 10.0
TARGET_TOLERANCE_M = 0.0005
MIMIC_TOLERANCE_M = 0.0005
HOLD_TOLERANCE_M = 0.0002
STATIONARY_TOLERANCE = 0.0002
_WORKER_SCHEMA = "openarm.gripper.moveit.worker.v1"
_PLANNING_PHASE = 2
_EXECUTING_PHASE = 3
_VERIFYING_PHASE = 4
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_ALLOWED_ARGUMENTS = {"side", "target_m", "duration_s", "hold_s"}
WorkerTransport = Callable[[dict[str, Any]], dict[str, Any]]


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def validate_openarm_gripper_move(action: ActionEnvelope) -> dict[str, Any]:
    """Validate and normalize one simulation-only gripper request."""

    if action.body_id != ROBOT_ID:
        raise ValueError(f"unsupported body_id: {action.body_id}")
    if action.capability_id != CAPABILITY_ID:
        raise ValueError(f"unsupported capability_id: {action.capability_id}")
    if action.execution_mode is not ExecutionMode.SIMULATION:
        raise ValueError("OpenArm gripper motion is enabled only in SIMULATION")
    unknown = set(action.arguments) - _ALLOWED_ARGUMENTS
    if unknown:
        raise ValueError(f"unknown arguments: {sorted(unknown)}")

    side = str(action.arguments.get("side", "")).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported side: {side}")
    if "target_m" not in action.arguments:
        raise ValueError("target_m is required")
    target_m = _number(action.arguments["target_m"], "target_m")
    duration_s = _number(action.arguments.get("duration_s", 2.0), "duration_s")
    hold_s = _number(action.arguments.get("hold_s", 1.0), "hold_s")
    if not MIN_TARGET_M <= target_m <= MAX_TARGET_M:
        raise ValueError("target_m must be between 0.0 and 0.044 m")
    if not MIN_DURATION_S <= duration_s <= MAX_DURATION_S:
        raise ValueError("duration_s must be between 0.25 and 10.0 s")
    if not MIN_HOLD_S <= hold_s <= MAX_HOLD_S:
        raise ValueError("hold_s must be between 0.0 and 10.0 s")
    return {
        "side": side,
        "target_m": target_m,
        "duration_s": duration_s,
        "hold_s": hold_s,
    }


def _failure(
    action: ActionEnvelope,
    code: str,
    message: str,
    *,
    state: ActionState = ActionState.FAILED,
    accepted: bool = False,
    artifact_directory: Path | None = None,
) -> ActionExecutionResult:
    return ActionExecutionResult(
        final_state=state,
        evidence_level=EvidenceLevel.REQUESTED,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision={
            "validation_type": "OpenArmGripperStaticPolicyAndMoveItEvidenceValidation",
            "allowed": False,
            "reason": message,
            "moveit_required": True,
            "direct_mujoco_fallback": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "planning_backend": "moveit2_move_group",
            "physics_executed": False,
            "direct_mujoco_fallback": False,
        },
        dispatch_result={
            "accepted": accepted,
            "transport": "isolated_ros2_action_worker",
            "action_name": "/move_action",
        },
        verification_result={"passed": False, "message": message},
        errors=[{"code": code, "message": message}],
        artifact_directory=str(artifact_directory) if artifact_directory else None,
        acknowledgement_stage=AcknowledgementStage.REQUEST_ACCEPTED,
    )


def _finite_nonnegative(payload: dict[str, Any], name: str) -> float | None:
    value = payload.get(name)
    if (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and float(value) >= 0.0
    ):
        return float(value)
    return None


def run_openarm_gripper_move_via_moveit(
    action: ActionEnvelope,
    *,
    artifact_root: Path,
    worker_command: str | None,
    action_name: str = "/move_action",
    server_timeout_sec: float = 5.0,
    result_timeout_sec: float = 30.0,
    cancel_timeout_sec: float = 3.0,
    transport: WorkerTransport | None = None,
) -> ActionExecutionResult:
    """Execute one gripper target through MoveIt and require measured evidence."""

    try:
        request = validate_openarm_gripper_move(action)
    except ValueError as exc:
        return _failure(
            action,
            "OPENARM_GRIPPER_ACTION_REJECTED",
            str(exc),
            state=ActionState.BLOCKED,
        )

    try:
        directory = _artifact_directory(artifact_root, action.action_id)
    except FileExistsError as exc:
        return _failure(action, "OPENARM_ACTION_ID_ALREADY_EXISTS", str(exc))
    except (OSError, ValueError) as exc:
        return _failure(action, "OPENARM_ARTIFACT_PATH_INVALID", str(exc))

    normalized_arguments = dict(request)
    arguments_sha256 = _canonical_arguments_sha256(normalized_arguments)
    worker_request = {
        "schema_version": _WORKER_SCHEMA,
        "request_id": action.action_id,
        "action_name": action_name,
        **normalized_arguments,
        "server_timeout_sec": server_timeout_sec,
        "result_timeout_sec": result_timeout_sec,
        "cancel_timeout_sec": cancel_timeout_sec,
    }
    try:
        if transport is not None:
            payload = transport(worker_request)
        else:
            if not worker_command:
                raise RuntimeError("MoveIt worker command is not configured")
            payload = _invoke_worker(
                worker_request,
                worker_command=worker_command,
                timeout_sec=(
                    server_timeout_sec
                    + result_timeout_sec
                    + cancel_timeout_sec
                    + request["hold_s"]
                    + 5.0
                ),
            )
        artifact_uri, artifact_sha256 = _write_worker_artifact(directory, payload)
    except subprocess.TimeoutExpired:
        return _failure(
            action,
            "MOVEIT_WORKER_TIMEOUT",
            "MoveIt worker exceeded its fail-closed process timeout",
            state=ActionState.TIMED_OUT,
            artifact_directory=directory,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        return _failure(
            action,
            "MOVEIT_TRANSPORT_UNAVAILABLE",
            str(exc),
            artifact_directory=directory,
        )

    if payload.get("schema_version") != _WORKER_SCHEMA:
        return _failure(
            action,
            "MOVEIT_WORKER_SCHEMA_INVALID",
            "MoveIt worker returned an unsupported gripper schema",
            artifact_directory=directory,
        )
    if payload.get("request_id") != action.action_id:
        return _failure(
            action,
            "MOVEIT_WORKER_IDENTITY_MISMATCH",
            "MoveIt worker response request_id does not match the ActionEnvelope",
            artifact_directory=directory,
        )

    feedback = payload.get("feedback") if isinstance(payload.get("feedback"), list) else []
    phases = {
        item.get("phase") for item in feedback if isinstance(item, dict)
    }
    target_error = _finite_nonnegative(payload, "target_error_m")
    mimic_error = _finite_nonnegative(payload, "mimic_error_m")
    driver_drift = _finite_nonnegative(payload, "driver_hold_drift_m")
    follower_drift = _finite_nonnegative(payload, "follower_hold_drift_m")
    stationary_delta = _finite_nonnegative(payload, "maximum_stationary_joint_delta")
    evidence_complete = bool(
        {_PLANNING_PHASE, _EXECUTING_PHASE, _VERIFYING_PHASE} <= phases
        and target_error is not None
        and target_error <= TARGET_TOLERANCE_M
        and mimic_error is not None
        and mimic_error <= MIMIC_TOLERANCE_M
        and driver_drift is not None
        and driver_drift <= HOLD_TOLERANCE_M
        and follower_drift is not None
        and follower_drift <= HOLD_TOLERANCE_M
        and stationary_delta is not None
        and stationary_delta <= STATIONARY_TOLERANCE
    )
    accepted = payload.get("accepted") is True
    timed_out = payload.get("timed_out") is True
    status = payload.get("status")
    success = bool(
        accepted
        and not timed_out
        and status == _STATUS_SUCCEEDED
        and payload.get("success") is True
        and evidence_complete
    )
    if success:
        state = ActionState.COMPLETED
        evidence = EvidenceLevel.TASK_VERIFIED
        acknowledgement = AcknowledgementStage.TASK_VERIFIED
        errors: list[dict[str, Any]] = []
    elif timed_out:
        state = ActionState.TIMED_OUT
        evidence = EvidenceLevel.DRIVER_CONFIRMED if accepted else EvidenceLevel.REQUESTED
        acknowledgement = AcknowledgementStage.PROTOCOL_ACKNOWLEDGED
        errors = [{"code": "MOVEIT_RESULT_TIMEOUT", "message": str(payload.get("message", ""))}]
    elif status == _STATUS_CANCELED:
        state = ActionState.CANCELLED
        evidence = EvidenceLevel.DRIVER_CONFIRMED
        acknowledgement = AcknowledgementStage.PROTOCOL_ACKNOWLEDGED
        errors = [{"code": "MOVEIT_ACTION_CANCELED", "message": str(payload.get("message", ""))}]
    else:
        state = ActionState.FAILED
        evidence = (
            EvidenceLevel.PHYSICALLY_OBSERVED
            if _EXECUTING_PHASE in phases
            else EvidenceLevel.REQUESTED
        )
        acknowledgement = (
            AcknowledgementStage.EFFECT_OBSERVED
            if _EXECUTING_PHASE in phases
            else AcknowledgementStage.REQUEST_ACCEPTED
        )
        code = (
            "MOVEIT_GRIPPER_EVIDENCE_INCOMPLETE"
            if payload.get("success") is True and not evidence_complete
            else "MOVEIT_GRIPPER_ACTION_FAILED"
        )
        errors = [{"code": code, "message": str(payload.get("message", state.value))}]

    verification = {
        "passed": success,
        "source": "moveit.move_group.gripper",
        "moveit_planning_observed": _PLANNING_PHASE in phases,
        "trajectory_execution_observed": _EXECUTING_PHASE in phases,
        "final_joint_verification_observed": _VERIFYING_PHASE in phases,
        "target_driver_joint_m": request["target_m"],
        "final_driver_joint_m": payload.get("final_driver_joint_m"),
        "final_follower_joint_m": payload.get("final_follower_joint_m"),
        "target_error_m": target_error,
        "mimic_error_m": mimic_error,
        "driver_hold_drift_m": driver_drift,
        "follower_hold_drift_m": follower_drift,
        "maximum_stationary_joint_delta": stationary_delta,
        "message": payload.get("message"),
    }
    return ActionExecutionResult(
        final_state=state,
        evidence_level=evidence,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision={
            "validation_type": "OpenArmGripperStaticPolicyAndMoveItEvidenceValidation",
            "allowed": True,
            "moveit_required": True,
            "direct_mujoco_fallback": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "planning_backend": "moveit2_move_group",
            "execution_backend": "moveit2_execute_trajectory",
            "controller_backend": "follow_joint_trajectory",
            "physics_executed": _EXECUTING_PHASE in phases,
            "direct_mujoco_fallback": False,
            "worker_artifact_sha256": artifact_sha256,
        },
        dispatch_result={
            "accepted": accepted,
            "transport": "isolated_ros2_action_worker",
            "action_name": action_name,
            "request_id": action.action_id,
        },
        driver_ack={"goal_id": payload.get("goal_id"), "status": status} if accepted else None,
        observations=feedback,
        verification_result=verification,
        artifacts=[artifact_uri],
        errors=errors,
        artifact_directory=str(directory),
        acknowledgement_stage=acknowledgement,
        normalized_arguments=normalized_arguments,
        arguments_sha256=arguments_sha256,
        final_error_m=target_error,
    )


__all__ = [
    "CAPABILITY_ID",
    "run_openarm_gripper_move_via_moveit",
    "validate_openarm_gripper_move",
]
