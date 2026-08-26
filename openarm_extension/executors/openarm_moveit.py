"""MoveIt-backed OpenArm executor for the official ROSClaw sandbox boundary."""

from __future__ import annotations

import hashlib
import json
import math
import re
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
)
from rosclaw.sandbox.openarm import validate_openarm_move_relative

_WORKER_SCHEMA = "openarm.moveit.worker.v1"
_PLANNING_PHASE = 2
_EXECUTING_PHASE = 3
_VERIFYING_PHASE = 4
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 5
_SAFE_ACTION_ID = re.compile(r"^action_[A-Za-z0-9_.-]{1,120}$")
WorkerTransport = Callable[[dict[str, Any]], dict[str, Any]]


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
            "validation_type": "OpenArmMoveItExecutionBoundary",
            "allowed": False,
            "reason": message,
            "moveit_required": True,
            "direct_mujoco_fallback": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "planning_backend": "moveit2",
            "physics_executed": False,
            "direct_mujoco_fallback": False,
        },
        dispatch_result={
            "accepted": accepted,
            "transport": "isolated_ros2_action_worker",
            "action_name": "/rosclaw/arm_motion",
        },
        verification_result={"passed": False, "message": message},
        errors=[{"code": code, "message": message}],
        artifact_directory=str(artifact_directory) if artifact_directory else None,
        acknowledgement_stage=AcknowledgementStage.REQUEST_ACCEPTED,
    )


def _invoke_worker(
    request: dict[str, Any],
    *,
    worker_command: str,
    timeout_sec: float,
) -> dict[str, Any]:
    command = Path(worker_command).expanduser().resolve()
    if not command.is_file():
        raise FileNotFoundError(f"MoveIt worker command does not exist: {command}")
    completed = subprocess.run(  # noqa: S603
        [str(command)],
        input=json.dumps(request, separators=(",", ":")),
        capture_output=True,
        text=True,
        timeout=timeout_sec,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip()[-2000:] or "worker exited without an error message"
        raise RuntimeError(f"MoveIt worker failed with exit {completed.returncode}: {detail}")
    output = completed.stdout.strip()
    if not output:
        raise RuntimeError("MoveIt worker returned no JSON")
    try:
        payload = json.loads(output)
    except json.JSONDecodeError as exc:
        raise RuntimeError("MoveIt worker returned invalid JSON") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("MoveIt worker response must be a JSON object")
    return payload


def _artifact_directory(root: Path, action_id: str) -> Path:
    if not _SAFE_ACTION_ID.fullmatch(action_id):
        raise ValueError("action_id contains unsupported characters")
    resolved_root = root.expanduser().resolve()
    directory = (resolved_root / action_id).resolve()
    if not directory.is_relative_to(resolved_root):
        raise ValueError("action_id escapes the sandbox artifact root")
    resolved_root.mkdir(parents=True, exist_ok=True)
    directory.mkdir(exist_ok=False)
    return directory


def _write_worker_artifact(directory: Path, payload: dict[str, Any]) -> tuple[str, str]:
    path = directory / "moveit_worker_result.json"
    encoded = json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False).encode("utf-8")
    temporary = path.with_suffix(".json.tmp")
    temporary.write_bytes(encoded)
    temporary.replace(path)
    return path.as_uri(), hashlib.sha256(encoded).hexdigest()


def _feedback_phases(payload: dict[str, Any]) -> set[int]:
    feedback = payload.get("feedback")
    if not isinstance(feedback, list):
        return set()
    phases: set[int] = set()
    for item in feedback:
        if isinstance(item, dict) and isinstance(item.get("phase"), int):
            phases.add(item["phase"])
    return phases


def _has_valid_pose(payload: dict[str, Any], field: str) -> bool:
    pose = payload.get(field)
    if not isinstance(pose, dict) or not str(pose.get("frame_id", "")).strip():
        return False
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return False
    values = [
        position.get("x"),
        position.get("y"),
        position.get("z"),
        orientation.get("x"),
        orientation.get("y"),
        orientation.get("z"),
        orientation.get("w"),
    ]
    return all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        for value in values
    )


def _canonical_arguments_sha256(arguments: dict[str, Any]) -> str:
    payload = json.dumps(
        arguments,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _reported_pose_error(payload: dict[str, Any]) -> float | None:
    value = payload.get("final_error_m")
    if (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and float(value) >= 0.0
    ):
        return float(value)
    return None


def _pose_position(pose: dict[str, Any]) -> tuple[float, float, float]:
    position = pose["position"]
    return (float(position["x"]), float(position["y"]), float(position["z"]))


def _pose_evidence_consistent(
    request: dict[str, Any],
    initial_pose: dict[str, Any],
    target_pose: dict[str, Any],
    final_pose: dict[str, Any],
    final_error_m: float,
) -> bool:
    if not (
        initial_pose["frame_id"] == target_pose["frame_id"] == final_pose["frame_id"] == "world"
    ):
        return False
    initial = _pose_position(initial_pose)
    target = _pose_position(target_pose)
    final = _pose_position(final_pose)
    translation = request["translation"]
    expected_target = (
        initial[0] + float(translation["x"]),
        initial[1] + float(translation["y"]),
        initial[2] + float(translation["z"]),
    )
    target_error = math.dist(target, expected_target)
    measured_final_error = math.dist(target, final)
    return target_error <= 1.0e-9 and abs(measured_final_error - final_error_m) <= 1.0e-6


def run_openarm_move_relative_via_moveit(
    action: ActionEnvelope,
    *,
    artifact_root: Path,
    worker_command: str | None,
    action_name: str = "/rosclaw/arm_motion",
    server_timeout_sec: float = 5.0,
    result_timeout_sec: float = 30.0,
    cancel_timeout_sec: float = 3.0,
    transport: WorkerTransport | None = None,
) -> ActionExecutionResult:
    """Execute a bounded OpenArm request only through MoveIt and the ROS bridge."""

    try:
        request = validate_openarm_move_relative(action)
    except ValueError as exc:
        return _failure(action, "OPENARM_ACTION_REJECTED", str(exc), state=ActionState.BLOCKED)

    try:
        directory = _artifact_directory(artifact_root, action.action_id)
    except FileExistsError as exc:
        return _failure(action, "OPENARM_ACTION_ID_ALREADY_EXISTS", str(exc))
    except (OSError, ValueError) as exc:
        return _failure(action, "OPENARM_ARTIFACT_PATH_INVALID", str(exc))

    normalized_arguments = {
        "arm": request["arm"],
        "reference_frame": request["reference_frame"],
        "translation": request["translation"],
        "velocity_scale": request["velocity_scale"],
        "acceleration_scale": request["acceleration_scale"],
        "avoid_collisions": request["avoid_collisions"],
    }
    arguments_sha256 = _canonical_arguments_sha256(normalized_arguments)

    worker_request = {
        "schema_version": _WORKER_SCHEMA,
        "request_id": action.action_id,
        "action_name": action_name,
        "arm": request["arm"],
        "reference_frame": request["reference_frame"],
        "translation": request["translation"],
        "velocity_scale": request["velocity_scale"],
        "acceleration_scale": request["acceleration_scale"],
        "avoid_collisions": request["avoid_collisions"],
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
                timeout_sec=server_timeout_sec + result_timeout_sec + cancel_timeout_sec + 5.0,
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
            "MoveIt worker returned an unsupported schema",
            artifact_directory=directory,
        )
    if payload.get("request_id") != action.action_id:
        return _failure(
            action,
            "MOVEIT_WORKER_IDENTITY_MISMATCH",
            "MoveIt worker response request_id does not match the ActionEnvelope",
            artifact_directory=directory,
        )

    accepted = payload.get("accepted") is True
    timed_out = payload.get("timed_out") is True
    status = payload.get("status")
    result_success = payload.get("success") is True
    phases = _feedback_phases(payload)
    planning_observed = _PLANNING_PHASE in phases
    execution_observed = _EXECUTING_PHASE in phases
    verification_observed = _VERIFYING_PHASE in phases
    initial_pose_reported = _has_valid_pose(payload, "initial_pose")
    target_pose_reported = _has_valid_pose(payload, "target_pose")
    final_pose_reported = _has_valid_pose(payload, "final_pose")
    final_error_m = _reported_pose_error(payload)
    initial_pose = payload.get("initial_pose")
    target_pose = payload.get("target_pose")
    final_pose = payload.get("final_pose")
    pose_evidence_consistent = bool(
        initial_pose_reported
        and target_pose_reported
        and final_pose_reported
        and final_error_m is not None
        and _pose_evidence_consistent(
            request,
            initial_pose,
            target_pose,
            final_pose,
            final_error_m,
        )
    )
    complete_moveit_evidence = (
        planning_observed
        and execution_observed
        and verification_observed
        and pose_evidence_consistent
    )
    success = bool(
        accepted
        and not timed_out
        and status == _STATUS_SUCCEEDED
        and result_success
        and complete_moveit_evidence
    )

    if success:
        state = ActionState.COMPLETED
        evidence = EvidenceLevel.TASK_VERIFIED
        acknowledgement = AcknowledgementStage.TASK_VERIFIED
        errors: list[dict[str, Any]] = []
    elif timed_out:
        state = ActionState.TIMED_OUT
        evidence = EvidenceLevel.DRIVER_CONFIRMED if accepted else EvidenceLevel.REQUESTED
        acknowledgement = (
            AcknowledgementStage.PROTOCOL_ACKNOWLEDGED
            if accepted
            else AcknowledgementStage.REQUEST_ACCEPTED
        )
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
            if execution_observed
            else EvidenceLevel.DISPATCH_CONFIRMED
            if accepted
            else EvidenceLevel.REQUESTED
        )
        acknowledgement = (
            AcknowledgementStage.EFFECT_OBSERVED
            if execution_observed
            else AcknowledgementStage.COMMAND_DISPATCHED
            if accepted
            else AcknowledgementStage.REQUEST_ACCEPTED
        )
        code = (
            "MOVEIT_EVIDENCE_INCOMPLETE"
            if result_success and not complete_moveit_evidence
            else "MOVEIT_ACTION_FAILED"
        )
        errors = [{"code": code, "message": str(payload.get("message", state.value))}]

    feedback = payload.get("feedback") if isinstance(payload.get("feedback"), list) else []
    return ActionExecutionResult(
        final_state=state,
        evidence_level=evidence,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision={
            "validation_type": "OpenArmStaticPolicyAndMoveItEvidenceValidation",
            "allowed": True,
            "moveit_required": True,
            "direct_mujoco_fallback": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "planning_backend": "moveit2_get_cartesian_path",
            "execution_backend": "moveit2_execute_trajectory",
            "controller_backend": "follow_joint_trajectory",
            "physics_executed": execution_observed,
            "direct_mujoco_fallback": False,
            "worker_artifact_sha256": artifact_sha256,
        },
        dispatch_result={
            "accepted": accepted,
            "transport": "isolated_ros2_action_worker",
            "action_name": action_name,
            "request_id": action.action_id,
        },
        driver_ack={
            "goal_id": payload.get("goal_id"),
            "status": status,
        }
        if accepted
        else None,
        observations=feedback,
        verification_result={
            "passed": success,
            "source": "rosclaw_motion.relative_motion_server",
            "moveit_planning_observed": planning_observed,
            "trajectory_execution_observed": execution_observed,
            "final_pose_verification_observed": verification_observed,
            "initial_pose_reported": initial_pose_reported,
            "target_pose_reported": target_pose_reported,
            "final_pose_reported": final_pose_reported,
            "pose_evidence_consistent": pose_evidence_consistent,
            "error_code": payload.get("error_code"),
            "message": payload.get("message"),
            "initial_pose": initial_pose,
            "target_pose": target_pose,
            "final_pose": final_pose,
            "final_error_m": final_error_m,
        },
        artifacts=[artifact_uri],
        errors=errors,
        artifact_directory=str(directory),
        acknowledgement_stage=acknowledgement,
        normalized_arguments=normalized_arguments,
        arguments_sha256=arguments_sha256,
        initial_pose=initial_pose if initial_pose_reported else None,
        target_pose=target_pose if target_pose_reported else None,
        final_pose=final_pose if final_pose_reported else None,
        final_error_m=final_error_m,
    )


__all__ = ["run_openarm_move_relative_via_moveit"]
