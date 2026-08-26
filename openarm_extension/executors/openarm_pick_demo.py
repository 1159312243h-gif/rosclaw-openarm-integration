"""Task-level MoveIt executor for the deterministic OpenArm pick demo."""

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

CAPABILITY_ID = "openarm.pick_demo"
OBJECT_ID = "pick_demo_block"
MIN_LIFT_M = 0.010
MAX_LIFT_M = 0.020
MIN_OBJECT_RISE_M = 0.008
_WORKER_SCHEMA = "openarm.pick_demo.worker.v1"
_STATUS_SUCCEEDED = 4
_ALLOWED_ARGUMENTS = {"side", "object_id", "lift_m"}
WorkerTransport = Callable[[dict[str, Any]], dict[str, Any]]


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def validate_openarm_pick_demo(action: ActionEnvelope) -> dict[str, Any]:
    """Validate the deliberately narrow, simulation-only demo contract."""

    if action.body_id != ROBOT_ID:
        raise ValueError(f"unsupported body_id: {action.body_id}")
    if action.capability_id != CAPABILITY_ID:
        raise ValueError(f"unsupported capability_id: {action.capability_id}")
    if action.execution_mode is not ExecutionMode.SIMULATION:
        raise ValueError("OpenArm pick demo is enabled only in SIMULATION")
    unknown = set(action.arguments) - _ALLOWED_ARGUMENTS
    if unknown:
        raise ValueError(f"unknown arguments: {sorted(unknown)}")
    side = str(action.arguments.get("side", "")).strip().lower()
    if side != "right":
        raise ValueError("the first deterministic pick demo supports only side='right'")
    object_id = str(action.arguments.get("object_id", "")).strip()
    if object_id != OBJECT_ID:
        raise ValueError(f"object_id must be {OBJECT_ID!r}")
    lift_m = _number(action.arguments.get("lift_m", 0.015), "lift_m")
    if not MIN_LIFT_M <= lift_m <= MAX_LIFT_M:
        raise ValueError("lift_m must be between 0.010 and 0.020 m")
    return {"side": side, "object_id": object_id, "lift_m": lift_m}


def _failure(
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
            "validation_type": "OpenArmPickDemoStaticPolicyAndTaskEvidenceValidation",
            "allowed": False,
            "reason": message,
            "simulation_only": True,
            "automatic_retry": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "physics_executed": False,
            "direct_mujoco_fallback": False,
        },
        dispatch_result={"accepted": accepted, "transport": "isolated_ros2_pick_worker"},
        verification_result={"passed": False, "message": message},
        errors=[{"code": code, "message": message}],
        artifact_directory=str(artifact_directory) if artifact_directory else None,
        acknowledgement_stage=AcknowledgementStage.REQUEST_ACCEPTED,
    )


def _finite(payload: dict[str, Any], name: str) -> float | None:
    value = payload.get(name)
    if (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    ):
        return float(value)
    return None


def run_openarm_pick_demo_via_moveit(
    action: ActionEnvelope,
    *,
    artifact_root: Path,
    worker_command: str | None,
    server_timeout_sec: float = 5.0,
    result_timeout_sec: float = 60.0,
    cancel_timeout_sec: float = 3.0,
    transport: WorkerTransport | None = None,
) -> ActionExecutionResult:
    """Execute one atomic demo request and require object-rise evidence."""

    try:
        request = validate_openarm_pick_demo(action)
    except ValueError as exc:
        return _failure("OPENARM_PICK_DEMO_REJECTED", str(exc), state=ActionState.BLOCKED)

    try:
        directory = _artifact_directory(artifact_root, action.action_id)
    except FileExistsError as exc:
        return _failure("OPENARM_ACTION_ID_ALREADY_EXISTS", str(exc))
    except (OSError, ValueError) as exc:
        return _failure("OPENARM_ARTIFACT_PATH_INVALID", str(exc))

    normalized = dict(request)
    arguments_sha256 = _canonical_arguments_sha256(normalized)
    worker_request = {
        "schema_version": _WORKER_SCHEMA,
        "request_id": action.action_id,
        **normalized,
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
                    + (4.0 * result_timeout_sec)
                    + cancel_timeout_sec
                    + 15.0
                ),
            )
        artifact_uri, artifact_sha256 = _write_worker_artifact(directory, payload)
    except subprocess.TimeoutExpired:
        return _failure(
            "PICK_DEMO_WORKER_TIMEOUT",
            "pick worker exceeded its fail-closed timeout",
            state=ActionState.TIMED_OUT,
            artifact_directory=directory,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        return _failure("PICK_DEMO_TRANSPORT_UNAVAILABLE", str(exc), artifact_directory=directory)

    if payload.get("schema_version") != _WORKER_SCHEMA:
        return _failure("PICK_DEMO_WORKER_SCHEMA_INVALID", "unsupported pick worker schema", artifact_directory=directory)
    if payload.get("request_id") != action.action_id:
        return _failure("PICK_DEMO_WORKER_IDENTITY_MISMATCH", "pick worker request_id mismatch", artifact_directory=directory)

    stages = payload.get("stages") if isinstance(payload.get("stages"), list) else []
    required_stages = {"scene_ready", "gripper_open", "approach", "gripper_close", "lift", "verify"}
    successful_stages = {
        str(stage.get("name"))
        for stage in stages
        if isinstance(stage, dict) and stage.get("success") is True
    }
    initial_z = _finite(payload, "initial_object_z_m")
    final_z = _finite(payload, "final_object_z_m")
    rise = _finite(payload, "object_rise_m")
    pose_consistent = bool(
        initial_z is not None
        and final_z is not None
        and rise is not None
        and abs((final_z - initial_z) - rise) <= 1.0e-6
    )
    evidence_complete = bool(
        required_stages <= successful_stages
        and pose_consistent
        and rise is not None
        and rise >= MIN_OBJECT_RISE_M
    )
    accepted = payload.get("accepted") is True
    timed_out = payload.get("timed_out") is True
    success = bool(
        accepted
        and not timed_out
        and payload.get("status") == _STATUS_SUCCEEDED
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
        errors = [{"code": "PICK_DEMO_RESULT_TIMEOUT", "message": str(payload.get("message", ""))}]
    else:
        state = ActionState.FAILED
        evidence = EvidenceLevel.PHYSICALLY_OBSERVED if "lift" in successful_stages else EvidenceLevel.REQUESTED
        acknowledgement = AcknowledgementStage.EFFECT_OBSERVED if "lift" in successful_stages else AcknowledgementStage.REQUEST_ACCEPTED
        code = "PICK_DEMO_EVIDENCE_INCOMPLETE" if payload.get("success") is True else "PICK_DEMO_FAILED"
        errors = [{"code": code, "message": str(payload.get("message", state.value))}]

    return ActionExecutionResult(
        final_state=state,
        evidence_level=evidence,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision={
            "validation_type": "OpenArmPickDemoStaticPolicyAndTaskEvidenceValidation",
            "allowed": True,
            "simulation_only": True,
            "automatic_retry": False,
        },
        simulation_result={
            "backend": "mujoco_moveit",
            "planning_backend": "moveit2",
            "physics_executed": any(name in successful_stages for name in {"approach", "gripper_close", "lift"}),
            "direct_mujoco_fallback": False,
            "worker_artifact_sha256": artifact_sha256,
        },
        dispatch_result={"accepted": accepted, "transport": "isolated_ros2_pick_worker", "request_id": action.action_id},
        observations=stages,
        verification_result={
            "passed": success,
            "object_id": request["object_id"],
            "initial_object_z_m": initial_z,
            "final_object_z_m": final_z,
            "object_rise_m": rise,
            "minimum_object_rise_m": MIN_OBJECT_RISE_M,
            "pose_evidence_consistent": pose_consistent,
            "required_stages_verified": required_stages <= successful_stages,
            "message": payload.get("message"),
        },
        artifacts=[artifact_uri],
        errors=errors,
        artifact_directory=str(directory),
        acknowledgement_stage=acknowledgement,
        normalized_arguments=normalized,
        arguments_sha256=arguments_sha256,
        final_error_m=None if rise is None else max(0.0, request["lift_m"] - rise),
    )


__all__ = ["CAPABILITY_ID", "run_openarm_pick_demo_via_moveit", "validate_openarm_pick_demo"]
