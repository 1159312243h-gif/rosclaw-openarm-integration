"""OpenArm Cartesian relative-motion executor for the official sandbox."""

from __future__ import annotations

import hashlib
import json
import math
import re
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

CAPABILITY_ID = "openarm.arm.move_relative"
ROBOT_ID = "openarm_dual_mujoco_01"
MAX_TRANSLATION_M = 0.02
MAX_POSITION_TOLERANCE_M = 0.0005
_SAFE_ACTION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_ALLOWED_ARGUMENTS = {
    "arm",
    "reference_frame",
    "translation",
    "velocity_scale",
    "acceleration_scale",
    "avoid_collisions",
}


def _failure(
    *,
    state: ActionState,
    code: str,
    message: str,
    policy_decision: dict[str, Any] | None = None,
) -> ActionExecutionResult:
    return ActionExecutionResult(
        final_state=state,
        evidence_level=EvidenceLevel.REQUESTED,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision=policy_decision or {"allowed": False, "reason": code.lower()},
        dispatch_result={"accepted": False, "physics_executed": False},
        verification_result={"passed": False, "message": message},
        errors=[{"code": code, "message": message}],
        acknowledgement_stage=AcknowledgementStage.REQUEST_ACCEPTED,
    )


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_openarm_move_relative(action: ActionEnvelope) -> dict[str, Any]:
    """Validate and normalize the OpenArm sandbox action contract."""

    if action.body_id != ROBOT_ID:
        raise ValueError(f"unsupported body_id: {action.body_id}")
    if action.capability_id != CAPABILITY_ID:
        raise ValueError(f"unsupported capability_id: {action.capability_id}")
    if action.execution_mode is not ExecutionMode.SIMULATION:
        raise ValueError("OpenArm relative motion is enabled only in SIMULATION")

    unknown = set(action.arguments) - _ALLOWED_ARGUMENTS
    if unknown:
        raise ValueError(f"unknown arguments: {sorted(unknown)}")

    arm = str(action.arguments.get("arm", "")).strip().lower()
    if arm not in {"left", "right"}:
        raise ValueError(f"unsupported arm: {arm}")
    reference_frame = str(action.arguments.get("reference_frame", "world")).strip()
    if reference_frame != "world":
        raise ValueError(f"unsupported reference_frame: {reference_frame}")

    translation = action.arguments.get("translation")
    if not isinstance(translation, dict) or set(translation) != {"x", "y", "z"}:
        raise ValueError("translation must contain exactly x, y, and z")
    vector = {axis: _number(translation[axis], f"translation.{axis}") for axis in "xyz"}
    norm = math.sqrt(sum(value * value for value in vector.values()))
    if norm <= 0.0:
        raise ValueError("translation must be non-zero")
    if any(abs(value) > MAX_TRANSLATION_M for value in vector.values()):
        raise ValueError("translation component exceeds 0.02 m safety limit")
    if norm > MAX_TRANSLATION_M:
        raise ValueError("translation norm exceeds 0.02 m safety limit")

    velocity_scale = _number(action.arguments.get("velocity_scale", 0.05), "velocity_scale")
    acceleration_scale = _number(
        action.arguments.get("acceleration_scale", 0.05), "acceleration_scale"
    )
    if not 0.01 <= velocity_scale <= 0.10:
        raise ValueError("velocity_scale must be between 0.01 and 0.10")
    if not 0.01 <= acceleration_scale <= 0.10:
        raise ValueError("acceleration_scale must be between 0.01 and 0.10")
    avoid_collisions = action.arguments.get("avoid_collisions", False)
    if not isinstance(avoid_collisions, bool):
        raise ValueError("avoid_collisions must be boolean")

    return {
        "arm": arm,
        "reference_frame": reference_frame,
        "translation": vector,
        "translation_norm_m": norm,
        "velocity_scale": velocity_scale,
        "acceleration_scale": acceleration_scale,
        "avoid_collisions": avoid_collisions,
    }


def _write_json(path: Path, payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True).encode("utf-8")
    path.write_bytes(encoded)
    return hashlib.sha256(encoded).hexdigest()


def _contacts(model: Any, data: Any, mujoco: Any) -> list[dict[str, Any]]:
    contacts: list[dict[str, Any]] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        contacts.append(
            {
                "geom1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                or f"geom{contact.geom1}",
                "geom2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                or f"geom{contact.geom2}",
                "distance": float(contact.dist),
            }
        )
    return contacts


def _initialize_observation(
    model: Any,
    data: Any,
    mujoco: Any,
    body_ids: list[int],
) -> tuple[Any, list[dict[str, Any]]]:
    """Synchronize MuJoCo state before recording the motion baseline."""

    mujoco.mj_forward(model, data)
    position = sum(data.xpos[body_id].copy() for body_id in body_ids) / len(body_ids)
    return position, _contacts(model, data, mujoco)


def run_openarm_move_relative(
    sandbox: Any | None,
    action: ActionEnvelope,
    *,
    artifact_root: Path,
) -> ActionExecutionResult:
    """Move one existing OpenArm MJCF wrist by bounded differential IK."""

    try:
        request = validate_openarm_move_relative(action)
    except ValueError as exc:
        return _failure(
            state=ActionState.BLOCKED,
            code="OPENARM_ACTION_REJECTED",
            message=str(exc),
            policy_decision={
                "validation_type": "OpenArmStaticPolicyValidation",
                "allowed": False,
                "reason": str(exc),
                "simulation_executed": False,
            },
        )

    if sandbox is None or not sandbox.has_physics:
        reason = sandbox.load_error if sandbox is not None else "Sandbox service is unavailable."
        return _failure(
            state=ActionState.FAILED,
            code="PHYSICS_UNAVAILABLE",
            message=reason or "OpenArm MuJoCo physics is unavailable.",
        )

    if not _SAFE_ACTION_ID.fullmatch(action.action_id):
        return _failure(
            state=ActionState.BLOCKED,
            code="INVALID_ACTION_ID",
            message="action_id contains unsupported characters",
        )

    import mujoco
    import numpy as np

    model = sandbox.physics_model
    data = sandbox.physics_data
    if model is None or data is None:
        return _failure(
            state=ActionState.FAILED,
            code="PHYSICS_UNAVAILABLE",
            message="Sandbox did not expose its MuJoCo model and data.",
        )

    arm = request["arm"]
    body_names = [
        f"openarm_{arm}_right_finger",
        f"openarm_{arm}_left_finger",
    ]
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in body_names]
    if any(body_id < 0 for body_id in body_ids):
        return _failure(
            state=ActionState.FAILED,
            code="END_EFFECTOR_BODY_MISSING",
            message="OpenArm MJCF does not contain both gripper finger bodies.",
        )
    body_name = f"openarm_{arm}_gripper_center"

    joint_names = [f"openarm_{arm}_joint{index}" for index in range(1, 8)]
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    if any(joint_id < 0 for joint_id in joint_ids):
        return _failure(
            state=ActionState.FAILED,
            code="ARM_JOINTS_MISSING",
            message=f"OpenArm MJCF does not contain all seven {arm} arm joints.",
        )

    qpos_indices = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    dof_indices = [int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids]
    commanded_qpos_indices = set(qpos_indices)
    frozen_qpos_indices = [
        index for index in range(model.nq) if index not in commanded_qpos_indices
    ]
    initial_qpos = data.qpos.copy()
    initial_qvel = data.qvel.copy()
    initial_time = float(data.time)
    initial_position, initial_contacts = _initialize_observation(
        model,
        data,
        mujoco,
        body_ids,
    )
    delta = np.asarray([request["translation"][axis] for axis in "xyz"], dtype=float)
    target = initial_position + delta
    position_tolerance = min(
        MAX_POSITION_TOLERANCE_M,
        request["translation_norm_m"] * 0.25,
    )
    initial_penetrations = {
        frozenset((item["geom1"], item["geom2"]))
        for item in initial_contacts
        if item["distance"] < -0.003
    }

    max_iterations = 400
    damping = 0.04
    motion_scale = min(request["velocity_scale"], request["acceleration_scale"])
    max_joint_step = 0.003 + 0.12 * motion_scale
    trajectory: list[dict[str, Any]] = []
    blocked_contact: dict[str, Any] | None = None
    success = False

    for iteration in range(max_iterations + 1):
        current = sum(data.xpos[body_id].copy() for body_id in body_ids) / len(body_ids)
        error = target - current
        error_norm = float(np.linalg.norm(error))
        if iteration % 10 == 0 or error_norm <= position_tolerance:
            trajectory.append(
                {
                    "iteration": iteration,
                    "time": float(data.time),
                    "end_effector": current.tolist(),
                    "error_m": error_norm,
                    "joint_positions": [float(data.qpos[index]) for index in qpos_indices],
                }
            )
        if error_norm <= position_tolerance:
            success = True
            break

        jacobian_pos = np.zeros((3, model.nv))
        for body_id in body_ids:
            body_jacobian_pos = np.zeros((3, model.nv))
            body_jacobian_rot = np.zeros((3, model.nv))
            mujoco.mj_jacBody(
                model,
                data,
                body_jacobian_pos,
                body_jacobian_rot,
                body_id,
            )
            jacobian_pos += body_jacobian_pos / len(body_ids)
        jacobian = jacobian_pos[:, dof_indices]
        joint_delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping**2 * np.eye(3), error
        )
        joint_delta = np.clip(joint_delta, -max_joint_step, max_joint_step)

        for offset, joint_id in enumerate(joint_ids):
            qpos_index = qpos_indices[offset]
            lower, upper = model.jnt_range[joint_id]
            data.qpos[qpos_index] = float(
                np.clip(data.qpos[qpos_index] + joint_delta[offset], lower, upper)
            )
            data.qvel[dof_indices[offset]] = 0.0

        mujoco.mj_forward(model, data)
        qpos_before_step = data.qpos.copy()
        mujoco.mj_step(model, data)
        data.qpos[frozen_qpos_indices] = qpos_before_step[frozen_qpos_indices]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        if request["avoid_collisions"]:
            for contact in _contacts(model, data, mujoco):
                pair = frozenset((contact["geom1"], contact["geom2"]))
                if contact["distance"] < -0.003 and pair not in initial_penetrations:
                    blocked_contact = contact
                    break
            if blocked_contact is not None:
                break

    final_position = sum(data.xpos[body_id].copy() for body_id in body_ids) / len(body_ids)
    final_error = float(np.linalg.norm(target - final_position))
    final_contacts = _contacts(model, data, mujoco)
    if blocked_contact is not None:
        success = False

    state_rolled_back = False
    if not success:
        data.qpos[:] = initial_qpos
        data.qvel[:] = initial_qvel
        data.time = initial_time
        mujoco.mj_forward(model, data)
        state_rolled_back = True

    artifact_dir = (artifact_root / action.action_id).resolve()
    artifact_root = artifact_root.resolve()
    if not artifact_dir.is_relative_to(artifact_root):
        return _failure(
            state=ActionState.BLOCKED,
            code="INVALID_ARTIFACT_PATH",
            message="action artifact path escaped the configured root",
        )
    artifact_dir.mkdir(parents=True, exist_ok=True)
    model_path = sandbox.model_path
    model_hash = (
        hashlib.sha256(model_path.read_bytes()).hexdigest()
        if model_path is not None and model_path.is_file()
        else ""
    )
    action_path = artifact_dir / "action.json"
    trajectory_path = artifact_dir / "trajectory.json"
    action_hash = _write_json(action_path, action.to_dict())
    trajectory_hash = _write_json(
        trajectory_path,
        {
            "schema_version": "rosclaw.openarm.sandbox.trajectory.v1",
            "body": body_name,
            "initial_position": initial_position.tolist(),
            "target_position": target.tolist(),
            "final_position": final_position.tolist(),
            "final_error_m": final_error,
            "trajectory": trajectory,
            "blocked_contact": blocked_contact,
        },
    )

    policy = {
        "validation_type": "OpenArmStaticPolicyValidation",
        "allowed": blocked_contact is None,
        "reason": "bounded_relative_motion" if blocked_contact is None else "simulated_collision",
        "translation_norm_m": request["translation_norm_m"],
        "simulation_executed": True,
    }
    verification = {
        "passed": success,
        "condition": "world_frame_cartesian_error_within_tolerance",
        "initial_position": initial_position.tolist(),
        "target_position": target.tolist(),
        "final_position": final_position.tolist(),
        "final_error_m": final_error,
        "tolerance_m": position_tolerance,
        "requested_translation_m": delta.tolist(),
        "achieved_translation_m": (final_position - initial_position).tolist(),
        "blocked_contact": blocked_contact,
    }
    simulation_result = {
        "backend": "mujoco",
        "has_physics": True,
        "physics_executed": True,
        "kinematics": "damped_least_squares_body_jacobian",
        "non_target_joints_frozen": True,
        "body": body_name,
        "model_path": str(model_path) if model_path else None,
        "model_hash": model_hash,
        "iterations": trajectory[-1]["iteration"] if trajectory else 0,
        "final_contacts_count": len(final_contacts),
        "state_rolled_back": state_rolled_back,
        "artifact_hashes": {
            action_path.name: action_hash,
            trajectory_path.name: trajectory_hash,
        },
    }

    if blocked_contact is not None:
        state = ActionState.BLOCKED
        evidence = EvidenceLevel.PHYSICALLY_OBSERVED
        errors = [{"code": "SIMULATED_COLLISION", "message": str(blocked_contact)}]
    elif success:
        state = ActionState.COMPLETED
        evidence = EvidenceLevel.TASK_VERIFIED
        errors = []
    else:
        state = ActionState.FAILED
        evidence = EvidenceLevel.PHYSICALLY_OBSERVED
        errors = [
            {
                "code": "OPENARM_IK_NOT_CONVERGED",
                "message": (
                    f"Final Cartesian error {final_error:.6f} m exceeds {position_tolerance:.6f} m."
                ),
            }
        ]

    return ActionExecutionResult(
        final_state=state,
        evidence_level=evidence,
        evidence_domain=EvidenceDomain.SIMULATION,
        policy_decision=policy,
        simulation_result=simulation_result,
        dispatch_result={
            "accepted": blocked_contact is None,
            "transport": "official_sandbox",
            "physics_executed": True,
        },
        observations=[
            {"kind": "initial_end_effector", "value": initial_position.tolist()},
            {"kind": "final_end_effector", "value": final_position.tolist()},
        ],
        verification_result=verification,
        artifacts=[action_path.as_uri(), trajectory_path.as_uri()],
        errors=errors,
        artifact_directory=str(artifact_dir),
        acknowledgement_stage=(
            AcknowledgementStage.TASK_VERIFIED if success else AcknowledgementStage.EFFECT_OBSERVED
        ),
    )


__all__ = [
    "CAPABILITY_ID",
    "MAX_POSITION_TOLERANCE_M",
    "ROBOT_ID",
    "run_openarm_move_relative",
    "validate_openarm_move_relative",
]
