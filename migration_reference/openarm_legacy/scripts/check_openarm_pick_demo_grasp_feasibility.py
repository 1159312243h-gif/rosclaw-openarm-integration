#!/usr/bin/env python3
"""Offline gate for collision-free opening and reachable two-finger contact."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco

OBJECT_BODY = "pick_demo_block"
OBJECT_GEOM = "pick_demo_block_geom"
OPEN_POSITION_M = 0.000
CLOSE_POSITION_M = 0.006
SETTLE_DURATION_S = 5.0
CLOSE_RAMP_DURATION_S = 1.0
CLOSE_HOLD_DURATION_S = 2.0
FINAL_WINDOW_DURATION_S = 0.5
GOAL_TOLERANCE_M = 0.0005
VELOCITY_TOLERANCE_MPS = 0.001
MAX_FINAL_DRIFT_M = 0.001
MAX_FINAL_OFFSET_M = 0.010
EXPECTED_OBJECT_POSITION = (0.11697, -0.15350, 0.52183)
RIGHT_FINGER_GEOMS = {
    "openarm_right_left_finger_collision",
    "openarm_right_right_finger_collision",
}
ALLOWED_OBJECT_CONTACT_GEOMS = {
    "pick_demo_support_geom",
    "pick_demo_cradle_x_negative",
    "pick_demo_cradle_x_positive",
    "pick_demo_cradle_y_negative",
    "pick_demo_cradle_y_positive",
    *RIGHT_FINGER_GEOMS,
}
ARM_POSITIONS = {
    **{
        f"openarm_left_joint{index}": value
        for index, value in enumerate((0.0, -0.35, 0.0, 0.7, 0.0, -0.35, 0.0), start=1)
    },
    **{
        f"openarm_right_joint{index}": value
        for index, value in enumerate((0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0), start=1)
    },
}


def _distance(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))


def _joint_addresses(model: mujoco.MjModel, name: str) -> tuple[int, int]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"joint is missing from scene: {name}")
    return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])


def _body_position(data: mujoco.MjData, body_id: int) -> tuple[float, float, float]:
    return tuple(float(value) for value in data.xpos[body_id])


def _object_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> set[str]:
    names: set[str] = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        pair = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)),
        }
        if OBJECT_GEOM not in pair:
            continue
        other = next(name for name in pair if name != OBJECT_GEOM)
        names.add(other or "<unnamed_geom>")
    return names


def _step_arms(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm_addresses: list[tuple[int, int, float]],
) -> None:
    for qpos_address, dof_address, target in arm_addresses:
        current = float(data.qpos[qpos_address])
        data.qpos[qpos_address] = current + (target - current) * 0.25
        data.qvel[dof_address] = 0.0
    mujoco.mj_step(model, data)


def check_grasp(model_path: Path) -> dict[str, object]:
    model_path = model_path.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for joint_id in range(model.njnt):
        dof_id = int(model.jnt_dofadr[joint_id])
        if dof_id >= 0:
            model.dof_damping[dof_id] = 0.5
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0

    arm_addresses: list[tuple[int, int, float]] = []
    for name, target in ARM_POSITIONS.items():
        qpos_address, dof_address = _joint_addresses(model, name)
        data.qpos[qpos_address] = target
        arm_addresses.append((qpos_address, dof_address, target))
    finger_addresses = []
    for side in ("left", "right"):
        for index in (1, 2):
            qpos_address, dof_address = _joint_addresses(
                model, f"openarm_{side}_finger_joint{index}"
            )
            data.qpos[qpos_address] = OPEN_POSITION_M
            data.qvel[dof_address] = 0.0
            if side == "right":
                finger_addresses.append((qpos_address, dof_address))

    actuator_ids = {}
    for side in ("left", "right"):
        name = f"openarm_{side}_gripper_position"
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise ValueError(f"actuator is missing from scene: {name}")
        actuator_ids[side] = int(actuator_id)
        data.ctrl[actuator_id] = OPEN_POSITION_M

    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY)
    if body_id < 0:
        raise ValueError(f"{OBJECT_BODY} is missing from scene")

    preclose_finger_contacts: set[str] = set()
    settle_steps = math.ceil(SETTLE_DURATION_S / float(model.opt.timestep))
    for _ in range(settle_steps):
        _step_arms(model, data, arm_addresses)
        preclose_finger_contacts.update(_object_contacts(model, data) & RIGHT_FINGER_GEOMS)
    settled_position = _body_position(data, body_id)

    total_close_duration = CLOSE_RAMP_DURATION_S + CLOSE_HOLD_DURATION_S
    close_steps = math.ceil(total_close_duration / float(model.opt.timestep))
    final_window_contacts: set[str] = set()
    unexpected_contacts: set[str] = set()
    final_window_reference: tuple[float, float, float] | None = None
    max_final_drift = 0.0
    for step in range(close_steps):
        elapsed = (step + 1) * float(model.opt.timestep)
        fraction = min(1.0, elapsed / CLOSE_RAMP_DURATION_S)
        data.ctrl[actuator_ids["right"]] = CLOSE_POSITION_M * fraction
        _step_arms(model, data, arm_addresses)
        contacts = _object_contacts(model, data)
        unexpected_contacts.update(contacts - ALLOWED_OBJECT_CONTACT_GEOMS)
        if elapsed >= total_close_duration - FINAL_WINDOW_DURATION_S:
            final_window_contacts.update(contacts)
            position = _body_position(data, body_id)
            if final_window_reference is None:
                final_window_reference = position
            max_final_drift = max(
                max_final_drift, _distance(position, final_window_reference)
            )

    final_position = _body_position(data, body_id)
    joint_positions = [float(data.qpos[address]) for address, _ in finger_addresses]
    joint_velocities = [abs(float(data.qvel[address])) for _, address in finger_addresses]
    max_goal_error = max(abs(CLOSE_POSITION_M - value) for value in joint_positions)
    max_velocity = max(joint_velocities)
    final_finger_contacts = final_window_contacts & RIGHT_FINGER_GEOMS
    final_offset = _distance(final_position, EXPECTED_OBJECT_POSITION)
    passed = bool(
        not preclose_finger_contacts
        and not unexpected_contacts
        and final_finger_contacts == RIGHT_FINGER_GEOMS
        and max_goal_error <= GOAL_TOLERANCE_M
        and max_velocity <= VELOCITY_TOLERANCE_MPS
        and max_final_drift <= MAX_FINAL_DRIFT_M
        and final_offset <= MAX_FINAL_OFFSET_M
    )
    result = {
        "schema_version": "openarm.pick_demo.grasp_feasibility.v1",
        "model": str(model_path),
        "open_position_m": OPEN_POSITION_M,
        "close_position_m": CLOSE_POSITION_M,
        "preclose_finger_contacts": sorted(preclose_finger_contacts),
        "settled_object_position_m": list(settled_position),
        "final_object_position_m": list(final_position),
        "final_object_offset_m": final_offset,
        "final_window_finger_contacts": sorted(final_finger_contacts),
        "unexpected_object_contacts": sorted(unexpected_contacts),
        "final_right_finger_joint_positions_m": joint_positions,
        "maximum_gripper_goal_error_m": max_goal_error,
        "maximum_gripper_velocity_mps": max_velocity,
        "maximum_final_object_drift_m": max_final_drift,
        "goal_tolerance_m": GOAL_TOLERANCE_M,
        "velocity_tolerance_mps": VELOCITY_TOLERANCE_MPS,
        "maximum_allowed_final_drift_m": MAX_FINAL_DRIFT_M,
        "maximum_allowed_final_offset_m": MAX_FINAL_OFFSET_M,
        "status": "PASS" if passed else "FAIL",
    }
    if not passed:
        raise RuntimeError(json.dumps(result, indent=2, sort_keys=True))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(check_grasp(args.model), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
