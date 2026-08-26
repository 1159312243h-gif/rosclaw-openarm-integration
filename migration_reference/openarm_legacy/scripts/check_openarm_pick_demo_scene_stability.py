#!/usr/bin/env python3
"""Offline long-duration stability gate for the derived pick-demo scene."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco

EXPECTED_OBJECT_POSITION = (0.11697, -0.15350, 0.52183)
MAX_INITIAL_OFFSET_M = 0.010
MAX_STABLE_DRIFT_M = 0.001
SETTLE_DURATION_S = 5.0
OBSERVATION_DURATION_S = 60.0
POSITION_GAIN = 0.25
OPEN_GRIPPER_POSITION_M = 0.000
OBJECT_GEOM = "pick_demo_block_geom"
ALLOWED_OBJECT_CONTACT_GEOMS = {
    "pick_demo_support_geom",
    "pick_demo_cradle_x_negative",
    "pick_demo_cradle_x_positive",
    "pick_demo_cradle_y_negative",
    "pick_demo_cradle_y_positive",
}

INITIAL_JOINT_POSITIONS = {
    **{
        f"openarm_left_joint{index}": value
        for index, value in enumerate((0.0, -0.35, 0.0, 0.7, 0.0, -0.35, 0.0), start=1)
    },
    **{
        f"openarm_right_joint{index}": value
        for index, value in enumerate((0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0), start=1)
    },
    "openarm_left_finger_joint1": OPEN_GRIPPER_POSITION_M,
    "openarm_left_finger_joint2": OPEN_GRIPPER_POSITION_M,
    "openarm_right_finger_joint1": OPEN_GRIPPER_POSITION_M,
    "openarm_right_finger_joint2": OPEN_GRIPPER_POSITION_M,
}


def _distance(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))


def _body_position(data: mujoco.MjData, body_id: int) -> tuple[float, float, float]:
    return tuple(float(value) for value in data.xpos[body_id])


def _body_quaternion(data: mujoco.MjData, body_id: int) -> tuple[float, float, float, float]:
    """Return the world-frame body quaternion in MuJoCo's wxyz order."""
    return tuple(float(value) for value in data.xquat[body_id])


def _quaternion_angle(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    dot = abs(sum(a * b for a, b in zip(left, right, strict=True)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _controlled_joint_addresses(model: mujoco.MjModel) -> list[tuple[int, int, float]]:
    addresses: list[tuple[int, int, float]] = []
    for name, target in INITIAL_JOINT_POSITIONS.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"controlled joint is missing from scene: {name}")
        addresses.append(
            (
                int(model.jnt_qposadr[joint_id]),
                int(model.jnt_dofadr[joint_id]),
                target,
            )
        )
    return addresses


def _joint_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"joint is missing from scene: {name}")
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _record_object_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    records: dict[str, dict[str, object]],
    *,
    phase: str,
) -> None:
    for index in range(data.ncon):
        contact = data.contact[index]
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)),
        }
        if OBJECT_GEOM not in names:
            continue
        other = next(name for name in names if name != OBJECT_GEOM)
        other = other or "<unnamed_geom>"
        distance = float(contact.dist)
        record = records.get(other)
        if record is None:
            record = {
                "allowed": other in ALLOWED_OBJECT_CONTACT_GEOMS,
                "first_phase": phase,
                "first_time_s": float(data.time),
                "last_phase": phase,
                "last_time_s": float(data.time),
                "first_contact_distance_m": distance,
                "minimum_contact_distance_m": distance,
                "minimum_contact_distance_phase": phase,
                "minimum_contact_distance_time_s": float(data.time),
                "maximum_penetration_m": max(0.0, -distance),
                "contact_point_sample_count": 0,
                "first_object_position_m": list(_body_position(data, body_id)),
                "first_object_quaternion_wxyz": list(_body_quaternion(data, body_id)),
                "first_right_gripper_joint_positions_m": {
                    "openarm_right_finger_joint1": _joint_position(
                        model, data, "openarm_right_finger_joint1"
                    ),
                    "openarm_right_finger_joint2": _joint_position(
                        model, data, "openarm_right_finger_joint2"
                    ),
                },
            }
            records[other] = record
        record["last_phase"] = phase
        record["last_time_s"] = float(data.time)
        if distance < float(record["minimum_contact_distance_m"]):
            record["minimum_contact_distance_m"] = distance
            record["minimum_contact_distance_phase"] = phase
            record["minimum_contact_distance_time_s"] = float(data.time)
        record["maximum_penetration_m"] = max(
            float(record["maximum_penetration_m"]), max(0.0, -distance)
        )
        record["contact_point_sample_count"] = (
            int(record["contact_point_sample_count"]) + 1
        )


def _initial_gripper_contact_sweep(model: mujoco.MjModel) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    finger_geoms = (
        "openarm_right_left_finger_collision",
        "openarm_right_right_finger_collision",
    )
    for opening in (0.000, 0.003, 0.006, 0.010):
        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0.0
        for name, configured_target in INITIAL_JOINT_POSITIONS.items():
            target = opening if name.startswith("openarm_right_finger_joint") else configured_target
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[int(model.jnt_qposadr[joint_id])] = target
        mujoco.mj_forward(model, data)

        contact_distances: dict[str, float] = {}
        for index in range(data.ncon):
            contact = data.contact[index]
            names = {
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)),
            }
            if OBJECT_GEOM not in names:
                continue
            other = next(name for name in names if name != OBJECT_GEOM) or "<unnamed_geom>"
            if other not in finger_geoms:
                continue
            distance = float(contact.dist)
            contact_distances[other] = min(contact_distances.get(other, distance), distance)

        geom_centers: dict[str, list[float]] = {}
        for name in finger_geoms:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            geom_centers[name] = [float(value) for value in data.geom_xpos[geom_id]]
        results.append(
            {
                "opening_m": opening,
                "finger_contact_active": bool(contact_distances),
                "finger_contact_distances_m": {
                    name: contact_distances[name] for name in sorted(contact_distances)
                },
                "finger_geom_centers_world_m": geom_centers,
            }
        )
    return results


def _step_controlled(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    addresses: list[tuple[int, int, float]],
) -> None:
    for qpos_address, dof_address, target in addresses:
        current = float(data.qpos[qpos_address])
        data.qpos[qpos_address] = current + (target - current) * POSITION_GAIN
        data.qvel[dof_address] = 0.0
    mujoco.mj_step(model, data)


def check_scene(
    model_path: Path,
    *,
    raise_on_failure: bool = True,
) -> dict[str, object]:
    model_path = model_path.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    # Match MujocoMoveItBridge initialization before evaluating the scene.
    for joint_id in range(model.njnt):
        dof_id = int(model.jnt_dofadr[joint_id])
        if dof_id >= 0:
            model.dof_damping[dof_id] = 0.5
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    addresses = _controlled_joint_addresses(model)
    for qpos_address, dof_address, target in addresses:
        data.qpos[qpos_address] = target
        data.qvel[dof_address] = 0.0
    mujoco.mj_forward(model, data)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pick_demo_block")
    if body_id < 0:
        raise ValueError("pick_demo_block is missing from scene")

    authored_position = _body_position(data, body_id)
    authored_quaternion = _body_quaternion(data, body_id)
    contact_records: dict[str, dict[str, object]] = {}
    _record_object_contacts(
        model,
        data,
        body_id,
        contact_records,
        phase="initial_forward",
    )
    settle_steps = math.ceil(SETTLE_DURATION_S / float(model.opt.timestep))
    for _ in range(settle_steps):
        _step_controlled(model, data, addresses)
        _record_object_contacts(model, data, body_id, contact_records, phase="settling")
    settled = _body_position(data, body_id)
    settled_quaternion = _body_quaternion(data, body_id)
    initial_offset = _distance(settled, EXPECTED_OBJECT_POSITION)

    max_drift = 0.0
    max_angular_drift = 0.0
    observation_steps = math.ceil(OBSERVATION_DURATION_S / float(model.opt.timestep))
    for _ in range(observation_steps):
        _step_controlled(model, data, addresses)
        _record_object_contacts(model, data, body_id, contact_records, phase="observation")
        max_drift = max(max_drift, _distance(_body_position(data, body_id), settled))
        max_angular_drift = max(
            max_angular_drift,
            _quaternion_angle(_body_quaternion(data, body_id), settled_quaternion),
        )
    final = _body_position(data, body_id)
    final_quaternion = _body_quaternion(data, body_id)
    final_offset = _distance(final, EXPECTED_OBJECT_POSITION)
    unexpected_contacts = sorted(
        name for name, record in contact_records.items() if record["allowed"] is False
    )
    passed = bool(
        initial_offset <= MAX_INITIAL_OFFSET_M
        and final_offset <= MAX_INITIAL_OFFSET_M
        and max_drift <= MAX_STABLE_DRIFT_M
        and not unexpected_contacts
    )
    result = {
        "schema_version": "openarm.pick_demo.stability.v3",
        "model": str(model_path),
        "simulated_settle_duration_s": SETTLE_DURATION_S,
        "simulated_observation_duration_s": OBSERVATION_DURATION_S,
        "initial_gripper_position_m": OPEN_GRIPPER_POSITION_M,
        "initial_gripper_contact_sweep": _initial_gripper_contact_sweep(model),
        "authored_object_position_m": list(authored_position),
        "authored_object_quaternion_wxyz": list(authored_quaternion),
        "settled_object_position_m": list(settled),
        "settled_object_quaternion_wxyz": list(settled_quaternion),
        "final_object_position_m": list(final),
        "final_object_quaternion_wxyz": list(final_quaternion),
        "initial_offset_m": initial_offset,
        "final_offset_m": final_offset,
        "maximum_stable_drift_m": max_drift,
        "settled_angular_offset_rad": _quaternion_angle(
            settled_quaternion, authored_quaternion
        ),
        "final_angular_offset_rad": _quaternion_angle(
            final_quaternion, authored_quaternion
        ),
        "maximum_stable_angular_drift_rad": max_angular_drift,
        "maximum_allowed_offset_m": MAX_INITIAL_OFFSET_M,
        "maximum_allowed_drift_m": MAX_STABLE_DRIFT_M,
        "object_contact_diagnostics": {
            name: contact_records[name] for name in sorted(contact_records)
        },
        "unexpected_object_contacts": unexpected_contacts,
        "status": "PASS" if passed else "FAIL",
    }
    if not passed and raise_on_failure:
        raise RuntimeError(json.dumps(result, indent=2, sort_keys=True))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="print diagnostics and return zero even when the stability gate fails",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            check_scene(args.model, raise_on_failure=not args.report_only),
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
