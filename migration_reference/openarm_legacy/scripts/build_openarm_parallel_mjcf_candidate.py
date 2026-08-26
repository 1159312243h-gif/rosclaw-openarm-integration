#!/usr/bin/env python3
"""Patch only the gripper kinematics of the validated formal MuJoCo model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


FINGER_JOINTS = tuple(
    f"openarm_{side}_finger_joint{number}"
    for side in ("left", "right")
    for number in (1, 2)
)
LEFT_ARM_JOINTS = tuple(f"openarm_left_joint{index}" for index in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"openarm_right_joint{index}" for index in range(1, 8))
ARM_INITIAL_POSITIONS = dict(
    zip(
        LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS,
        (
            0.0,
            -0.35,
            0.0,
            0.7,
            0.0,
            -0.35,
            0.0,
            0.0,
            0.0,
            0.0,
            0.3,
            0.0,
            0.0,
            0.0,
        ),
        strict=False,
    )
)
MIMIC_SOLREF = (0.004, 1.0)
MIMIC_TOLERANCE_M = 0.0005
DYNAMIC_TARGETS_M = (0.011, 0.022, 0.044)
SCENE_NAMES = {
    "sun",
    "floor",
    "table",
    "red_block",
    "blue_block",
    "green_block",
    "target_marker",
}
EXPECTED_FORMAL_MJCF_SHA256 = (
    "788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(value: str | None) -> np.ndarray:
    values = np.asarray(
        [float(item) for item in (value or "0 0 0").split()], dtype=float
    )
    if values.shape != (3,):
        raise ValueError(f"expected a three-vector, got {value!r}")
    return values


def format_vector(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def resolve_urdf(urdf: Path, description_root: Path, output: Path) -> None:
    tree = ET.parse(urdf)
    root = tree.getroot()
    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)

    prefix = "package://openarm_description/"
    mesh_dir = output.parent / "unique_collision_meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for index, mesh in enumerate(root.iter("mesh")):
        filename = mesh.get("filename", "")
        if filename.startswith(prefix):
            resolved = description_root / filename[len(prefix) :]
            if not resolved.is_file():
                raise FileNotFoundError(resolved)
            # MuJoCo deduplicates URDF mesh assets by path. Signed scales on
            # repeated finger.stl references then leak between left/right
            # collision geoms, so each temporary reference needs a unique path.
            unique = mesh_dir / f"mesh_{index:03d}{resolved.suffix.lower()}"
            shutil.copyfile(resolved, unique)
            mesh.set("filename", str(unique.resolve()))

    for child in list(root):
        if child.tag in {"ros2_control", "transmission", "gazebo", "mujoco"}:
            root.remove(child)
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {
            "balanceinertia": "true",
            "discardvisual": "true",
            "fusestatic": "false",
            "strippath": "false",
        },
    )
    tree.write(output, encoding="utf-8", xml_declaration=True)


def compile_reference_urdf(urdf: Path, description_root: Path) -> Any:
    with tempfile.TemporaryDirectory(prefix="openarm_parallel_reference_") as temp_dir:
        resolved = Path(temp_dir) / "reference.urdf"
        resolve_urdf(urdf, description_root, resolved)
        return mujoco.MjModel.from_xml_path(str(resolved))


def body_for_joint(root: ET.Element, joint_name: str) -> tuple[ET.Element, ET.Element]:
    for body in root.iter("body"):
        for joint in body.findall("joint"):
            if joint.get("name") == joint_name:
                return body, joint
    raise ValueError(f"body for joint not found: {joint_name}")


def geom_for_joint(model: Any, joint_name: str) -> tuple[int, int]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"joint not found: {joint_name}")
    body_id = int(model.jnt_bodyid[joint_id])
    geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) == body_id
    ]
    if len(geom_ids) != 1:
        raise ValueError(
            f"expected one collision geom for {joint_name}, found {geom_ids}"
        )
    return body_id, geom_ids[0]


def set_side_position(
    model: Any, data: Any, side: str, position: float
) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for number in (1, 2):
        joint_name = f"openarm_{side}_finger_joint{number}"
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        data.qpos[int(model.jnt_qposadr[joint_id])] = position
    mujoco.mj_forward(model, data)


def finger_delta(model: Any, data: Any, side: str, position: float) -> np.ndarray:
    set_side_position(model, data, side, position)
    geom_ids = tuple(
        geom_for_joint(model, f"openarm_{side}_finger_joint{number}")[1]
        for number in (1, 2)
    )
    return np.asarray(data.geom_xpos[geom_ids[1]], dtype=float) - np.asarray(
        data.geom_xpos[geom_ids[0]], dtype=float
    )


def projected_sweep(model: Any, side: str) -> list[dict[str, float]]:
    data = mujoco.MjData(model)
    epsilon = 1e-5
    initial = finger_delta(model, data, side, 0.0)
    rate = (finger_delta(model, data, side, epsilon) - initial) / epsilon
    rate_norm = float(np.linalg.norm(rate))
    if rate_norm < 1e-9:
        raise ValueError(f"{side} finger separation does not respond to joint motion")
    axis = rate / rate_norm
    return [
        {
            "driver_joint_m": position,
            "projected_separation_m": abs(
                float(np.dot(finger_delta(model, data, side, position), axis))
            ),
        }
        for position in (0.0, 0.011, 0.022, 0.033, 0.044)
    ]


def patch_gripper_frames(
    root: ET.Element,
    formal_model: Any,
    reference_model: Any,
) -> dict[str, Any]:
    formal_data = mujoco.MjData(formal_model)
    reference_data = mujoco.MjData(reference_model)
    adjustments: dict[str, Any] = {}
    epsilon = 1e-5

    for side in ("left", "right"):
        formal_initial = finger_delta(formal_model, formal_data, side, 0.0)
        formal_rate = (
            finger_delta(formal_model, formal_data, side, epsilon) - formal_initial
        ) / epsilon
        formal_rate_norm = float(np.linalg.norm(formal_rate))
        formal_axis = formal_rate / formal_rate_norm
        formal_signed = float(np.dot(formal_initial, formal_axis))

        reference_initial = finger_delta(
            reference_model, reference_data, side, 0.0
        )
        reference_rate = (
            finger_delta(reference_model, reference_data, side, epsilon)
            - reference_initial
        ) / epsilon
        reference_axis = reference_rate / float(np.linalg.norm(reference_rate))
        target_opening = abs(float(np.dot(reference_initial, reference_axis)))

        target_signed = math.copysign(target_opening, formal_signed)
        equivalent_shift = (target_signed - formal_signed) / formal_rate_norm
        if not 0.0 < equivalent_shift < 0.044:
            raise ValueError(
                f"{side} required finger shift is implausible: {equivalent_shift}"
            )

        for number in (1, 2):
            joint_name = f"openarm_{side}_finger_joint{number}"
            body, joint = body_for_joint(root, joint_name)
            if any(body.get(name) for name in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
                raise ValueError(f"rotated finger body is unsupported: {joint_name}")
            old_axis = vector(joint.get("axis"))
            old_position = vector(body.get("pos"))
            body.set("pos", format_vector(old_position + old_axis * equivalent_shift))
            joint.set("axis", format_vector(-old_axis))

        adjustments[side] = {
            "formal_projected_opening_m": abs(formal_signed),
            "reference_projected_opening_m": target_opening,
            "body_shift_m": equivalent_shift,
        }
    return adjustments


def geom_vertices_world(model: Any, data: Any, geom_id: int) -> np.ndarray:
    mesh_id = int(model.geom_dataid[geom_id])
    if mesh_id < 0:
        raise ValueError(f"finger geom {geom_id} is not a mesh")
    address = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    vertices = np.asarray(model.mesh_vert[address : address + count], dtype=float)
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    position = np.asarray(data.geom_xpos[geom_id], dtype=float)
    return vertices @ rotation.T + position


def distal_vertices(vertices: np.ndarray, body_position: np.ndarray) -> np.ndarray:
    distances = np.linalg.norm(vertices - body_position, axis=1)
    threshold = float(np.quantile(distances, 0.75))
    selected = vertices[distances >= threshold]
    return selected if len(selected) else vertices


def closest_points(left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    best_distance = math.inf
    best_pair: tuple[np.ndarray, np.ndarray] | None = None
    for start in range(0, len(left), 256):
        block = left[start : start + 256]
        squared = np.sum((block[:, None, :] - right[None, :, :]) ** 2, axis=2)
        flat_index = int(np.argmin(squared))
        row, column = np.unravel_index(flat_index, squared.shape)
        distance = float(squared[row, column])
        if distance < best_distance:
            best_distance = distance
            best_pair = (block[row].copy(), right[column].copy())
    if best_pair is None:
        raise ValueError("finger mesh contains no vertices")
    return best_pair


def world_to_body(model: Any, data: Any, body_id: int, point: np.ndarray) -> np.ndarray:
    rotation = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
    position = np.asarray(data.xpos[body_id], dtype=float)
    return (point - position) @ rotation


def add_contact_sites(root: ET.Element, model: Any) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for side in ("left", "right"):
        entries = []
        for number in (1, 2):
            joint_name = f"openarm_{side}_finger_joint{number}"
            body_id, geom_id = geom_for_joint(model, joint_name)
            vertices = distal_vertices(
                geom_vertices_world(model, data, geom_id),
                np.asarray(data.xpos[body_id], dtype=float),
            )
            entries.append((joint_name, body_id, vertices))
        point1, point2 = closest_points(entries[0][2], entries[1][2])
        for number, ((joint_name, body_id, _), point) in enumerate(
            zip(entries, (point1, point2)), start=1
        ):
            body, _ = body_for_joint(root, joint_name)
            ET.SubElement(
                body,
                "site",
                {
                    "name": f"openarm_{side}_finger{number}_contact_site",
                    "pos": format_vector(world_to_body(model, data, body_id, point)),
                    "size": "0.0025",
                    "type": "sphere",
                    "rgba": "0.1 0.9 0.2 0.8",
                },
            )


def add_gripper_dynamics(root: ET.Element) -> None:
    for element_name in ("equality", "actuator"):
        for existing in list(root.findall(element_name)):
            root.remove(existing)
    equality = ET.SubElement(root, "equality")
    actuator = ET.SubElement(root, "actuator")
    for side in ("left", "right"):
        driver = f"openarm_{side}_finger_joint1"
        follower = f"openarm_{side}_finger_joint2"
        ET.SubElement(
            equality,
            "joint",
            {
                "name": f"openarm_{side}_finger_mimic",
                "joint1": follower,
                "joint2": driver,
                "polycoef": "0 1 0 0 0",
                "solref": "0.004 1",
            },
        )
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"openarm_{side}_gripper_position",
                "joint": driver,
                "kp": "200",
                "ctrllimited": "true",
                "ctrlrange": "0 0.044",
                "forcelimited": "true",
                "forcerange": "-9 9",
            },
        )


def set_all_grippers(model: Any, data: Any, position: float) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for name in FINGER_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = position
    mujoco.mj_forward(model, data)


def site_calibration(model: Any) -> dict[str, list[dict[str, float]]]:
    data = mujoco.MjData(model)
    result: dict[str, list[dict[str, float]]] = {}
    for side in ("left", "right"):
        site_ids = tuple(
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_SITE,
                f"openarm_{side}_finger{number}_contact_site",
            )
            for number in (1, 2)
        )
        set_all_grippers(model, data, 0.0)
        initial_delta = np.asarray(data.site_xpos[site_ids[1]]) - np.asarray(
            data.site_xpos[site_ids[0]]
        )
        axis = initial_delta / np.linalg.norm(initial_delta)
        samples = []
        for position in np.linspace(0.0, 0.044, 9):
            set_all_grippers(model, data, float(position))
            delta = np.asarray(data.site_xpos[site_ids[1]]) - np.asarray(
                data.site_xpos[site_ids[0]]
            )
            samples.append(
                {
                    "driver_joint_m": float(position),
                    "signed_contact_gap_m": float(np.dot(delta, axis)),
                    "contact_site_distance_m": float(np.linalg.norm(delta)),
                }
            )
        signed = [sample["signed_contact_gap_m"] for sample in samples]
        if any(right >= left - 1e-7 for left, right in zip(signed, signed[1:])):
            raise ValueError(f"{side} contact gap is not strictly decreasing: {signed}")
        result[side] = samples
    return result


def dynamic_gripper_validation(model: Any) -> dict[str, Any]:
    """Verify free-space actuator settling and mimic residual."""

    model.dof_damping[:] = 0.5
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    arm_addresses = {}
    for name, position in ARM_INITIAL_POSITIONS.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"dynamic validation arm joint missing: {name}")
        arm_addresses[name] = (
            int(model.jnt_qposadr[joint_id]),
            int(model.jnt_dofadr[joint_id]),
            position,
        )

    gripper_addresses = {}
    actuator_ids = {}
    for side in ("left", "right"):
        driver = f"openarm_{side}_finger_joint1"
        follower = f"openarm_{side}_finger_joint2"
        driver_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, driver)
        follower_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, follower)
        actuator_name = f"openarm_{side}_gripper_position"
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        if min(driver_id, follower_id, actuator_id) < 0:
            raise ValueError(f"dynamic validation gripper objects missing: {side}")
        gripper_addresses[side] = {
            "driver_qpos": int(model.jnt_qposadr[driver_id]),
            "driver_dof": int(model.jnt_dofadr[driver_id]),
            "follower_qpos": int(model.jnt_qposadr[follower_id]),
            "follower_dof": int(model.jnt_dofadr[follower_id]),
        }
        actuator_ids[side] = int(actuator_id)

    duration_s = 5.0
    steps = int(math.ceil(duration_s / float(model.opt.timestep)))
    samples = []
    maximum_mimic_error = 0.0
    maximum_driver_error = 0.0
    for target in DYNAMIC_TARGETS_M:
        data = mujoco.MjData(model)
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        for qpos_address, _dof_address, position in arm_addresses.values():
            data.qpos[qpos_address] = position
        for actuator_id in actuator_ids.values():
            data.ctrl[actuator_id] = target
        mujoco.mj_forward(model, data)

        for _ in range(steps):
            for qpos_address, dof_address, position in arm_addresses.values():
                data.qpos[qpos_address] = position
                data.qvel[dof_address] = 0.0
            mujoco.mj_step(model, data)

        for side, addresses in gripper_addresses.items():
            driver_position = float(data.qpos[addresses["driver_qpos"]])
            follower_position = float(data.qpos[addresses["follower_qpos"]])
            driver_error = abs(driver_position - target)
            mimic_error = abs(follower_position - driver_position)
            maximum_driver_error = max(maximum_driver_error, driver_error)
            maximum_mimic_error = max(maximum_mimic_error, mimic_error)
            samples.append(
                {
                    "side": side,
                    "target_m": target,
                    "driver_position_m": driver_position,
                    "follower_position_m": follower_position,
                    "driver_error_m": driver_error,
                    "mimic_error_m": mimic_error,
                    "driver_velocity_mps": float(
                        data.qvel[addresses["driver_dof"]]
                    ),
                    "follower_velocity_mps": float(
                        data.qvel[addresses["follower_dof"]]
                    ),
                }
            )

    if maximum_driver_error > MIMIC_TOLERANCE_M:
        raise ValueError(
            "dynamic driver error exceeds tolerance: "
            f"{maximum_driver_error:.9f} m"
        )
    if maximum_mimic_error > MIMIC_TOLERANCE_M:
        raise ValueError(
            "dynamic mimic error exceeds tolerance: "
            f"{maximum_mimic_error:.9f} m"
        )
    return {
        "duration_s": duration_s,
        "contacts_disabled": True,
        "tolerance_m": MIMIC_TOLERANCE_M,
        "maximum_driver_error_m": maximum_driver_error,
        "maximum_mimic_error_m": maximum_mimic_error,
        "samples": samples,
    }


def validate_model(model: Any, root: ET.Element) -> dict[str, Any]:
    errors = []
    for name in FINGER_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            errors.append(f"missing {name}")
            continue
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_SLIDE):
            errors.append(f"{name} is not slide")
        if not np.allclose(model.jnt_range[joint_id], (0.0, 0.044), atol=1e-9):
            errors.append(f"{name} range={model.jnt_range[joint_id].tolist()}")
    if errors:
        raise ValueError("; ".join(errors))
    if model.neq != 2 or model.nu != 2:
        raise ValueError(f"expected 2 equalities/actuators, got {model.neq}/{model.nu}")
    for equality_id in range(model.neq):
        if not np.allclose(
            model.eq_solref[equality_id], MIMIC_SOLREF, atol=1e-12
        ):
            errors.append(
                f"equality {equality_id} solref="
                f"{model.eq_solref[equality_id].tolist()}"
            )
    if errors:
        raise ValueError("; ".join(errors))
    names = {element.get("name") for element in root.find("worldbody").iter()}
    missing_scene = sorted(SCENE_NAMES - names)
    if missing_scene:
        raise ValueError(f"candidate scene lost nodes: {missing_scene}")
    return {
        "bodies": int(model.nbody),
        "joints": int(model.njnt),
        "geoms": int(model.ngeom),
        "equalities": int(model.neq),
        "actuators": int(model.nu),
        "sites": int(model.nsite),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--description-root", type=Path, required=True)
    parser.add_argument("--scene-source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if sha256(args.scene_source) != EXPECTED_FORMAL_MJCF_SHA256:
        raise ValueError("formal model hash changed; refusing in-place-derived build")

    args.output_dir.mkdir(parents=True, exist_ok=False)
    output_urdf = args.output_dir / "robot.urdf"
    output_mjcf = args.output_dir / "robot.mjcf.xml"
    calibration = args.output_dir / "gripper_calibration.json"
    report_path = args.output_dir / "validation_report.json"
    shutil.copyfile(args.urdf, output_urdf)

    formal_model = mujoco.MjModel.from_xml_path(str(args.scene_source))
    reference_model = compile_reference_urdf(args.urdf, args.description_root)
    candidate_tree = ET.parse(args.scene_source)
    candidate_root = candidate_tree.getroot()
    candidate_root.set("model", "openarm_parallel_gripper_in_place_candidate")
    adjustments = patch_gripper_frames(
        candidate_root, formal_model, reference_model
    )

    with tempfile.TemporaryDirectory(prefix="openarm_in_place_gripper_") as temp_dir:
        frame_candidate = Path(temp_dir) / "frame_candidate.xml"
        candidate_tree.write(frame_candidate, encoding="utf-8", xml_declaration=True)
        frame_model = mujoco.MjModel.from_xml_path(str(frame_candidate))
        add_contact_sites(candidate_root, frame_model)

    add_gripper_dynamics(candidate_root)
    ET.indent(candidate_tree, space="  ")
    candidate_tree.write(output_mjcf, encoding="utf-8", xml_declaration=True)

    final_model = mujoco.MjModel.from_xml_path(str(output_mjcf))
    model_summary = validate_model(final_model, candidate_root)
    reference_sweep = {
        side: projected_sweep(reference_model, side) for side in ("left", "right")
    }
    candidate_sweep = {
        side: projected_sweep(final_model, side) for side in ("left", "right")
    }
    maximum_error = max(
        abs(candidate["projected_separation_m"] - reference["projected_separation_m"])
        for side in ("left", "right")
        for candidate, reference in zip(
            candidate_sweep[side], reference_sweep[side]
        )
    )
    if maximum_error > 1e-5:
        raise ValueError(f"projected gripper sweep mismatch: {maximum_error}")

    contact_samples = site_calibration(final_model)
    dynamic_validation = dynamic_gripper_validation(final_model)
    calibration.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "semantics": "driver joint increases from open toward closed",
                "driver_joint_range_m": [0.0, 0.044],
                "samples": contact_samples,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    report = {
        "status": "PASS",
        "strategy": "formal_model_in_place_gripper_patch",
        "model": model_summary,
        "adjustments": adjustments,
        "maximum_projected_sweep_error_m": maximum_error,
        "mimic_solref": list(MIMIC_SOLREF),
        "dynamic_gripper_validation": dynamic_validation,
        "formal_model_sha256": sha256(args.scene_source),
        "candidate_urdf_sha256": sha256(output_urdf),
        "candidate_mjcf_sha256": sha256(output_mjcf),
        "calibration_sha256": sha256(calibration),
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (args.output_dir / "model.sha256").write_text(
        f"{sha256(output_urdf)}  robot.urdf\n{sha256(output_mjcf)}  robot.mjcf.xml\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    print(f"candidate: {args.output_dir}")
    print("OPENARM IN-PLACE PARALLEL MJCF CANDIDATE BUILD: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
