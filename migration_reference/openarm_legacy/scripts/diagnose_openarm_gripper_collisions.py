#!/usr/bin/env python3
"""Compare candidate URDF and MJCF gripper collisions at identical states.

This is a read-only SIMULATION diagnostic. It subscribes to /joint_states but
does not publish commands, call controllers, or modify either candidate model.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import tempfile
import time
import xml.etree.ElementTree as ET
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ARM_JOINTS = tuple(
    f"openarm_{side}_joint{index}"
    for side in ("left", "right")
    for index in range(1, 8)
)
EFFECTIVE_ARM_JOINTS = tuple(
    f"openarm_{side}_joint{index}"
    for side in ("left", "right")
    for index in (3, 5, 6, 7)
)
FINGER_JOINTS = tuple(
    f"openarm_{side}_finger_joint{index}"
    for side in ("left", "right")
    for index in (1, 2)
)
EXPECTED_JOINTS = ARM_JOINTS + FINGER_JOINTS
FOCUS_LINKS = tuple(
    f"openarm_{side}_{suffix}"
    for side in ("left", "right")
    for suffix in (
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
        "ee_base_link",
        "ee_link1",
        "ee_link2",
    )
)
SWEEP_POSITIONS_M = (0.0, 0.011, 0.022, 0.033, 0.044)
INITIAL_POSITIONS = dict(
    zip(
        ARM_JOINTS,
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
        strict=True,
    )
)
INITIAL_POSITIONS.update({name: 0.0 for name in FINGER_JOINTS})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--mjcf", required=True, type=Path)
    parser.add_argument("--description-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--joint-state-json",
        type=Path,
        help="Optional name-to-position JSON map; otherwise read /joint_states.",
    )
    parser.add_argument("--joint-state-timeout", type=float, default=15.0)
    parser.add_argument("--use-candidate-initial-state", action="store_true")
    parser.add_argument("--require-candidate-alignment", action="store_true")
    return parser.parse_args()


def resolve_urdf(source: Path, description_root: Path, output: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()

    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)

    package_prefix = "package://openarm_description/"
    mesh_dir = output.parent / "unique_collision_meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for index, mesh in enumerate(root.iter("mesh")):
        filename = mesh.get("filename", "")
        if filename.startswith(package_prefix):
            resolved = description_root / filename[len(package_prefix) :]
            if not resolved.is_file():
                raise FileNotFoundError(resolved)
            # Keep signed-scale mesh instances independent in MuJoCo's URDF
            # importer; these copies exist only inside the temporary directory.
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


def load_models(
    urdf: Path, mjcf_path: Path, description_root: Path
) -> tuple[Any, Any]:
    with tempfile.TemporaryDirectory(prefix="openarm_collision_diagnostic_") as tmp:
        resolved_urdf = Path(tmp) / "robot.collision.urdf"
        resolve_urdf(urdf, description_root, resolved_urdf)
        urdf_model = mujoco.MjModel.from_xml_path(str(resolved_urdf))
    mjcf_model = mujoco.MjModel.from_xml_path(str(mjcf_path.resolve()))
    return urdf_model, mjcf_model


def read_joint_states(timeout_s: float) -> dict[str, float]:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise RuntimeError(
            "rclpy/sensor_msgs are required unless --joint-state-json is used"
        ) from exc

    class JointStateReader(Node):
        def __init__(self) -> None:
            super().__init__("openarm_gripper_collision_diagnostic")
            self.positions: dict[str, float] = {}
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(JointState, "/joint_states", self.on_state, qos)

        def on_state(self, message: Any) -> None:
            for name, position in zip(message.name, message.position, strict=False):
                if name in EXPECTED_JOINTS and math.isfinite(float(position)):
                    self.positions[name] = float(position)

    rclpy.init(args=None)
    node = JointStateReader()
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(name in node.positions for name in EXPECTED_JOINTS):
                return dict(node.positions)
        missing = sorted(set(EXPECTED_JOINTS) - set(node.positions))
        raise TimeoutError(f"timed out waiting for /joint_states; missing {missing}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def load_joint_positions(args: argparse.Namespace) -> dict[str, float]:
    if args.use_candidate_initial_state:
        if args.joint_state_json is not None:
            raise ValueError(
                "--use-candidate-initial-state and --joint-state-json are exclusive"
            )
        return dict(INITIAL_POSITIONS)
    if args.joint_state_json is None:
        return read_joint_states(args.joint_state_timeout)
    raw = json.loads(args.joint_state_json.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("--joint-state-json must contain a JSON object")
    result = {str(name): float(value) for name, value in raw.items()}
    missing = sorted(set(EXPECTED_JOINTS) - set(result))
    if missing:
        raise ValueError(f"joint-state JSON is missing {missing}")
    return result


def joint_address(model: Any, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"model is missing joint {name}")
    if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_SLIDE) and name in FINGER_JOINTS:
        raise ValueError(f"{name} is not a slide joint")
    return int(model.jnt_qposadr[joint_id])


def set_state(model: Any, data: Any, positions: dict[str, float]) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for name in EXPECTED_JOINTS:
        data.qpos[joint_address(model, name)] = positions[name]
    mujoco.mj_forward(model, data)


def body_labels(model: Any) -> dict[int, str]:
    labels: dict[int, str] = {}
    for body_id in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name:
            labels[body_id] = name
    for name in FINGER_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        labels[int(model.jnt_bodyid[joint_id])] = name.replace("finger_joint", "ee_link")
    return labels


def geom_vertices_world(model: Any, data: Any, geom_id: int) -> np.ndarray:
    geom_type = int(model.geom_type[geom_id])
    position = np.asarray(data.geom_xpos[geom_id], dtype=float)
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        address = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = np.asarray(model.mesh_vert[address : address + count], dtype=float)
        return vertices @ rotation.T + position

    center = np.asarray(model.geom_aabb[geom_id, :3], dtype=float)
    half = np.asarray(model.geom_aabb[geom_id, 3:], dtype=float)
    corners = np.asarray(
        [
            center + np.asarray((x, y, z)) * half
            for x in (-1.0, 1.0)
            for y in (-1.0, 1.0)
            for z in (-1.0, 1.0)
        ]
    )
    return corners @ rotation.T + position


def link_aabbs(model: Any, data: Any, labels: dict[int, str]) -> dict[str, Any]:
    grouped: dict[str, list[np.ndarray]] = {}
    for geom_id in range(model.ngeom):
        label = labels.get(int(model.geom_bodyid[geom_id]), "")
        if label in FOCUS_LINKS:
            grouped.setdefault(label, []).append(geom_vertices_world(model, data, geom_id))
    result: dict[str, Any] = {}
    for label, groups in grouped.items():
        vertices = np.concatenate(groups)
        lower = np.min(vertices, axis=0)
        upper = np.max(vertices, axis=0)
        result[label] = {
            "min": lower.tolist(),
            "max": upper.tolist(),
            "center": ((lower + upper) / 2.0).tolist(),
            "extent": (upper - lower).tolist(),
        }
    return result


def focus_body_poses(model: Any, data: Any, labels: dict[int, str]) -> dict[str, Any]:
    result = {}
    for body_id, label in labels.items():
        if label not in FOCUS_LINKS:
            continue
        result[label] = {
            "position_m": np.asarray(data.xpos[body_id], dtype=float).tolist(),
            "rotation_matrix": np.asarray(data.xmat[body_id], dtype=float)
            .reshape(3, 3)
            .tolist(),
        }
    return result


def focus_geom_poses(model: Any, data: Any, labels: dict[int, str]) -> list[dict[str, Any]]:
    result = []
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        label = labels.get(body_id, "")
        if label not in FOCUS_LINKS:
            continue
        vertices = geom_vertices_world(model, data, geom_id)
        lower = np.min(vertices, axis=0)
        upper = np.max(vertices, axis=0)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        result.append(
            {
                "link": label,
                "geom": geom_name or f"geom_{geom_id}",
                "local_position_m": np.asarray(
                    model.geom_pos[geom_id], dtype=float
                ).tolist(),
                "world_position_m": np.asarray(
                    data.geom_xpos[geom_id], dtype=float
                ).tolist(),
                "world_aabb_min_m": lower.tolist(),
                "world_aabb_max_m": upper.tolist(),
            }
        )
    return result


def joint_frames(model: Any, data: Any) -> dict[str, Any]:
    result = {}
    for name in EXPECTED_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"model is missing joint {name}")
        result[name] = {
            "anchor_m": np.asarray(data.xanchor[joint_id], dtype=float).tolist(),
            "axis": np.asarray(data.xaxis[joint_id], dtype=float).tolist(),
        }
    return result


def canonical_pair(first: str, second: str) -> tuple[str, str]:
    return tuple(sorted((first, second)))  # type: ignore[return-value]


def contacts(model: Any, data: Any, labels: dict[int, str]) -> list[dict[str, Any]]:
    by_pair: dict[tuple[str, str], dict[str, Any]] = {}
    for index in range(data.ncon):
        contact = data.contact[index]
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        label1 = labels.get(body1, "")
        label2 = labels.get(body2, "")
        if not label1.startswith("openarm_") or not label2.startswith("openarm_"):
            continue
        pair = canonical_pair(label1, label2)
        entry = by_pair.setdefault(
            pair,
            {
                "pair": list(pair),
                "contact_count": 0,
                "minimum_distance_m": math.inf,
                "maximum_penetration_m": 0.0,
            },
        )
        entry["contact_count"] += 1
        distance = float(contact.dist)
        entry["minimum_distance_m"] = min(entry["minimum_distance_m"], distance)
        entry["maximum_penetration_m"] = max(
            entry["maximum_penetration_m"], max(0.0, -distance)
        )
    return [by_pair[pair] for pair in sorted(by_pair)]


def pair_set(entries: Iterable[dict[str, Any]]) -> set[tuple[str, str]]:
    return {tuple(entry["pair"]) for entry in entries}


def focus_pairs(side: str) -> set[tuple[str, str]]:
    finger = f"openarm_{side}_ee_link2"
    return {
        canonical_pair(finger, f"openarm_{side}_link{number}")
        for number in (3, 4, 5)
    }


def state_report(model: Any, positions: dict[str, float]) -> dict[str, Any]:
    data = mujoco.MjData(model)
    set_state(model, data, positions)
    labels = body_labels(model)
    return {
        "contacts": contacts(model, data, labels),
        "aabbs": link_aabbs(model, data, labels),
        "body_poses": focus_body_poses(model, data, labels),
        "geom_poses": focus_geom_poses(model, data, labels),
        "joint_frames": joint_frames(model, data),
    }


def sweep_report(model: Any, base: dict[str, float], side: str) -> list[dict[str, Any]]:
    result = []
    relevant = focus_pairs(side)
    for position in SWEEP_POSITIONS_M:
        state = dict(base)
        state[f"openarm_{side}_finger_joint1"] = position
        state[f"openarm_{side}_finger_joint2"] = position
        report = state_report(model, state)
        entries = [
            entry for entry in report["contacts"] if tuple(entry["pair"]) in relevant
        ]
        result.append({"finger_position_m": position, "contacts": entries})
    return result


def normalized_pairs(entries: Iterable[dict[str, Any]], side: str) -> set[tuple[str, str]]:
    prefix = f"openarm_{side}_"
    return {
        tuple(part.replace(prefix, "openarm_SIDE_") for part in entry["pair"])
        for entry in entries
        if all(part.startswith(prefix) for part in entry["pair"])
    }


def home_symmetry(model: Any) -> dict[str, Any]:
    state = {name: 0.0 for name in EXPECTED_JOINTS}
    report = state_report(model, state)
    left = normalized_pairs(report["contacts"], "left")
    right = normalized_pairs(report["contacts"], "right")
    return {
        "left_pairs": [list(pair) for pair in sorted(left)],
        "right_pairs": [list(pair) for pair in sorted(right)],
        "symmetric": left == right,
        "left_only": [list(pair) for pair in sorted(left - right)],
        "right_only": [list(pair) for pair in sorted(right - left)],
    }


def aabb_agreement(
    urdf: dict[str, Any],
    mjcf: dict[str, Any],
    selected: Iterable[str] | None = None,
) -> dict[str, Any]:
    all_common_names = set(urdf) & set(mjcf)
    common_names = set(all_common_names)
    if selected is not None:
        common_names &= set(selected)
    common = sorted(common_names)
    if not common:
        raise ValueError("URDF and MJCF have no common focus-link AABBs")

    anchor = "openarm_right_link3"
    if anchor not in all_common_names:
        raise ValueError(f"URDF and MJCF are missing AABB anchor {anchor}")
    urdf_anchor = np.asarray(urdf[anchor]["center"], dtype=float)
    mjcf_anchor = np.asarray(mjcf[anchor]["center"], dtype=float)
    entries = {}
    maximum_center_error = 0.0
    maximum_extent_error = 0.0
    for name in common:
        urdf_center = np.asarray(urdf[name]["center"], dtype=float) - urdf_anchor
        mjcf_center = np.asarray(mjcf[name]["center"], dtype=float) - mjcf_anchor
        center_error = float(np.linalg.norm(urdf_center - mjcf_center))
        extent_error = float(
            np.linalg.norm(
                np.asarray(urdf[name]["extent"], dtype=float)
                - np.asarray(mjcf[name]["extent"], dtype=float)
            )
        )
        maximum_center_error = max(maximum_center_error, center_error)
        maximum_extent_error = max(maximum_extent_error, extent_error)
        entries[name] = {
            "root_translation_normalized_center_error_m": center_error,
            "extent_error_m": extent_error,
        }
    return {
        "anchor": anchor,
        "root_translation_m": (mjcf_anchor - urdf_anchor).tolist(),
        "maximum_center_error_m": maximum_center_error,
        "maximum_extent_error_m": maximum_extent_error,
        "links": entries,
    }


def joint_frame_agreement(
    urdf: dict[str, Any],
    mjcf: dict[str, Any],
    selected: Iterable[str] = EXPECTED_JOINTS,
) -> dict[str, Any]:
    anchor = "openarm_right_joint3"
    urdf_root = np.asarray(urdf[anchor]["anchor_m"], dtype=float)
    mjcf_root = np.asarray(mjcf[anchor]["anchor_m"], dtype=float)
    result = {}
    maximum_anchor_error = 0.0
    maximum_axis_error = 0.0
    for name in selected:
        urdf_anchor = np.asarray(urdf[name]["anchor_m"], dtype=float) - urdf_root
        mjcf_anchor = np.asarray(mjcf[name]["anchor_m"], dtype=float) - mjcf_root
        urdf_axis = np.asarray(urdf[name]["axis"], dtype=float)
        mjcf_axis = np.asarray(mjcf[name]["axis"], dtype=float)
        anchor_error = float(np.linalg.norm(urdf_anchor - mjcf_anchor))
        axis_error = float(np.linalg.norm(urdf_axis - mjcf_axis))
        maximum_anchor_error = max(maximum_anchor_error, anchor_error)
        maximum_axis_error = max(maximum_axis_error, axis_error)
        result[name] = {
            "root_translation_normalized_anchor_error_m": anchor_error,
            "axis_vector_error": axis_error,
            "urdf_anchor_m": urdf[name]["anchor_m"],
            "mjcf_anchor_m": mjcf[name]["anchor_m"],
            "urdf_axis": urdf[name]["axis"],
            "mjcf_axis": mjcf[name]["axis"],
        }
    return {
        "anchor": anchor,
        "root_translation_m": (mjcf_root - urdf_root).tolist(),
        "maximum_anchor_error_m": maximum_anchor_error,
        "maximum_axis_vector_error": maximum_axis_error,
        "joints": result,
    }


def main() -> int:
    args = parse_args()
    for path in (args.urdf, args.mjcf, args.description_root):
        if not path.exists():
            raise FileNotFoundError(path)

    positions = load_joint_positions(args)
    urdf_model, mjcf_model = load_models(args.urdf, args.mjcf, args.description_root)
    urdf_current = state_report(urdf_model, positions)
    mjcf_current = state_report(mjcf_model, positions)

    urdf_pairs = pair_set(urdf_current["contacts"])
    mjcf_pairs = pair_set(mjcf_current["contacts"])
    focus = focus_pairs("left") | focus_pairs("right")
    urdf_focus = urdf_pairs & focus
    mjcf_focus = mjcf_pairs & focus
    agreement = aabb_agreement(urdf_current["aabbs"], mjcf_current["aabbs"])
    frame_agreement = joint_frame_agreement(
        urdf_current["joint_frames"], mjcf_current["joint_frames"]
    )
    arm_frame_agreement = joint_frame_agreement(
        urdf_current["joint_frames"],
        mjcf_current["joint_frames"],
        EFFECTIVE_ARM_JOINTS,
    )
    finger_frame_agreement = joint_frame_agreement(
        urdf_current["joint_frames"],
        mjcf_current["joint_frames"],
        FINGER_JOINTS,
    )
    finger_aabb_agreement = aabb_agreement(
        urdf_current["aabbs"],
        mjcf_current["aabbs"],
        (
            f"openarm_{side}_ee_link{number}"
            for side in ("left", "right")
            for number in (1, 2)
        ),
    )
    arm_frames_agree = (
        arm_frame_agreement["maximum_anchor_error_m"] <= 0.001
        and arm_frame_agreement["maximum_axis_vector_error"] <= 0.001
    )
    finger_frames_agree = (
        finger_frame_agreement["maximum_anchor_error_m"] <= 0.001
        and finger_frame_agreement["maximum_axis_vector_error"] <= 0.001
    )
    finger_collision_geometry_agrees = (
        finger_aabb_agreement["maximum_center_error_m"] <= 0.001
        and finger_aabb_agreement["maximum_extent_error_m"] <= 0.001
    )
    finger_sweep = {
        source: {
            side: sweep_report(model, positions, side)
            for side in ("left", "right")
        }
        for source, model in (("urdf", urdf_model), ("mjcf", mjcf_model))
    }
    finger_arm_sweep_clear = all(
        not sample["contacts"]
        for sources in finger_sweep.values()
        for samples in sources.values()
        for sample in samples
    )
    candidate_alignment_passed = (
        arm_frames_agree
        and finger_frames_agree
        and finger_collision_geometry_agrees
        and urdf_focus == mjcf_focus
        and not urdf_focus
        and not mjcf_focus
        and finger_arm_sweep_clear
    )

    result = {
        "status": "DIAGNOSTIC_COMPLETE",
        "mode": "SIMULATION_READ_ONLY",
        "inputs": {
            "urdf": str(args.urdf.resolve()),
            "mjcf": str(args.mjcf.resolve()),
            "description_root": str(args.description_root.resolve()),
            "joint_positions": positions,
        },
        "current_state": {
            "urdf": urdf_current,
            "mjcf": mjcf_current,
            "finger_arm_focus_pairs": {
                "urdf": [list(pair) for pair in sorted(urdf_focus)],
                "mjcf": [list(pair) for pair in sorted(mjcf_focus)],
                "agree": urdf_focus == mjcf_focus,
            },
            "all_robot_pair_difference": {
                "urdf_only": [list(pair) for pair in sorted(urdf_pairs - mjcf_pairs)],
                "mjcf_only": [list(pair) for pair in sorted(mjcf_pairs - urdf_pairs)],
            },
            "aabb_agreement": agreement,
            "joint_frame_agreement": frame_agreement,
            "arm_joint_frame_agreement": arm_frame_agreement,
            "finger_joint_frame_agreement": finger_frame_agreement,
            "finger_aabb_agreement": finger_aabb_agreement,
        },
        "finger_sweep": finger_sweep,
        "zero_arm_pose_symmetry": {
            "urdf": home_symmetry(urdf_model),
            "mjcf": home_symmetry(mjcf_model),
        },
        "decision_hints": {
            "model_frames_agree": (
                agreement["maximum_center_error_m"] <= 0.001
                and agreement["maximum_extent_error_m"] <= 0.001
                and frame_agreement["maximum_anchor_error_m"] <= 0.001
                and frame_agreement["maximum_axis_vector_error"] <= 0.001
            ),
            "finger_arm_focus_contacts_agree": urdf_focus == mjcf_focus,
            "finger_arm_focus_contacts_present_in_urdf": bool(urdf_focus),
            "finger_arm_focus_contacts_present_in_mjcf": bool(mjcf_focus),
            "arm_joint_frames_agree": arm_frames_agree,
            "finger_joint_frames_agree": finger_frames_agree,
            "finger_collision_geometry_agrees": finger_collision_geometry_agrees,
            "finger_arm_sweep_clear": finger_arm_sweep_clear,
            "candidate_alignment_passed": candidate_alignment_passed,
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result["decision_hints"], indent=2, sort_keys=True))
    print(f"report: {args.output.resolve()}")
    if args.require_candidate_alignment and not candidate_alignment_passed:
        print("OPENARM CANDIDATE KINEMATIC/COLLISION ALIGNMENT: FAIL")
        return 1
    if args.require_candidate_alignment:
        print("OPENARM CANDIDATE KINEMATIC/COLLISION ALIGNMENT: PASS")
    else:
        print("OPENARM GRIPPER COLLISION DIAGNOSTIC: COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
