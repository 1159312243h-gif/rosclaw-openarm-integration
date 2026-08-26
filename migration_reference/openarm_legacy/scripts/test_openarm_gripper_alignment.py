#!/usr/bin/env python3
"""Cross-check the OpenArm parallel gripper across URDF, MuJoCo, and ROS control."""

from __future__ import annotations

import argparse
import ast
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


FINGER_JOINTS = tuple(
    f"openarm_{side}_finger_joint{number}"
    for side in ("left", "right")
    for number in (1, 2)
)
EXPECTED_AXIS = {
    name: (0.0, -1.0 if name.endswith("joint1") else 1.0, 0.0)
    for name in FINGER_JOINTS
}
EXPECTED_RANGE = (0.0, 0.044)
TOLERANCE = 1e-7


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str
    blocking: bool = True


def vector(value: str | None) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    result = tuple(float(item) for item in value.split())
    if len(result) != 3:
        raise ValueError(f"expected three-vector, got {value!r}")
    return result


def close(left: float, right: float, tolerance: float = TOLERANCE) -> bool:
    return math.isclose(left, right, rel_tol=0.0, abs_tol=tolerance)


def vectors_close(left: tuple[float, ...], right: tuple[float, ...]) -> bool:
    return len(left) == len(right) and all(close(a, b) for a, b in zip(left, right))


def add_check(
    checks: list[Check],
    name: str,
    passed: bool,
    success: str,
    failure: str,
    *,
    blocking: bool = True,
) -> None:
    checks.append(
        Check(
            name=name,
            status="PASS" if passed else ("FAIL" if blocking else "WARN"),
            detail=success if passed else failure,
            blocking=blocking,
        )
    )


def urdf_joint_map(root: ET.Element) -> dict[str, ET.Element]:
    return {joint.get("name", ""): joint for joint in root.findall("joint")}


def mjcf_joint_map(root: ET.Element) -> dict[str, tuple[ET.Element, ET.Element]]:
    result: dict[str, tuple[ET.Element, ET.Element]] = {}
    for body in root.iter("body"):
        for joint in body.findall("joint"):
            name = joint.get("name")
            if name:
                result[name] = (joint, body)
    return result


def check_urdf(root: ET.Element, checks: list[Check]) -> dict[str, ET.Element]:
    joints = urdf_joint_map(root)
    missing = sorted(set(FINGER_JOINTS) - joints.keys())
    add_check(
        checks,
        "urdf.finger_joints_present",
        not missing,
        "all four finger joints are present",
        f"missing joints: {missing}",
    )
    if missing:
        return joints

    wrong_types = {
        name: joints[name].get("type")
        for name in FINGER_JOINTS
        if joints[name].get("type") != "prismatic"
    }
    add_check(
        checks,
        "urdf.prismatic_topology",
        not wrong_types,
        "all four finger joints are prismatic",
        f"unexpected joint types: {wrong_types}",
    )

    axis_errors: dict[str, tuple[float, float, float]] = {}
    range_errors: dict[str, tuple[float, float]] = {}
    for name in FINGER_JOINTS:
        joint = joints[name]
        actual_axis = vector(joint.find("axis").get("xyz"))
        if not vectors_close(actual_axis, EXPECTED_AXIS[name]):
            axis_errors[name] = actual_axis
        limit = joint.find("limit")
        actual_range = (float(limit.get("lower")), float(limit.get("upper")))
        if not vectors_close(actual_range, EXPECTED_RANGE):
            range_errors[name] = actual_range

    add_check(
        checks,
        "urdf.axes",
        not axis_errors,
        "joint1 uses -Y and joint2 uses +Y on both grippers",
        f"axis mismatches: {axis_errors}",
    )
    add_check(
        checks,
        "urdf.ranges",
        not range_errors,
        "all finger ranges are 0.000-0.044 m",
        f"range mismatches: {range_errors}",
    )

    mimic_errors: list[str] = []
    for side in ("left", "right"):
        driver = f"openarm_{side}_finger_joint1"
        follower = f"openarm_{side}_finger_joint2"
        mimic = joints[follower].find("mimic")
        if (
            mimic is None
            or mimic.get("joint") != driver
            or not close(float(mimic.get("multiplier", "1")), 1.0)
            or not close(float(mimic.get("offset", "0")), 0.0)
        ):
            mimic_errors.append(follower)
    add_check(
        checks,
        "urdf.mimic",
        not mimic_errors,
        "joint2 mimics joint1 with multiplier +1 on both grippers",
        f"invalid mimic declarations: {mimic_errors}",
    )
    return joints


def check_mjcf(
    root: ET.Element,
    urdf_joints: dict[str, ET.Element],
    checks: list[Check],
) -> dict[str, tuple[ET.Element, ET.Element]]:
    joints = mjcf_joint_map(root)
    missing = sorted(set(FINGER_JOINTS) - joints.keys())
    add_check(
        checks,
        "mjcf.finger_joints_present",
        not missing,
        "all four finger joints are present",
        f"missing joints: {missing}",
    )
    if missing:
        return joints

    topology_errors: dict[str, str | None] = {}
    axis_errors: dict[str, tuple[float, float, float]] = {}
    range_errors: dict[str, tuple[float, float]] = {}
    origin_errors: dict[str, dict[str, tuple[float, float, float]]] = {}
    for name in FINGER_JOINTS:
        joint, body = joints[name]
        if joint.get("type", "hinge") != "slide":
            topology_errors[name] = joint.get("type")
        actual_axis = vector(joint.get("axis"))
        if not vectors_close(actual_axis, EXPECTED_AXIS[name]):
            axis_errors[name] = actual_axis
        actual_range = tuple(float(item) for item in joint.get("range", "").split())
        if not vectors_close(actual_range, EXPECTED_RANGE):
            range_errors[name] = actual_range

        if name in urdf_joints:
            urdf_origin = vector(urdf_joints[name].find("origin").get("xyz"))
            mjcf_body_origin = vector(body.get("pos"))
            mjcf_joint_offset = vector(joint.get("pos"))
            mjcf_origin = tuple(a + b for a, b in zip(mjcf_body_origin, mjcf_joint_offset))
            if not vectors_close(urdf_origin, mjcf_origin):
                origin_errors[name] = {
                    "urdf": urdf_origin,
                    "mjcf": mjcf_origin,
                }

    add_check(
        checks,
        "mjcf.slide_topology",
        not topology_errors,
        "all four MuJoCo finger joints are slide joints",
        f"unexpected joint types: {topology_errors}",
    )
    add_check(
        checks,
        "mjcf.axes",
        not axis_errors,
        "MuJoCo axes match the parallel-gripper URDF",
        f"axis mismatches: {axis_errors}",
    )
    add_check(
        checks,
        "mjcf.ranges",
        not range_errors,
        "MuJoCo ranges match 0.000-0.044 m",
        f"range mismatches: {range_errors}",
    )
    add_check(
        checks,
        "cross_model.joint_origins",
        not origin_errors,
        "URDF and MuJoCo finger joint origins match",
        f"joint frame representations differ: {origin_errors}",
        blocking=False,
    )

    equality_pairs = set()
    equality = root.find("equality")
    if equality is not None:
        for element in equality.findall("joint"):
            first = element.get("joint1") or element.get("joint")
            second = element.get("joint2")
            if first and second:
                equality_pairs.add(frozenset((first, second)))
    missing_equalities = []
    for side in ("left", "right"):
        pair = frozenset(
            (
                f"openarm_{side}_finger_joint1",
                f"openarm_{side}_finger_joint2",
            )
        )
        if pair not in equality_pairs:
            missing_equalities.append(side)
    add_check(
        checks,
        "mjcf.mimic_equalities",
        not missing_equalities,
        "MuJoCo constrains each follower joint to its driver",
        f"missing joint equality for: {missing_equalities}",
    )

    actuator_joints = {
        element.get("joint")
        for actuator in root.findall("actuator")
        for element in actuator
        if element.get("joint")
    }
    missing_actuators = [
        f"openarm_{side}_finger_joint1"
        for side in ("left", "right")
        if f"openarm_{side}_finger_joint1" not in actuator_joints
    ]
    add_check(
        checks,
        "mjcf.driver_actuators",
        not missing_actuators,
        "both gripper driver joints have MuJoCo actuators",
        f"no native actuator for: {missing_actuators}",
        blocking=False,
    )
    return joints


def controller_keys_from_bridge(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if not any(isinstance(target, ast.Name) and target.id == "CONTROLLER_JOINTS" for target in targets):
            continue
        value = node.value
        if isinstance(value, ast.Dict):
            return {
                key.value
                for key in value.keys
                if isinstance(key, ast.Constant) and isinstance(key.value, str)
            }
    return set()


def check_control_path(
    bridge: Path,
    moveit_controllers: Path,
    srdf: Path,
    checks: list[Check],
) -> None:
    bridge_controllers = controller_keys_from_bridge(bridge)
    expected = {"left_gripper_controller", "right_gripper_controller"}
    missing_bridge = sorted(expected - bridge_controllers)
    add_check(
        checks,
        "bridge.gripper_action_servers",
        not missing_bridge,
        "bridge exposes both gripper trajectory controllers",
        f"missing bridge controllers: {missing_bridge}",
    )

    moveit_text = moveit_controllers.read_text(encoding="utf-8")
    missing_moveit = sorted(name for name in expected if name not in moveit_text)
    add_check(
        checks,
        "moveit.gripper_controllers",
        not missing_moveit,
        "MoveIt config registers both gripper controllers",
        f"missing MoveIt controllers: {missing_moveit}",
    )

    srdf_root = ET.parse(srdf).getroot()
    groups = {group.get("name") for group in srdf_root.findall("group")}
    expected_groups = {"left_gripper", "right_gripper"}
    missing_groups = sorted(expected_groups - groups)
    add_check(
        checks,
        "moveit.srdf_gripper_groups",
        not missing_groups,
        "SRDF defines both gripper planning groups",
        f"missing SRDF groups: {missing_groups}",
    )


def compile_urdf_collision_model(
    urdf_path: Path,
    description_root: Path,
    mujoco: Any,
) -> Any:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            link.remove(visual)

    prefix = "package://openarm_description/"
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(prefix):
            resolved = description_root / filename[len(prefix) :]
            if not resolved.is_file():
                raise FileNotFoundError(resolved)
            mesh.set("filename", str(resolved.resolve()))

    for child in list(root):
        if child.tag in {"ros2_control", "transmission", "gazebo", "mujoco"}:
            root.remove(child)
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {"balanceinertia": "true", "discardvisual": "true", "fusestatic": "false", "strippath": "false"},
    )

    with tempfile.TemporaryDirectory(prefix="openarm_gripper_alignment_") as temp_dir:
        resolved_urdf = Path(temp_dir) / "candidate.collision.urdf"
        tree.write(resolved_urdf, encoding="utf-8", xml_declaration=True)
        return mujoco.MjModel.from_xml_path(str(resolved_urdf))


def finger_geom_id(model: Any, mujoco: Any, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"joint absent from MuJoCo model: {joint_name}")
    body_id = int(model.jnt_bodyid[joint_id])
    geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) == body_id
    ]
    if len(geom_ids) != 1:
        raise ValueError(
            f"expected one collision geom on {joint_name} body, got {geom_ids}"
        )
    return geom_ids[0]


def set_gripper_position(
    model: Any,
    data: Any,
    mujoco: Any,
    side: str,
    position: float,
) -> None:
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for number in (1, 2):
        name = f"openarm_{side}_finger_joint{number}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = position
    mujoco.mj_forward(model, data)


def check_mujoco_sweep(
    mjcf_path: Path,
    urdf_path: Path,
    description_root: Path,
    checks: list[Check],
) -> dict[str, Any]:
    import mujoco

    formal_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    candidate_model = compile_urdf_collision_model(
        urdf_path, description_root, mujoco
    )
    formal_data = mujoco.MjData(formal_model)
    candidate_data = mujoco.MjData(candidate_model)
    samples: dict[str, Any] = {}
    max_gap_error = 0.0
    for side in ("left", "right"):
        joint_names = tuple(
            f"openarm_{side}_finger_joint{number}" for number in (1, 2)
        )
        formal_geom_ids = tuple(
            finger_geom_id(formal_model, mujoco, name) for name in joint_names
        )
        candidate_geom_ids = tuple(
            finger_geom_id(candidate_model, mujoco, name) for name in joint_names
        )

        side_samples = []
        for position in (0.0, 0.022, 0.044):
            set_gripper_position(
                formal_model, formal_data, mujoco, side, position
            )
            set_gripper_position(
                candidate_model, candidate_data, mujoco, side, position
            )
            formal_separation = math.dist(
                formal_data.geom_xpos[formal_geom_ids[0]],
                formal_data.geom_xpos[formal_geom_ids[1]],
            )
            candidate_separation = math.dist(
                candidate_data.geom_xpos[candidate_geom_ids[0]],
                candidate_data.geom_xpos[candidate_geom_ids[1]],
            )
            gap_error = abs(formal_separation - candidate_separation)
            max_gap_error = max(max_gap_error, gap_error)
            side_samples.append(
                {
                    "joint_position_m": position,
                    "candidate_urdf_compiled_separation_m": candidate_separation,
                    "formal_mjcf_separation_m": formal_separation,
                    "absolute_error_m": gap_error,
                }
            )
        samples[side] = {
            "symmetric_sweep": side_samples,
        }

    add_check(
        checks,
        "cross_model.kinematic_sweep",
        max_gap_error <= 1e-6,
        "compiled URDF and formal MuJoCo finger separations agree across the full stroke",
        f"maximum compiled-model separation error is {max_gap_error:.6f} m",
    )
    return samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--description-root", type=Path, required=True)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--moveit-controllers", type=Path, required=True)
    parser.add_argument("--srdf", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    checks: list[Check] = []
    urdf_root = ET.parse(args.urdf).getroot()
    mjcf_root = ET.parse(args.mjcf).getroot()
    urdf_joints = check_urdf(urdf_root, checks)
    check_mjcf(mjcf_root, urdf_joints, checks)
    check_control_path(args.bridge, args.moveit_controllers, args.srdf, checks)
    sweep = check_mujoco_sweep(
        args.mjcf,
        args.urdf,
        args.description_root,
        checks,
    )

    failed = [check for check in checks if check.blocking and check.status == "FAIL"]
    report = {
        "overall": "PASS" if not failed else "FAIL",
        "checks": [asdict(check) for check in checks],
        "mujoco_sweep": sweep,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    for check in checks:
        print(f"[{check.status}] {check.name}: {check.detail}")
    print(f"report: {args.report}")
    print(f"OPENARM GRIPPER MODEL ALIGNMENT: {report['overall']}")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
