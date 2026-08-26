#!/usr/bin/env python3
"""Validate a generated OpenArm parallel-gripper URDF candidate."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


PARALLEL_FINGER_OFFSET_M = 0.0334627307176665
SOURCE_EXPECTED = {
    "openarm_left_finger_joint1": (
        (0.0, -PARALLEL_FINGER_OFFSET_M, 0.1025),
        "0 1 0",
        0.0,
        0.044,
        None,
    ),
    "openarm_left_finger_joint2": (
        (0.0, PARALLEL_FINGER_OFFSET_M, 0.1025),
        "0 -1 0",
        0.0,
        0.044,
        "openarm_left_finger_joint1",
    ),
    "openarm_right_finger_joint1": (
        (0.0, -PARALLEL_FINGER_OFFSET_M, 0.1025),
        "0 1 0",
        0.0,
        0.044,
        None,
    ),
    "openarm_right_finger_joint2": (
        (0.0, PARALLEL_FINGER_OFFSET_M, 0.1025),
        "0 -1 0",
        0.0,
        0.044,
        "openarm_right_finger_joint1",
    ),
}

COLLAPSED_EXPECTED_ORIGINS = {
    f"openarm_{side}_finger_joint1": (
        0.0,
        PARALLEL_FINGER_OFFSET_M,
        -0.1025,
    )
    for side in ("left", "right")
}
COLLAPSED_EXPECTED_ORIGINS.update(
    {
        f"openarm_{side}_finger_joint2": (
            0.0,
            -PARALLEL_FINGER_OFFSET_M,
            -0.1025,
        )
        for side in ("left", "right")
    }
)

EXPECTED_ARM_TAIL = {
    "openarm_left_joint6": ((0.0375, 0.0, -0.1205), (1.0, 0.0, 0.0)),
    "openarm_left_joint7": ((-0.0375, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "openarm_right_joint6": ((0.0375, 0.0, -0.1205), (1.0, 0.0, 0.0)),
    "openarm_right_joint7": ((-0.0375, 0.0, 0.0), (0.0, -1.0, 0.0)),
}


def normalized_vector(value: str) -> tuple[float, float, float]:
    parts = tuple(float(item) for item in value.split())
    if len(parts) != 3:
        raise ValueError(f"expected three-vector, got {value!r}")
    return parts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument(
        "--expect",
        choices=("parallel", "pinch"),
        default="parallel",
    )
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}

    for name, (expected_origin, expected_axis) in EXPECTED_ARM_TAIL.items():
        joint = joints.get(name)
        assert joint is not None, f"missing corrected arm joint: {name}"
        origin = joint.find("origin")
        axis = joint.find("axis")
        assert origin is not None, f"missing origin: {name}"
        assert axis is not None, f"missing axis: {name}"
        actual_origin = normalized_vector(origin.get("xyz"))
        actual_axis = normalized_vector(axis.get("xyz"))
        assert actual_origin == expected_origin, (name, actual_origin)
        assert actual_axis == expected_axis, (name, actual_axis)
    print("OPENARM JOINT6/JOINT7 CANONICAL KINEMATICS: PASS")

    if args.expect == "pinch":
        for name in SOURCE_EXPECTED:
            joint = joints.get(name)
            assert joint is not None, f"missing joint: {name}"
            assert joint.get("type") == "revolute", (name, joint.get("type"))
        print("OPENARM DEFAULT PINCH GRIPPER REGRESSION: PASS")
        return 0

    mounts = {
        side: joints.get(f"openarm_{side}_ee_mount_joint")
        for side in ("left", "right")
    }
    assert all(mount is None for mount in mounts.values()) or all(
        mount is not None for mount in mounts.values()
    ), "parallel mount collapse must be symmetric"
    mount_collapsed = all(mount is None for mount in mounts.values())

    if not mount_collapsed:
        for side, mount in mounts.items():
            origin = mount.find("origin")
            assert origin is not None, f"missing parallel mount origin: {side}"
            assert normalized_vector(origin.get("xyz")) == (0.0, 0.0, 0.0)
            rpy = normalized_vector(origin.get("rpy"))
            assert abs(abs(rpy[0]) - 3.141592653589793) <= 1e-12, (side, rpy)
            assert abs(rpy[1]) <= 1e-12 and abs(rpy[2]) <= 1e-12, (side, rpy)
        print("OPENARM PARALLEL MOUNT FRAME: PASS")
    else:
        print("OPENARM PARALLEL MOUNT FRAME COLLAPSED: DETECTED")

    for name, (source_origin, axis, lower, upper, mimic_target) in SOURCE_EXPECTED.items():
        joint = joints.get(name)
        assert joint is not None, f"missing joint: {name}"
        assert joint.get("type") == "prismatic", (name, joint.get("type"))

        origin = joint.find("origin")
        assert origin is not None, f"missing origin: {name}"
        actual_origin = normalized_vector(joint.find("origin").get("xyz"))
        expected_origin = (
            COLLAPSED_EXPECTED_ORIGINS[name]
            if mount_collapsed
            else source_origin
        )
        assert all(
            abs(actual - expected) <= 1e-12
            for actual, expected in zip(actual_origin, expected_origin, strict=True)
        ), (name, actual_origin)
        actual_rpy = normalized_vector(origin.get("rpy"))
        if mount_collapsed:
            assert abs(abs(actual_rpy[0]) - 3.141592653589793) <= 1e-12, (
                name,
                actual_rpy,
            )
            assert abs(actual_rpy[1]) <= 1e-12, (name, actual_rpy)
            assert abs(actual_rpy[2]) <= 1e-12, (name, actual_rpy)
        else:
            assert all(abs(value) <= 1e-12 for value in actual_rpy), (
                name,
                actual_rpy,
            )
        actual_axis = normalized_vector(joint.find("axis").get("xyz"))
        expected_axis = normalized_vector(axis)
        assert actual_axis == expected_axis, (name, actual_axis)

        limit = joint.find("limit")
        assert limit is not None, f"missing limit: {name}"
        assert float(limit.get("lower")) == lower, (name, limit.get("lower"))
        assert float(limit.get("upper")) == upper, (name, limit.get("upper"))

        mimic = joint.find("mimic")
        if mimic_target is None:
            assert mimic is None, f"unexpected mimic: {name}"
        else:
            assert mimic is not None, f"missing mimic: {name}"
            assert mimic.get("joint") == mimic_target, (name, mimic.get("joint"))
            assert float(mimic.get("multiplier", "1")) == 1.0
            assert float(mimic.get("offset", "0")) == 0.0

    if args.baseline:
        baseline_root = ET.parse(args.baseline).getroot()
        baseline_joints = {
            joint.get("name"): joint for joint in baseline_root.findall("joint")
        }

        def signature(joint: ET.Element) -> tuple:
            def attributes(tag: str) -> tuple[tuple[str, str], ...]:
                node = joint.find(tag)
                return tuple(sorted(node.attrib.items())) if node is not None else ()

            return (
                joint.get("type"),
                attributes("parent"),
                attributes("child"),
                attributes("origin"),
                attributes("axis"),
                attributes("limit"),
            )

        # joint6/joint7 intentionally replace the incompatible historical v2
        # signatures; joints 1-5 must remain byte-for-byte equivalent.
        unchanged_arm_names = [
            f"openarm_{side}_joint{number}"
            for side in ("left", "right")
            for number in range(1, 6)
        ]
        differences = [
            name
            for name in unchanged_arm_names
            if name not in joints
            or name not in baseline_joints
            or signature(joints[name]) != signature(baseline_joints[name])
        ]
        assert not differences, f"arm joint differences: {differences}"
        print("OPENARM JOINT1-JOINT5 SIGNATURES UNCHANGED: PASS")

    print("OPENARM PARALLEL GRIPPER URDF CANDIDATE: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
