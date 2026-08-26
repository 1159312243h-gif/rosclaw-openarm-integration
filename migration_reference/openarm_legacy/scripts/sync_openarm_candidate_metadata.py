#!/usr/bin/env python3
"""Synchronize OpenArm e-URDF and safety metadata from a candidate URDF."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


FINGER_CONTRACT = {
    f"openarm_{side}_finger_joint{number}": {
        "axis": [0.0, 1.0 if number == 1 else -1.0, 0.0],
        "mimic": (
            None
            if number == 1
            else {
                "joint": f"openarm_{side}_finger_joint1",
                "multiplier": 1.0,
                "offset": 0.0,
            }
        ),
    }
    for side in ("left", "right")
    for number in (1, 2)
}


def _floats(value: str | None, default: list[float] | None = None) -> list[float]:
    if value is None:
        return list(default or [])
    return [float(item) for item in value.split()]


def _parse_link(node: ET.Element) -> dict[str, Any]:
    result: dict[str, Any] = {"name": node.attrib["name"]}
    mass = node.find("./inertial/mass")
    if mass is not None and mass.get("value") is not None:
        result["mass"] = float(mass.attrib["value"])
    return result


def _parse_joint(node: ET.Element) -> dict[str, Any]:
    parent = node.find("parent")
    child = node.find("child")
    if parent is None or child is None:
        raise ValueError(f"URDF joint lacks parent or child: {node.attrib.get('name')}")

    result: dict[str, Any] = {
        "name": node.attrib["name"],
        "type": node.attrib.get("type", "fixed"),
        "parent": parent.attrib["link"],
        "child": child.attrib["link"],
    }
    axis = node.find("axis")
    if axis is not None:
        result["axis"] = _floats(axis.get("xyz"), [1.0, 0.0, 0.0])

    limit = node.find("limit")
    if limit is not None:
        result["limits"] = {
            key: float(limit.attrib[key])
            for key in ("effort", "lower", "upper", "velocity")
            if key in limit.attrib
        }

    origin = node.find("origin")
    if origin is not None:
        result["origin"] = {
            key: _floats(origin.get(key), [0.0, 0.0, 0.0])
            for key in ("xyz", "rpy")
            if origin.get(key) is not None
        }

    mimic = node.find("mimic")
    if mimic is not None:
        result["mimic"] = {
            "joint": mimic.attrib["joint"],
            "multiplier": float(mimic.attrib.get("multiplier", "1")),
            "offset": float(mimic.attrib.get("offset", "0")),
        }
    return result


def parse_urdf(path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"Expected a URDF robot root in {path}")
    return (
        [_parse_link(node) for node in root.findall("link")],
        [_parse_joint(node) for node in root.findall("joint")],
    )


def _same(left: Any, right: Any) -> bool:
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), abs_tol=1e-9)
    if isinstance(left, list) and isinstance(right, list) and len(left) == len(right):
        return all(_same(a, b) for a, b in zip(left, right, strict=True))
    if isinstance(left, dict) and isinstance(right, dict) and left.keys() == right.keys():
        return all(_same(left[key], right[key]) for key in left)
    return left == right


def _require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a YAML mapping")
    return value


def validate_parallel_finger_contract(joints: list[dict[str, Any]]) -> None:
    by_name = {joint["name"]: joint for joint in joints}
    for name, expected in FINGER_CONTRACT.items():
        joint = by_name.get(name)
        if joint is None:
            raise ValueError(f"Candidate URDF is missing {name}")
        if joint["type"] != "prismatic":
            raise ValueError(f"{name} type is {joint['type']}, expected prismatic")
        if not _same(joint.get("axis"), expected["axis"]):
            raise ValueError(f"{name} axis is {joint.get('axis')}, expected {expected['axis']}")
        limits = joint.get("limits", {})
        for key, value in (("lower", 0.0), ("upper", 0.044)):
            if not _same(limits.get(key), value):
                raise ValueError(f"{name} {key} limit is {limits.get(key)}, expected {value}")
        if float(limits.get("velocity", 0.0)) <= 0.0:
            raise ValueError(f"{name} must have a positive velocity limit")
        if not _same(joint.get("mimic"), expected["mimic"]):
            raise ValueError(
                f"{name} mimic is {joint.get('mimic')}, expected {expected['mimic']}"
            )


def _merge_links(
    parsed: list[dict[str, Any]], existing: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    existing_by_name = {link.get("name"): link for link in existing}
    merged = []
    for link in parsed:
        item = dict(existing_by_name.get(link["name"], {}))
        item.update(link)
        if link["name"] == "world":
            item["type"] = "world"
        else:
            item.setdefault("type", "link")
        merged.append(item)
    return merged


def _joint_limit_maps(
    joints: list[dict[str, Any]],
) -> tuple[dict[str, dict[str, float]], dict[str, float]]:
    position: dict[str, dict[str, float]] = {}
    velocity: dict[str, float] = {}
    for joint in joints:
        limits = joint.get("limits", {})
        name = joint["name"]
        if "lower" in limits and "upper" in limits:
            position[name] = {"lower": limits["lower"], "upper": limits["upper"]}
        if "velocity" in limits:
            velocity[name] = limits["velocity"]
    return position, velocity


def _write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=False), encoding="ascii"
    )


def synchronize(urdf: Path, eurdf: Path, safety: Path) -> dict[str, Any]:
    links, joints = parse_urdf(urdf)
    validate_parallel_finger_contract(joints)

    eurdf_data = yaml.safe_load(eurdf.read_text(encoding="utf-8")) or {}
    safety_data = yaml.safe_load(safety.read_text(encoding="utf-8")) or {}
    if not isinstance(eurdf_data, dict) or not isinstance(safety_data, dict):
        raise ValueError("e-URDF and safety metadata must be YAML mappings")

    position_limits, velocity_limits = _joint_limit_maps(joints)
    independent_dof = sum(
        joint["type"] not in {"fixed", "floating"} and "mimic" not in joint
        for joint in joints
    )
    movable_joint_count = sum(
        joint["type"] not in {"fixed", "floating"} for joint in joints
    )

    eurdf_data["links"] = _merge_links(links, eurdf_data.get("links", []))
    eurdf_data["joints"] = joints
    eurdf_data["dof"] = independent_dof
    eurdf_safety = _require_mapping(
        eurdf_data.setdefault("safety_limits", {}), "e-URDF safety_limits"
    )
    eurdf_safety["joint_limits"] = position_limits
    eurdf_safety["velocity_limits"] = velocity_limits
    eurdf_metadata = _require_mapping(
        eurdf_data.setdefault("metadata", {}), "e-URDF metadata"
    )
    eurdf_metadata["urdf_source"] = "robot.urdf"
    eurdf_metadata["movable_joint_count"] = movable_joint_count
    eurdf_metadata["independent_dof"] = independent_dof

    safety_limits = _require_mapping(
        safety_data.setdefault("safety_limits", {}), "safety safety_limits"
    )
    safety_limits["joint_limits"] = position_limits
    safety_limits["velocity_limits"] = velocity_limits

    _write_yaml(eurdf, eurdf_data)
    _write_yaml(safety, safety_data)
    validate_metadata(urdf, eurdf, safety)
    return {
        "status": "PASS",
        "urdf": str(urdf.resolve()),
        "eurdf": str(eurdf.resolve()),
        "safety": str(safety.resolve()),
        "movable_joint_count": movable_joint_count,
        "independent_dof": independent_dof,
        "finger_joints": sorted(FINGER_CONTRACT),
    }


def validate_metadata(urdf: Path, eurdf: Path, safety: Path) -> None:
    links, joints = parse_urdf(urdf)
    validate_parallel_finger_contract(joints)
    eurdf_data = yaml.safe_load(eurdf.read_text(encoding="utf-8")) or {}
    safety_data = yaml.safe_load(safety.read_text(encoding="utf-8")) or {}
    if not isinstance(eurdf_data, dict) or not isinstance(safety_data, dict):
        raise ValueError("e-URDF and safety metadata must be YAML mappings")
    position_limits, velocity_limits = _joint_limit_maps(joints)

    eurdf_safety = _require_mapping(
        eurdf_data.get("safety_limits"), "e-URDF safety_limits"
    )
    safety_limits = _require_mapping(
        safety_data.get("safety_limits"), "safety safety_limits"
    )

    checks = {
        "e-URDF links": eurdf_data.get("links"),
        "e-URDF joints": eurdf_data.get("joints"),
        "e-URDF joint limits": eurdf_safety.get("joint_limits"),
        "e-URDF velocity limits": eurdf_safety.get("velocity_limits"),
        "safety joint limits": safety_limits.get("joint_limits"),
        "safety velocity limits": safety_limits.get("velocity_limits"),
    }
    expected = {
        "e-URDF links": _merge_links(links, eurdf_data.get("links", [])),
        "e-URDF joints": joints,
        "e-URDF joint limits": position_limits,
        "e-URDF velocity limits": velocity_limits,
        "safety joint limits": position_limits,
        "safety velocity limits": velocity_limits,
    }
    for label, actual in checks.items():
        if not _same(actual, expected[label]):
            raise ValueError(f"{label} does not match candidate URDF")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--eurdf", type=Path, required=True)
    parser.add_argument("--safety", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.check:
        validate_metadata(args.urdf, args.eurdf, args.safety)
        result = {"status": "PASS", "mode": "check-only"}
    else:
        result = synchronize(args.urdf, args.eurdf, args.safety)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
