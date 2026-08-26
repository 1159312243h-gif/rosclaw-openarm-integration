#!/usr/bin/env python3
"""Fail-closed structural checks for the OpenArm pick-demo patch."""

from __future__ import annotations

import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

FORMAL_MJCF_SHA256 = "ae35f356ad09bd68a30bb69e4419d540e6b9c1c011e061e04b1a448af3d4265e"
FORMAL_URDF_SHA256 = "17ad3c2fb58f34d132f91a1a077d351f12153b6cef1964de3ef87e5fe68c7b12"
CAPABILITY = "openarm.pick_demo"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def main() -> int:
    project = Path(__file__).resolve().parents[1]
    model = project / "models" / "openarm_dual_mujoco_01"
    formal_mjcf = model / "robot.mjcf.xml"
    formal_urdf = model / "robot.urdf"
    scene = model / "robot.pick_demo.mjcf.xml"
    require(digest(formal_mjcf) == FORMAL_MJCF_SHA256, "formal MJCF hash changed")
    require(digest(formal_urdf) == FORMAL_URDF_SHA256, "formal URDF hash changed")
    root = ET.parse(scene).getroot()
    require(root.get("model") == "openarm_pick_demo_v8", "pick scene model id is wrong")
    option = root.find("option")
    require(
        option is not None and option.get("gravity") == "0 0 -9.81",
        "pick scene must use explicit Earth gravity",
    )
    block = root.find(".//body[@name='pick_demo_block']")
    free_joint = block.find("joint[@type='free']") if block is not None else None
    require(free_joint is not None, "dynamic pick block is missing")
    require(free_joint.get("damping") == "0.2", "pick block damping is not pinned")
    geom = block.find("geom")
    require(geom is not None and geom.get("name") == "pick_demo_block_geom", "pick block geom is missing")
    require(geom.get("size") == "0.01500 0.02000 0.01500", "pick block dimensions changed")
    support = root.find(".//body[@name='pick_demo_support']")
    require(
        support is not None and support.find("joint[@type='free']") is None,
        "static pick support is missing",
    )
    support_geom = support.find("geom[@name='pick_demo_support_geom']")
    require(
        support_geom is not None and support_geom.get("name") == "pick_demo_support_geom",
        "pick support geom is missing",
    )
    support_z = float(support.get("pos").split()[2])
    support_half_z = float(support_geom.get("size").split()[2])
    support_half_x, support_half_y, _ = map(float, support_geom.get("size").split())
    block_z = float(block.get("pos").split()[2])
    block_half_z = float(geom.get("size").split()[2])
    require(
        abs((support_z + support_half_z) - (block_z - block_half_z)) <= 2e-6,
        "pick support does not meet the block bottom",
    )
    block_half_x, block_half_y, _ = map(float, geom.get("size").split())
    require(
        support_half_x >= block_half_x and support_half_y >= block_half_y,
        "pick support does not fully cover the block footprint",
    )
    guide_names = {
        "pick_demo_cradle_x_negative",
        "pick_demo_cradle_x_positive",
        "pick_demo_cradle_y_negative",
        "pick_demo_cradle_y_positive",
    }
    guides = {
        geom.get("name"): geom
        for geom in support.findall("geom")
        if geom.get("name") in guide_names
    }
    require(set(guides) == guide_names, "passive cradle guides are incomplete")
    require(
        all(float(geom.get("size").split()[2]) * 2.0 <= 0.003 for geom in guides.values()),
        "passive cradle guides are too tall",
    )
    x_inner_edge = abs(float(guides["pick_demo_cradle_x_positive"].get("pos").split()[0])) - float(
        guides["pick_demo_cradle_x_positive"].get("size").split()[0]
    )
    y_inner_edge = abs(float(guides["pick_demo_cradle_y_positive"].get("pos").split()[1])) - float(
        guides["pick_demo_cradle_y_positive"].get("size").split()[1]
    )
    require(
        x_inner_edge - block_half_x >= 0.0015
        and y_inner_edge - block_half_y >= 0.0015,
        "passive cradle guides do not leave enough initial clearance",
    )
    formal_root = ET.parse(formal_mjcf).getroot()
    require(formal_root.find(".//body[@name='pick_demo_block']") is None, "formal model contains demo object")
    require(formal_root.find(".//body[@name='pick_demo_support']") is None, "formal model contains demo support")

    capabilities = yaml.safe_load((model / "capabilities.yaml").read_text(encoding="utf-8"))
    ids = {entry.get("id") for entry in capabilities.get("capabilities", [])}
    require(CAPABILITY in ids, "capability manifest does not expose openarm.pick_demo")
    eurdf = yaml.safe_load((model / "robot.eurdf.yaml").read_text(encoding="utf-8"))
    require(CAPABILITY in {entry.get("name") for entry in eurdf.get("capabilities", [])}, "e-URDF does not expose openarm.pick_demo")
    semantic = yaml.safe_load((model / "semantic.yaml").read_text(encoding="utf-8"))
    require(CAPABILITY in semantic.get("task_descriptions", {}), "semantic task description is missing")
    safety = yaml.safe_load((model / "safety.yaml").read_text(encoding="utf-8"))
    limits = safety.get("safety_limits", {}).get("capability_limits", {})
    require(CAPABILITY in limits, "safety limits are missing openarm.pick_demo")

    worker = (project / "ros_ws" / "src" / "rosclaw_openarm_bringup" / "scripts" / "rosclaw_pick_demo_worker.py")
    launch = (project / "ros_ws" / "src" / "rosclaw_openarm_bringup" / "launch" / "openarm_pick_demo.launch.py")
    stability = project / "scripts" / "check_openarm_pick_demo_scene_stability.py"
    require(worker.is_file() and launch.is_file(), "pick worker or launch file is missing")
    worker_source = worker.read_text(encoding="utf-8")
    require("MAX_INITIAL_OFFSET_M = 0.010" in worker_source, "scene position guard is missing")
    require("MAX_SCENE_DRIFT_M = 0.001" in worker_source, "scene stability guard is missing")
    require("if not scene_ready:" in worker_source, "scene readiness is not fail-closed")
    launch_source = launch.read_text(encoding="utf-8")
    require('"position_gain": 0.25' in launch_source, "pick position gain is not pinned")
    require('"goal_tolerance": 0.003' in launch_source, "pick goal tolerance changed")
    require('"goal_time_margin": 7.0' in launch_source, "pick goal time margin is not pinned")
    require(
        '"gripper_initial_position": 0.000' in launch_source,
        "pick initial gripper clearance is not pinned",
    )
    require(
        "OPEN_GRIPPER_POSITION_M = 0.000" in worker_source,
        "pick open-gripper clearance is not pinned",
    )
    require(
        "CLOSE_GRIPPER_POSITION_M = 0.006" in worker_source,
        "pick contact target is not pinned",
    )
    stability_source = stability.read_text(encoding="utf-8")
    require(
        "OPEN_GRIPPER_POSITION_M = 0.000" in stability_source,
        "stability gate gripper clearance is not pinned",
    )
    require(
        "unexpected_contacts = sorted(" in stability_source
        and '"unexpected_object_contacts": unexpected_contacts' in stability_source
        and "and not unexpected_contacts" in stability_source,
        "stability gate robot-contact guard is missing",
    )
    print("OPENARM PICK DEMO PATCH STRUCTURE: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
