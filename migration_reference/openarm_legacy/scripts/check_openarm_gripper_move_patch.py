#!/usr/bin/env python3
"""Dependency-free structural checks for the gripper capability patch."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def main() -> int:
    python_files = [
        "src/rosclaw/sandbox/openarm_gripper_moveit.py",
        "src/rosclaw/sandbox/runtime_adapter.py",
        "ros_ws/src/rosclaw_openarm_bringup/scripts/rosclaw_moveit_action_worker.py",
        "ros_ws/src/rosclaw_openarm_bringup/launch/openarm_sim.launch.py",
        "tests/sandbox/test_openarm_gripper_moveit_capability.py",
    ]
    for relative in python_files:
        ast.parse(source(relative), filename=relative)

    executor = source("src/rosclaw/sandbox/openarm_gripper_moveit.py")
    adapter = source("src/rosclaw/sandbox/runtime_adapter.py")
    worker = source(
        "ros_ws/src/rosclaw_openarm_bringup/scripts/rosclaw_moveit_action_worker.py"
    )
    launch = source("ros_ws/src/rosclaw_openarm_bringup/launch/openarm_sim.launch.py")
    assert 'CAPABILITY_ID = "openarm.gripper.move"' in executor
    assert 'MIN_TARGET_M = 0.0' in executor
    assert 'MAX_TARGET_M = 0.044' in executor
    assert "validate_openarm_gripper_move(action)" in executor
    assert "transport(worker_request)" in executor
    assert '"openarm.gripper.move"' in adapter
    assert 'GRIPPER_SCHEMA_VERSION = "openarm.gripper.moveit.worker.v1"' in worker
    assert 'goal.request.group_name = f"{side}_gripper"' in worker
    assert "goal.request.num_planning_attempts = 1" in worker
    assert "goal.planning_options.replan = False" in worker
    assert "requested_speed_mps" in worker
    assert '"enable_gripper_controllers": True' in launch
    assert '"enable_gripper_controllers": "true"' in launch

    for relative in (
        "models/openarm_dual_mujoco_01/capabilities.yaml",
        "models/openarm_dual_mujoco_01/semantic.yaml",
        "models/openarm_dual_mujoco_01/robot.eurdf.yaml",
        "models/openarm_dual_mujoco_01/safety.yaml",
    ):
        assert "openarm.gripper.move" in source(relative), relative
    print("OPENARM GRIPPER MOVE PATCH STRUCTURE: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
