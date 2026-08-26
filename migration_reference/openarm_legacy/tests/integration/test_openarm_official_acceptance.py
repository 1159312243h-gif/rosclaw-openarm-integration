#!/usr/bin/env python3
"""Official ROSClaw + ROS 2 + MoveIt + MuJoCo acceptance for OpenArm."""

from __future__ import annotations

import asyncio
import hashlib
import json
import math
from pathlib import Path
from uuid import UUID

import yaml

from rosclaw.agent.detectors import build_project_profile
from rosclaw.mcp.adapters.runtime_client import RuntimeClient

PROJECT = Path(__file__).resolve().parents[2]
MODEL = PROJECT / "models" / "openarm_dual_mujoco_01" / "robot.mjcf.xml"
MODEL_HASHES = (
    PROJECT / "models" / "openarm_dual_mujoco_01" / "model.sha256"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def expected_model_hashes() -> dict[str, str]:
    return {
        name: digest
        for digest, name in (
            line.split() for line in MODEL_HASHES.read_text(encoding="ascii").splitlines()
        )
    }


async def run() -> None:
    print("OFFICIAL ROSCLAW OPENARM SANDBOX ACCEPTANCE")
    print("ROS 2 + MoveIt 2 + MuJoCo simulation only; no real robot is used.")

    profile = build_project_profile(project_root=PROJECT)
    assert profile.profile_path == PROJECT / "runtime.yaml"
    assert profile.robot_id == "openarm_dual_mujoco_01"
    assert len(profile.runtime_profile) > 0
    yaml.safe_load(MODEL.read_text(encoding="utf-8")) if MODEL.suffix == ".yaml" else None

    expected_hashes = expected_model_hashes()
    model_before = sha256(MODEL)
    assert model_before == expected_hashes[MODEL.name]
    assert sha256(MODEL.with_name("robot.urdf")) == expected_hashes["robot.urdf"]
    client = RuntimeClient(
        project_root=PROJECT,
        robot_id=profile.robot_id,
        runtime_profile=profile.runtime_profile,
    )
    try:
        skills = await client.list_skills(full_ids=True)
        assert any(
            skill.get("name") == "openarm/openarm-move-relative"
            for skill in skills["skills"]
        ), skills
        arguments = {
            "arm": "right",
            "reference_frame": "world",
            "translation": {"x": 0.0, "y": 0.0, "z": 0.002},
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
            "avoid_collisions": True,
        }
        result = await client.sandbox_run(
            capability_id="openarm.arm.move_relative",
            arguments=arguments,
        )
        receipt = result["receipt"]
        action_id = receipt["action_id"]
        assert action_id.startswith("action_")
        action_uuid = UUID(action_id.removeprefix("action_"))
        assert action_uuid.version == 4
        assert str(action_uuid) == action_id.removeprefix("action_")
        assert receipt["mode"] == "SIMULATION"
        assert receipt["body_id"] == "openarm_dual_mujoco_01"
        assert receipt["capability_id"] == "openarm.arm.move_relative"
        assert receipt["final_state"] == "COMPLETED", receipt
        assert receipt["evidence_level"] == "TASK_VERIFIED", receipt
        assert receipt["acknowledgement_stage"] == "TASK_VERIFIED", receipt
        assert receipt["verification_result"]["passed"] is True, receipt
        assert receipt["simulation_result"]["backend"] == "mujoco_moveit"
        assert receipt["simulation_result"]["planning_backend"] == (
            "moveit2_get_cartesian_path"
        )
        assert receipt["simulation_result"]["execution_backend"] == (
            "moveit2_execute_trajectory"
        )
        assert receipt["simulation_result"]["controller_backend"] == (
            "follow_joint_trajectory"
        )
        assert receipt["simulation_result"]["direct_mujoco_fallback"] is False
        assert receipt["simulation_result"]["physics_executed"] is True
        verification = receipt["verification_result"]
        assert verification["moveit_planning_observed"] is True
        assert verification["trajectory_execution_observed"] is True
        assert verification["final_pose_verification_observed"] is True
        assert verification["final_pose_reported"] is True

        assert receipt["normalized_arguments"] == arguments
        canonical_arguments = json.dumps(
            arguments,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
        assert receipt["arguments_sha256"] == hashlib.sha256(
            canonical_arguments
        ).hexdigest()

        initial = receipt["initial_pose"]
        target = receipt["target_pose"]
        final = receipt["final_pose"]
        assert initial["frame_id"] == target["frame_id"] == final["frame_id"] == "world"
        for axis in ("x", "y", "z"):
            assert math.isclose(
                target["position"][axis],
                initial["position"][axis] + arguments["translation"][axis],
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        measured_final_error = math.dist(
            [target["position"][axis] for axis in ("x", "y", "z")],
            [final["position"][axis] for axis in ("x", "y", "z")],
        )
        assert math.isclose(
            measured_final_error,
            receipt["final_error_m"],
            rel_tol=0.0,
            abs_tol=1.0e-6,
        )

        persisted = await client.get_execution_receipt(action_id)
        assert persisted["integrity_verified"] is True
        assert persisted["receipt"]["action_id"] == action_id
        receipt_path = Path(persisted["receipt_path"])
        assert receipt_path.is_file()
        worker_artifact = receipt_path.with_name("moveit_worker_result.json")
        assert worker_artifact.is_file()
        assert sha256(worker_artifact) == receipt["simulation_result"][
            "worker_artifact_sha256"
        ]
        assert sha256(MODEL) == model_before

        print("runtime profile: loaded")
        print("runtime skill: registered")
        print("official MoveIt capability: registered")
        print("final state:", receipt["final_state"])
        print("evidence:", receipt["evidence_level"])
        print("server action id:", action_id)
        print("argument digest: verified")
        print("pose reconstruction: verified")
        print("worker artifact integrity: verified")
        print("receipt integrity: verified")
        print("OpenArm MJCF unchanged:", model_before)
        print("OFFICIAL ROSCLAW OPENARM SANDBOX ACCEPTANCE: PASS")
    finally:
        runtime = getattr(client, "_runtime", None)
        if runtime is not None:
            runtime.stop()


def test_openarm_official_acceptance() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    test_openarm_official_acceptance()
