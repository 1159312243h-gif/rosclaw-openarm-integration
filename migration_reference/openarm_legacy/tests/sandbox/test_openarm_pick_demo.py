"""Focused contract and evidence tests for ``openarm.pick_demo``."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from rosclaw.kernel import ActionEnvelope, ActionState, EvidenceLevel, ExecutionMode
from rosclaw.sandbox.openarm import ROBOT_ID
from rosclaw.sandbox.openarm_pick_demo import (
    CAPABILITY_ID,
    run_openarm_pick_demo_via_moveit,
    validate_openarm_pick_demo,
)


def _action(**arguments: object) -> ActionEnvelope:
    payload: dict[str, object] = {
        "side": "right",
        "object_id": "pick_demo_block",
        "lift_m": 0.015,
    }
    payload.update(arguments)
    return ActionEnvelope(
        action_id="action_pick_demo_test_001",
        actor_id="pytest",
        agent_framework="pytest",
        session_id="pick-demo-test",
        body_id=ROBOT_ID,
        capability_id=CAPABILITY_ID,
        arguments=payload,
        execution_mode=ExecutionMode.SIMULATION,
    )


def _payload(*, rise: float = 0.0145, missing_stage: str | None = None) -> dict[str, object]:
    names = ["scene_ready", "gripper_open", "approach", "gripper_close", "lift", "verify"]
    initial = 0.50
    final = initial + rise
    return {
        "schema_version": "openarm.pick_demo.worker.v1",
        "request_id": "action_pick_demo_test_001",
        "accepted": True,
        "timed_out": False,
        "status": 4,
        "success": True,
        "message": "pick demo verified",
        "stages": [
            {"name": name, "success": True}
            for name in names
            if name != missing_stage
        ],
        "initial_object_z_m": initial,
        "final_object_z_m": final,
        "object_rise_m": rise,
    }


def test_contract_accepts_narrow_deterministic_request() -> None:
    assert validate_openarm_pick_demo(_action()) == {
        "side": "right",
        "object_id": "pick_demo_block",
        "lift_m": 0.015,
    }


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"side": "left"}, "supports only"),
        ({"object_id": "red_block"}, "object_id must be"),
        ({"lift_m": 0.021}, "between 0.010 and 0.020"),
        ({"retry": True}, "unknown arguments"),
    ],
)
def test_invalid_request_is_rejected_before_worker_dispatch(
    tmp_path: Path,
    override: dict[str, object],
    message: str,
) -> None:
    called = False

    def transport(_request: dict[str, object]) -> dict[str, object]:
        nonlocal called
        called = True
        return _payload()

    result = run_openarm_pick_demo_via_moveit(
        _action(**override),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )
    assert result.final_state is ActionState.BLOCKED
    assert message in result.errors[0]["message"]
    assert result.simulation_result["physics_executed"] is False
    assert called is False


def test_complete_object_rise_evidence_is_task_verified(tmp_path: Path) -> None:
    result = run_openarm_pick_demo_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: _payload(),
    )
    assert result.final_state is ActionState.COMPLETED
    assert result.evidence_level is EvidenceLevel.TASK_VERIFIED
    assert result.verification_result["passed"] is True
    assert result.verification_result["object_rise_m"] == pytest.approx(0.0145)
    assert result.errors == []


@pytest.mark.parametrize(
    "payload",
    [_payload(rise=0.0079), _payload(missing_stage="verify")],
)
def test_success_claim_without_complete_task_evidence_is_rejected(
    tmp_path: Path,
    payload: dict[str, object],
) -> None:
    result = run_openarm_pick_demo_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: payload,
    )
    assert result.final_state is ActionState.FAILED
    assert result.evidence_level is not EvidenceLevel.TASK_VERIFIED
    assert result.errors[0]["code"] == "PICK_DEMO_EVIDENCE_INCOMPLETE"


def test_derived_scene_has_one_dynamic_demo_object_and_leaves_formal_model_unchanged() -> None:
    project = Path(__file__).resolve().parents[2]
    formal = project / "models" / ROBOT_ID / "robot.mjcf.xml"
    scene = project / "models" / ROBOT_ID / "robot.pick_demo.mjcf.xml"
    formal_root = ET.parse(formal).getroot()
    scene_root = ET.parse(scene).getroot()
    assert formal_root.find(".//body[@name='pick_demo_block']") is None
    block = scene_root.find(".//body[@name='pick_demo_block']")
    assert block is not None
    free_joint = block.find("joint[@type='free']")
    assert free_joint is not None
    assert free_joint.get("damping") == "0.2"
    assert scene_root.find("option").get("gravity") == "0 0 -9.81"
    support = scene_root.find(".//body[@name='pick_demo_support']")
    assert support is not None
    assert support.find("joint[@type='free']") is None
    assert support.find("geom[@name='pick_demo_support_geom']") is not None
    support_size = [
        float(value)
        for value in support.find("geom[@name='pick_demo_support_geom']").get("size").split()
    ]
    assert support_size[:2] == pytest.approx([0.040, 0.040])
    assert {
        geom.get("name")
        for geom in support.findall("geom")
        if geom.get("name", "").startswith("pick_demo_cradle_")
    } == {
        "pick_demo_cradle_x_negative",
        "pick_demo_cradle_x_positive",
        "pick_demo_cradle_y_negative",
        "pick_demo_cradle_y_positive",
    }
    assert formal_root.find(".//body[@name='pick_demo_support']") is None


def test_pick_runtime_is_fail_closed_and_keeps_strict_arm_tolerance() -> None:
    project = Path(__file__).resolve().parents[2]
    bringup = project / "ros_ws" / "src" / "rosclaw_openarm_bringup"
    worker = (bringup / "scripts" / "rosclaw_pick_demo_worker.py").read_text(
        encoding="utf-8"
    )
    launch = (bringup / "launch" / "openarm_pick_demo.launch.py").read_text(
        encoding="utf-8"
    )
    stability = (project / "scripts" / "check_openarm_pick_demo_scene_stability.py").read_text(
        encoding="utf-8"
    )
    assert "MAX_INITIAL_OFFSET_M = 0.010" in worker
    assert "MAX_SCENE_DRIFT_M = 0.001" in worker
    assert "if not scene_ready:" in worker
    assert '"position_gain": 0.25' in launch
    assert '"goal_tolerance": 0.003' in launch
    assert '"goal_time_margin": 7.0' in launch
    assert '"gripper_initial_position": 0.000' in launch
    assert "OPEN_GRIPPER_POSITION_M = 0.000" in worker
    assert "CLOSE_GRIPPER_POSITION_M = 0.006" in worker
    assert "OPEN_GRIPPER_POSITION_M = 0.000" in stability
    assert "unexpected_contacts = sorted(" in stability
    assert '"unexpected_object_contacts": unexpected_contacts' in stability
    assert "and not unexpected_contacts" in stability
