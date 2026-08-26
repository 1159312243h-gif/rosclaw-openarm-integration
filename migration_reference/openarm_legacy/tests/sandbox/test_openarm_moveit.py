"""Fail-closed tests for the MoveIt-backed OpenArm sandbox executor."""

from __future__ import annotations

import ast
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import pytest
import yaml

from rosclaw.core.event_bus import EventBus
from rosclaw.kernel import ActionEnvelope, ActionGateway, ActionState, ExecutionMode
from rosclaw.sandbox.openarm import CAPABILITY_ID, ROBOT_ID
from rosclaw.sandbox.openarm_moveit import run_openarm_move_relative_via_moveit
from rosclaw.sandbox.runtime_adapter import SandboxRuntimeAdapter
from scripts.prepare_openarm_candidate_runtime import prepare
from scripts.sync_openarm_candidate_metadata import validate_metadata
from scripts.verify_openarm_candidate_binding import verify


def _action(action_id: str = "action_moveit_test_001") -> ActionEnvelope:
    return ActionEnvelope(
        action_id=action_id,
        actor_id="pytest",
        agent_framework="pytest",
        session_id="session-test",
        body_id=ROBOT_ID,
        capability_id=CAPABILITY_ID,
        arguments={
            "arm": "right",
            "reference_frame": "world",
            "translation": {"x": 0.0, "y": 0.0, "z": 0.002},
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
            "avoid_collisions": True,
        },
        execution_mode=ExecutionMode.SIMULATION,
    )


def _worker_payload(*, phases: list[int], success: bool = True) -> dict[str, Any]:
    return {
        "schema_version": "openarm.moveit.worker.v1",
        "request_id": "action_moveit_test_001",
        "server_available": True,
        "accepted": True,
        "timed_out": False,
        "status": 4 if success else 6,
        "success": success,
        "error_code": 0 if success else -6,
        "message": "Cartesian motion completed" if success else "execution failed",
        "goal_id": "001122",
        "initial_pose": {
            "frame_id": "world",
            "position": {"x": 0.1, "y": -0.2, "z": 0.498},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "target_pose": {
            "frame_id": "world",
            "position": {"x": 0.1, "y": -0.2, "z": 0.5},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "final_pose": {
            "frame_id": "world",
            "position": {"x": 0.1, "y": -0.2, "z": 0.5},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "final_error_m": 0.0,
        "feedback": [
            {"phase": phase, "progress": index / max(1, len(phases)), "message": "ok"}
            for index, phase in enumerate(phases, start=1)
        ],
    }


def _parallel_gripper_urdf(*, joint_type: str = "prismatic") -> str:
    links = ["world", "openarm_body_link0"]
    joints = [
        "<joint name='body_mount' type='fixed'>"
        "<parent link='world'/><child link='openarm_body_link0'/></joint>"
    ]
    for side in ("left", "right"):
        base = f"openarm_{side}_ee_base_link"
        links.append(base)
        joints.append(
            f"<joint name='{side}_base_mount' type='fixed'>"
            f"<parent link='openarm_body_link0'/><child link='{base}'/></joint>"
        )
        for number, axis in ((1, "0 1 0"), (2, "0 -1 0")):
            link = f"openarm_{side}_ee_link{number}"
            name = f"openarm_{side}_finger_joint{number}"
            links.append(link)
            mimic = (
                ""
                if number == 1
                else f"<mimic joint='openarm_{side}_finger_joint1' "
                "multiplier='1.0' offset='0.0'/>"
            )
            joints.append(
                f"<joint name='{name}' type='{joint_type}'>"
                f"<parent link='{base}'/><child link='{link}'/><axis xyz='{axis}'/>"
                "<limit effort='9.0' lower='0.0' upper='0.044' velocity='20.943946'/>"
                f"{mimic}</joint>"
            )
    link_xml = "".join(f"<link name='{name}'/>" for name in links)
    return f"<robot name='candidate'>{link_xml}{''.join(joints)}</robot>"


def _launch_parameter(path: Path, name: str) -> float:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values, strict=False):
            if isinstance(key, ast.Constant) and key.value == name:
                result = ast.literal_eval(value)
                if isinstance(result, (int, float)):
                    return float(result)
    raise AssertionError(f"Launch parameter not found: {name}")


def _function_assignment(path: Path, function: str, name: str) -> Any:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    definition = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == function
    )
    for node in ast.walk(definition):
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"Assignment not found in {function}: {name}")


def _module_assignment(path: Path, name: str) -> Any:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"Module assignment not found: {name}")


def test_parallel_gripper_launch_validator_uses_collapsed_mount_axes() -> None:
    project = Path(__file__).resolve().parents[2]
    moveit_launch = (
        project
        / "ros_ws/src/openarm_moveit_config/launch/moveit_minimal.launch.py"
    )

    assert _function_assignment(
        moveit_launch,
        "validate_parallel_gripper_urdf",
        "expected_axes",
    ) == {
        1: (0.0, 1.0, 0.0),
        2: (0.0, -1.0, 0.0),
    }


def test_parallel_gripper_moveit_limits_include_acceleration() -> None:
    project = Path(__file__).resolve().parents[2]
    moveit_launch = (
        project
        / "ros_ws/src/openarm_moveit_config/launch/moveit_minimal.launch.py"
    )

    assert _module_assignment(
        moveit_launch, "PARALLEL_GRIPPER_MAX_VELOCITY_MPS"
    ) == 0.05
    assert _module_assignment(
        moveit_launch, "PARALLEL_GRIPPER_MAX_ACCELERATION_MPS2"
    ) == 0.2
    source = moveit_launch.read_text(encoding="utf-8")
    assert '"has_acceleration_limits": True' in source
    assert "add_parallel_gripper_joint_limits(joint_limits)" in source


def test_parallel_gripper_state_boundary_normalization_is_narrow() -> None:
    project = Path(__file__).resolve().parents[2]
    bridge = (
        project
        / "ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py"
    )

    assert _module_assignment(
        bridge, "GRIPPER_STATE_BOUNDARY_TOLERANCE_M"
    ) == 0.00005
    source = bridge.read_text(encoding="utf-8")
    assert "return normalize_joint_state_position(name, position)" in source
    assert "lower - tolerance <= position < lower" in source
    assert "upper < position <= upper + tolerance" in source


def test_dual_arm_controller_and_execution_deadline_configuration() -> None:
    project = Path(__file__).resolve().parents[2]
    controller_config = yaml.safe_load(
        (
            project
            / "ros_ws/src/openarm_moveit_config/config/moveit_controllers.yaml"
        ).read_text(encoding="utf-8")
    )
    bridge_config = yaml.safe_load(
        (
            project
            / "ros_ws/src/rosclaw_openarm_bringup/config/openarm_sim.yaml"
        ).read_text(encoding="utf-8")
    )["mujoco_moveit_bridge"]["ros__parameters"]
    moveit_launch = (
        project
        / "ros_ws/src/openarm_moveit_config/launch/moveit_minimal.launch.py"
    )

    manager = controller_config["moveit_simple_controller_manager"]
    assert manager["controller_names"] == [
        "left_arm_controller",
        "right_arm_controller",
    ]
    assert manager["right_arm_controller"]["joints"] == [
        f"openarm_right_joint{index}" for index in range(1, 8)
    ]
    assert bridge_config["goal_tolerance"] == 0.003
    moveit_margin = _launch_parameter(
        moveit_launch,
        "trajectory_execution.allowed_goal_duration_margin",
    )
    assert moveit_margin > bridge_config["goal_time_margin"]


def test_candidate_arm_gain_is_isolated_from_formal_config() -> None:
    project = Path(__file__).resolve().parents[2]
    formal_config = yaml.safe_load(
        (
            project
            / "ros_ws/src/rosclaw_openarm_bringup/config/openarm_sim.yaml"
        ).read_text(encoding="utf-8")
    )["mujoco_moveit_bridge"]["ros__parameters"]
    candidate_launch = (
        project
        / "ros_ws/src/rosclaw_openarm_bringup/launch/"
        "openarm_moveit_gripper_candidate.launch.py"
    ).read_text(encoding="utf-8")

    assert formal_config["position_gain"] == 0.15
    assert formal_config["goal_tolerance"] == 0.003
    assert 'default_value="0.25"' in candidate_launch
    assert '"position_gain": ParameterValue(' in candidate_launch


def test_candidate_mcp_activation_links_body_inside_candidate_home() -> None:
    project = Path(__file__).resolve().parents[2]
    script = (project / "scripts/activate_openarm_candidate_mcp.sh").read_text(
        encoding="utf-8"
    )

    assert 'profile["robot"]["zoo_path"]' in script
    assert 'profile["robot"]["id"]' in script
    assert 'ROSCLAW_HOME="$CANDIDATE_HOME"' in script
    assert 'ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO"' in script
    assert 'body link-eurdf "$BODY_ID"' in script
    assert '--workspace "$CANDIDATE_HOME"' in script
    assert '--env ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO"' in script


def test_candidate_body_link_helper_is_isolated_and_hash_checked() -> None:
    project = Path(__file__).resolve().parents[2]
    script = (project / "scripts/link_openarm_candidate_body.sh").read_text(
        encoding="utf-8"
    )

    assert 'profile["workspace"]["home"]' in script
    assert 'profile["robot"]["zoo_path"]' in script
    assert '[[ "$BODY_ID" == "openarm_dual_mujoco_01" ]]' in script
    assert "sha256sum -c model.sha256" in script
    assert 'export ROSCLAW_HOME="$CANDIDATE_HOME"' in script
    assert 'export ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO"' in script
    assert 'body link-eurdf "$BODY_ID"' in script
    assert '--workspace "$CANDIDATE_HOME"' in script


def test_candidate_runtime_binds_receipt_model_to_candidate(tmp_path: Path) -> None:
    project = tmp_path / "project"
    body = project / "models" / "openarm_dual_mujoco_01"
    candidate = tmp_path / "candidate"
    output = tmp_path / "runtime"
    body.mkdir(parents=True)
    candidate.mkdir()

    for name in (
        "benchmark.yaml",
        "capabilities.yaml",
        "robot.eurdf.yaml",
        "safety.yaml",
        "semantic.yaml",
    ):
        (body / name).write_text(f"name: {name}\n", encoding="ascii")
    (body / "robot.mjcf.xml").write_text("<mujoco model='formal'/>", encoding="ascii")
    (candidate / "robot.urdf").write_text(
        _parallel_gripper_urdf(), encoding="ascii"
    )
    (candidate / "robot.mjcf.xml").write_text(
        "<mujoco model='candidate'/>", encoding="ascii"
    )

    provenance = prepare(project, candidate, output)
    runtime = output / "runtime.yaml"
    result = verify(
        candidate,
        runtime,
        candidate / "robot.mjcf.xml",
        candidate / "robot.urdf",
    )
    profile = yaml.safe_load(runtime.read_text(encoding="ascii"))

    assert result["status"] == "PASS"
    assert provenance["candidate_hashes"]["robot.mjcf.xml"] == result[
        "runtime_mjcf_sha256"
    ]
    assert provenance["formal_model_hash"] != result["runtime_mjcf_sha256"]
    assert profile["robot"]["zoo_path"] == str(output / "models")
    assert profile["workspace"]["home"] == str(output / "home")
    assert profile["sandbox"]["artifact_root"] == str(output / "artifacts/sandbox")
    assert provenance["metadata_synchronization"]["status"] == "PASS"
    validate_metadata(
        output / "models/openarm_dual_mujoco_01/robot.urdf",
        output / "models/openarm_dual_mujoco_01/robot.eurdf.yaml",
        output / "models/openarm_dual_mujoco_01/safety.yaml",
    )
    eurdf = yaml.safe_load(
        (output / "models/openarm_dual_mujoco_01/robot.eurdf.yaml").read_text(
            encoding="ascii"
        )
    )
    finger_joints = {
        joint["name"]: joint
        for joint in eurdf["joints"]
        if "finger_joint" in joint["name"]
    }
    assert set(finger_joints) == {
        "openarm_left_finger_joint1",
        "openarm_left_finger_joint2",
        "openarm_right_finger_joint1",
        "openarm_right_finger_joint2",
    }
    assert all(joint["type"] == "prismatic" for joint in finger_joints.values())
    assert finger_joints["openarm_left_finger_joint2"]["mimic"]["multiplier"] == 1.0


def test_candidate_binding_fails_for_a_different_active_model(tmp_path: Path) -> None:
    project = tmp_path / "project"
    body = project / "models" / "openarm_dual_mujoco_01"
    candidate = tmp_path / "candidate"
    output = tmp_path / "runtime"
    other = tmp_path / "other.mjcf.xml"
    body.mkdir(parents=True)
    candidate.mkdir()

    for name in (
        "benchmark.yaml",
        "capabilities.yaml",
        "robot.eurdf.yaml",
        "safety.yaml",
        "semantic.yaml",
    ):
        (body / name).write_text("{}\n", encoding="ascii")
    (body / "robot.mjcf.xml").write_text("<mujoco model='formal'/>", encoding="ascii")
    (candidate / "robot.urdf").write_text(
        _parallel_gripper_urdf(), encoding="ascii"
    )
    (candidate / "robot.mjcf.xml").write_text(
        "<mujoco model='candidate'/>", encoding="ascii"
    )
    other.write_text("<mujoco model='other'/>", encoding="ascii")
    prepare(project, candidate, output)

    result = verify(
        candidate,
        output / "runtime.yaml",
        other,
        candidate / "robot.urdf",
    )

    assert result["status"] == "FAIL"
    assert result["checks"]["candidate_mjcf_is_active"] is False


def test_candidate_runtime_rejects_old_pinch_finger_joint_types(tmp_path: Path) -> None:
    project = tmp_path / "project"
    body = project / "models" / "openarm_dual_mujoco_01"
    candidate = tmp_path / "candidate"
    output = tmp_path / "runtime"
    body.mkdir(parents=True)
    candidate.mkdir()

    for name in (
        "benchmark.yaml",
        "capabilities.yaml",
        "robot.eurdf.yaml",
        "safety.yaml",
        "semantic.yaml",
    ):
        (body / name).write_text("{}\n", encoding="ascii")
    (body / "robot.mjcf.xml").write_text("<mujoco model='formal'/>", encoding="ascii")
    (candidate / "robot.urdf").write_text(
        _parallel_gripper_urdf(joint_type="revolute"), encoding="ascii"
    )
    (candidate / "robot.mjcf.xml").write_text(
        "<mujoco model='candidate'/>", encoding="ascii"
    )

    with pytest.raises(ValueError, match="expected prismatic"):
        prepare(project, candidate, output)


def test_moveit_executor_requires_complete_planning_execution_evidence(tmp_path: Path) -> None:
    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: _worker_payload(phases=[1, 2, 3, 4]),
    )

    assert result.final_state is ActionState.COMPLETED
    assert result.simulation_result["backend"] == "mujoco_moveit"
    assert result.simulation_result["planning_backend"] == "moveit2_get_cartesian_path"
    assert result.simulation_result["execution_backend"] == "moveit2_execute_trajectory"
    assert result.simulation_result["controller_backend"] == "follow_joint_trajectory"
    assert result.simulation_result["physics_executed"] is True
    assert result.simulation_result["direct_mujoco_fallback"] is False
    assert result.verification_result["passed"] is True
    assert result.verification_result["moveit_planning_observed"] is True
    assert result.verification_result["trajectory_execution_observed"] is True
    assert result.verification_result["final_pose_verification_observed"] is True
    assert result.verification_result["pose_evidence_consistent"] is True
    assert result.normalized_arguments == _action().arguments
    expected_arguments = json.dumps(
        _action().arguments,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    assert result.arguments_sha256 == hashlib.sha256(expected_arguments).hexdigest()
    assert result.initial_pose["position"]["z"] == 0.498
    assert result.target_pose["position"]["z"] == 0.5
    assert result.final_pose["position"]["z"] == 0.5
    assert result.final_error_m == 0.0
    artifact = tmp_path / _action().action_id / "moveit_worker_result.json"
    assert artifact.is_file()
    assert result.simulation_result["worker_artifact_sha256"] == hashlib.sha256(
        artifact.read_bytes()
    ).hexdigest()


def test_gateway_receipt_preserves_normalized_motion_evidence(tmp_path: Path) -> None:
    gateway = ActionGateway()
    call_count = 0

    def execute(action: ActionEnvelope):
        nonlocal call_count
        call_count += 1
        return run_openarm_move_relative_via_moveit(
            action,
            artifact_root=tmp_path,
            worker_command=None,
            transport=lambda _request: _worker_payload(phases=[1, 2, 3, 4]),
        )

    gateway.register_executor(CAPABILITY_ID, ExecutionMode.SIMULATION, execute)

    receipt = gateway.submit(_action())
    payload = receipt.to_dict()

    assert payload["normalized_arguments"] == _action().arguments
    assert payload["arguments_sha256"]
    assert payload["initial_pose"]["position"]["z"] == 0.498
    assert payload["target_pose"]["position"]["z"] == 0.5
    assert payload["final_pose"]["position"]["z"] == 0.5
    assert payload["final_error_m"] == 0.0
    receipt_path = tmp_path / _action().action_id / "receipt.json"
    assert receipt_path.is_file()

    initial = payload["initial_pose"]["position"]
    target = payload["target_pose"]["position"]
    final = payload["final_pose"]["position"]
    translation = payload["normalized_arguments"]["translation"]
    assert target == {
        axis: initial[axis] + translation[axis] for axis in ("x", "y", "z")
    }
    assert math.dist(
        [target[axis] for axis in ("x", "y", "z")],
        [final[axis] for axis in ("x", "y", "z")],
    ) == payload["final_error_m"]

    persisted = receipt_path.read_bytes()
    duplicate = gateway.submit(_action())
    assert duplicate is receipt
    assert receipt_path.read_bytes() == persisted
    assert call_count == 1


def test_arguments_digest_changes_when_normalized_arguments_change(tmp_path: Path) -> None:
    first = run_openarm_move_relative_via_moveit(
        _action("action_moveit_test_001"),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: _worker_payload(phases=[1, 2, 3, 4]),
    )
    second_action = _action("action_moveit_test_002")
    second_action.arguments["velocity_scale"] = 0.06
    second_payload = _worker_payload(phases=[1, 2, 3, 4])
    second_payload["request_id"] = second_action.action_id
    second = run_openarm_move_relative_via_moveit(
        second_action,
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: second_payload,
    )

    assert first.arguments_sha256 != second.arguments_sha256


def test_success_claim_without_moveit_planning_feedback_is_rejected(tmp_path: Path) -> None:
    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: _worker_payload(phases=[1, 3, 4]),
    )

    assert result.final_state is ActionState.FAILED
    assert result.verification_result["passed"] is False
    assert result.errors[0]["code"] == "MOVEIT_EVIDENCE_INCOMPLETE"


def test_success_claim_without_final_pose_is_rejected(tmp_path: Path) -> None:
    payload = _worker_payload(phases=[1, 2, 3, 4])
    payload["final_pose"] = None

    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: payload,
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "MOVEIT_EVIDENCE_INCOMPLETE"
    assert result.verification_result["final_pose_reported"] is False


def test_success_claim_with_inconsistent_target_pose_is_rejected(tmp_path: Path) -> None:
    payload = _worker_payload(phases=[1, 2, 3, 4])
    payload["target_pose"]["position"]["z"] = 0.51

    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=lambda _request: payload,
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "MOVEIT_EVIDENCE_INCOMPLETE"
    assert result.verification_result["pose_evidence_consistent"] is False


def test_missing_worker_fails_without_direct_mujoco_fallback(tmp_path: Path) -> None:
    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "MOVEIT_TRANSPORT_UNAVAILABLE"
    assert result.simulation_result["backend"] == "mujoco_moveit"
    assert result.simulation_result["physics_executed"] is False
    assert result.simulation_result["direct_mujoco_fallback"] is False


def test_runtime_registration_has_no_direct_mujoco_fallback(tmp_path: Path) -> None:
    adapter = SandboxRuntimeAdapter(
        {
            "engine": "fixture",
            "world_id": "empty",
            "robot_id": ROBOT_ID,
            "artifact_root": str(tmp_path),
            "openarm_moveit_required": True,
            "openarm_moveit_worker": None,
        },
        event_bus=EventBus(),
    )

    result = adapter.execute_action(_action())

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "MOVEIT_TRANSPORT_UNAVAILABLE"
    assert result.simulation_result["direct_mujoco_fallback"] is False


def test_worker_request_uses_normalized_bounded_arguments(tmp_path: Path) -> None:
    captured: dict[str, Any] = {}

    def transport(request: dict[str, Any]) -> dict[str, Any]:
        captured.update(request)
        return _worker_payload(phases=[1, 2, 3, 4])

    run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )

    assert captured["action_name"] == "/rosclaw/arm_motion"
    assert captured["arm"] == "right"
    assert captured["reference_frame"] == "world"
    assert captured["translation"] == {"x": 0.0, "y": 0.0, "z": 0.002}
    assert captured["avoid_collisions"] is True


def test_invalid_action_id_is_blocked_before_worker_dispatch(tmp_path: Path) -> None:
    called = False

    def transport(_request: dict[str, Any]) -> dict[str, Any]:
        nonlocal called
        called = True
        return _worker_payload(phases=[1, 2, 3, 4])

    result = run_openarm_move_relative_via_moveit(
        _action("action_../../escape"),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "OPENARM_ARTIFACT_PATH_INVALID"
    assert called is False


def test_existing_action_artifact_is_never_overwritten(tmp_path: Path) -> None:
    existing = tmp_path / _action().action_id
    existing.mkdir()
    marker = existing / "moveit_worker_result.json"
    marker.write_text("historical", encoding="utf-8")
    called = False

    def transport(_request: dict[str, Any]) -> dict[str, Any]:
        nonlocal called
        called = True
        return _worker_payload(phases=[1, 2, 3, 4])

    result = run_openarm_move_relative_via_moveit(
        _action(),
        artifact_root=tmp_path,
        worker_command=None,
        transport=transport,
    )

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "OPENARM_ACTION_ID_ALREADY_EXISTS"
    assert marker.read_text(encoding="utf-8") == "historical"
    assert called is False
