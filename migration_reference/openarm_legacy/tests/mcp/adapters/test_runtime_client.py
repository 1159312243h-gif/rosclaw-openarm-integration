"""Unit tests for the RuntimeClient facade."""

from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock
from uuid import UUID

import pytest

from rosclaw.core.event_bus import EventBus
from rosclaw.mcp.adapters.runtime_client import RuntimeClient
from rosclaw.mcp.schemas.common import MCPError
from rosclaw.mcp.tools import _request_action, _sandbox_run


class _FakeSense:
    def __init__(self) -> None:
        self.is_stale = False
        self.state_age_ms = 42

    def get_latest_state(self) -> dict[str, Any]:
        return {"joint_positions": [0.1] * 6, "source": "hardware:test_feedback"}

    def get_body_sense(self) -> dict[str, Any]:
        return {"temperature": "normal"}

    def get_readiness(self) -> dict[str, Any]:
        return {"overall": "READY"}


class _FakeRuntime:
    """Minimal runtime double for RuntimeClient wiring tests."""

    def __init__(self) -> None:
        self.event_bus = EventBus()
        self.sense = _FakeSense()
        self.memory = MagicMock()
        self.memory.find_similar_experiences.return_value = []
        self.skill_manager = MagicMock()
        self.skill_manager.registry = None
        self.skill_manager.list_skills.return_value = []
        self.sandbox = MagicMock()
        self.sandbox.validate_trajectory.return_value = {
            "is_safe": True,
            "risk_score": 0.0,
            "reason": "ok",
            "violations": [],
            "replay_id": "replay-1",
        }
        self.sandbox.simulate_step.return_value = {"qpos": [0.2] * 6}
        self.sandbox.model_path = None
        self.submit_action = MagicMock()
        self.episode_recorder = MagicMock()
        self.episode_recorder.list_episodes.return_value = []

    def request_emergency_stop(self, reason: str, *, source: str) -> dict[str, Any]:
        return {
            "request_id": "stop-fake",
            "reason": reason,
            "source": source,
            "targets": ["fake_driver"],
            "request_dispatched": True,
            "driver_acknowledged": True,
            "physical_stop_observed": False,
            "stopped": False,
            "final_status": "ACKNOWLEDGED",
            "mode": "runtime",
        }


@pytest.fixture
def client_with_runtime() -> RuntimeClient:
    daemon = MagicMock()
    daemon.emergency_stop.return_value = {
        "request_id": "stop-fake",
        "reason": "integration test",
        "source": "mcp.emergency_stop",
        "targets": ["fake_driver"],
        "request_dispatched": True,
        "driver_acknowledged": True,
        "physical_stop_observed": False,
        "stopped": False,
        "final_status": "ACKNOWLEDGED",
        "mode": "runtime",
    }
    client = RuntimeClient(
        project_root=Path("/tmp/rosclaw-test"),
        robot_id="test_bot",
        runtime_profile={},
        daemon_client=daemon,
    )
    client._runtime = _FakeRuntime()
    client._adapter_cache = None
    return client


async def test_get_robot_state_returns_live(client_with_runtime: RuntimeClient) -> None:
    response = await client_with_runtime.get_robot_state()
    assert response["robot_id"] == "test_bot"
    assert response["mode"] == "live"
    assert response["body_state"]["joint_positions"] == [0.1] * 6
    assert response["age_ms"] == 42


async def test_get_robot_state_fixture_when_sense_missing() -> None:
    client = RuntimeClient(
        project_root=Path("/tmp/rosclaw-test"),
        robot_id="fixture_bot",
        runtime_profile={},
        fixture_mode=True,
    )
    response = await client.get_robot_state()
    assert response["mode"] == "fixture"
    assert response["robot_id"] == "fixture_bot"


async def test_list_skills_delegates_to_adapter(client_with_runtime: RuntimeClient) -> None:
    client_with_runtime._runtime.skill_manager.list_skills.return_value = [
        MagicMock(to_dict=lambda: {"name": "pick"}),
    ]
    response = await client_with_runtime.list_skills(skill_type="manipulation")
    assert response["count"] == 1
    assert response["skills"][0]["name"] == "pick"


async def test_query_memory_delegates_to_adapter(client_with_runtime: RuntimeClient) -> None:
    client_with_runtime._runtime.memory.find_similar_experiences.return_value = [
        {"instruction": "pick", "outcome": "success"},
    ]
    response = await client_with_runtime.query_memory("pick", limit=1)
    assert response["count"] == 1
    assert response["experiences"][0]["outcome"] == "success"


async def test_validate_trajectory_delegates_to_sandbox(client_with_runtime: RuntimeClient) -> None:
    response = await client_with_runtime.validate_trajectory([[0.0] * 6, [0.1] * 6])
    assert response["is_safe"] is True
    assert response["replay_id"] == "replay-1"


async def test_sandbox_run_delegates_to_sandbox(client_with_runtime: RuntimeClient) -> None:
    response = await client_with_runtime.sandbox_run([0.1] * 6)
    assert response["mode"] == "simulation"
    assert response["physics_state"]["qpos"] == [0.2] * 6


async def test_sandbox_run_submits_high_level_action_through_runtime(
    client_with_runtime: RuntimeClient,
) -> None:
    receipt = MagicMock()
    receipt.to_dict.return_value = {
        "action_id": "action_openarm_001",
        "execution_mode": "SIMULATION",
        "final_state": "COMPLETED",
        "evidence_level": "TASK_VERIFIED",
    }
    client_with_runtime._runtime.submit_action.return_value = receipt

    response = await client_with_runtime.sandbox_run(
        capability_id="openarm.arm.move_relative",
        arguments={
            "arm": "right",
            "reference_frame": "world",
            "translation": {"x": 0.0, "y": 0.0, "z": 0.002},
        },
    )

    submitted = client_with_runtime._runtime.submit_action.call_args.args[0]
    assert submitted.capability_id == "openarm.arm.move_relative"
    assert submitted.execution_mode.value == "SIMULATION"
    assert submitted.action_id.startswith("action_")
    assert UUID(submitted.action_id.removeprefix("action_"))
    assert response["receipt"]["final_state"] == "COMPLETED"
    assert response["usable_for_real_execution"] is False


def test_model_visible_motion_tools_do_not_accept_action_id() -> None:
    assert "action_id" not in inspect.signature(_sandbox_run).parameters
    assert "action_id" not in inspect.signature(_request_action).parameters


async def test_request_action_generates_action_id_at_mcp_service_boundary(
    client_with_runtime: RuntimeClient,
) -> None:
    captured: Any = None

    def request_action(action: Any) -> dict[str, Any]:
        nonlocal captured
        captured = action
        return {"action_id": action.action_id, "state": "QUEUED"}

    client_with_runtime._daemon_client.request_action.side_effect = request_action

    response = await client_with_runtime.request_action(
        capability_id="openarm.arm.move_relative",
        arguments={"arm": "right"},
        execution_mode="SHADOW",
        body_snapshot_hash="sha256:test",
        wait_timeout_sec=0.0,
    )

    assert captured is not None
    assert captured.action_id.startswith("action_")
    assert UUID(captured.action_id.removeprefix("action_"))
    assert response["action_id"] == captured.action_id


async def test_sandbox_run_rejects_mixed_legacy_and_capability_inputs(
    client_with_runtime: RuntimeClient,
) -> None:
    with pytest.raises(MCPError, match="cannot be combined"):
        await client_with_runtime.sandbox_run(
            [0.0] * 6,
            capability_id="openarm.arm.move_relative",
            arguments={},
        )


async def test_get_execution_receipt_reads_legacy_receipt_without_new_audit_fields(
    tmp_path: Path,
) -> None:
    action_id = "action_openarm_001"
    artifact_root = tmp_path / "artifacts"
    artifact_dir = artifact_root / action_id
    artifact_dir.mkdir(parents=True)
    receipt = {
        "action_id": action_id,
        "execution_mode": "SIMULATION",
        "trust_level": "SIMULATED",
    }
    payload = json.dumps(receipt).encode("utf-8")
    (artifact_dir / "receipt.json").write_bytes(payload)
    (artifact_dir / "receipt.sha256").write_text(
        f"{hashlib.sha256(payload).hexdigest()}  receipt.json\n",
        encoding="ascii",
    )
    client = RuntimeClient(
        project_root=tmp_path,
        robot_id="openarm_dual_mujoco_01",
        runtime_profile={"sandbox": {"artifact_root": str(artifact_root)}},
    )

    result = await client.get_execution_receipt(action_id)

    assert result["integrity_verified"] is True
    assert result["receipt"] == receipt


async def test_get_execution_receipt_blocks_tampered_action_receipt(
    tmp_path: Path,
) -> None:
    action_id = "action_openarm_001"
    artifact_root = tmp_path / "artifacts"
    artifact_dir = artifact_root / action_id
    artifact_dir.mkdir(parents=True)
    (artifact_dir / "receipt.json").write_text("{}", encoding="utf-8")
    (artifact_dir / "receipt.sha256").write_text(
        f"{'0' * 64}  receipt.json\n",
        encoding="ascii",
    )
    client = RuntimeClient(
        project_root=tmp_path,
        robot_id="openarm_dual_mujoco_01",
        runtime_profile={"sandbox": {"artifact_root": str(artifact_root)}},
    )

    with pytest.raises(MCPError, match="digest mismatch"):
        await client.get_execution_receipt(action_id)


async def test_practice_query_delegates_to_recorder(client_with_runtime: RuntimeClient) -> None:
    client_with_runtime._runtime.episode_recorder.list_episodes.return_value = [
        {"episode_id": "ep-1"},
    ]
    response = await client_with_runtime.practice_query(limit=5)
    assert response["count"] == 1
    assert response["episodes"][0]["episode_id"] == "ep-1"


async def test_emergency_stop_delegates_to_daemon(client_with_runtime: RuntimeClient) -> None:
    response = await client_with_runtime.emergency_stop("integration test")
    assert response["stopped"] is False
    assert response["mode"] == "runtime"
    assert response["final_status"] == "ACKNOWLEDGED"
    client_with_runtime._daemon_client.emergency_stop.assert_called_once_with(
        "integration test",
        source="mcp.emergency_stop",
    )


async def test_emergency_stop_degraded_without_runtime() -> None:
    client = RuntimeClient(
        project_root=Path("/tmp/rosclaw-test"),
        robot_id="test_bot",
        runtime_profile={},
        fixture_mode=True,
    )
    response = await client.emergency_stop("no runtime")
    assert response["stopped"] is False
    assert response["mode"] == "fixture"
    assert response["execution_mode"] == "FIXTURE"
    assert "physical E-stop" in response["note"]


async def test_live_mode_never_falls_back_to_fixture_on_runtime_failure() -> None:
    client = RuntimeClient(
        project_root=Path("/tmp/rosclaw-test"),
        robot_id="real_bot",
        runtime_profile={},
    )
    client._runtime_error = "model missing"

    with pytest.raises(MCPError) as error:
        await client.get_robot_state()

    assert error.value.code == "RUNTIME_UNAVAILABLE"
    assert error.value.details["trust_level"] == "UNAVAILABLE"


def test_openarm_runtime_rejects_profile_that_does_not_require_moveit(
    tmp_path: Path,
) -> None:
    client = RuntimeClient(
        project_root=tmp_path,
        robot_id="openarm_dual_mujoco_01",
        runtime_profile={"sandbox": {"moveit": {"required": False}}},
    )

    assert client._ensure_runtime() is None
    assert client._runtime_error is not None
    assert "sandbox.moveit.required: true" in client._runtime_error
