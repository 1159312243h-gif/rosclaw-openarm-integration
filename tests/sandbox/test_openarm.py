"""Official sandbox contract tests for the OpenArm extension."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from rosclaw.core.event_bus import EventBus
from rosclaw.kernel import ActionEnvelope, ActionState, ExecutionMode
from rosclaw.sandbox.openarm import (
    CAPABILITY_ID,
    MAX_POSITION_TOLERANCE_M,
    ROBOT_ID,
    _initialize_observation,
    run_openarm_move_relative,
    validate_openarm_move_relative,
)
from rosclaw.sandbox.runtime_adapter import SandboxRuntimeAdapter


def _action(**arguments: object) -> ActionEnvelope:
    payload = {
        "arm": "right",
        "reference_frame": "world",
        "translation": {"x": 0.0, "y": 0.0, "z": 0.002},
        "velocity_scale": 0.05,
        "acceleration_scale": 0.05,
        "avoid_collisions": False,
    }
    payload.update(arguments)
    return ActionEnvelope(
        actor_id="test",
        agent_framework="pytest",
        session_id="session-test",
        body_id=ROBOT_ID,
        capability_id=CAPABILITY_ID,
        arguments=payload,
        execution_mode=ExecutionMode.SIMULATION,
    )


def test_openarm_contract_normalizes_bounded_motion() -> None:
    request = validate_openarm_move_relative(_action())

    assert request["arm"] == "right"
    assert request["translation_norm_m"] == pytest.approx(0.002)
    assert request["reference_frame"] == "world"
    assert request["translation_norm_m"] > MAX_POSITION_TOLERANCE_M


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"arm": "center"}, "unsupported arm"),
        ({"reference_frame": "tool"}, "unsupported reference_frame"),
        ({"translation": {"x": 0.02, "y": 0.02, "z": 0.0}}, "norm exceeds"),
        ({"translation": {"x": 0.0, "y": 0.0, "z": 0.0}}, "non-zero"),
        ({"unexpected": True}, "unknown arguments"),
    ],
)
def test_openarm_contract_fails_closed(override: dict[str, object], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        validate_openarm_move_relative(_action(**override))


def test_openarm_executor_fails_when_mujoco_backend_is_missing(tmp_path: Path) -> None:
    sandbox = MagicMock()
    sandbox.has_physics = False
    sandbox.load_error = "model unavailable"

    result = run_openarm_move_relative(sandbox, _action(), artifact_root=tmp_path)

    assert result.final_state is ActionState.FAILED
    assert result.errors[0]["code"] == "PHYSICS_UNAVAILABLE"
    assert result.dispatch_result["accepted"] is False


def test_openarm_initial_observation_is_recorded_after_mj_forward() -> None:
    model = SimpleNamespace()
    data = SimpleNamespace(
        xpos=[np.zeros(3), np.zeros(3)],
        ncon=0,
        contact=[],
    )
    mujoco = MagicMock()

    def synchronize(_model: object, current_data: object) -> None:
        current_data.xpos[0] = np.asarray([0.1, 0.2, 0.3])
        current_data.xpos[1] = np.asarray([0.3, 0.4, 0.5])

    mujoco.mj_forward.side_effect = synchronize

    position, contacts = _initialize_observation(model, data, mujoco, [0, 1])

    mujoco.mj_forward.assert_called_once_with(model, data)
    assert position == pytest.approx([0.2, 0.3, 0.4])
    assert contacts == []


def test_openarm_runtime_adapter_registers_only_official_capability() -> None:
    adapter = SandboxRuntimeAdapter(
        {
            "engine": "fixture",
            "world_id": "empty",
            "robot_id": ROBOT_ID,
        },
        event_bus=EventBus(),
    )

    assert adapter.supported_capabilities == (CAPABILITY_ID, "sandbox.reach")


def test_other_bodies_do_not_receive_openarm_executor() -> None:
    adapter = SandboxRuntimeAdapter(
        {"engine": "fixture", "world_id": "empty", "robot_id": "ur5e"},
        event_bus=EventBus(),
    )

    assert adapter.supported_capabilities == ("sandbox.reach",)
