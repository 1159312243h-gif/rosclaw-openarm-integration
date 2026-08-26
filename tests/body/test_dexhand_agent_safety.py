"""Tests for dexterous-hand safety hardening and Agent query answers."""

from __future__ import annotations

from rosclaw.body.compiler import EffectiveBodyCompiler
from rosclaw.body.query import BodyQueryEngine
from rosclaw.body.safety import SafetyInvariantEngine
from rosclaw.body.schema import (
    BodyYaml,
    CalibrationYaml,
    EurdfProfile,
)


def _make_hand_eurdf(real_robot_allowed: bool = False) -> EurdfProfile:
    return EurdfProfile(
        profile_id="dexhands/test_hand/right",
        profile_version="1.0.0",
        vendor="Test",
        model="test_hand",
        display_name="Test Hand (Right)",
        capability_hints={
            "all": [
                "Open Hand",
                "Close Hand Slowly",
                "OK Gesture",
                "Countdown Gesture",
            ],
        },
        forbidden_capabilities=[
            {
                "id": "fast_full_close",
                "description": "Close all fingers/joints to maximum at high speed",
                "reason": "Risk of collision and motor overload",
                "severity": "critical",
                "enforcement": {
                    "policy_block": True,
                    "sandbox_block": True,
                    "real_robot_block": True,
                },
            },
            {
                "id": "forceful_grasp_without_current_limit",
                "description": "Forceful grasping without active current/torque limit",
                "reason": "Can damage the hand and grasped object",
                "severity": "critical",
                "enforcement": {
                    "policy_block": True,
                    "sandbox_block": True,
                    "real_robot_block": True,
                },
            },
        ],
        safety={
            "safety_level": "STRICT",
            "environment": {
                "real_robot_execution_allowed": real_robot_allowed,
                "sandbox_required": True,
            },
        },
        joints=[{"name": "j0"}],
        actuators=[{"name": "j0"}],
    )


def _make_hand_body(real_robot_allowed: bool = False) -> BodyYaml:
    return BodyYaml(
        body_instance={"id": "test-hand-001", "robot_model": "dexhands/test_hand/right"},
        model_ref={"eurdf_uri": "rosclaw://eurdf/dexhands/test_hand/right@1.0.0"},
        calibration={"status": "factory_default"},
        capabilities={"enabled": [], "disabled": [], "degraded": []},
        forbidden_capabilities=[
            {
                "id": "fast_full_close",
                "description": "Close all fingers/joints to maximum at high speed",
                "reason": "Risk of collision and motor overload",
                "severity": "critical",
                "enforcement": {
                    "policy_block": True,
                    "sandbox_block": True,
                    "real_robot_block": True,
                },
            },
            {
                "id": "forceful_grasp_without_current_limit",
                "description": "Forceful grasping without active current/torque limit",
                "reason": "Can damage the hand and grasped object",
                "severity": "critical",
                "enforcement": {
                    "policy_block": True,
                    "sandbox_block": True,
                    "real_robot_block": True,
                },
            },
        ],
        agent_policy={
            "physical_execution_requires_sandbox": True,
            "direct_real_robot_execution_allowed": real_robot_allowed,
            "human_approval_required_for_high_risk": True,
        },
    )


def test_compiler_degrades_gestures_when_uncalibrated() -> None:
    compiler = EffectiveBodyCompiler()
    effective = compiler.compile(
        eurdf=_make_hand_eurdf(),
        body=_make_hand_body(),
        calibration=CalibrationYaml(),
    )
    caps = effective.capabilities
    assert "OK Gesture" in caps["degraded"]
    assert "Countdown Gesture" in caps["degraded"]


def test_compiler_sim_only_when_real_robot_blocked() -> None:
    compiler = EffectiveBodyCompiler()
    effective = compiler.compile(
        eurdf=_make_hand_eurdf(real_robot_allowed=False),
        body=_make_hand_body(real_robot_allowed=False),
        calibration=CalibrationYaml(),
    )
    caps = effective.capabilities
    # Manipulation capabilities are degraded to simulation-only, not enabled.
    assert "Open Hand" in caps["degraded"]
    assert "Close Hand Slowly" in caps["degraded"]
    assert "fast_full_close" in caps["blocked"]
    assert "forceful_grasp_without_current_limit" in caps["blocked"]


def test_compiler_allows_real_hardware_when_cleared() -> None:
    compiler = EffectiveBodyCompiler()
    body = _make_hand_body(real_robot_allowed=True)
    body.calibration = {"status": "validated"}
    effective = compiler.compile(
        eurdf=_make_hand_eurdf(real_robot_allowed=True),
        body=body,
        calibration=CalibrationYaml(validation={"status": "validated"}),
    )
    caps = effective.capabilities
    assert "OK Gesture" in caps["enabled"]
    assert "Countdown Gesture" in caps["enabled"]
    assert "fast_full_close" in caps["blocked"]


def test_safety_engine_blocks_critical_forbidden_capabilities() -> None:
    engine = SafetyInvariantEngine()
    body = _make_hand_body()
    mods = engine.apply(body, [], CalibrationYaml())
    assert "fast_full_close" in mods["disabled"]
    assert "forceful_grasp_without_current_limit" in mods["disabled"]
    assert any("fast_full_close" in w for w in mods["warnings"])


def test_query_blocks_fast_full_close() -> None:
    effective = EffectiveBodyCompiler().compile(
        eurdf=_make_hand_eurdf(),
        body=_make_hand_body(),
        calibration=CalibrationYaml(),
    )
    engine = BodyQueryEngine(effective, _make_hand_body(), CalibrationYaml(), [])
    result = engine.answer("Can this hand close fast?")
    assert "forbidden" in result.answer.lower() or "blocked" in result.answer.lower()
    assert result.actionable_policy


def test_query_blocks_ok_gesture_on_real_hardware() -> None:
    effective = EffectiveBodyCompiler().compile(
        eurdf=_make_hand_eurdf(),
        body=_make_hand_body(),
        calibration=CalibrationYaml(),
    )
    engine = BodyQueryEngine(effective, _make_hand_body(), CalibrationYaml(), [])
    result = engine.answer("Can this hand do OK gesture on real hardware?")
    assert "blocked" in result.answer.lower() or "not allowed" in result.answer.lower()
    assert "real robot execution" in result.answer.lower()


def test_query_sandbox_first_for_countdown_gesture() -> None:
    effective = EffectiveBodyCompiler().compile(
        eurdf=_make_hand_eurdf(),
        body=_make_hand_body(),
        calibration=CalibrationYaml(),
    )
    engine = BodyQueryEngine(effective, _make_hand_body(), CalibrationYaml(), [])
    result = engine.answer("Can this hand perform countdown gesture 5-4-3-2-1?")
    assert "sandbox" in result.answer.lower()
    assert result.actionable_policy


def test_query_blocks_forceful_grasp() -> None:
    effective = EffectiveBodyCompiler().compile(
        eurdf=_make_hand_eurdf(),
        body=_make_hand_body(),
        calibration=CalibrationYaml(),
    )
    engine = BodyQueryEngine(effective, _make_hand_body(), CalibrationYaml(), [])
    result = engine.answer("Can I forcefully grasp an object?")
    assert "forbidden" in result.answer.lower() or "blocked" in result.answer.lower()
    assert "current" in result.answer.lower() or "torque" in result.answer.lower()


def test_query_allows_ok_gesture_when_cleared() -> None:
    body = _make_hand_body(real_robot_allowed=True)
    body.calibration = {"status": "validated"}
    effective = EffectiveBodyCompiler().compile(
        eurdf=_make_hand_eurdf(real_robot_allowed=True),
        body=body,
        calibration=CalibrationYaml(validation={"status": "validated"}),
    )
    engine = BodyQueryEngine(
        effective, body, CalibrationYaml(validation={"status": "validated"}), []
    )
    result = engine.answer("Can this hand do OK gesture on real hardware?")
    assert "yes" in result.answer.lower()
