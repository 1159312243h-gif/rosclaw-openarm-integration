#!/usr/bin/env python3
"""Execute the deterministic OpenArm pick demo as one fail-closed task."""

from __future__ import annotations

import math
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

import rosclaw_moveit_action_worker as moveit_worker

SCHEMA_VERSION = "openarm.pick_demo.worker.v1"
OBJECT_TOPIC = "/openarm/pick_demo/object_pose"
EXPECTED_OBJECT_POSITION = (0.11697, -0.15350, 0.52183)
MAX_INITIAL_OFFSET_M = 0.010
SCENE_SAMPLE_DURATION_SEC = 0.25
MAX_SCENE_DRIFT_M = 0.001
MIN_SCENE_SAMPLES = 3
OPEN_GRIPPER_POSITION_M = 0.000
CLOSE_GRIPPER_POSITION_M = 0.006


def _response(request_id: str, **values: Any) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id,
        "accepted": False,
        "timed_out": False,
        "status": GoalStatus.STATUS_UNKNOWN,
        "success": False,
        "message": "",
        "stages": [],
        "initial_object_z_m": None,
        "final_object_z_m": None,
        "object_rise_m": None,
        **values,
    }


def _position(pose: dict[str, Any]) -> tuple[float, float, float]:
    values = pose["position"]
    return (float(values["x"]), float(values["y"]), float(values["z"]))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b, strict=True)))


def _observe_object_window(
    timeout_sec: float,
    *,
    sample_duration_sec: float,
    minimum_samples: int,
) -> dict[str, Any]:
    context = Context()
    node = None
    executor = None
    first: PoseStamped | None = None
    latest: PoseStamped | None = None
    first_received_at: float | None = None
    sample_count = 0
    max_drift_m = 0.0
    rclpy.init(context=context)
    try:
        node = rclpy.create_node("openarm_pick_demo_object_observer", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def callback(message: PoseStamped) -> None:
            nonlocal first, latest, first_received_at, sample_count, max_drift_m
            if first is None:
                first = message
                first_received_at = time.monotonic()
            latest = message
            sample_count += 1
            if first is not None:
                first_position = (
                    float(first.pose.position.x),
                    float(first.pose.position.y),
                    float(first.pose.position.z),
                )
                latest_position = (
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    float(message.pose.position.z),
                )
                max_drift_m = max(max_drift_m, _distance(first_position, latest_position))

        subscription = node.create_subscription(PoseStamped, OBJECT_TOPIC, callback, 10)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            now = time.monotonic()
            if (
                latest is not None
                and first_received_at is not None
                and sample_count >= minimum_samples
                and now - first_received_at >= sample_duration_sec
            ):
                break
        if latest is None:
            raise TimeoutError(
                f"dedicated pick scene is not active; no object pose on {OBJECT_TOPIC}"
            )
        if sample_count < minimum_samples:
            raise TimeoutError(
                f"object pose stream is incomplete: got {sample_count} samples, "
                f"need {minimum_samples}"
            )
        return {
            "first_pose": moveit_worker._pose_to_dict(first),
            "latest_pose": moveit_worker._pose_to_dict(latest),
            "sample_count": sample_count,
            "max_drift_m": max_drift_m,
        }
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def _observe_object(timeout_sec: float) -> dict[str, Any]:
    observation = _observe_object_window(
        timeout_sec,
        sample_duration_sec=0.0,
        minimum_samples=1,
    )
    return observation["latest_pose"]


def _check_scene_ready(timeout_sec: float) -> tuple[bool, str, dict[str, Any]]:
    observation = _observe_object_window(
        timeout_sec,
        sample_duration_sec=SCENE_SAMPLE_DURATION_SEC,
        minimum_samples=MIN_SCENE_SAMPLES,
    )
    pose = observation["latest_pose"]
    position = _position(pose)
    if pose.get("frame_id") != "world":
        return False, "object pose frame must be world", observation
    if not all(math.isfinite(value) for value in position):
        return False, "object pose contains a non-finite position", observation
    offset = _distance(position, EXPECTED_OBJECT_POSITION)
    observation["expected_position_m"] = list(EXPECTED_OBJECT_POSITION)
    observation["initial_offset_m"] = offset
    observation["maximum_initial_offset_m"] = MAX_INITIAL_OFFSET_M
    observation["maximum_scene_drift_m"] = MAX_SCENE_DRIFT_M
    if offset > MAX_INITIAL_OFFSET_M:
        return (
            False,
            f"object is outside the pick start region: offset={offset:.6f} m",
            observation,
        )
    if float(observation["max_drift_m"]) > MAX_SCENE_DRIFT_M:
        return (
            False,
            "object is not stable: "
            f"drift={float(observation['max_drift_m']):.6f} m",
            observation,
        )
    return True, "object pose is in range and stable", observation


def _stage(name: str, result: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": name,
        "success": result.get("success") is True,
        "accepted": result.get("accepted") is True,
        "status": result.get("status"),
        "message": result.get("message"),
        "error_code": result.get("error_code"),
        "goal_id": result.get("goal_id"),
        "feedback": result.get("feedback", []),
    }


def _arm_request(base: dict[str, Any], suffix: str, z: float) -> dict[str, Any]:
    return {
        "schema_version": moveit_worker.ARM_SCHEMA_VERSION,
        "request_id": f"{base['request_id']}_{suffix}",
        "action_name": "/rosclaw/arm_motion",
        "arm": "right",
        "reference_frame": "world",
        "translation": {"x": 0.0, "y": 0.0, "z": z},
        "velocity_scale": 0.05,
        "acceleration_scale": 0.05,
        "avoid_collisions": False,
        "server_timeout_sec": base["server_timeout_sec"],
        "result_timeout_sec": base["result_timeout_sec"],
        "cancel_timeout_sec": base["cancel_timeout_sec"],
    }


def _gripper_request(base: dict[str, Any], suffix: str, target: float) -> dict[str, Any]:
    return {
        "schema_version": moveit_worker.GRIPPER_SCHEMA_VERSION,
        "request_id": f"{base['request_id']}_{suffix}",
        "action_name": "/move_action",
        "side": "right",
        "target_m": target,
        "duration_s": 1.0,
        "hold_s": 0.25,
        "server_timeout_sec": base["server_timeout_sec"],
        "result_timeout_sec": base["result_timeout_sec"],
        "cancel_timeout_sec": base["cancel_timeout_sec"],
    }


def execute_pick(request: dict[str, Any]) -> dict[str, Any]:
    request_id = str(request.get("request_id", ""))
    if request.get("side") != "right" or request.get("object_id") != "pick_demo_block":
        raise ValueError("unsupported deterministic pick request")
    lift_m = float(request["lift_m"])
    base = {
        "request_id": request_id,
        "server_timeout_sec": float(request.get("server_timeout_sec", 5.0)),
        "result_timeout_sec": float(request.get("result_timeout_sec", 60.0)),
        "cancel_timeout_sec": float(request.get("cancel_timeout_sec", 3.0)),
    }
    stages: list[dict[str, Any]] = []
    try:
        scene_ready, scene_message, scene_observation = _check_scene_ready(
            base["server_timeout_sec"]
        )
    except TimeoutError as exc:
        return _response(request_id, message=str(exc), stages=stages)
    initial_pose = scene_observation["latest_pose"]
    initial_z = float(initial_pose["position"]["z"])
    stages.append(
        {
            "name": "scene_ready",
            "success": scene_ready,
            "message": scene_message,
            "object_pose": initial_pose,
            "sample_count": scene_observation["sample_count"],
            "max_drift_m": scene_observation["max_drift_m"],
            "initial_offset_m": scene_observation.get("initial_offset_m"),
        }
    )
    if not scene_ready:
        return _response(
            request_id,
            message=f"scene readiness failed: {scene_message}",
            stages=stages,
            initial_object_z_m=initial_z,
        )

    actions = (
        (
            "gripper_open",
            lambda: moveit_worker.execute_gripper(
                _gripper_request(base, "open", OPEN_GRIPPER_POSITION_M)
            ),
        ),
        ("approach", lambda: moveit_worker.execute(_arm_request(base, "approach", 0.005))),
        (
            "gripper_close",
            lambda: moveit_worker.execute_gripper(
                _gripper_request(base, "close", CLOSE_GRIPPER_POSITION_M)
            ),
        ),
        ("lift", lambda: moveit_worker.execute(_arm_request(base, "lift", lift_m))),
    )
    accepted = False
    for name, operation in actions:
        result = operation()
        accepted = accepted or result.get("accepted") is True
        stages.append(_stage(name, result))
        if result.get("success") is not True:
            return _response(
                request_id,
                accepted=accepted,
                timed_out=result.get("timed_out") is True,
                status=result.get("status", GoalStatus.STATUS_UNKNOWN),
                message=f"stage {name} failed: {result.get('message', '')}",
                stages=stages,
                initial_object_z_m=initial_z,
            )

    try:
        final_pose = _observe_object(base["server_timeout_sec"])
    except TimeoutError as exc:
        return _response(
            request_id,
            accepted=True,
            message=str(exc),
            stages=stages,
            initial_object_z_m=initial_z,
        )
    final_z = float(final_pose["position"]["z"])
    rise = final_z - initial_z
    verified = rise >= 0.008
    stages.append(
        {
            "name": "verify",
            "success": verified,
            "object_pose": final_pose,
            "object_rise_m": rise,
            "minimum_object_rise_m": 0.008,
        }
    )
    return _response(
        request_id,
        accepted=True,
        status=GoalStatus.STATUS_SUCCEEDED,
        success=verified,
        message="pick demo verified" if verified else "object did not rise with the gripper",
        stages=stages,
        initial_object_z_m=initial_z,
        final_object_z_m=final_z,
        object_rise_m=rise,
        scene_conditions={
            "gravity_mps2": [0.0, 0.0, -9.81],
            "physical_support": "pick_demo_support",
            "attachment_assistance": False,
        },
    )


__all__ = ["SCHEMA_VERSION", "execute_pick"]
