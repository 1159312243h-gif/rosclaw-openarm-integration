#!/usr/bin/env python3
"""Execute one typed ArmMotion request and return one JSON result on stdout."""

from __future__ import annotations

import json
import sys
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rosclaw_interfaces.action import ArmMotion
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

ARM_SCHEMA_VERSION = "openarm.moveit.worker.v1"
GRIPPER_SCHEMA_VERSION = "openarm.gripper.moveit.worker.v1"
PICK_DEMO_SCHEMA_VERSION = "openarm.pick_demo.worker.v1"
SCHEMA_VERSION = ARM_SCHEMA_VERSION
ARM_JOINTS = tuple(
    f"openarm_{side}_joint{number}"
    for side in ("left", "right")
    for number in range(1, 8)
)
FINGER_JOINTS = tuple(
    f"openarm_{side}_finger_joint{number}"
    for side in ("left", "right")
    for number in (1, 2)
)


def _pose_to_dict(message: Any) -> dict[str, Any]:
    pose = message.pose
    return {
        "frame_id": str(message.header.frame_id),
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def _optional_pose_to_dict(message: Any) -> dict[str, Any] | None:
    if message is None or not str(message.header.frame_id).strip():
        return None
    return _pose_to_dict(message)


def _response(
    request_id: str,
    *,
    schema_version: str = ARM_SCHEMA_VERSION,
    **values: Any,
) -> dict[str, Any]:
    return {
        "schema_version": schema_version,
        "request_id": request_id,
        "server_available": False,
        "accepted": False,
        "timed_out": False,
        "status": GoalStatus.STATUS_UNKNOWN,
        "success": False,
        "error_code": 0,
        "message": "",
        "goal_id": None,
        "initial_pose": None,
        "target_pose": None,
        "final_pose": None,
        "final_error_m": None,
        "feedback": [],
        **values,
    }


def _read_request() -> dict[str, Any]:
    raw = sys.stdin.read(65537)
    if len(raw) > 65536:
        raise ValueError("worker request exceeds 64 KiB")
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise ValueError("worker request must be a JSON object")
    if payload.get("schema_version") not in {
        ARM_SCHEMA_VERSION,
        GRIPPER_SCHEMA_VERSION,
        PICK_DEMO_SCHEMA_VERSION,
    }:
        raise ValueError("unsupported worker request schema")
    return payload


def _build_goal(request: dict[str, Any]) -> ArmMotion.Goal:
    translation = request["translation"]
    goal = ArmMotion.Goal()
    goal.request_id = str(request["request_id"])
    goal.arm = str(request["arm"])
    goal.operation = ArmMotion.Goal.MOVE_RELATIVE
    goal.reference_frame = str(request["reference_frame"])
    goal.translation.x = float(translation["x"])
    goal.translation.y = float(translation["y"])
    goal.translation.z = float(translation["z"])
    goal.velocity_scale = float(request["velocity_scale"])
    goal.acceleration_scale = float(request["acceleration_scale"])
    goal.avoid_collisions = bool(request["avoid_collisions"])
    return goal


def _spin_until(executor: Any, predicate: Any, timeout: float, description: str) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        value = predicate()
        if value is not None and value is not False:
            return value
    raise TimeoutError(f"timed out waiting for {description}")


def _max_delta(before: dict[str, float], after: dict[str, float], names: tuple[str, ...]) -> float:
    return max(abs(after[name] - before[name]) for name in names)


def execute_gripper(request: dict[str, Any]) -> dict[str, Any]:
    request_id = str(request.get("request_id", ""))
    action_name = str(request.get("action_name", "/move_action"))
    side = str(request["side"])
    target = float(request["target_m"])
    duration = float(request["duration_s"])
    hold = float(request["hold_s"])
    server_timeout = float(request.get("server_timeout_sec", 5.0))
    result_timeout = float(request.get("result_timeout_sec", 30.0))
    cancel_timeout = float(request.get("cancel_timeout_sec", 3.0))
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported gripper side: {side}")
    if not 0.0 <= target <= 0.044:
        raise ValueError("target_m must be between 0.0 and 0.044 m")
    if not 0.25 <= duration <= 10.0:
        raise ValueError("duration_s must be between 0.25 and 10.0 s")
    if not 0.0 <= hold <= 10.0:
        raise ValueError("hold_s must be between 0.0 and 10.0 s")
    feedback = [{"phase": 1, "progress": 0.05, "message": "Validating gripper state"}]
    positions: dict[str, float] = {}
    context = Context()
    node = None
    client = None
    executor = None
    rclpy.init(context=context)
    try:
        node = rclpy.create_node(
            f"openarm_gripper_worker_{request_id[-12:].replace('-', '_')}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def on_joint_state(message: JointState) -> None:
            positions.update(
                (name, float(position))
                for name, position in zip(message.name, message.position, strict=False)
            )

        _subscription = node.create_subscription(
            JointState,
            "/joint_states",
            on_joint_state,
            qos_profile_sensor_data,
        )
        client = ActionClient(node, MoveGroup, action_name)
        if not client.wait_for_server(timeout_sec=server_timeout):
            return _response(
                request_id,
                schema_version=GRIPPER_SCHEMA_VERSION,
                message=f"MoveGroup action server is unavailable: {action_name}",
                feedback=feedback,
            )
        required = set(ARM_JOINTS + FINGER_JOINTS)
        _spin_until(
            executor,
            lambda: True if required.issubset(positions) else None,
            server_timeout,
            "complete joint state",
        )
        baseline = dict(positions)
        driver = f"openarm_{side}_finger_joint1"
        follower = f"openarm_{side}_finger_joint2"
        other_side = "left" if side == "right" else "right"
        stationary = ARM_JOINTS + (
            f"openarm_{other_side}_finger_joint1",
            f"openarm_{other_side}_finger_joint2",
        )
        requested_speed_mps = abs(target - baseline[driver]) / duration
        velocity_scale = min(max(requested_speed_mps / 0.05, 0.01), 1.0)
        constraint = JointConstraint()
        constraint.joint_name = driver
        constraint.position = target
        constraint.tolerance_above = 0.0005
        constraint.tolerance_below = 0.0005
        constraint.weight = 1.0
        goal = MoveGroup.Goal()
        goal.request.group_name = f"{side}_gripper"
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scale
        goal.request.max_acceleration_scaling_factor = velocity_scale
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"
        goal.request.goal_constraints = [
            Constraints(
                name=f"{side}_gripper_target",
                joint_constraints=[constraint],
            )
        ]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        feedback.append({"phase": 2, "progress": 0.20, "message": "Planning gripper motion"})
        send_future = client.send_goal_async(goal)
        goal_handle = _spin_until(
            executor,
            lambda: send_future.result() if send_future.done() else None,
            server_timeout,
            "MoveGroup goal acceptance",
        )
        if not goal_handle.accepted:
            return _response(
                request_id,
                schema_version=GRIPPER_SCHEMA_VERSION,
                server_available=True,
                message="MoveGroup gripper goal was rejected",
                feedback=feedback,
            )
        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + result_timeout
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future, timeout_sec=cancel_timeout)
            return _response(
                request_id,
                schema_version=GRIPPER_SCHEMA_VERSION,
                server_available=True,
                accepted=True,
                timed_out=True,
                message="MoveGroup gripper execution timed out; cancel requested",
                goal_id=goal_id,
                feedback=feedback,
            )
        wrapped = result_future.result()
        moveit_code = int(wrapped.result.error_code.val) if wrapped is not None else 0
        action_success = bool(
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and moveit_code == MoveItErrorCodes.SUCCESS
        )
        if not action_success:
            return _response(
                request_id,
                schema_version=GRIPPER_SCHEMA_VERSION,
                server_available=True,
                accepted=True,
                status=int(wrapped.status) if wrapped is not None else GoalStatus.STATUS_UNKNOWN,
                error_code=moveit_code,
                message="MoveGroup gripper planning or execution failed",
                goal_id=goal_id,
                feedback=feedback,
            )
        feedback.append(
            {
                "phase": 3,
                "progress": 0.75,
                "message": "MoveIt gripper trajectory executed",
            }
        )
        feedback.append({"phase": 4, "progress": 0.90, "message": "Verifying gripper joint state"})
        driver_samples: list[float] = []
        follower_samples: list[float] = []
        deadline = time.monotonic() + hold
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            driver_samples.append(positions[driver])
            follower_samples.append(positions[follower])
        if not driver_samples:
            driver_samples.append(positions[driver])
            follower_samples.append(positions[follower])
        final = dict(positions)
        target_error = abs(final[driver] - target)
        mimic_error = abs(final[follower] - final[driver])
        driver_drift = max(driver_samples) - min(driver_samples)
        follower_drift = max(follower_samples) - min(follower_samples)
        stationary_delta = _max_delta(baseline, final, stationary)
        verified = bool(
            target_error <= 0.0005
            and mimic_error <= 0.0005
            and driver_drift <= 0.0002
            and follower_drift <= 0.0002
            and stationary_delta <= 0.0002
        )
        return _response(
            request_id,
            schema_version=GRIPPER_SCHEMA_VERSION,
            server_available=True,
            accepted=True,
            status=GoalStatus.STATUS_SUCCEEDED,
            success=verified,
            error_code=0 if verified else -4,
            message="Gripper motion verified" if verified else "Gripper verification failed",
            goal_id=goal_id,
            feedback=feedback,
            final_driver_joint_m=final[driver],
            final_follower_joint_m=final[follower],
            target_error_m=target_error,
            mimic_error_m=mimic_error,
            driver_hold_drift_m=driver_drift,
            follower_hold_drift_m=follower_drift,
            maximum_stationary_joint_delta=stationary_delta,
            requested_duration_s=duration,
            velocity_scale=velocity_scale,
        )
    finally:
        if client is not None:
            client.destroy()
        if executor is not None and node is not None:
            executor.remove_node(node)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def execute(request: dict[str, Any]) -> dict[str, Any]:
    if request.get("schema_version") == PICK_DEMO_SCHEMA_VERSION:
        from rosclaw_pick_demo_worker import execute_pick

        return execute_pick(request)
    if request.get("schema_version") == GRIPPER_SCHEMA_VERSION:
        return execute_gripper(request)
    request_id = str(request.get("request_id", ""))
    action_name = str(request.get("action_name", "/rosclaw/arm_motion"))
    server_timeout = float(request.get("server_timeout_sec", 5.0))
    result_timeout = float(request.get("result_timeout_sec", 30.0))
    cancel_timeout = float(request.get("cancel_timeout_sec", 3.0))
    feedback_events: list[dict[str, Any]] = []
    context = Context()
    node = None
    client = None
    executor = None
    rclpy.init(context=context)
    try:
        node = rclpy.create_node(
            f"openarm_moveit_worker_{request_id[-12:].replace('-', '_')}",
            context=context,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, ArmMotion, action_name)
        if not client.wait_for_server(timeout_sec=server_timeout):
            return _response(
                request_id,
                message=f"ArmMotion action server is unavailable: {action_name}",
            )

        def feedback_callback(message: Any) -> None:
            feedback = message.feedback
            feedback_events.append(
                {
                    "phase": int(feedback.phase),
                    "progress": float(feedback.progress),
                    "message": str(feedback.message),
                }
            )

        send_future = client.send_goal_async(
            _build_goal(request),
            feedback_callback=feedback_callback,
        )
        executor.spin_until_future_complete(send_future, timeout_sec=server_timeout)
        if not send_future.done():
            return _response(
                request_id,
                server_available=True,
                timed_out=True,
                message="Timed out while sending ArmMotion goal",
                feedback=feedback_events,
            )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return _response(
                request_id,
                server_available=True,
                message="ArmMotion goal was rejected",
                feedback=feedback_events,
            )

        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + result_timeout
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future, timeout_sec=cancel_timeout)
            return _response(
                request_id,
                server_available=True,
                accepted=True,
                timed_out=True,
                message="ArmMotion execution timed out; cancel requested",
                goal_id=goal_id,
                feedback=feedback_events,
            )

        wrapped = result_future.result()
        if wrapped is None:
            return _response(
                request_id,
                server_available=True,
                accepted=True,
                message="ArmMotion returned no result",
                goal_id=goal_id,
                feedback=feedback_events,
            )
        result = wrapped.result
        return _response(
            request_id,
            server_available=True,
            accepted=True,
            status=int(wrapped.status),
            success=bool(result.success),
            error_code=int(result.error_code),
            message=str(result.message),
            goal_id=goal_id,
            initial_pose=_optional_pose_to_dict(result.initial_pose),
            target_pose=_optional_pose_to_dict(result.target_pose),
            final_pose=_optional_pose_to_dict(result.final_pose),
            final_error_m=float(result.final_error_m),
            feedback=feedback_events,
        )
    finally:
        if client is not None:
            client.destroy()
        if executor is not None and node is not None:
            executor.remove_node(node)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def main() -> None:
    try:
        request = _read_request()
        payload = execute(request)
    except Exception as exc:  # noqa: BLE001
        request_id = str(locals().get("request", {}).get("request_id", ""))
        schema_version = str(locals().get("request", {}).get("schema_version", ARM_SCHEMA_VERSION))
        payload = _response(
            request_id,
            schema_version=schema_version,
            message=f"MoveIt worker error: {exc}",
        )
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")


if __name__ == "__main__":
    main()
