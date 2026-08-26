#!/usr/bin/env python3
"""Send one gripper trajectory and verify that it reaches and holds."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


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


class GripperAcceptance(Node):
    def __init__(self, side: str) -> None:
        super().__init__("openarm_gripper_candidate_acceptance")
        self.side = side
        self.positions: dict[str, float] = {}
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{side}_gripper_controller/follow_joint_trajectory",
        )
        self.parameter_client = AsyncParameterClient(
            self, "/mujoco_moveit_bridge"
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.positions.update(
            (name, float(position))
            for name, position in zip(
                message.name, message.position, strict=False
            )
        )

    def spin_until(self, predicate, timeout: float, description: str) -> Any:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            value = predicate()
            if value:
                return value
        raise TimeoutError(f"timed out waiting for {description}")


def max_delta(before: dict[str, float], after: dict[str, float], names) -> float:
    return max(abs(after[name] - before[name]) for name in names)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--expected-model-path", type=Path, required=True)
    parser.add_argument("--target", type=float, default=0.022)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hold", type=float, default=5.0)
    parser.add_argument("--target-tolerance", type=float, default=0.0005)
    parser.add_argument("--mimic-tolerance", type=float, default=0.0005)
    parser.add_argument("--hold-tolerance", type=float, default=0.0002)
    parser.add_argument("--stationary-tolerance", type=float, default=0.0002)
    args = parser.parse_args()

    if not 0.0 <= args.target <= 0.044:
        parser.error("--target must be in the calibrated range 0.000-0.044 m")
    if args.duration <= 0.0 or args.hold <= 0.0:
        parser.error("--duration and --hold must be positive")

    rclpy.init()
    node = GripperAcceptance(args.side)
    try:
        if not node.parameter_client.wait_for_services(timeout_sec=10.0):
            raise RuntimeError("MuJoCo bridge parameter service is unavailable")
        parameter_future = node.parameter_client.get_parameters(
            ["model_path", "enable_gripper_controllers"]
        )
        parameter_response = node.spin_until(
            lambda: (
                parameter_future.result() if parameter_future.done() else None
            ),
            10.0,
            "MuJoCo bridge parameters",
        )
        active_model_path = Path(
            parameter_response.values[0].string_value
        ).expanduser().resolve()
        expected_model_path = args.expected_model_path.expanduser().resolve()
        gripper_controllers_enabled = bool(
            parameter_response.values[1].bool_value
        )
        if active_model_path != expected_model_path:
            raise RuntimeError(
                "refusing to send: active MuJoCo model does not match "
                f"--expected-model-path ({active_model_path} != "
                f"{expected_model_path})"
            )
        if not gripper_controllers_enabled:
            raise RuntimeError(
                "refusing to send: enable_gripper_controllers is false"
            )

        if not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                f"{args.side} gripper FollowJointTrajectory action is unavailable"
            )

        required = set(ARM_JOINTS + FINGER_JOINTS)
        node.spin_until(
            lambda: required.issubset(node.positions),
            10.0,
            "a complete /joint_states sample",
        )
        baseline = dict(node.positions)

        driver = f"openarm_{args.side}_finger_joint1"
        follower = f"openarm_{args.side}_finger_joint2"
        other_side = "left" if args.side == "right" else "right"
        stationary_joints = ARM_JOINTS + (
            f"openarm_{other_side}_finger_joint1",
            f"openarm_{other_side}_finger_joint2",
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [driver]
        point = JointTrajectoryPoint()
        point.positions = [args.target]
        whole_seconds = int(args.duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (args.duration - whole_seconds) * 1_000_000_000
        )
        goal.trajectory.points = [point]

        send_future = node.client.send_goal_async(goal)
        goal_handle = node.spin_until(
            lambda: send_future.result() if send_future.done() else None,
            10.0,
            "goal acceptance",
        )
        if not goal_handle.accepted:
            raise RuntimeError("gripper trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        wrapped_result = node.spin_until(
            lambda: result_future.result() if result_future.done() else None,
            args.duration + 10.0,
            "trajectory result",
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                "gripper trajectory did not succeed: "
                f"status={wrapped_result.status}, "
                f"error_code={wrapped_result.result.error_code}, "
                f"error_string={wrapped_result.result.error_string!r}"
            )

        hold_driver_samples = []
        hold_follower_samples = []
        hold_deadline = time.monotonic() + args.hold
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            hold_driver_samples.append(node.positions[driver])
            hold_follower_samples.append(node.positions[follower])

        final = dict(node.positions)
        target_error = abs(final[driver] - args.target)
        mimic_error = abs(final[follower] - final[driver])
        hold_drift = max(hold_driver_samples) - min(hold_driver_samples)
        follower_hold_drift = max(hold_follower_samples) - min(
            hold_follower_samples
        )
        stationary_delta = max_delta(baseline, final, stationary_joints)

        checks = {
            "target_reached": target_error <= args.target_tolerance,
            "mimic_consistent": mimic_error <= args.mimic_tolerance,
            "driver_held": hold_drift <= args.hold_tolerance,
            "follower_held": follower_hold_drift <= args.hold_tolerance,
            "other_joints_stationary": (
                stationary_delta <= args.stationary_tolerance
            ),
        }
        report = {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "side": args.side,
            "target_driver_joint_m": args.target,
            "final_driver_joint_m": final[driver],
            "final_follower_joint_m": final[follower],
            "target_error_m": target_error,
            "mimic_error_m": mimic_error,
            "driver_hold_drift_m": hold_drift,
            "follower_hold_drift_m": follower_hold_drift,
            "maximum_stationary_joint_delta": stationary_delta,
            "hold_duration_s": args.hold,
            "checks": checks,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        print(
            "OPENARM GRIPPER ROS ACTION CANDIDATE: "
            + ("PASS" if all(checks.values()) else "FAIL")
        )
        return 0 if all(checks.values()) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
