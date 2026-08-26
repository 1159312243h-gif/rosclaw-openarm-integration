#!/usr/bin/env python3
"""Plan and execute one candidate gripper target through MoveIt's MoveGroup."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


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


class MoveItGripperAcceptance(Node):
    def __init__(self) -> None:
        super().__init__("openarm_gripper_moveit_candidate_acceptance")
        self.positions: dict[str, float] = {}
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.move_group_parameters = AsyncParameterClient(self, "/move_group")
        self.bridge_parameters = AsyncParameterClient(
            self, "/mujoco_moveit_bridge"
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.positions.update(
            (name, float(position))
            for name, position in zip(
                message.name, message.position, strict=False
            )
        )

    def spin_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            value = predicate()
            if value:
                return value
        raise TimeoutError(f"timed out waiting for {description}")


def max_delta(before: dict[str, float], after: dict[str, float], names) -> float:
    return max(abs(after[name] - before[name]) for name in names)


def parameter_values(node, client, names, description):
    if not client.wait_for_services(timeout_sec=10.0):
        raise RuntimeError(f"{description} parameter service is unavailable")
    future = client.get_parameters(names)
    response = node.spin_until(
        lambda: future.result() if future.done() else None,
        10.0,
        f"{description} parameters",
    )
    return response.values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-model-path", type=Path, required=True)
    parser.add_argument("--expected-urdf-path", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--target", type=float, required=True)
    parser.add_argument("--hold", type=float, default=5.0)
    parser.add_argument("--target-tolerance", type=float, default=0.0005)
    parser.add_argument("--mimic-tolerance", type=float, default=0.0005)
    parser.add_argument("--hold-tolerance", type=float, default=0.0002)
    parser.add_argument("--stationary-tolerance", type=float, default=0.0002)
    args = parser.parse_args()

    if not 0.0 <= args.target <= 0.044:
        parser.error("--target must be in the calibrated range 0.000-0.044 m")

    rclpy.init()
    node = MoveItGripperAcceptance()
    try:
        bridge_values = parameter_values(
            node,
            node.bridge_parameters,
            ["model_path", "enable_gripper_controllers"],
            "MuJoCo bridge",
        )
        active_model = Path(bridge_values[0].string_value).resolve()
        expected_model = args.expected_model_path.expanduser().resolve()
        if active_model != expected_model or not bridge_values[1].bool_value:
            raise RuntimeError(
                "refusing to send: bridge is not running the expected "
                "gripper candidate"
            )

        move_group_values = parameter_values(
            node,
            node.move_group_parameters,
            [
                "rosclaw_robot_description_path",
                "rosclaw_gripper_candidate_mode",
            ],
            "MoveIt",
        )
        active_urdf = Path(move_group_values[0].string_value).resolve()
        expected_urdf = args.expected_urdf_path.expanduser().resolve()
        if active_urdf != expected_urdf or not move_group_values[1].bool_value:
            raise RuntimeError(
                "refusing to send: MoveIt is not running the expected "
                "parallel-gripper URDF"
            )

        if not node.move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveIt /move_action is unavailable")

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
        stationary = ARM_JOINTS + (
            f"openarm_{other_side}_finger_joint1",
            f"openarm_{other_side}_finger_joint2",
        )

        constraint = JointConstraint()
        constraint.joint_name = driver
        constraint.position = args.target
        constraint.tolerance_above = args.target_tolerance
        constraint.tolerance_below = args.target_tolerance
        constraint.weight = 1.0

        goal = MoveGroup.Goal()
        goal.request.group_name = f"{args.side}_gripper"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"
        goal.request.goal_constraints = [
            Constraints(
                name=f"{args.side}_gripper_target",
                joint_constraints=[constraint],
            )
        ]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = node.move_group.send_goal_async(goal)
        goal_handle = node.spin_until(
            lambda: send_future.result() if send_future.done() else None,
            15.0,
            "MoveIt goal acceptance",
        )
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the gripper goal")

        result_future = goal_handle.get_result_async()
        wrapped_result = node.spin_until(
            lambda: result_future.result() if result_future.done() else None,
            30.0,
            "MoveIt plan and execution result",
        )
        moveit_code = int(wrapped_result.result.error_code.val)
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or moveit_code != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                "MoveIt plan/execution failed: "
                f"action_status={wrapped_result.status}, "
                f"moveit_error_code={moveit_code}"
            )

        driver_samples = []
        follower_samples = []
        deadline = time.monotonic() + args.hold
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            driver_samples.append(node.positions[driver])
            follower_samples.append(node.positions[follower])

        final = dict(node.positions)
        target_error = abs(final[driver] - args.target)
        mimic_error = abs(final[follower] - final[driver])
        driver_drift = max(driver_samples) - min(driver_samples)
        follower_drift = max(follower_samples) - min(follower_samples)
        stationary_delta = max_delta(baseline, final, stationary)
        checks = {
            "moveit_execution_succeeded": True,
            "target_reached": target_error <= args.target_tolerance,
            "mimic_consistent": mimic_error <= args.mimic_tolerance,
            "driver_held": driver_drift <= args.hold_tolerance,
            "follower_held": follower_drift <= args.hold_tolerance,
            "other_joints_stationary": (
                stationary_delta <= args.stationary_tolerance
            ),
        }
        passed = all(checks.values())
        report = {
            "status": "PASS" if passed else "FAIL",
            "path_verified": {
                "moveit_urdf": str(active_urdf),
                "mujoco_mjcf": str(active_model),
                "planning_group": f"{args.side}_gripper",
                "controller": f"{args.side}_gripper_controller",
            },
            "target_driver_joint_m": args.target,
            "final_driver_joint_m": final[driver],
            "final_follower_joint_m": final[follower],
            "target_error_m": target_error,
            "mimic_error_m": mimic_error,
            "driver_hold_drift_m": driver_drift,
            "follower_hold_drift_m": follower_drift,
            "maximum_stationary_joint_delta": stationary_delta,
            "checks": checks,
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        print(
            "OPENARM GRIPPER MOVEIT CANDIDATE: "
            + ("PASS" if passed else "FAIL")
        )
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
