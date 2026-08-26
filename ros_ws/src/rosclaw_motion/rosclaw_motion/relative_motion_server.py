#!/usr/bin/env python3
"""ROS 2 ActionServer for structured relative arm motions through MoveIt."""

import copy
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from rosclaw_interfaces.action import ArmMotion
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


ARM_JOINTS = {
    "left": [f"openarm_left_joint{index}" for index in range(1, 8)],
    "right": [f"openarm_right_joint{index}" for index in range(1, 8)],
}

ARM_BASE_LINKS = {
    "left": "openarm_left_base_link",
    "right": "openarm_right_base_link",
}

ARM_TIP_LINKS = {
    "left": "openarm_left_ee_base_link",
    "right": "openarm_right_ee_base_link",
}

ERROR_SUCCESS = 0
ERROR_INVALID_GOAL = -1
ERROR_NOT_READY = -2
ERROR_TF = -3
ERROR_PATH = -4
ERROR_MOVEIT_REJECTED = -5
ERROR_EXECUTION = -6
ERROR_CANCELED = -7
ERROR_INTERNAL = -8


class RelativeMotionServer(Node):
    def __init__(self) -> None:
        super().__init__("relative_motion_server")

        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("cartesian_service", "/compute_cartesian_path")
        self.declare_parameter("execute_action", "/execute_trajectory")
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("revolute_jump_threshold", 0.25)
        self.declare_parameter("minimum_path_fraction", 0.999)
        self.declare_parameter("max_cartesian_speed", 0.05)
        self.declare_parameter("verification_position_tolerance", 0.003)
        self.declare_parameter("verification_timeout", 8.0)
        self.declare_parameter("max_translation", 0.15)
        self.declare_parameter("default_velocity_scale", 0.20)
        self.declare_parameter("default_acceleration_scale", 0.20)
        self.declare_parameter("left_base_link", ARM_BASE_LINKS["left"])
        self.declare_parameter("right_base_link", ARM_BASE_LINKS["right"])
        self.declare_parameter("left_tip_link", ARM_TIP_LINKS["left"])
        self.declare_parameter("right_tip_link", ARM_TIP_LINKS["right"])

        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.base_links = {
            "left": str(self.get_parameter("left_base_link").value),
            "right": str(self.get_parameter("right_base_link").value),
        }
        self.tip_links = {
            "left": str(self.get_parameter("left_tip_link").value),
            "right": str(self.get_parameter("right_tip_link").value),
        }

        self.declare_parameter("request_id_cache_size", 4096)
        self.latest_joint_state: Optional[JointState] = None
        self.state_lock = threading.RLock()
        self.busy_arms = set()
        self.active_request_ids = set()
        self.seen_request_ids = {}
        self.execute_goal_handles = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        self.callback_group = ReentrantCallbackGroup()
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            str(self.get_parameter("cartesian_service").value),
            callback_group=self.callback_group,
        )
        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            str(self.get_parameter("execute_action").value),
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            ArmMotion,
            "/rosclaw/arm_motion",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "RosClaw relative motion server v0.4.2 listening on "
            "/rosclaw/arm_motion (MOVE_RELATIVE, Cartesian Path)"
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state = copy.deepcopy(message)

    def _has_arm_joint_state(self, arm: str) -> bool:
        state = self.latest_joint_state
        return state is not None and all(
            name in state.name for name in ARM_JOINTS[arm]
        )

    @staticmethod
    def _goal_key(goal_handle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    def _goal_callback(self, request: ArmMotion.Goal) -> GoalResponse:
        arm = request.arm.strip().lower()
        reference_frame = request.reference_frame.strip().lower()
        request_id = request.request_id.strip()
        if arm not in ARM_JOINTS:
            self.get_logger().error(f"Rejecting goal: invalid arm '{request.arm}'")
            return GoalResponse.REJECT
        if request.operation != ArmMotion.Goal.MOVE_RELATIVE:
            self.get_logger().error("Rejecting goal: only MOVE_RELATIVE is implemented")
            return GoalResponse.REJECT
        if reference_frame not in ("world", "base", "tool"):
            self.get_logger().error(
                f"Rejecting goal: invalid frame '{request.reference_frame}'"
            )
            return GoalResponse.REJECT
        if not request_id:
            self.get_logger().error("Rejecting goal: request_id is required")
            return GoalResponse.REJECT

        delta = request.translation
        if not all(math.isfinite(value) for value in (delta.x, delta.y, delta.z)):
            self.get_logger().error("Rejecting goal: translation must be finite")
            return GoalResponse.REJECT
        distance = math.sqrt(delta.x**2 + delta.y**2 + delta.z**2)
        maximum = float(self.get_parameter("max_translation").value)
        if distance <= 1.0e-6 or distance > maximum:
            self.get_logger().error(
                f"Rejecting goal: translation {distance:.4f} m is outside "
                f"(0, {maximum:.4f}]"
            )
            return GoalResponse.REJECT

        for name, value in (
            ("velocity_scale", request.velocity_scale),
            ("acceleration_scale", request.acceleration_scale),
        ):
            if not math.isfinite(value) or value < 0.0 or value > 1.0:
                self.get_logger().error(
                    f"Rejecting goal: {name} must be in [0.0, 1.0]"
                )
                return GoalResponse.REJECT

        with self.state_lock:
            if arm in self.busy_arms:
                self.get_logger().warning(f"Rejecting goal: {arm} arm is busy")
                return GoalResponse.REJECT
            if request_id in self.active_request_ids or request_id in self.seen_request_ids:
                self.get_logger().warning(
                    f"Rejecting duplicate request_id '{request_id}'"
                )
                return GoalResponse.REJECT
            self.busy_arms.add(arm)
            self.active_request_ids.add(request_id)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self.state_lock:
            execute_goal = self.execute_goal_handles.get(
                self._goal_key(goal_handle)
            )
        if execute_goal is not None:
            execute_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _publish_feedback(
        self,
        goal_handle,
        phase: int,
        progress: float,
        message: str,
        pose: Optional[PoseStamped] = None,
    ) -> None:
        feedback = ArmMotion.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.message = message
        if pose is not None:
            feedback.current_pose = pose
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        success: bool,
        error_code: int,
        message: str,
        final_pose: Optional[PoseStamped] = None,
        *,
        initial_pose: Optional[PoseStamped] = None,
        target_pose: Optional[PoseStamped] = None,
        final_error_m: Optional[float] = None,
    ) -> ArmMotion.Result:
        result = ArmMotion.Result()
        result.success = success
        result.error_code = error_code
        result.message = message
        if initial_pose is not None:
            result.initial_pose = initial_pose
        if target_pose is not None:
            result.target_pose = target_pose
        if final_pose is not None:
            result.final_pose = final_pose
        if final_error_m is not None:
            result.final_error_m = float(final_error_m)
        return result

    def _current_pose(self, arm: str) -> PoseStamped:
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            self.tip_links[arm],
            Time(),
        )
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _rotate_vector(vector: Vector3, rotation: Quaternion) -> Tuple[float, float, float]:
        ux, uy, uz = rotation.x, rotation.y, rotation.z
        scalar = rotation.w
        vx, vy, vz = vector.x, vector.y, vector.z
        dot_uv = ux * vx + uy * vy + uz * vz
        dot_uu = ux * ux + uy * uy + uz * uz
        cross_x = uy * vz - uz * vy
        cross_y = uz * vx - ux * vz
        cross_z = ux * vy - uy * vx
        return (
            2.0 * dot_uv * ux + (scalar * scalar - dot_uu) * vx + 2.0 * scalar * cross_x,
            2.0 * dot_uv * uy + (scalar * scalar - dot_uu) * vy + 2.0 * scalar * cross_y,
            2.0 * dot_uv * uz + (scalar * scalar - dot_uu) * vz + 2.0 * scalar * cross_z,
        )

    def _translation_in_planning_frame(
        self,
        arm: str,
        reference_frame: str,
        translation: Vector3,
        current_pose: PoseStamped,
    ) -> Tuple[float, float, float]:
        if reference_frame == "world":
            return translation.x, translation.y, translation.z
        if reference_frame == "tool":
            return self._rotate_vector(translation, current_pose.pose.orientation)

        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            self.base_links[arm],
            Time(),
        )
        return self._rotate_vector(translation, transform.transform.rotation)

    def _target_pose(self, arm: str, request: ArmMotion.Goal) -> Tuple[PoseStamped, PoseStamped]:
        current = self._current_pose(arm)
        dx, dy, dz = self._translation_in_planning_frame(
            arm,
            request.reference_frame.strip().lower(),
            request.translation,
            current,
        )
        target = copy.deepcopy(current)
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x += dx
        target.pose.position.y += dy
        target.pose.position.z += dz
        return current, target

    def _cartesian_request(
        self,
        arm: str,
        command: ArmMotion.Goal,
        target_pose: PoseStamped,
    ) -> GetCartesianPath.Request:
        request = GetCartesianPath.Request()
        request.header = copy.deepcopy(target_pose.header)
        request.start_state.joint_state = copy.deepcopy(self.latest_joint_state)
        request.group_name = f"{arm}_arm"
        request.link_name = self.tip_links[arm]
        request.waypoints = [copy.deepcopy(target_pose.pose)]
        request.max_step = float(
            self.get_parameter("cartesian_max_step").value
        )
        request.jump_threshold = float(
            self.get_parameter("cartesian_jump_threshold").value
        )
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = float(
            self.get_parameter("revolute_jump_threshold").value
        )
        request.avoid_collisions = command.avoid_collisions

        velocity = command.velocity_scale or float(
            self.get_parameter("default_velocity_scale").value
        )
        acceleration = command.acceleration_scale or float(
            self.get_parameter("default_acceleration_scale").value
        )
        request.max_velocity_scaling_factor = min(max(velocity, 0.01), 1.0)
        request.max_acceleration_scaling_factor = min(
            max(acceleration, 0.01), 1.0
        )
        request.cartesian_speed_limited_link = self.tip_links[arm]
        request.max_cartesian_speed = float(
            self.get_parameter("max_cartesian_speed").value
        )
        return request

    def _validate_cartesian_response(
        self,
        arm: str,
        response: GetCartesianPath.Response,
    ) -> Tuple[bool, str, float]:
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return (
                False,
                "Cartesian path failed: "
                f"error={response.error_code.val}, "
                f"source='{response.error_code.source}', "
                f"message='{response.error_code.message}'",
                0.0,
            )

        minimum_fraction = float(
            self.get_parameter("minimum_path_fraction").value
        )
        if response.fraction < minimum_fraction:
            return (
                False,
                f"Cartesian path fraction {response.fraction:.3f} is below "
                f"required {minimum_fraction:.3f}",
                0.0,
            )

        trajectory = response.solution.joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            return False, "Cartesian path returned an empty trajectory", 0.0
        if any(name not in trajectory.joint_names for name in ARM_JOINTS[arm]):
            return False, "Cartesian trajectory is missing arm joints", 0.0

        current = dict(
            zip(self.latest_joint_state.name, self.latest_joint_state.position)
        )
        previous = [float(current[name]) for name in trajectory.joint_names]
        max_joint_step = 0.0
        previous_time = -1.0
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != len(trajectory.joint_names):
                return (
                    False,
                    f"Cartesian trajectory point {index} has invalid positions",
                    0.0,
                )
            positions = [float(value) for value in point.positions]
            max_joint_step = max(
                max_joint_step,
                max(abs(value - old) for value, old in zip(positions, previous)),
            )
            previous = positions
            point_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) / 1.0e9
            )
            if point_time < previous_time:
                return False, "Cartesian trajectory timing is not monotonic", 0.0
            previous_time = point_time

        if previous_time <= 0.0:
            return False, "Cartesian trajectory has no time parameterization", 0.0
        maximum_jump = float(
            self.get_parameter("revolute_jump_threshold").value
        )
        if maximum_jump > 0.0 and max_joint_step > maximum_jump:
            return (
                False,
                f"Cartesian trajectory joint step {max_joint_step:.4f} rad "
                f"exceeds {maximum_jump:.4f} rad",
                0.0,
            )
        return True, "Cartesian path computed", max_joint_step

    async def _compute_cartesian_path(
        self,
        arm: str,
        command: ArmMotion.Goal,
        target_pose: PoseStamped,
    ) -> Tuple[Optional[GetCartesianPath.Response], str, float]:
        request = self._cartesian_request(arm, command, target_pose)
        response = await self.cartesian_client.call_async(request)
        if response is None:
            return None, "Cartesian path service returned no response", 0.0
        valid, message, max_joint_step = self._validate_cartesian_response(
            arm, response
        )
        if not valid:
            return None, message, max_joint_step
        return response, message, max_joint_step

    async def _execute_trajectory(
        self,
        response: GetCartesianPath.Response,
        server_goal_handle,
    ) -> Tuple[bool, str]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = copy.deepcopy(response.solution)
        execute_goal = await self.execute_client.send_goal_async(goal)
        if execute_goal is None or not execute_goal.accepted:
            return False, "MoveIt rejected the Cartesian trajectory"

        key = self._goal_key(server_goal_handle)
        with self.state_lock:
            self.execute_goal_handles[key] = execute_goal
        if server_goal_handle.is_cancel_requested:
            await execute_goal.cancel_goal_async()

        wrapped_result = await execute_goal.get_result_async()
        with self.state_lock:
            self.execute_goal_handles.pop(key, None)
        if server_goal_handle.is_cancel_requested:
            return False, "Motion was canceled"
        if wrapped_result is None:
            return False, "ExecuteTrajectory returned no result"

        result = wrapped_result.result
        success = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code.val == MoveItErrorCodes.SUCCESS
        )
        if not success:
            return (
                False,
                "ExecuteTrajectory failed: "
                f"status={wrapped_result.status}, "
                f"error={result.error_code.val}, "
                f"message='{result.error_code.message}'",
            )
        return True, "Cartesian motion completed"

    async def _delay(self, seconds: float) -> None:
        future = Future()

        def wake() -> None:
            if not future.done():
                future.set_result(True)

        timer = self.create_timer(
            seconds,
            wake,
            callback_group=self.callback_group,
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def _execute_callback(self, goal_handle):
        request = goal_handle.request
        arm = request.arm.strip().lower()
        request_id = request.request_id.strip()
        try:
            self._publish_feedback(
                goal_handle,
                ArmMotion.Feedback.VALIDATING,
                0.05,
                "Validating ROS and MoveIt state",
            )
            if not self._has_arm_joint_state(arm):
                goal_handle.abort()
                return self._result(
                    False,
                    ERROR_NOT_READY,
                    f"/joint_states does not contain all {arm} arm joints",
                )
            if not self.cartesian_client.service_is_ready():
                goal_handle.abort()
                return self._result(
                    False,
                    ERROR_NOT_READY,
                    "/compute_cartesian_path unavailable",
                )
            if not self.execute_client.server_is_ready():
                goal_handle.abort()
                return self._result(
                    False,
                    ERROR_NOT_READY,
                    "/execute_trajectory unavailable",
                )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, ERROR_CANCELED, "Canceled before planning")

            try:
                current_pose, target_pose = self._target_pose(arm, request)
            except Exception as exc:
                goal_handle.abort()
                return self._result(False, ERROR_TF, f"TF lookup failed: {exc}")

            self.get_logger().info(
                f"Request '{request_id}': {arm} arm, "
                f"frame={request.reference_frame.strip().lower()}, "
                f"target=({target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, "
                f"{target_pose.pose.position.z:.3f})"
            )

            self._publish_feedback(
                goal_handle,
                ArmMotion.Feedback.PLANNING,
                0.20,
                "Computing Cartesian path",
                current_pose,
            )
            cartesian_response, message, max_joint_step = (
                await self._compute_cartesian_path(
                    arm,
                    request,
                    target_pose,
                )
            )
            if cartesian_response is None:
                self.get_logger().error(f"Request '{request_id}': {message}")
                goal_handle.abort()
                return self._result(False, ERROR_PATH, message, current_pose)
            trajectory = cartesian_response.solution.joint_trajectory
            duration = trajectory.points[-1].time_from_start
            duration_seconds = float(duration.sec) + float(duration.nanosec) / 1.0e9
            self.get_logger().info(
                f"Request '{request_id}': Cartesian fraction="
                f"{cartesian_response.fraction:.3f}, "
                f"points={len(trajectory.points)}, "
                f"max_joint_step={max_joint_step:.6f} rad, "
                f"duration={duration_seconds:.3f} s"
            )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    False,
                    ERROR_CANCELED,
                    "Canceled after Cartesian planning",
                )

            self._publish_feedback(
                goal_handle,
                ArmMotion.Feedback.EXECUTING,
                0.50,
                "Executing Cartesian trajectory",
                current_pose,
            )
            success, message = await self._execute_trajectory(
                cartesian_response,
                goal_handle,
            )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, ERROR_CANCELED, message)
            if not success:
                goal_handle.abort()
                return self._result(False, ERROR_EXECUTION, message)

            try:
                verification_pose = self._current_pose(arm)
            except Exception:
                verification_pose = current_pose
            self._publish_feedback(
                goal_handle,
                ArmMotion.Feedback.VERIFYING,
                0.95,
                "Verifying final end-effector position",
                verification_pose,
            )
            verification_tolerance = float(
                self.get_parameter("verification_position_tolerance").value
            )
            deadline = time.monotonic() + float(
                self.get_parameter("verification_timeout").value
            )
            final_pose = current_pose
            position_error = math.inf
            while time.monotonic() < deadline:
                await self._delay(0.05)
                try:
                    final_pose = self._current_pose(arm)
                except Exception:
                    continue
                dx = target_pose.pose.position.x - final_pose.pose.position.x
                dy = target_pose.pose.position.y - final_pose.pose.position.y
                dz = target_pose.pose.position.z - final_pose.pose.position.z
                position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
                if position_error <= verification_tolerance:
                    break

            if position_error > verification_tolerance:
                message = (
                    "MoveIt/controller reported success, but final Cartesian "
                    f"error is {position_error:.4f} m"
                )
                self.get_logger().error(f"Request '{request_id}': {message}")
                goal_handle.abort()
                return self._result(
                    False,
                    ERROR_EXECUTION,
                    message,
                    final_pose,
                    initial_pose=current_pose,
                    target_pose=target_pose,
                    final_error_m=position_error,
                )
            goal_handle.succeed()
            self.get_logger().info(
                f"Request '{request_id}' completed: "
                f"final=({final_pose.pose.position.x:.3f}, "
                f"{final_pose.pose.position.y:.3f}, "
                f"{final_pose.pose.position.z:.3f}), "
                f"position_error={position_error:.4f} m"
            )
            return self._result(
                True,
                ERROR_SUCCESS,
                message,
                final_pose,
                initial_pose=current_pose,
                target_pose=target_pose,
                final_error_m=position_error,
            )
        except Exception as exc:
            self.get_logger().error(f"Unhandled ArmMotion error: {exc}")
            if goal_handle.is_active:
                goal_handle.abort()
            return self._result(False, ERROR_INTERNAL, str(exc))
        finally:
            with self.state_lock:
                self.busy_arms.discard(arm)
                self.active_request_ids.discard(request_id)
                self.seen_request_ids[request_id] = None
                cache_size = max(
                    1, int(self.get_parameter("request_id_cache_size").value)
                )
                while len(self.seen_request_ids) > cache_size:
                    oldest_request_id = next(iter(self.seen_request_ids))
                    self.seen_request_ids.pop(oldest_request_id, None)
                self.execute_goal_handles.pop(self._goal_key(goal_handle), None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeMotionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.action_server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
