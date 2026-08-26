#!/usr/bin/env python3
"""Run the complete MoveIt/MuJoCo stack with explicit gripper candidates."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("rosclaw_openarm_bringup"))
    moveit_share = Path(get_package_share_directory("openarm_moveit_config"))
    parameters = str(bringup_share / "config" / "openarm_sim.yaml")

    model_path = LaunchConfiguration("model_path")
    robot_description_path = LaunchConfiguration("robot_description_path")
    headless = LaunchConfiguration("headless")
    arm_position_gain = LaunchConfiguration("arm_position_gain")

    bridge = Node(
        package="rosclaw_openarm_bringup",
        executable="mujoco_moveit_bridge.py",
        name="mujoco_moveit_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            parameters,
            {
                "model_path": model_path,
                "srdf_path": str(moveit_share / "srdf" / "openarm.srdf"),
                "headless": ParameterValue(headless, value_type=bool),
                "enable_gripper_controllers": True,
                "position_gain": ParameterValue(
                    arm_position_gain, value_type=float
                ),
            },
        ],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / "launch" / "moveit_minimal.launch.py")
        ),
        launch_arguments={
            "robot_description_path": robot_description_path,
            "enable_gripper_controllers": "true",
        }.items(),
    )

    motion_server = Node(
        package="rosclaw_motion",
        executable="relative_motion_server",
        name="relative_motion_server",
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                description="Absolute path to the validated candidate MJCF",
            ),
            DeclareLaunchArgument(
                "robot_description_path",
                description="Absolute path to the matching candidate URDF",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "arm_position_gain",
                default_value="0.25",
                description=(
                    "Candidate-only arm tracking gain; formal simulation config "
                    "remains unchanged"
                ),
            ),
            bridge,
            moveit,
            motion_server,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bridge,
                    on_exit=[Shutdown(reason="MuJoCo candidate bridge exited")],
                )
            ),
        ]
    )
