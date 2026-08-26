#!/usr/bin/env python3
"""Run the MuJoCo bridge against an explicit gripper candidate model."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("rosclaw_openarm_bringup"))
    moveit_share = Path(get_package_share_directory("openarm_moveit_config"))
    parameters = str(bringup_share / "config" / "openarm_sim.yaml")
    model_path = LaunchConfiguration("model_path")
    headless = LaunchConfiguration("headless")

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
            },
        ],
    )

    shutdown_on_bridge_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge,
            on_exit=[Shutdown(reason="MuJoCo candidate bridge exited")],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                description="Absolute path to the validated candidate MJCF",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            bridge,
            shutdown_on_bridge_exit,
        ]
    )
