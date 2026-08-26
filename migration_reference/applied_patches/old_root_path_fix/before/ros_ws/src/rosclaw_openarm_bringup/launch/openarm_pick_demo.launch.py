#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("rosclaw_openarm_bringup"))
    moveit_share = Path(get_package_share_directory("openarm_moveit_config"))
    project = Path(os.environ.get("OPENARM_PROJECT_ROOT", "/home/hkz/openarm_rosclaw"))
    model_path = LaunchConfiguration("model_path")
    robot_description_path = LaunchConfiguration("robot_description_path")
    headless = LaunchConfiguration("headless")
    parameters = str(bringup_share / "config" / "openarm_sim.yaml")

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
                "allowed_finger_contact_geoms": ["pick_demo_block_geom"],
                "observed_object_body": "pick_demo_block",
                "observed_object_topic": "/openarm/pick_demo/object_pose",
                "gripper_initial_position": 0.000,
                "position_gain": 0.25,
                "goal_tolerance": 0.003,
                "goal_time_margin": 7.0,
            },
        ],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / "launch" / "moveit_minimal.launch.py")),
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
                default_value=str(project / "models" / "openarm_dual_mujoco_01" / "robot.pick_demo.mjcf.xml"),
            ),
            DeclareLaunchArgument(
                "robot_description_path",
                default_value=str(project / "models" / "openarm_dual_mujoco_01" / "robot.urdf"),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            bridge,
            moveit,
            motion_server,
            RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[Shutdown(reason="MuJoCo bridge exited")])),
        ]
    )
