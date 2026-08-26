cmake_minimum_required(VERSION 3.8)
project(rosclaw_openarm_bringup)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/mujoco_moveit_bridge.py
    scripts/rosclaw_moveit_action_worker.py
    scripts/rosclaw_pick_demo_worker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
