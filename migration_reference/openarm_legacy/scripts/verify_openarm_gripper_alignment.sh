#!/usr/bin/env bash
set -eo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$PROJECT/.rosclaw/artifacts/validation/gripper_alignment_$STAMP.json"
URDF=/tmp/openarm_v20_parallel_alignment.urdf

unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

cd "$PROJECT/ros_ws"
colcon build --symlink-install --packages-select openarm_description
source install/setup.bash

XACRO="$PROJECT/ros_ws/src/openarm_description/assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro"
xacro "$XACRO" \
  robot_preset:=openarm_rosclaw_parallel_bimanual \
  collapse_internal_empty_links:=true \
  emit_grasp_frame:=false \
  use_fake_hardware:=true \
  -o "$URDF"

check_urdf "$URDF"

/home/hkz/rosclaw_env/bin/python \
  "$PROJECT/scripts/test_openarm_gripper_alignment.py" \
  --urdf "$URDF" \
  --mjcf "$PROJECT/models/openarm_dual_mujoco_01/robot.mjcf.xml" \
  --description-root "$PROJECT/ros_ws/src/openarm_description" \
  --bridge "$PROJECT/ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py" \
  --moveit-controllers "$PROJECT/ros_ws/src/openarm_moveit_config/config/moveit_controllers.yaml" \
  --srdf "$PROJECT/ros_ws/src/openarm_moveit_config/srdf/openarm.srdf" \
  --report "$REPORT"
