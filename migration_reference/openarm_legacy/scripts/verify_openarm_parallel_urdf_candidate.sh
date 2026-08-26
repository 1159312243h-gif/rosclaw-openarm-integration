#!/usr/bin/env bash
set -eo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
EXPECTED_MJCF_SHA256="788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1"

unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

cd "$PROJECT/ros_ws"
colcon build --symlink-install --packages-select openarm_description
source install/setup.bash

XACRO="$PROJECT/ros_ws/src/openarm_description/assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro"
DEFAULT_URDF=/tmp/openarm_v20_default_regression.urdf
PARALLEL_URDF=/tmp/openarm_v20_parallel_candidate.urdf
FORMAL_URDF="$PROJECT/models/openarm_dual_mujoco_01/robot.urdf"
FORMAL_MJCF="$PROJECT/models/openarm_dual_mujoco_01/robot.mjcf.xml"
VALIDATOR="$PROJECT/scripts/validate_openarm_parallel_urdf_candidate.py"

xacro "$XACRO" \
  robot_preset:=default_bimanual \
  collapse_internal_empty_links:=true \
  emit_grasp_frame:=false \
  use_fake_hardware:=true \
  -o "$DEFAULT_URDF"

xacro "$XACRO" \
  robot_preset:=openarm_rosclaw_parallel_bimanual \
  collapse_internal_empty_links:=true \
  emit_grasp_frame:=false \
  use_fake_hardware:=true \
  -o "$PARALLEL_URDF"

check_urdf "$DEFAULT_URDF"
check_urdf "$PARALLEL_URDF"

/home/hkz/rosclaw_env/bin/python "$VALIDATOR" \
  "$DEFAULT_URDF" --expect pinch

/home/hkz/rosclaw_env/bin/python "$VALIDATOR" \
  "$PARALLEL_URDF" --expect parallel --baseline "$FORMAL_URDF"

echo "$EXPECTED_MJCF_SHA256  $FORMAL_MJCF" | sha256sum --check --status
echo "OPENARM FORMAL MJCF UNCHANGED: PASS"
echo "candidate URDF: $PARALLEL_URDF"
echo "OPENARM PARALLEL URDF CANDIDATE ACCEPTANCE: PASS"
