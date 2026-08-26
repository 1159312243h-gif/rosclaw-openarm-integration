#!/usr/bin/env bash
set -eo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${OPENARM_CANDIDATE_DIR:-/home/hkz/openarm_parallel_mjcf_in_place_candidate_$STAMP}"
URDF=/tmp/openarm_v20_parallel_mjcf_candidate.urdf
FORMAL_MJCF="$PROJECT/models/openarm_dual_mujoco_01/robot.mjcf.xml"

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
  "$PROJECT/scripts/validate_openarm_parallel_urdf_candidate.py" \
  "$URDF" \
  --expect parallel \
  --baseline "$PROJECT/models/openarm_dual_mujoco_01/robot.urdf"

/home/hkz/rosclaw_env/bin/python \
  "$PROJECT/scripts/build_openarm_parallel_mjcf_candidate.py" \
  --urdf "$URDF" \
  --description-root "$PROJECT/ros_ws/src/openarm_description" \
  --scene-source "$FORMAL_MJCF" \
  --output-dir "$OUTPUT_DIR"

/home/hkz/rosclaw_env/bin/python \
  "$PROJECT/scripts/diagnose_openarm_gripper_collisions.py" \
  --urdf "$OUTPUT_DIR/robot.urdf" \
  --mjcf "$OUTPUT_DIR/robot.mjcf.xml" \
  --description-root "$PROJECT/ros_ws/src/openarm_description" \
  --output "$OUTPUT_DIR/kinematic_collision_alignment.json" \
  --use-candidate-initial-state \
  --require-candidate-alignment

echo "788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1  $FORMAL_MJCF" \
  | sha256sum --check --status
echo "OPENARM FORMAL MJCF UNCHANGED: PASS"
echo "candidate viewer command:"
echo "/home/hkz/rosclaw_env/bin/python $PROJECT/scripts/view_openarm_parallel_mjcf_candidate.py $OUTPUT_DIR/robot.mjcf.xml"
echo "OPENARM IN-PLACE PARALLEL MJCF CANDIDATE ACCEPTANCE: PASS"
