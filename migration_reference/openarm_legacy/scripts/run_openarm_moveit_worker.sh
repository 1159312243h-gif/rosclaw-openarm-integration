#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${OPENARM_PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
ROS_SETUP="${OPENARM_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WORKSPACE_SETUP="${OPENARM_ROS_WORKSPACE_SETUP:-$PROJECT_ROOT/ros_ws/install/setup.bash}"

source_setup() {
  set +u
  # ROS-generated setup scripts may read optional variables before defining them.
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

[[ -r "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
[[ -r "$WORKSPACE_SETUP" ]] || {
  echo "Missing OpenArm ROS workspace setup: $WORKSPACE_SETUP" >&2
  echo "Build it with: cd $PROJECT_ROOT/ros_ws && colcon build --symlink-install" >&2
  exit 1
}

source_setup "$ROS_SETUP"
source_setup "$WORKSPACE_SETUP"

exec ros2 run rosclaw_openarm_bringup rosclaw_moveit_action_worker.py
