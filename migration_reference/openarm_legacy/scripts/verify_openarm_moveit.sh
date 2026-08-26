#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
VENV="${VENV:-/home/hkz/rosclaw_env}"
ROS_SETUP="${OPENARM_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_WS="$PROJECT_ROOT/ros_ws"
SYSTEM_DIST_PACKAGES="${OPENARM_SYSTEM_DIST_PACKAGES:-/usr/lib/python3/dist-packages}"
LOG="${OPENARM_MOVEIT_LOG:-/tmp/openarm_moveit_acceptance.log}"
LAUNCH_PID=""

source_setup() {
  set +u
  # ROS-generated setup scripts may read optional variables before defining them.
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

cleanup() {
  local pid="${LAUNCH_PID:-}"
  local deadline
  LAUNCH_PID=""

  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return
  fi

  kill -INT "$pid" 2>/dev/null || true
  deadline=$((SECONDS + 10))
  while kill -0 "$pid" 2>/dev/null && ((SECONDS < deadline)); do
    sleep 0.2
  done

  if kill -0 "$pid" 2>/dev/null; then
    echo "MoveIt bringup did not stop after SIGINT; sending SIGTERM" >&2
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
    deadline=$((SECONDS + 5))
    while kill -0 "$pid" 2>/dev/null && ((SECONDS < deadline)); do
      sleep 0.2
    done
  fi

  if kill -0 "$pid" 2>/dev/null; then
    echo "MoveIt bringup did not stop after SIGTERM; sending SIGKILL" >&2
    pkill -KILL -P "$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

[[ -r "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
[[ -x "$VENV/bin/python" ]] || { echo "Missing Python: $VENV/bin/python" >&2; exit 1; }

source_setup "$ROS_SETUP"
VENV_SITE_PACKAGES="$(
  "$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))'
)"
if [[ -d "$SYSTEM_DIST_PACKAGES" ]]; then
  export PYTHONPATH="$VENV_SITE_PACKAGES:$SYSTEM_DIST_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"
fi
"$VENV/bin/python" -c "import catkin_pkg, pydantic, typing_extensions"
cd "$ROS_WS"
colcon build --symlink-install --packages-up-to rosclaw_openarm_bringup

source_setup "$ROS_WS/install/setup.bash"
chmod +x \
  "$PROJECT_ROOT/scripts/run_openarm_moveit_worker.sh" \
  "$ROS_WS/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py" \
  "$ROS_WS/src/rosclaw_openarm_bringup/scripts/rosclaw_moveit_action_worker.py"

export OPENARM_PROJECT_ROOT="$PROJECT_ROOT"
ros2 launch rosclaw_openarm_bringup openarm_sim.launch.py headless:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!

deadline=$((SECONDS + 90))
while ((SECONDS < deadline)); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "OpenArm MoveIt bringup exited early. Log: $LOG" >&2
    tail -n 120 "$LOG" >&2 || true
    exit 1
  fi
  if ros2 action list 2>/dev/null | grep -Fxq '/rosclaw/arm_motion' \
    && ros2 service list 2>/dev/null | grep -Fxq '/compute_cartesian_path' \
    && ros2 action list 2>/dev/null | grep -Fxq '/execute_trajectory' \
    && ros2 action list 2>/dev/null \
      | grep -Fxq '/left_arm_controller/follow_joint_trajectory' \
    && ros2 action list 2>/dev/null \
      | grep -Fxq '/right_arm_controller/follow_joint_trajectory'; then
    break
  fi
  sleep 1
done

ros2 action list | grep -Fxq '/rosclaw/arm_motion' || {
  echo "ArmMotion action did not become ready. Log: $LOG" >&2
  tail -n 120 "$LOG" >&2 || true
  exit 1
}
ros2 service list | grep -Fxq '/compute_cartesian_path'
ros2 action list | grep -Fxq '/execute_trajectory'
ros2 action list | grep -Fxq '/left_arm_controller/follow_joint_trajectory'
ros2 action list | grep -Fxq '/right_arm_controller/follow_joint_trajectory'

# Require the MuJoCo bridge to publish robot state before asking MoveIt to plan.
if ! timeout 20 ros2 topic echo /joint_states --once >/dev/null; then
  echo "MuJoCo bridge did not publish /joint_states. Log: $LOG" >&2
  tail -n 120 "$LOG" >&2 || true
  exit 1
fi
sleep 2

source "$VENV/bin/activate"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export ROSCLAW_HOME="${ROSCLAW_HOME:-$PROJECT_ROOT/.rosclaw}"
export ROSCLAW_EURDF_ZOO="$PROJECT_ROOT/models"
export PROJECT_ROOT

cd "$PROJECT_ROOT"
PYTHONDONTWRITEBYTECODE=1 "$VENV/bin/python" \
  tests/integration/test_openarm_official_acceptance.py

echo "OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS"
