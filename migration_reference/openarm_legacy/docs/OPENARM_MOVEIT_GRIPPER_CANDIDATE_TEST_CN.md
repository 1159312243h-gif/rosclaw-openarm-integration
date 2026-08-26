# OpenArm MoveIt 平移抓手候选集成测试

本测试使用一对匹配的候选模型：

```text
URDF: /home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2/robot.urdf
MJCF: /home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2/robot.mjcf.xml
```

正式模型不会被覆盖。原 `openarm_sim.launch.py` 仍默认加载旧 `output.urdf`
和正式 MJCF，且默认不注册抓手控制器。

## 1. 应用补丁

先在当前候选仿真终端按 `Ctrl+C`。

```bash
cd /home/hkz/openarm_rosclaw

STAMP="$(date +%Y%m%d_%H%M%S)"
tar -czf "/home/hkz/openarm_moveit_gripper_before_$STAMP.tar.gz" \
  ros_ws/src/openarm_moveit_config \
  ros_ws/src/rosclaw_openarm_bringup/launch \
  ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py \
  ros_ws/src/rosclaw_openarm_bringup/config/openarm_sim.yaml

unzip -o /home/hkz/openarm_moveit_gripper_candidate_20260812.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x scripts/test_openarm_gripper_moveit.py
```

## 2. 构建

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

VENV_SITE="$(
  /home/hkz/rosclaw_env/bin/python -c \
  'import sysconfig; print(sysconfig.get_path("purelib"))'
)"
export PYTHONPATH="$VENV_SITE:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

cd /home/hkz/openarm_rosclaw/ros_ws
colcon build --symlink-install --packages-select \
  openarm_moveit_config \
  rosclaw_openarm_bringup

source install/setup.bash
```

## 3. 终端一：启动完整候选栈

```bash
CANDIDATE_DIR=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2
URDF="$CANDIDATE_DIR/robot.urdf"
MJCF="$CANDIDATE_DIR/robot.mjcf.xml"

sha256sum "$URDF" "$MJCF"

ros2 launch rosclaw_openarm_bringup \
  openarm_moveit_gripper_candidate.launch.py \
  robot_description_path:="$URDF" \
  model_path:="$MJCF" \
  headless:=false
```

预期哈希：

```text
7b575471e2f974f2ecf500c4ca620a2e96d4c82e57418331d7f9ed11114da149  robot.urdf
ae35f356ad09bd68a30bb69e4419d540e6b9c1c011e061e04b1a448af3d4265e  robot.mjcf.xml
```

启动日志必须包含：

```text
MoveIt robot description: .../openarm_parallel_mjcf_mimic_candidate_20260811_v2/robot.urdf
MoveIt gripper controllers enabled: True
```

## 4. 终端二：只读核对

重新加载 ROS、虚拟环境和工作区，然后运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

ros2 action list -t | grep -E \
  'move_action|gripper_controller|arm_controller|rosclaw/arm_motion'

ros2 param get /move_group rosclaw_robot_description_path
ros2 param get /move_group rosclaw_gripper_candidate_mode
ros2 param get /mujoco_moveit_bridge model_path
ros2 param get /mujoco_moveit_bridge enable_gripper_controllers
```

四个参数必须分别指向候选 URDF、`true`、候选 MJCF、`true`。

## 5. 通过 MoveIt 测试右抓手

```bash
CANDIDATE_DIR=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2

/home/hkz/rosclaw_env/bin/python \
  /home/hkz/openarm_rosclaw/scripts/test_openarm_gripper_moveit.py \
  --expected-urdf-path "$CANDIDATE_DIR/robot.urdf" \
  --expected-model-path "$CANDIDATE_DIR/robot.mjcf.xml" \
  --side right \
  --target 0.011 \
  --hold 5.0
```

成功时最后一行必须是：

```text
OPENARM GRIPPER MOVEIT CANDIDATE: PASS
```

报告中的 `path_verified` 必须列出候选 URDF、候选 MJCF、
`right_gripper` 和 `right_gripper_controller`。

右侧通过前不要执行左侧、边界位置或手臂回归。
