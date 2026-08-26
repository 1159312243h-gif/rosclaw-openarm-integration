# OpenArm 平移抓手 ROS 候选闭环测试

本测试只运行 MuJoCo 候选模型，不启动 MoveIt、ROSClaw ActionGateway 或真实机械臂执行器。
正式 MJCF 不会被覆盖，抓手控制器在默认配置中保持关闭。

候选模型：

```text
/home/hkz/openarm_parallel_mjcf_in_place_candidate_20260811_165134/robot.mjcf.xml
```

候选模型预期 SHA256：

```text
cea01c025d82dca88954a0d90627bce9502480973cc097b3287653189e238062
```

## 1. 应用补丁

```bash
cd /home/hkz/openarm_rosclaw

STAMP="$(date +%Y%m%d_%H%M%S)"
tar -czf "/home/hkz/openarm_gripper_ros_before_$STAMP.tar.gz" \
  ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py \
  ros_ws/src/rosclaw_openarm_bringup/config/openarm_sim.yaml \
  ros_ws/src/rosclaw_openarm_bringup/launch \
  scripts

unzip -o /home/hkz/openarm_gripper_ros_candidate_v3_20260811.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x \
  ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py \
  scripts/test_openarm_gripper_ros_action.py
```

## 2. 校验并编译

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

VENV_SITE="$(
  /home/hkz/rosclaw_env/bin/python -c \
  'import sysconfig; print(sysconfig.get_path("purelib"))'
)"
export PYTHONPATH="$VENV_SITE:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

PROJECT=/home/hkz/openarm_rosclaw
CANDIDATE=/home/hkz/openarm_parallel_mjcf_in_place_candidate_20260811_165134/robot.mjcf.xml

sha256sum "$CANDIDATE"

cd "$PROJECT/ros_ws"
colcon build --symlink-install \
  --packages-select rosclaw_openarm_bringup

source install/setup.bash
```

必须先确认 `sha256sum` 等于文档顶部的候选模型哈希。

## 3. 终端一：启动候选仿真

继续在完成第 2 步环境设置的终端运行：

```bash
ros2 launch rosclaw_openarm_bringup \
  openarm_gripper_candidate.launch.py \
  model_path:="$CANDIDATE" \
  headless:=false
```

启动后抓手应保持完全张开，不会自行循环运动。终端应列出：

```text
/left_gripper_controller/follow_joint_trajectory
/right_gripper_controller/follow_joint_trajectory
```

## 4. 终端二：发送一次右抓手目标

新建终端并重新设置环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

VENV_SITE="$(
  /home/hkz/rosclaw_env/bin/python -c \
  'import sysconfig; print(sysconfig.get_path("purelib"))'
)"
export PYTHONPATH="$VENV_SITE:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

PROJECT=/home/hkz/openarm_rosclaw
CANDIDATE=/home/hkz/openarm_parallel_mjcf_in_place_candidate_20260811_165134/robot.mjcf.xml

source "$PROJECT/ros_ws/install/setup.bash"

ros2 action list -t | grep gripper_controller

/home/hkz/rosclaw_env/bin/python \
  "$PROJECT/scripts/test_openarm_gripper_ros_action.py" \
  --expected-model-path "$CANDIDATE" \
  --side right \
  --target 0.011 \
  --duration 2.0 \
  --hold 5.0
```

该命令只发送一次目标。重启候选仿真后抓手从 `0.000 m` 张开状态出发，
约用 2 秒闭合到 `0.011 m`，等待位置和速度连续稳定后返回成功，再保持至少 5 秒。

## 5. 验收结果

成功时最后一行必须是：

```text
OPENARM GRIPPER ROS ACTION CANDIDATE: PASS
```

同时目视确认：

- 只有右抓手移动；
- 两个右手指同步、对称；
- 到达中间位置后不再运动；
- 左抓手和两条机械臂保持不动；
- 手指没有脱离手腕。

不要继续测试 `0.000` 和 `0.044`，直到 `0.022` 的自动验收和目视验收都通过。
