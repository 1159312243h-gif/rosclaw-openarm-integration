# OpenArm 候选运行绑定与终点收敛修复

本补丁只用于候选模型验收，不覆盖正式 URDF、MJCF 或 `runtime.yaml`。

修复内容：

- 候选 MoveIt/MuJoCo launch 使用独立的 `arm_position_gain=0.25`；
- 正式 `openarm_sim.yaml` 仍保持 `position_gain=0.15` 和 `goal_tolerance=0.003`；
- ROSClaw 候选 profile 从候选 `robot.mjcf.xml` 自动计算 Receipt 哈希；
- 候选 ROSClaw home、artifact 和 model zoo 与正式目录隔离；
- 控制器失败时记录误差最大的关节、目标值和实际值；
- 提供 OpenClaw MCP 候选启用和正式恢复脚本。

## 1. 准备候选运行目录

```bash
cd /home/hkz/openarm_rosclaw
source /home/hkz/rosclaw_env/bin/activate

CANDIDATE=/home/hkz/openarm_parallel_end_effector_candidate_20260812_v3_3
RUNTIME_DIR=/home/hkz/openarm_parallel_candidate_runtime_20260812_v3_3

/home/hkz/rosclaw_env/bin/python \
  scripts/prepare_openarm_candidate_runtime.py \
  --candidate "$CANDIDATE" \
  --output "$RUNTIME_DIR"

sha256sum \
  "$CANDIDATE/robot.urdf" \
  "$RUNTIME_DIR/models/openarm_dual_mujoco_01/robot.urdf" \
  "$CANDIDATE/robot.mjcf.xml" \
  "$RUNTIME_DIR/models/openarm_dual_mujoco_01/robot.mjcf.xml"
```

预期候选 MJCF 两行均为：

```text
ae35f356ad09bd68a30bb69e4419d540e6b9c1c011e061e04b1a448af3d4265e
```

正式模型应保持：

```text
788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1
```

## 2. 构建和重启候选仿真

先在旧 launch 终端按 `Ctrl+C`，然后：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

cd /home/hkz/openarm_rosclaw/ros_ws
colcon build --symlink-install --packages-select \
  openarm_moveit_config \
  rosclaw_openarm_bringup
source install/setup.bash

CANDIDATE=/home/hkz/openarm_parallel_end_effector_candidate_20260812_v3_3

ros2 launch rosclaw_openarm_bringup \
  openarm_moveit_gripper_candidate.launch.py \
  robot_description_path:="$CANDIDATE/robot.urdf" \
  model_path:="$CANDIDATE/robot.mjcf.xml" \
  arm_position_gain:=0.25 \
  headless:=false
```

## 3. 检查绑定

在新终端：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

CANDIDATE=/home/hkz/openarm_parallel_end_effector_candidate_20260812_v3_3
RUNTIME_DIR=/home/hkz/openarm_parallel_candidate_runtime_20260812_v3_3

ACTIVE_MJCF="$(ros2 param get /mujoco_moveit_bridge model_path | sed -n 's/^String value is: //p')"
ACTIVE_URDF="$(ros2 param get /move_group rosclaw_robot_description_path | sed -n 's/^String value is: //p')"

/home/hkz/rosclaw_env/bin/python \
  /home/hkz/openarm_rosclaw/scripts/verify_openarm_candidate_binding.py \
  --candidate "$CANDIDATE" \
  --runtime "$RUNTIME_DIR/runtime.yaml" \
  --active-mjcf "$ACTIVE_MJCF" \
  --active-urdf "$ACTIVE_URDF"
```

必须得到 `status: PASS` 后才能切换 MCP。

## 4. 临时启用候选 MCP

```bash
export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"

/home/hkz/openarm_rosclaw/scripts/activate_openarm_candidate_mcp.sh \
  /home/hkz/openarm_parallel_candidate_runtime_20260812_v3_3/runtime.yaml
```

探测必须显示 `rosclaw: 22 tools`。然后才能通过 OpenClaw 执行一次新的
`sandbox_run`。不得复用旧 Action ID，不得自动重试。

## 5. 验收后恢复正式 MCP

无论候选动作成功还是失败，完成结果收集后执行：

```bash
export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"
/home/hkz/openarm_rosclaw/scripts/restore_openarm_formal_mcp.sh
```

恢复只影响 OpenClaw MCP 配置，不提升或覆盖正式模型。
