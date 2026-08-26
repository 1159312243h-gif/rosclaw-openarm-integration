# OpenClaw、ROSClaw、MoveIt 与 MuJoCo 调度 OpenArm

本文基于已经通过 MoveIt 黑盒验收的项目基线，说明项目文件结构，以及用户在 OpenClaw 中输入一条自然语言指令后，系统如何通过官方 ROSClaw、MoveIt 2 和 MuJoCo 完成 OpenArm 仿真运动。Action ID 与 Receipt 审计增强属于其上的增量，覆盖到虚拟机后必须重新执行完整验收，不能沿用旧测试结果。

当前正式项目目录：

```text
/home/hkz/openarm_rosclaw
```

当前 ROSClaw 版本为 `1.0.1`，Python 实际加载位置为：

```text
/home/hkz/openarm_rosclaw/src/rosclaw/__init__.py
```

当前已验收范围仅为 `SIMULATION`。本文不宣称真实机械臂已经接通。

## 1. 总体架构

当前项目不是重新实现一套机器人框架，而是在官方 ROSClaw 1.0.1 的目录结构、MCP、Runtime、ActionGateway、Sandbox 和 Receipt 机制上增加 OpenArm 支持。

```text
用户自然语言
  -> OpenClaw 自动选择 openarm-move-relative Agent Skill
  -> 名为 rosclaw 的官方 22-tool MCP
  -> RuntimeClient.sandbox_run()
  -> 官方 Runtime.submit_action()
  -> 官方 ActionGateway.submit()
  -> OpenArm MoveIt Sandbox Executor
  -> 隔离 ROS Action worker
  -> /rosclaw/arm_motion
  -> MoveIt GetCartesianPath
  -> MoveIt ExecuteTrajectory
  -> FollowJointTrajectory
  -> mujoco_moveit_bridge.py
  -> MuJoCo 物理仿真
  -> 最终 TF 位置验证
  -> 带 SHA256 完整性校验的 ExecutionReceipt
```

MoveIt 是正式执行链路中的必经层。如果 MoveIt、ROS Action Server 或 MuJoCo bridge 不可用，动作必须失败关闭，不允许回退到直接修改 MuJoCo 关节的旧实现。

## 2. 项目文件结构

```text
/home/hkz/openarm_rosclaw/
├── .agents/                    # 面向 OpenClaw/Codex 等 Agent 的操作说明
│   └── skills/
│       ├── openarm-move-relative/
│       ├── rosclaw/
│       └── rosclaw-simforge/
├── .codex/                     # Codex 项目级配置
├── bin/                        # 官方 ROSClaw 命令辅助程序
├── configs/                    # 官方框架配置资源
├── deploy/                     # 官方部署资源
├── docs/                       # 架构和操作文档
├── models/                     # ROSClaw Body、URDF 和 MuJoCo 模型
├── ros_ws/                     # ROS 2 Jazzy 工作空间
│   └── src/
├── scripts/                    # 构建、启动、安装和验收脚本
├── skills/                     # ROSClaw Runtime Skill 包
├── src/rosclaw/                # 官方 ROSClaw 1.0.1 及 OpenArm 扩展
├── tests/                      # Python、配置和集成测试
├── worker_plugins/             # 官方隔离 worker 插件机制
├── .mcp.json                   # 官方 Agent onboarding 生成的项目级 MCP 描述
├── AGENTS.md                   # Agent 进入项目后的通用约束
├── CLAUDE.md                   # Claude Code 项目说明
├── ROSCLAW.md                  # ROSClaw 项目操作手册
├── runtime.yaml                # 当前 OpenArm Runtime 总配置
├── pyproject.toml              # Python 包与构建配置
├── README.md                   # 项目入口说明
└── ARCHITECTURE.md             # 官方总体架构说明
```

### 2.1 OpenClaw 实际配置

项目内的 `.mcp.json` 用于官方 Agent onboarding 和项目检查。OpenClaw 当前真正读取的原生配置位于：

```text
/home/hkz/.openclaw/openclaw.json
```

其中名为 `rosclaw` 的 MCP 启动命令等价于：

```text
/home/hkz/rosclaw_env/bin/python
  -m rosclaw.entrypoint
  mcp serve
  --profile /home/hkz/openarm_rosclaw/runtime.yaml
  --project /home/hkz/openarm_rosclaw
  --log-level ERROR
```

MCP 名称保持官方名称 `rosclaw`，没有重新命名。

### 2.2 两种 Skill 的区别

项目中存在两个层次的 Skill，它们不是重复文件。

第一种是 Agent Skill：

```text
.agents/skills/openarm-move-relative/SKILL.md
```

部署后位于：

```text
/home/hkz/.openclaw/workspace/skills/openarm-move-relative/SKILL.md
```

它负责让 OpenClaw 理解用户语言、自动选择正确工作流、换算单位、映射方向并决定 MCP 工具顺序。

第二种是 ROSClaw Runtime Skill：

```text
skills/openarm-move-relative/
```

其中包括：

```text
skill.yaml                 # Skill 身份、能力和版本
behavior_tree.xml          # 运行步骤结构
safety.yaml                # 安全约束
providers.yaml             # Provider 要求
policies/policy.yaml       # 执行策略
policies/params/default.yaml
prompts/                   # planner、executor、verifier、recovery 提示
e-urdf-compat.yaml         # Body 兼容条件
tests/test_schema.py       # Skill 格式测试
.rosclaw/hashes.json       # 文件完整性摘要
.rosclaw/lock.yaml         # 锁定后的 Skill 元数据
```

Agent Skill 解决“OpenClaw 应该怎么理解和调用”；Runtime Skill 解决“ROSClaw 是否注册、允许并能够执行该能力”。

## 3. OpenArm 模型

正式模型目录：

```text
models/openarm_dual_mujoco_01/
├── robot.urdf             # ROS 和 MoveIt 使用的机器人结构
├── robot.mjcf.xml         # MuJoCo 使用的物理模型
├── robot.eurdf.yaml       # ROSClaw Effective Body 描述
├── capabilities.yaml      # Body 能力声明
├── safety.yaml            # 关节和能力限制
├── semantic.yaml          # 语义分组和末端信息
├── benchmark.yaml         # 模型验收信息
└── model.sha256           # 模型完整性校验
```

当前模型包含左右两条七自由度机械臂以及左右抓手。当前已启用能力是：

```text
openarm.arm.move_relative:SIMULATION
```

抓手模型存在，但抓取 Skill 和执行能力尚未完成正式验收。

## 4. ROS 2 工作空间

```text
ros_ws/src/
├── openarm_description/
├── openarm_moveit_config/
├── rosclaw_interfaces/
├── rosclaw_motion/
└── rosclaw_openarm_bringup/
```

### 4.1 openarm_description

保存 OpenArm 的 Xacro、关节、惯性、网格和抓手描述，是生成 ROS 机器人模型的基础。

### 4.2 openarm_moveit_config

保存 MoveIt 需要的 SRDF、运动学、关节限制、规划管线和控制器配置。主要文件包括：

```text
config/moveit_controllers.yaml
config/kinematics.yaml
launch/moveit_minimal.launch.py
```

### 4.3 rosclaw_interfaces

定义类型化 ROS Action：

```text
rosclaw_interfaces/action/ArmMotion.action
```

它明确规定动作请求、Feedback 和 Result 的字段，避免使用无结构字符串控制机械臂。

### 4.4 rosclaw_motion

实现 `/rosclaw/arm_motion` Action Server：

```text
rosclaw_motion/rosclaw_motion/relative_motion_server.py
```

它负责读取当前 TF、调用 MoveIt 规划、执行轨迹，并在完成后重新读取 TF 验证最终位置。

### 4.5 rosclaw_openarm_bringup

负责把 MoveIt、Motion Server 和 MuJoCo bridge 一次启动起来。

```text
launch/openarm_sim.launch.py
config/openarm_sim.yaml
scripts/mujoco_moveit_bridge.py
scripts/rosclaw_moveit_action_worker.py
```

`mujoco_moveit_bridge.py` 提供左右臂 `FollowJointTrajectory` controller，并把轨迹真正施加到 MuJoCo 物理模型。

`rosclaw_moveit_action_worker.py` 是一次性隔离 ActionClient。它接收 ROSClaw Executor 的 JSON 请求，转换成 `ArmMotion.Goal`，等待结果后返回 JSON。

## 5. 运行前提

OpenClaw 调用的 worker 只是 ROS ActionClient，不负责启动整个 MoveIt 系统。因此正式从 OpenClaw 发出动作前，必须先启动 bringup：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

ros2 launch rosclaw_openarm_bringup \
  openarm_sim.launch.py headless:=false
```

`headless:=false` 用于人工演示，可以看到 MuJoCo 窗口。自动验收使用 `headless:=true`。

OpenClaw Gateway 和 `rosclaw` MCP 还必须处于可用状态：

```bash
openclaw gateway status
openclaw mcp probe rosclaw
```

MCP 探测应显示 `22 tools`。

## 6. 示例：右臂沿世界坐标系 +Z 移动 2 mm

用户在 OpenClaw 输入：

```text
仅在 SIMULATION 模式下，让右臂末端沿世界坐标系 +Z 方向移动 2 毫米。
```

### 第一步：OpenClaw 自动选择 Agent Skill

OpenClaw 能看到所有 eligible Skill 的名称和 description。`openarm-move-relative` 的 description 包含 OpenArm、左右臂、移动、抬高、降低、毫米、厘米、X/Y/Z、MoveIt-to-MuJoCo 等语义，因此它比通用 `rosclaw` Skill 更匹配这条指令。

用户不需要说出 Skill 名称。OpenClaw 选中后读取完整 `SKILL.md`，并按照其中的限制规划。

### 第二步：归一化自然语言

Skill 将指令转换为：

```yaml
arm: right
reference_frame: world
translation:
  x: 0.0
  y: 0.0
  z: 0.002
velocity_scale: 0.05
acceleration_scale: 0.05
avoid_collisions: true
execution_mode: SIMULATION
```

转换规则：

```text
向上 = world +Z
2 mm = 0.002 m
```

单步三维向量模长上限为 `0.02 m`，因此本次只有一步。如果总位移超过 20 mm，Skill 按向量模长拆分，并且逐步串行执行。

如果用户只说“向左”而没有定义 world X/Y 方向，Skill 必须先询问，不能猜测。

### 第三步：通过官方 MCP 检查环境

OpenClaw 按 Skill 规定调用名为 `rosclaw` 的官方 MCP：

| 顺序 | MCP 工具 | 目的 |
|---|---|---|
| 1 | `get_body_state` | 确认 Body 是 `openarm_dual_mujoco_01` |
| 2 | `list_body_capabilities` | 确认 `openarm.arm.move_relative:SIMULATION` 可用 |
| 3 | `list_skills` | 确认 Runtime Skill 已注册 |
| 4 | `validate_body_action` | 检查 Body、Capability 和风险是否允许提出 |
| 5 | `sandbox_run` | 提交受限仿真动作 |
| 6 | `get_execution_receipt` | 独立读取并校验最终 Receipt |

只有 `validate_body_action` 返回 `allowed_to_propose=true`，才能继续执行。

### 第四步：构造 ActionEnvelope

`sandbox_run` 的主要实现位于：

```text
src/rosclaw/mcp/adapters/runtime_client.py
```

RuntimeClient 在服务端构造 ActionEnvelope：

```text
body_id = openarm_dual_mujoco_01
capability_id = openarm.arm.move_relative
execution_mode = SIMULATION
required_evidence = TASK_VERIFIED
timeout_sec = 30.0
fail_closed = true
```

Action ID 由 ROSClaw 服务端生成，使用 `action_<uuid>` 格式；模型不能提供或复用
Action ID，也不能把模式改成 REAL、选择其他机器人或选择未注册 Capability。
这一约束同时适用于模型可见的 `sandbox_run` 和 `request_action`。内部反序列化仍能
读取历史 Action ID，以支持旧 Receipt、守护进程恢复和幂等查询，但不会把该字段暴露给模型。

### 第五步：进入官方 Runtime 与 ActionGateway

调用链：

```text
RuntimeClient.sandbox_run()
  -> Runtime.submit_action()
  -> ActionGateway.submit()
```

对应文件：

```text
src/rosclaw/core/runtime.py
src/rosclaw/kernel/action_gateway.py
```

ActionGateway 负责：

- 相同 Action ID 的幂等处理；
- 抑制同一动作的并发重复执行；
- 为 `openarm_dual_mujoco_01` 获取独占资源 Lease；
- 按 Capability 和执行模式选择 Executor；
- 管理超时和状态转换；
- 校验证据等级；
- 生成并持久化 ExecutionReceipt。

### 第六步：选择 OpenArm MoveIt Executor

Runtime 初始化时，将：

```text
openarm.arm.move_relative:SIMULATION
```

注册到：

```text
SandboxRuntimeAdapter.execute_action()
  -> run_openarm_move_relative_via_moveit()
```

对应文件：

```text
src/rosclaw/sandbox/runtime_adapter.py
src/rosclaw/sandbox/openarm_moveit.py
```

`openarm_moveit.py` 会复用：

```text
src/rosclaw/sandbox/openarm.py
```

中的 `validate_openarm_move_relative()`，检查：

- Body 和 Capability 是否正确；
- 模式是否为 SIMULATION；
- arm 是否为 left/right；
- frame 是否为 world；
- translation 是否恰好包含 x/y/z；
- 每个分量和三维模长是否不超过 20 mm；
- 速度、加速度和碰撞参数是否合法。

`openarm.py` 中保留的直接 MuJoCo 差分 IK 不是当前正式执行后端。正式注册只指向 `openarm_moveit.py`。

### 第七步：启动隔离 worker

Executor 将动作转换为小型 JSON 请求，并调用：

```text
scripts/run_openarm_moveit_worker.sh
```

该脚本加载 ROS 2 Jazzy 和 `ros_ws/install/setup.bash`，再启动：

```text
rosclaw_openarm_bringup/rosclaw_moveit_action_worker.py
```

worker 构造类型化 `ArmMotion.Goal`，发送到：

```text
/rosclaw/arm_motion
```

ROS 依赖被隔离在 worker 子进程中，不进入 OpenClaw 模型参数层。

### 第八步：Motion Server 调用 MoveIt

Motion Server 位于：

```text
ros_ws/src/rosclaw_motion/rosclaw_motion/relative_motion_server.py
```

它依次执行：

1. 检查 `/joint_states`、TF、MoveIt service 和 action 是否就绪；
2. 从 TF 读取右臂当前末端位姿；
3. 计算 `目标位置 = 当前位置 + world (0, 0, 0.002)`；
4. 调用 MoveIt `GetCartesianPath`；
5. 检查路径 fraction、轨迹非空、关节集合、时间单调性和关节跳变；
6. 调用 MoveIt `ExecuteTrajectory`；
7. 等待 controller 执行；
8. 重新读取最终 TF；
9. 检查最终位置误差不超过 `0.0005 m`。

OpenClaw 和 Skill 不计算逆运动学，也不生成关节角。笛卡尔路径、逆运动学和关节轨迹由 MoveIt 完成。

### 第九步：FollowJointTrajectory 驱动 MuJoCo

MoveIt 将右臂轨迹发送到：

```text
/right_arm_controller/follow_joint_trajectory
```

接收端是：

```text
ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py
```

bridge 负责：

1. 校验轨迹关节名称、轨迹点和时间；
2. 按 `time_from_start` 插值关节目标；
3. 更新 MuJoCo 控制状态；
4. 调用 `mujoco.mj_step()` 推进物理仿真；
5. 发布 `/joint_states`；
6. 检测执行容差；
7. 将 controller 成功或失败返回给 MoveIt。

因此 MuJoCo 中的右臂不是被 OpenClaw 直接改位置，而是在 MoveIt 生成轨迹后，由标准 ROS controller 接口驱动。

### 第十步：验证 MoveIt 确实经过

worker 记录 `ArmMotion` Feedback 阶段：

```text
VALIDATING = 1
PLANNING   = 2
EXECUTING  = 3
VERIFYING  = 4
```

即使 Result 写着 success，只要缺少 `PLANNING`、`EXECUTING` 或 `VERIFYING` 任一阶段，`openarm_moveit.py` 都不能生成 `TASK_VERIFIED`。

成功证据必须包含：

```text
simulation_result.backend = mujoco_moveit
simulation_result.planning_backend = moveit2_get_cartesian_path
simulation_result.execution_backend = moveit2_execute_trajectory
simulation_result.controller_backend = follow_joint_trajectory
simulation_result.physics_executed = true
simulation_result.direct_mujoco_fallback = false

verification_result.moveit_planning_observed = true
verification_result.trajectory_execution_observed = true
verification_result.final_pose_verification_observed = true
verification_result.final_pose_reported = true
```

### 第十一步：生成并校验 Receipt

动作证据保存在：

```text
/home/hkz/openarm_rosclaw/.rosclaw/artifacts/sandbox/<action_id>/
├── moveit_worker_result.json
├── receipt.json
└── receipt.sha256
```

OpenClaw 使用同一 Action ID 调用 `get_execution_receipt`。该工具重新计算 `receipt.json` 的 SHA256，并与 `receipt.sha256` 比较。

新 Receipt 还保存动作语义和位姿证据：

```json
{
  "normalized_arguments": {
    "arm": "right",
    "reference_frame": "world",
    "translation": {"x": 0.0, "y": 0.0, "z": 0.002}
  },
  "arguments_sha256": "...",
  "initial_pose": {"frame_id": "world", "position": {}, "orientation": {}},
  "target_pose": {"frame_id": "world", "position": {}, "orientation": {}},
  "final_pose": {"frame_id": "world", "position": {}, "orientation": {}},
  "final_error_m": 0.0004
}
```

`arguments_sha256` 对规范化后的完整参数做确定性 JSON SHA256。执行器还会验证：目标位置必须等于
初始位置加请求的 world-frame 位移，`final_error_m` 必须等于目标位置与最终位置的欧氏距离。
因此 Receipt 不只证明“发生过一次成功运动”，还能证明本次执行对应请求的 `+Z 2 mm`。

Artifact 目录和 Receipt 文件均采用独占创建。同一 Action ID 在同一 Gateway 中返回原 Receipt；
跨进程发现同名 Artifact 时明确返回 `OPENARM_ACTION_ID_ALREADY_EXISTS`，不得覆盖历史证据。
旧 Receipt 没有这些新增字段时仍按原始 JSON 读取并完成 SHA256 校验。

只有以下条件全部成立，OpenClaw 才能报告动作完成：

```text
integrity_verified = true
receipt.final_state = COMPLETED
receipt.evidence_level = TASK_VERIFIED
receipt.acknowledgement_stage = TASK_VERIFIED
receipt.verification_result.passed = true
receipt.mode = SIMULATION
receipt.simulation_result.backend = mujoco_moveit
receipt.simulation_result.physics_executed = true
receipt.simulation_result.direct_mujoco_fallback = false
receipt.arguments_sha256 = <64 个十六进制字符>
receipt.initial_pose/target_pose/final_pose = <完整 world-frame 位姿>
receipt.final_error_m = <目标位姿到最终位姿的误差>
```

Action 被接受、Action ID 被创建、MoveIt 接受轨迹或画面出现位姿变化，都不能单独证明任务成功。

## 7. 失败策略

系统按照 fail-closed 原则处理失败：

- Body、Capability 或 Runtime Skill 不匹配时不提交；
- 请求方向含糊时先询问；
- 位移超过单步限制时拆分；
- 前一步没有可信 Receipt 时不执行下一步；
- MoveIt worker 不存在或超时时返回失败；
- ROS Action Server 不可用时返回失败；
- MoveIt 路径 fraction 不足时返回失败；
- controller 失败时返回失败；
- 最终 TF 误差超过 0.5 mm 时返回失败；
- Receipt 缺失或摘要不匹配时不声明成功；
- 不自动改变方向、缩短步长、换臂或重试。

## 8. 当前能力边界

当前已经验收：

- OpenClaw 自动看到 `openarm-move-relative`；
- 官方 `rosclaw` MCP 暴露 22 个工具；
- OpenArm Runtime Skill 已注册；
- 双臂、双抓手模型加载；
- 单臂 world-frame 相对平移；
- MoveIt `GetCartesianPath` 和 `ExecuteTrajectory`；
- `FollowJointTrajectory` 到 MuJoCo；
- 最终 TF 验证；
- 带完整性校验的 ExecutionReceipt。

当前尚未验收：

- 真实机械臂 REAL 模式；
- 抓手开合或抓取 Skill；
- 双臂并发动作；
- 旋转、绝对位姿、回零；
- 视觉闭环抓取。

## 9. 验收命令

完整项目验收：

```bash
source /home/hkz/rosclaw_env/bin/activate
export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"

cd /home/hkz/openarm_rosclaw
./scripts/verify_openarm_project.sh --cutover
```

关键通过标志：

```text
46 passed
OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS
OPENCLAW OFFICIAL ROSCLAW PROJECT SKILLS: PASS
rosclaw MCP: 22 tools
OPENARM ROSCLAW PROJECT: PASS
```

MoveIt 专项验收：

```bash
cd /home/hkz/openarm_rosclaw
./scripts/verify_openarm_moveit.sh
```

只有看到：

```text
OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS
```

才证明 ROSClaw、MoveIt、ROS controller 和 MuJoCo 当前链路完整可用。
