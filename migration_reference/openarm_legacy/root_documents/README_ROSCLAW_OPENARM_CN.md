# OpenArm ROSClaw 自主机器人闭环集成

> 当前阶段：官方 ROSClaw 22-tool MCP 到 OpenArm MoveIt/MuJoCo 基础运动闭环已打通  
> 文档日期：2026-08-07  
> ROS 2：Jazzy  
> ROSClaw：1.0.1，基于官方提交 `ef26afd327637116f4a81e0ecb7ea582660956b8`  
> OpenClaw：2026.6.11 (`e085fa1`)  
> 当前执行模式：`SIMULATION`  
> 正式项目目录：`/home/hkz/openarm_rosclaw`

## 1. 项目目标

本项目将 OpenClaw、官方 ROSClaw、ROS 2、MoveIt 2、MuJoCo 和 OpenArm 双臂机器人组合成可治理、可观察、可验证并可逐步迁移到真实硬件的机器人智能体平台。

项目最终目标不是让用户输入毫米级机械臂位移，而是允许用户提出简单任务：

```text
把桌上的红色方块放到左边托盘里。
抓起蓝色瓶子并放进收纳盒。
把相机看到的零件移动到指定区域。
```

目标自主闭环为：

```text
用户自然语言任务
  -> OpenClaw 自动选择高层 Agent Skill
  -> Agent 识别目标物体和目标位置
  -> ROSClaw 官方 MCP 读取 Body、Capability 和运行状态
  -> 相机采集 RGB-D 数据
  -> 目标识别、三维定位和歧义消解
  -> MoveIt PlanningScene 建模
  -> 抓取候选、IK 和无碰撞路径规划
  -> ROSClaw Runtime / ActionGateway 分阶段执行
  -> 机械臂接近和抓手闭合
  -> 抓取结果验证
  -> 搬运、放置和撤退
  -> 相机与机器人状态验证放置结果
  -> 任务级 ExecutionReceipt
  -> Agent 根据证据报告完成、停止或请求用户处理
  -> Practice / Memory 保存可复用经验
```

当前已完成的是这条长期链路的底座：OpenClaw Skill 发现、官方 MCP、Body/Capability、ActionGateway、MoveIt、ROS Action、MuJoCo、最终 TF 验证和 Receipt。

当前的 `openarm-move-relative` 是低层运动能力和诊断工具，不是最终 pick-and-place 用户界面。

## 2. 当前实现状态

| 模块 | 当前状态 | 说明 |
|---|---|---|
| 官方 ROSClaw 1.0.1 框架 | 已完成 | 已合并到唯一 OpenArm 项目，不再依赖旧项目目录运行 |
| Python 运行来源 | 已完成 | editable install 指向 `/home/hkz/openarm_rosclaw/src/rosclaw` |
| 官方 ROSClaw MCP | 已完成 | 名称保持 `rosclaw`，精确暴露 22 tools |
| OpenClaw Gateway | 已完成 | systemd user service 正常，connectivity probe 为 ok |
| 官方 Agent Skills | 已完成 | `rosclaw`、`rosclaw-simforge` 已安装并可见 |
| OpenArm Agent Skill | 已完成 | `openarm-move-relative` 已安装并可见，可按语义自动选择 |
| OpenArm Runtime Skill | 已完成 | `openarm/openarm-move-relative` 已注册 |
| OpenArm Body | 已完成 | `openarm_dual_mujoco_01` 能生成 Effective Body |
| OpenArm 双臂和抓手模型 | 已完成 | URDF、MJCF、mesh 和左右抓手均保留 |
| `/rosclaw/arm_motion` | 已完成 | 类型化 ROS 2 Action，当前正式实现相对平移 |
| MoveIt Cartesian Path | 已完成 | 使用 `GetCartesianPath`，拒绝不完整路径 |
| MoveIt ExecuteTrajectory | 已完成 | 通过标准 MoveIt execution action 执行 |
| FollowJointTrajectory | 已完成 | 左右臂 controller 已连接 MuJoCo bridge |
| MuJoCo 物理执行 | 已完成 | 轨迹推进、joint_states 和 TF 正常 |
| 最终位置验证 | 已完成 | 最终 world-frame TF 误差门限为 0.5 mm |
| ActionGateway Receipt | 已完成 | 成功和失败均生成结构化 Receipt |
| Receipt 完整性 | 已完成 | `receipt.json` 和 `receipt.sha256` 独立验证 |
| 自动化测试 | 已完成 | 46 tests 和 MoveIt 黑盒验收通过 |
| 无 Skill 名称 OpenClaw 最终演示 | 待最后人工确认 | 需要带 MuJoCo viewer 完成一次展示并保存证据 |
| 抓手控制 Capability | 未实现 | 模型存在，正式抓手 Action/Skill 尚未开发 |
| RGB-D 相机 | 未实现 | 仿真和真实相机型号、接口、标定尚待确定 |
| 目标识别与三维定位 | 未实现 | 后续感知 Provider/Skill |
| pick-and-place 高层 Skill | 未实现 | 后续主要用户入口 |
| SHADOW | 未实现 | 真机前必须先完成 |
| REAL 硬件执行 | 未实现 | 当前禁止声称真机已经打通 |

最终验收结果：

```text
46 passed
OFFICIAL ROSCLAW OPENARM SANDBOX ACCEPTANCE: PASS
OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS
OPENCLAW OFFICIAL ROSCLAW PROJECT SKILLS: PASS
rosclaw MCP: 22 tools
Gateway connectivity probe: ok
OPENARM ROSCLAW PROJECT: PASS
```

## 3. 当前正式数据链

### 3.1 OpenClaw 工具发现链

```text
OpenClaw Agent
  -> /home/hkz/.openclaw/workspace/skills
  -> 自动看到 rosclaw / rosclaw-simforge / openarm-move-relative
  -> /home/hkz/.openclaw/openclaw.json
  -> 名为 rosclaw 的官方 stdio MCP
  -> /home/hkz/rosclaw_env/bin/python
  -> rosclaw.entrypoint mcp serve
  -> 官方 22-tool boundary
```

OpenClaw 原生 MCP 配置不依赖 `PYTHONPATH`，Python editable install 已直接指向当前项目。

### 3.2 SIMULATION 动作链

```text
OpenClaw
  -> get_body_state
  -> list_body_capabilities
  -> list_skills
  -> validate_body_action
  -> sandbox_run
  -> RuntimeClient.sandbox_run
  -> ActionEnvelope(execution_mode=SIMULATION)
  -> Runtime.submit_action
  -> ActionGateway.submit
  -> Body 独占资源 Lease
  -> SandboxRuntimeAdapter.execute_action
  -> run_openarm_move_relative_via_moveit
  -> 隔离 ROS Action worker
  -> /rosclaw/arm_motion
  -> MoveIt GetCartesianPath
  -> MoveIt ExecuteTrajectory
  -> FollowJointTrajectory
  -> mujoco_moveit_bridge
  -> MuJoCo 物理推进
  -> 最终 TF 验证
  -> ExecutionReceipt
  -> get_execution_receipt 完整性校验
```

当前 SIMULATION 路径使用进程内官方 Runtime 和 ActionGateway，不经过旧的独立 `openarm_rosclawd`。

官方 `request_action` 主要用于未来 SHADOW/REAL daemon 边界。当前仿真使用官方 `sandbox_run`。

### 3.3 ROS 与 MoveIt 链

```text
ArmMotion.Goal
  -> /rosclaw/arm_motion
  -> relative_motion_server.py
  -> 当前末端 TF
  -> 目标位置 = 当前位置 + world translation
  -> /compute_cartesian_path
  -> 路径 fraction 和轨迹检查
  -> /execute_trajectory
  -> right/left_arm_controller/follow_joint_trajectory
  -> mujoco_moveit_bridge.py
  -> mujoco.mj_step()
  -> /joint_states
  -> robot_state_publisher / TF
  -> final pose verification
```

OpenClaw 和 Skill 不计算关节角。MoveIt 负责笛卡尔路径、逆运动学和关节轨迹。

## 4. 工程目录结构

```text
/home/hkz/openarm_rosclaw/
├── .agents/
│   └── skills/
│       ├── openarm-move-relative/
│       ├── rosclaw/
│       └── rosclaw-simforge/
├── .codex/
├── bin/
├── configs/
├── deploy/
├── docs/
│   ├── OPENCLAW_INTEGRATION.md
│   ├── OPENCLAW_OPENARM_EXECUTION_FLOW_CN.md
│   └── P0_AGENT_INTEGRATION.md
├── models/
│   └── openarm_dual_mujoco_01/
├── ros_ws/
│   └── src/
│       ├── openarm_description/
│       ├── openarm_moveit_config/
│       ├── rosclaw_interfaces/
│       ├── rosclaw_motion/
│       └── rosclaw_openarm_bringup/
├── scripts/
├── skills/
│   └── openarm-move-relative/
├── src/
│   └── rosclaw/
├── tests/
├── worker_plugins/
├── .mcp.json
├── AGENTS.md
├── CLAUDE.md
├── ROSCLAW.md
├── README.md
├── README_ROSCLAW_OPENARM_CN.md
├── OPENARM_ROSCLAW_PLAN_CN.md
├── runtime.yaml
├── pyproject.toml
└── ARCHITECTURE.md
```

### 4.1 运行环境关系

```text
/home/hkz/openarm_rosclaw
  -> 当前唯一源码和项目配置
  -> editable install 到 /home/hkz/rosclaw_env

/home/hkz/rosclaw_env
  -> Python 3.12 虚拟环境
  -> 启动 rosclaw.entrypoint MCP

/opt/ros/jazzy
  + /home/hkz/openarm_rosclaw/ros_ws/install
  -> ROS 2、MoveIt、ArmMotion 和 bringup 运行环境

/home/hkz/.openclaw
  -> OpenClaw 原生配置和 workspace Skills

/home/hkz/openarm_rosclaw/.rosclaw
  -> 当前项目 Receipt、artifact 和运行产物
```

`/home/hkz/rosclaw_official` 和旧 backup/archive 目录不再参与正式运行。

## 5. OpenArm 模型文件

```text
models/openarm_dual_mujoco_01/
├── robot.urdf
├── robot.mjcf.xml
├── robot.eurdf.yaml
├── capabilities.yaml
├── safety.yaml
├── semantic.yaml
├── benchmark.yaml
└── model.sha256
```

### 5.1 robot.urdf

供 ROS、MoveIt 和 robot_state_publisher 使用，描述双臂、抓手、关节、link、限位和 mesh。

### 5.2 robot.mjcf.xml

供 MuJoCo 使用，描述物理 body、joint、geom、actuator 和仿真初始状态。当前固定 SHA256：

```text
788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1
```

### 5.3 robot.eurdf.yaml

提供 ROSClaw Body 描述，包含 frames、joints、actuators、sensors、capabilities 和 model references。

### 5.4 capabilities.yaml

当前正式声明并验收：

```text
openarm.arm.move_relative:SIMULATION
```

抓手、绝对位姿、回零和抓取能力不能因为模型中存在相关关节就提前声明为已完成。

### 5.5 safety.yaml

保存关节限位、速度/加速度边界、单步位移限制和执行模式边界。

### 5.6 semantic.yaml

把技术模型映射为左右臂、末端、抓手、planning group 和 world frame 等语义。

## 6. ROS 2 与运动层逐文件说明

### 6.1 openarm_description

```text
ros_ws/src/openarm_description/
```

保存 OpenArm Xacro、mesh、关节、惯性和抓手描述，是 MoveIt 机器人模型来源。

### 6.2 openarm_moveit_config

```text
ros_ws/src/openarm_moveit_config/
├── config/
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── ompl_planning.yaml
│   └── moveit_controllers.yaml
├── launch/moveit_minimal.launch.py
└── srdf/
```

作用：

- 定义左右臂 MoveIt planning group；
- 配置 IK；
- 配置规划器；
- 配置 joint limits；
- 注册左右 `FollowJointTrajectory` controller；
- 启动 move_group 和执行能力。

### 6.3 rosclaw_interfaces

```text
ros_ws/src/rosclaw_interfaces/action/ArmMotion.action
```

定义类型化 Goal、Result 和 Feedback。当前 Goal 支持相对移动所需的 arm、frame、translation、速度、加速度和碰撞参数。

该包只定义接口，不执行动作。

### 6.4 rosclaw_motion

```text
ros_ws/src/rosclaw_motion/rosclaw_motion/relative_motion_server.py
```

主要职责：

1. 校验 request ID、arm、operation 和 frame；
2. 读取当前末端 TF；
3. 构造 world-frame 目标位姿；
4. 调用 MoveIt `GetCartesianPath`；
5. 拒绝不完整或异常轨迹；
6. 调用 MoveIt `ExecuteTrajectory`；
7. 传播取消和超时；
8. 重新读取最终 TF；
9. 用 0.5 mm 门限验证任务结果；
10. 返回 `ArmMotion.Result`。

### 6.5 rosclaw_openarm_bringup

```text
ros_ws/src/rosclaw_openarm_bringup/
├── config/openarm_sim.yaml
├── launch/openarm_sim.launch.py
└── scripts/
    ├── mujoco_moveit_bridge.py
    └── rosclaw_moveit_action_worker.py
```

`openarm_sim.launch.py` 是当前 ROS 仿真的统一启动入口，启动：

- MuJoCo bridge；
- MoveIt minimal bringup；
- relative motion server。

`mujoco_moveit_bridge.py` 提供左右臂 trajectory controller、推进 MuJoCo、发布 joint states 并返回 controller 结果。

`rosclaw_moveit_action_worker.py` 是隔离的一次性 ROS ActionClient。它从 stdin 读取 JSON，构造 `ArmMotion.Goal`，收集 Feedback 和 Result，再向 ROSClaw Executor 返回 JSON。

## 7. ROSClaw 核心层逐文件说明

### 7.1 MCP 工具注册

```text
src/rosclaw/mcp/server.py
src/rosclaw/mcp/tools/__init__.py
```

创建官方 FastMCP Server，并注册精确 22 个官方工具。项目没有新增第 23 个工具。

### 7.2 RuntimeClient

```text
src/rosclaw/mcp/adapters/runtime_client.py
```

`sandbox_run()` 接收 Capability 和动作参数，在服务端构造：

```text
body_id = openarm_dual_mujoco_01
capability_id = openarm.arm.move_relative
execution_mode = SIMULATION
required_evidence = TASK_VERIFIED
timeout_sec = 30
fail_closed = true
```

`get_execution_receipt()` 从 artifact 目录读取 Receipt，重新计算 SHA256 后返回。

### 7.3 Runtime

```text
src/rosclaw/core/runtime.py
```

负责：

- 加载 `runtime.yaml`；
- 初始化 Body、Skill、Sandbox 和 Provider；
- 持有 ActionGateway；
- 把 Sandbox 支持的 Capability 注册为 SIMULATION Executor；
- 接收 canonical ActionEnvelope。

### 7.4 ActionGateway

```text
src/rosclaw/kernel/action_gateway.py
```

负责：

- Action ID 幂等；
- 并发重复动作抑制；
- Capability + mode Executor 选择；
- Action deadline；
- Body 独占资源 Lease；
- 状态迁移；
- EvidenceLevel 检查；
- Receipt 生成和持久化；
- `receipt.sha256` 写入。

### 7.5 SandboxRuntimeAdapter

```text
src/rosclaw/sandbox/runtime_adapter.py
```

为当前 Body 注册：

```text
openarm.arm.move_relative
  -> run_openarm_move_relative_via_moveit()
```

### 7.6 openarm_moveit.py

```text
src/rosclaw/sandbox/openarm_moveit.py
```

正式 OpenArm SIMULATION Executor。职责：

1. 调用参数白名单校验；
2. 创建动作 artifact 目录；
3. 构造 worker JSON；
4. 启动隔离 worker；
5. 检查 Action 是否被接受；
6. 检查 Feedback 是否经过 PLANNING、EXECUTING 和 VERIFYING；
7. 检查最终 pose 是否完整；
8. 生成 `ActionExecutionResult`；
9. 明确记录 `direct_mujoco_fallback=false`。

### 7.7 openarm.py

```text
src/rosclaw/sandbox/openarm.py
```

当前正式链路复用其中的 `validate_openarm_move_relative()`，检查：

- Body、Capability 和 SIMULATION mode；
- left/right；
- world frame；
- translation 恰好包含 x/y/z；
- 数值有限且非零；
- 单分量和三维模长不超过 20 mm；
- velocity/acceleration scale 位于 `[0.01, 0.10]`；
- 未知参数 fail closed。

文件中保留的直接 MuJoCo Jacobian IK 是历史/离线参考，不是当前 Runtime 注册的正式执行后端。

## 8. 两类 Skill

### 8.1 Agent Skill

源码：

```text
.agents/skills/openarm-move-relative/SKILL.md
```

OpenClaw 安装位置：

```text
/home/hkz/.openclaw/workspace/skills/openarm-move-relative/SKILL.md
```

作用：

- 根据 description 自动匹配用户意图；
- 把毫米、厘米转换为米；
- 把“向上/抬高”映射为 world `+Z`；
- 对含糊方向进行询问；
- 把大于 20 mm 的向量拆成串行小步；
- 规定官方 MCP 工具顺序；
- 规定 Receipt 成功条件；
- 失败后停止，不自动重试或改变动作。

### 8.2 Runtime Skill

```text
skills/openarm-move-relative/
├── skill.yaml
├── behavior_tree.xml
├── safety.yaml
├── providers.yaml
├── e-urdf-compat.yaml
├── policies/
├── prompts/
├── tests/
└── .rosclaw/
```

作用：

- 声明 Skill ID；
- 声明 Body 和 Capability 兼容性；
- 声明安全和 Provider 条件；
- 提供 planner/executor/verifier/recovery 指令；
- 提供 schema 测试与完整性锁定。

Agent Skill 负责“OpenClaw 怎么理解和调用”，Runtime Skill 负责“ROSClaw 是否注册并允许该能力”。

## 9. 示例：右臂向上移动 2 mm

用户输入：

```text
仅在 SIMULATION 模式下，让右臂末端沿世界坐标系 +Z 方向移动 2 毫米。
```

OpenClaw 归一化为：

```yaml
arm: right
reference_frame: world
translation: {x: 0.0, y: 0.0, z: 0.002}
velocity_scale: 0.05
acceleration_scale: 0.05
avoid_collisions: true
execution_mode: SIMULATION
```

调用顺序：

```text
get_body_state
  -> list_body_capabilities
  -> list_skills
  -> validate_body_action
  -> sandbox_run
  -> get_execution_receipt
```

只有以下条件全部成立才报告完成：

```text
integrity_verified = true
final_state = COMPLETED
evidence_level = TASK_VERIFIED
acknowledgement_stage = TASK_VERIFIED
verification_result.passed = true
mode = SIMULATION
backend = mujoco_moveit
planning_backend = moveit2_get_cartesian_path
execution_backend = moveit2_execute_trajectory
controller_backend = follow_joint_trajectory
physics_executed = true
direct_mujoco_fallback = false
```

更完整的逐步说明见：

```text
docs/OPENCLAW_OPENARM_EXECUTION_FLOW_CN.md
```

## 10. Receipt 与运行产物

动作产物目录：

```text
/home/hkz/openarm_rosclaw/.rosclaw/artifacts/sandbox/<action_id>/
├── moveit_worker_result.json
├── receipt.json
└── receipt.sha256
```

`moveit_worker_result.json` 保存 ROS Action 反馈、初始/目标/最终位姿、最终误差和结果；其 SHA256
写入 Receipt，用于证明 Receipt 引用的 Worker 证据未发生变化。

`receipt.json` 保存 ActionGateway 的 Body、Capability、Lease、状态转换、执行证据、验证结果和最终状态，
并包含规范化动作参数、参数 SHA256、初始位姿、目标位姿、最终位姿及最终误差。

`receipt.sha256` 用于检测 Receipt 被修改或损坏。

Action ID 由 ROSClaw 服务端以 `action_<uuid>` 格式生成，模型可见的 `sandbox_run` 和
`request_action` 都不能传入该字段。同一 ID 在内存中重复提交时返回原 Receipt；跨进程发现同名
Artifact 时明确拒绝，目录和 Receipt 文件均不允许覆盖。历史 Receipt 即使没有新增审计字段，仍可读取并校验。

## 11. 已完成测试与证据

### 11.1 Python 和配置测试

测试覆盖官方 MCP/Sandbox 相关回归、ActionGateway、OpenArm 参数校验、MoveIt Executor、配置迁移和 Skill 安装。
本次 Action ID 与 Receipt 增强加入了新的自动化测试，最终通过数量以虚拟机执行
`./scripts/verify_openarm_project.sh --cutover` 的输出为准，不沿用旧版本的固定计数。

### 11.2 ROS 2 构建

以下包已通过 `colcon build --symlink-install`：

```text
rosclaw_interfaces
openarm_moveit_config
rosclaw_motion
rosclaw_openarm_bringup
```

### 11.3 MoveIt 黑盒测试

```text
OFFICIAL ROSCLAW OPENARM SANDBOX ACCEPTANCE: PASS
OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS
```

测试证明：

- Runtime profile 加载；
- Runtime Skill 注册；
- MoveIt Capability 注册；
- MoveIt planning 被观察到；
- trajectory execution 被观察到；
- MuJoCo physics 执行；
- 最终 TF 验证通过；
- Receipt 完整性通过；
- MJCF 哈希未变化。

### 11.4 OpenClaw 集成

```text
OPENCLAW OFFICIAL ROSCLAW PROJECT SKILLS: PASS
MCP probe: rosclaw, 22 tools
Gateway connectivity probe: ok
OPENARM ROSCLAW PROJECT: PASS
```

## 12. 当前可以做什么

当前系统可以：

1. 统一启动 OpenArm 双臂 MuJoCo、MoveIt 和 relative motion server；
2. 通过官方 ROSClaw MCP 读取 Body、Capability 和 Skill；
3. 通过官方 `sandbox_run` 提交受限 SIMULATION Capability；
4. 通过 ActionGateway 获得幂等、Lease、状态转换和 Receipt；
5. 让 MoveIt 为左右臂规划相对笛卡尔路径；
6. 通过 FollowJointTrajectory 驱动 MuJoCo；
7. 根据最终 TF 判断是否真正完成；
8. 拒绝越界、未知字段、错误 frame、错误 arm 和非仿真模式；
9. 验证 Receipt SHA256；
10. 让 OpenClaw 看见并自动匹配 `openarm-move-relative`。

当前系统还不能：

1. 通过相机识别物体；
2. 估计物体三维位置或六自由度姿态；
3. 自动更新 MoveIt PlanningScene；
4. 控制抓手执行正式抓取；
5. 验证物体是否抓牢或放到目标位置；
6. 执行完整 pick-and-place；
7. 执行双臂协同；
8. 执行 REAL；
9. 声称具有真机安全认证。

## 13. 启动与关闭

### 13.1 构建 ROS 工作空间

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hkz/openarm_rosclaw/ros_ws
colcon build --symlink-install --packages-up-to rosclaw_openarm_bringup
```

### 13.2 启动可视化 MuJoCo/MoveIt

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

ros2 launch rosclaw_openarm_bringup \
  openarm_sim.launch.py headless:=false
```

自动测试使用：

```text
headless:=true
```

### 13.3 检查 OpenClaw

```bash
export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"

openclaw gateway status
openclaw mcp probe rosclaw
openclaw skills check
```

### 13.4 OpenClaw 会话

在 bringup 正常后新建 OpenClaw 会话，直接输入用户目标，不主动提供 Skill 名称。

### 13.5 关闭顺序

1. 停止 OpenClaw 继续提交新动作；
2. 等待已有动作产生终态 Receipt；
3. 停止 ROS bringup；
4. 确认没有遗留 MoveIt、bridge 或 motion server 进程。

## 14. 完整验收

```bash
source /home/hkz/rosclaw_env/bin/activate
export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"

cd /home/hkz/openarm_rosclaw
chmod +x scripts/*.sh
./scripts/verify_openarm_project.sh --cutover
```

专项 MoveIt 验收：

```bash
./scripts/verify_openarm_moveit.sh
```

只有全部 PASS，才能声称当前 SIMULATION 基础运动链可用。

## 15. 外部依赖和固定版本

### 15.1 ROSClaw

```text
版本：1.0.1
上游提交：ef26afd327637116f4a81e0ecb7ea582660956b8
当前源码：/home/hkz/openarm_rosclaw/src/rosclaw
```

### 15.2 Python

```text
环境：/home/hkz/rosclaw_env
导入：/home/hkz/openarm_rosclaw/src/rosclaw/__init__.py
```

### 15.3 ROS 2

```text
发行版：Jazzy
系统环境：/opt/ros/jazzy
项目 overlay：/home/hkz/openarm_rosclaw/ros_ws/install
```

### 15.4 OpenClaw

```text
版本：2026.6.11 (e085fa1)
命令：/home/hkz/.nvm/versions/node/v24.18.0/bin/openclaw
配置：/home/hkz/.openclaw/openclaw.json
workspace：/home/hkz/.openclaw/workspace
```

## 16. 已知限制和安全边界

1. 当前正式能力只支持 `SIMULATION`；
2. 当前只实现单臂 world-frame 相对平移；
3. 单步三维位移不超过 20 mm；
4. 速度和加速度比例限制为 `[0.01, 0.10]`；
5. 最终位置误差必须不超过 0.5 mm；
6. MoveIt 是必经层，禁止直接 MuJoCo fallback；
7. MoveIt/Action Server/controller/TF 任一不可用都失败关闭；
8. 当前抓手模型存在但正式抓手 Capability 未实现；
9. 当前没有 RGB-D 感知和场景重建；
10. 当前没有完整抓放任务状态机；
11. 当前没有真机 authorization、急停、watchdog 和硬件 EvidenceDomain 验收；
12. SIMULATION Receipt 不能用于证明真实硬件安全性。

## 17. 后续技术路线

后续目标不是继续增加更多“向某方向移动若干毫米”的用户 Skill，而是建立分层的自主操作能力。

建议顺序：

```text
当前基础运动能力封板
  -> 抓手基础 Capability 和 Skill
  -> 仿真 RGB-D 相机
  -> 目标识别和三维定位
  -> MoveIt PlanningScene
  -> 抓取候选和放置规划
  -> openarm-pick-place 高层 Skill
  -> MuJoCo 完整抓放闭环
  -> 真实相机和手眼标定
  -> SHADOW
  -> 真机空载低速验收
  -> 真机抓手验收
  -> 固定物体抓取
  -> 相机闭环抓取
  -> OpenClaw 自然语言 REAL 任务
```

详细计划见：

```text
OPENARM_ROSCLAW_PLAN_CN.md
```

## 18. SIMULATION 到 REAL 的迁移原则

仿真和真机共享：

- 用户任务语义；
- Agent Skill；
- Runtime Skill；
- Body Capability 契约；
- MoveIt 规划逻辑；
- 任务状态机；
- Receipt schema。

仿真和真机分别实现：

- 相机 Provider；
- 机械臂 Executor；
- 抓手 Executor；
- controller；
- 标定数据；
- EvidenceDomain。

正式 REAL 必须经过：

```text
真实相机和标定
  -> SHADOW 规划验证
  -> 官方 request_action
  -> rosclawd
  -> AuthorizationContext
  -> Capability scope
  -> watchdog / emergency stop
  -> 真实 OpenArm Executor
  -> 真实反馈验证
  -> REAL ExecutionReceipt
```

不能把 `SIMULATION` 字段直接改成 REAL，也不能恢复旧平行 MCP 绕过官方边界。

## 19. 已淘汰实现

以下内容只作为历史材料，不再属于正式运行入口：

- `rosclaw-openarm` 五工具 MCP；
- `rosclaw-openarm-memory` MCP；
- 旧独立 `openarm_rosclawd`；
- 旧 OpenArm Goal Mapper/Adapter 体系；
- provenance probe/acceptance MCP；
- `openarm-provenance-diagnostic` OpenClaw 插件；
- 不经过 MoveIt 的直接 MuJoCo 正式后端；
- `/home/hkz/rosclaw_official` 作为运行源码；
- 多个并行 OpenArm 项目副本。

这些实现中的安全思想可以按官方扩展点重新采用，但不能恢复为平行控制体系。

## 20. 当前阶段结论

当前已经证明：OpenClaw 能发现符合官方规范的 OpenArm Skill，官方 `rosclaw` MCP 能加载 OpenArm Body 和 Capability，官方 Runtime 与 ActionGateway 能将受限 SIMULATION Action 路由到 MoveIt Executor，MoveIt 能通过标准 ROS Action 和 FollowJointTrajectory 驱动 MuJoCo，并根据最终 TF 生成完整性可验证的 `TASK_VERIFIED` Receipt。

这代表“自主机器人执行底座”已经完成，但不代表完整自主抓取或真实硬件已经完成。

下一阶段应围绕抓手、RGB-D 相机、目标三维定位、MoveIt PlanningScene、抓取/放置规划和高层 `openarm-pick-place` Skill 展开，并从项目开始就保留 SHADOW 和 REAL 的迁移边界。
