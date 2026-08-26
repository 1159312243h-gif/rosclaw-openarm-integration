# OpenArm ROSClaw 后续开发计划

> 计划基线：2026-08-07 迁移完成版本  
> 当前项目：`/home/hkz/openarm_rosclaw`  
> 当前已验证模式：`SIMULATION`  
> 最终目标：用户通过简单自然语言下达物体操作任务，系统完成感知、规划、抓取、搬运、放置和结果验证，并可从仿真迁移到真实 OpenArm。

## 1. 最终用户体验

最终用户不应需要输入：

```text
右臂沿世界坐标系 +Z 移动 2 毫米。
```

这类指令只适合作为运动能力开发、标定和故障诊断工具。

最终用户指令应接近：

```text
把桌上的红色方块放到左边托盘里。
抓起蓝色瓶子，放到指定区域。
把相机看到的零件放进收纳盒。
```

系统需要自主完成：

```text
理解任务
  -> 获取相机数据
  -> 识别目标物体和目标位置
  -> 建立三维场景
  -> 选择机械臂和抓取姿态
  -> 规划无碰撞接近路径
  -> 移动到预抓取位姿
  -> 接近物体
  -> 闭合抓手
  -> 验证抓取成功
  -> 抬升并搬运
  -> 移动到放置位姿
  -> 打开抓手
  -> 验证物体已放到目标位置
  -> 返回完整任务 Receipt
```

## 2. 总体原则

后续开发必须继续建立在官方 ROSClaw 1.0.1 框架上：

- MCP 名称保持 `rosclaw`；
- 不新增 OpenArm 专用平行 MCP；
- 优先复用官方 22 个 MCP 工具；
- Agent Skill 按官方 `SKILL.md` 规范编写；
- Runtime Skill 按官方 Skill package 模板编写；
- 动作必须经过 Runtime 和 ActionGateway；
- SIMULATION 使用官方 `sandbox_run`；
- SHADOW/REAL 使用官方 `request_action` 和 `rosclawd` 边界；
- MoveIt 负责机械臂和抓取路径规划；
- 感知、规划、执行和验证必须分层；
- 每个有副作用的阶段都必须有可审计证据；
- 仿真通过不等于真机可用；
- 未完成单独硬件验收前，不允许 REAL；
- 不引入 UR5e 等无关机器人模型；
- 保留 OpenArm 双臂和双抓手模型。

## 3. 四层目标架构

### 3.1 任务层

面向用户的高层 Agent Skill，例如候选名称：

```text
openarm-pick-place
```

它负责理解“抓什么、放到哪里”，组织完整任务，不直接计算关节角，不直接调用 ROS，也不要求用户指定毫米级动作。

### 3.2 能力层

高层 Skill 组合多个受治理 Capability。以下名称只是候选，开发前必须对照官方 ROSClaw 命名和 schema 定稿：

```text
感知目标        detect / localize object
更新场景        update planning scene
规划抓取        plan grasp candidates
移动到位姿      move to constrained pose
控制抓手        open / close gripper
验证抓取        verify object attached
规划放置        plan placement
验证放置        verify object at destination
```

这些能力应通过官方 Runtime 注册，不增加新的 MCP 工具。

### 3.3 执行层

```text
ROSClaw Runtime
  -> ActionGateway
  -> Capability + execution mode Executor
  -> ROS 2 typed Action/Service
  -> MoveIt PlanningScene / IK / trajectory
  -> ros2_control controller
  -> MuJoCo 或真实 OpenArm
  -> Observation + Verification
  -> ExecutionReceipt
```

### 3.4 硬件层

仿真和真机应尽量共享上层任务与 Capability 契约，只替换 Provider/Executor：

| 层次 | SIMULATION | REAL |
|---|---|---|
| 相机 | MuJoCo/模拟 RGB-D | 真实 RGB-D 相机 |
| TF | 仿真标定 | 实际手眼和外参标定 |
| 机械臂 | MuJoCo bridge | OpenArm hardware controller |
| 抓手 | 仿真 gripper controller | 真实抓手 controller |
| 规划 | MoveIt 2 | MoveIt 2 |
| 证据 | 仿真图像、TF、关节、接触 | 真实图像、TF、关节、力/电流 |
| 入口 | `sandbox_run` | `request_action` + `rosclawd` |

## 4. 已完成基线

### P0-A 官方框架迁移

状态：已完成。

- ROSClaw 1.0.1 已合并到唯一项目；
- Python editable 安装指向当前项目；
- OpenClaw 只注册官方 `rosclaw` MCP；
- MCP 精确暴露 22 tools；
- 旧 OpenArm MCP 和 provenance 插件已退出正式配置；
- 官方 `rosclaw` 和 `rosclaw-simforge` Agent Skills 已安装。

### P0-B OpenArm Body 与模型

状态：已完成。

- `openarm_dual_mujoco_01` Body 可加载；
- 左右七自由度机械臂和左右抓手保留；
- URDF、MJCF、Capability、Safety 和 Semantic 文件完整；
- MJCF 固定 SHA256 保持不变。

### P0-C 低层相对运动能力

状态：已完成，定位为诊断和基础动作能力。

- Agent Skill 支持自动语义选择；
- Runtime Skill 已注册；
- `openarm.arm.move_relative:SIMULATION` 已启用；
- world-frame 相对平移和 20 mm 单步限制已实现；
- MoveIt 是必经层；
- 最终 TF 验证和 Receipt 完整性已实现。

该 Skill 后续用于：

- MoveIt/TF/controller 冒烟测试；
- 标定后的微小位移验证；
- 抓取流程内部受限接近或撤退；
- 故障诊断。

它不作为最终 pick-and-place 用户界面。

### P0-D 自动化验收

状态：已完成。

```text
46 passed
OPENARM MOVEIT BLACK-BOX ACCEPTANCE: PASS
OPENCLAW OFFICIAL ROSCLAW PROJECT SKILLS: PASS
rosclaw MCP: 22 tools
OPENARM ROSCLAW PROJECT: PASS
```

## 5. 当前立即收尾

### P0-E 无 Skill 名称黑盒验收

状态：待执行一次最终人工验收。

使用当前 2 mm 指令验证自动路由、MoveIt、MuJoCo viewer 和 Receipt。这不是最终产品交互，只是当前基础能力的封板测试。

### P0-F 历史目录清理和最终归档

状态：等待 P0-E 完成。

保留：

```text
/home/hkz/openarm_rosclaw
/home/hkz/rosclaw_env
/home/hkz/.openclaw
/home/hkz/.rosclaw
/home/hkz/.nvm
```

完成最新源码归档后，再清理旧 `backup`、`before`、`stage`、`partial`、`archive`、`gitlab` 和 `rosclaw_official`。

## 6. P1：抓手基础能力

在开发完整抓取任务前，先建立可独立验证的抓手能力。

### P1-A 抓手契约

确定：

- 左抓手或右抓手；
- 打开、关闭或目标开合宽度；
- 最大速度、力或电流限制；
- 目标位置容差；
- 空抓、夹不到、物体滑落和过载判定；
- SIMULATION 与 REAL 的共同字段。

### P1-B Agent Skill 与 Runtime Skill

新增官方格式的抓手 Agent Skill 和 Runtime Skill。Agent 必须在缺少左右侧、目标对象或动作含糊时询问，不得自行加力或重复闭合。

### P1-C ROS 执行链

优先使用类型化 ROS Action 和标准 gripper/trajectory controller：

```text
ActionGateway
  -> Gripper Executor
  -> ROS typed Action
  -> gripper controller
  -> MuJoCo gripper joints
  -> position/contact verification
  -> Receipt
```

不得从 Agent 或 MCP 直接修改 MuJoCo `qpos`。

### P1-D 验收

覆盖左右抓手、合法开合、越界阻止、空抓、超时、取消、重复 Action ID、Receipt 完整性和自动 Skill 选择。

## 7. P2：相机与三维感知

### P2-A 选择和接入相机

优先使用 RGB-D 相机。需要明确仿真相机和最终真实相机型号，并在 ROS 2 中提供：

```text
RGB image
Depth image
CameraInfo
PointCloud2（如需要）
camera frame TF
```

### P2-B 相机标定

必须完成：

- 相机内参；
- 深度尺度；
- 相机到 world/base 的外参；
- 手眼标定（如果相机装在末端）；
- 时间同步；
- TF 连通性；
- 仿真和真机 frame 命名统一。

没有可信标定时，只能报告检测结果，不能执行抓取。

### P2-C 目标识别

感知 Provider 至少返回：

```text
object_id
class/name
confidence
pose
frame_id
timestamp
dimensions
segmentation or bounding region
```

需要区分：

- 仅二维检测；
- 三维位置估计；
- 六自由度姿态估计；
- 多个同类物体消歧；
- 目标是否仍然可见。

用户说“红色方块”而画面中有两个红色方块时，Agent 必须询问或使用明确的空间描述消歧。

### P2-D 感知验证

相机能力先作为只读 Capability 验收，确保不会因为检测模型误报而立即触发运动。

## 8. P3：MoveIt 场景和抓取规划

### P3-A PlanningScene

将桌面、托盘、障碍物和目标物体写入 MoveIt PlanningScene：

- 物体位置与尺寸；
- 支撑面；
- 障碍物；
- 允许碰撞矩阵；
- 抓取后 attached collision object；
- 放置后 detach 和场景更新。

### P3-B 抓取候选

生成多个候选抓取姿态，并按以下条件筛选：

- 抓手几何兼容；
- IK 可解；
- 接近路径无碰撞；
- 抓取后可抬升；
- 从抓取位姿到放置位姿存在路径；
- 距离关节限位和奇异位姿有裕量。

不要只规划“能抓到”，还要规划“抓到后能搬走并放下”。

### P3-C 分段动作

标准抓取序列：

```text
预抓取位姿
  -> 直线接近
  -> 关闭抓手
  -> 验证抓取
  -> 直线抬升
```

标准放置序列：

```text
预放置位姿
  -> 直线下降
  -> 打开抓手
  -> 验证释放
  -> 直线撤退
```

每个关键阶段失败后停止，不自动换目标、加大力或重新抓取。

## 9. P4：高层 pick-and-place Skill

### P4-A Agent Skill

新增面向用户意图的 `openarm-pick-place` Agent Skill。description 应覆盖中英文抓取、拿起、搬运、放入、放到、pick、place 等语义。

它负责：

- 提取目标物体；
- 提取目标区域；
- 询问歧义；
- 选择感知、规划、抓取和放置能力；
- 严格按照 Receipt 推进状态机；
- 汇总最终任务结果。

### P4-B Runtime Skill

创建完整 Runtime Skill package，声明：

- 所需相机 Provider；
- 所需机械臂和抓手 Capability；
- Body 兼容性；
- 感知置信度门槛；
- 工作空间和障碍约束；
- 抓取、搬运、放置和恢复策略；
- 任务级证据要求。

### P4-C 任务状态机

建议状态：

```text
TASK_RECEIVED
PERCEPTION_READY
TARGET_LOCALIZED
SCENE_READY
GRASP_PLANNED
PREGRASP_REACHED
GRASP_EXECUTED
GRASP_VERIFIED
PLACE_PLANNED
PLACE_EXECUTED
PLACE_VERIFIED
TASK_VERIFIED
```

只有前一状态有可信证据，才能进入下一状态。

### P4-D 任务级 Receipt

除单动作 Receipt 外，还需要任务级汇总：

- 原始用户目标；
- 目标物体和目标位置；
- 使用的 perception snapshot；
- 每一步 Action ID；
- 抓取前后图像或点云证据；
- 规划和碰撞检查结果；
- 抓取验证；
- 放置验证；
- 最终状态和失败阶段。

## 10. P5：MuJoCo 完整闭环验收

在真机前，MuJoCo 必须覆盖完整任务而不只是机械臂空载移动：

1. 场景中生成物体、托盘和障碍物；
2. 模拟 RGB-D 相机发布数据；
3. 从相机数据识别目标，不读取隐藏的仿真真值作为正式结果；
4. MoveIt PlanningScene 与 MuJoCo 场景一致；
5. 规划并执行抓取；
6. 验证物体随抓手移动；
7. 规划并执行放置；
8. 验证物体位于目标区域；
9. 生成任务级 `TASK_VERIFIED` Receipt；
10. 验证失败场景不会继续运动。

需要建立场景矩阵：

- 单物体；
- 多个同类物体；
- 部分遮挡；
- 目标不可达；
- 抓取姿态不可行；
- 路径存在障碍；
- 抓取后滑落；
- 放置区域被占用；
- 相机数据过期；
- controller 或 MoveIt 超时。

## 11. P6：SHADOW 和真机准备

REAL 不是把 `SIMULATION` 字段改名。先完成 SHADOW：

```text
同一用户任务
  -> 真实相机读取
  -> 真实场景建模
  -> MoveIt 规划
  -> 不发送硬件动作
  -> 输出预测轨迹、碰撞检查和风险报告
```

SHADOW 通过后才能准备 REAL。

真机前置项：

1. 真实 OpenArm ROS 2 hardware interface；
2. 真实左右臂 controller；
3. 真实抓手 controller；
4. 真实相机驱动；
5. 相机内外参和手眼标定；
6. 机械臂零位和关节限位标定；
7. MoveIt collision geometry 校验；
8. 工作台、托盘和安全区测量；
9. 急停、watchdog 和通信丢失停止；
10. 低速、低加速度和受限工作空间；
11. 操作者授权和任务确认；
12. 真实反馈和 EvidenceDomain；
13. 失败后的安全撤退或保持策略；
14. 真实任务日志和 Receipt。

## 12. P7：REAL 分阶段启用

### P7-A 无物体空载动作

先验证单臂低速、短距离、无障碍 MoveIt 轨迹和最终 TF。

### P7-B 抓手空载

验证抓手开合、力/电流限制、超时和急停。

### P7-C 固定已知物体

使用固定位置、已知尺寸、软质物体，操作者现场确认后执行抓取。

### P7-D 相机闭环抓取

再启用相机识别位置驱动抓取，仍限制工作空间、速度和目标类别。

### P7-E 自然语言任务

最后才允许用户通过 OpenClaw 下达“抓取某物并放到某处”的 REAL 任务。

每一阶段都需要独立验收，不允许跨阶段直接启用。

## 13. 安全和身份治理

REAL 必须使用官方 `request_action` 和 `rosclawd`：

- AuthorizationContext 和 Capability scope；
- 独占资源 Lease；
- emergency stop latch；
- watchdog；
- action deadline；
- 真实 Body snapshot；
- 操作者身份和审批记录；
- Action ID 由服务端生成，模型可见工具不接受自定义 ID；
- 重复 Action ID 在内存中返回原结果，跨进程同名 Artifact 明确拒绝且不覆盖；
- 重放和过期请求防护；
- 可追溯 Receipt，包括规范化参数摘要和初始/目标/最终位姿证据。

Provenance 或请求身份签名如果重新需要，必须作为官方边界扩展接入，不能恢复旧的平行 MCP 和插件体系。

## 14. Memory 与 Practice

动作经验应在核心抓取闭环稳定后接入官方 Practice/Memory：

- 成功任务形成可查询 Episode；
- 失败任务记录失败阶段和证据；
- 可复用物体抓取候选和放置策略；
- Memory 只提供建议，不绕过感知置信度、安全、ActionGateway 或 MoveIt；
- 查询历史结果不得重新执行动作；
- 自动恢复必须由用户确认并生成新 Action ID。

不恢复旧的 `rosclaw-openarm-memory` 专用 MCP。

## 15. 每个新 Skill 的完成标准

一个新 Skill 只有同时满足以下条件才算完成：

1. Agent Skill 符合官方格式并能自动选中；
2. Runtime Skill schema、兼容性和完整性检查通过；
3. Body Capability 已注册；
4. 只使用官方 `rosclaw` MCP；
5. Executor 通过 ActionGateway 注册；
6. 感知、规划、执行、验证职责清晰；
7. ROS/MoveIt/controller/MuJoCo 或真实硬件边界清晰；
8. 成功、失败、超时、取消、重复和歧义测试齐全；
9. Receipt 达到要求的 EvidenceLevel；
10. 不存在未声明 fallback；
11. 自动化测试和 OpenClaw 黑盒测试均通过；
12. SIMULATION、SHADOW、REAL 的能力声明与真实验收一致；
13. README、执行流程文档和本 Plan 已同步更新。

## 16. 建议开发顺序

```text
当前 2 mm 基础能力封板
  -> 抓手基础能力
  -> 仿真 RGB-D 相机
  -> 目标识别和三维定位
  -> MoveIt PlanningScene
  -> 抓取候选和放置规划
  -> openarm-pick-place 高层 Skill
  -> MuJoCo 完整抓放闭环
  -> 真实相机与标定
  -> SHADOW
  -> 真机空载动作
  -> 真机抓手
  -> 固定物体抓取
  -> 相机闭环抓取
  -> OpenClaw 自然语言 REAL 任务
```

下一项正式开发工作建议是：先确定目标相机与抓手控制接口，同时实现抓手 SIMULATION Capability。这样可以在不改动官方 MCP 边界的情况下，为完整 pick-and-place Skill 建立必要的执行基础。
