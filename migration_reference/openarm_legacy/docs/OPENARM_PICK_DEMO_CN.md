# OpenArm Pick Demo v8（仅仿真）

`openarm.pick_demo` 是固定右抓手、固定物块的任务级仿真能力。它不修改正式的
`robot.urdf` 或 `robot.mjcf.xml`，只使用派生场景 `robot.pick_demo.mjcf.xml`。

## v8 修复

- 修正 finger joint 语义：`0.000 m` 为完全张开，数值增大时两指向中心闭合。
- 将演示物块夹持方向宽度从 `52 mm` 调整为 `40 mm`，确保完全张开时无初始碰撞。
- 安装前同时执行静置稳定性门槛和抓手闭合可达性门槛。

## v7 支撑修复

- 被动托盘底面扩大为 `80 mm × 80 mm`，完整覆盖 `30 mm × 52 mm` 物块底面，
  避免物块在窄支撑上倾斜下沉。
- 四条低矮限位边与物块之间增加 `2 mm` 初始净空，消除零间隙多点接触导致的
  求解器过约束；限位边仍仅高 `2 mm`，不会形成 attachment 或阻止向上抓取。

## v6 抓手几何修复

- 根据离线碰撞扫描确定 joint 语义：数值越小抓手越张开，数值越大抓手越闭合。
- `gripper_open` 和初始值统一为 `0.000 m`。
- 演示物块沿夹持方向宽 `0.040 m`，完全张开时具有明确碰撞净空。
- `gripper_close` 固定为 `0.006 m`，对应物块表面的轻微预压位置。
- 离线稳定性门槛除位置漂移外，还明确拒绝物块与任何机器人碰撞体接触；安装前
  仍需通过 5 秒稳定期加 60 秒静态 MuJoCo 仿真。

## 场景约束

- 派生场景使用 `0 0 -9.81 m/s²` 重力，并在物块下方增加带四条低矮限位边的
  被动物理托盘，避免等待期间漂移或从支撑台滑落。
- 物块保持六自由度动态关节，只增加 `0.2` 被动物理阻尼以耗散微小初始扰动。
- 安装 `--check-only` 会在临时目录生成场景并执行 5 秒稳定期加 60 秒静态 MuJoCo
  仿真；位置偏差或后续漂移超限时，在修改项目文件前拒绝安装。
- `scene_ready` 在任何运动前连续采样物块位姿；初始位置偏差超过 `0.010 m`，或
  `0.25 s` 内漂移超过 `0.001 m`，立即拒绝且不执行后续阶段。
- pick 专用启动参数固定为 `position_gain=0.25`、`goal_tolerance=0.003`、
  `goal_time_margin=7.0`。精度要求没有放宽。

## 任务边界

- 仅支持 `SIMULATION`。
- 仅支持右抓手和 `pick_demo_block`。
- 一次 MCP `sandbox_run` 对应一份任务级 Receipt。
- 失败后不自动重试；任一阶段失败即停止。
- 所有机械臂和抓手运动仍通过 MoveIt、FollowJointTrajectory 和 MuJoCo。
- 禁止直接修改 MuJoCo 状态，禁止焊接、吸附或虚拟 attachment。
- 只有观测到物块高度至少增加 `0.008 m` 才能产生 `TASK_VERIFIED`。

内部阶段依次为：场景检查、抓手张开、沿 world +Z 接近 5 mm、抓手闭合、
沿 world +Z 抬升、物块位姿验证。

## 单次验收

```text
这是用户明确授权的一次新的 OpenArm pick_demo 单次仿真验收。

仅在 SIMULATION 模式下执行。
严格只调用一次 sandbox_run。
禁止调用 request_action。
禁止调用其他工具。
禁止自动重试；失败后立即停止。

sandbox_run 参数：
- capability_id: openarm.pick_demo
- arguments:
  - side: right
  - object_id: pick_demo_block
  - lift_m: 0.015

只报告 action_id、final_state、evidence_level、acknowledgement_stage、各阶段结果、
initial_object_z_m、final_object_z_m、object_rise_m、verification_result 和 errors。
```
