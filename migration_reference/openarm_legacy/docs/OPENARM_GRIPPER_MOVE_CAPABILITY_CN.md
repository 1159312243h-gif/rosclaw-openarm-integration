# OpenArm gripper.move 功能增量

本补丁新增正式 capability：`openarm.gripper.move`。

## 契约

```yaml
capability_id: openarm.gripper.move
execution_mode: SIMULATION
arguments:
  side: left | right
  target_m: 0.0..0.044
  duration_s: 0.25..10.0
  hold_s: 0.0..10.0
```

执行链路：ROSClaw `sandbox_run` -> 隔离 worker -> MoveIt `MoveGroup` ->
左右 gripper controller -> MuJoCo。不存在直接 MuJoCo fallback。

非法侧别、越界目标、非法时长和未知参数在 worker 调用前返回 `BLOCKED`，不创建
运动请求。worker 内部还会再次校验边界。

只有以下证据全部存在且在容差内时才返回 `TASK_VERIFIED`：

- MoveIt 规划成功；
- MoveIt 轨迹执行成功；
- driver joint 到达目标；
- mimic follower 与 driver 一致；
- 保持阶段漂移合格；
- 其他机械臂及另一侧抓手保持静止。

## 验收边界

本补丁只声明和验收 SIMULATION，不声明硬件就绪。每个运动请求只执行一次，失败后
停止，不自动重试。
