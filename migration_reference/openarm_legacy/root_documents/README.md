# OpenArm ROSClaw

OpenArm ROSClaw is a focused OpenArm project built from the official ROSClaw
1.0.1 runtime at commit `ef26afd327637116f4a81e0ecb7ea582660956b8`.
It keeps the official execution framework and integrations while replacing the
robot-specific distribution content with the OpenArm model and motion Skill.

## Included official ROSClaw components

- Complete `src/rosclaw` runtime, including Body, Capability, Sandbox,
  ActionGateway, Receipt, Memory, Practice, Auto, Darwin and the official MCP.
- Official 22-tool MCP server under `src/rosclaw/mcp`.
- Official Agent Skills: `rosclaw` and `rosclaw-simforge`.
- Official Runtime built-in Skills: `realsense_capture_rgbd` and
  `scene_risk_scan` under `src/rosclaw/skill/builtins`.
- Official Runtime plugins and handlers under `src/rosclaw/runtime`.
- Official RH56 LeRobot worker plugin under `worker_plugins/`.
- Official MCP and OpenClaw integration documents under `docs/`.

## OpenArm components

- `models/openarm_dual_mujoco_01/`: OpenArm URDF, unchanged MJCF, bundled
  meshes, Body metadata, capabilities and safety limits.
- `.agents/skills/openarm-move-relative/`: automatic natural-language routing
  for bounded left/right arm motion.
- `skills/openarm-move-relative/`: ROSClaw Runtime Skill package.
- `src/rosclaw/sandbox/openarm_moveit.py`: official Sandbox executor that uses
  an isolated ROS Action worker and requires MoveIt planning/execution evidence.
- `ros_ws/`: active ROS 2 Jazzy, MoveIt 2, ArmMotion server and MuJoCo
  FollowJointTrajectory bridge used by the OpenArm simulation capability.
- `runtime.yaml`: simulation-only OpenArm Runtime profile.
- `tests/`: selected official MCP/Sandbox regression tests and OpenArm
  acceptance tests.

## Deliberately excluded

- Official UR5e, G1, Go2 and other unrelated robot models.
- Official product demos, tutorials, benchmarks and experiments for unrelated
  robots.
- Legacy `rosclaw-openarm`, memory/provenance MCP servers, custom daemon and
  OpenClaw provenance plugin.

The ROS workspace under `ros_ws/` is required by the current OpenArm execution
path. The Runtime fails closed when its MoveIt worker or ROS Action server is
unavailable; there is no direct-MuJoCo fallback.

## Runtime path

```text
OpenClaw
  -> automatic Agent Skill selection
  -> official rosclaw 22-tool MCP
  -> validate_body_action
  -> sandbox_run
  -> isolated ArmMotion worker
  -> MoveIt GetCartesianPath / ExecuteTrajectory
  -> MuJoCo FollowJointTrajectory bridge
  -> integrity-checked ExecutionReceipt
```

Detailed Chinese walkthrough of the required MoveIt execution path:

- [OpenClaw 调度 OpenArm 相对运动的当前实现](docs/OPENCLAW_OPENARM_EXECUTION_FLOW_CN.md)

Run the complete simulation verification and switch OpenClaw to this project:

```bash
source /home/hkz/rosclaw_env/bin/activate
cd /home/hkz/openarm_rosclaw
chmod +x scripts/*.sh
./scripts/verify_openarm_project.sh --cutover
```

This project supports `SIMULATION` only. It does not authorize REAL robot
motion.
