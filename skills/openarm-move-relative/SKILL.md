# OpenArm bounded relative motion

## Skill ID

`openarm/openarm-move-relative`

## Intent

Move one selected OpenArm end effector by a bounded translation in the `world`
frame. This is a MoveIt-backed simulation ROSClaw Runtime skill, not a direct
ROS or OpenClaw tool implementation.

## Preconditions

- Effective Body is `openarm_dual_mujoco_01`.
- Capability `openarm.arm.move_relative` is available.
- Arm is exactly `left` or `right`.
- Translation is finite, non-zero, expressed in metres, and has norm at most
  `0.02` metres per Action.
- Runtime and the OpenArm MoveIt worker are healthy.
- `/rosclaw/arm_motion`, MoveIt Cartesian planning, ExecuteTrajectory and the
  MuJoCo FollowJointTrajectory bridge are available.

## Effects

- A terminal `ExecutionReceipt` exists.
- Final state is `COMPLETED`.
- Evidence and acknowledgement stages are `TASK_VERIFIED`.
- Verification passed in the `SIMULATION` evidence domain.

## Runtime Contract

- Input: arm, world-frame translation, velocity scale and acceleration scale.
- Output: trace, runtime events and integrity-checked ExecutionReceipt.
- Official MCP tool: `sandbox_run` with `capability_id` and `arguments`; do not
  pass `joint_positions` or a client-generated `action_id` for this capability.
  ROSClaw generates the Action ID and the caller uses the returned ID for the
  exact `get_execution_receipt` lookup.
- Runtime route: official MCP -> RuntimeClient -> ActionGateway ->
  `SandboxTaskExecutorRegistry` -> isolated ROS Action worker -> MoveIt
  `GetCartesianPath` -> `ExecuteTrajectory` -> `FollowJointTrajectory` ->
  OpenArm MuJoCo bridge.
- The executor requires planning, execution and final-pose verification
  feedback plus a finite final end-effector pose. It never falls back to direct
  MuJoCo differential IK.

## Safety Envelope

- `sandbox_first`; never claim REAL execution.
- Validate before dispatch and stop after any failed or uncertain step.
- Never publish ROS topics or invoke MoveIt/MuJoCo directly from an Agent;
  use only the Runtime-owned isolated worker.
- Use only the official ROSClaw MCP and Runtime boundary.

## Evidence

- See `evidence/reports/` after `rosclaw skill eval` has run in the official
  ROSClaw environment.
