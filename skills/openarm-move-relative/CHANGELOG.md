# Changelog

## [0.2.0] - 2026-08-07

### Changed
- Made MoveIt Cartesian planning and ExecuteTrajectory mandatory for the
  OpenArm simulation capability.
- Added an isolated ROS Action worker and removed the Runtime's direct-MuJoCo
  execution fallback.
- Required MoveIt planning, controller execution and final-pose evidence in
  every successful Receipt.

## [0.1.0] - 2026-08-06

### Added
- Official ROSClaw Agent Skill, Runtime Skill, e-URDF profile, MuJoCo executor,
  acceptance test, and integrity-checked Receipt workflow.

### Changed
- Removed legacy MCP, memory-retry, and direct Agent-to-ROS assumptions from
  the simulation contract.
