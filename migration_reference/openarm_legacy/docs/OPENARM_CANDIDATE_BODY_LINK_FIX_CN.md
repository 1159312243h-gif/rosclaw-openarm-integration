# OpenArm 候选 Body 链接修复

本修复只在候选运行目录中建立 ROSClaw Body 状态，不修改候选或正式
URDF/MJCF，也不提交 `sandbox_run`。

当前候选 MCP 已激活时，只执行独立链接脚本，不要再次运行 MCP 激活脚本：

```bash
cd /home/hkz/openarm_rosclaw
source /home/hkz/rosclaw_env/bin/activate

RUNTIME=/home/hkz/openarm_parallel_candidate_runtime_20260812_v3_3/runtime.yaml

bash -n scripts/link_openarm_candidate_body.sh
./scripts/link_openarm_candidate_body.sh "$RUNTIME"

export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"
openclaw gateway restart
openclaw mcp probe rosclaw
```

随后通过 MCP 依次进行只读预检：

1. `get_body_state` 应返回 Body `openarm_dual_mujoco_01`。
2. `list_body_capabilities` 应包含 `openarm.arm.move_relative` 的
   `SIMULATION` 能力。
3. `list_skills` 应包含 `openarm/openarm-move-relative`。
4. `validate_body_action` 必须返回 `allowed_to_propose=true`。

只有四项全部通过，才可由用户另行明确要求发起一次新的 `sandbox_run`。
Capability 必须是 `openarm.arm.move_relative`，不得使用
`openarm_move_end_effector_relative`，失败后不得自动重试。
