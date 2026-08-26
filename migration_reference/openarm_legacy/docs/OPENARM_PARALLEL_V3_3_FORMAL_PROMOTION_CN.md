# OpenArm 平行夹爪 v3.3 正式提升

本流程仅提升已经通过 SIMULATION 验收的模型。它不执行运动，也不声明硬件就绪。
v3 同时切换正式模型目录和正式 `.rosclaw/body` Effective Body，避免正式 MCP 继续读取
旧 pinch Body 快照。

## 固定输入

- 候选运行时：`/home/hkz/openarm_parallel_candidate_runtime_20260813_v3_3_metadata_v1`
- 验收证据：`/home/hkz/openarm_parallel_v3_3_acceptance_evidence_20260813`
- 正式 Body：`/home/hkz/openarm_rosclaw/models/openarm_dual_mujoco_01`

提升器固定验证旧正式 URDF/MJCF、新候选 URDF/MJCF/e-URDF/safety、候选及正式
Effective Body，以及左右臂 Receipt 的 SHA256。任何哈希漂移都会拒绝提升。

## 1. 安装补丁并运行测试

```bash
cd /home/hkz/openarm_rosclaw

sha256sum /home/hkz/openarm_parallel_v3_3_formal_promotion_v3_20260813.zip

STAMP="$(date +%Y%m%d_%H%M%S)"
tar -czf "/home/hkz/openarm_promotion_scripts_before_$STAMP.tar.gz" \
  scripts/sync_openarm_candidate_metadata.py \
  tests/sandbox/test_openarm_moveit.py

unzip -o \
  /home/hkz/openarm_parallel_v3_3_formal_promotion_v3_20260813.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x \
  scripts/promote_openarm_parallel_v3_3.py \
  scripts/rollback_openarm_parallel_v3_3.sh

source /home/hkz/rosclaw_env/bin/activate

python -m py_compile \
  scripts/promote_openarm_parallel_v3_3.py \
  scripts/sync_openarm_candidate_metadata.py

bash -n scripts/rollback_openarm_parallel_v3_3.sh

python -m pytest \
  tests/sandbox/test_openarm_promotion.py \
  tests/sandbox/test_openarm_moveit.py \
  -q
```

## 2. Check-only

默认不传 `--apply`，因此不会修改正式模型。

```bash
cd /home/hkz/openarm_rosclaw
source /home/hkz/rosclaw_env/bin/activate

RUNTIME=/home/hkz/openarm_parallel_candidate_runtime_20260813_v3_3_metadata_v1
EVIDENCE=/home/hkz/openarm_parallel_v3_3_acceptance_evidence_20260813

/home/hkz/rosclaw_env/bin/python \
  scripts/promote_openarm_parallel_v3_3.py \
  --runtime "$RUNTIME" \
  --evidence "$EVIDENCE"
```

预期：

```text
"mode": "check-only"
"status": "PASS"
"live_processes": []
"simulation_only": true
"hardware_readiness_claimed": false
```

如果为 `BLOCKED`，先正常停止列出的 ROS/MoveIt/MuJoCo/MCP/OpenClaw Gateway
进程，然后重新执行一次 check-only。不要使用 `kill -9` 作为常规停止方法。Gateway
可使用 `openclaw gateway stop` 正常停止。

## 3. 应用提升

只在 check-only 为 PASS 后执行：

```bash
/home/hkz/rosclaw_env/bin/python \
  scripts/promote_openarm_parallel_v3_3.py \
  --runtime "$RUNTIME" \
  --evidence "$EVIDENCE" \
  --apply
```

脚本分别在同一父目录建立完整 model staging 和 Effective Body staging，通过
`renameat2(RENAME_EXCHANGE)` 交换正式模型及 `.rosclaw/body`。旧内容分别保存在输出
中的 `backup` 与 `body_backup`。任一步校验或提升记录写入失败时，脚本自动恢复两者。

## 4. 回滚检查和应用

```bash
RECORD=/home/hkz/openarm_rosclaw/.rosclaw/promotions/openarm_parallel_v3_3_latest.json

./scripts/rollback_openarm_parallel_v3_3.sh "$RECORD"
```

预期 `mode` 为 `rollback-check-only`。确认后才执行：

```bash
./scripts/rollback_openarm_parallel_v3_3.sh "$RECORD" --apply
```

回滚也使用目录原子交换，并要求相关运行进程全部停止。
