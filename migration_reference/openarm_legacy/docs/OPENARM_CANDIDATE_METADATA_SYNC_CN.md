# OpenArm 候选模型元数据同步

候选 v3.3 的 URDF/MJCF 已通过隔离仿真验收，但正式 Body 元数据仍描述旧 pinch
夹爪。正式提升前必须先从候选 URDF 重建 `robot.eurdf.yaml` 和 `safety.yaml`。

本补丁只修改候选运行时生成流程，不覆盖正式模型目录，也不执行运动。

## 1. 安装补丁并运行聚焦测试

```bash
cd /home/hkz/openarm_rosclaw

unzip -o \
  /home/hkz/openarm_candidate_metadata_sync_fix_20260813.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x scripts/sync_openarm_candidate_metadata.py

source /home/hkz/rosclaw_env/bin/activate

python -m py_compile \
  scripts/sync_openarm_candidate_metadata.py \
  scripts/prepare_openarm_candidate_runtime.py

python -m pytest tests/sandbox/test_openarm_moveit.py -q
```

## 2. 生成全新的隔离候选运行时

不要复用先前包含旧元数据的运行时目录。

```bash
cd /home/hkz/openarm_rosclaw
source /home/hkz/rosclaw_env/bin/activate

CANDIDATE=/home/hkz/openarm_parallel_end_effector_candidate_20260812_v3_3
RUNTIME=/home/hkz/openarm_parallel_candidate_runtime_20260813_v3_3_metadata_v1

/home/hkz/rosclaw_env/bin/python \
  scripts/prepare_openarm_candidate_runtime.py \
  --candidate "$CANDIDATE" \
  --output "$RUNTIME"
```

生成器会拒绝任何不满足以下合同的候选：

- 四个 finger joint 均为 `prismatic`
- driver/follower 轴分别为 `(0, 1, 0)` 与 `(0, -1, 0)`
- 位置范围为 `0.0..0.044 m`
- follower mimic multiplier 为 `1.0`，offset 为 `0.0`
- e-URDF 与 safety 的位置/速度限位完全来自候选 URDF

## 3. 只读一致性检查

```bash
/home/hkz/rosclaw_env/bin/python \
  scripts/sync_openarm_candidate_metadata.py \
  --urdf "$RUNTIME/models/openarm_dual_mujoco_01/robot.urdf" \
  --eurdf "$RUNTIME/models/openarm_dual_mujoco_01/robot.eurdf.yaml" \
  --safety "$RUNTIME/models/openarm_dual_mujoco_01/safety.yaml" \
  --check

grep -nE \
  'finger_joint|type: prismatic|lower: 0\.0|upper: 0\.044|multiplier: 1\.0' \
  "$RUNTIME/models/openarm_dual_mujoco_01/robot.eurdf.yaml"

cat "$RUNTIME/provenance.json"
```

预期脚本输出：

```text
"status": "PASS"
"mode": "check-only"
```

在这些检查通过前，不要替换正式目录中的任何文件。即使通过，也仍需单独完成正式
提升脚本的 check-only、备份、进程占用检查、原子替换与自动回滚验证。
