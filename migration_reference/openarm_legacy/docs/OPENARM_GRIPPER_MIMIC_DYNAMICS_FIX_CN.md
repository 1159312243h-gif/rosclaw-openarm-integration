# OpenArm 抓手 mimic 动力学修正

该修正不会覆盖正式 MJCF。它从正式模型重新生成候选模型，并将左右抓手
mimic equality 的 `solref` 设为 `0.004 1`。

候选构建器会在输出 PASS 前自动执行以下动力学检查：

- 左右抓手分别验证；
- 主关节目标 `0.011/0.022/0.044 m`；
- 每组在禁用接触的自由空间条件下运行 5 秒；
- 主关节目标误差不超过 `0.5 mm`；
- 主从关节残差不超过 `0.5 mm`；
- 正式 MJCF SHA256 保持不变。

## 1. 应用构建器补丁

```bash
cd /home/hkz/openarm_rosclaw

cp -a scripts/build_openarm_parallel_mjcf_candidate.py \
  "scripts/build_openarm_parallel_mjcf_candidate.py.before_mimic_$(date +%Y%m%d_%H%M%S)"

unzip -o /home/hkz/openarm_gripper_mimic_dynamics_fix_v2_20260811.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x \
  scripts/build_openarm_parallel_mjcf_candidate.py \
  scripts/verify_openarm_parallel_mjcf_candidate.sh
```

## 2. 生成全新候选

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

cd /home/hkz/openarm_rosclaw

export OPENARM_CANDIDATE_DIR=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2
test ! -e "$OPENARM_CANDIDATE_DIR" || {
  echo "candidate directory already exists: $OPENARM_CANDIDATE_DIR"
  return 1 2>/dev/null || exit 1
}

./scripts/verify_openarm_parallel_mjcf_candidate.sh
```

输出必须包含：

```text
OPENARM IN-PLACE PARALLEL MJCF CANDIDATE BUILD: PASS
OPENARM FORMAL MJCF UNCHANGED: PASS
OPENARM IN-PLACE PARALLEL MJCF CANDIDATE ACCEPTANCE: PASS
```

并检查报告：

```bash
/home/hkz/rosclaw_env/bin/python -c \
'import json,sys; r=json.load(open(sys.argv[1])); print(json.dumps(r["dynamic_gripper_validation"], indent=2))' \
"$OPENARM_CANDIDATE_DIR/validation_report.json"
```

`maximum_mimic_error_m` 必须小于或等于 `0.0005`。

## 3. 使用新候选重新启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

CANDIDATE=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2/robot.mjcf.xml

ros2 launch rosclaw_openarm_bringup \
  openarm_gripper_candidate.launch.py \
  model_path:="$CANDIDATE" \
  headless:=false
```

## 4. 重新验收左右抓手

在第二个终端重新加载 ROS、虚拟环境及工作区，然后依次运行：

```bash
CANDIDATE=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2/robot.mjcf.xml

/home/hkz/rosclaw_env/bin/python \
  /home/hkz/openarm_rosclaw/scripts/test_openarm_gripper_ros_action.py \
  --expected-model-path "$CANDIDATE" \
  --side right --target 0.011 --duration 2.0 --hold 5.0

/home/hkz/rosclaw_env/bin/python \
  /home/hkz/openarm_rosclaw/scripts/test_openarm_gripper_ros_action.py \
  --expected-model-path "$CANDIDATE" \
  --side left --target 0.011 --duration 2.0 --hold 5.0
```

两次都必须输出 `OPENARM GRIPPER ROS ACTION CANDIDATE: PASS`。
