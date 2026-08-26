# OpenArm 抓手碰撞模型对照诊断

## 目的

MoveIt 返回 `START_STATE_IN_COLLISION (-10)`，当前已知接触为：

- `openarm_right_ee_link2` 与 `openarm_right_link3`
- `openarm_right_ee_link2` 与 `openarm_right_link4`
- `openarm_right_ee_link2` 与 `openarm_right_link5`

本诊断在完全相同的 `/joint_states` 下比较候选 URDF 与候选 MJCF。它不发送
控制指令、不调用 action server、不修改模型，适用于当前 SIMULATION 候选环境。

## 安装

在 Windows PowerShell 中上传 ZIP：

```powershell
scp C:\Users\kezhengh\Documents\Codex\2026-07-21\new-chat-2\openarm_gripper_collision_diagnostic_v2_20260812.zip hkz@192.168.222.128:/home/hkz/
```

在 VM 新终端中解压：

```bash
sha256sum /home/hkz/openarm_gripper_collision_diagnostic_v2_20260812.zip

unzip -o \
  /home/hkz/openarm_gripper_collision_diagnostic_v2_20260812.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x \
  /home/hkz/openarm_rosclaw/scripts/diagnose_openarm_gripper_collisions.py
```

## 运行

保持 `openarm_moveit_gripper_candidate.launch.py` 所在终端继续运行。在另一个 VM
终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate
source /home/hkz/openarm_rosclaw/ros_ws/install/setup.bash

PROJECT=/home/hkz/openarm_rosclaw
CANDIDATE=/home/hkz/openarm_parallel_mjcf_mimic_candidate_20260811_v2
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$PROJECT/.rosclaw/artifacts/validation/gripper_collision_diagnostic_$STAMP.json"

/home/hkz/rosclaw_env/bin/python \
  "$PROJECT/scripts/diagnose_openarm_gripper_collisions.py" \
  --urdf "$CANDIDATE/robot.urdf" \
  --mjcf "$CANDIDATE/robot.mjcf.xml" \
  --description-root "$PROJECT/ros_ws/src/openarm_description" \
  --output "$REPORT"

echo "REPORT=$REPORT"
```

脚本会订阅一次完整的 `/joint_states`，然后在独立的 MuJoCo 实例中完成全部扫描。
它不会让机械臂或抓手运动。

## 输出判读

终端会打印四个布尔提示：

- `model_frames_agree=true`：URDF/MJCF 关注链节的相对 AABB 基本一致；
- `right_focus_contacts_agree=true`：两模型对三组目标接触的判断一致；
- `right_focus_contacts_present_in_urdf=true`：URDF 确实存在目标接触；
- `right_focus_contacts_present_in_mjcf=true`：关闭 bridge 同臂屏蔽后，MJCF 也存在目标接触。

查看精简结果：

```bash
/home/hkz/rosclaw_env/bin/python - "$REPORT" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
print(json.dumps({
    "decision_hints": data["decision_hints"],
    "right_focus_pairs": data["current_state"]["right_focus_pairs"],
    "aabb_agreement": data["current_state"]["aabb_agreement"],
    "joint_frame_agreement": data["current_state"]["joint_frame_agreement"],
    "zero_arm_pose_symmetry": data["zero_arm_pose_symmetry"],
}, indent=2, ensure_ascii=False))
PY
```

不要在取得该报告前修改 SRDF 的 `disable_collisions`。如果两模型不一致，应先修复
候选几何；如果两模型一致，还需结合开臂姿态扫描与实物结构确认，才能决定是否加入
三条精确的 Allowed Collision Matrix 条目。
