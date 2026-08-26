# OpenArm joint6/joint7 候选运动学修复

## 已确认的问题

正式 MuJoCo 模型与 v2 URDF 的 `joint6/joint7` 原点和轴不一致。该问题同时存在于
旧 pinch URDF 与 parallel 候选 URDF，并非抓手控制器或 SRDF 问题。

本补丁只修改 v2 描述源和候选验收脚本。它不会覆盖：

- `models/openarm_dual_mujoco_01/robot.urdf`
- `models/openarm_dual_mujoco_01/robot.mjcf.xml`
- `ros_ws/src/openarm_description/output.urdf`
- MoveIt SRDF

第二版按 v2 URDF 与正式 MJCF 之间既有的局部坐标基映射
`diag(1,-1,-1)` 转换 `joint6/joint7` 的原点和轴，不向末端重复加入固定旋转。

## 安装与构建

停止旧 candidate launch。该构建本身不会启动 ROS 控制器或执行动作。

```bash
cd /home/hkz/openarm_rosclaw
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="/home/hkz/openarm_arm_tail_fix_before_$STAMP.tar.gz"

tar -czf "$BACKUP" \
  ros_ws/src/openarm_description/assets/robot/openarm_v2.0/config/arm/joint/joint_origins.yaml \
  ros_ws/src/openarm_description/assets/robot/openarm_v2.0/config/arm/joint/joint_axes.yaml \
  scripts/validate_openarm_parallel_urdf_candidate.py \
  scripts/diagnose_openarm_gripper_collisions.py \
  scripts/verify_openarm_parallel_mjcf_candidate.sh

sha256sum /home/hkz/openarm_arm_tail_kinematics_candidate_fix_v2_20260812.zip

unzip -o \
  /home/hkz/openarm_arm_tail_kinematics_candidate_fix_v2_20260812.zip \
  -d /home/hkz/openarm_rosclaw

chmod +x \
  scripts/diagnose_openarm_gripper_collisions.py \
  scripts/validate_openarm_parallel_urdf_candidate.py \
  scripts/verify_openarm_parallel_mjcf_candidate.sh

source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

export OPENARM_CANDIDATE_DIR=/home/hkz/openarm_parallel_arm_tail_candidate_20260812_v2

./scripts/verify_openarm_parallel_mjcf_candidate.sh
```

## 验收边界

只有下列条件全部满足，脚本才打印最终 PASS：

- `joint1-joint5` 相对历史模型未改变；
- `joint6/joint7` 使用正式 MJCF 的 canonical 原点和轴；
- 正式 MJCF SHA256 未改变；
- URDF/MJCF 所有 14 个 arm joint 世界锚点误差不超过 `1 mm`；
- URDF/MJCF 所有 arm joint 世界轴向量误差不超过 `0.001`；
- 四个 finger collision AABB 中心与尺寸误差不超过 `1 mm`；
- 初始姿态不存在已知的 `ee_link2` 与 `link3/4/5` 接触；
- URDF 与 MJCF 对上述接触的判断一致。

如果脚本在 `OPENARM CANDIDATE KINEMATIC/COLLISION ALIGNMENT: FAIL` 停止，保留
候选目录和 `kinematic_collision_alignment.json`，不要启动 MoveIt，也不要添加 SRDF
碰撞豁免。
