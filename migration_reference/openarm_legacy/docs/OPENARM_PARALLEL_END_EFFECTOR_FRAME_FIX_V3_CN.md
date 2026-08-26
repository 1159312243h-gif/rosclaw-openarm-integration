# OpenArm 平移夹爪末端坐标系候选修正 v3

本修正仅用于 SIMULATION 候选模型，不覆盖正式 MJCF，也不启用 REAL 模式。

## 修正依据

- 保留已经验证的 arm joint6/joint7 运动学修正。
- parallel gripper 覆盖 pinch assembly 的 `ee_mount_point.x = 0.0205 m`，安装平移设为零。
- parallel gripper 安装坐标系在 arm link7 原点使用 `Rx(pi)`。
- finger joint 源原点经过安装旋转的逆变换，使展开后的 joint anchor 与候选 MJCF 一致。
- finger collision/visual mesh 的局部原点和反射规则与正式 MJCF 中的 parallel finger 定义一致。

## 严格验收门

候选只有同时满足以下条件才通过：

- arm joint3/5/6/7 anchor 误差不超过 1 mm，axis 向量误差不超过 0.001；
- 四个 finger joint anchor 误差不超过 1 mm，axis 向量误差不超过 0.001；
- 四个 finger AABB center/extent 误差不超过 1 mm；
- URDF 与 MJCF 在左右夹爪全行程 0/11/22/33/44 mm 均不存在 finger-to-arm 关注碰撞；
- 正式 MJCF SHA256 仍为 `788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1`。

验收失败时不要启动 MoveIt，不要增加 SRDF 碰撞排除，也不要放宽误差阈值。

## 候选构建

```bash
cd /home/hkz/openarm_rosclaw
source /opt/ros/jazzy/setup.bash
source /home/hkz/rosclaw_env/bin/activate

export OPENARM_CANDIDATE_DIR=/home/hkz/openarm_parallel_end_effector_candidate_20260812_v3
./scripts/verify_openarm_parallel_mjcf_candidate.sh
```

只有输出以下三项才可继续：

```text
OPENARM CANDIDATE KINEMATIC/COLLISION ALIGNMENT: PASS
OPENARM FORMAL MJCF UNCHANGED: PASS
OPENARM IN-PLACE PARALLEL MJCF CANDIDATE ACCEPTANCE: PASS
```
