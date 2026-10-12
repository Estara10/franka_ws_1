# A1 P2a 记录：几何、双臂 IK 与组合碰撞（不运动）

对应阶段计划 `从A1开始的阶段执行器提示词.md` 中“当前执行器提示词：P2a”。准则以 `PROJECT_GUIDE.md` 为准。
**本轮未进入 P2b，未提交/推送 Git。**

## 结论摘要

- ✅ 实现并通过针对性测试与真实 MoveIt 检查：几何、双臂组合 IK、组合碰撞三部分。
- ✅ **几何**：通过。样例 footprint 在配置包络内，最小余量 **69.565 mm**。
- ✅ **双臂 IK**：通过。左右解写入**同一组合状态**，两状态 8 个腕部 FK 残差全为 **0.0000 mm / 0.0000°**。
- ❌ **组合碰撞**：**未通过**。预抓取状态干净（四类接触均为 0），但抓取状态出现
  **`aluminum_rod ↔ mj_left_hand`、`aluminum_rod ↔ mj_right_hand`** 两处接触。
- ⇒ **本轮没有得到通过全部三项的样本**。按要求**未放宽碰撞规则或容差**，如实报告拒绝原因。
- ✅ 无运动边界成立：所有 P2a 运行中 APPROACH/GRASP/双臂规划/夹爪/weld/attach 计数均为 0。

## 一、更正上一轮的一处错误陈述

上一轮（P1b 修复轮）我报告 SRDF“**0 个跨臂条目、0 个 base_link 条目**”。**该说法是错的。**

实际 `franka_moveit_config/srdf/dual_panda_generated.srdf`（launch 在 `sim_dual_moveit.launch.py:118-124`
直接读取该文件）共 **121** 条 `disable_collisions`，其中**跨臂 41 条**，含大面积“躯干可轻微接触”豁免，
例如 `mj_left_link2 ↔ mj_right_link2..5`、`mj_left_link5 ↔ mj_right_link6`（`reason="Adjacent_Arms"`
/ `"User_Allowed_Collision"`）。运行时 ACM 实测也证实：验证脚本在其中找到 **39 条**跨臂允许项。

错误来源：当时用 `grep ... | grep ... | grep -c` 统计，**RTK 钩子对链式管道做了改写，返回了空的匹配结果，
我没有复核就采信了**。本轮改用 Python 直接解析 XML 得到真实计数（41）。教训已记入记忆，后续计数不再依赖链式管道。

这一条直接影响本轮：运行时的 ACM **确实存在会隐藏双臂互碰的大范围豁免**，正是 `PROJECT_GUIDE` 风险清单第 4 条。

## 二、修改内容

| 动作 | 文件 |
|---|---|
| 新增 | `include/dual_arm_carry_task/a1_sample_validator.hpp`、`src/a1_sample_validator.cpp`（**纯逻辑，无 ROS**） |
| 新增 | `test/test_a1_sample_validator.cpp`（23 例） |
| 修改 | `dual_arm_controller.{hpp,cpp}`：新增 `runP2aChecks()`（几何/IK/碰撞），`prepareFromA1Truth()` 内按 `p2a_check` 调用 |
| 修改 | `carry_task_context.hpp`、`carry_task_state_machine.cpp`：新增 P2a 参数（默认关闭） |
| 修改 | `launch/dual_arm_carry_task.launch.py`：新增 8 个 P2a 参数，默认保持原行为 |
| 修改 | `scripts/a1_run_scene.sh`：新增 `--p2a-check`、`--p2a-workspace`、`--p2a-workspace-source` |
| 修改 | `CMakeLists.txt`（新库 + gtest 目标 + 链接） |

**未改动**：硬件、夹爪实现、MuJoCo、场景 XML/SRDF、旧一键脚本、APPROACH/GRASP 运动执行算法、
视觉缓存。`rod_pose_source` 的 static/vision/a1_truth 三个入口行为不变。

### 关键实现点

1. **几何**：完整杆 BOX footprint 四角按 yaw 旋转后取 AABB，要求四角全部落在配置包络内
   （footprint 与包络均凸，故四角在内即整体在内）。余量为负时报**实际负值**，不夹到 0。
2. **目标构造（共享，供 P2b/P3 复用）**：`buildPreGraspTarget` / `buildGraspTarget`。
   左右身份 = 杆长轴（局部 Y）符号；末端 yaw = **杆 yaw + 45°**，即 `setRPY(π, 0, yaw+π/4)`。
3. **IK**：每次尝试在同一 `RobotState` 上先左后右求解，写入**同一组合状态**；有界重试 + 每次超时。
   每个通过限位/残差的尝试都按“预抓取→抓取变化”评分，**取最小者**（见第四节）。
4. **碰撞**：`/get_planning_scene` 取**实时场景**（含刚同步的杆与真实环境），
   `PlanningScene::setPlanningSceneMsg` 还原后检查；**不新建只有 robot_model 的裸场景**。
   杆不在场景中时直接失败，不声称已查环境。

## 三、目标变换依据（逐项核实，未把 0.15/0.13 当真值使用）

| 项 | 核实结果 | 依据 |
|---|---|---|
| 末端 link | **`mj_left_link8` / `mj_right_link8`**（运行时日志确认，非假设） | SRDF `mj_left_arm` chain `tip_link="mj_left_link8"`；`getEndEffectorLink()` 运行期打印 |
| 局部 Y = 长轴 | 是，`size=(0.04, 0.40, 0.04)` | `objects.xml` `size="0.02 0.2 0.02"`（半尺寸） |
| 左右身份 | 左 = 杆 **+长轴端**，右 = −长轴端 | A0 `left.y = obj_y + 0.15`；URDF 左臂基座在 base_link **+0.26** |
| 抓取偏移 0.15 | A0 的 `final_y_offset`，**设计取值**（非几何推导）：杆半长 0.20 ⇒ 抓取点在两端内缩 0.05 | `dual_arm_controller.cpp` executeApproach |
| 末端姿态 | `setRPY(π, 0, π/4)`，yaw 项是**基准 45°**，故实际 yaw = 杆 yaw + 45° | 同上 |
| 高度含义 | `approach_height=0.28` / `grasp_height=0.13` 是**腕部 link8 原点 z**，**不是** TCP/指尖 | 组末端为 link8；`hand_tcp` 在 hand 系 z=**0.1034**，hand 与 link8 同原点仅绕 Z 转 −45° |
| TCP 位置 | 腕 roll=π 时 TCP_z = link8_z − 0.1034 ⇒ 抓取时 TCP ≈ 0.0266 m | `hand.xacro` `hand_tcp_joint origin 0 0 0.1034` |

**约定核对（直接证据，非推断）**：运行日志中本模块构造的四元数与
`tf2::Quaternion::setRPY(π, 0, yaw+45°)`（A0 所用 API）角度差 **0.000e+00 rad**（yaw=10° 与 0° 两例均是）。
yaw=0 时目标退化为 `(obj_x, obj_y±0.15, h)` + `setRPY(π,0,π/4)`，与 A0 逐位一致（单测断言 1e-12）。

## 四、IK 判据的一处返工（如实记录）

首版实现把“合理跳变”定义为**相对机器人当前起始位形**的单关节最大变化，阈值 2.0 rad。
真实运行下抓取状态为 **2.1252 rad** → 判 `E_IK_JUMP` 拒绝。

返工理由（不是放宽容差）：prepare 时刻机器人处于 MuJoCo 起始位形，**该位形不是抓取起点**；
且抓取姿态按设计与之相差 **180° 腕部翻转**（A0 的 `setRPY(π,…)`），因此“相对起始位形”必然很大且无判别力。
改为：**记录**相对起始位形的差值（不作判据），**判据改为“预抓取→抓取”的变化**——
两个静态目标之间的肘部翻转才是真正会让下降不可行的缺陷。

同时发现首版 IK“取第一次成功解”使结果**依赖运气**：同一样本重复运行，右臂下降变化在
**1.8588 / 2.2633 rad** 之间跳变（左臂稳定 0.5608）。因此改为**有界尝试内取下降变化最小者**，并记录
限位/残差剔除次数。改后同一右臂为 **0.5823 / 1.1815 rad**（两次独立运行），判据不再被触发。

**保留说明**：所选 IK 分支仍非逐位可复现（右臂 0.58 vs 1.18 rad），但**判定结论可复现**（两次均
`geometry=passed ik=passed collision=failed`）。首次运行的原始数据保留在 `a1_p2a_check_yawp10`，未删除、未改写。

## 五、碰撞检查结果与成因

| 状态 | 杆接触 | 双臂互碰 | 自碰撞 | 其他世界物体 |
|---|---|---|---|---|
| 预抓取 z=0.280（不许任何杆接触） | 0 | 0 | 0 | 0 |
| 抓取 z=0.130（仅允许手指-杆） | **2** | 0 | 0 | 0 |

接触对为 `aluminum_rod ↔ mj_left_hand`、`aluminum_rod ↔ mj_right_hand` —— **是 hand 连杆，不是手指**。

**成因（量化，未夸大）**：hand 的各碰撞体在模型里带 `safety_distance=0.03`（`dual_panda_arm_sim.urdf.xacro:36`
调用 `xacro:hand … safety_distance="0.03"`，`hand.xacro` 中半径统一加了该值）：最低碰撞体由
半径 0.04 → **0.07**。腕 roll=π 时 hand 碰撞体世界 z 范围：

- **带 padding：z ∈ [0.0200, 0.1600]** ↔ 杆 z ∈ [0.0007, 0.0407] ⇒ **重叠 20.7 mm**
- **去掉 padding：z ∈ [0.0500, 0.1300]** ⇒ **不重叠，余量 9.3 mm**

⇒ 即 **物理上的手并未穿杆；接触来自碰撞体外扩（padding）叠加偏低的 `grasp_height`**。
但 MoveIt 规划用的就是带 padding 的几何，所以按本轮“只允许明确手指-杆接触、不全臂豁免”的规则，
**判为碰撞是正确的**。

A0 之所以能跑通，是因为 `executeGrasp` 调用了 `allow_gripper_collision(true)`，它把
**hand 与两指一起**对杆豁免（`gripper_links` 含 `mj_*_hand`）——即 A0 恰好豁免了本处接触。

> ⚠️ **本节结论已更正**：上面的“去掉 padding 后不重叠，余量 9.3 mm”**只覆盖了前三个碰撞体，是不完整的**。
> 完整六体分析与三个模型的区分见文末“追加：P2a 碰撞模型与支撑面补齐”§A。原结果与数字保留不动。

**本轮未自行决定怎么办**，交规划器选择：(a) 维持规则、抬高 `grasp_height` 至手部脱离；
(b) 明确把 hand-杆接触列为**有依据的允许接触**并在代码与记录中写明，而不是靠全局豁免掩盖。

## 六、真实 MoveIt 检查（一条仿真实例，不运动）

```bash
./scripts/a1_run_scene.sh --prepare-mode --p2a-check --x 0.48 --y 0.03 --yaw-deg 10 --run-id a1_p2a_check_yawp10e
```

| 运行 | 结果 | 备注 |
|---|---|---|
| `a1_p2a_check_yawp10` | geometry=passed / ik=**failed**(E_IK_JUMP) / collision=NOT_RUN | 首版“相对起始位形”判据，保留 |
| `a1_p2a_check_yawp10b` | geometry=passed / ik=passed / collision=**failed** | 修正判据后首次得到碰撞结论 |
| `a1_p2a_check_yawp10d` | geometry=passed / ik=failed(E_IK_DESCENT_JUMP 2.2633) | “取首次成功解”的抖动证据 |
| **`a1_p2a_check_yawp10e`** | **geometry=passed / ik=passed / collision=failed** | 最佳尝试选择后，主样本 |
| **`a1_p2a_check_yawp10h`** | **同上（结论可复现）** | 独立复跑 |
| `a1_p2a_check_yawp10c/f/g`、`a1_p2a_negative_far` | 未产出结论 | **既有启动崩溃**，见第八节，原样保留 |

样本（`--x 0.48 --y 0.03 --yaw-deg 10`）：稳定真值 yaw 10.0000°、z 0.020654、footprint
x=[0.4256, 0.5344] y=[-0.1704, 0.2304]；`run_id` 与 `frame=world → base_link` 处理同 P1b。

**负例（同一条真实实例）**：`a1_p2a_negative_far_b`（`--x 0.90 --y 0.0 --yaw-deg 0`）

```
geometry=failed (E_GEO_OUT_OF_WORKSPACE, 最小余量 -220.000 mm)
ik=failed       (E_IK_NO_SOLUTION，左臂，5 次尝试)
collision=NOT_RUN (E_IK_NOT_SOLVED —— IK 未解出时不谎称已查碰撞)
```

覆盖了要求的**边界越界**与**不可达**负例；**明确碰撞**负例即上面的 hand-杆接触。
该 run 早于 `[P2A_RESULT]` 机器行的修改，故其结论见日志中的分项行（`a1_p2a_check_yawp10`/`10b` 同）。

**无运动证据**：上述每个含 task 日志的运行中，`状态: APPROACH`、`状态: GRASP`、`双臂规划成功`、
`双臂移动完成`、`Weld约束已激活`、`Object attached`、夹爪指令出现次数**均为 0**；终态为 `PREPARED`。

## 七、构建与测试

```bash
export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"     # 规避 anaconda Python 3.13 遮蔽系统 Python 3.10
colcon build --packages-select dual_arm_carry_task --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test  --packages-select dual_arm_carry_task --ctest-args -R "test_a1_"
```

**5 个套件 122 例全部通过，0 失败**（新增 `test_a1_sample_validator` 23 例；其余 26/16/35/22）。
构建 0 warning / 0 error。新增测试覆盖：yaw=0 与 A0 目标逐位一致、左右身份在每个 yaw 下沿长轴、
yaw 同时作用于偏移与姿态、预抓取/抓取仅 z 不同、footprint（0°/90°/15°）、包络越界（含“中心在内但杆端越界”）、
支撑高度、倾角、坏输入与坏配置、关节限位（含非有限与尺寸不匹配）、q/−q 等价残差、
跨臂/杆/手指条目分类、汇总必须三项全过且 path 恒为 NOT_RUN。

## 八、遗留问题（分级）

| 级别 | 项 |
|---|---|
| **需规划器决策** | 抓取状态 hand-杆接触（第五节）：抬高 `grasp_height` 还是明确声明为有依据的允许接触。 |
| 允许后续改进 | 运行时 ACM 含 **39–41 条跨臂允许项**（SRDF 大范围豁免）。本轮在**验证副本**中关闭后双臂互碰仍为 0，说明当前样本未受其影响；但该豁免会掩盖后续样本的真实互碰，建议 P2b 前明确取舍。**未改动全局 SRDF/ACM。** |
| 允许后续改进 | IK 分支非逐位可复现（判定结论可复现，见第四节）。可考虑用 `setFromIK` 的 consistency limits 约束到种子附近。 |
| 允许后续改进 | 配置工作包络 x=[0.30,0.70] y=[−0.30,0.30] 是**研究包络**，来源已写入参数与日志；支撑面为无限 plane，**未声称是物理桌面边缘**。其默认值由 P1a 采样范围按 15° 最大 footprint 外扩 +80 mm 得到，非碰撞/可达性推导。 |
| 允许后续改进 | **既有启动崩溃**：`mujoco_node` `spin_some() called while already spinning` → SIGABRT → `move_group` segfault，服务不出现。本轮 3 次触发（`…10c`、`…10f`、`…10g`、`a1_p2a_negative_far`），**全部原样保留、无隐藏重试**；脚本按既有约定记录 `E_SERVICES_UNAVAILABLE` 并非零退出。另有一次 `a1_p2a_check_yawp10f` 因上一 run 拆除竞态被“实例占用”守卫拦下（`ros2 service list` 守护进程缓存未过期；`--no-daemon` 复查为空，确认无存活实例）。 |
| 当前不需要解决 | `p2_acceptance=null`、`path_validation=NOT_RUN` 保持不变；样本可搬运性由 P2b 判定。未实现路径优化、阻抗、视觉。 |

## 九、说明

本轮**只**做静态检查，**未执行任何抓取或搬运**，也**未**证明任何样本可搬运。
`[P2A_RESULT]` 行在所有退出路径都会输出；脚本在 `--p2a-check` 下以该行决定退出码
（三项全 `passed` 才为 0，否则 4），因此 `PREPARED` 不会被误读成样本通过。

---

# 追加：P2a 碰撞模型与支撑面补齐（2026-10-12）

本轮按规划器“P2a 碰撞模型与支撑面核查”执行：**不改高度、不新增 hand–杆豁免、不进入运动、不提交 Git**。
原有拒绝结果与失败数据全部保留，未删除、未改成成功。

## A. 更正：手掌六体完整核对，以及三个不同模型的区分

前一轮的“去掉 padding 后手掌仍余 9.3 mm”**只覆盖了前三个碰撞体**（`hand.xacro` 中中心 z=0.04、
半径 0.04 的那组），**结论不完整**。完整核对如下。

`hand.xacro` 的手掌共 **6** 个碰撞体；hand 与 `mj_*_link8` **同原点、仅绕 Z 转 −45°**，
且腕部 roll=π 时手掌局部 +Z 精确映射到世界 −Z（对任意 yaw 成立），故 **world_z = link8_z − local_z**，
是精确关系而非近似。`safety_distance=0.03` 由 `dual_panda_arm_sim.urdf.xacro:36` 传入，
**已写进 URDF 形状本身**（半径 0.04→0.07、0.02→0.05），不是 MoveIt 运行时 padding，
`setLinkPadding(0)` 去不掉。

| # | 形状 | 局部 z 中心 | 半径 原始/带外扩 | 世界 z @ link8=0.13（原始） | 世界 z（带外扩） |
|---|---|---|---|---|---|
| 1–3 | 圆柱(轴∥Y) + 两个球 y=±0.05 | 0.04 | 0.04 / 0.07 | [0.05, 0.13] | [0.02, 0.16] |
| **4–6** | 圆柱(轴∥Y) + 两个球 y=±0.05 | 0.10 | 0.02 / 0.05 | **[0.01, 0.05]** | **[−0.02, 0.08]** |

杆世界 z ∈ [0.000654, 0.040654]（实测中心 0.020654），支撑面 z=0。

- **原始（去外扩）URDF 模型**：1–3 组最低到 0.05 > 杆顶 0.0407 ⇒ **不碰杆**；
  **4–6 组最低到 0.01 < 杆顶 ⇒ 碰杆**。⇒ 造成接触的是**后三个**体，与上一轮的说法正好相反。
- **带外扩 URDF 模型**：六体全部与杆重叠；4–6 组还低到 **−0.02**，**穿过地面**。
- 这与 MoveIt 实测完全吻合：抓取状态 `aluminum_rod↔mj_*_hand` 共 2 处（每手 1 处）。

**第三个模型：仿真实际使用的 MuJoCo 碰撞体。** `mj_dual.xml` 中
`<body name="mj_left_hand" quat="0.9238795 0 0 -0.3826834">`（w 在前，即绕 Z −45°，与 URDF 一致，
无 `pos`）下只有**一个**碰撞 geom：`<geom mesh="hand_c" class="collision"/>`，网格为
`franka_description/mujoco/franka/assets/hand.stl`（二进制，200 三角面，文件 10084 B = 84+200×50，
无 `scale`）。直接解析该 STL 顶点：

- 局部 AABB：x[−0.0316, 0.0316]、y[−0.1040, 0.1004]、**z[−0.0259, 0.0660]**
- 世界 z @ link8=0.13：**[0.0640, 0.1559]**
- ⇒ 相对杆顶 0.0407 **余量 23.3 mm**；相对地面 z=0 **余量 64.0 mm**。

**z 区间完全不相交 ⇒ 二者不可能相交**（z 分离是充分判据，此时横向位置无需再判）。

| 模型 | 与杆 | 与地面 |
|---|---|---|
| URDF 带外扩（MoveIt 实际使用） | **穿透** | **穿透** |
| URDF 去外扩 | **穿透**（仅后三体） | 不穿透（最低 0.01） |
| MuJoCo 实际（hand_c 网格） | **净空 23.3 mm** | **净空 64.0 mm** |

**关键点**：URDF 手掌在抓取高度**低到 z=−0.02，即落到地面以下**——手不可能位于地面之下。
这说明该 URDF 手掌近似体明显偏大，而不是“抓取高度必然导致碰撞”。
同时必须说明：`hand.stl` 本身也是**凸包碰撞代理**，不等于真实硬件外形，本轮**不声称** mesh 与真实手掌等价。

## B. 支撑面补齐与地面检查

`runP2aChecks` 之前只读现有场景，**未补支撑面**——杆的姿态/高度几何检查不能代替机器人与地面的碰撞检查。
本轮在**验证副本**中补齐（`processCollisionObjectMsg`，只改本地 `PlanningScene`，**不发布、不改全局**）：

- 来源：`dual_scene.xml` 的 `<geom name="floor" size="0 0 0.05" type="plane" .../>`（无 `pos` ⇒ 无限平面位于 z=0）。
- 表示：**薄板 BOX 6.0×6.0×0.02，顶面恰在 z=support_z**。

> ⚠️ **此处上一轮的推断有误，已更正**：`shape_msgs/SolidPrimitive` 没有 PLANE 常量**不等于**
> CollisionObject 不支持平面——`moveit_msgs/CollisionObject.msg` 本身就有独立的
> `shape_msgs/Plane[] planes` 与 `plane_poses`。薄板只覆盖有限 XY 与 z∈[support_z−0.02, support_z]，
> **不能称为完整无限地面**。更正与后续见文末“追加：P2a 局部手掌模型一致化”§F。
- 先检查是否已存在同名对象，存在则不重复添加。
- **允许项仅限固定安装**：`base_link`、两臂 `link0`、两臂 `link1` ↔ 支撑面。
  依据是几何而非笼统豁免：这三类的碰撞体跨越安装平面，且其 z 范围**不随臂位形变化**
  （`link0` 球心局部 z=0.06 r=0.09 ⇒ [−0.03,0.15]；`link1` 的 joint1 在 z=0.333、最低球局部 z=−0.333
  ⇒ 球心恰好落在 z=0，r=0.09 ⇒ [−0.09,0.09]，joint1 只绕竖轴转）。
  **手部与手指不在豁免内**，其余连杆与地面的接触一律检查。

**地面负例（明确为验证注入，非采样位姿）**：构造低于支撑面的目标（z=−0.05）并求 IK，用同一场景与同一 ACM 检查。
结果：**地面/支撑面接触 8 处**（link6/7/8、hand 等自由连杆）→ **被正确拒绝**，并已断言“若注入未被拒绝则整项按失败处理”，
即该检查**已被证明能拒绝机器人穿地**。

## C. 本轮真实运行结果

```bash
./scripts/a1_run_scene.sh --prepare-mode --p2a-check --x 0.48 --y 0.03 --yaw-deg 10 --run-id a1_p2a_scene_yawp10c
```

| 状态 | 杆接触 | 地面/支撑面 | 双臂互碰 | 自碰撞 | 其他世界物体 |
|---|---|---|---|---|---|
| **预抓取 z=0.280** | 0 | **0** | 0 | 0 | 0 |
| 抓取 z=0.130 | 2 | 2 | 0 | 0 | 0 |
| 地面负例注入 z=−0.050 | 6 | 8 | 0 | 1 | 0（**正确拒绝**） |

- **预抓取组合状态五项全为 0**，在补齐支撑面、恢复跨臂检查、关闭 39 条跨臂豁免之后依然干净。
- 抓取状态仍是 **hand** 与杆（2）及地面（2）接触；**手指-杆接触 0 处**（手指本来就未触到）。
- 双臂互碰在所有状态均为 **0**。
- `P2A_RESULT geometry=passed ik=passed collision=failed path_validation=NOT_RUN p2_acceptance=null static_passed=false`
- **无运动边界**：APPROACH/GRASP/双臂规划/双臂移动/夹爪/weld/attach 计数**均为 0**；终态 `PREPARED`。
- 构建 0 warning；测试 **124 例 0 失败**（`test_a1_sample_validator` 由 23 → 25）。

数据路径：`experiment_results/a1_runs/a1_p2a_scene_yawp10c/`（`…b` 为同批的启动崩溃保留，`…` 为首次未加安装豁免的版本）。

## D. 结论与最小修正建议（交规划器决定）

**结论**：在**仿真实际使用的碰撞模型**下，`grasp_height=0.13` 时手掌相对杆有 **23.3 mm** 净空、
相对地面有 64.0 mm 净空，**不发生穿透**；造成拒绝的是 **URDF 的六体手掌近似（且带 0.03 外扩）明显偏大**——
它甚至低到地面以下 2 cm，物理上不可能。

**建议（按优先级）**

1. **不要用抬高 `grasp_height` 来消碰撞**。要让带外扩的 URDF 手掌离开杆，需 link8 抬到约 0.19 m
   （0.0407+0.15），腕部抬高 6 cm，指尖将**抓空**；这正是规划器提示要避免的。
2. **推荐：抓取近场改用仿真实际使用的模型作局部验证。** 即在验证副本中，对 `hand` 连杆
   **只在该状态**用 MuJoCo 的 `hand_c` 网格（或其在 hand 系下的 AABB）替代那 6 个 URDF 近似体，
   其余连杆、ACM、场景不变；手指-杆仍严格按手指判定。建议同时保留“URDF 模型下的结果”一并输出，
   使两种模型的差异始终可见，而不是把 URDF 结果替换掉。
3. 若规划器希望保留 URDF 模型为唯一判据，则需先**核对 URDF 手掌近似体是否应缩到与 `hand_c` 相当**——
   这属于修改 `franka_description` 的碰撞几何，影响 A0 与全部规划，本轮**未做**，需单独授权。

**本轮未做**：未改 `grasp_height`、未把 hand–杆加入 ALWAYS、未改 URDF/SRDF/XML/ACM、
未实现 P2b 路径检查、未进入运动。`path_validation=NOT_RUN`、`p2_acceptance=null` 保持不变。

## E. 新增遗留

| 级别 | 项 |
|---|---|
| **需规划器决策** | 建议 2 与建议 3 的取舍（局部换用 MuJoCo 模型 / 修正 URDF 手掌几何）。 |
| 允许后续改进 | 支撑面用**有限薄板**近似无限平面（±3 m 外不覆盖）；`shape_msgs/SolidPrimitive` 无 PLANE 常量导致无法直接传平面。 |
| 允许后续改进 | 固定安装豁免集合 `base_link/link0/link1` 由本轮几何证据得出并已写入注释与日志；若规划器认为 `link1` 不应豁免，改一处常量即可。 |
| 允许后续改进 | 地面负例用 IK 构造；若 IK 偶然失败，日志会明确记“注入未执行”并**不**视为通过（不会静默放过）。 |
| 允许后续改进 | 既有启动崩溃本轮再次触发（`a1_p2a_scene_yawp10b`，`spin_some`+segfault），原样保留、无隐藏重试。 |

---

# 追加：P2a 局部手掌模型一致化（2026-10-12）

按规划器决策实施：**保持 `grasp_height=0.13`**，在独立验证模型中把两只手掌的碰撞几何替换为
仿真实际 `hand_c` 网格的**保守 AABB**；不新增 hand–杆/地面 ALWAYS，不改全局 URDF/SRDF/模型/运动入口。
本轮**未进入运动、未提交 Git**；旧运行数据未改写。

## F. 更正：CollisionObject 支持独立平面字段

上一轮我从 `shape_msgs/SolidPrimitive` 没有 PLANE 常量，**错误地推断** CollisionObject 无法承载平面。
事实是 `moveit_msgs/CollisionObject.msg` 有**独立的** `shape_msgs/Plane[] planes` 与 `plane_poses`。
因此：

- 现有的 6×6×0.02 薄板**只覆盖有限 XY 与 z∈[support_z−0.02, support_z]**，
  **不能称为完整无限地面**；低于 −0.02 或超出 ±3 m 的区域不在检查域内。
- 本轮按规划器意见**不重写地面机制**（不要求为这一事实返工）。P2b 若需要更大深度/范围，
  改用 `planes`+`plane_poses` 的真正平面，或足够深的 BOX。
- 地面负例注入到 z=−0.05，仍在薄板覆盖深度内，故该负例结论不受影响。

## G. 局部手掌模型替换的实现

**网格来源与核对**（运行时读取**已安装**的场景文件，非源码树）：

```
/home/lidio/franka_ws_1/install/franka_description/share/franka_description/mujoco/franka/assets/hand.stl
600 顶点（二进制 STL，200 三角面），未施加额外缩放
局部 AABB: x[-0.031636, 0.031616]  y[-0.103990, 0.100426]  z[-0.025925, 0.065962]
```

z 范围与规划器独立解析的 `[-0.0259248, 0.0659622]` 一致。读到的顶点会**断言全部落在包围盒内**
（`aabbContainsAll`）；网格读不到、尺寸自相矛盾或缺顶点时**明确失败并停止**，绝不用空模型顶替。

**替换方式**：`hand` 与 `mj_*_link8` 同原点，故局部 AABB 的**中心不是原点**。
替换体为 BOX，**中心 = (min+max)/2 = (−0.000010, −0.001782, 0.020019)**、
**尺寸 = max−min = (0.063252, 0.204416, 0.091887)**，作为 collision origin 偏置放进 `mj_*_hand`。

**独立模型，不碰共享对象**：从本节点自己的 `robot_description` / `robot_description_semantic`
**重新解析**一份 URDF，只清空并重建 `mj_left_hand`、`mj_right_hand` 两个 link 的 `collision_array`，
再与同一 SRDF 一起构造一个**新的** `RobotModel` 与新的 `PlanningScene`。
共享 RobotModel 与活跃 PlanningScene **未被就地修改**，也**没有**用 ACM 关闭手掌接触。

**世界一致**：新场景应用**同一条** `/get_planning_scene` 消息（含杆），并同样补上支撑面，
因此两个模型在**完全相同的世界**上比较。ACM 按名字生效，故同一份
`base_acm`/`grasp_acm`（含 39 条跨臂豁免关闭、手指-杆规则、固定安装豁免）对两个模型都适用。

**状态映射已实测核对**：把 IK 解按同名关节映射到新模型后，抓取目标 FK 残差
**0.000001 mm / 0.000000°**，即两模型状态一致，不是假定。

## H. 本轮结果（同一非零 yaw 样本，一次仿真实例）

```bash
./scripts/a1_run_scene.sh --prepare-mode --p2a-check --x 0.48 --y 0.03 --yaw-deg 10 --run-id a1_p2a_handmodel_yawp10b
```

| 模型 | 状态 | 杆 | 地面/支撑面 | 双臂互碰 | 自碰撞 | 其他 |
|---|---|---|---|---|---|---|
| `urdf_reference`（对照） | 预抓取 | 0 | 0 | 0 | 0 | 0 |
| `urdf_reference`（对照） | 抓取 | **2** | **2** | 0 | 0 | 0 |
| **`mujoco_hand_aabb`（本轮判据）** | 预抓取 | **0** | **0** | 0 | 0 | 0 |
| **`mujoco_hand_aabb`（本轮判据）** | 抓取 | **0** | **0** | 0 | 0 | 0 |

⇒ 选定模型下 **`geometry=passed ik=passed collision=passed`，`static_passed=true`**，脚本退出码 0。
原 URDF 结果**同时输出**为对照，两者不混为同一模型的结论。

**两个负例都成立**（均为**验证注入**，非实际机器人运动）：

| 注入 | 结果 |
|---|---|
| 与替换后手掌 AABB **完全重合**的障碍 | 检出 **4** 处接触（含 `a1_hand_obstacle_injected↔mj_left_hand`）⇒ **替换几何确实参与检查**，没有被静默关掉 |
| 目标 z=−0.05（**穿地**） | 检出 **6** 处地面接触 ⇒ **地面检查确实能拒绝穿地** |

若任一注入未被拒绝，代码按**失败**处理（`E_P2A_HAND_NEGATIVE_MISSED` / `E_P2A_GROUND_NEGATIVE_MISSED`），
不会静默放过。

**无运动边界**：APPROACH/GRASP/双臂规划/双臂移动/夹爪/weld/attach 计数**均为 0**；终态 `PREPARED`。

**测试**：**128 例 0 失败**（`test_a1_sample_validator` 25 → 29，新增 AABB 中心/尺寸与含点断言、
空点集、越界点、二进制 STL 读取、以及"缺失/尺寸矛盾文件必须报错而非返回空模型"）。
构建 0 warning。

数据路径：`experiment_results/a1_runs/a1_p2a_handmodel_yawp10b/`
（`…a` 为同批启动崩溃保留；`a1_p2a_scene_yawp10*` 为本轮之前的场景版，全部保留）。

## I. 观察与遗留

| 级别 | 项 |
|---|---|
| 观察（本轮引入，如实记录） | 替换后的 BOX 在 y 方向较宽（±0.10），几何上与同臂的 `link6/link7/link8` 重叠。这些同臂配对**本来就在 SRDF 中禁用**（`hand↔link7` 为 `Adjacent`、`hand↔link8` 为 `Adjacent`、`hand↔link6` 为 `Never`），故不产生报告；`link6` 的 `Never` 属既有的大范围豁免（PROJECT_GUIDE 风险 #4），非本轮引入。**不影响本轮结论**：手-杆、手-地面、双臂互碰三项都实际参与检查。 |
| 需规划器知悉 | 地面仍是有限薄板（见 §F），不能称为完整无限地面；P2b 若要更大检查域需改用 `planes`/`plane_poses` 或更深的 BOX。 |
| 允许后续改进 | `mujoco_hand_aabb` 是 hand_c 凸包的保守 AABB，比凸包本身更大，因此**偏保守**（更易报碰撞）；它仍不等于 `hand_c` 凸包，更不等于真实硬件。 |
| 允许后续改进 | `hand.stl` 路径经 `franka_description` 的 share 目录解析；若该包未安装则明确失败（不会退回源码路径）。 |
| 允许后续改进 | 既有启动崩溃本轮触发 1 次（`a1_p2a_handmodel_yawp10`），原样保留、无隐藏重试。 |
| 当前不需要解决 | `p2_acceptance=null`、`path_validation=NOT_RUN` 保持不变。**静态通过不等于可搬运**，也不证明真实手掌安全；联合预抓取路径属 P2b。 |

**本轮未做**：未改 `grasp_height`、未新增 hand–杆/地面 ALWAYS、未改 `franka_description` 源码/URDF/SRDF/XML、
未改运动/夹爪/硬件/控制器、未进入 P2b、未执行任何运动。

---

# 追加：P2a 记录的两处更正（2026-10-12，随 P2b 交付补记）

规划器指出两处表述不准确，均**不改变已通过的结论**，不重跑 P2a：

1. **地面负例的机理**：注入目标 z=−0.05 **并不落在**当时薄板的 z∈[−0.02, 0] 内。
   该负例之所以被检出，是因为**该机器人状态下的部分碰撞体**与该薄板相交，
   **不是**因为目标点在板内。原文"目标 z=−0.05（低于支撑面）"易被读成点在板内，特此更正。
   （同时薄板已按 P2b 要求改为**深 2.0 m 的 BOX**，见 P2b 记录；顶面仍在 z=0。）
2. **手掌负例的障碍位姿**：首次实现把障碍放在 **hand 连杆原点**，而替换后的 AABB **中心是偏置的**
   （中心 z=0.020019），两者**确实重叠并检出了碰撞**，因此负例有效；
   但不应表述为"几何完全重合"。**已改为把障碍放在 AABB 的真实中心并采用 hand 姿态**，
   使二者真正同中心同姿态（最近一次运行检出 5 处接触，仍通过）。
