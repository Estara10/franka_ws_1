# P4/S2 第一步：双臂搬运轨迹规划器对照（规划-only + 条件执行）——执行器提示词

日期：2026-10-17。状态：待执行。执行人：执行器。
本文是任务书，不是结论；完成后由规划器在此基础上修改与推进。

---

## 0. 背景与目标

无 weld 接触抓取的 40 cm / 2 s 完整流程已跑通并冻结（commit `2bc766a`、`316cefd`）。
当前进入 P4/S2（真实轨迹优化）。第一步先做**规划器对照**（Phase 1，不执行、不改接触）：

- 同一场景、同一起点、同一目标；
- 用当前自定义路径（物体空间 IK + IPTP）与 MoveIt CHOMP 各规划一条双臂轨迹；
- 逐点比较双臂闭链残差、碰撞、限位与平滑度；
- 用数据判断 MoveIt CHOMP 能否直接用于双臂搬运，还是必须在物体空间路径上做优化。

若 Phase 1 证明 CHOMP 能维持双臂闭链约束（≤5 mm/2°，无碰撞/超限），再在同一会话中做
**一次条件执行对照**（Phase 2，见 2.5）；否则停在 Phase 1，由规划器决定替代方案。

已确认：launch 中配置了 `chomp_interface/CHOMPPlanner`；在无窗口 MoveIt 场景里对
`dual_panda` 组给一个平凡关节目标，`ompl` 与 `chomp` 都返回成功（code=1）。但平凡
目标不能说明优化质量，必须在真实搬运端点上核对。

## 1. 当前代码事实（以工作树为准，先核对再改）

- `src/multipanda_ros2/dual_arm_carry_task/src/contact_load_probe.cpp`：
  - `runContactLoadProbe`：抓取、抬升、保持、重建立、运输、落回、释放；
  - `makePlan(origin, delta, plan, vscale, ascale, min_total, dyaw)`：物体空间平移+偏航
    插值、双臂 IK、碰撞/限位检查、IPTP+统一时间伸缩、五阶插值检查；
  - `valid(state)`：双臂反推杆位姿残差、支撑间隙、碰撞；
  - `object_to_hand[2]`、`eefs`、`criterion_model`、`scene`、`acm`、`measured()`。
- 运输段当前调用：
  `makePlan(lifted, transport_delta, transport, 1.0, 0.3, 2.0, transport_dyaw)`，
  目标 `(0.40, 0.00, +20°)`，然后 `execute(transport,"TRANSPORT")`。
- MoveIt 接口：`ctx_->dual_arm_`（`MoveGroupInterface`，组名 `dual_panda`）；
  launch 已注册 `chomp` 与 `ompl` 两条 pipeline。
- CHOMP 参数：`franka_moveit_config/config/chomp_planning.yaml`。

## 2. 实现要求（基础版）

### 2.1 新增一个规划对照模式（默认关闭）

- 增加参数 `contact_plan_compare`（bool，默认 false），从 launch → 状态机 → context
  传到 `contact_load_probe`；只影响 `contact_probe` 入口。
- 当 `contact_plan_compare=true` 时：完成预抓取、下降、闭合、抬升、保持、重建立之后，
  **不执行运输**，改为执行下面的规划对照，然后记录并结束本轮（状态可记为
  `PLAN_COMPARE_DONE`，不得写成 DONE 或承载成功）。
- 当 `contact_plan_compare=false` 时：现有行为完全不变（仍执行 IPTP 运输、落回、释放）。

### 2.2 两条轨迹

**基线 A（当前路径）**：直接复用现有 `makePlan(...)` 生成的 `transport` 计划，
不执行。

**候选 B（MoveIt CHOMP）**：
- 起点：当前实测双臂状态；
- 终点：基线 A 轨迹最后一个点的关节位置（同一目标，只换规划器）；
- 调用 `ctx_->dual_arm_`：`setJointValueTarget(终点关节名/位置)`、
  `setPlannerId("chomp")`、`setPlanningTime(...)`、`setMaxVelocityScalingFactor/`
  `setMaxAccelerationScalingFactor`（与基线相同的 1.0/0.3 或明确记录实际值）、`plan()`；
- 可选再加一条 `ompl` 作为普通规划器对照（同一终点）；是否加由执行器决定并记录。
- 记录成功/错误码、规划耗时、轨迹点数、总时长。

### 2.3 逐点评估（两条轨迹用同一套函数）

对每条轨迹的每个点（以及按 20 ms 插值的样本，样本数记录）：

1. **双臂闭链残差**：用 `RobotState` 的左右 EEF FK × `object_to_hand^-1` 得到两侧
   反推杆位姿，记录二者位置差（mm）与姿态差（deg）的最大值；
2. **杆的隐含路径**：反推杆位姿相对基线目标位姿的位置/角度偏差的最大值；
3. **碰撞**：用与现有 `valid()` 相同的 `scene`/`criterion_model`/`acm` 检查，
   记录碰撞对数与首对名称；
4. **关节限位/速度/加速度**：记录超限的关节、数值与限值；
5. **平滑度**：关节加加速度（jerk）的有限差分最大值与 p95，说明差分窗口；
6. **时长/点数**。

### 2.4 输出

- 每个 run 目录写 `plan_compare.csv`：一行一个样本，列包含
  `planner, sample_index, t, left_inferred_x/y/z, right_inferred_x/y/z,
  dual_pos_err_mm, dual_ang_err_deg, rod_path_pos_err_mm, rod_path_ang_err_deg,
  collision, joint_limit_violation, vel_violation, acc_violation`；
- `task_launch.log` 打印两条轨迹的汇总行：
  `[PLAN_COMPARE] planner=... success=... planning_time=... points=... duration=...
   max_dual_pos_mm=... max_dual_ang_deg=... collisions=... limit_violations=...`;
- 最后打印 `[PLAN_COMPARE_RESULT]`，明确是否两条都成功、CHOMP 的最大双臂残差是否
  ≤5 mm/2°。

### 2.5 Phase 2：执行对照（有条件，基础版一并实现）

仅当 Phase 1 同时满足：CHOMP 规划成功、最大双臂残差 ≤5 mm/2°、无碰撞、无限位/速度/
加速度超限时，才执行 Phase 2；否则记录原因、停止，不执行。

Phase 2 在同一 `contact_probe` 会话中：

1. 用 CHOMP 计划替换 IPTP 运输（同一起点、同一目标、同一速度/加速度缩放与时间参数化）；
2. 执行运输，后续重建立、落回、释放按现有逻辑不变；
3. 记录：运输实际执行时长、实际关节速度/加速度峰值、逐点双臂残差、接触滑移、
   `CONTACT_PROBE_RESULT`；
4. 与 IPTP 基线（最近 run `..._164743`、`..._185847`）对比时长、平滑度、滑移与成功状态。

Phase 2 只跑一次；失败按现有失败路径停止，不改接触参数、不重试。

## 3. 运行方式

```bash
# 构建
source /opt/ros/humble/setup.bash
export MUJOCO_DIR=/home/lidio/Libraries/mujoco
colcon build --packages-select dual_arm_carry_task
ctest --test-dir build/franka_hardware -R '^test_gripper_shared_state$' --output-on-failure

# 一次规划-only 运行（无窗口）
./scripts/run_contact_probe_local.sh --headless   # 需先通过 launch 参数打开 contact_plan_compare
```

`run_contact_probe_local.sh` 目前不传业务参数；执行器可以在 `a1_run_scene.sh` 里为该
模式追加 `contact_plan_compare:=true`（仅此模式），或在 launch 默认值上临时开启后恢复。
要求：**只跑一次**这个规划对照；启动阶段失败最多重试一次。

## 4. 验收与停止

| 观察 | 含义 | 动作 |
|---|---|---|
| 两条轨迹都规划成功，CHOMP 最大双臂残差 ≤5 mm/2°，无碰撞/超限 | CHOMP 可直接作为 S2 候选 | 记录，进入“执行对照”下一轮 |
| CHOMP 成功但双臂残差 >5 mm/2° | MoveIt CHOMP 不维持双臂闭链约束 | 记录；S2 改为物体空间路径 + IK 上优化，不直接用 CHOMP |
| CHOMP 规划失败/超时 | 当前场景/参数下不可用 | 记录错误码与耗时；不扫描参数 |
| 基线 A 失败 | 现有路径被本次改动破坏 | 停止，修复；不得用 CHOMP 结果掩盖 |
| 接触流程（模式关闭）行为变化 | 侵入性回归 | 停止，回退到模式隔离 |

**停止条件**：一次规划对照运行结束即停；不改接触、驱动、速度、滑移、模型或判据；
不为了“让 CHOMP 成功”而调 CHOMP 参数或放宽 5 mm/2°。

## 5. 边界

- 不执行 CHOMP 轨迹，不改变接触抓取行为；
- 不改 `noslip_iterations`、速度、滑移窗口、夹持力、pad、过滤、场景/ACM；
- 不改 `ideal_weld`；
- 不提交、不推送（除非用户另行要求）；
- 保留所有失败目录与原始 CSV。

## 6. 交付

1. 修改文件与 `git diff --stat`；
2. 构建/测试结果；
3. run 目录；
4. 两条轨迹的 `[PLAN_COMPARE]` 汇总与 `[PLAN_COMPARE_RESULT]`；
5. `plan_compare.csv` 的字段与样本数；
6. 明确回答：CHOMP 的最大双臂残差是多少、是否 ≤5 mm/2°、是否有碰撞/超限；
7. 若进入 Phase 2：执行结果、时长/平滑度/滑移对比，以及 `CONTACT_PROBE_RESULT`；
8. 任何偏离本任务书之处。
