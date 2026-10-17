# S3 外部笛卡尔阻抗最小链路与空载验证（2026-10-17）

范围：只接通双臂笛卡尔阻抗的运行链路并做一次空载验证。不改夹爪、接触、质量、摩擦、
求解器、验收窗口或任务目标；不启用 weld，不执行抓取、承载、搬运或外力扰动；
不安装依赖，不提交、不推送。

**本轮完成只代表“外部笛卡尔阻抗链路通过空载验证”，不代表接触承载通过，不代表扰动恢复
更好，不代表柔顺研究完成。本轮未执行搬运，不宣称滑移、接触力或成功率改善。**

## 1. 结论

**通过。** 一次有效运行
`experiment_results/a1_runs/dual_impedance_no_load_20261017_r3`（bridge 退出码 0）：

| 预设观察标准 | 结果 |
|---|---|
| 控制器始终互斥 | `switch_controller` 两次 `ok=1`，STRICT |
| 无非有限输出 | 控制律计数 `nonfinite=0` |
| 无持续限幅 | `magnitude_limited=0`，`rate_limited=0` |
| 保持与跟踪误差 ≤ 5 mm / 2° | **2.16 mm / 0.38°**（全程最大） |
| 目标被拒 | 0 次 `Discarding` |
| 切回后状态如实报告 | bridge 日志记录 seed 与退出码 0 |

## 2. 实际运行的控制链（不是只看类名）

1. **`sim_multi_mode_controller` 实际加载的内部控制器**：`multi_mode_controller/MultiModeController`，
   由 `sim_dual_panda_ros_controllers.yaml` 的 `controllers: [dual_cartesian_impedance_controller]`
   指定，资源 `mj_left&mj_right`。`DualCartesianImpedanceController` 只覆盖 ROS 回调，
   扭矩律来自基类 `ComlessDualCartesianImpedanceController::computeTauImpl`。
   该控制器**不由 launch 自动加载**，本轮用
   `ros2 run controller_manager spawner sim_multi_mode_controller --inactive` 先加载并配置。
2. **坐标与顺序**：`robot_data_` 顺序为 `[mj_left, mj_right]`（`resourceToVector` 拆分 `&`），
   因此 goal 的 `l_pose → poses[0] = 左`、`r_pose → poses[1] = 右`。
   末端位姿来自 `franka::RobotState::O_T_EE`（`robot_sim.cpp` 用 site7 相对 link0 计算），
   误差、速度与 `eeZeroJacobian` 全部在**各自手臂 link0 基座系**中表达，三者坐标一致。
   注意：这不是 MoveIt 的 `base_link` 系，两手基座分别在 y=±0.26 m。
3. **目标与参数**：话题 `mj_left_and_mj_right/dual_cartesian_impedance_controller/desired_pose`，
   服务 `.../parameters`。目标经 `Buffered<>` 双缓冲，在控制线程 `update()` 后生效。
4. **effort 到 actuator**：`MultiModeController::update` 写
   `<arm>_joint<i>/effort` → 硬件 `write()` 的 `ControlMode::JointTorque` 分支 →
   `d->ctrl[act_trq_indices_[i]]`。
5. **补偿**：MuJoCo 模型全部 24 个手臂 body 带 `gravcomp="1"`，`robot_sim.cpp` 也明确把
   `qfrc_gravcomp` 加进发布的 `tau_J`。**重力已由仿真器抵消**，控制律只叠加
   `getCoriolisForceVector()`。再加 URDF 重力项会重复补偿。URDF 与 MJCF 的动力学差异是
   未补偿的残差，本轮不处理。
6. **JTC 与多模式控制器**：两者分别占用 `position` 与 `effort` 命令接口，**ros2_control
   本身不阻止二者同时 active**。互斥只能由切换编排保证：本轮用
   `switch_controller(strictness=STRICT)`，并要求切换失败即整体失败。
7. **力矩模式下旧 actuator**：`perform_command_mode_switch` 在 `JointTorque` 分支调用
   `set_torque_control(...,1)`、`set_position_servo(...,0)`、`set_velocity_servo(...,0)`；
   这些函数写的是 `actuator_gainprm[0]`（**增益**），不是 `ctrl`。增益为 0 时位置/速度
   actuator 无论 ctrl 多旧都不出力，因此确实关闭。

## 3. 本轮对控制律的修改

`ComlessDualCartesianImpedanceController::computeTauImpl`：

```text
tau_task = Jᵀ·( -K·e + D·(Vd - J·q̇) )
tau      = tau_task + N_null + tau_coriolis + tau_collision + tau_manipulability
```

- 阻尼项改为作用在**参考速度与实际速度之差**上；原来只有 `-D·J·q̇`，等价于把移动参考
  当成静止。`Vd` 由新消息字段 `l_twist/r_twist` 提供（`DualCartesianImpedanceGoal` 最小扩展，
  接收端同步更新并拒绝非有限 twist）。
- 力矩保护（`utils/torque_limits.h`，纯函数、可测）：
  **双向**幅值限幅 `panda_limits::tau_max = (87,87,87,87,12,12,12) Nm`（Franka 公布的控制
  参数，与 MJCF 的 motor `ctrlrange` 一致），随后按
  `panda_limits::tauD_max = 1000 Nm/s × dt` 做变化率限制，`dt` 用控制周期实测墙钟时间；
  `dt` 非法时退回 1 ms 并封顶 100 ms。**没有使用统一的 10 Nm。**
- 非有限输入：不转发，**保持上一次力矩**（不清零，避免悬空时掉臂），并计数。
- 限幅前后量：每 500 个控制周期打印一次 `last_raw` / `last_applied` 与三个计数器。
- 未改：刚度/阻尼默认值、零空间项、碰撞/可操作性项、`panda_limits` 本身。

刚度/阻尼仍为**固定常值**（`K=diag(400,400,400,20,20,20)`、`damping_ratio=0.8`、
`nullspace_stiffness=10`），本轮不做方向性柔顺、不做参数扫描；两个独立阻抗环共享参考，
**不能称为已完成物体空间协调阻抗**。

## 4. 参考桥与切换

新增 `dual_impedance_no_load_bridge`（`dual_arm_carry_task`）：

- 用 `mj_left_and_mj_right/get_robot_states` 读当前 `O_T_EE` 与 q，**以实测状态初始化**目标
  （不是远端点，也不是离线重建的运输轨迹）。
- 参考在各自基座系中生成：两手共同沿 +z 抬 5 mm，姿态保持，5 阶 ramp（两端速度/加速度为 0），
  100 Hz 发布 pose + twist + 零空间关节参考。
- 切换：STRICT 切到阻抗 → 等目标话题出现订阅者（控制器确实激活）→ 保持 3 s → 移动 1 s →
  保持 3 s → STRICT 切回 JTC → **从当时实际关节状态**发一条 0→0.5 s 的保持轨迹。
- 失败即非零退出；不自动重试、不循环切换。夹爪全程未发任何目标。

## 5. 测试

`test_torque_limits`（5 项，全部通过）：双向幅值限幅使用声明的分级限值（而非统一 10 Nm）、
变化率限制在两个方向且随 `dt` 缩放、非有限输入保持上一力矩、上一力矩非有限时退回零、
非法 `dt` 有界回退。

其余既有测试（S1 计时/位姿、接触验收、夹爪协议、S2 几何优化器 13 项、时间剖面 6 项）仍通过。

## 6. 运行记录

- `..._r1`：失败。切换服务调用超时 —— 节点从未 spin 执行器，服务响应无人处理。
- `..._r2`：成功（链路验证），但未记录跟踪误差。
- `..._r3`：**本轮有效运行**，加入跟踪误差测量。
  全程最大 2.16 mm / 0.38°；力矩保护三个计数器均为 0；0 次目标拒绝。

命令与产物：

```bash
RUN_ID=<run_id> bash scripts/run_dual_impedance_no_load.sh --headless
# 产物：sim_dual_moveit.log、bridge.log、controllers_before.txt、controllers_after.txt
```

有效参数：`hold_seconds=3.0`、`move_amplitude_m=0.005`、`move_seconds=1.0`、
`final_hold_seconds=3.0`、`rate_hz=100.0`；`ROS_DOMAIN_ID=113`，headless，
单一仿真实例，退出后按 symlink/pid 清理本轮进程。

## 7. 已知限制

- **参考超时未在控制器内实现**：目标流停止时控制器保持最后目标（安全），但没有失效检测，
  也不会进入失败路径。本轮只有 bridge 侧的墙钟超时。
- **独立审阅**：按任务要求安排了一次聚焦审阅，但本轮会话的 subagent 被 Guard 拒绝
  （`AGENT_BUDGET_EXHAUSTED`），实际由执行器自查完成，**不构成真正独立审阅**。
  自查覆盖：资源互斥、补偿来源、残留 actuator、缓冲/线程交接、失败行为。
- `hasError()` 在本构建中无人消费，被拒目标只留下节流的 WARNING 日志，控制器不进入错误态。
- 激活瞬间 `start_()` 在切换服务线程里调用 `desired_.update()`，与 `Buffered<>` 的
  常规控制线程写者存在一个窄窗口；`content_` 的常规读写都在控制线程内，无持续竞争。
- URDF 与 MJCF 的 Coriolis 不完全一致，残差未补偿，表现为小的常值偏置力矩。
- 仅空载、仅一次；没有扰动、没有接触、没有滑移数据。
- 训练/演示用的固定刚度未做方向性设计，横向柔顺留待下一轮。
- **清理缺陷（已修）**：本轮脚本只传了 `headless:=true`，而 `sim_dual_moveit.launch.py` 里
  RViz 由独立的 `use_rviz` 开关控制，默认启动；RViz 不响应 SIGINT，导致 `ros2 launch`
  等待子进程无法退出，整套 launch 树（rviz2 / move_group / robot_state_publisher）留在后台。
  已用该 run 自己的进程组 SIGTERM 清理，并修改脚本：headless 时同时传 `use_rviz:=false`，
  清理改为 SIGINT → 等待 → SIGTERM → SIGKILL 的逐级回收。

## 8. 下一轮

接抓持后的悬空保持与扰动对照（2 N、0.2 s、横向、平滑起落），先刚性/阻抗同轨迹比较，
再决定是否进入正式 2×2 对照。本轮不进入。
