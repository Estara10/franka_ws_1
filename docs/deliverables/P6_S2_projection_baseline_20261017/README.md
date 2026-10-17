# P6/S2 投影优化基线快照（2026-10-17）

本目录是本轮 S2 运输段闭链投影优化**修改前**的轻量工作区快照。当前工作树不是
`HEAD`：`HEAD` 之外还有未提交的接触搬运实现与本轮文档，因此仅用 `HEAD` 或
`diff --stat` 无法恢复本轮实际基线。

## 1. 快照内容

| 文件 | 内容 |
|---|---|
| `HEAD.txt` | 基准提交 `0b92fa193da6f823a50ef345cc938a476e0f6541`（分支 `main`） |
| `git_status.txt` | 修改前完整 `git status --porcelain` |
| `task_files.patch` | 任务相关已跟踪文件相对 `HEAD` 的补丁（含暂存与未暂存） |
| `untracked/docs/` | 参与本轮的未跟踪文档副本（S2/S3 计划、Codex 咨询提示词、接触批量与 P4/S2b 结果） |

补丁覆盖：`src/multipanda_ros2/dual_arm_carry_task`（含 `contact_load_probe.cpp`、
`dual_arm_controller.cpp`、状态机、上下文、launch、CMake）、
`src/multipanda_ros2/franka_hardware/src/sim/franka_mj_hardware_system.cpp`、
`scripts/a1_run_scene.sh`、`scripts/run_contact_probe_local.sh`、
`docs/CONTACT_HANDOFF_20261016.md`。

不复制整个 `experiment_results` 树，不做全量哈希。本快照不提交、不推送。

## 2. 冻结的比较基线参数

修改前唯一改动就是本轮新增的 S2 模块；基线本身使用既有成功配置：

| 项 | 值 | 来源 |
|---|---|---|
| 场景 | 固定初始位姿 `(0.50, 0.00, +10°)`，目标 `(0.40, 0.00, +20°)` | `run_contact_probe_local.sh` 的 `--x 0.50 --y 0.0 --yaw-deg 10.0` |
| 抬升 / 保持 | 0.40 m / 2.0 s | `contact_load_probe.cpp` 常量 |
| 夹持力 | 10 N/指（腱执行器名义 0.90×2F） | 同上 |
| 运输拉伸 | **1×**（`CONTACT_TRANSPORT_STRETCH=1.0`，既有机制） | P4/S2b 计时修正 |
| 内部自适应柔顺 | 关闭（`CONTACT_PROBE_RIGID=true`） | `run_contact_probe_local.sh` |
| IPTP 速度/加速度缩放 | 运输 `vscale=1.0, ascale=0.3`；抬升 `1.0/0.4` | `contact_load_probe.cpp` 的 `makePlan` 调用 |
| 时间伸缩规则 | `s1::scaleTimedTrajectory` + `stretch=max(min_total, min_total/T, 0.02/Δt_min)` | 未修改 |
| 双手一致性判定 | 5 mm / 2° | `pos_tol` / `angle_tol` |
| IK 航点残差 | 0.5 mm / 0.1° | `makePlan` |
| 航点步长 | 5 mm / 1°，最少 20 段 | `kPathSteps` |
| 关节跳变上限 | 0.15 rad/段；插值细分 0.01 rad | `makePlan` |
| 滑移窗口 | 20 mm（用户授权，未改动） | `lift_slip_tol` |

本轮没有拆分或重构 `min_total`，没有改计时公式。

## 3. 输入来源

本轮**没有**现成的、按关节名排列的 14 关节运输段实测路径：既有成功 run 只保存
`contact_probe.csv`（杆位姿、双手位姿、接触/力）与 `plan_compare.csv`（推断杆位姿），
不含运输计划的关节航点；`p2b_path.csv` 是抓取前路径，不能当作运输路径。

因此运输段的离线样本由现有代码重建，并在日志中标记为
`source=reconstructed_offline_planning_sample measured=false`：

- 起点：A1 P2a 标称双臂抓取解（`a1_verify_.grasp`），经同名关节组映射到 live 模型，
  并核对 FK 与 A1 抓取位姿的残差；
- `lifted`：同一份标称解沿物体 +Z 抬升 0.40 m 的标称抬升计划末点；
- 物体参考路径与 `G_L/G_R`：由冻结真值位姿与上述标称解按 `makePlan` 同一公式生成；
- 模型/判据场景/ACM/四指状态/支撑面：与 `runContactLoadProbe` 完全相同（live 模型求解、
  hand-AABB criterion 模型检查）。

该样本不是某次运行的实测原始路径，不能当作历史运行轨迹引用。

## 4. 复现命令

```bash
# 构建
colcon build --packages-select dual_arm_carry_task --cmake-args -DBUILD_TESTING=ON

# 单元测试（纯数学，不含机器人模型）
./build/dual_arm_carry_task/test_coordinated_path_optimizer

# 离线运输段对照（仅规划，不运动；单实例 MuJoCo + MoveIt）
CONTACT_PROBE_RUN_ID=<run_id> bash scripts/run_transport_opt_offline.sh --headless
```

`run_transport_opt_offline.sh` 固定 `CONTACT_TRANSPORT_PLAN_ONLY=1` 与
`CONTACT_TRANSPORT_STRETCH=1.0`，其余继承 `run_contact_probe_local.sh`。
