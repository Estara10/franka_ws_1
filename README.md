# 双臂 Franka 协作搬运

两台 Franka Panda 在 MuJoCo 中协同夹持并搬运铝杆，重点研究双臂协调轨迹规划、轨迹优化
和柔顺控制。技术栈为 ROS 2 Humble、MoveIt 2、ros2_control 和 MuJoCo。

本项目是大学生创新创业训练项目，优先推进学习研究、核心算法、功能验证与成果展示。
所有 AI Agent 的长期开发准则以 [`项目指南`](docs/PROJECT_GUIDE.md) 为准，采用与任务
风险相称的测试和审阅，避免过度工程化。

本次结题仅进行 MuJoCo 仿真验证，不开展实际机器测试。修订后的结题目标与实验评价方案见
[`结题目标修订说明`](docs/deliverables/结题目标修订说明.md)。

当前唯一开发主线位于 `src/multipanda_ros2`。早期 `src/dual_arm_*` 四包已移动到
[`archive/`](archive/README.md)，不再参与构建。

## 当前状态

- **结题研究状态与边界见：[项目指南](docs/PROJECT_GUIDE.md)。**
- 固定场景无 weld 接触抓取已完成抬升、保持、运输/小角度旋转、释放和退回，仍有滑移。
- S2 几何投影目标下降约 1.47%；固定几何时间剖面在一致的连续时间口径下 jerk RMS 下降约
  3.15%。两项均为单个重建离线样本，未显示明确执行收益。
- S3 无 weld 接触下的 JTC/MMC 悬空保持 A/B 各完成 3 次；每次均完成保持、切回和释放，
  MMC 杆世界 Z 位移中位数为 −8.48 mm。固定/方向性 MMC 的 +X 短脉冲对照各 3 次，未显示明确优势。
  结果证明悬空保持链路可运行，不代表 MMC 完整搬运或性能优于 JTC。
- A1 真值准备、静态与预抓取路径检查已具备；随机场景全过程统计尚未完成。

结题主图：S2 离线指标 `docs/figures/s2_offline_metrics_single_sample_20261020.png`；
JTC/MMC 悬空位移 `docs/figures/hover_ab_vertical_displacement_20261020.png`；
固定/方向性脉冲 `experiment_results/reports/hover_directional_perturbation_20261020/mmc_directional_perturbation_comparison.png`。
指标口径、逐次结果和限制见 [本机 S2/S3 对照记录](docs/CONTACT_HOVER_AB_COMPARISON_20261020.md)
及 [S2 几何投影结果](docs/deliverables/P6_S2_projection_optimizer_result_20261017.md)、
[S2 时间剖面结果](docs/deliverables/P6_S2_time_profile_result_20261017.md)。

详细架构、阶段顺序和已知风险见 [`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)。

## 环境与构建

目标环境为 Ubuntu 22.04、ROS 2 Humble、MoveIt 2 和 MuJoCo。已有工作区可按需构建：

```bash
cd /home/lidio/franka_ws_1
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
```

全量构建可使用 `colcon build`。构建会写入 `build/`、`install/` 和 `log/`。

本次 Ubuntu 22.04 迁移的 MuJoCo/DQ Robotics 版本选择和便捷构建命令见
[`docs/LOCAL_BUILD.md`](docs/LOCAL_BUILD.md)。当前本机建议运行 `bash scripts/build_local.sh`。
2026-10-19 的最新代码状态、实验结论和明日入口见
[`docs/HANDOFF_20261019.md`](docs/HANDOFF_20261019.md)。接续本次消息/控制器变更请按交接文档重新构建。

## 启动

推荐从仓库根目录执行：

```bash
./start_dual_arm_task.sh
```

脚本提供：

- 模式 A：固定场景，以及 `rigid`、`compliant`、`chomp_only`、`compliant_chomp` 旧模式名。

一键脚本当前会检查源码时间并可能自动构建，也会创建日志。
这些行为属于已记录技术债。运行前建议先确认 `git status`，不要把它当作纯只读启动器。

MoveIt 启动文件已经自动包含 MuJoCo。不要再单独启动第二个 MuJoCo 实例。需要分步调试时见
[`MANUAL_STARTUP.md`](MANUAL_STARTUP.md)。

## 状态机

```text
INIT → APPROACH → GRASP → WAITING_PHYSICS → LIFT → TRANSPORT
     → ROTATE → DESCEND → PLACE → RETREAT → DONE
```

任务节点、参数和报告说明见
[`src/multipanda_ros2/dual_arm_carry_task/README.md`](src/multipanda_ros2/dual_arm_carry_task/README.md)。

## 常用只读检查

加载环境后可检查：

```bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

正常停止请优先在启动终端按 `Ctrl+C`。仓库现有启动脚本仍包含较宽泛的强制进程清理，
在共享 ROS 环境中使用前需要确认不会影响其他会话。

## 数据、文档与工具状态

- `experiment_results/`：本阶段原样保留，包括原始数据、图表和既有报告。
- `docs/deliverables/`：项目交付物，例如中期检查报告。
- `archive/`：旧包、历史修补代码、旧文档和运行产物；有 `COLCON_IGNORE` 隔离。
- `check_environment.sh`、`diagnose.sh`：遗留脚本，含旧工作区路径和失效入口；不被当前流程调用，
  可由用户确认后删除。
- `scripts/`：实验记录、离线绘图、指标比较和实时面板。完整用途和稳定命令见
  [`scripts/README.md`](scripts/README.md)。
- `run_latency_benchmark.sh`、`run_task_metrics_benchmark.sh`：会连续启动完整仿真实验的基准脚本，
  不是普通只读分析命令。

## 铝杆参数说明

项目目标质量为 0.5 kg，尺寸为 40 cm × 4 cm × 4 cm。当前 MuJoCo 模型按尺寸和密度计算
约为 0.64 kg；本阶段没有修改物理模型，后续仿真阶段将统一为 0.5 kg 并重新验证实验。

## 许可证

各 ROS 包沿用其包内声明的许可证；第三方上游组件以各自许可证为准。
