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

- **结题选图入口：[图片说明与分类](docs/figures/README.md)。** 4张主图统一在 `docs/figures/结题主图/`，细分结果放 `辅助图/`，旧版与探索图保留在 `历史归档/`。
2026-10-25当前结果：A/B/C/D四组有效搬运均为3/3。D3初始化时的状态查询预算不一致已修复，新增运行`final_abcd_20261025_D3_repaired1`实际完成抬升、运输、下降、释放及撤退。原D3在闭爪后、承载运动前中止的记录保持FAILED，单列于`excluded_initializations.csv`；全流程尝试与有效搬运样本分别统计，不将其解释为所有系统尝试均成功。运动、夹持、阻抗增益和承载保护参数未改。 B组连续参考jerk RMS/峰值平均下降6.63%/23.83%，D组三次平均下降4.07%/4.83%；释放平面误差均值A/B/C/D为5.43/5.88/10.11/10.48 mm。各组释放后统一等待约2 s仿真时间测量。这些jerk指标来自参考曲线，不是实测关节jerk。 见`docs/HANDOFF_20261025.md`及统一实验记录。本机报告第十二版已更新，`report/`不提交、不上传。
- 队友离线接续所需原始轨迹和记录已打包；拉取后按交接文档解压，无需为获取样本重新跑仿真。现有ABCD图表的统计修订项见交接开头，勿直接当作最终结题数值。
- **结题研究状态与边界见：[项目指南](docs/PROJECT_GUIDE.md)。**
- 固定场景无 weld 接触抓取已完成抬升、保持、运输/小角度旋转、释放和退回，仍有滑移。
- S2 几何投影目标下降约 1.47%；旧时间剖面实现的 3.15% 连续 jerk 数字只保留为修正前历史记录。
  新实现直接优化连续 jerk，并增加峰值、忠实导出和最终轨迹检查；正式12路径离线验收为12/12候选接受，逐次连续RMS变化约−6.57%至−3.42%，但新候选尚未执行。
- R/S/O 离线比较已完成：O/S RMS 中位改善约 `−4.86%`，未达到 `−5%` gate；因此不执行 O，后续按 R/S 结果整理图表和正文。
- R/S 四次完整搬运已完成（R1→S1→S2→R2）：四次 `DONE`，weld/约束关闭；计划运输时长均值 R/S 为
  `2.2366/2.6787 s`，动作墙钟均值为 `2.5412/3.2571 s`。O 因离线 gate 为 0 未执行。
- 最终时间—平滑度主图：[02_运输时间与平滑度.png](docs/figures/结题主图/02_运输时间与平滑度.png)；
  统一结论见 [20261022_结题收口_结果.md](docs/deliverables/20261022_结题收口_结果.md)。
- S3 无 weld 接触下的 JTC/MMC 悬空保持 A/B 各完成 3 次；每次均完成保持、切回和释放，
  MMC 杆世界 Z 位移中位数为 −8.48 mm。固定/方向性 MMC 的 +X 短脉冲对照各 3 次，未显示明确优势。
  该项是历史悬空保持结果；随后已完成固定场景MMC完整搬运，最新记录见10月22日交接，尚不支持性能全面优于JTC。
- A1 真值准备、静态与预抓取路径检查已具备；随机场景全过程统计尚未完成。

当前结题主图为完整搬运对照、运输时间与平滑度、载荷补偿减少下沉、方向性柔顺与受力让步，
文件与使用说明见[图片目录](docs/figures/README.md)。早期单样本优化、未补偿悬空保持及旧版统计图均已归档。
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
最新代码状态、实验结论和明日入口见
[`docs/HANDOFF_20261022.md`](docs/HANDOFF_20261022.md)。接续本次消息/控制器变更请按交接文档重新构建；
[`10月21日记录`](docs/HANDOFF_20261019.md)作为历史交接保留。

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
