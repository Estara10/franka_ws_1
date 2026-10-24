# 根目录 Python 工具说明

结题图片位置统一见[图片说明](../docs/figures/README.md)：`plot_report_figures_20261024.py`使用已有记录生成4张中文主图，写入 `docs/figures/结题主图/`（PNG/SVG），不启动仿真。原技术分析脚本输出至 `历史归档/技术分析版/`，细分图输出至 `辅助图/`；`analyze_s2s3_extended.py` 的 `--fig-dir` 指定分类目录共同根目录，默认 `docs/figures`。

本目录集中存放项目级 Python 辅助工具。所有默认输入、输出路径均由脚本位置反推仓库根目录，
因此不依赖调用时的当前目录。`start_dual_arm_task.sh` 和 benchmark 脚本也使用绝对定位调用它们。

## 根目录 Shell 文件

- `start_dual_arm_task.sh`：当前唯一一键入口，启动模式 A/B、MoveIt/MuJoCo、任务节点、可选记录器
  和后处理。它还可能自动构建、修改模式 B 场景文件并清理进程，运行前应检查 Git 状态。
- `run_latency_benchmark.sh`：启动一次完整模式 A 实验，读取最新通信延迟报告并按阈值生成验收摘要。
- `run_task_metrics_benchmark.sh`：依次启动两种模式 A 实验，再调用本目录的指标比较工具。
- `check_environment.sh`：早期环境检查脚本，绑定已废弃工作区和不存在的启动入口；当前流程不调用，
  建议用户确认后删除。
- `diagnose.sh`：早期运行诊断脚本，绑定已废弃日志目录和工作区；当前流程不调用，建议用户确认后删除，
  如未来需要诊断工具应按现有节点、控制器和日志结构重新实现。

## 运行时记录与监控

- `experiment_data_recorder.py`：订阅双腕力、关节状态、自适应参数和任务状态，以固定频率记录
  `experiment_results/data/experiment_<mode>_<timestamp>.csv`。由一键脚本按开关自动启动。
- `sensor_dashboard.py`：可选 Tk 实时面板，显示双腕受力、内力代理、关节加速度范数和参数缩放；
  任务完成时也会保存 CSV。需要显示环境，不是主任务成功的必要条件。

## 离线分析

- `analyze_hover_ab.py`：重建固定场景 JTC/MMC 3 s HOLD 的逐次指标与图表。杆/手 ΔZ 为相对窗口入口的世界坐标位移；旧 task `*_slip_m` 字段在输出中标作抓取参考残差，不是局部接触滑移。
- `plot_mmc_directional_perturbation.py`：从六次固定/方向性 MMC 扰动运行生成 pulse-aligned 原始图、施力前局部线性漂移扣除图和逐次汇总 CSV。去趋势结果依赖局部线性趋势延续假设。
- `plot_s2_offline_summary.py`：生成 S2 单个重建离线样本的几何目标与连续时间 jerk RMS 前后图；没有重复运行范围，也没有执行数据。
- `analyze_time_profile_alignment_20261023.py`：读取正式 12 个 ABCD 运行的基础运输轨迹，调用
  `time_profile_replay` 执行修正后的连续目标优化、忠实导出和最终接受检查；结果写入
  `experiment_results/reports/final_tradeoff_20261022/time_profile_alignment/`。运行前需先构建
  `dual_arm_carry_task`，该脚本不启动 MuJoCo。
- `analyze_time_profile_rs_o_20261023.py`：对同一 12 个基础运输轨迹分别计算 R 原时长、S 统一
  1.2 倍伸时和 O 同 1.2 倍预算的时间优化，生成逐运行 CSV、组汇总、gate CSV 和图
  `docs/figures/辅助图/03_时间分配优化对照.png`；不启动 MuJoCo，gate 关闭时仍保留 R/S 结果。
  `time_profile_replay` 的 `--duration-scale` 只改变目标时长，不改变几何路径；`1.0` 用于 R，
  `1.2` 用于 S/O。
- R/S 完整搬运执行使用 `CONTACT_TRANSPORT_TIME_SCALE=1.0/1.2`，只作用于运输主段；默认
  `CONTACT_TRANSPORT_STRETCH=2.0` 保持不手工改成 `2.4`。运行前确认
  `CONTACT_PROBE_ABCD_GROUP=none`，并在每次运行后保存 `contact_transport_planned.csv` 和
  `contact_transport_meta.txt`。
- `analyze_rs_execution_20261023.py`：汇总 R1/S1/S2/R2 四次完整搬运的完成状态、计划/动作运输时长、
  释放误差、杆底面间隙、抓取参考残差和总耗时，结果写入
  `experiment_results/reports/final_tradeoff_20261022/rs_execution/`。
- `plot_final_time_smoothness_20261023.py`：合并 R/S/O 离线结果与 R/S 四次执行结果，生成最终
  `fig_time_smoothness_final.png/svg`。`scientific_style.py` 提供统一的 Science/Nature 风格、
  配色和中文 fallback 字体；本机没有 Microsoft YaHei 时使用 Noto Sans CJK，避免中文方框。

- `plot_experiment_results.py`：读取最新实验 CSV 或 task-metrics 同步样本，生成整周期分析图；
  一键脚本会在任务结束后自动调用。
- `compare_task_metrics.py`：比较两份 task-metrics 汇总 CSV，输出阈值判断 CSV/Markdown；
  `run_task_metrics_benchmark.sh` 会调用它。
- `plot_comm_latency.py`：为最近最多 10 份通信延迟样本分别生成时序图。
- `plot_comm_latency_stats.py`：聚合多次通信延迟实验，输出均值、P95 等统计图和 CSV。
- `plot_mode_comparison.py`：从四种历史模式的最新记录生成对比图和综合评分。当前模式名仍是历史标签，
  其中 `chomp_only`、`compliant_chomp` 不能作为已经真正运行 CHOMP 的证据。

从任意目录均可使用绝对路径，例如：

```bash
python3 /home/lidio/franka_ws_1/scripts/plot_experiment_results.py
python3 /home/lidio/franka_ws_1/scripts/plot_comm_latency_stats.py
python3 /home/lidio/franka_ws_1/scripts/plot_mode_comparison.py
```

涉及 ROS 订阅的记录器和面板应先加载工作空间环境。离线绘图工具需要现有 Python 绘图库，
本次整理不安装任何依赖。

## 实验性视觉

- `vision_core.py`：模式 B 顶置相机检测框架，发布 `/detected_rod_pose`。当前没有 CameraInfo/可靠
  TF 标定、角度稳定与位姿冻结，不能作为完成的视觉管线。

该文件由 `start_dual_arm_task.sh` 的模式 B 分支自动定位，不需要手工修改路径。
