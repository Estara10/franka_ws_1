# 根目录 Python 工具说明

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
