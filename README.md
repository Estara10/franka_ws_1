# 双 Franka 协作搬运仿真项目

本项目基于 ROS 2、MoveIt 2、ros2_control 和 MuJoCo，搭建双 Franka Panda 夹取并搬运铝杆的仿真系统。研究围绕三项内容展开：真实接触下的双臂搬运、运输轨迹的时间分配优化，以及笛卡尔阻抗与载荷补偿。

项目结题工作仅在 MuJoCo 中完成。这里的结果用于说明指定模型、场景和参数下的仿真表现，不代表真机实验或对其他场景的普遍保证。

## 研究内容与主要结果

### 1. 双臂搬运系统

系统将 MoveIt 2 用于运动规划与碰撞检查，ros2_control 负责控制器管理，MuJoCo 提供刚体动力学和指垫接触。任务由接近、夹取、抬升、运输、下降、释放和撤退组成；双臂围绕同一杆件参考生成末端目标，并通过协调逆运动学形成双臂运动。真实接触搬运实验关闭了手与杆之间的 weld 或杆固定约束，杆由夹爪接触承载。

固定场景的 80 次 A/B/C/D 对照中，A、B、D 各完成 20/20，C 完成 19/20。释放位置误差均值为 A 5.63 mm、B 6.25 mm、C 10.03 mm、D 10.53 mm。C、D 同时采用阻抗控制和载荷前馈，不能将其组间差异单独归因于某一种控制方法。逐次结果和指标定义见[80 次完整搬运对照](experiment_results/reports/abcd_n20_20261025/results.md)。

另有一组位置控制与“阻抗＋载荷前馈”的完整搬运性能分析，各分析 10 次有效完整运行。阻抗与载荷前馈组合将运输竖向误差 RMS 均值从 6.71 mm 降至 0.86 mm；释放水平误差从 5.51 mm 增至 9.00 mm，任务墙钟时长从 31.46 s 增至 35.21 s。这体现了承载跟踪与放置精度、运行时间之间的取舍。B 组另有两次释放后真值测量中断，未纳入这组有效性能统计，因此这些数据不表示全部任务尝试的成功率。详见[完整搬运性能分析](experiment_results/reports/fullcarry_perturb_ab_20261026_release_repair/valid_runs_analysis/results.md)。

### 2. 轨迹优化

时间分配优化在固定空间路径和固定总时长下，对单调时间映射进行参数搜索，以连续参考轨迹的 jerk 为优化指标。优化器使用 Bernstein 参数化时间映射和 Nelder–Mead 搜索。80 次对照中，B、D 组的连续参考 jerk RMS 相对各自基础参考平均下降 5.64% 和 4.58%；优化计算平均耗时约 4.23 s 和 4.78 s。实验没有显示运输时间或放置误差同步改善。这里的 jerk 是规划参考曲线指标，不是机械臂实测 jerk。方法与逐次统计见[主实验结果](experiment_results/reports/abcd_n20_20261025/results.md)及[补充对照](experiment_results/reports/explanatory_20261025/results.md)。

### 3. 柔顺控制

柔顺控制采用双臂独立笛卡尔阻抗，并可叠加沿世界竖直方向的杆载荷前馈。悬空保持专项中，前馈关闭与开启各有 10 次有效样本；3.5 s 时杆的平均竖向位移由 −9.084 mm 改善为 −2.997 mm，平均少下沉约 6.09 mm。方向性刚度专项在 2 N 水平脉冲下，将 X 向刚度从 400 N/m 降至 200 N/m 后，杆的脉冲末端让步增加约 0.515 mm；撤力后的残余也较大，体现出让步能力与恢复之间的取舍。

这些专项研究针对悬空保持工况。它们支持在该仿真条件下讨论载荷补偿和方向性刚度的作用，不单独证明完整搬运性能提升。样本数、统计方法及逐次数据见[柔顺控制专项结果](experiment_results/reports/compliance_hover_20261025_r1/results.md)。

## 构建与启动

目标环境为 Ubuntu 22.04、ROS 2 Humble、MoveIt 2 和 MuJoCo，并需满足仓库中各 ROS 包声明的依赖。

```bash
source /opt/ros/humble/setup.bash
cd /path/to/franka_ws_1
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
./start_dual_arm_task.sh
```

`sim_dual_moveit.launch.py` 已包含 MuJoCo，不要再启动第二个仿真实例。启动模式、参数及分步运行方法见[手动启动说明](MANUAL_STARTUP.md)和[任务包说明](src/multipanda_ros2/dual_arm_carry_task/README.md)。首次在另一台电脑运行时，应按该电脑的 ROS、MoveIt、MuJoCo 安装情况完成依赖配置和构建。

## 目录说明

- `src/multipanda_ros2/`：当前 ROS 2 主代码。
- `docs/`：架构说明、研究记录、交接和图表索引；建议从[项目指南](docs/PROJECT_GUIDE.md)开始阅读。
- `scripts/`：构建、实验数据分析及绘图脚本，具体用途见[scripts 说明](scripts/README.md)。
- `experiment_results/reports/`：汇总数据、统计结果和图表。10 月 27 日的重复批次仅作存档，未纳入当前选用结论，详见对应的归档说明。
- `experiment_results/a1_runs/`：本机 MuJoCo 逐次运行目录，体积较大，未上传 GitHub。
- `archive/`：旧版代码及历史资料，不参与当前工作区构建。
- `report/`：本机结题 Word 文件，含个人信息，已排除在 GitHub 仓库之外。

汇总报告中的逐次 CSV、统计表和图表随仓库保存；原始 MuJoCo 运行文件保留在本机或项目备份中。若需要复核某次运行，请使用对应批次的索引文件定位本地原始目录。

## 数据解释边界

- 本项目结果来自仿真，不是实体机器人测试。
- jerk 结果描述规划参考曲线；不能等同于机械臂实测 jerk。
- 阻抗与载荷前馈组合实验不能把组合效果单独归因于阻抗或前馈。
- 悬空保持专项和完整搬运实验是不同工况，应分别解释。
- 10 月 27 日的重复阻抗批次按项目决定仅存档，不纳入当前选用统计。
