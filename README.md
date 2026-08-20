# 双臂 Franka 协作搬运

两台 Franka Panda 在 MuJoCo 中协同夹持并搬运铝杆，重点研究双臂协调轨迹规划、轨迹优化
和柔顺控制。技术栈为 ROS 2 Humble、MoveIt 2、ros2_control 和 MuJoCo。

当前唯一开发主线位于 `src/multipanda_ros2`。早期 `src/dual_arm_*` 四包已移动到
[`archive/`](archive/README.md)，不再参与构建。

## 当前状态

- A0 固定场景的完整状态机已有成功运行记录，是当前回归主线。
- 轨迹仍包含手工逐点 IK，完整碰撞检查和真实 CHOMP 优化尚待实现。
- 当前内部柔顺是 MuJoCo 硬件层的自适应关节伺服增益。
- 外部笛卡尔阻抗监督器默认关闭，完整轨迹桥和控制器切换链尚待实现。
- A1 可复现随机场景尚待开发。
- 模式 B 视觉仅保留实验框架，不属于当前验收主线。

详细架构、阶段顺序和已知风险见 [`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)。

## 环境与构建

目标环境为 Ubuntu 22.04、ROS 2 Humble、MoveIt 2 和 MuJoCo。已有工作区可按需构建：

```bash
cd /home/lidio/franka_ws_1
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
```

全量构建可使用 `colcon build`。构建会写入 `build/`、`install/` 和 `log/`。

## 启动

推荐从仓库根目录执行：

```bash
./start_dual_arm_task.sh
```

脚本提供：

- 模式 A：固定场景，以及 `rigid`、`compliant`、`chomp_only`、`compliant_chomp` 旧模式名。
- 模式 B：视觉动态抓取框架。

一键脚本当前会检查源码时间并可能自动构建，也会创建日志；模式 B 还会修改 MuJoCo 场景源码。
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
- `check_environment.sh`、`diagnose.sh`：暂时保留，但仍含旧工作区路径，后续修复前不是权威入口。
- `plot_*.py`、`compare_task_metrics.py`、`experiment_data_recorder.py`、`sensor_dashboard.py`：
  实验分析与监控工具，不是状态机运行的必要组件。

## 铝杆参数说明

项目目标质量为 0.5 kg，尺寸为 40 cm × 4 cm × 4 cm。当前 MuJoCo 模型按尺寸和密度计算
约为 0.64 kg；本阶段没有修改物理模型，后续仿真阶段将统一为 0.5 kg 并重新验证实验。

## 许可证

各 ROS 包沿用其包内声明的许可证；第三方上游组件以各自许可证为准。
