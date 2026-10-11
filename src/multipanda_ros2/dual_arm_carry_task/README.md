# dual_arm_carry_task

当前双 Franka 协作搬运任务包。公共项目架构、柔顺定义和开发阶段见仓库根目录
[`docs/PROJECT_GUIDE.md`](../../../docs/PROJECT_GUIDE.md)。

## 节点与职责

- `dual_arm_carry_task_node`：任务参数、状态机和运动执行。
- `communication_latency_monitor_node`：基于带时间戳消息的通信延迟统计。
- `task_metrics_monitor_node`：双臂同步代理量、能量代理量和末端误差统计。
- `adaptive_compliance_supervisor_node`：外部笛卡尔阻抗参数监督器，默认关闭。

核心源文件：

- `src/carry_task_state_machine.cpp`
- `src/dual_arm_controller.cpp`
- `src/gripper_controller.cpp`
- `src/adaptive_compliance_supervisor.cpp`

状态机：

```text
INIT → APPROACH → GRASP → WAITING_PHYSICS → LIFT → TRANSPORT
     → ROTATE → DESCEND → PLACE → RETREAT → DONE / ERROR
```

## 构建

在仓库根目录执行：

```bash
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
```

## 启动

完整系统推荐由仓库根脚本启动：

```bash
./start_dual_arm_task.sh
```

如果 MoveIt、MuJoCo 和控制器已经就绪，可单独启动任务：

```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py
```

该 launch 的关键默认值如下：

| 参数 | launch 默认值 | 说明 |
|---|---:|---|
| `left_arm_group` | `mj_left_arm` | 左臂 MoveGroup |
| `right_arm_group` | `mj_right_arm` | 右臂 MoveGroup |
| `dual_arm_group` | `dual_panda` | 双臂 MoveGroup |
| `control_mode` | `compliant` | 旧控制模式名 |
| `enable_rotate` | `true` | 是否执行 ROTATE |
| `approach_height` | `0.28` | 接近高度，m |
| `grasp_height` | `0.13` | 抓取高度，m |
| `lift_height` | `0.40` | 抬升高度，m |
| `use_vision` | `false` | 实验性模式 B 开关 |
| `enable_latency_monitor` | `true` | 通信延迟监控 |
| `enable_task_metrics_monitor` | `true` | 任务指标监控 |
| `enable_adaptive_compliance_supervisor` | `false` | 外部阻抗参数监督器 |

节点源码本身的 `control_mode` fallback 为 `compliant_chomp`，但正常通过 launch 启动时会收到
launch 的 `compliant` 默认值；一键脚本会显式传入交互选择。`enable_rotate` 在两处均默认为
`true`。

示例：

```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant \
  enable_rotate:=true \
  approach_height:=0.28 \
  grasp_height:=0.13 \
  lift_height:=0.40
```

## 当前模式边界

现有旧模式名为 `rigid`、`compliant`、`chomp_only` 和 `compliant_chomp`：

- `rigid`：关闭 MuJoCo 硬件层内部柔顺。
- `compliant`：启用基于内力的自适应关节伺服增益。
- `chomp_only`、`compliant_chomp`：会选择不同轨迹参数，但核心任务目前没有真正调用 CHOMP。

外部监督器依赖多模式阻抗控制器，而模式 A 的 MoveIt 执行依赖位置轨迹控制器；二者不能同时
active。没有完成轨迹桥和安全控制器切换前，不应把监督器开关视作完整柔顺闭环。

## 监控输出

启动脚本把正式运行结果写入 `experiment_results`，包括：

- 通信延迟摘要与 CSV。
- 任务指标摘要与 CSV。
- 可选采集原始数据和绘图。

这些指标当前包含代理量：同步误差不是严格的时间同步误差，能量不是标定后的实际能耗，末端
误差也不是直接测得的杆最终位姿。实验结论必须同时保留指标定义。

## 调试

```bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /task_stage
ros2 topic echo /task_status
ros2 topic echo /joint_states --once
```

MoveIt 仿真 launch 已包含 MuJoCo，不要重复启动。完整手动顺序见仓库根
[`MANUAL_STARTUP.md`](../../../MANUAL_STARTUP.md)。

## 测试现状

本包已有 A1 采样、稳定判据和服务故障的离线测试；状态机、轨迹、夹爪及 PlanningScene
的完整回归仍不齐全。新增核心行为选择针对性单测、接口冒烟或相关仿真验证，不要求每次
修改补齐全系统测试。测试与审阅强度遵循 [项目指南](../../../docs/PROJECT_GUIDE.md)
的长期开发准则；非关键历史 lint/格式问题不阻断功能开发，未验证项如实说明。

## 许可证

Apache-2.0
