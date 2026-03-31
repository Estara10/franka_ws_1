# 双臂协作搬运任务

## 功能概述

本包实现 Franka 双臂协作搬运任务，已包含多阶段状态机、控制模式分支、任务阶段发布与量化评估链路。

当前主任务状态机如下：

1. INIT
2. APPROACH
3. GRASP
4. LIFT
5. TRANSPORT
6. ROTATE（可选）
7. DESCEND
8. PLACE
9. RETREAT
10. DONE / ERROR

**当前代码采用了 MVC 组件化架构**，核心定义见 `include/dual_arm_carry_task/carry_task_context.hpp` 文件中的 `TaskState` 枚举，具体行为实现在 `src/dual_arm_controller.cpp` 和 `src/gripper_controller.cpp`。

## 快速启动

### 推荐方式：一键脚本

在仓库根目录执行：

```bash
./start_dual_arm_task.sh
```

脚本会自动完成：

1. 环境加载与残留进程清理
2. MoveIt2 + MuJoCo 启动
3. 任务节点启动
4. 通信时延监测与任务指标监测（默认启用）
5. 可选 GUI 采集（ENABLE_SENSOR_GUI=true 时）

### 直接启动任务 launch

```bash
source install/setup.bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py
```

## 常用参数

```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant_chomp \
  enable_rotate:=false \
  approach_height:=0.28 \
  grasp_height:=0.13 \
  lift_height:=0.40
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| left_arm_group | mj_left_arm | 左臂 MoveGroup 名称 |
| right_arm_group | mj_right_arm | 右臂 MoveGroup 名称 |
| dual_arm_group | dual_panda | 双臂协同组名称 |
| control_mode | compliant | 控制策略模式（rigid/compliant/chomp_only/compliant_chomp 等） |
| enable_rotate | false | 是否执行 ROTATE 阶段 |
| approach_height | 0.28 | 接近高度（m） |
| grasp_height | 0.13 | 抓取高度（m） |
| lift_height | 0.40 | 抬升高度（m） |

## 监测与报告

默认会输出到专门结果目录 experiment_results：

1. 通信时延报告：experiment_results/reports/comm_latency_*_summary.md / .csv
2. 任务量化指标报告：experiment_results/reports/task_metrics_*_summary.md / .csv
3. GUI 采集原始数据：experiment_results/data/experiment_*.csv
4. 离线分析图：experiment_results/plots/*.png

若启用 GUI 采集，脚本会自动调用 plot_experiment_results.py 生成图表到 experiment_results/plots。

## 包内节点

`dual_arm_carry_task_node` 主任务节点现已重构为 MVC 架构，其主要源文件包含：

1. `carry_task_state_machine.cpp`：主任务状态机与节点生命周期管理核心
2. `dual_arm_controller.cpp`：双臂 MoveIt 运动学规划、轨迹插补及控制逻辑
3. `gripper_controller.cpp`：双侧 Franka 夹爪 Action Client 同步抓取控制器

其他独立的监测节点包括：

4. communication_latency_monitor.cpp：通信时延统计节点
5. task_metrics_monitor.cpp：同步误差/能耗代理/末端误差统计节点
6. adaptive_compliance_supervisor.cpp：主从+自适应阻抗闭环监督节点

## 故障排查

### 规划失败

常见原因：目标超工作空间、碰撞约束、IK 无解。

建议：

1. 先降低目标动作幅度（尤其 TRANSPORT/DESCEND）。
2. 检查 enable_rotate 与控制模式组合。
3. 对照任务日志中阶段切换与 MoveIt 报错定位。

### 仿真未就绪

若任务节点启动后无动作，先检查：

```bash
ros2 node list
ros2 service list | grep controller_manager
```

### 报告未生成

检查 task_status 是否发布 DONE/ERROR，以及 launch 中监测节点开关：

1. enable_latency_monitor
2. enable_task_metrics_monitor

## 许可证

Apache-2.0
