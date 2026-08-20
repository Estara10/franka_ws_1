# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

双臂 Franka Panda 协作搬运任务 —— 两台 Panda 机械臂协同夹持一根铝合金杆（40cm × 4cm × 4cm，0.5kg），从初始位置搬运到目标位置并旋转放置。基于 ROS 2 Humble + MuJoCo 物理仿真 + MoveIt 2 运动规划。

## 常用命令

```bash
# 编译全部包
colcon build

# 单独编译（修改源码后）
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config

# 加载环境（每次新终端必须执行）
source install/setup.bash

# 一键启动（模式A：经典搬运）
./start_dual_arm_task.sh

# 单独启动 MuJoCo 仿真（调试用）
ros2 launch franka_bringup dual_franka_sim.launch.py

# 单独启动 MoveIt2（会自动启动 MuJoCo）
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=true

# 单独启动任务节点
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py control_mode:=compliant_chomp

# 查看节点/话题/控制器
ros2 node list
ros2 control list_controllers
ros2 topic echo /joint_states --once

# 停止所有进程
pkill -9 mujoco_node; pkill -9 ros2; pkill -9 rviz2
```

## 架构

### 两层架构

**底层：仿真 + 运动规划**
- MuJoCo 物理仿真（`franka_hardware` 的 `FrankaMJHardwareSystem` 作为 ros2_control 插件）
- MoveIt 2 + Pilz 工业运动规划器（PTP） + TOTG 时间参数化 + Ruckig 加加速度平滑
- `dual_panda_arm_controller`：JointTrajectoryController，控制全部 14 个关节（双臂各 7 轴）

**上层：任务状态机**（`dual_arm_carry_task` 包）
- FSM: INIT → APPROACH → GRASP → WAITING_PHYSICS → LIFT → TRANSPORT → ROTATE → DESCEND → PLACE → RETREAT → DONE
- 四个核心 C++ 类：`DualArmCarryTask`（状态机主循环）、`DualArmController`（2044行，核心运动逻辑）、`GripperController`（夹爪控制）、`AdaptiveComplianceSupervisor`（自适应柔顺监督）
- 两个 Python 监控节点：`communication_latency_monitor`（通信时延）、`task_metrics_monitor`（同步误差/能耗/跟踪精度）

### 四种控制模式（模式A子模式）

| 模式 | 内部柔顺 | CHOMP 优化 |
|------|---------|-----------|
| `rigid` | 关 | 关 |
| `compliant` | 开 | 关 |
| `chomp_only` | 关 | 开 |
| `compliant_chomp`（默认）| 开 | 开 |

内部柔顺由 MuJoCo 硬件层环境变量 `MJ_INTERNAL_COMPLIANCE_*` 控制，非 ROS 话题。

### 模式 B（视觉动态抓取）

基础设施存在但未完成。`vision_core.py` 桥接 MuJoCo 顶置相机话题 `/cameras/overhead_camera/rgb/image_raw` → `/overhead_camera/image_raw`，预留了 OpenCV 图像处理接口。`start_dual_arm_task.sh` 支持模式 B 的参数输入（初始坐标、目标坐标、偏航角），但对应 C++ 动态规划管线尚未实现。

### 关键源文件

```
src/multipanda_ros2/dual_arm_carry_task/
├── src/carry_task_state_machine.cpp    # main(), FSM 主循环
├── src/dual_arm_controller.cpp         # 2044行——每个状态的具体运动实现
├── src/gripper_controller.cpp          # 同步控制双臂夹爪
├── src/adaptive_compliance_supervisor.cpp  # 力反馈自适应阻抗调节
├── src/communication_latency_monitor.cpp   # 通信时延统计
├── src/task_metrics_monitor.cpp        # 同步误差/能耗/跟踪精度
├── include/dual_arm_carry_task/        # 对应头文件
└── launch/dual_arm_carry_task.launch.py    # 任务启动文件（489行，~50参数）

src/multipanda_ros2/franka_moveit_config/
├── launch/sim_dual_moveit.launch.py    # MoveIt2 启动（自动包含 MuJoCo）
├── config/dual_panda.srdf              # 规划组定义（panda_1, panda_2, dual_arm）
├── config/kinematics.yaml              # KDL 运动学求解器
└── config/moveit_controllers.yaml      # 控制器映射

src/multipanda_ros2/franka_description/
└── mujoco/franka/objects.xml           # 铝杆模型定义（物体位置在此修改）

src/multipanda_ros2/franka_hardware/
└── src/sim/franka_mj_hardware_system.cpp  # MuJoCo ↔ ros2_control 硬件接口
```

### 启动顺序（重要）

1. `sim_dual_moveit.launch.py` 启动 MoveIt2，该 launch 文件**自动包含 MuJoCo 启动**
2. **不要重复启动 MuJoCo**，否则节点冲突
3. 等待约 10-15 秒让 move_group + mujoco_node + controller_manager 就绪
4. 启动 `dual_arm_carry_task` 任务节点

### 控制器冲突（关键约束）

`dual_panda_arm_controller`（位置轨迹控制器）和 `sim_multi_mode_controller`（阻抗控制器）**不能同时 active**。经典搬运流程依赖前者，自适应柔顺监督依赖后者。默认配置下 `adaptive_compliance_supervisor` 关闭以避免冲突。

### 碰撞矩阵

SRDF 由 `dual_panda.srdf.xacro` 通过 xacro 生成，包含 ~50+ 条 `disable_collisions` 规则。运行时 `DualArmController::initializeCollisionMatrix()` 进一步通过 `AllowedCollisionMatrix` 动态追加 arm-to-arm 规则。夹取阶段调用 `allow_gripper_collision(true)` 临时放行夹爪-物体碰撞。

### 抓取策略

双臂夹紧后发布 `grasp_weld_active=true` 激活 MuJoCo 焊接约束（刚体绑定），同时调用 MoveIt `attachObject()` 将铝杆作为 `AttachedCollisionObject` 附加到左臂夹爪上。放置时逆操作：解除 weld、detach、恢复为世界碰撞对象。

### 物体坐标系约定

- 铝杆中心初始位于 base_link 系 `(0.5, 0.0, 0.02)`
- 夹爪间距对称：左臂从 y=+0.15，右臂从 y=-0.15 接近（相对杆中心）
- Z 轴参考：approach=0.28m，grasp=0.13m，lift=0.40m

## 补充工具脚本

根目录 Python 脚本（`plot_*.py`、`compare_task_metrics.py`、`experiment_data_recorder.py`、`sensor_dashboard.py`）用于实验后数据分析和实时监控，非任务运行时必要组件。`scripts/patches/` 存放历史修复脚本，非活跃代码。
