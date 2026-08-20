# 快速启动指南

## 一句话启动

```bash
cd ~/franka_ws && ./start_interactive_sim.sh
```

然后在 RViz 中拖动机器人，点击 "Plan & Execute" 开始！

---

## 系统配置修复清单

✅ **已修复的文件：**

1. **[config/sim/dual_sim_controllers.yaml](franka_bringup/config/sim/dual_sim_controllers.yaml)**
   - 修改：`command_interfaces: [effort]` → `command_interfaces: [position]`
   - 位置：第 148 行

2. **[config/sim_dual_panda_ros_controllers.yaml](franka_moveit_config/config/sim_dual_panda_ros_controllers.yaml)**
   - 修改：`command_interfaces: [effort]` → `command_interfaces: [position]`
   - 移除：PID gains 配置（不适用于 position 控制）

3. **[config/dual_panda_ros_controllers.yaml](franka_moveit_config/config/dual_panda_ros_controllers.yaml)**
   - 修改：同上
   - 用途：真实机器人配置

---

## 最常用的 5 个命令

```bash
# 1. 启动完整系统（仿真 + MoveIt2 + RViz）
cd ~/franka_ws && ./start_interactive_sim.sh

# 2. 检查控制器状态
ros2 control list_controllers

# 3. 监听关节状态（验证机器人在动）
ros2 topic echo /joint_states --rate=10

# 4. 停止所有进程
pkill -f "ros2 launch"

# 5. 重新编译修改后的包
cd ~/franka_ws && colcon build --packages-select franka_bringup franka_moveit_config
```

---

## RViz 中的 3 个关键按钮

在右侧 **MotionPlanning** 面板中：

| 按钮 | 功能 | 何时使用 |
|------|------|--------|
| **Plan** | 仅规划轨迹（显示彩虹线） | 检查轨迹是否合理 |
| **Execute** | 执行已规划的轨迹 | Plan 成功后按此键 |
| **Plan & Execute** | 一键规划并执行 | 99% 的时间都用这个 |

---

## 故障排查速查表

| 问题 | 症状 | 解决方案 |
|------|------|--------|
| 看不到交互标记 | RViz 中没有彩色箭头 | ☑️ Interactive Markers → 按 R 键 |
| 规划失败 | "Unable to plan" | 目标位置更近 → 增加 Planning Attempts |
| 规划成功但不动 | 轨迹显示但机器人不动 | `ros2 control list_controllers` 检查激活状态 |
| 仿真崩溃 | 启动后立即退出 | `colcon build` 重新编译 |

---

## 文件位置速查

| 用途 | 路径 |
|------|------|
| 仿真控制器配置 | `src/multipanda_ros2/franka_bringup/config/sim/dual_sim_controllers.yaml` |
| MoveIt2 仿真配置 | `src/multipanda_ros2/franka_moveit_config/config/sim_dual_panda_ros_controllers.yaml` |
| 真实机器人配置 | `src/multipanda_ros2/franka_moveit_config/config/dual_panda_ros_controllers.yaml` |
| 启动脚本 | `./start_interactive_sim.sh` |
| 本指南 | `src/multipanda_ros2/RVIZ_INTERACTIVE_GUIDE.md` |
| 问题排查 | `src/multipanda_ros2/CONTROLLER_TROUBLESHOOTING.md` |

---

## 关键概念一览

### 为什么要改成 Position 接口？

```
Effort Control (力/扭矩控制)
    ↓ 用于直接发送扭矩命令
    ↓ 需要 PID 增益参数
    ↗ 不支持轨迹跟踪

Position Control (位置控制)
    ↓ 用于发送目标关节角度
    ↓ 控制器内部处理 PID
    ↗ ✅ 完全支持轨迹跟踪
    ↗ ✅ 与 MoveIt2 兼容
    ↗ ✅ 与 JointTrajectoryController 兼容
```

### 系统如何工作

```
你的鼠标  →  RViz 交互标记  →  MoveIt2 IK  →  轨迹规划  →  JointTrajectoryController
    ↓
MuJoCo 仿真执行  ←  FrankaMjHardwareSystem  ←  Position 命令
    ↓
机器人实时动起来！
```

---

## 验证安装成功

启动后应该看到：

```
✅ MuJoCo 仿真窗口打开
✅ RViz 显示两只黄色机器人臂
✅ 日志中出现：Command interfaces are [position]
✅ 日志中出现：Arm mj_left current mode: joint_position
✅ RViz 中可以拖动机器人末端（彩色箭头）
✅ Plan & Execute 完成后机器人在仿真中移动
```

---

## 进阶：理解控制层级

```
Level 1: 硬件接口 (FrankaMjHardwareSystem)
         导出：position, velocity, effort 命令接口

         ↓

Level 2: 控制器 (Controller)
         dual_panda_arm_controller
         类型：JointTrajectoryController
         命令：position 接口

         ↓

Level 3: 运动规划 (Motion Planning)
         MoveIt2 + OMPL
         输入：目标位置/方向
         输出：关节轨迹

         ↓

Level 4: 用户界面 (User Interface)
         RViz 交互标记
         MoveIt2 MotionPlanning 面板
```

---

## 相关资源链接

- **官方 MoveIt2 文档**: https://moveit.picknik.ai/
- **ROS2 Control 框架**: https://control.ros.org/
- **JointTrajectoryController 文档**: https://github.com/ros-controls/ros2_controllers
- **MuJoCo 官方**: https://mujoco.org/

---

## 常见问题 FAQ

**Q: 为什么改成 position 而不是 velocity？**
A: Position 更直观，与 MoveIt2 的轨迹规划天然配合。Velocity 需要额外的速度规划。

**Q: 能同时控制两只臂吗？**
A: 完全可以！dual_panda_arm_controller 包含了 14 个关节（两只臂各 7 个）。

**Q: 规划失败了怎么办？**
A: 首先尝试离当前位置更近的目标位置，然后增加 Planning Attempts 次数。

**Q: 如何记录和回放轨迹？**
A: 规划完成后可以在 RViz 中保存轨迹为 YAML 文件，之后可以直接发送给控制器执行。

---

## 下一步学习

1. 📖 阅读完整的 [RViz 交互操作指南](RVIZ_INTERACTIVE_GUIDE.md)
2. 🔧 了解 [控制器问题排查细节](CONTROLLER_TROUBLESHOOTING.md)
3. 🚀 尝试规划更复杂的轨迹
4. 📊 研究 MoveIt2 的高级功能（碰撞检测、轨迹约束等）

---

**祝你使用愉快！** 🎮
