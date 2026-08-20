# 手动启动指南

正常使用优先运行仓库根目录的 `./start_dual_arm_task.sh`。本指南用于分终端调试 MoveIt、
MuJoCo 和任务节点。

## 前提

工作区已经完成构建。每个新终端都需要加载环境：

```bash
cd /home/lidio/franka_ws_1
source install/setup.bash
```

`sim_dual_moveit.launch.py` 会自动包含 MuJoCo。不要先运行 `dual_franka_sim.launch.py` 再运行
MoveIt launch，否则会产生重复节点和控制器冲突。

## 终端 1：MoveIt、MuJoCo 和控制器

```bash
cd /home/lidio/franka_ws_1
source install/setup.bash
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=true
```

等待 `move_group`、MuJoCo 和 `controller_manager` 就绪。通常需要约 10–15 秒，具体取决于机器。

## 终端 2：检查系统状态

```bash
cd /home/lidio/franka_ws_1
source install/setup.bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

模式 A 当前依赖 `dual_panda_arm_controller`。它必须为 active；
`sim_multi_mode_controller` 不得同时 active。

## 终端 3：启动任务

直接使用任务 launch：

```bash
cd /home/lidio/franka_ws_1
source install/setup.bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant \
  enable_rotate:=true
```

该 launch 默认启用通信延迟和任务指标监控，默认关闭外部自适应柔顺监督器。常用高度参数：

```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant \
  approach_height:=0.28 \
  grasp_height:=0.13 \
  lift_height:=0.40 \
  enable_rotate:=true
```

说明：launch 的 `control_mode` 默认值为 `compliant`；一键脚本交互模式的默认选择目前仍是
`compliant_chomp`。后者只是历史模式名，当前核心任务没有真正调用 CHOMP。

## 观察任务

```bash
ros2 topic echo /task_stage
ros2 topic echo /task_status
ros2 topic echo /joint_states --once
```

正常状态顺序包含 `WAITING_PHYSICS`：

```text
INIT → APPROACH → GRASP → WAITING_PHYSICS → LIFT → TRANSPORT
     → ROTATE → DESCEND → PLACE → RETREAT → DONE
```

若任一运动失败，应看到 `ERROR`，不能把状态跳转到 `DONE` 当作成功。

## 停止

优先在任务终端和 MoveIt 终端分别按 `Ctrl+C`，等待进程正常退出。只在确认当前机器没有其他
ROS/MuJoCo 会话时，才使用针对具体进程的强制停止；不要把宽泛 `pkill -9` 当作常规步骤。

## 已知限制

- 外部阻抗监督器默认关闭，因为其控制器与位置轨迹控制器存在资源冲突。
- 模式 B 尚未完成相机标定和可靠位姿闭环。
- `check_environment.sh`、`diagnose.sh` 仍含旧路径，后续修复前请使用本页命令检查。
- 详细风险和当前能力见 [`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)。
