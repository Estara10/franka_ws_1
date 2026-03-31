# 🤖 双臂协作搬运任务 - 使用指南

## 📋 快速开始

### 一键启动（推荐）

```bash
cd ~/franka_ws_1
colcon build
source install/setup.bash
./start_dual_arm_task.sh
```

**会自动启动:**
1. MoveIt2 运动规划器
2. MuJoCo 物理仿真（自动启动）
3. RViz 可视化
4. 双臂搬运任务节点

**预计时间:** 约25秒完全启动

---

## 📊 系统架构说明

### 核心代码 MVC 架构重构 ✨
目前的 `dual_arm_carry_task` 节点已完成了彻底的模块化（MVC）拆分，结构更加清晰、易于扩展：
- `carry_task_state_machine.cpp`：负责 ROS 节点生命周期与主状态机的流转。
- `dual_arm_controller.cpp`：直接对接 MoveIt 2 控制机械臂，处理 IK、轨迹规划与时间参数参数化（TOTG）。
- `gripper_controller.cpp`：负责双臂 Action 客户端的并发协同夹取与释放控制。
- `carry_task_context.hpp`：内部传递共享指针和系统状态参数的设计字典。

### 启动序列说明 (重要发现)
⚠️ **MoveIt2 会自动启动 MuJoCo！** 

之前的错误是因为重复启动了 MuJoCo，导致：
- 节点名称冲突
- 控制器重复加载
- 参数服务器混乱

### 正确的启动顺序
```
1. MoveIt2 启动 (sim_dual_moveit.launch.py)
   ├─ 自动启动 MuJoCo 仿真
   ├─ 加载 robot_description
   ├─ 加载 robot_description_semantic
   ├─ 启动 move_group
   ├─ 启动 RViz
   └─ 加载控制器

2. 等待40秒让系统完全就绪

3. 启动任务节点 (dual_arm_carry_task)
   └─ 连接到 MoveIt2 参数服务器
```

---

## 🎯 任务执行流程

### 完整状态机流转 (FSM)
```text
INIT (初始化)
  ↓ 双臂移动到 ready 待命位置
APPROACH (接近)
  ↓ 移动到物体两侧上方
GRASP (抓取)
  ↓ 下降到预定抓取高度，并同步夹紧夹爪
LIFT (抬起)
  ↓ 携带对象无碰撞、绝对稳定地垂直抬升
TRANSPORT / ROTATE (选配：位移与旋转)
  ↓ 平移并协同旋转一定角度
DESCEND (下降)
  ↓ 紧贴地面释放高度（防悬空掉落保护）
PLACE (放置)
  ↓ 柔顺松开双侧夹爪
RETREAT (撤退)
  ↓ 脱离接触并返回初始位
DONE / ERROR (任务完成/异常退出)
```

### 物体配置
- **类型:** 铝合金杆
- **位置:** (0.5, 0.0, 0.04) m
- **尺寸:** 40cm × 4cm × 4cm
- **质量:** 0.5 kg

---

## 🔧 实用工具

### 环境检测
```bash
cd ~/mujoco_franka/franka_ws
source install/setup.bash
./check_environment.sh
```

### 系统诊断
```bash
cd ~/mujoco_franka/franka_ws
source install/setup.bash
./diagnose.sh
```

会生成 `diagnostic_report_*.txt` 包含所有错误和警告。

---

## 📝 日志查看

### 日志位置
所有日志保存在 `~/mujoco_franka/franka_ws/launch_logs/`

### 查看日志
```bash
cd ~/mujoco_franka/franka_ws

# 查看最新日志
ls -lt launch_logs/

# 实时跟踪 MoveIt2+MuJoCo
tail -f launch_logs/moveit_mujoco_*.log

# 实时跟踪任务节点
tail -f launch_logs/task_*.log

# 搜索错误
grep -i error launch_logs/*.log

# 搜索警告
grep -i warn launch_logs/*.log
```

### 用编辑器打开日志
```bash
# 使用 VS Code
code ~/mujoco_franka/franka_ws/launch_logs/

# 或使用 gedit
gedit ~/mujoco_franka/franka_ws/launch_logs/task_*.log &
```

---

## 🎨 参数调整

### 修改任务参数
```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  approach_height:=0.20 \
  grasp_height:=0.08 \
  lift_height:=0.30
```

### 修改物体位置
编辑文件 (`src` 内文件，请勿编辑 `install` 内的文件): `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml`

```xml
<body name="obj_rod_01" pos="0.5 0.0 0.04">  <!-- 修改这里的坐标 -->
  <geom type="box" size="0.2 0.02 0.02" .../>
</body>
```

修改后需要重启系统。

### 修改运动速度
编辑文件: `src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_controller.cpp` 

```cpp
left_arm_->setMaxVelocityScalingFactor(0.5);    // 0.1-1.0
left_arm_->setMaxAccelerationScalingFactor(0.5);
```

重新编译:
```bash
cd ~/mujoco_franka/franka_ws
colcon build --packages-select dual_arm_carry_task
source install/setup.bash
```

---

## ⚠️ 常见问题

### Q1: 任务节点报错 "robot_description_semantic not found"

**原因:** 启动时机不对或系统重复启动

**解决:**
```bash
# 停止所有进程
pkill -9 ros2; pkill -9 rviz2; pkill -9 mujoco_node

# 使用正确的脚本重新启动
./start_dual_arm_task.sh
```

### Q2: 看到 "节点重复" 警告

**原因:** MuJoCo 被启动了多次

**解决:** 只使用 `start_dual_arm_task.sh`，不要手动启动 MuJoCo

### Q3: 机械臂不移动

**检查:**
```bash
# 1. 检查控制器
ros2 control list_controllers

# 2. 检查关节状态
ros2 topic echo /joint_states --once

# 3. 检查任务节点是否运行
ros2 node list | grep dual_arm_carry_task
```

### Q4: RViz 显示错误或不显示机器人

**解决:**
```bash
# 清理 RViz 缓存
rm -rf ~/.rviz2/

# 重启系统
./start_dual_arm_task.sh
```

---

## 🚫 停止系统

### 正常停止
在启动脚本终端按 `Ctrl+C`

### 强制停止
```bash
pkill -9 mujoco_node
pkill -9 ros2
pkill -9 rviz2
```

---

## 📚 其他文档

- **DEBUG_GUIDE.md** - 详细的调试指南
- **MANUAL_STARTUP.md** - 手动分步启动说明
- **README_START.md** - 原始快速开始指南

---

## 🎉 成功标志

任务成功执行时应该看到:

✅ **日志输出** (task_*.log):
```
[INFO] [dual_arm_carry_task]: === 双臂协作搬运任务节点初始化 ===
[INFO] [dual_arm_carry_task]: ✓ MoveIt 接口初始化完成
[INFO] [dual_arm_carry_task]: [状态: INIT] 移动到初始位置...
[INFO] [dual_arm_carry_task]: [状态: APPROACH] 接近物体...
[INFO] [dual_arm_carry_task]: [状态: GRASP] 夹紧物体...
[INFO] [dual_arm_carry_task]: [状态: LIFT] 抬起物体...
========================================
    ✓ 任务完成！
========================================
```

✅ **MuJoCo 窗口:**
- 灰色铝合金杆被双臂夹持
- 物体悬空约 25cm 高度

✅ **RViz 窗口:**
- 显示双臂 Panda 机器人
- 规划路径以绿色显示
- 机械臂位置与 MuJoCo 同步

---

## 📞 需要帮助？

遇到问题时:

1. 运行诊断工具: `./diagnose.sh`
2. 查看生成的报告: `cat diagnostic_report_*.txt`
3. 检查日志: `cat launch_logs/task_*.log`
4. 复制错误信息寻求帮助

---

**现在就开始！运行 `./start_dual_arm_task.sh` 🚀**

## 📁 仓库与文件结构说明 (GitHub Repository Structure)

本仓库包含了能够完整重现双臂协作搬运的全部核心代码和脚本，同时移除了所有体积庞大的环境依赖包及编译残留。


- `src/`: 核心的ROS 2源代码包（包含 `multipanda_ros2`、`mujoco_ros_pkgs` 及 `dual_arm_*` 独立控制、描述、任务包）。此前误散落在根目录的重复包已被彻底合并并清理至此，保持纯净标准的 ROS 2 工作空间结构。
- `scripts/patches/`: 我们为了修复不同依赖库与插件所用的 Python 脚本工具现已分类至此，不再污染主目录。
- 根目录脚本: 仅保留功能核心，诸如 `start_dual_arm_task.sh`（总控启动脚本）、`vision_core.py`（视觉中枢）、`sensor_dashboard.py`（数据看板）均存放在此，它们能够直调底层编译后的ROS资源。
- `.gitignore`: 已经为您做好了严谨的分类，任何涉及到本地环境构建的（例如 `build/`, `install/`, `log/`），以及巨型测试数据文件夹 `experiment_data/` 均已被安全跳过。每次将改动 `git push` 到 GitHub 时都将自动为您节省大量空间。

### 第一次拉取(Clone)仓库后该怎么做？
如果你或你的组员刚刚在另一台机器上 clone 了本仓库，**必须先进行编译**，否则无法运行脚本：
```bash
cd ~/mujoco_franka/franka_ws
# 编译所有必须模块
colcon build
# 读取环境变量
source install/setup.bash
# 开始测试
./start_dual_arm_task.sh
```
