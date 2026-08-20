# RViz 中的 MoveIt2 交互操作完全指南

## 系统现状 ✅

当前系统已经成功启动：
- MuJoCo 仿真服务器运行中
- FrankaMjHardwareSystem 硬件接口已激活
- dual_panda_arm_controller 已加载（position 接口）
- RViz + MoveIt2 正在运行

---

## 📍 第一步：定位交互标记

### 在 RViz 3D 窗口中：

1. **查找机器人末端的彩色箭头**
   - 左臂末端（mj_left_hand_tcp）：会看到彩色箭头
   - 右臂末端（mj_right_hand_tcp）：会看到彩色箭头
   
2. **箭头颜色含义**
   - 🔴 **红色轴** = X 方向（向前/向后）
   - 🟢 **绿色轴** = Y 方向（向左/向右）
   - 🔵 **蓝色轴** = Z 方向（向上/向下）

3. **如果看不到箭头**
   - 在左侧 Displays 面板中找到 "Interactive Markers"
   - 确保复选框被勾选 ✓
   - 按 R 键重置视角

---

## ✋ 第二步：拖动末端执行器

### 基本操作：

1. **点击并拖动红色箭头**
   ```
   目标：将左臂末端向前移动 10cm
   步骤：
   ① 在 3D 视图中找到左臂末端的红色箭头
   ② 点击并按住鼠标左键
   ③ 拖动鼠标向前（远离机器人）
   ④ 释放鼠标
   ⑤ 观察末端目标位置（应该有半透明的预览）
   ```

2. **拖动绿色和蓝色箭头**
   ```
   绿色箭头：左右移动
   蓝色箭头：上下移动
   ```

3. **同时旋转末端执行器（高级）**
   ```
   目标：旋转末端方向
   步骤：
   ① 在交互标记周围寻找圆形环（旋转控制器）
   ② 点击并拖动圆环旋转末端
   ```

---

## 🎯 第三步：规划和执行轨迹

### 在 RViz 右侧 MotionPlanning 面板中：

#### 方法 A：一键规划和执行（推荐）
```
1. 拖动末端到新位置
2. 找到 "Plan & Execute" 按钮
3. 点击 "Plan & Execute"
4. 等待规划完成（1-3 秒）
5. 看着机器人在 MuJoCo 中移动！
```

#### 方法 B：分步操作（调试用）
```
1. 拖动末端到新位置
2. 点击 "Plan" 按钮
3. RViz 会显示规划的轨迹（彩虹线）
4. 如果轨迹看起来合理，点击 "Execute" 执行
5. 如果轨迹有问题，可以调整目标位置后重新规划
```

#### 方法 C：重置到初始位置
```
1. 在 MotionPlanning 面板找到 "Reset" 按钮
2. 点击 "Reset"
3. 机器人会回到初始位置（两臂垂直向上）
```

---

## 🖥️ 视图控制

### 在 RViz 3D 窗口中：

| 操作 | 快捷键/鼠标 | 说明 |
|------|-----------|------|
| **旋转视图** | 中键拖动 | 用鼠标中键按住并拖动 |
| **平移视图** | 右键拖动 | 用鼠标右键按住并拖动 |
| **缩放视图** | 滚轮 | 向上/向下滚动鼠标滚轮 |
| **重置视角** | R 键 | 一键回到默认视角 |
| **焦点聚焦** | 点击物体 | 聚焦在某个物体上 |

---

## ⚙️ 调整规划参数

### 在 MotionPlanning 面板中的 "Planner" 标签页：

#### Planning Attempts（规划尝试次数）
```
默认：5
如果规划经常失败，增加到 10 或 15
更多次数 = 更容易找到解决方案，但会更慢
```

#### Planner Selection（规划器选择）
```
默认：RRTstar（较快，结果不是最优）
可选：PRM（更优的路径，但较慢）
可选：RRT（最快，但质量最差）
```

#### Planning Time（每次尝试的规划时间）
```
默认：5 秒
如果复杂场景规划失败，增加到 10 秒
```

---

## 🔧 常见问题和解决方案

### ❌ 看不到交互式标记

**症状：** RViz 中看不到末端的彩色箭头

**解决方案：**
```bash
1. 检查 Displays 面板：
   - 左侧 Displays → MotionPlanning → Interactive Markers ✓
   
2. 重置视角：
   - 按 R 键
   
3. 如果还是看不到，重启系统：
   - pkill -f "ros2 launch"
   - 等待 5 秒
   - ./start_interactive_sim.sh
```

### ❌ 规划失败（"Unable to plan"）

**症状：** 点击 "Plan" 或 "Plan & Execute" 后显示规划失败

**常见原因和解决方案：**

1. **目标位置超出工作空间**
   ```
   症状：目标位置离当前位置太远
   解决：选择更近的目标位置
   ```

2. **目标位置不可达**
   ```
   症状：目标位置在障碍物内或无法通过逆运动学求解
   解决：尝试稍微改变位置，增加规划尝试次数
   ```

3. **规划器超时**
   ```
   症状：规划花费很长时间
   解决：
   ① 增加 "Planning Time"
   ② 或减少 "Planning Attempts"
   ```

### ❌ 规划成功但机器人不动

**症状：** "Plan & Execute" 完成，但 MuJoCo 中的机器人没有移动

**诊断步骤：**

```bash
1. 检查控制器状态：
   ros2 control list_controllers
   
   输出应该包含：
   dual_panda_arm_controller [active] [JointTrajectoryController]
   
2. 检查 joint_states 是否更新：
   ros2 topic echo /joint_states --rate=10
   
   应该看到实时变化的关节位置
   
3. 检查 MuJoCo 仿真是否还在运行：
   ps aux | grep mujoco_node
   
4. 查看轨迹执行反馈：
   ros2 topic echo /dual_panda_arm_controller/follow_joint_trajectory/result
```

**常见解决方案：**

```bash
# 方案 1：重启控制器
ros2 control switch_controllers --stop dual_panda_arm_controller
sleep 2
ros2 control switch_controllers --start dual_panda_arm_controller

# 方案 2：重启整个系统
pkill -f "ros2 launch"
./start_interactive_sim.sh
```

---

## 📊 完整工作流程

```
用户操作
   ↓
[1] 在 RViz 中拖动末端执行器的交互标记
   ↓
[2] 发送目标姿态给 MoveIt2
   ↓
[3] 运动学求解（IK）计算关节角度
   ↓
[4] 轨迹规划（OMPL）生成无碰撞路径
   ↓
[5] 规划结果返回给 RViz（彩虹线显示）
   ↓
[6] 用户点击 "Execute" 或 "Plan & Execute"
   ↓
[7] 轨迹发送给 JointTrajectoryController
   ↓
[8] 控制器执行关节位置命令
   ↓
[9] FrankaMjHardwareSystem 转换为 MuJoCo 命令
   ↓
[10] MuJoCo 仿真执行物理计算
   ↓
[11] 关节状态反馈给 RViz
   ↓
[12] RViz 实时显示机器人运动
   ↓
结束（机器人到达目标位置）
```

---

## 🚀 高级技巧

### 1. 同时控制两只臂

```
左臂规划执行完成后，点击右臂末端的交互标记
重复规划和执行步骤
两只臂可以独立或协调完成任务
```

### 2. 查看轨迹细节

```
规划完成后，在 MotionPlanning 面板点击 "Trajectory Preview"
可以逐步播放规划的轨迹，检查关节角度变化
```

### 3. 记录和保存轨迹

```
在 RViz 中记录轨迹的步骤：
① 规划完成后，右键点击轨迹线
② 选择 "Save Trajectory"
③ 输入文件名和路径
④ 保存为 YAML 或 Bag 文件，以后可以回放
```

### 4. 调整速度和加速度

```
在 MotionPlanning 面板 → Execution
- Velocity Scaling：0.0 ~ 1.0（1.0 = 100% 速度）
- Acceleration Scaling：0.0 ~ 1.0
减小这些值可以让机器人运动更平缓
```

---

## 📋 检查清单

启动前确认：

- [ ] 执行了 `./start_interactive_sim.sh`
- [ ] MuJoCo 仿真窗口已打开
- [ ] RViz 显示两只黄色机器人臂
- [ ] Displays 中的 "Interactive Markers" 已勾选
- [ ] 可以在 RViz 中看到末端执行器的彩色箭头

操作前确认：

- [ ] 目标位置在机器人工作空间内
- [ ] 没有障碍物挡住规划的路径
- [ ] 控制器状态为 [active]

---

## 📞 快速命令参考

```bash
# 启动系统
cd ~/franka_ws && ./start_interactive_sim.sh

# 检查控制器
ros2 control list_controllers

# 监听 joint_states
ros2 topic echo /joint_states

# 监听轨迹执行结果
ros2 topic echo /dual_panda_arm_controller/follow_joint_trajectory/result

# 查看所有 ROS2 节点
ros2 node list

# 查看所有 ROS2 话题
ros2 topic list

# 停止系统
pkill -f "ros2 launch"
```

---

## 🎓 学习资源

**MoveIt2 官方文档：**
https://moveit.picknik.ai/

**RViz 交互标记：**
https://wiki.ros.org/rviz/Tutorials

**ROS2 控制框架：**
https://control.ros.org/

---

## ✅ 总结

现在你已经掌握了：
1. ✅ 如何定位和拖动交互标记
2. ✅ 如何规划和执行轨迹
3. ✅ 如何调整视图和参数
4. ✅ 如何排查常见问题
5. ✅ 如何进行高级操作

**开始玩耍吧！** 🎮

在 RViz 中拖动机器人臂，看着 MuJoCo 中的仿真实时响应。这是机器人开发中最有趣的部分！
