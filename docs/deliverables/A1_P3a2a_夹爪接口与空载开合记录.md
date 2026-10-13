# A1 P3a-2a 记录：夹爪接口可靠性与空载开合

对应阶段计划“当前执行器提示词：P3a-2a，夹爪接口可靠性与空载开合”。
**未下降、未抓杆、未启用 weld/attach、未提交 Git。** 未沿用旧风险表要求重复修改已被移除的栈回调。

## 一、当前接口语义（按当前源码核对，非沿用历史结论）

来源：`franka_hardware/src/sim/gripper_sim_action_server.cpp`

| 项 | 实际语义 |
|---|---|
| `command.position` | **单指位移**。服务器计算目标开口为 `kTargetWidth = 2 * command.position` |
| `result.position` | **总开口**（米）。`control_msgs/GripperCommand` 定义为 "the current gripper gap size" |
| `result.reached_goal` | 由 `command_handler()` 的返回值设置；本服务器**开环**，不校验实际开口 |
| `result.stalled` | **该服务器从不设置**（恒 false）⇒ 此字段在本环境无信息量 |
| `result.effort` | 恒 0.；`simGripperGrasp` 的 force/epsilon 形参**未使用** |
| ABORTED 触发 | 目标开口越界（> max_width 或 < 0），或 command_handler 返回 false |
| `mj_*_finger_joint1` | **prismatic，范围 [0.0, 0.04]**，轴 ±Y；`finger_joint2` 为 mimic。**关节位置即单指位移**，可直接与 `command.position` 比较 |

⇒ 两个 `position` **不可互换**，且**都不能默认等同杆宽**（杆宽 0.04 m；A0 的 `gripper_close_position=0.010`
对应目标开口 0.020 m）。

## 二、修改

| 动作 | 文件 |
|---|---|
| 修改 | `gripper_controller.{hpp,cpp}`：**持有并复用**左右 client；分阶段有界等待；总预算；失败即取消；去掉 `closing && ABORTED` 无条件成功 |
| 新增 | `gripper_protocol.{hpp,cpp}`（**纯逻辑，无 ROS**）：结果码映射、结果分类、迟到接受判定、预算夹取 |
| 新增 | `test/test_gripper_protocol.cpp`（12 例） |
| 修改 | `carry_task_context.hpp`、`carry_task_state_machine.cpp`、launch、`scripts/a1_run_scene.sh`（`--gripper-selftest`）、CMake |

**接口可靠性**：
- 左右 client 只在首次使用时创建，之后**复用**（日志 `创建并复用 … action 客户端`）。
- 重叠调用用 `try_lock` + `in_flight` **拒绝**，不排队、不加通用框架。
- 阶段预算（server/goal/result）各自有界，且都被**夹取到单一总预算**内
  （`a1g::remainingBudget`），因此不会把两个串行满超时说成一个总超时；实测三段各 1.6/2.4/1.9 s，
  总预算 20 s。
- **失败即取消**：被拒/失败/超时都会对本轮已接受的目标发 `async_cancel_goal`，
  记录取消是否确认，并明确**不声称物理已停止**。
- **迟到接受**：`decideLateAcceptance` 把“仅在宽限期内到达”的接受判为 **`CANCEL_LATE`（取消并失败）**，
  绝不继续；完全无响应判为 `UNKNOWN_STOP`（状态未知，停止本轮，不重试、不开始下一次）。
- **去掉 `closing && ABORTED` 无条件成功**：ABORTED/CANCELED/未知码一律失败；
  即使测量开口正好落在目标上，也不把 action 失败改写成成功（`E_GRIPPER_ABORTED_CLOSING`）。
  闭合中止另记 `contact_unproven`（接触未证实），不据此声称抓取。

## 三、本轮我自己发现并修掉的两个 bug

1. **结果码映射错误（真实运行暴露）**：我最初把 `rclcpp_action::ResultCode` 当作 0/1/2/3 映射。
   实际本发行版中它**别名 action_msgs GoalStatus**：`SUCCEEDED=4`、`CANCELED=5`、`ABORTED=6`。
   于是**一次成功的打开被当成“未知码”而判失败**（首轮 `code=4 → E_GRIPPER_UNKNOWN_CODE`）。
   已按真实值修正并加断言：`code=4` 不得再被读成 UNKNOWN。
2. **重复声明已存在参数**：我新增了 `gripper_close_position` 参数与 launch 实参，
   而节点**早已声明**该参数（A0 默认 0.010），导致 `ParameterAlreadyDeclaredException`
   使节点中止；且 launch 的 0.02 默认值会**静默改变 A0 的闭合位置**。已删除新增项，
   空载闭合**复用**节点既有 `gripper_close_pos_`（日志显示 `close_cmd=0.0100`，A0 行为不变）。

另：曾尝试在脚本侧等待 action server，但 `ros2 action list --no-daemon` 在该场景下不可靠；
已改回由节点自身做有界 server 等待（预算 3 s → **10 s**，仍在总预算内），不再在脚本里做 CLI 发现。

## 四、真实空载开合（`a1_p3a2a_gripper_selftest_g`）

```bash
./scripts/a1_run_scene.sh --gripper-selftest --run-id a1_p3a2a_gripper_selftest_g
```

```
[GRIPPER_RESULT] state=GRIPPER_TEST_DONE failed_step=- open_cmd=0.0400 close_cmd=0.0100 arm_motion=none weld=0 attach=0
```

| 步骤 | 命令（单指） | 目标开口 | 协议结果 | 服务器报告 gap | **实测指关节** |
|---|---|---|---|---|---|
| open | 0.0400 | 0.0800 | code=4 SUCCEEDED, reached_goal=1 | 0.0876 | **0.0000** |
| empty_close | 0.0100 | 0.0200 | code=4 SUCCEEDED, reached_goal=1 | **−0.0142** | **0.0000** |
| reopen | 0.0400 | 0.0800 | code=4 SUCCEEDED, reached_goal=1 | −0.0139 | **0.0000** |

**无运动边界**：APPROACH/GRASP/双臂规划/weld/attach 计数**均为 0**；夹爪是唯一被命令的部件。

## 五、如实结论：接口通了，但仿真里夹爪并没有实际动

三点必须分开说，不能混为一谈：

1. **action 接口层面成功**：两台 server 可达、goal 被接受、结果收到、SUCCEEDED。
   `gripper_selftest_status.json` 的 `reached=true` 只表示这一层。
2. **服务器报告的 `gap` 不可信**：open 报 0.0876（已超过 max_width 0.08），
   close 报 **−0.0142（负值）**，reopen 后又**停在 −0.0139（几乎没有变化）**。
   该值是服务器**内部宽度模型**，既不对应命令，也不对应关节。
3. **实测指关节恒为 0.0000**：`/joint_states` 共发布 18 个关节（含 4 个 finger 关节），
   其中 `mj_*_finger_joint1/2` 全程为 **0.0**，即**仿真中夹爪指关节从未移动**（原始证据见
   `joint_states_after_cycle.txt`）。`reached_goal=true` 因此**不能**作为“开口已到达”的证据。

⇒ 本轮**不声称空载开合在物理上成功**；能交付的是**接口可靠性**（复用的 client、有界等待、
取消、迟到处理、去掉 ABORTED 误判）与一次真实的接口往返记录。
空载开合成功本来也不等于抓取成功；而此处连“开口变化”都未在关节上发生。

**这对下一步有直接影响**：P3a-2b（下降与首次物体抓取）依赖夹爪真正闭合。
在夹具/仿真驱动侧查明指关节为何不动之前，抓取无法成立。本轮按提示词**不扩大范围**去改
MuJoCo/硬件/action server，只如实上报。

## 六、测试

```bash
colcon build --packages-select dual_arm_carry_task --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test  --packages-select dual_arm_carry_task --ctest-args -R "test_a1_|test_gripper_"
```

**6 个套件 159 例全部通过，0 失败**，构建 0 warning（新增 `test_gripper_protocol` 12 例）。

覆盖：SUCCEEDED 成功；**闭合 ABORTED 绝不成功**（含“开口看似达标也不改判”）；CANCELED/未知码失败；
闭合成功但未达目标标 `contact_unproven`；**真实结果码 4/5/6 的映射**（含“4 不得读成 UNKNOWN”回归）；
迟到接受 → `CANCEL_LATE`；无响应 → `UNKNOWN_STOP`；预算夹取与总预算耗尽。

**口径声明**：这些是**纯逻辑故障注入测试**，不是 ROS client 的实测；
真实 client 行为由上节的实机往返记录覆盖，未穷举并发/DDS 极端情况。

## 七、遗留问题（分级）

| 级别 | 项 |
|---|---|
| **需规划器决策 / 可能阻断 P3a-2b** | 仿真中 `mj_*_finger_joint1` 恒为 0，夹爪未实际开合；action server 报告的 gap 与之矛盾。抓取前需先查明驱动侧原因。 |
| 允许后续改进 | 服务器 `reached_goal` 开环、`stalled` 恒 false、`effort` 恒 0；接触语义（抓取时 ABORTED 是否代表接触）仍需结合物体验证。 |
| 允许后续改进 | 取消只记录“请求已发出/是否确认”，不声称物理已停止。 |
| 允许后续改进 | 迟到接受/无响应分支为纯逻辑测试覆盖，未在真实 server 上注入。 |
| 允许后续改进 | 既有启动崩溃仍偶发；本轮 7 次运行**全部保留**，含 1 次启动崩溃、1 次 server 未就绪（已修）。 |
| 当前不需要解决 | 下降、抓杆、weld、attach、视觉均未开放。 |

数据路径：`experiment_results/a1_runs/a1_p3a2a_gripper_selftest_g/`
（`gripper_selftest.log`、`joint_states_after_cycle.txt`、`gripper_selftest_status.json`、
`sim_dual_moveit.log`、`controllers.txt`）。

---

# 追加：P3a-2a 夹爪单位与真实反馈修复（2026-10-13）

按规划器线索做小范围修复：仿真夹爪 server、硬件交接、`joint_states` 来源、client 结果判定。
**未下降、未抓杆、未启用 weld/attach、未提交 Git。** 机械臂控制未改。

## A. 更正上一轮两处过强结论

1. **“指关节从未移动”不成立。** 该结论来自全局 `/joint_states` 的零值，而当时
   `jsp_source_list` 指向**不存在的话题**（见下），全局汇总本来就拿不到指关节真值。
   **修复后实测**：全局 `/joint_states` 的四个指关节为 **0.0400 m**（完全张开），
   即指关节**确实在动**。
2. **“gap 是纯内部虚拟宽度”不成立。** `getGripperState()` 的
   `width = gripper_states_ptr_->at(1) * 2`，而 `at(1)` 由
   `FrankaMjHardwareSystem::read()` 写入 `d_->qpos[gripper_joint_qpos_indices_[0]]`，
   即**真实指 qpos**。所以 gap 是真实反馈，不是虚拟模型。上一轮的说法已按实纠正。

## B. 修复内容

| # | 问题 | 修复 |
|---|---|---|
| 1 | `jsp_source_list` 用了 `mj_*_gripper_sim_node/joint_states/joint_states`（**重复段**），服务器实际发布 `~/joint_states` ⇒ 汇总拿不到指关节 | `sim_dual_moveit.launch.py` 改为 `mj_*_gripper_sim_node/joint_states` |
| 2 | **单位混用**：服务器把米制 width 除以 0.0003137 写成 0–255，硬件却把该值直接写入 actuator；该 actuator 是位置型、驱动定长 tendon `0.5*q1+0.5*q2`、**ctrlrange=[0,0.08]**、关节范围 [0,0.04] ⇒ 任何命令都被钳到 0.08 | 新增 `writeGapCommand(gap_m)`：写入 **`gap/2`（单指位移，米）** 并钳到 [0,0.04]；三处写入点（move/grasp/homing/stop）统一，**0–255 转换全部删除** |
| 3 | 开环固定步数 + `+50` 步可能越过目标，成功由开环返回值决定 | 新增 `driveToGap()`：步数按距离推导（**去掉 +50**）、另有 5 s 墙钟上限；**成功由真实指反馈 `measuredGap()` 达到目标容差决定**；取消/超时返回 false |
| 4 | client 对所有 SUCCEEDED 一律判成功，NaN/负值/越界 gap 也通过 | `classifyResult` 增加反馈有效性检查（**对所有结果码**）：非有限、负值、超过模型最大开口 ⇒ 拒绝，且**不截断掩盖** |

`simGripperHoming` 也改为两段反馈驱动；`simGripperStop` 改为按同一单位保持当前开口。
force/epsilon 仍未实现，未新增力控或 effort 估计。

## C. 真实运行结果（`a1_p3a2a_fix_units`）

```
[GRIPPER_RESULT] state=FAILED failed_step=open open_cmd=0.0400 close_cmd=0.0100 arm_motion=none weld=0 attach=0
左: code=4 reached_goal=1 gap=0.0843 m → E_GRIPPER_GAP_OUT_OF_RANGE（> 模型最大 0.08）
右: code=4 reached_goal=1 gap=0.0319 m → E_OK
```

- **新 client 判定生效**：越界反馈被拒绝，不再被当作成功。
- **单位修复生效**：全局 `/joint_states` 的指关节现在为真实值 `0.0400`（修复前恒 0）。
- **左右不一致**（0.0843 vs 0.0319）说明两指此刻处于不同状态。

**残留问题（未解决，P3a-2a 仍不通过）**：左指出现 **0.0843 m > 模型最大 0.08 m**
（等价于单指 qpos ≈ 0.0422 > 关节上限 0.04），即**瞬态越过关节软限**——
位置 actuator 以 kp=1200 推向 tendon 目标时冲过上限。当前没有把它截断成 0.08 掩盖，
而是如实拒绝。需要下一步决定：在 server 侧按关节范围限制 tendon 目标/加阻尼，
或按实际模型确认 ctrlrange 与关节上限的对应关系。

⇒ **本轮仍未获得一次干净通过的空载开闭开**，故不输出 `GRIPPER_TEST_DONE`。
按要求保留失败并停止，不进入 P3a-2b。

## D. 测试

**160 例 0 失败**（新增异常反馈拒绝用例：负值 / NaN / Inf / 超过模型最大 ⇒ 拒绝；
0.0 与 0.08 边界仍通过）。构建 0 warning（`franka_hardware` 一条既有符号比较告警非本次引入）。

## E. 遗留

| 级别 | 项 |
|---|---|
| **阻断 P3a-2a 收尾** | 指关节瞬态越过 0.04 上限，导致 gap 越界被拒；需决定限幅/阻尼方式（本轮未自行改模型或 ctrlrange）。 |
| 允许后续改进 | 未实现真实接触力/effort；`force`/`epsilon` 仍无效；`reached_goal` 现由 `driveToGap` 的真实反馈决定。 |
| 允许后续改进 | 未建立最小 ROS 假 action server 来对真实 client 注入单侧失败/超时/迟到；本轮仍以纯逻辑测试覆盖这些判定。 |
| 允许后续改进 | 取消仍只报告“请求已发出/是否确认”，不声称物理已停止。 |
