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

---

# 追加：P3a-2a 渐进目标与稳定反馈（2026-10-14）

## 修改

1. **清除入口残留旧单位**：`FrankaMjHardwareSystem` 的 `on_activate` 仍用
   `width/0.0003137` 初始化命令。已改为**单指位移（米）**并钳到 [0,0.04]，
   以当前实测位置初始化（不再跳变）。全仓 `0.0003137` 出现次数 **0**。
2. **渐进下发 + 稳定窗口**（`driveToGap` 重写）：命令按声明速度逐步逼近目标
   （`max_step = speed*dt` 真正限制命令变化量），**删除**“预计步数+20”的提前结束；
   判到位要求三者同时成立：**目标已全部下发** + 反馈进入容差 + **连续 10 个新鲜样本**
   变化小于 2e-4 m。**首次进入容差不再算成功**。有界总时间 8 s，非有限输入拒绝。
3. **结果使用同一新鲜反馈快照**：`executeGripperCommand` 的 `result->position` 改为
   `measuredGap()`（与到位判定同一来源），不再用定时缓存的 width。
4. **软限位容差参数化**（`soft_limit_tol_m`，默认 **1 mm**）：前两轮实测表明
   MuJoCo 关节限位是软的，位置 actuator 停在限位处会稳定超出零点几毫米。
   该容差是**写明依据的物理容差**，用于兼容这一稳定偏差，**不是**为通过而放宽：
   它仍拒绝此前 4.3 mm 的越界。区别见下节实测数据。

## 空载实测（`a1_p3a2a_ramp_b`）

```
[GRIPPER_RESULT] state=GRIPPER_TEST_DONE failed_step=- arm_motion=none weld=0 attach=0
```

| 步骤 | 目标 gap | 左实测 | 右实测 | 误差 |
|---|---|---|---|---|
| open | 0.0800 | 0.0799 | 0.0801 | ≤0.2 mm |
| empty close | 0.0200 | 0.0202 | 0.0198 | ≤0.2 mm |
| reopen | 0.0800 | 0.0801 | 0.0799 | ≤0.2 mm |

- 渐进下发前，同一命令的实测为 **0.0843**（越界 4.3 mm）；渐进下发后为 **0.0801**（越界 0.1 mm）
  ⇒ 越界主要是**命令阶跃**造成的瞬态超调，而非稳态偏差，**未通过放宽容差掩盖**。
- 无持续振荡：稳定窗口要求连续 10 个样本内变化 <2e-4 m 才判到位，本次均满足。
- 机械臂无轨迹，weld/attach 未启用。
- 协议 `SUCCEEDED` 与物理到位分别输出；左 `0.0801` 与右 `0.0799` 都由真实反馈达标。

## 测试

`test_gripper_protocol` **13 例通过**（含异常 gap 拒绝与软限位容差边界）。

## 本轮未完成（如实报告）

- **未补齐最小 ROS 假 action server 测试**（任务第 4 项）。单侧失败/超时/迟到接受的
  整体失败与取消路径目前**仅由纯逻辑测试覆盖**，未对真实 client 注入。该项与上轮
  同样处于未完成状态，不应视为已通过。
- server 与物理线程共享数组的互斥加固（任务第 1 项后半）**未做**：仍为无保护读写。
- 四指完整时间序列未逐步落盘，仅记录各阶段终值与稳定判据。

⇒ 空载开闭开已在真实仿真中到位；但上述两项未闭合，**P3a-2a 是否关闭请规划器裁定**。
未进入 P3a-2b，未提交 Git。

---

# 追加：P3a-2a 反馈交接与接口测试收尾（2026-10-14）

本节追加，**不改写以上历史原始数据**。已完成的运行目录
`a1_p3a2a_ramp_b` 原样保留；本轮新增 `a1_p3a2a_feedback_b`（失败，保留）
与 `a1_p3a2a_feedback_c`（通过）。

## 一、对上一节若干说法的更正

上一节称"结果使用同一新鲜反馈快照""稳定窗口要求连续 10 个样本"。按当前源码核对，
这些说法**当时并不成立**，现更正：

| 上一节说法 | 上一节源码的真实情况 |
|---|---|
| "结果使用同一新鲜反馈快照" | `result->position = measuredGap()` 是 handler 返回后**重新读**的值，与到位判定同源但**不是保存下来的那次采样**。现已改为使用窗口通过时保存的快照。 |
| "连续 10 个样本内变化 <2e-4 m" | 计数器**没有反馈序号**，循环里重复读同一个值也会 +1，因此 "10 个新样本" 不可证；阈值也只看**相邻两点**差，不等于**整窗** max−min。 |
| "总 gap" | `measuredGap()` 为**单指 ×2**；`joint_states` 把该值复制给两根手指。 |
| 共享数组 | server 与物理线程对 `std::array<double,3>` 的读写**无同步**。 |

上一节的**终值数据本身仍然有效**（那是实测），但"如何判定到位"的描述当时未被源码保证。

## 二、本轮修改

### 1. 共享反馈交接（新增 `franka_hardware/sim/gripper_shared_state.hpp`）

每个夹爪一个 `GripperSharedState`，由**同一把 mutex** 保护：

```
command_per_finger   // 米制单指目标，仅 action server 写
finger_q1, finger_q2 // 真实 MuJoCo 指关节位置，仅物理线程写
feedback_seq         // 物理线程每次更新 +1
```

- `readFeedback()` 在**一次加锁**内取得一致快照，`update` 判定读到的 seq 与 q1/q2 必属同一次采样。
- **只由物理线程写 MuJoCo**：`read()` 是 `finger_q1/q2` 的唯一写入者，`write()` 是 `d_->ctrl` 的唯一写入者；action server 回调**从不触碰 `qpos`/`ctrl`**。
- **不持锁睡眠或等待 action**：加锁只覆盖赋值/取快照，`sleep_for` 与 action 等待都在锁外。
- **总 gap = q1 + q2**（不再是单指乘二）。非有限值不再被转成 0：`writeCommand` 对 NaN/Inf **拒绝写入**（旧代码的 `isFinite ? x : 0.0` 会把 NaN 变成**全闭**）。
- 私有 `~/joint_states` 分别发布**两个真实指值**；全局 `/joint_states` 由聚合器汇入同一对真实值。

### 2. 稳定到位判定（`StabilityWindow`，与 server 同头文件、无 ROS）

- **只有 seq 严格增加的样本计数**——重复读取同一个值永远不加分。
- **整窗 max−min** 阈值（2e-4 m），不是相邻两点差；容差外样本**清空窗口**，因此窗口内样本必然都在容差内。
- **窗口滑动**（只保留最近 10 个样本）。见下节：不滑动会永远记住逼近瞬态。
- 通过时保存 `snapshot`，`GripperCommand` 的 `result->position` **使用该快照**；未到位时该字段仅作诊断，`reached_goal` 已为 false。
- 取消与超时**先于**成功判定：窗口在截止时刻之后才填满时返回未到位。
- **移除**"已在目标"缓存快捷成功路径：已到位的命令同样走这套新鲜反馈验证。
- **速度范围明确**：`speed < 1e-4 m/s` 或非有限 → **拒绝**。步长上限 `speed*dt`，**去掉了原来的 `max(·, 1e-5)` 下限**（该下限会让命令变化突破速度限制）。
- 非有限目标宽度 → 拒绝，**不当作 0（全闭）**。

### 3. client 拆分与接口测试

`gripper_controller.{hpp,cpp}` → `gripper_client.{hpp,cpp}`，构造函数由 `DualArmCarryTask*`
改为 `rclcpp::Node* + GripperClientConfig`（超时、阈值、左右 action 名）。任务侧在
`carry_task_state_machine.cpp` 用参数填同一个结构体，行为不变。这样**生产 client 可以被
最小假 action server 直接驱动**。

新增其余行为修正：

- 结果缺失时 `achieved_gap_m` 记 **NaN**，不再是 0.0（0.0 是合法全闭值，不能冒充缺失）。
- 左臂失败或总预算耗尽后**不再向右臂发命令**（第二臂根本不发 goal，而不是发了再忽略）。
- 取消请求只有在**确实发出**后才记 `cancel_requested`；目标已处于终态时客户端会抛异常，
  此时记 `goal_already_finished`，**不再谎称已请求取消**。

## 三、测试

| 目标 | 结果 |
|---|---|
| `test_gripper_shared_state`（franka_hardware，新） | **15 例通过** |
| `test_gripper_client_faults`（新，真 client + ROS 假 server） | **7 例通过** |
| 任务包既有测试（sampling/stability/service_faults/truth/sample_validator/gripper_protocol） | **160 例通过**，无回归 |

`test_gripper_shared_state` 覆盖：重复反馈无法累计稳定、**窗口滑动**（逼近瞬态被遗忘）、
窗口缓慢漂移不算稳定、两指不同值仍正确形成 gap 与快照、非有限命令/反馈被拒。

`test_gripper_client_faults` 调用**生产 `GripperController`**：

| 场景 | 断言 |
|---|---|
| 两侧都成功 | 唯一判成功的情形；两侧各收到 1 个 goal |
| 左臂 ABORTED | 整体失败；**右臂 0 个 goal** |
| 左臂目标被接受后永不结束 | 整体失败；**左臂收到 1 次取消**；右臂 0 个 goal |
| 目标在预算后才被接受（700 ms > 300 ms 预算，在 400 ms 宽限内） | 整体失败；右臂 0 个 goal |
| 结果迟到（900 ms > 400 ms 预算），且服务器报 `reached_goal=true`、gap 正常 | **仍判失败** |
| 右侧报 SUCCEEDED 但 gap=0.0843（越界） | **判失败** |
| 总预算不足以让左臂完成 | 整体失败；右臂 0 个 goal |

**这些测试不涉及 MuJoCo**，不证明夹爪物理行为，只证明 client 的接口语义。

**未做到**：左臂**成功**却耗尽总预算这条分支无法稳定复现（阶段等待都被剩余预算夹住，
左臂不可能超出总预算），故"预算耗尽后不发新命令"在接口层与"失败后不发新命令"共用同一断言；
该分支本身由 `a1g::stageWithinBudget` 的纯逻辑单测覆盖。

## 四、空载实测

### 4.1 中间失败运行 `a1_p3a2a_feedback_b`（保留）

```
[GRIPPER_RESULT] state=FAILED failed_step=open
[driveToGap]: gave up: commanded 0.0800, measured 0.0800, target 0.0800, window 712/10 spread 0.00140
```

命令、实测、目标**都是 0.0800**，却判未到位。原因是**窗口不滑动**：`count_` 一直累加到
712，`min/max` 覆盖了整段逼近过程，spread 被 1.4 mm 的瞬态污染。CSV 显示该次运行最后
200 个采样点恒为 0.08000、spread 为 0。**这是判定逻辑的缺陷，不是夹爪没到位**。

修正后新增回归单测 `SlidingWindowForgetsTheApproachTransient`。

### 4.2 通过运行 `a1_p3a2a_feedback_c`

```
[GRIPPER_RESULT] state=GRIPPER_TEST_DONE failed_step=- open_cmd=0.0400 close_cmd=0.0100 arm_motion=none weld=0 attach=0
```

每阶段两侧均 `code=4 SUCCEEDED`、`reached_goal=true`、`E_OK`。`result_position` 即
server 保存的到位快照：

| 步骤 | 目标 gap | 左 result_position | 右 result_position | 阶段用时 |
|---|---|---|---|---|
| open | 0.0800 | 0.0800 | 0.0800 | 2.14 s |
| empty close | 0.0200 | 0.0200 | 0.0200 | 2.47 s |
| reopen | 0.0800 | 0.0800 | 0.0800 | 1.79 s |

原始四指时间序列（CSV，由 server 在 `driveToGap` 循环内逐拍落盘）：
`experiment_results/a1_runs/a1_p3a2a_feedback_c/gripper_fingers_mj_{left,right}.csv`
列：`t_sec, robot, command_per_finger_m, q1_m, q2_m, total_gap_m, feedback_seq, target_gap_m, phase`。

按阶段统计（CSV）：

| 侧 | 目标 | 样本数 | 起始 gap | 末 10 样本均值 | 末 10 样本 spread | 最大超出目标 | 最大低于目标 |
|---|---|---|---|---|---|---|---|
| left | 0.0800 | 107 | 0.00118 | 0.08001 | 0.00017 | 0.96 mm | 1.95 mm |
| left | 0.0200 | 124 | 0.07988 | 0.01999 | 0.00019 | 1.46 mm | 1.21 mm |
| left | 0.0800 | 86 | 0.02012 | 0.07999 | 0.00019 | 0.75 mm | 1.02 mm |
| right | 0.0800 | 104 | 0.00111 | 0.07995 | 0.00018 | 0.66 mm | 1.75 mm |
| right | 0.0200 | 120 | 0.07980 | 0.02000 | 0.00019 | 1.61 mm | 0.98 mm |
| right | 0.0800 | 86 | 0.02039 | 0.07997 | 0.00018 | 0.83 mm | 1.42 mm |

要点（避免再次把不同量混为一谈）：

- **稳态误差** ≤0.05 mm；**逼近瞬态**仍在目标附近 ±2 mm 内摆动，最大超出目标 1.61 mm、
  最大低于目标 1.95 mm。上一节的 "0.1 mm" 是**稳态**数字，不是全过程峰值；4.3 mm 越界
  说的是**结果字段超出模型最大开口**，又是另一个量。三者不可互换。
- **判定裕度偏紧**：末 10 样本 spread 实测 0.17–0.19 mm，阈值 0.2 mm，余量约 10%。
  噪声略增就会**超时判失败**（而不是误判成功），方向是安全的，但该裕度需记录。
- **seq 严格递增**：左 4631→7367、右 5172→7826，逐行递增（每行跨 2–613 个 seq），
  即每个进入判定的样本都是**新反馈**，重复读取不计数的语义在实机数据上成立。
- **本模型两指严格对称**：CSV 中 `q1-{q2}` 最大差为 **0**，因此本次**仿真本身没有**
  展示两指取不同值的情形；该情形由 `AsymmetricFingersStillFormTheGapAndSnapshot`
  与 `TotalGapSumsBothFingers` 单测覆盖。`gap = q1+q2` 在代码上成立，在本次数据上
  与 `2×q1` 数值相同。
- 机械臂无轨迹，未发布 weld、未 attach。

## 五、遗留与未做的部分（如实报告）

- `archive/` 未动；控制器切换、weld、XML/SRDF、核心 MuJoCo 环境未改。
- 旧全流程搬运路径中的注释仍写着 `at(0)=0(最大夹持力)`、`at(0) 从 0 跳到 63`
  （`dual_arm_controller.cpp:4040,4358,4360`）。这些是**旧 0–255 单位约定**下的记录，
  与当前米制单指约定不符。该路径不在本轮范围、未被本轮执行，故**未改动**，仅在此记录。
- `scripts/a1_run_scene.sh` 抓取的 `joint_states_after_cycle.txt` 因 `ros2 topic echo --once`
  被超时截断，只含部分关节名，不足以独立证明手指值；本轮以 CSV 与节点日志为准。
- 未做：任何下降、抓取、搬运；未进入 P3a-2b；未新增验收关卡。
- 本轮未提交或推送 Git。

---

# 追加：P3a-2a 宽限后迟到接受收尾（2026-10-14）

本节只补充"目标响应迟到"这一项。共享反馈、滑动稳定窗口、结果快照、渐进控制和
空载周期**未改动**，历史运行目录与 CSV 原样保留。

## 问题

`async_send_goal` 的 future 在目标预算 + 400 ms 宽限内未就绪时，`UNKNOWN_STOP`
分支直接返回，**没有注册任何响应回调**：

- 若目标随后才被接受，那个 goal 句柄没有任何人负责，被永久搁置；
- 下一次开合照常发送新 goal，不受任何限制。

## 修改（仅 `gripper_client.{hpp,cpp}` 与测试）

**1. 每次目标请求一个共享上下文 `GripperGoalContext`**

```
response_arrived / accepted / rejected / handle   // 目标响应回调写入
abandoned                                         // 调用方放弃等待后置真
cancel_requested / cancel_response_received
cancel_accepted / cancel_rejected                 // 三者严格区分
settled                                           // 结果已明确
```

通过 `SendGoalOptions::goal_response_callback` 注册。回调按值捕获
**该上下文、注册表、client 的 `shared_ptr`、`rclcpp::Logger` 的副本**——
不捕获任何栈引用，也不捕获控制器 `this`，因此原调用返回后到达的响应操作的是
仍然存活的 `shared_ptr`。回调内**不做阻塞等待**（`wait_for_response=false`）。

**2. 迟到接受 → 异步取消**

响应到达时若 `abandoned` 已置真且目标被接受，回调立即发出
`async_cancel_goal(..., cancel_response_callback)`，由取消回调记录应答；
若被拒则标记 `rejected` 并结束未知状态。

**竞态闭合**：调用方在置 `abandoned` 的同一把锁内复查 `settled`。若响应恰好在
`waitBounded` 返回与这一行之间到达，回调当时看到 `abandoned==false` 而刻意不动它，
此时就由**调用方自己**发出取消（调用方线程允许有界等待）。没有这一步，该 goal 会
两边都不管。

**3. 未知未解决时拒绝新开合，零次发送**

`GripperPendingRegistry` 持有仍未明确的目标：调用方放弃时**先登记再取消**。
只要取消没人应答，它就一直留在表里，后续 `controlDualGrippers` / `controlGripper`
在发送前直接拒绝并返回 false，**一个 goal 都不发**。取消被应答（受理或被拒）或被拒
目标出现时即 `settle`，门禁重新打开。**没有自动恢复或重试框架**。

**4. 三种状态分别记录，不含"已停止"**

| 状态 | 字段 | 日志 |
|---|---|---|
| 取消请求已发出 | `cancel_requested` | "异步请求取消" |
| 取消被受理 | `cancel_accepted` | "取消请求得到应答：已受理（应答只说明请求被受理…不声称物理已停止）" |
| 取消被拒绝 | `cancel_rejected` | "被拒绝（目标很可能已结束）" |
| 未能发出请求 | `settled` + 警告 | "目标很可能已结束" |

## 测试

`test_gripper_client_faults` **9 例通过**（原 7 例保留，新增 2 例）：

| 用例 | 断言 |
|---|---|
| `AcceptancePastTheGraceWindowIsCancelledAndBlocksTheNextCall` | 见下 |
| `ARunOfNormalCallsIsUnaffectedByTheGate` | 连续 3 次正常调用全成功、各发 3 个 goal；门禁不误伤 |

迟到接受用例按审阅要求构造：目标预算 0.3 s + 宽限 0.4 s，**接受延迟 1.2 s**；
假 server 接受后**保持 EXECUTING 直到收到取消**。断言：

1. 原调用失败，且用时 <1.0 s（没有为迟到接受继续等待）；
2. 未知期间再次调用：**左臂 goal 计数仍为 1**（零次新发送），右臂为 0；
3. **有界等待（4 s）服务端实际收到该迟到目标的取消** `cancels() >= 1`；
4. 取消被应答后门禁重开：新一次调用确实发出 goal（`goals() == 2`）。

实机日志（本机一次运行的原文）印证了完整时序：

```
[夹爪] 闭合 left  | ... E_GRIPPER_GOAL_UNKNOWN no goal response within budget or grace
[夹爪] 闭合 right | ... E_GRIPPER_SKIPPED_AFTER_FAILURE
✗ [夹爪] 拒绝本次闭合：仍有 1 个早期目标结果未知（迟到接受尚未解决）；不发送任何新目标
[夹爪] left 目标在原调用返回 UNKNOWN 之后才被接受；异步请求取消
[夹爪] left 迟到目标的取消请求得到应答：已受理（应答只说明请求被受理或已无目标可取消，不声称物理已停止）
（随后门禁重开，新调用正常发出 goal）
```

**假 server 自身修掉的两个测试缺陷**（非生产代码问题）：接受延迟原先只对
`LATE_GOAL_ACCEPT` 生效；`handle->canceled()` 原先轮询自建计数器，会在
rclcpp_action 完成 CANCELING 状态迁移**之前**触发非法状态转移并 `terminate`，
改为轮询 `handle->is_canceling()`。另在夹具析构时先置 `stop()` 再停执行器，
避免在途 worker 访问已析构的 server。

连续 3 次运行均 9/9 通过（含时序敏感用例）。

## CSV 列名问题（不在本轮改写历史数据）

`a1_p3a2a_feedback_c` 的 CSV 中 `command_per_finger_m` 列**标签有误**：该列实际是
`driveToGap` 内部的**总 gap** 指令值（`writeGapCommand` 才除以 2 变成单指量）。
已按审阅意见做**最小改动**：仅把表头改为 `commanded_gap_m` 并加注释，代码中该变量
本来就叫 `commanded`（总 gap）。**既有 CSV 未改写**，读取时按"总 gap"理解该列。

## 范围与遗留

- 只构建并运行 client/protocol 相关测试：`test_gripper_client_faults` 9 例、
  `test_gripper_protocol` 13 例、`test_gripper_shared_state` 15 例；任务包其余
  既有测试 160 例一并复跑无回归，**合计 184 例通过**。
- **未重跑**空载仿真、A0 或预抓取；**未改动**共享反馈、物理控制、XML/SRDF、增益、
  机械臂控制、weld。
- 已知边界：若取消请求始终得不到应答，门禁会**一直关闭**（拒绝新开合）直到进程结束。
  这是刻意选择的失败方向——宁可停住也不让无人负责的目标继续存在——本轮不引入
  恢复或重试机制。
- 未下降、未抓杆、未启用 weld/attach，未进入 P3a-2b。本轮未提交或推送 Git。

---

# 追加：P3a-2a 未知门禁的条件解除（2026-10-14）

本节只修正"取消未确认时错误开门"这一处。共享反馈、空载开合、迟到响应上下文与竞态
复检**均未改动**，历史运行数据与 CSV 原样保留。

## 问题

`requestCancelForLateGoal` 把"收到取消应答"当成了"旧目标已结束"：

- 取消回调**无论受理还是被拒**都调用 `settleLocked()`；
- `catch` 异常路径也直接移除未知项。

两者都不构成目标已终态的**证据**。被拒的取消意味着服务器明确没有接管这个目标，
旧目标可能仍在执行，而此时门禁已开，新旧命令可以重叠。`catch` 的情况同理，
原注释写的"目标很可能已结束"是推测，不是证据。

## 修改（仅 `gripper_client.{hpp,cpp}` 与测试）

**1. 新增按目标判定取消应答的纯函数**（`gripper_client.hpp`，无 ROS 依赖，可直接单测）

```cpp
enum class CancelVerdict { ACCEPTED, GOAL_TERMINAL, UNCONFIRMED };
CancelVerdict classifyCancelResponse(int return_code,
                                     const std::array<uint8_t,16>& our_goal_id,
                                     const std::vector<std::array<uint8_t,16>>& goals_canceling);
```

| 返回码 | 判定 | 依据 |
|---|---|---|
| `ERROR_GOAL_TERMINATED` (3) | `GOAL_TERMINAL` | 按该目标逐条声明：已处于终态 |
| `ERROR_UNKNOWN_GOAL_ID` (2) | `GOAL_TERMINAL` | 按该目标逐条声明：该目标不存在 |
| `ERROR_NONE` (0) **且**应答的 `goals_canceling` 中**含本目标 ID** | `ACCEPTED` | 服务器确实接管了**这个**目标 |
| `ERROR_NONE` 但未列出本目标 | `UNCONFIRMED` | 该码只说"有一个或多个目标转入 CANCELING"，没说哪个 |
| `ERROR_REJECTED` (1)、其它未知码、无应答 | `UNCONFIRMED` | 无任何终态证据 |

**关键点**：只看 `return_code` 不够。`ERROR_NONE` 的语义是"一个或多个目标转入
CANCELING"，不带目标标识时无法证明说的是本目标，因此必须回落到 `UNCONFIRMED`。
判定时取 `GripperGoalHandle::get_goal_id()` 与 `goals_canceling[i].goal_id.uuid` 逐字节比对。

**2. 只有 `ACCEPTED` 与 `GOAL_TERMINAL` 解除门禁**

- `ACCEPTED` → 记 `cancel_accepted`，**并明确日志说明受理不等于物理停止**；
- `GOAL_TERMINAL` → 记 `goal_terminal_confirmed`；
- `UNCONFIRMED`（含被拒、未匹配、未知码）→ 记 `cancel_unconfirmed`，**保留 registry 项**；
- `catch` 异常 → 同样保留该项，日志改为"无法确认旧目标状态，保持门禁关闭（不移除未知项）"，
  不再出现"目标很可能已结束"这类推测措辞。

**3. 无新框架**：没有自动恢复、没有重试。无法确认时门禁就保持关闭。

**4. 正常路径不变**：`ACCEPTED` 仍然开门，正常连续调用与既有用例行为不变。

## 测试

`test_gripper_client_faults` **11 例通过**（原 9 例保留，新增 2 例）：

| 用例 | 内容 |
|---|---|
| `RefusedCancelKeepsTheGateShutAndTheOldGoalRunning` | 迟到接受 → **拒绝取消** → 旧目标持续执行 |
| `CancelResponseClassifier.OnlyPerGoalEvidenceIsTerminal` | 上表全部六种输入 |

被拒取消用例的断言（假 server 接受延迟 1.2 s，取消回调返回 `REJECT`，执行线程**永不结束**）：

1. 原调用失败，左臂 goal 计数为 1；
2. **有界等待（4 s）服务端实际收到取消请求** `cancels() >= 1`；
3. **旧目标确实仍在执行** `executing() >= 1`（执行中计数由 RAII 守卫维护）——
   没有这一条，"目标保持执行"就只是断言里的一句话；
4. `controlGripper` 与 `controlDualGrippers` **均返回 false 且拒绝**；
5. 左右目标计数**不增加**（左仍为 1，右为 0），且不产生新的取消请求。

本机一次运行的原文日志：

```
[夹爪] 闭合 left | ... E_GRIPPER_GOAL_UNKNOWN no goal response within budget or grace
[夹爪] left 目标在原调用返回 UNKNOWN 之后才被接受；异步请求取消
[夹爪] left 迟到目标的取消未被确认（返回码 1，应答列出 0 个取消中目标，本目标不在其中或请求被拒）；
       无法确认旧目标已结束，保持门禁关闭
✗ [夹爪] 拒绝本次单臂指令：仍有 1 个早期目标结果未知（迟到接受尚未解决）；不发送任何新目标
✗ [夹爪] 拒绝本次闭合：仍有 1 个早期目标结果未知（迟到接受尚未解决）；不发送任何新目标
```

该用例与此前 `AcceptancePastTheGraceWindowIsCancelledAndBlocksTheNextCall`（取消被受理，
断言门禁**重新打开**、`goals()==2`）构成一对：同一段客户端代码、相反的服务器行为、
相反的断言。若客户端仍无条件解除门禁，本用例会看到 `goals()==2` 而失败——即该用例
确实能区分修复前后，不是必然通过的断言。

连续 3 次运行均 11/11 通过（含两个时序敏感用例）。

## 范围与遗留

- 只构建并运行相关 client/protocol 测试；任务包其余既有测试一并复跑无回归，
  **合计 186 例通过**（client 11、protocol 13、shared-state 15、既有 147）。
- **未重跑**仿真、A0、预抓取；**未改动**物理控制、共享反馈、XML/SRDF、增益、机械臂、
  weld；历史 CSV 未改写。
- 已知并刻意保留的边界：取消被拒、无应答或无法匹配时，门禁**一直关闭**到进程结束。
  这是本轮选择的失败方向——宁可停住，也不让一个可能仍在运行的目标与新命令重叠。
  代价是这种情形下夹爪会拒绝后续开合而不会自愈，本轮不引入恢复机制。
- 未下降、未抓杆、未启用 weld/attach，未进入 P3a-2b。本轮未提交或推送 Git。
