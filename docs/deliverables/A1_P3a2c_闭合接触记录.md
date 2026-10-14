# A1 P3a-2c 闭合接触记录

本阶段只在本轮实际到达 `GRASP_POSE_REACHED` 后显式闭合夹爪并验证杆接触。杆全程仍由支撑面托住；没有启用 weld/attach，没有抬升或搬运，不声称承载或抗滑成功。

## 几何与接触身份

- MuJoCo 杆为 `obj_rod_01` body/geom，box 半尺寸 `(0.02, 0.20, 0.02) m`，长轴为杆局部 Y。
- MuJoCo 手指 body 名为 `mj_left_left_finger`、`mj_left_right_finger`、`mj_right_left_finger`、`mj_right_right_finger`；对应 MoveIt/SRDF link 名使用不带分隔下划线的 `mj_{left,right}_{left,right}finger`。物理反馈按 MuJoCo body ID 对 MuJoCo contact geom pair 分类，不按 SRDF 别名猜测。
- 物理线程启动日志列出手指 body 下的实际 geom ID，以及杆 geom `obj_rod_01`；共享反馈同时携带左右指接触布尔值、真实 `q1/q2` 和更新序号。ROS 回调只读消息快照。
- Franka `finger_joint1/2` 的 URDF 轴分别为手/关节坐标系 `+Y/-Y`。不能只抵消两个 yaw：实际旋转链是 `Rz(θ+45°) Rx(π) Rz(-45°) = Rz(θ+90°) Rx(π)`。作用于局部 `+Y` 的闭合轴沿杆局部 `X`（短轴/宽度方向），与杆局部 `Y` 长轴垂直；期望抓取姿态下，杆沿闭合轴的几何投影约为 `0.04 m`，而沿长轴投影为 `0.40 m`。运行时还需由 MoveIt finger joint 实际轴及 FK 核对；实际接触 gap 始终以四指反馈和 geom contact 为准。
- 单指命令位置是每指位移；动作目标总 gap 按 action 定义计算；MuJoCo 实际总 gap 取 `q1 + q2`。三者在记录中分别保留。

## 运行前判据

- 新开关 `grasp_contact_close` 默认关闭，必须与 `grasp_descent`、本轮 `pregrasp_execute`、A1 truth、`prepare_only` 以及 P2a/P2b 检查链同时满足；与 `gripper_selftest` 互斥。开关关闭时终态仍是 `GRASP_POSE_REACHED`。
- 接触 action 复用渐进有限速度命令。每只手只在左右手指分别对 `obj_rod_01` 接触，且 16 个新物理反馈样本的总 gap 整窗 spread ≤ `0.0002 m` 时报告协议成功；空载目标到达但未双指接触会 ABORT。action 超时、ABORTED、CANCELED、UNKNOWN 均为失败。
- 任务层在两只手都完成 action 后，另外要求每只手各 16 个新 ROS 反馈序号、每指均持续接触且总 gap spread ≤ `0.0002 m`（以 50 Hz 发布约 0.30 s）。这层任务验收与 action 协议结果分别记录。
- 闭合前先读取 MuJoCo 杆真值，继续要求相对冻结真值误差 ≤ `0.005 m / 5°`。闭合前后记录两臂关节反馈偏差，验收上限 `0.002 rad`；记录物理杆位姿变化。
- 闭合状态碰撞检查使用 P2a 独立验证场景与其 `grasp_acm` 副本，只容许四个手指 link 与杆对象接触；手掌、地面、其余环境和双臂碰撞仍检查，不改全局 ACM/SRDF。
- 不启用力控，不把 action `effort` 当作测量；接触阶段目标仅为确认接触，杆仍由支撑面支撑。

## 验证与结果

- 构建：`colcon build --packages-select dual_arm_a1_msgs franka_hardware dual_arm_carry_task`，通过。
- 针对性测试：`test_a1_sample_validator`、`test_gripper_protocol`、`test_gripper_client_faults`、`test_gripper_shared_state` 均通过。新增 validator 用例覆盖四指非零且彼此不同的按名映射、缺失/旧序号/过旧/冲突来源拒绝、只有类型化实测时显式映射，以及完整非交换旋转链在多组 yaw 下沿杆短轴的方向。协议测试覆盖无接触、单侧、双侧稳定、重复旧序号与仅 gap 稳定不通过。
- `a1_p3a2c_descent_off_regression`：启动时默认 ROS 日志目录只读，仿真未进入服务就绪；原始目录保留。
- `a1_p3a2c_descent_off_regression_b`：仿真已启动，但沙箱拒绝 ROS DDS discovery socket，120 s 内服务未就绪；未启动任务，原始目录保留。
- `a1_p3a2c_descent_off_regression_c`：设置 `/tmp` 下的 `ROS_HOME` 后服务、MuJoCo 真值、任务节点均正常；P2a/P2b 检查通过，预抓取运动到位并完成双夹爪空载打开。随后 R1 对最终轨迹的严格碰撞检查在 51 个状态（26 个航点、25 段细分）发现 52 处杆—手指碰撞（四指均有报告），地面/支撑面、双臂互碰、自碰撞及其他世界碰撞均为 0。R1 按既有规则拒绝下发下降运动，因此本轮没有到达 `GRASP_POSE_REACHED`，也没有发送物体闭合 action。记录在 `experiment_results/a1_runs/a1_p3a2c_descent_off_regression_c/`。
- 状态来源定位：本次空载开爪 action 由夹爪 server 私有硬件反馈确认总 gap 左 `0.080031 m`、右 `0.079989 m`；类型化 `/gripper_contact_feedback` 消息携带物理 `q1/q2`、physics `update_seq` 和消息时间。旧下降代码没有消费该类型化快照，而直接把 `MoveGroupInterface::getCurrentState()` 复制进 criterion state，因此本轮旧日志 final_base 四指最大值为 `0.0000 m`，此前成功记录为 `0.0400 m`。本次运行进一步确认 `joint_state_publisher` 一直等待 `/robot_description`，聚合 `/joint_states` 和 MoveIt CurrentStateMonitor 均没有四指变量；launch 源话题名虽已是实际私有话题，但 publisher 没完成 URDF 配置。旧 R1 杆—指报告是在 criterion 状态四指值为零时的模型结果，不能据此认定 MuJoCo 中打开手指发生了真实碰撞。
- 状态交接修复：开爪 action 成功后最多等待 `2.0 s`。每侧必须收到高于开爪前序号的新鲜类型化反馈，消息时间非零且接收年龄 ≤`0.50 s`；若 `/joint_states` 带有完整四指值则要求逐项差 ≤`0.0001 m`，来源冲突或只包含部分四指即停止。确认全局消息没有任何四指项时，使用 fresh typed snapshot 显式覆盖 CSM 状态，并将同一状态用于下降 IK seed 与最终 criterion base；映射前 CSM 值、映射后值、源序号和消息时间均记录。14 个手臂轨迹关节仍按关节名更新，不清零四指。
- `a1_p3a2c_finger_state_regression`：新快照逻辑首次运行时发现 global `/joint_states` 缺少 `mj_left_finger_joint1`，因此在规划/下降前停止；记录保留在该 run 目录。
- `a1_p3a2c_finger_state_regression_b`：同 yaw +10°、接触开关关闭。新鲜类型化源序号为 left `12567`、right `12562`，源时间戳分别为 `25131999999 ns`、`25121999999 ns`；实测四值按 `mj_left_finger_joint1/2`, `mj_right_finger_joint1/2` 顺序为 `0.040000, 0.040000, 0.040011, 0.040011 m`。global 四指项缺失；同一时刻 CSM 四值全为 `0.000000 m`，映射后四项逐值等于硬件反馈。运行时 MoveIt finger joint1 轴经 FK 得到左 `(0.985018,0.172454,-0.000216)`、右 `(0.985016,0.172462,-0.000492)`；杆局部 Y 世界轴约 `(-0.173648,0.984808,0)`，闭合轴与杆长轴夹角均为 `89.931°`，与非交换旋转链预测差 `0.00123/0.00130`。严格 R1 final trajectory 在 51 个状态中碰撞 `0`，随后实际下降到达 `GRASP_POSE_REACHED`。详细日志和轨迹保存在该 run 目录。
- `a1_p3a2c_contact_statefix_yawp10`：满足前一回归的下降门槛后按约定启动显式接触运行。但 P1b runner 首先报告 `/set_body_state`、`/get_body_state`、`/get_sim_info` 服务不可用，未发布本轮 truth，任务节点未启动；仿真日志显示 controller spawner 最终无法连接 `/controller_manager/list_controllers`。该运行未发送夹爪闭合 action，原始数据保留在该 run 目录。按约定在实质运行故障后停止，不重试。
- 本轮没有 `GRASP_CONTACT_CONFIRMED`，也没有接触 gap、左右指接触身份/更新序号、闭合后臂偏差或杆位姿变化的接触阶段记录。接触协议/判据实现与单测仍保留，但接触功能验收尚未完成；本阶段不声称杆已承载、抗滑或搬运成功。
- 历史记录 `a1_p3a2b_final_verify_regression` 曾在同一名义非零 yaw 场景通过 R1 并到达 `GRASP_POSE_REACHED`，但不替代本轮实际运行结果。本轮保留失败证据，不重试。

## 最新运行（2026-10-14）：到达抓取位姿，但闭合入口未传入节点

- Run：`a1_p3a2c_contact_yaw10_20261014`，使用 x=0.50、y=0、yaw=+10°；`ROS_HOME=/tmp/a1_p3a2c_ros_home`、`ROS_DOMAIN_ID=77`、`ROS_LOCALHOST_ONLY=1`。启动前无其他仿真实例。只读真实请求确认 `/get_body_state(obj_rod_01)`、`/get_sim_info`、`/controller_manager/list_controllers` 成功，`dual_panda_arm_controller=active`，`sim_multi_mode_controller` 未 active，左右 `GripperCommand` action server 均可用。
- 首次探测器误用了 Conda Python 3.13，无法加载 Humble Python 3.10 的 `rclpy` 扩展；原始错误保存在 `readiness_probe.json`。改用系统 `/usr/bin/python3` 后在同一仿真实例完成真实请求检查，没有重启仿真。
- Scene runner 成功应用并发布本轮 truth，杆中心 `(0.500000, 0.000000, 0.020654) m`。任务 P2a/P2b 通过，预抓取到位；打开 action 左/右均 `SUCCEEDED`（结果 gap `0.079998/0.080020 m`）。下降使用新鲜类型化四指快照：`mj_left_finger_joint1/2=0.040000/0.040000 m`、`mj_right_finger_joint1/2=0.040012/0.040012 m`，源序号 left/right=`82458/82459`，消息时间 `164914000000/164916000000 ns`。global `/joint_states` 缺四指项，映射后均与该实测值一致。
- 本轮 R1 严格最终轨迹检查 51 个状态、碰撞 0；未放行指-杆接触。下降新鲜反馈窗口为 30 点 / `0.300 s`，最大单关节变化 `0.002000 rad`（保持既定门槛），末端误差左 `0.5755 mm / 0.3874°`、右 `0.6580 mm / 0.3549°`，实际到达 `GRASP_POSE_REACHED`。下降前后 MuJoCo 杆真值变化为 `0.0000 mm / 0.0000°`。
- 随后任务保持开爪并结束在 `GRASP_POSE_REACHED`，没有闭合 action 结果、接触窗口判定或 `P3A2C_RESULT`。反馈 CSV 覆盖 `2026-10-14T04:29:49Z` 至 `04:31:07Z`，左右各 3898 条；序号分别从 left `71629→110596`、right `71630→110597`，记录到的左右指对杆接触标志全为 false，末尾总 gap 均为 `0.080000 m`。闭合后的臂偏差与杆变化没有测得，因为闭合根本未执行。
- 具体原因已定位：源和已安装的 `dual_arm_carry_task.launch.py` 都声明了 `grasp_contact_close` launch 参数，但任务节点 `Node(parameters=[...])` 漏传该参数；状态机因此使用默认 `false`。这解释了日志只报告 `GRASP_POSE_REACHED`、夹爪保持打开，不是 action ABORTED，也不是物理接触判据失败。本轮没有修改生产代码；发现任务已经开始后即停止，不使用额外启动额度重试。只对本轮拥有的 task、truth runner、反馈订阅器和仿真进程组发送停止信号。
- 运行证据位于 `experiment_results/a1_runs/a1_p3a2c_contact_yaw10_20261014/`，包括 `runner.log`、`manifest.json`、`task_launch.log`、`task_key_events.txt`、`readiness_probe_system_python.json`、`contact_feedback.csv` 和运行命令/状态记录。

当时状态（参数修复前）：四指状态交接问题已由 `a1_p3a2c_finger_state_regression_b` 解决；但 launch 参数未传给任务节点，P3a-2c 双侧闭合接触尚未实测。后续修复及运行结果见下方追加记录。

## 追加运行（2026-10-14）：launch 参数修复后首次显式接触

- 修复 `dual_arm_carry_task.launch.py` 任务节点参数映射，显式把 `grasp_contact_close` LaunchConfiguration 传给节点，默认仍为 `false`；节点启动日志打印实际读取值。源及安装目录均已核对。`a1_p3a2c_contact_paramfix_yaw10_20261014` 的无运动参数冒烟中，显式 true 被节点读取为 `true` 并因缺少 `grasp_descent` 按预期拒绝；省略参数时节点读取 `false`。日志与命令保存在该 run 目录。
- 同一 run 使用 x=`0.50 m`、y=`0`、yaw=`+10°`，ROS_HOME=`/tmp/a1_p3a2c_ros_home`、ROS_DOMAIN_ID=`77`，仅启动一个仿真。truth、controller/action 就绪检查通过；位置控制器 active，多模式控制器未 active。P2a/P2b、预抓取、打开和下降均通过。空载打开 action 成功；下降使用新鲜四指实测快照，关节顺序 `mj_left_finger_joint1/2`, `mj_right_finger_joint1/2` 的值为 `0.040000, 0.040000, 0.040022, 0.040022 m`。类型化源序号 left/right=`41074/41079`；global `/joint_states` 缺四指项，映射值逐关节与实测值一致。
- R1 严格轨迹检查为 51 状态、0 碰撞，未放行指-杆接触；下降反馈窗 30 点 / `0.300 s`，最大变化 `0.001980 rad`，到达 `GRASP_POSE_REACHED`。下降前后杆真值无变化。闭合前窗口预先设为每手 16 个新序号样本（约 `0.30 s`）、gap 整窗 spread ≤`0.0002 m`、机械臂最大关节偏差 ≤`0.0020 rad`，杆真值容差 `0.005 m / 5°`。
- 显式闭合 goal 已发送：左侧 Grasp action server 收到 goal，客户端等待超时并请求取消；服务端以 `contact_unproven` 报告 `FAILED`（最终 gap `0.041388 m`、协议窗口样本数 0）。由于左侧失败，右侧没有收到闭合 goal。任务终态 `FAILED / E_P3A2C_ACTION_FAILED_OR_UNCERTAIN`，不是成功；没有发布 `GRASP_CONTACT_CONFIRMED`。最终反馈左侧 `q1/q2=0.020502/0.020502 m`、总 gap `0.041004 m`、两指接触标志均为 true、序号 `46905`；右侧 `q1/q2=0.040000/0.040000 m`、总 gap `0.080000 m`、两指接触均 false、序号 `46900`。左侧曾短暂出现杆接触，反馈身份有波动，未形成合格稳定窗口；右侧无杆接触。反馈订阅 CSV 保存逐条接收时间、消息时间、四指位置、gap、两侧接触身份及序号。
- 闭合前后机械臂最大关节偏差 `0.007788 rad`（`mj_left_joint7`，超过 `0.002000 rad` 门槛）；杆物理真值变化 `0.000362 m`、转角 `0.02342°`。因此 action 失败、双侧任务接触验收失败，且机械臂稳定门槛也未满足。没有重试、没有改阈值或碰撞豁免；只停止本次运行拥有的进程。杆仍由支撑面托住，未启用 weld/attach，未抬升或搬运。
- 数据位于 `experiment_results/a1_runs/a1_p3a2c_contact_paramfix_yaw10_20261014/`，含实际参数日志、R1/下降/闭合事件、动作服务日志、杆与关节记录、反馈 CSV、运行命令及进程清理记录。四指 R1 状态交接问题已解决；本次 launch 断链已修复，但 P3a-2c 尚未达到 `GRASP_CONTACT_CONFIRMED`。本轮按实质接触失败停止。

## 追加运行（2026-10-14）：接触后保持目标实验

- 离线复核 `a1_p3a2c_contact_paramfix_yaw10_20261014`：左 Grasp goal 在 `04:38:52.818756Z` 收到，目标为零 gap、速度 `0.1 m/s`；首个双指对杆接触是 `04:38:53.242919Z`、seq `43114`、gap `0.0420551 m`。取消时刻 `04:39:00.819373Z`，与旧 server `8 s` 截止 `04:39:00.828369Z` 几乎重合。取消前左侧有 333 个双指接触样本、分 31 段，最长连续 41 点；这些接触样本 gap 为 `0.0410027–0.0420551 m`，整窗 spread 超过 `0.0002 m`。这是反复真实双指接触但 gap 不稳定，不是“完全没碰到杆”。旧 run 没有逐循环命令 trace，命令目标按源码与 goal 记录为 0 gap、渐进闭合。
- 实现仅作用于显式 Grasp 接触路径：首次新鲜双指接触后固定 target=`max(当时命令gap, 实测gap)`，预压量 `0 m`；之后只限速移动到该 target，不再向内推进。触杆身份丢失时清除 16 点窗口、保留固定 target；单指接触不进保持、不通过。截止改为 server `6.0 s`，client 等待最多 `7.5 s`，留 `1.5 s` 返回余量。空载 Move/GripperCommand 仍走原分支；没有使用未实现的 force 字段。
- `franka_hardware` 共享 server 判据测试 22 项、`dual_arm_carry_task` 的 `test_gripper_protocol` 和 `test_gripper_client_faults` 均通过。首次宽泛 `colcon test` 同时触发了既有格式/XML lint 失败与默认只读 ROS_HOME 初始化问题；随后将 ROS_HOME 指向 `/tmp`，对以上三个测试定向运行，全部通过。相关包 `franka_hardware`、`dual_arm_carry_task` 构建通过。
- 首次启动 run `a1_p3a2c_contact_hold_yaw10_20261014` 在 truth/task 前因沙箱拒绝 DDS UDP socket，120 s 内服务未就绪；runner 自动停止本次拥有的单一仿真。保留日志后，在独立 run 目录重新启动一次，未并行保留仿真实例。实际 run `a1_p3a2c_contact_hold_yaw10_20261014_retry1` 使用 x=`0.50 m`、y=`0`、yaw=`+10°`，truth 和 controller inventory 到位；任务确认 `dual_panda_arm_controller=active`、多模式控制器非 active。P2a/P2b、预抓取、开爪和 R1–R3 均通过；R1 严格检查 51 状态、0 碰撞。下降 30 点 / `0.301 s`，最大变化 `0.001972 rad`。新鲜四指值 left=`0.040000/0.040000 m`、right=`0.040014/0.040014 m`，源序号 `13273/13272`，时间 `26543999999/26541999999 ns`。
- 左侧接触 goal 已发送，右侧未发送。server 首次看到双侧触杆时 seq=`15497`、实测 gap=`0.041973 m`，上一个命令 gap 已到 `0.034000 m`，hold target 固定为 `0.041973 m`（零预压）。随后 CSV 中 command 不再内向推进；hold 阶段实测 gap=`0.0418821–0.0423419 m`，spread `0.0004598 m`；双侧触杆共 29 点，最长连续 3 点，接触反复丢失，验收窗不断重置。6 s deadline 后 action `ABORTED`，协议失败、没有 `GRASP_CONTACT_CONFIRMED`。这证明 inward push 已停止，但当前固定 gap 下仍无法维持稳定双指接触；不据此调整预压或速度，本轮停止。
- 机械臂基线单独观测 `0.302 s / 15` 个新反馈样本：自然保持最大 excursion `0.001557 rad`（`mj_left_joint7`），FK 左漂移 `0.115 mm / 0.07573°`、右 `0.130 mm / 0.06898°`。从闭合起点到 6 s action 结束的附加区间最大 joint delta `0.006199 rad`；FK 左 `0.448 mm / 0.29309°`、右 `0.505 mm / 0.26825°`。从原始闭合前快照到结束总偏差 `0.007755 rad`，不能把它全归因于夹紧；contact 区间也包含其间的持续漂移。杆真值变化 `0.000014 m / 0.00441°`。`0.002 rad` 门槛保持不变且本次超限，保留失败。
- 原始 trace、任务/action/server 日志和失败启动日志分别保存在两个新 run 目录；实际 trace 有一条内容为 `move` 的畸形行，原始文件保留，另附去除该行的 `gripper_trace_valid_rows.csv`（698 条有效记录）。最终状态为 `FAILED / E_P3A2C_ACTION_FAILED_OR_UNCERTAIN`，无任务级双侧稳定 gap 验收；任务开始后未重试或扫描参数。未 weld/attach、抬升或搬运。

## 追加运行（2026-10-14）：0.01 m/s 接近与 0.5 mm 固定预压

- 唯一 run：`a1_p3a2c_contact_preload_yaw10_20261014`，x=`0.50 m`、y=`0`、yaw=`+10°`，`ROS_HOME=/tmp/a1_p3a2c_preload_ros_home`、`ROS_DOMAIN_ID=82`、`ROS_LOCALHOST_ONLY=1`。启动前该 domain 无实时 ROS 服务、进程表无仿真栈；本次只启动一个包含 MuJoCo 的 MoveIt 仿真。节点实际参数日志为 `grasp_contact_close=true`。controller inventory 中 `dual_panda_arm_controller=active`，`sim_multi_mode_controller` 不在 active 列表。
- P2a/P2b 通过，预抓取到达；夹爪打开 action 成功（总 gap 左=`0.079988 m`、右=`0.080007 m`）。下降沿用新鲜类型化四指快照：left=`0.040000/0.040000 m`、right=`0.039983/0.039983 m`，源序号 left/right=`13595/13602`；CSM 四值原为 0，映射后逐关节等于实测值。R1 严格轨迹检查 51 状态、碰撞 0，未放行指-杆接触。下降窗口 31 点 / `0.304 s`、最大关节变化 `0.002000 rad`，末端目标误差左 `0.567 mm / 0.382°`、右 `0.648 mm / 0.350°`；实际到达 `GRASP_POSE_REACHED`。
- 左侧显式 Grasp goal 已收到，速度目标为总 gap `0.01 m/s`；右侧未发 goal。trace 中 approach 末段命令变化约 `0.0001 m / 0.010 s`（`0.00980 m/s`）。首次新鲜双指接触 seq=`17597`，实测 gap=`0.041966 m`，此前命令 gap=`0.040300 m`；trace 对应限速后的命令=`0.040400 m`，命令减实测=`-1.566 mm`。锁存仅发生一次，保持目标=`0.041466 m`（总预压=`0.500 mm`）。hold 后命令到达并固定在该目标，未继续向零 gap 推进。
- 左侧 hold 阶段 194 条 trace 行中有 28 条双指对杆接触，最长连续仅 2 行；双指接触样本 gap=`0.0419433–0.0419978 m`，但接触身份多次丢失，服务器每次重置稳定窗。该阶段全部 gap 范围 `0.0419433–0.0422138 m`，spread=`0.271 mm`；在服务器 6 s deadline 到期时窗口为 `0/16`，最终 gap=`0.042057 m`、seq=`18579`，命令与实测差=`-0.591 mm`。左 action 结果 code=`6`（ROS `ABORTED`）、`success=0`、协议 `contact_unproven`；任务级右夹爪仍打开且未收到闭合 goal。最终 ROS 接触反馈 left gap=`0.042074 m`、两指标志 false、seq=`18765`；right gap=`0.080000 m`、两指标志 false、seq=`18770`。终态 `FAILED / E_P3A2C_ACTION_FAILED_OR_UNCERTAIN`，未输出 `GRASP_CONTACT_CONFIRMED`。
- 机械臂闭合前自然保持观测 `0.302 s / 15` 个新样本（seq `9549→9708`）：最大 excursion=`0.001534 rad`（left joint7），FK 左漂移 `0.114 mm / 0.07462°`、右 `0.128 mm / 0.06796°`。动作结束后的新鲜观察为 `0.343 s / 35` 点，最大逐关节 range=`0.000017 rad`，反馈年龄 `0.013 s`，短窗通过；相对原抓取目标的 FK 误差左 `0.040 mm / 0.0042°`、右 `0.015 mm / 0.0016°`，通过 `5 mm / 2°`。原闭合前快照至结束累计时间 `6.673 s`，最大关节位移=`0.007750 rad`（left joint7）；从预观测末端至结束为 `0.006217 rad`，其中包含约 6 s 自然持续漂移，不能全归因于夹爪接触。接触起点至结束 FK 漂移左 `0.465 mm / 0.29533°`、右 `0.506 mm / 0.26892°`，单独记录，不作为累计 0.002 rad 超限拒绝条件。
- 杆闭合前 MuJoCo 真值为 `(0.500000, 0.000000, 0.020654) m`；结束真值读取有效，位移=`0.000030 m`、转角=`0.00305°`。本次既有日志只保存杆结束有效标记与位移/转角差，没有单独落盘结束 xyz，故不补造结束坐标。杆仍由支撑面托住；无 weld/attach、抬升或搬运。本轮按接触 action 实质失败停止，未重试或扫描参数。
- 记录文件位于 `experiment_results/a1_runs/a1_p3a2c_contact_preload_yaw10_20261014/`；`gripper_fingers_.csv` 保存逐条物理 trace（relative time、命令/实测 gap、target、阶段、接触身份、序号），`task_launch.log` 与 `sim_dual_moveit.log` 保存参数、action、窗口、机械臂和杆变化。构建通过；定向测试最终通过：共享夹爪状态 23 项、接触协议 16 项、客户端故障 11 项、新增任务验收 3 项。
