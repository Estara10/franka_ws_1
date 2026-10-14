# A1 P3a-2b：双臂下降到抓取位姿（到位即停）

## 本轮范围

在 P3a-1 已到位预抓取位姿的基础上，打开双爪并沿世界 Z 下降到抓取位姿，**到位即停**。
成功输出独立的 `GRASP_POSE_REACHED`。

**本轮不做**：闭合抓杆、启用 weld、attach、抬升、搬运、放置。夹爪在整段结束时**保持打开**。

## 一、显式入口与门禁

新增 `grasp_descent`（默认 **false**）。为 true 时在**构造期**要求：

| 条件 | 不满足时的行为 |
|---|---|
| `pregrasp_execute=true` | ERROR，非零退出，**不初始化控制器** |
| 与 `gripper_selftest` 互斥 | ERROR，非零退出 |
| `descent_steps > 0` | ERROR，非零退出 |

`pregrasp_execute=true` 本身已经要求 `a1_truth`、`use_vision=false`、`prepare_only=true`、
`p2a_check=true`、`p2b_check=true`，因此下降是预抓取入口的严格延伸，**不可能进入旧完整
GRASP/FSM**，也不会解锁 `prepare_only=false`。

下降前用**已通过的现有夹爪客户端**打开双爪，并要求真实反馈到位；打开失败、或存在未解决的
未知目标门禁（P3a-2a 的拒绝路径）时，**不下降**并报 `E_2B_GRIPPER_OPEN_FAILED`。

实测三条拒绝路径（无需仿真，节点单独启动）：

```
✗ grasp_descent=true 需要 pregrasp_execute=true（下降必须从本轮实际达到的预抓取位姿开始）
✗ gripper_selftest 与 pregrasp_execute 不能同时启用
✗ descent_steps 必须为正（当前 0），不把一次跳跃当作下降轨迹
⇒ 均「参数组合无效（构造期已拒绝），未初始化控制器，非零退出」，退出码 1
```

## 二、下降轨迹的生成

- **同一冻结真值 + 现有目标生成器**：复用 `buildDualTarget`，沿用既定
  `approach_height=0.28`、`grasp_height=0.13`、抓取偏移 0.15 m、末端基准 yaw 45°。
  新增的 `buildDescentSchedule(truth, geom, steps)` 只在高度上插值，因此
  **两端就是 P2a 已校验的预抓取/抓取目标**，实测端点差 `0.000000 mm / 0.000000°`。
- **起点 = 本次实际状态**：从 `getCurrentState()` 映射进判据模型，并要求与**刚到位**的
  预抓取解的手臂关节差 ≤ 0.02 rad（本次 0.002802 rad）。**不使用默认姿态或缓存位姿**。
- **组合状态协调 IK**：21 个目标在**同一个 `RobotState`** 上依次求解（左臂 + 右臂），
  每一步以上一步的解为种子，取改动最小的可行解。**不是两条独立单臂轨迹**。
- **IK 在 live 模型上求解，再映射进判据模型**：判据模型由 URDF+SRDF 独立装配，不含
  IK 求解器插件，直接求解会因与下降无关的原因失败。运动学两模型一致，判据模型只替换
  手掌碰撞几何。

## 三、检查（下发前）

| 检查 | 结果 |
|---|---|
| 21 个航点限位 | 通过 |
| 相邻航点最大单关节变化 | 0.0277 rad（第 6 段），阈值 2.0 |
| **整段竖直性（FK 实测，对每臂首点）** | 左 max XY 0.0001 mm / 0.0001°；右同（阈值 2 mm / 1°） |
| 碰撞：航点 | 21 个航点，**0 处** |
| 碰撞：段内插值 | 20 个状态（步长 0.05 rad），**0 处** |
| TOTG 时间参数化 | 26 轨迹点，2.405 s |
| 最终轨迹复核 | 14 关节名唯一完整；末点 t=2.405 s；速度字段 26/26 点（最大占用比 0.1000）；加速度字段 26/26 点（最大占用比 0.1000） |
| 最终点 FK 残差（对抓取目标） | 左 0.0000 mm / 0.0000°，右 0.0000 mm / 0.0000° |

**竖直性的口径**：本轮的下降是「沿世界 Z 的笛卡尔插值 + 逐点联合 IK」，不是关节空间 PTP。
上表是**由实际 FK 测得**的偏差，报告只用这个测量值说明"接近竖直"，**不把关节轨迹直接
称为竖直直线**。

**碰撞口径**：全程使用**严格 ACM**（`base_acm`），**没有**启用抓取高度的手指-杆豁免，
也**没有**通过全局 ACM 放行手掌或地面。本次未出现任何指-杆接触；若出现，本轮会**停止并
报告**，而不是就地放宽规则。这里的检查是**离散采样**（航点 + 步长 0.05 rad 的插值），
**不是连续碰撞的数学保证**。

## 四、执行与到位

低速初值 `pregrasp_vel_scale = pregrasp_acc_scale = 0.100`；复用 P3a-1 的有界执行助手、
控制器互斥检查和起点一致性检查（当前关节与轨迹首点最大差 0.002697 rad，阈值 0.020）。

执行返回码 1（MoveIt SUCCESS），用时 2.554 s（计划 2.405 s）。

到位采用**新的短反馈窗口**（6 次采样 / 0.30 s，窗口内最大关节变化 0.000731 rad），
功能容差沿用 **5 mm / 2°**：

| | 位置误差 | 姿态误差 |
|---|---|---|
| 左臂 | **0.9690 mm** | **0.6424°** |
| 右臂 | **1.1115 mm** | **0.5879°** |

到位后对**实际达到的航点**再测整段竖直性：左/右 max XY **0.0001 mm**、max 姿态 **0.0001°**。

## 五、杆的状态：物理测量与场景对象分开报告

```
下降前 MuJoCo 物理真值（/get_body_state）: (0.500000, 0.000000, 0.020654)
                                          q = (0.00000, -0.00000, 0.08716, 0.99619)
物理杆变化（前→后）        : 位移 0.0000 mm，转角 0.0000°   ← MuJoCo 物理
规划场景对象变化（前→后）  : 位移 0.0000 mm，转角 0.0000°   ← /get_planning_scene，非物理测量
```

两者**分别记录、不互相代替**。若 MuJoCo 真值服务不可用，日志明确写"**未测得**，本轮不
声称杆未被扰动"，而不是拿场景对象充当物理测量。

## 六、终止状态

```
[P3A2B_RESULT] state=GRASP_POSE_REACHED detail=GRASP_POSE_REACHED
[prepare] P3a-2b: GRASP_POSE_REACHED (arms only; gripper still OPEN; no grasp, no carry)
prepare exit code: 0
```

日志全文检索确认：**无**夹爪闭合、**无** `weld_active`、**无** attach、**无**抬升。
`GRASP_POSE_REACHED` 是独立终态，**不是**任务 DONE。

## 七、回归与测试

**默认不开下降的回归**（`a1_p3a2b_descent_off_regression`，同一 yaw=10° 场景）：
`PREGRASP_REACHED`，退出码 0，日志中 `P3A-2B` 行数 **0** —— 关闭开关时该路径完全不被进入，
原有行为保持不变。

**单测**：`test_a1_sample_validator` **54 例**（新增 6 例），覆盖
`buildDescentSchedule` 的"steps≤0 返回空"、"两端等于既有目标"、"逐点只改高度"，
以及 `straightDescentDeviation` 的直线零偏差、水平漂移与旋转分别被测出、空序列与非有限
序列被拒。

**全套**：192 例通过（sampling 26、stability 16、service_faults 35、truth 22、
sample_validator 54、gripper_protocol 13、gripper_client_faults 11、gripper_shared_state 15）。

## 八、过程中暴露并修复的问题（保留原样，不隐藏）

| 运行 | 现象 | 原因 | 处理 |
|---|---|---|---|
| `..._yawp10b` | `E_2B_NO_VERIFY_CONTEXT` | 我把 `descent_start` 这个**输出**写进了前置门禁，形成循环依赖 | 门禁改为只检查判据模型/场景 |
| `..._yawp10c` | `E_2B_START_NOT_PREGRASP` 0.040004 rad | 恰好等于指关节位移：起点比对把**因本轮开爪而变化**的指关节算了进去 | 起点比对只看**手臂**关节，日志明示指关节不参与 |
| `..._yawp10d/e` | `E_2B_IK_NO_SOLUTION`（首点即失败） | 判据模型无 IK 求解器插件，在其上求解必然失败 | IK 改在 **live 模型**求解后映射进判据模型（与 P2a 同法） |
| `..._yawp10f` | 节点 Eigen 断言崩溃（`Vector3d::operator()` 越界） | `collectArmJoints` 是**追加**语义，我复用了同一个名字向量，臂名累积到 28 而数值仍为 14 | 名字向量固定一次（14 个），每处取值用独立临时向量，并加维度校验 |
| `..._yawp10g` | 下降实际成功，脚本却报 `TIMEOUT` | 缺少 `GRASP_POSE_REACHED` 的**终态标记**，脚本匹配不到 | 补终态发布日志与脚本判定分支（`--grasp-descent` 下先于预抓取标记判定） |

失败运行目录全部保留。`..._yawp10g` 与 `..._yawp10h` 的下降本身都成功，区别只在脚本能否
识别终态。

## 九、局限（如实）

- **单场景**：只验证了 `x=0.50, y=0.0, yaw=+10°` 一个非零 yaw 场景。按本轮要求不做多 seed
  统计，**不声称该下降对任意采样位姿都成立**。
- **竖直性是离散测量**：由 21 个已验证航点的 FK 测得（0.0001 mm），另有 20 个段内插值状态
  参与碰撞检查，但都不是连续保证；执行过程中的实时偏差未逐拍记录。
- **碰撞是离散采样**：航点 + 0.05 rad 步长插值，不是连续碰撞的数学保证。
- **零接触不等于安全余量**：本轮杆未被扰动（物理测量 0.0000 mm）只说明**该位姿、该轨迹**
  下未发生接触，未给出间隙裕度。
- **起点一致性依赖前置阶段**：下降要求手臂关节与刚到位预抓取解相差 ≤ 0.02 rad；指关节
  因开爪而变化，被显式排除在该比对之外（这一点写在日志里，不是隐含假设）。
- **未知门禁的既有边界仍适用**：若夹爪侧存在未解决的未知目标，开爪会被拒绝，从而**不下降**
  并以失败终止，不自愈。

## 十、文件与参数

- 新增纯逻辑：`a1_sample_validator.{hpp,cpp}` 的 `buildDescentSchedule`、
  `StraightDescentDeviation` / `straightDescentDeviation`。
- 控制器：`runGraspDescent`、`buildAndVerifyGraspDescent`、`readMujocoRodTruth`；
  `A1VerifyContext` 增加下降相关字段（抓取目标、抓取解、下降起点/航点/Plan、实测偏差）。
- 状态机：`grasp_descent`、`descent_steps`、`descent_max_xy_dev_m`、
  `descent_max_angle_dev_deg`、`mujoco_*` 参数与三条门禁；新终态 `GRASP_POSE_REACHED`。
- launch：同名 8 个参数，默认保持原行为。脚本：`--grasp-descent`（隐含
  `--pregrasp-execute --prepare-mode` 的既有要求）。
- **未改动**：硬件、夹爪实现、仿真夹爪 server、XML/SRDF、控制器切换、weld、增益。

## 十一、运行数据位置

- 下降到位（成功）：`experiment_results/a1_runs/a1_p3a2b_descent_yawp10h/`
  （`task_launch.log`、`prepare_status.json`、`p2b_path.csv`、`sim_dual_moveit.log`）
- 默认不开下降回归：`experiment_results/a1_runs/a1_p3a2b_descent_off_regression/`
- 失败保留：`..._yawp10b` `..._yawp10c` `..._yawp10d` `..._yawp10e` `..._yawp10f` `..._yawp10g`

本轮未提交或推送 Git。

---

# 追加：P3a-2b 局部收尾（最终轨迹校验 / 真值参考核对 / 到位新鲜度）（2026-10-14）

本节只记录 R1–R3 三处局部修正。下降轨迹生成、IK、竖直性定义、夹爪打开时机、
`GRASP_POSE_REACHED` 边界与历史实验数据全部保留，未闭合抓杆、未搬运。

## R1 校验真正执行的那条最终轨迹

**原缺陷**：碰撞与限位检查跑在 TOTG **之前**的 21 个 IK 航点上；TOTG 之后只查了
名称/时间/导数，而"最终点 FK"仍取 `descent_waypoints.back()`（旧 IK 点）。也就是说，
**真正被下发的 26 点消息从未被完整检查过**。

**现实现**（`buildAndVerifyGraspDescent`）：

1. **先**做 TOTG，得到 `descent_plan.trajectory_`；此后所有检查以这条消息为对象。
2. 以 `mapLiveInto(getCurrentState())` 为基线重建判据模型状态——此时夹爪**已在
   R2 之前打开**，基线因此带有**本轮真实打开的指关节值**（日志打印最大指位移 0.0400 m）。
3. 逐点：位置维度、有限性、14 关节限位、以及严格 ACM 碰撞。
4. 段内插值（按 `p2b_max_joint_step_rad` 细分）同样查碰撞。
5. **末点 FK 取自最终消息的最后一点**（`finalStateOf(jt.points.back().positions)`）。
6. 以上全部通过后才置 `descent_plan_valid`；任何一项不过，`runGraspDescent` 都不会调用
   `execute`。

本次实测：`TOTG: 26 轨迹点（IK 航点 21）`、`最终轨迹碰撞检查: 51 个状态
（含 26 个航点、25 段细分），碰撞 0 处`、`最终轨迹末点 FK 残差: 左 0.0000 mm/0.0000°`。

### 负例（受控注入，默认关闭，**不下发任何运动**）

新增 `descent_negative_final`（`position` | `collision` | 空）。注入施加在**最终消息**上，
以便被检验的正是最终轨迹检查本身。

| 注入 | 结果 | 下发下降？ |
|---|---|---|
| `position`（末点第 1 关节 +100 rad） | `E_2B_FINAL_LIMIT at point 25: joint mj_left_joint1 = 100.253246` | **否**（日志中"下发下降轨迹" 0 次） |
| `collision`（在**末点自身**左末端 (0.474, 0.148, 0.130) 插 4 cm 障碍） | `E_2B_FINAL_PATH_COLLISION: 其他世界物体=67 例: a1_descent_final_obstacle_injected<->mj_left_hand` | **否**（0 次） |

碰撞注入刻意放在**末点自身的末端连杆**上，而不是"抓取位置附近"——后者在两只手并不
经过该点时不构成碰撞，什么也证明不了。第一次正是这样写的，实测未触发，已改正。

## R2 执行前核对杆参考仍适用

**原缺陷**：`before_phys` 只打印；读取失败也继续执行。

**现实现**：下降前必须取得**有效** MuJoCo 杆真值，并与本轮**冻结参考**比较；
容差沿用既有位姿一致性设置（`descent_truth_pos_tol_m=5 mm`、
`descent_truth_angle_tol_deg=5°`，与本轮预抓取阶段核对规划场景用的是同一组界限），
并在日志中写明实际差值与容差。读取失败 → `E_2B_TRUTH_UNAVAILABLE`；超出容差 →
`E_2B_TRUTH_REFERENCE_STALE`；两者都**不下发**。该检查已前移到构建下降轨迹**之前**，
参考不再适用时不做无谓的规划。

**执行后**：若测量失败，日志明确写"下降后的物理杆测量**失败**……本轮**未测得**杆是否
被扰动，不对此作任何声称"，不再用"前=ok/后=fail"的含糊措辞。

### 负例（新增 `descent_negative_truth`，默认关闭）

| 注入 | 结果 | 下发下降？ |
|---|---|---|
| `unavailable`（服务名指向不存在的服务） | `E_2B_TRUTH_UNAVAILABLE: the physical rod could not be measured before the descent` | **否** |
| `offset`（比较用冻结参考抬高 50 mm） | `E_2B_TRUTH_REFERENCE_STALE: ... position 50.000000 mm, angle 0.000000 deg` | **否** |

正例中的核对记录：`冻结参考仍适用性核对: 位置差 0.0000 mm（容差 5.0 mm），角度差 0.0000°`。

## R3 到位反馈必须新鲜，并纠正测量口径

**原缺陷**：到位窗口 `sleep` 后直接 `getCurrentState()`，无法证明反馈更新；
`settled_delta` 只比较相邻两次采样。

**现实现**：任务节点订阅 `joint_states_topic`（默认 `/joint_states`），回调只记录
**序号 + 到达时刻 + 关节值**（短锁、不做任何工作）。累积逻辑放在**无 ROS 依赖**的
`a1v::FreshWindow`，因此单测驱动的是同一份代码：

- **序号未推进的样本一律不计数**（`E_FRESH_STALE_SEQ`），缺关节或非有限值同样拒绝
  （`E_FRESH_INCOMPLETE`）；
- 判定用**整个窗口的每关节 max−min**，不是相邻差；
- 窗口**滑动**：只保留最近 `window_sec` 的样本。不滑动会把动作起始的大幅移动永远留在
  视野里，已经停稳的手臂也会被判失败。

`awaitFreshArrival` 在有界时间内（窗口 + 2 s）持续取样，直到**出现一个整窗安静的滑动窗口**
才判到位；超时即失败。**容差没有改动**（仍为 `pregrasp_stable_tol_rad = 0.002`）。
`executePregrasp` 与 `runGraspDescent` **共用同一个 helper**，没有第二套实现。

**口径纠正**：执行后不再写"实际达到的航点/实测偏差"。日志与本节一律写
**"规划航点竖直性"**，并注明"这是**规划航点的 FK 偏差**，不是执行轨迹的实测偏差"；
本轮不采集控制频率的执行轨迹，报告不声称做过这种测量。

## 验证与回归

**单测**：`test_a1_sample_validator` **59 例**（新增 5 例），覆盖
重复序号不可累计到位、新样本在整窗内被接受、整窗缓慢漂移被判不合格（每步仅为容差
1/5，相邻差判据会放过）、缺关节/非有限样本被拒且不计数、零样本不算到位。
全套 **197 例**通过（sampling 26、stability 16、service_faults 35、truth 22、
sample_validator 59、gripper_protocol 13、gripper_client_faults 11、gripper_shared_state 15）。

**同场景非零 yaw 下降回归**（`a1_p3a2b_final_verify_regression`，x=0.50 y=0.0 yaw=+10°）：

```
[P3A-1] 到位窗口（新鲜反馈）: 30 个样本 / 0.302 s（序号 6476），窗口内最大单关节变化 0.001987 rad（阈值 0.002000）
        左臂 位置误差 0.2866 mm，姿态误差 0.3773°  |  右臂 位置误差 0.6411 mm，姿态误差 0.3946°
[P3A-2B] 到位窗口（新鲜反馈）: 30 个样本 / 0.302 s（序号 9567），窗口内最大单关节变化 0.001991 rad（阈值 0.002000）
        左臂 位置误差 0.5685 mm，姿态误差 0.3828°  |  右臂 位置误差 0.6501 mm，姿态误差 0.3507°
[P3A2B_RESULT] state=GRASP_POSE_REACHED  ⇒ prepare exit code: 0
```

## 遗留问题（需要规划器裁定，未自行放宽）

1. **整窗容差正处于临界，余量不足 1%。** 两次到位窗口的实测整窗变化为
   **0.001987 / 0.001991 rad，阈值 0.002000 rad**——通过，但余量只有 0.5%–0.7%。
   同一轮更早的一次运行（`a1_p3a2b_neg_final_position` 的前一次尝试）在预抓取阶段
   实测整窗变化 **0.003553 rad** 而被判 `E_FRESH_NOT_SETTLED`，直接终止了那一轮。
   也就是说：**相邻差判据（旧）系统性看不见的这段残余蠕动是真实存在的**，量级约
   0.002–0.0036 rad（≈0.11°–0.20°），正好骑在 0.002 rad 这条线上。
   本轮**没有**放宽容差来制造通过；但这条阈值是按"相邻差"选定的，直接当作"整窗变化"
   使用并不等价。建议规划器明确一个按整窗变化定义、写明依据的阈值（例如以功能容差
   2° 为参照给出量级），而不是继续沿用 0.002 rad。**在该决定之前，下降回归存在间歇性
   失败的可能。**
2. 窗口是"找一个安静的 0.302 s 滑动窗口"，最长等待窗口长度 + 2 s。若手臂持续蠕动，
   会正确地判失败而不是等待更久。
3. **P2B 规划存在运行时波动**：同一位姿的一次运行中 PTP 返回 `-2`（规划失败），
   重跑即通过。这是既有的规划器非确定性，不是本轮引入，也未在本轮处理。
4. 一次负例运行（`--descent-negative-final collision` 的首次尝试）撞上**既有的启动期
   崩溃**（`sim_stack_deaths.txt`），与代码改动无关，重跑通过。
5. 其余局限沿用上一节：单场景、竖直性为离散测量、碰撞为离散采样、零接触不等于安全余量。

**未改动**：硬件、夹爪、仿真夹爪 server、XML/SRDF、控制器切换、weld、增益。
历史原始日志与 CSV 未改写。

## 本轮数据位置

- 回归（通过）：`experiment_results/a1_runs/a1_p3a2b_final_verify_regression/`
- 负例（全部拒绝、零次下发）：`a1_p3a2b_neg_final_position`、
  `a1_p3a2b_neg_final_collision`、`a1_p3a2b_neg_truth_unavailable`、
  `a1_p3a2b_neg_truth_offset`
- 上一轮的成果运行保留：`a1_p3a2b_descent_yawp10h`、`a1_p3a2b_descent_off_regression`

本轮未提交或推送 Git。
