# A1 P1a 验证记录：配置、采样与场景应用

对应 `docs/deliverables/A1_P0_审阅结论与P1a执行提示词.md` 的执行器提示词。
本轮完成两件事：**（1）按审阅 8 项修正 P0 文档；（2）实现受限 P1a。**

**完成度声明（务必先读）**

- ✅ **已完成：P1a 的场景应用链条** —— 候选 → `/set_body_state` 应用 → 稳定判据 → 读回 → 清单。
  三个已知测试位姿在**各自新实例**中全部达到 `SCENE_READY_UNVALIDATED`。
- ❌ **未完成（也非本轮范围）：P1 完整** —— P1 还包含 P1b（真值话题、任务输入冻结、
  PlanningScene 同步），本轮**未实现**。
- ✅ **本轮已完成（修复轮）**：审阅 R1–R5 的正确性缺陷已修复，并新增针对性测试
  （§12.1）；补做一次 **sampled 真仿真**（§12.2，与 dry-run 候选逐位一致）。
  R6（关闭期崩溃）按本轮范围记录为已知限制，不修 `mujoco_ros`/`move_group`。
- ❌ **未完成：A1** —— A1 还包含 P2 样本有效性、P3 全流程泛化等，本轮**未触及**。
- ❌ **未做**：抓取、任务接入、yaw 运动泛化、放置评价、CHOMP、外部阻抗、视觉。
- `SCENE_READY_UNVALIDATED` **只表示“位姿已应用并稳定读回”**，
  **不代表该样本有效、可达或可搬运**（那是 P2），也不代表任务能执行。

---

## 1. 第一步：P0 文档纠偏

对 `A1_P0_接口核查与实现方案.md` 按审阅 8 项逐一修正，并在文首新增
**“修订说明（v2）”** 表，逐条记录改了什么、为什么改。要点：

| # | 修正 |
|---|---|
| 1 | `ros_api.cpp:282,292` 用默认 `Header`，返回 `stamp` 恒为零 → **不得用于新鲜度判断**（本轮已实测确认，见 §5.5）。删除“stamp 晚于 INIT”方案；P1a 只记录请求/响应的 steady 与 ROS 时间区间。P1b 改为 `run_id` 握手，并写明**两个独立 latched 话题不构成原子配对**。 |
| 2 | 稳定判据重写：仿真时钟有效且**严格递增**、**最小仿真时长**、**整窗**（任意两样本间最大差，非相邻）、有效 twist 范数、steady-clock 超时；暂停/回跳/模型重载一律不得判成功。 |
| 3 | 新增**单次写入规则**：写超时/失败即停止本轮，**不重试、不换下一候选**；并说明 `SetBodyStateCB` 在 TF 失败时可能已清零 twist（响应失败 ≠ 无副作用）。 |
| 4 | §10 重写：P1a 只验收“候选 → 应用 → 稳定读回 → 清单”，状态为 `SCENE_READY_UNVALIDATED`，**不填 `accepted=true`**。 |
| 5 | §4.2 删除 `ros__parameters` 之外的 `rod_target`（无法作为标准 ROS 参数加载）与自相矛盾的 `sample_x_yaw_only`。 |
| 6 | A0 措辞修正：默认入口未改 ≠ 同一进程会自动回到 XML 初态；须用**新实例**且只用**当前真实存在**的参数。 |
| 7 | world/base_link 降级为**当前模型的配置假设**（不等于 TF 链存在）；抓取公式降级为**候选推导**；MoveIt 单 link attach 是常规表达，缺口是**右手相对物体约束未表达**；ACM 对照检查改为**收紧**（原“放宽”表述会允许更多碰撞，是错的）。 |
| 8 | V5 标注为**不属于 P1a 服务往返试验**；Ruckig 后复检**不以航点是否变化为条件**；“不存在”改为限定范围的“未见”。 |

---

## 2. 第二步：P1a 变更列表

以 `src/multipanda_ros2/dual_arm_carry_task` 为根。**未修改**任务/运动/夹爪/硬件源码、
现有 launch、任何 XML、SRDF 或旧一键脚本；未安装任何依赖。

| 动作 | 文件 | 行数 |
|---|---|---|
| 新增 | `config/a1_scene_sampling.yaml` | 101 |
| 新增 | `include/dual_arm_carry_task/a1_sampling.hpp` | 417 |
| 新增 | `src/a1_sampling.cpp` | 827 |
| 新增 | `src/a1_scene_runner_node.cpp` | 1082 |
| 修改 | `CMakeLists.txt`（+`mujoco_ros_msgs`、`a1_sampling` 库、`a1_scene_runner_node`、config 安装、3 个 gtest 目标） | — |
| 修改 | `package.xml`（+`<depend>mujoco_ros_msgs`、`<test_depend>ament_cmake_gtest`） | — |
| 新增 | `test/test_a1_sampling.cpp`（23 例） | 451 |
| 新增 | `test/test_a1_stability.cpp`（16 例） | 233 |
| 新增 | `test/test_a1_service_faults.cpp`（18 例） | 451 |
| 新增 | `scripts/a1_run_scene.sh` | 337 |
| 输出 | `experiment_results/a1_runs/<run_id>/` | — |

**设计要点**

- 纯逻辑（采样、几何、稳定判据、单候选编排）放在 `a1_sampling.{hpp,cpp}`，
  **不依赖 ROS**，可离线单测；ROS 侧（参数、服务客户端、轮询、清单落盘）在
  `a1_scene_runner_node.cpp`，通过 `BodyStateTransport` / `Clock` 抽象注入，便于故障注入。
- 节点名 `a1_scene_sampling` 与配置文件顶层键一致，`--params-file` 可直接加载。
- 截图式确定性：`splitmix64` + **显式整数→双精度映射** `u=(v>>11)*2^-53`，
  **不使用** `std::uniform_real_distribution`（其算法标准未规定）。
- 清单同时写**完整配置规范化字符串**与 **FNV-1a 64 校验和**，并标注其为
  “change-detection checksum, not cryptographic”。

### 2.1 一处超出 P0 原方案的补充（请重点审阅）

审阅要求“至少三个**已知**测试位姿（yaw=0、正角、负角）”，而均匀采样**无法产生 yaw 恰好为 0**。
为此增加了显式位姿入口：节点 `--x/--y/--yaw-deg`（脚本同名参数，三者必须同时给出），
清单中以 `pose_source` 区分：

- `"sampled"` —— 正常采样路径（P1a 主线）；
- `"explicit_override"` —— 已知位姿验证路径。

该入口**只用于验证**，不绕过配置校验（仍要求取值有限），也不改变采样主线。
若规划器认为不应存在此入口，可移除而不影响其他交付物。

---

## 3. 构建与测试：命令与结果

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select dual_arm_carry_task --event-handlers console_direct+
colcon test  --packages-select dual_arm_carry_task --ctest-args -R "test_a1_"
```

- 构建：`Finished <<< dual_arm_carry_task`，**无 error、无 warning**（`-Wall -Wextra -Wpedantic`）。
- 生成了新可执行文件 `a1_scene_runner_node` 与静态库 `liba1_sampling.a`，
  安装 config 至 `install/dual_arm_carry_task/share/dual_arm_carry_task/config/`。

**结果：3 个测试套件 57 例全部通过，0 失败。**

| 套件 | 用例数 | 结果 |
|---|---|---|
| `test_a1_sampling` | 23 | 100% PASS |
| `test_a1_stability` | 16 | 100% PASS |
| `test_a1_service_faults` | 18 | 100% PASS |

覆盖（对齐审阅的“验证要求 / 单元·离线”）：

- **确定性**：同 seed 同 index 重复生成一致；`firstN` 等于逐个 `candidate(i)`；不同 seed 结果不同；
  400 个候选全部落在配置范围内且**覆盖**范围（非聚集）；`z` 由支撑面+半高+间隙推导。
- **非法输入**：NaN / ±Inf / `x_min>=x_max` / yaw 越界 / 负 seed / `max_total_candidates<1` /
  半尺寸非正 / 负间隙 / 空算法版本 —— 逐一给出机读原因码。
- **四角几何**：yaw=0 长轴沿世界 Y（±0.40 m）；yaw=90° 长轴转到世界 X；任意 yaw 与解析解一致；
  包围盒包含四角；四角 z = 支撑面。
- **`q`/`-q` 姿态距离**：等价；已知角度（0.5、π、0.75 rad）正确。
- **稳定判据负例**：暂停（时钟冻结）重复读数、`sim_paused`、时钟回跳、仿真时钟无效、
  仿真窗口不足、仿真/墙钟推进比过低、**慢漂移**（相邻差均在容差内但整窗超限）、
  姿态漂移、twist 超限、twist 非有限、不在落地高度、未平躺、模型重载、样本不足。
- **服务故障注入（假 transport，非真实仿真）**：写不可用 / 被拒绝 / 超时
  → 均为 `SCENE_APPLY_FAILED`，且 **`write_calls == 1`**（不重试、不换候选）；
  前置条件失败（sim_info 不可用 / `model_valid=false` / 基线读取失败）→ **`write_calls == 0`**；
  非法候选索引 / 非法配置 / 非有限显式位姿 → `NOT_RUN` 且 0 次写；
  稳定超时、非落地位姿、暂停、模型重载 → 停止且仅 1 次写；
  **中断标志** → `INTERRUPTED` + `E_INTERRUPTED`（写入已发生的位姿仍保留在记录中）。

### 3.1 lint 状态（如实报告；含生成时间与范围说明）

**范围与时间**：
- 下文“包整体 lint 失败”的数字来自**本轮之前**的 `build/dual_arm_carry_task/test_results/` 缓存结果，
  该缓存是**旧配置**生成的，**不包含本轮新增的 `a1_*` 文件**，所以它既不是本轮的全面检查、
  其“新增文件是否干净”也不能由它证明。
- 本轮对**改动过的文件**单独跑了 `ament_cpplint`（时间：本轮修复完成后，命令见 §11），
  结论与旧缓存**分开**报告如下。


- **本轮改动文件**（`a1_sampling.{hpp,cpp}`、`a1_scene_runner_node.cpp`、三个 `test_a1_*.cpp`）
  经 `ament_cpplint` 检查：`include_order` / 行宽 / 注释间距 / `runtime/int` /
  `readability/casting` **均已修完**；仅剩 **`legal/copyright`（缺版权头）**
  与 `readability/fn_size`（`main` 约 600 行，严重度最低档）
  两项，均为**本包既有风格**（既有文件同样缺版权头）。
- 该版权项是**包内既有状况**：`copyright` 套件对本包全部 10 个既有文件同样报错，
  故我未单独给新文件加头，以与周边代码保持一致。
- 本包 lint 整体**本来就是失败状态**（`colcon test-result` 汇总 589 测试 / 521 失败，
  全部集中在 `cpplint`/`copyright`/`flake8`/`pep257`/`uncrustify`/`lint_cmake`/`xmllint`，
  且不涉及 `a1_*` 文件）。**这不是本轮引入的，也不在本轮修复范围内。**

---

## 4. 真实仿真：三个已知测试位姿

命令（每次一个**新实例**，`ROS_DOMAIN_ID=77`，`ROS_LOCALHOST_ONLY=1`）：

```bash
./scripts/a1_run_scene.sh --x 0.50 --y 0.0   --yaw-deg 0.0   --run-id a1_p1a_pose_yaw000
./scripts/a1_run_scene.sh --x 0.48 --y 0.03  --yaw-deg 10.0  --run-id a1_p1a_pose_yaw_p10
./scripts/a1_run_scene.sh --x 0.52 --y -0.03 --yaw-deg -10.0 --run-id a1_p1a_pose_yaw_n10
```

| 项 | yaw 0° | yaw +10° | yaw −10° |
|---|---|---|---|
| 设定位姿 (x, y, yaw) | 0.50, 0.00, 0.00° | 0.48, 0.03, +10.00° | 0.52, −0.03, −10.00° |
| 设定 z（推导值） | 0.022 | 0.022 | 0.022 |
| 写结果 | SUCCESS | SUCCESS | SUCCESS |
| **稳定后读回** (x, y, z) | 0.500000, 0.000000, 0.020654 | 0.480000, 0.030000, 0.020654 | 0.520000, −0.030000, 0.020654 |
| 读回 yaw | +0.000° | +10.000° | −10.000° |
| **设置↔读回 偏差** dx / dy | +0.0000 mm / +0.0000 mm | +0.0000 mm / +0.0000 mm | +0.0000 mm / +0.0000 mm |
| **稳定后物理偏差** dz | −1.346 mm | −1.346 mm | −1.346 mm |
| 杆质量（服务读回） | 0.6399999857 kg | 同左 | 同左 |
| 稳定窗口（仿真时长） | 0.980 s | 1.000 s | 0.982 s |
| 仿真/墙钟推进比 | 0.99998 | 0.99998 | 1.00202 |
| 轮询次数 | 51 | 54 | 54 |
| 终态 | `SCENE_READY_UNVALIDATED` | 同左 | 同左 |

**这三组数值的来源与限制（审阅修正）**：它们全部来自**稳定之后的读回**，
即 `settle` 收敛后 `readback_pose` 的值。因此“设置↔读回偏差”**不是独立的即时写入精度测量**
（那需要在写入后立即读回、不经过落地过程），而是“落定之后杆的实际位姿与设定值的差”。
把它当作写入精度的独立证据属于过度解释。

- **设置↔读回偏差**：`x`、`y`、`yaw` 的偏差为 **0.0000 mm / 0.000°**，
  说明 `/set_body_state` 的写入与 `/get_body_state` 的读回**在该精度上完全一致**，
  四元数换序正确。
- **稳定后物理偏差**：`dz = −1.346 mm`，三次运行**完全一致**。
  原因是应用高度刻意比落点高 `apply_clearance_m = 2 mm`（避免初始穿透），
  杆自由下落 2 mm 后由接触求解器稳定在 `z = 0.020654`（而非解析值 0.020000，
  差 0.654 mm 是软接触平衡位置）。
  **这属于物理落地行为，不是写入误差**，且可复现。
- **这是“稳定后的物理结果”，不是“放置精度”**：本轮 `z` 是从 2 mm 高处落下的结果，
  不能当作搬运放置误差证据（后者属 P3c）。

**新增的事实性确认（P0 待核实项）**

- **`model_path` = `install/franka_description/share/franka_description/mujoco/franka/dual_scene.xml`**
  → 首次以运行证据确认**实际加载的是安装树中的 `dual_scene.xml`**（V1 的一部分）。
- `load_count` 在整轮内**保持不变**（前后均为 1），未见模型重载。
- **服务返回的 `pose.header.stamp` 恒为零**（`service_pose_stamp_is_zero = true`），
  三处运行一致 → 实测证实 P0 修订第 1 项。
- `sim_info_after.paused = false` 全程；仿真/墙钟推进比 ≈ 1.0（`realtime` 默认 1.0）。

**稳定判据的实际行为（一次有价值的观察）**

`settle_reason_trace` 记录了原因变化，三次运行的形状相同：

```
a1_p1a_pose_yaw000 : 1 E_STABILITY_INSUFFICIENT_SAMPLES → 3 E_SIM_WINDOW_TOO_SHORT
                     → 26 E_TWIST_EXCEEDED → 51 E_OK
a1_p1a_pose_yaw_p10: 1 … → 3 … → 26 E_TWIST_EXCEEDED → 54 E_OK
a1_p1a_pose_yaw_n10: 1 … → 3 … → 27 E_TWIST_EXCEEDED → 54 E_OK
```

即：**判据正确地拒绝在窗口仍含落地瞬态时判稳定**（落点速度约 0.2 m/s，远超 0.01 容差）。
实现上把窗口**按仿真时间滑动**（保留约 1~2 倍 `min_sim_window_sec`），
而不是按固定样本数截断 —— 后者在 50 Hz 下会保留约 4 s 历史
（`max_window_samples=200`），使同一次运行要到 ~4 s 才收敛。
该修正后收敛时间 ~1.1 s。`max_window_samples` 降级为纯安全上限。

---

## 5. A0 回归（新实例、现有参数）

命令（`ROS_DOMAIN_ID=78`，独立于 A1 运行；**不传 `rod_pose_source`**，该参数本轮不存在）：

```bash
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=false
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant use_vision:=false \
  metrics_sync_target_mm:=5.0 metrics_ee_target_mm:=2.0 \
  metrics_descend_place_z:=0.13 metrics_descend_shift_x:=999.0
```

**柔顺环境变量（实际配置）**：两次运行均**未导出任何 `MJ_INTERNAL_COMPLIANCE_*`**
（`compliance_env.txt` 只有说明行）。因此模拟器使用其**编译期默认**：
`enabled = true`、`threshold = 2.0`、`k_gain = 0.08`、`d_gain = 0.15`、`min_k = 0.10`、`max_d = 2.5`
（`franka_mj_hardware_system.cpp:361–391`）。
**注意这与旧一键脚本不同**（脚本设 `MIN_K=0.45 / MAX_D=1.80`）。
本轮**不把由此产生的任何行为差异归因于 A1**。

| 尝试 | run_id | FSM 终态 | 备注 |
|---|---|---|---|
| 1 | `a0_regression_20261011_114907` | **ERROR** | 见 §6.1：夹爪 action 结果超时（**服务器侧实际已成功**） |
| 2 | `a0_regression_20261011_115034` | **DONE**（全流程 11 次状态转换） | 未传 `metrics_descend_*`，指标不可比（见 §6.4） |
| 3 | `a0_regression_20261011_115221` | **DONE** | 传入与一键脚本一致的指标参数，指标可比 |

**尝试 3 的指标报告**（`launch_logs/task_metrics_20261011_195231_summary.md`）：

| 指标 | 值 |
|---|---:|
| sync_error_mean_mm | 0.018 |
| sync_error_max_mm | 0.949 |
| sync_within_target | PASS |
| ee_terminal_error_mm | **2.687** |
| ee_within_target | **FAIL** |

与历史参照（`experiment_results/reports/task_metrics_20260821_194653_..._summary.md`，
`ee_terminal_error_mm = 2.671`、sync PASS）**同一量级**。
⇒ 结论的**严格范围**：3 次尝试（2 次 DONE、1 次 ERROR）**只能证明**
（a）现有 A0 入口仍然可以跑完整流程、（b）存在**间歇性**故障。
它**不能证明可靠性未退化**（样本量太小，且没有改动前的同环境基线对照），
也**不能把夹爪根因确定**为 client 每次新建（见 §6.1）。
末端 2 mm 目标**仍未达标**，这是既有的、已被文档记录的状态，**本轮不声称已修复**。

**数据落盘位置说明**：`experiment_results/a1_runs/<run_id>/` 是本轮 A1 交付物的输出目录。
A0 的指标报告由**既有**监控节点按自身默认写到 `launch_logs/`
（`task_metrics_monitor` 的 `result_dir` 为空时默认 launch_logs），
本轮**未改变**该行为，也未把 A0 报告搬移。

---

## 6. 失败记录

**记录完整性（审阅修正）**：并非“全部失败均保留”。本轮迭代中我**删除并重跑**过若干 run 目录，
因此下列记录**已不存在，无法验证**：

- **首次 `a1_p1a_pose_yaw_n10` 的启动期 `move_group` segfault**：当时该目录被 `rm -rf` 后重跑成功，
  原始日志**未保存**。现存的 `a1_p1a_pose_yaw_n10` 是**重跑成功**的那一次，不含该失败证据。
- 早期 `a1_p1a_pose_yaw000`、`a1_p1a_dryrun_01`、`a1_final_check` 的中间版本同样被删除后重建。

现存的失败记录如下（可核对）：

### 6.1 A0 尝试 1：夹爪 action 结果超时（历史缺陷）

日志证据（`a0_regression_20261011_114907/sim_dual_moveit.log`）：

```
1791719362.558  [mj_left_gripper_sim_node] Calling simGripperMove
1791719363.355  [mj_left_gripper_sim_node] simGripperMove DONE: at(1)=0.0438
1791719363.355  [mj_left_gripper_sim_node] GripperCommand succeeded
...
1791719370.558  [dual_arm_carry_task]  ✗ 等待双臂夹爪执行结果超时
1791719370.765  [dual_arm_carry_task]  ✗ 任务失败！  (ERROR)
```

**服务端在 0.8 s 内成功完成**（含右臂），但任务端 `async_get_result` 在 **8 s** 后超时，
`预打开夹爪失败` → FSM 进入 `ERROR`。

- **不能据此确定根因**。这次记录只证明“服务端已完成、客户端未在 8 s 内收到结果”这一现象，
  与 P0 的 **B5**（`gripper_controller.cpp:25–26` 每次调用新建 action client、`:64–68` 结果超时即放弃）
  属于**同一缺陷类**，但**不足以把根因确定为 client 每次新建**：也可能是 DDS/发现或
  executor 调度的偶发。要定因需要单独的可复现实验，本轮不做。
- **不是本轮引入**：本轮未改动 `franka_hardware`、`gripper_controller.cpp`、任何 launch 或任务源码；
  A1 runner 是**独立可执行文件**，A0 回归期间**未运行**。
- 同一参数的新实例（尝试 2、3）**均成功到 DONE**，说明是**间歇性**故障。
- **按审阅要求：记录并停止，不在本轮扩大修复范围。**

### 6.2 我的 A0 辅助脚本自身的缺陷（已修正）

第一次 A0 尝试我的辅助脚本用 `ros2 topic echo --once /task_status`（**未指定类型**）在任务节点
尚未启动时订阅，立即失败并把错误文本写进 `task_status.txt`；等待循环判断“文件非空”即继续，
于是在 APPROACH 中途就拆掉了仿真。第二次起改为：先等 `/task_status` 出现，
再用显式类型 `std_msgs/msg/String` 订阅，并以**任务节点自身日志**中的
`已发布任务完成状态(DONE)` / `已发布任务失败状态(ERROR)` 作为有界等待条件。
**该缺陷属于我的临时验证脚本，不在交付物中**（交付脚本 `scripts/a1_run_scene.sh` 无此问题）。

### 6.3 启动/关闭期的既有崩溃（两次成功运行的共同现象）

`sim_dual_moveit.launch.py` 在**关闭阶段**稳定出现：

```
[mujoco_node-3] terminate called after throwing an instance of 'std::runtime_error'
[mujoco_node-3]   what():  Node needs to be associated with an executor.
→ mujoco_node exit code -6 (SIGABRT)
→ joint_state_publisher exit code 1
→ move_group exit code -11 (Segmentation fault)
```

两次成功运行（115034、115221）**均出现**，与本轮改动无关（未触碰 mujoco_ros / move_group 配置）。
另外在 A1 的一次运行中，`move_group` 在**启动阶段**就 segfault 过
（`a1_p1a_pose_yaw_n10` 的首次尝试，脚本已记录到 `sim_stack_deaths.txt`），
重跑即成功 —— 也属既有的启动期不稳定。
现已由脚本**检出并记录**：若仿真日志出现 `process has died` / `Segmentation fault`，
写入 `sim_stack_deaths.txt` 并在 runner 成功时把退出码提升为 4，避免把坏栈当成干净运行。

### 6.4 指标口径陷阱：`ee_terminal_error_mm` 不可比的一次实例

A0 尝试 2 未传 `metrics_descend_place_z`，监控节点回退到 `enable_rotate ? 0.18 : 0.16`
（`task_metrics_monitor.cpp:65–69`），而控制器实际用的是实测 `place_release_hand_z_ref_`（≈0.13）。
两者相差约 50 mm，于是报告出 `ee_terminal_error_mm = 48.854`。
传入一键脚本一致的参数后即为 2.687 mm。
⇒ **这正是 P0 §2.4 指出的“两套目标来源”问题的一次实测复现**：
该指标**必须**与控制器使用同一目标来源，否则会把配置差异误报成精度退化。

---

## 7. 中断与超时行为（审阅第 5、7 项）

- **单次写入**：写超时/失败/不可用 → 停止本轮，`write_attempted=true`、原因码机读，
  **不重试、不换候选**。单测已断言 `write_calls == 1`。
- **中断留清单**：节点安装 `SIGINT`/`SIGTERM` 处理器（仅做原子置位），
  轮询循环检出后以 `INTERRUPTED` / `E_INTERRUPTED` 结束并**仍写清单**，退出码 130。
- **脚本不会挂死**：runner 由 `timeout --signal=TERM --kill-after=20` 包裹（默认 180 s）；
  `ros2 control list_controllers` 也限制 20 s（实测该命令在 controller_manager 已死时
  **确实会无限阻塞**，这是本轮遇到的真实挂死原因）。
- **不覆盖既有 run**：若 `manifest.json` 已存在，脚本与节点都直接拒绝运行。
- **进程归属**：仿真以 `setsid` 启动，脚本记录自身 PGID 并在退出时**只对自己的进程组**
  先 SIGINT、后 SIGTERM、最后 SIGKILL，并 `wait` 回收；**从不使用 `pkill`/`killall`**。
  当 PGID 不可用或等于脚本自身 PGID 时退化为只对 PID 发信号。

---

## 8. P0 待核实事项：本轮更新

| 编号 | 状态 | 证据 |
|---|---|---|
| V1 服务可用 | ✅ 已确认 | `/set_body_state`、`/get_body_state`、`/get_sim_info` 均在根命名空间可用；`model_valid=1`、`model_path` 指向安装树 `dual_scene.xml`、`load_count=1` |
| V2 写入/读回一致 | ✅ 已确认 | 三组位姿 dx=dy=0.0000 mm、yaw 完全一致；`z` 差 −1.346 mm 属落地物理行为（已区分） |
| V3 空 `admin_hash` | ✅ 已确认 | 三次运行均以空 hash 成功 |
| V4 `frame_id="world"` | ✅ 部分确认 | `"world"` 可用；`"base_link"` **未测（NOT_RUN）**，本轮固定用 `"world"` |
| V5 XML `euler` 语义 | ⛔ 不属于 P1a | 按审阅要求，不列入服务往返结论 |
| V6 抓取几何常量 | ⬜ 未做 | 属共享目标构造（P2–P3） |
| V7 自由关节 twist 参考系 | ⬜ 未做（不影响） | 主判据用整窗位姿变化，不依赖 twist 帧 |
| V8 稳定收敛时间 | ✅ 已测 | 窗口 0.98–1.00 s 仿真时长、51–54 次轮询、~1.1 s 墙钟；落地瞬态约 0.55 s 后被排除 |
| V9 `isPathValid` 重载 | ⬜ 未做 | 属 P2 |
| V10 Ruckig 是否改位置 | ⬜ 未做 | 按修订，复检要求不以其为条件 |
| V11 控制器实际频率 | ⬜ 未做 | 本轮未测量回调频率；`controllers.txt` 已随每轮保存 |
| V12 两控制器互斥 | ⬜ 未做 | `controllers.txt` 已保存供后续核对 |
| V13 一键脚本是否改源码树 | ⬜ 未运行验证 | 本轮未运行一键脚本 |
| V14 手指命名差异 | ⬜ 未做 | 仅记录，未影响本轮 |
| V15 末端 link 是 `link8` 还是 `hand_tcp` | ⬜ 未做 | 属共享目标构造（P2–P3） |
| **新** 杆质量 | ✅ 已确认 | 服务读回 **0.6399999857 kg**，与 `objects.xml` box 体积×density 推导的 0.64 kg 一致 |

---

## 9. manifest 样例与数据路径

数据路径：`experiment_results/a1_runs/<run_id>/`

```
manifest.json          主清单（可解析 JSON，已用 python3 -m json.tool 校验）
settle_samples.csv     全部轮询原始样本（poll, ros_time, steady, pos, quat, twist 范数, 有效标志）
runner.log             runner 节点 stdout/stderr
sim_dual_moveit.log    本轮专属的仿真 launch 日志
services_seen.txt      观测到的服务清单
controllers.txt        ros2 control list_controllers 输出（或 NOT_RUN 原因）
sim_stack_deaths.txt   仅当检出进程死亡时存在
```

`a1_p1a_pose_yaw000/manifest.json` 关键字段（节选）：

```json
{
  "run_id": "a1_p1a_pose_yaw000",
  "scene_state": "SCENE_READY_UNVALIDATED",
  "pose_source": "explicit_override",
  "requested_seed": 20261011,
  "algorithm_version": "a1-uniform-splitmix64-v1",
  "sampling_range_version": "a1-dbg-v1",
  "range_is_reachability_verified": false,
  "rod_center_z_derivation": {"support_surface_z": 0, "rod_half_height": 0.02,
                              "apply_clearance_m": 0.002, "rod_center_z": 0.022},
  "config": {"canonical_string": "...", "fnv1a64": "0x...",
             "note": "change-detection checksum, not cryptographic"},
  "git_commit": "9abc210f...", "git_dirty": true,
  "ros_session": {"ros_domain_id": "77", "ros_localhost_only": "1", "use_sim_time": true},
  "model_checksums": {"dual_scene.xml": "0xe6b2...", "mj_dual.xml": "0x674c...",
                      "objects.xml": "0x6af2...", "dual_panda_generated.srdf": "0x7c14..."},
  "compliance_env": {"MJ_INTERNAL_COMPLIANCE_ENABLED": null, "...": null},
  "candidate": {"index": 0, "pose_candidate": {"x": 0.5, "y": 0, "z": 0.022, "yaw_rad": 0}},
  "write": {"attempted": true, "outcome": "SUCCESS", "response_success": true,
            "request_ros_time": 2.430, "response_ros_time": 2.438,
            "request_steady": 0.780, "response_steady": 0.790},
  "readback": {"count": 50, "poll_count": 51, "rod_mass_kg": 0.6399999857,
               "baseline_pose": {"x": 0.5, "y": 0.0, "z": 0.0206537},
               "service_pose_stamp_is_zero": true, "history_csv": "settle_samples.csv"},
  "settle": {"sim_window_sec": 0.98, "clock_advance_ratio": 0.999979,
             "max_window_pos_delta": 3.613084e-04, "max_window_angle_delta_deg": 0.0,
             "max_twist_lin": 9.849966e-03, "converged": true, "reason_code": "E_OK"},
  "settle_reason_trace": [{"poll": 1, "reason_code": "E_STABILITY_INSUFFICIENT_SAMPLES"},
                          {"poll": 3, "reason_code": "E_SIM_WINDOW_TOO_SHORT"},
                          {"poll": 26, "reason_code": "E_TWIST_EXCEEDED"},
                          {"poll": 51, "reason_code": "E_OK"}],
  "readback_pose": {"pos": {"x": 0.5, "y": 0.0, "z": 0.020654},
                    "quat": {"x": 0, "y": 0, "z": 0, "w": 1}},
  "p2_acceptance": null, "task_outcome": null, "placement_error": null,
  "notes": []
}
```

**未执行项一律为 `null`，未伪造任何观测值。**
`compliance_env` 为 **runner 自身进程**的环境；脚本启动的仿真进程继承同一环境，
故对本轮取值有代表性，但**未直接查询仿真进程**（限制见 §10）。

---

## 10. 遗留问题与限制

1. **`compliance_env` 记录的是 runner 进程的环境**，不是仿真进程的。
   两者由同一脚本启动、环境相同，因此有代表性；但严格来说应改为查询
   `mujoco_node` 的进程环境或新增参数回读。
2. **`controllers` 字段为 `null`**：节点不查询控制器清单，改由脚本保存
   `controllers.txt`。清单中已用 `notes` 说明，但字段本身未关联到文件路径。
3. **只验证了平躺 + 纯 yaw 位姿**，且只跑了 3 个位姿；未做随机采样的真实应用
   （`pose_source="sampled"` 目前**只有 dry-run 证据**，没有真实仿真运行证据）。
4. **`--x/--y/--yaw-deg` 显式入口是本轮新增**（§2.1），需规划器确认是否保留。
5. **A0 的既有缺陷未修**：夹爪 action 结果间歇性超时（§6.1）、
   关闭期 mujoco_node/move_group 崩溃（§6.3）、末端 2 mm 未达标。
   按审阅要求只记录、不扩大修复范围。
6. **`--` 未验证 `frame_id="base_link"`**（V4），后续若要在 PlanningScene 与真值之间
   直接比对，需要先确认这条路径。
7. **`sample_x_yaw_only` 等被删字段**：若后续阶段需要“只采样 x/yaw”，需以新的、
   不与 y 采样冲突的字段重新设计。
8. 本轮**未运行** `colcon test` 的全量 lint 修复；包的 lint 失败是既有状况（§3.1）。
9. **仿真启动期存在间歇性崩溃**（本轮实测复现）：`mujoco_node` 以
   `spin_some() called while already spinning` 抛异常后 SIGABRT，随后 `move_group` segfault，
   服务始终不出现。**这是既有问题**：旧一键脚本 `start_dual_arm_task.sh:669` 已经在 grep 同一条
   错误信息并最多重试启动 2 次（`MAX_SIM_RETRIES`）。本轮脚本**不隐藏重试**：
   检测到即记 `sim_stack_deaths.txt` / `script_status.json` 并以非零退出，由人决定是否重跑
   （本轮 sampled 运行中 4 次尝试成功 2 次，失败 2 次均为该问题）。
10. **`--x/--y/--yaw-deg` 入口**与本轮新增的 `script_status.json` / `A1_RESULT` 约定
   属本轮新增，需规划器确认保留。

---

## 11. 复现命令汇总

```bash
# 构建
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select dual_arm_carry_task --event-handlers console_direct+

# 离线测试
colcon test --packages-select dual_arm_carry_task --ctest-args -R "test_a1_"

# 干跑（不启动仿真、不调用服务）
./scripts/a1_run_scene.sh --dry-run --seed 20261011 --candidate-index 0

# 单候选真实应用（每轮一个新实例）
./scripts/a1_run_scene.sh --x 0.50 --y 0.00 --yaw-deg 0.0 --run-id a1_p1a_pose_yaw000
./scripts/a1_run_scene.sh --seed 20261011 --candidate-index 0 --run-id a1_p1a_sampled_00

# A0 回归（新实例；柔顺环境变量与一键脚本不同，须按 §5 记录）
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=false
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  control_mode:=compliant use_vision:=false \
  metrics_sync_target_mm:=5.0 metrics_ee_target_mm:=2.0 \
  metrics_descend_place_z:=0.13 metrics_descend_shift_x:=999.0
```

**停止方式**：`scripts/a1_run_scene.sh` 自行回收其仿真进程组；
手动 A0 用各终端 `Ctrl+C`，若需强杀先 `ros2 node list` 定位再对**具体 PID** 操作。
本轮全程**未使用** `pkill`/`killall`。

---

## 12. 修复记录（本轮，对应审阅 R1–R6）

修复范围仅限 `a1_sampling.{hpp,cpp}`、`a1_scene_runner_node.cpp`、
`config/a1_scene_sampling.yaml`、`test/test_a1_*`、`scripts/a1_run_scene.sh`、
本包 `CMakeLists.txt` / `package.xml`。**未**修改任务/运动/夹爪/硬件、现有 launch/XML/SRDF、
旧一键脚本或 `mujoco_ros`。

| 项 | 状态 | 修复要点 |
|---|---|---|
| **R1 补正** 最终健康查询返回后仍可能报成功 | ✅ 已修 | 健康确认查询本身也是阻塞调用：其**返回之后、成功判定之前**再次检查取消与 deadline，**取消/超时优先于成功**；已写入的记录保留，**不重试**。补两个针对性测试（见 §12.3），并用**反向对照**确认二者在移除该检查后确实失败。 |
| **R1** 取消/超时仍可能写入或报成功 | ✅ 已修 | 写前先查取消与预算（**预取消 0 次写**）；写后、每次阻塞读取后、以及**判定 STABLE 之前**都复查取消与 deadline，**取消/超时优先于成功**；服务调用预算 `min(service_call_timeout, 剩余预算)`，单次调用不得长于总预算（配置校验强制）。每次读取同时记录**请求/响应**的 steady 与 ROS 时刻，样本时间取**响应**时刻。 |
| **R2** 健康查询失败仍报成功 | ✅ 已修 | 前置检查新增 `paused` 与 `model_path`；轮询中健康查询失败／`model_valid=false`／模型路径变化**立即判失败**（不再沿用旧状态）；成功前**必须**有一次有效最终健康确认，且与写入前的契约一致（`model_valid`、`paused`、`model_path`、`load_count`），否则 `E_HEALTH_UNCONFIRMED` 等。 |
| **R3** 实例归属与启动检查 | ✅ 已修（简单方式） | 启动前用带超时的 `ros2 service list` 检查同 domain 是否已有相关服务，**有则拒绝启动**且不清理他人进程；启动期若 launch 报组件死亡则**在写入前停止**；所有 discovery CLI 均加墙钟超时（挂死的 `ros2 control list_controllers` 已实测会无限阻塞）。未引入复杂并发锁（按本轮范围，单用户串行）。 |
| **R4** 覆盖记录/落盘失败仍报成功 | ✅ 已修 | 运行目录在**任何副作用之前**独占保留（脚本 `mkdir` 失败即拒绝；节点默认也独占，仅接受脚本标记 `.reserved_by_a1_run_scene` 且无既有 manifest 的目录）；`run_id` 校验字符集、长度并拒绝 `.`/`..`/分隔符；manifest 与 CSV 落盘失败**返回非零**（退出码 5）并与“场景动作成功”分开报告；CSV 先写 `.part` 再改名；失败路径写 `script_status.json`；节点输出单行机读 `A1_RESULT`。 |
| **R5** 参数与姿态校验 | ✅ 已修 | 校验实际使用的全部数值（有限、正、上下界、整数）；**`max_write_attempts_per_run` 现在会被读取并要求为 1**；`sim_info_check_every >= 1`（禁止静默关闭健康检查）；**拒绝非零 roll/pitch**（P1a 只支持平躺 + 纯 yaw）；候选姿态要求**单位四元数**（不再只查分量有限）；CLI 严格解析（拒绝尾随字符、重复标志、越界）。按本轮要求**取消完整配置 hash 要求**：仍保留采样子配置校验和，并新增 `effective_parameters` 记录实际取值。 |
| **R6** 关闭崩溃与日志完整性 | ⏳ 后续改进（如实记录） | cleanup 完成后新增**最终审计**，把关闭期崩溃单独写入 `sim_stack_deaths_after_cleanup.txt` 并注明“已知限制，非本轮运行失败”，**不声称生命周期正常**。未修 `mujoco_ros`/`move_group`。 |

### 12.1 新增的针对性测试（对应审阅“已复现缺陷”）

`test_a1_service_faults` 新增：预取消 0 次写、写后取消保留记录且不成功、
**读取超出预算不得判成功**（复现 `late read` 分支顺序缺陷）、轮询期健康查询失败、
轮询期 `model_valid=false`、轮询期模型路径变化、**最终健康确认失败**、
健康确认成功、策略拒绝（关闭健康检查 / 写入次数≠1 / 单次调用超预算 / 非有限或非正 /
窗口样本下限）、非法策略 0 次写、**非单位四元数候选拒绝**、`run_id` 校验矩阵。
`test_a1_sampling` 新增：**拒绝非零 roll/pitch**、单位四元数判定、显式位姿产出的四元数为单位四元数。

### 12.2 本轮新增的 sampled 真仿真（审阅要求）

```bash
./scripts/a1_run_scene.sh --seed 20261011 --candidate-index 0 --run-id a1_p1a_sampled_c00e
```

| 项 | 值 |
|---|---|
| `pose_source` | `sampled` |
| 采样配置校验和 | `0xdc7418648d47e609`（与 dry-run 一致） |
| dry-run 候选 0 | x=0.539375727173, y=0.023514664296, yaw=0.053097459785 rad |
| 实际应用候选 | x=0.539375727173, y=0.023514664296, yaw=0.053097459785 rad |
| **两者是否一致** | **是（逐位一致）** |
| 稳定后读回 | (0.539376, 0.023515, 0.020654)，yaw +3.0423°（设定 +3.0423°） |
| 设置↔读回 | dx = +0.0000 mm, dy = +0.0000 mm |
| 稳定后 dz | −1.346 mm（与前三次 explicit 位姿一致） |
| 杆质量 | 0.6399999857 kg |
| 稳定窗口 / 轮询 | 1.000 s 仿真 / 55 次 |
| `health_confirmed` | true（前后 `load_count` 均为 1、`model_valid` true、`paused` false） |
| 终态 | `SCENE_READY_UNVALIDATED` |
| 数据路径 | `experiment_results/a1_runs/a1_p1a_sampled_c00e/` |

成功**只表示**“位姿已应用并稳定读回”，**不代表可抓取，也不代表整个 A1 已通过**
（`p2_acceptance` / `task_outcome` / `placement_error` 均为 `null`）。

### 12.3 最终健康查询的取消/预算补正（本轮追加）

在 `applyCandidateAndSettle` 的 STABLE 分支中，最终健康查询返回后、判定成功之前，
新增两项检查（`a1_sampling.cpp`）：

1. 若已请求取消 → `INTERRUPTED` / `E_INTERRUPTED`；
2. 若响应时刻已超过 settling budget → `SCENE_SETTLE_FAILED` / `E_SCENE_SETTLE_TIMEOUT`。

两条路径都**保留已应用的位姿与已记录的样本**（`write_calls == 1`），**不重试、不换候选**。

| 新增测试 | 断言 |
|---|---|
| `CancelDuringFinalHealthQueryIsNotSuccess` | 最终查询期间取消 → `INTERRUPTED`，`health_confirmed == false`，`write_calls == 1` |
| `FinalHealthQueryReturningPastTheBudgetIsNotSuccess` | 最终查询返回时已超预算 → `E_SCENE_SETTLE_TIMEOUT`，`health_confirmed == false`，`write_calls == 1` |

**反向对照**：临时移除这两项检查后，两个测试**都失败**；恢复后通过 ⇒ 测试确实覆盖该分支，
不是空跑。服务故障套件 35 例、A1 全套 77 例全部通过（§12.4）。

### 12.4 本轮测试结果

| 套件 | 用例数 | 结果 |
|---|---|---|
| `test_a1_sampling` | 26 | 100% PASS |
| `test_a1_stability` | 16 | 100% PASS |
| `test_a1_service_faults` | 35 | 100% PASS |
| **合计** | **77** | **0 失败** |

本轮**未重复** sampled 真仿真、A0 回归或完整系统验收（按本轮要求）。

---

## 13. 结论与下一步

**本轮交付可判定通过的部分**：P0 文档已按 8 项修正；P1a 的“候选 → 仿真应用 → 稳定读回 → 清单”
链条在 3 个已知位姿上可复现地成功，写入精度与落地物理偏差已分别量化，
判据正确拒绝暂停/漂移/瞬态，服务故障与中断均留清单，A0 入口未破坏。

**建议的下一步（交规划器决定）**

- **P1b**：真值话题与任务输入冻结 + PlanningScene 同步（按 §4.7 的 `run_id` 握手设计）。
  注意 P1b **不做** yaw 运动泛化（属 P2–P3 的共享目标构造）与放置评价（属 P3c）。
- 若规划器要求，先补 **`pose_source="sampled"` 的真实仿真运行**证据，
  以及 V4 的 `"base_link"` 路径核实。
- 夹爪 action 结果超时（§6.1）建议**单列可靠性任务**，在 P2 的有限抓取试运行之前处理。

**本轮修复后的状态**：审阅 R1–R5 已修复并有针对性测试覆盖（§12）；R6（关闭崩溃）
按本轮范围作为已知限制记录并单独归档，不阻塞 P1b。sampled 真仿真通过并与 dry-run 候选逐位一致。

**未决问题**：`--x/--y/--yaw-deg` 与 `script_status.json`/`A1_RESULT` 约定是否保留；
§10 的 1、2、6、9 项是否需要在 P1b 前处理（9 为既有启动期崩溃，建议单列可靠性任务）。
