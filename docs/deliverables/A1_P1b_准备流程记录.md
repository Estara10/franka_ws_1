# A1 P1b 记录：真值接入与场景同步（无运动）

对应阶段计划 `从A1开始的阶段计划与执行器提示词.md:182–238`。准则以 `PROJECT_GUIDE.md` 为准。

## 完成度

- ✅ **已完成**：稳定真值 → 独立 A1 消息 → 任务接收并冻结 → PlanningScene 同步并读回确认 → `PREPARED`。
  一次非零 yaw（+10°）真实新实例准备流程通过，三方位姿完全一致。
- ✅ **无运动边界成立**：A1 流程只经历 `INIT`，无 `APPROACH`/`GRASP`、无夹爪动作、无 weld、无 attach。
- ❌ **未做（本轮范围外）**：随机位姿抓取、完整搬运、yaw 抓取泛化、放置评价、优化器、阻抗、视觉。**未进入 P2**。

`PREPARED` **不是**任务 `DONE`：它只表示真值已冻结且规划场景已同步确认。

## 变更摘要

| 动作 | 文件 |
|---|---|
| 新增 | `dual_arm_a1_msgs/{package.xml,CMakeLists.txt,msg/RodPoseTruth.msg}`（纯消息包，工作区既有惯例） |
| 新增 | `dual_arm_carry_task/include/.../a1_truth.hpp`、`src/a1_truth.cpp`（纯逻辑，无 ROS） |
| 新增 | `dual_arm_carry_task/test/test_a1_truth.cpp`（21 例） |
| 修改 | `carry_task_context.hpp`（`TaskState::PREPARED`、来源参数、真值 latch） |
| 修改 | `carry_task_state_machine.cpp`（参数、冲突规则、真值订阅、PREPARED 出口） |
| 修改 | `dual_arm_controller.{hpp,cpp}`（**仅** `prepareFromA1Truth()`、`syncAluminumRodFromTruth()`、INIT 早退分支） |
| 修改 | `launch/dual_arm_carry_task.launch.py`（5 个新参数，默认保持原行为） |
| 修改 | `a1_scene_runner_node.cpp`（`--publish-truth/--truth-topic/--hold-sec`）、`scripts/a1_run_scene.sh`（`--prepare-mode`） |
| 修改 | `CMakeLists.txt`、`package.xml`（依赖与测试目标） |

关键设计：`run_id` 与位姿在**同一条消息**内（两个 latched 话题无法原子配对）；`header.stamp` 只作发布者本地参考，
接收方**不用它判新鲜度**（服务 stamp 恒为零），改为 `run_id` 匹配 + 接收方本地时刻；真值话题
`/a1_rod_pose_truth` 与 `/detected_rod_pose` 完全分离，`use_vision=false`。

## 构建与测试

**环境要求（非显然，必须记录）**：本机 anaconda 的 Python 3.13 在 PATH 中优先，会导致
rosidl 的 Python 生成器失败（`ImportError: cannot import name 'generate_py'`，生成器脚本用
`#!/usr/bin/env python3`）。工作区既有消息包也是用系统 Python 构建的。正确命令：

```bash
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
colcon build --packages-select dual_arm_a1_msgs dual_arm_carry_task \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select dual_arm_carry_task --ctest-args -R "test_a1_"
```

**结果：4 个套件 98 例全部通过，0 失败。**
`test_a1_sampling` 26、`test_a1_stability` 16、`test_a1_service_faults` 35、**`test_a1_truth` 21**。
新增测试覆盖：run_id 错配、期望 run_id 未设置、frame 为空、非有限数值、非单位/零范数姿态、
尺寸越界或退化、**冻结后再来消息不改变参考**（latch-once，含重复同值消息）、缺消息保持未冻结、
frame 映射（同名 / world↔base_link / 不支持）、场景读回比对（位置、姿态含 **q/−q 等价**、尺寸、非有限）。

## 真实准备流程

```bash
./scripts/a1_run_scene.sh --prepare-mode --x 0.48 --y 0.03 --yaw-deg 10 --run-id a1_p1b_prepare_yawp10c
```

数据路径：`experiment_results/a1_runs/a1_p1b_prepare_yawp10c/`
（`manifest.json`、`settle_samples.csv`、`runner.log`、`task_launch.log`、`prepare_status.json`、
`services_seen.txt`、`controllers.txt`、`sim_dual_moveit.log`）

**三方一致性（本轮核心结论）**

| 环节 | 位置 (x, y, z) | 姿态 | 尺寸 | 质量 |
|---|---|---|---|---|
| 稳定读回（runner 发布） | 0.480000, 0.030000, 0.020654 | yaw 10.0000° | 0.04/0.40/0.04 | 0.640000 kg |
| 任务冻结输入 | 0.480000, 0.030000, 0.020654 | yaw 10.0000° | 0.04/0.40/0.04 | 0.640000 kg |
| MoveIt 世界物体读回 | 0.480000, 0.030000, 0.020654 | yaw 10.0000° | 0.04/0.40/0.04 | — |

读回核对：**Δpos 0.0000 mm、Δangle 0.0000°、Δsize 0.000e+00 m**（容差 1 mm / 0.5° / 1e-6 m）。
`run_id` 匹配；`frame=world` 对规划坐标系 `base_link`，按源码依据的**显式单位变换**处理
（SRDF 固定 `virtual_joint` + URDF/MuJoCo 基座位置一致），**未声称 TF 存在**。

**无运动证据**：任务日志状态转换只有 `INIT` 一次；`APPROACH`/`GRASP`/`夹爪`/`weld`/`attach` 出现次数均为 0；
`/detected_rod_pose` 订阅未创建。

## 静态来源回归

默认入口（`rod_pose_source` 未设置 ⇒ `static`，与 `use_vision=false`）在新实例中跑到 INIT 即停止：

- 日志 `杆位姿来源: static`，日志无 A1 真值订阅；
- 该次运行**未进入** APPROACH，未发夹爪/weld/attach（计数均为 0）；
- 同时刻从 `move_group` 读回世界物体，与原有硬编码值一致：
  `id='aluminum_rod'`、`frame_id='base_link'`、`pose=(0.5, 0.0, 0.02)` 单位姿态、
  `dimensions=[0.04, 0.4, 0.04]`。

⇒ 默认 static 的**来源与 INIT 场景构造未变**。未重复跑 A0 完整搬运（按要求）。

## 失败与问题记录

1. **上一轮遗留缺陷（本轮发现并修复）**：`a1_scene_runner_node.cpp` 中 4 处
   `reject({"E_...", "说明", {}})` 被按 `ValidationResult{bool ok, reason_code, detail}` 初始化，
   导致 `ok=true` 且机读码变成说明文字。已改为显式三字段初始化；构建不再有 narrowing 警告。
   （上一轮我报告“无警告”是基于一次未重编译的增量构建，属**说法不准**，特此更正。）
2. **PlanningScene 读回约定（本轮关键 bug）**：`applyCollisionObject` 把放置写在**对象位姿**
   （`object.pose`）上，读回时 `primitive_poses[0]` 为**单位**。只读 primitive pose 会得到 (0,0,0)，
   首次准备流程因此以 `E_SCENE_POS_MISMATCH`（481 mm）失败 —— **同步失败确实停止了准备，没有误报成功**。
   已改为用 `tf2::Transform` **组合对象位姿与 primitive 位姿**，对两种约定都正确；
   静态回归的读回证据（`pose=(0.5,0,0.02)`、`primitive_poses` 为单位）独立佐证了该约定。
3. **启动期间歇崩溃（既有问题，本轮再次复现 1 次）**：`mujoco_node` 抛
   `spin_some() called while already spinning` 后 SIGABRT，`move_group` 随之 segfault，
   服务始终不出现。该次运行**保留**在
   `experiment_results/a1_runs/a1_p1b_static_regression_20261011_124917/`，未删除、未掩盖，
   随后另开新 run 成功。
4. **static 回归首次尝试进入了 APPROACH**：脚本原先在停任务前抓取场景，且 stdout 缓冲导致
   marker 检测滞后，该次运行发出了**夹爪预开**指令（`发送双臂夹爪同步指令 -> 0.040`），
   **未执行任何机械臂轨迹**（无 `双臂规划成功`/`双臂移动完成`）。记录：
   `.../a1_p1b_static_regression_20261011_124822/`。修正检测顺序后重跑，第二次停在 INIT。
5. **遗留进程（需要处理）**：当前存在一个**我自己的**孤儿仿真实例，未由我清理：
   PID `276836`（`ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=false headless:=false`）
   与其子进程 `276847`（`move_group`），环境为 `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1`
   —— 与 `a1_run_scene.sh` 的默认完全一致，且其 `mujoco_node` 已不存在。来源应是
   **上一轮某次被中断、未产出 manifest 的 P1a 运行**（脚本 EXIT trap 未能回收）。
   按项目规则我**未清理不属于本次调用的进程**。建议用**定向**命令处理：
   `kill -INT 276836 276847`（不要用 `pkill`）。它可能与后续运行的启动稳定性互相干扰。

## 遗留问题（分级）

| 级别 | 项 |
|---|---|
| 允许后续改进 | 关闭期崩溃（`mujoco_node`/`move_group`）与启动期 `spin_some` 崩溃：**既有**，旧一键脚本已用重试规避；本轮如实记录、不自动重试、不掩盖。 |
| 允许后续改进 | `controllers` 字段仍为 `null`（由脚本落盘 `controllers.txt`）；`compliance_env` 记录的是 runner 进程环境。 |
| 允许后续改进 | `--x/--y/--yaw-deg` 显式入口、`script_status.json`/`A1_RESULT`、`truth` 记录块均为前两轮新增约定，待规划器确认保留。 |
| 当前不需要解决 | 真值只用于准备与记录，`p2_acceptance`/`task_outcome`/`placement_error` 保持 `null`；有效性检查与目标泛化属 P2/P3。 |

## 说明

`PREPARED` 之后不进入运动，因此**尚未**验证真值输入下的任何轨迹规划或执行；
yaw 运动泛化、放置评价与样本有效性检查均**未实现**，不得据本轮结果声称 A1 已通过。
