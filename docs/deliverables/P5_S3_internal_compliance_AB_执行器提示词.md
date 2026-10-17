# P5/S3 内部柔顺 A/B（同一接触流程）——执行器提示词

日期：2026-10-17。状态：待执行。执行人：执行器。
本文是任务书，不是结论；完成后由规划器在此基础上修改与推进。

**前置条件**：先执行
[`P5_S3_baseline_执行器提示词.md`](P5_S3_baseline_执行器提示词.md) 并确认刚性流程
在当前工作树下跑通（`state=DONE`）。基线未通过时，不执行本文的代码修改与柔顺运行。

---

## 0. 背景

1. 无 weld 接触抓取的 40 cm / 2 s 完整流程已冻结（`2bc766a`、`316cefd`），
   刚性配置（内部柔顺关闭）下 DONE run 为 `185606`、`185847`；
   当前工作树下的 S3 刚性基线已复跑通过：`..._192912`（DONE，抬升 0.392 m、
   保持 2.050 s、max_slip 9.045 mm、落点误差 −3.34/−4.41 mm、偏航误差 −1.14°）。
2. S2 规划对照已完成（run `..._191610`，结果见
   [`P4_S2_plan_compare_result_20261017.md`](P4_S2_plan_compare_result_20261017.md)）：
   CHOMP 可作对照但默认运输仍用物体空间 IK + IPTP。
3. 现在做 S3 的第一步：**固定增益（内部柔顺关闭）vs 内部柔顺（开启）**，
   同一场景、同一轨迹、同一参数，只切换内部柔顺开关。

这是**对照实验**，不是“让柔顺通过”的调参任务。只跑一种默认配置，不扫描增益或阈值。

## 1. 代码事实（先核对，再改）

- `src/multipanda_ros2/franka_hardware/src/sim/franka_mj_hardware_system.cpp`
  `write()` 中的内部柔顺：读取环境变量 `MJ_INTERNAL_COMPLIANCE_ENABLED`（缺省为
  开启）、`_STRESS_THRESHOLD`（默认 2.0）、`_K_GAIN`（0.08）、`_D_GAIN`（0.15）、
  `_MIN_K`（0.10）、`_MAX_D`（2.5）；按 `|(fy_left + fy_right)/2|` 超过阈值时
  按比例降低 `kp`、提高 `kv`，只作用于 `ControlMode::JointPosition`。
  它是“基于内力的自适应关节伺服增益”，不是笛卡尔阻抗。
- `scripts/a1_run_scene.sh`：`contact_probe` 且 `CONTACT_PROBE_RIGID=true` 时
  `export MJ_INTERNAL_COMPLIANCE_ENABLED=false` 并设 `TASK_CONTROL_MODE=rigid`；
  否则不导出该变量，`TASK_CONTROL_MODE=compliant`。
- `scripts/run_contact_probe_local.sh`：当前**拒绝** `CONTACT_PROBE_RIGID!=true`，
  固定初始位姿 `(0.50, 0.0, +10°)` 和刚性配置。

## 2. 本轮允许的修改（仅这些）

### 2.1 运行入口增加显式开关

在 `scripts/run_contact_probe_local.sh` 增加参数 `--internal-compliance`：

- 解析后置 `COMPLIANT=1`；
- 仅当该参数存在时，把原来的拒绝逻辑改为：
  `export CONTACT_PROBE_RIGID=false` 且 `export MJ_INTERNAL_COMPLIANCE_ENABLED=true`；
- 未给该参数时行为完全不变（仍拒绝继承来的 `CONTACT_PROBE_RIGID=false`）；
- 默认 `RUN_ID` 前缀改为 `lidio_contact_probe_compliant_<时间>`，便于区分；
- 启动时打印一行明确的 profile 说明（rigid / compliant 二选一）。

### 2.2 增加内部柔顺的观测量（不改控制律）

在 `franka_mj_hardware_system.cpp` 的内部柔顺分支中增加**限频日志**（约 1 Hz，
`loop_cnt % 1000 == 0`）：打印当前 `internal_stress`、`current_K_scale`、
`current_D_scale`。目的：证明柔顺确实被触发、触发到什么程度；不改变取值范围、
不改变控制计算。

除以上两处外，不改任何代码。

## 3. 构建与运行

```bash
source /opt/ros/humble/setup.bash
export MUJOCO_DIR=/home/lidio/Libraries/mujoco
cd /home/lidio/franka_ws_1
colcon build --packages-select franka_hardware
bash -n scripts/run_contact_probe_local.sh

# 一次 headless 对照运行（唯一一次）
./scripts/run_contact_probe_local.sh --headless --internal-compliance
```

- 只跑一次；启动阶段失败（`E_SIM_INFO_UNAVAILABLE`、DDS 未就绪等）最多重试一次。
- 运行中若失败于物理/接触/规划，**不重试**，保留目录并如实记录失败状态。
- 必须在 `sim_dual_moveit.log` 中确认：
  `MuJoCo internal compliance: enabled | threshold=2.00 K_gain=0.080 ...`，
  以及新增的 K/D scale 日志确有变化（K scale < 1 至少出现一次）。
- 若 K scale 始终为 1.0，说明未触发，按“未触发”记录，不调阈值。

## 4. 指标（两组用同一口径）

刚性基线：`..._192912`（当前工作树、同构建、headless，DONE）；历史 `..._185606`、
`..._185847`（S2 代码加入前）可作参考。柔顺组：本轮 run。逐项记录：

1. `[CONTACT_PROBE_RESULT]` 的 `state` 与 `detail`；是否完成落回与释放；
   `[CONTACT_PROBE_RELEASED]` 的 `weld/attach/force_hold` 状态。
2. `[CONTACT_LOAD_VALIDATED]`：`max_slip`、`max_angle`、`duty`、`samples`。
3. `contact_probe.csv` 分阶段（`LIFT/HOLD/TRANSPORT/LOWER`）左右最大滑移与
   `rod_z` 范围。
4. 闭合窗口：左右各指接受样本数、gap spread（是否仍满足 20 样本/16 通过/≤1.0 mm）。
5. 接触法向力：`gripper_fingers_*.csv` 的 `normal_left_n/normal_right_n` 分阶段
   均值与最小值；`contact_probe.csv` 的 `left/right_normal1_n,2_n` 可交叉核对。
6. 最终杆真值位姿 vs 目标 `(0.40, 0.00, +20°)`：x/y 误差、偏航误差。
7. 内部柔顺触发程度：`min K scale`、`max D scale`、触发时刻所在阶段。
8. 关键阶段耗时（抬升、保持、运输、落回、开爪）。

提取用同一段脚本/命令，写进报告；若之后要批量对比，再落成
`scripts/summarize_contact_probe_run.py`，本轮不强制。

## 5. 判读与停止

| 观察 | 含义 | 动作 |
|---|---|---|
| 全程完成，滑移仍在 20 mm 窗口内，放置误差与刚性同量级 | 内部柔顺可用 | 记录；后续可做 3+3 重复 |
| 全程完成但滑移/误差明显变大 | 内部柔顺有代价 | 记录；默认仍用刚性 |
| 抓取或运输失败（掉杆、超时、duty 崩溃） | 该配置不可用 | 记录失败阶段与状态；默认仍用刚性 |
| K scale 始终 1.0 | 柔顺未触发 | 如实记录“未触发”，不调阈值 |
| 启动失败（环境） | 非物理结果 | 最多重试一次，仍失败则停止并说明 |

无论结果如何：**不改接触参数、夹持力、速度、滑移窗口、模型、ACM、规划器**；
不为了让柔顺成功而调 `THRESHOLD/K_GAIN/D_GAIN/MIN_K/MAX_D`。

## 6. 边界

- 不执行 CHOMP 轨迹；不改 S2 对照代码；不改 weld / 杆约束 / attach。
- 不提交、不推送（除非用户另行要求）；保留所有 run 目录与原始 CSV。
- 实验 headless；只有用户主动要看的验证轮才用 `--use-rviz`。

## 7. 交付

1. 修改文件与 `git diff --stat`；
2. 构建/语法检查结果；
3. run 目录（有效/无效分别列出）；
4. 第 4 节的对照表（柔顺组 vs 两条刚性基线）；
5. 内部柔顺触发证据（K/D scale 日志摘录）；
6. 结论：内部柔顺是否让流程仍完整、滑移与放置误差如何变化、是否保留为可用选项；
7. 任何偏离本任务书之处。
