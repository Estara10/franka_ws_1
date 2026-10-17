# P5/S3 Baseline（刚性剖面）复跑——执行器提示词

日期：2026-10-17。状态：待执行。执行人：执行器。
本文是任务书，不是结论。**本轮不改任何代码**，只确认当前工作树下完整流程仍能跑通，
并记录一组与柔顺组同构建、同场景的刚性基线。

---

## 0. 目的与顺序

S3 要做“固定增益 vs 内部柔顺”的对照。现有刚性 DONE run `185606`、`185847` 早于
S2 对照代码的加入，因此先用当前工作树补一组基线：

1. **本轮（Baseline）**：刚性剖面，跑一次 40 cm / 2 s 完整流程，确认 DONE；
2. **下一轮（Compliant）**：才做 `P5_S3_internal_compliance_AB_执行器提示词.md` 里的
   两处最小修改（wrapper 开关 + K/D 观测日志），再跑柔顺组。

先跑通，再修改；本轮不修流程、不调参数。

## 1. 前置核对

- `git status` 记录工作树：S2 对照代码未提交，默认关闭（`contact_plan_compare=false`）。
- 本轮**不允许**打开 `contact_plan_compare`；日志中不应出现 `[PLAN_COMPARE]`。
- 确认没有残留的仿真实例；不要用宽泛 `pkill`，只处理本会话拥有且确认的进程。
- 构建当前工作树（S2 代码已构建过，若增量无变化会很快）：

```bash
source /opt/ros/humble/setup.bash
export MUJOCO_DIR=/home/lidio/Libraries/mujoco
cd /home/lidio/franka_ws_1
colcon build --packages-select dual_arm_carry_task
```

## 2. 运行

```bash
./scripts/run_contact_probe_local.sh --headless
```

- 唯一一次正式运行；启动阶段失败（`E_SIM_INFO_UNAVAILABLE`、DDS 未就绪）最多重试一次。
- 物理/接触/规划失败**不重试**，保留目录并如实记录失败状态。
- 必须确认：
  - 启动日志包含 `[contact_probe] rigid validation profile: internal MuJoCo compliance disabled`；
  - `sim_dual_moveit.log` 包含 `MuJoCo internal compliance: disabled`；
  - 只有这一个仿真实例，未启动 RViz（headless）；
  - 没有 `[PLAN_COMPARE]` 输出。

## 3. 基线验收门槛

同时满足才算跑通：

1. `task_state=DONE`（不是 `PLAN_COMPARE_DONE`，也不是失败码），
   `detail` 含完整流程：40 cm 抬升、2 s 保持、带偏航运输、落回、释放。
2. `[CONTACT_LOAD_VALIDATED]`：`lift ≥ 0.39 m`、`sim_hold ≥ 2.0 s`、
   `max_slip ≤ 20 mm`（用户已授权的窗口），`max_angle` 如实记录。
3. `[CONTACT_PROBE_RELEASED]`：`open=true`、`weld=OFF`、`attach=none`。
4. 最终杆真值相对目标 `(0.40, 0.00, +20°)` 的位置误差与偏航误差如实记录。
5. 四指 duty 如实记录；某指 `<0.80` 只记录警告，不中止（现行 A 方案）。

**未通过时**：不调参、不改代码、不重跑，保留失败目录，报告失败状态、阶段和关键数值，
由规划器决定修法。

## 4. 指标表（本轮填刚性组）

| 指标 | 数值 |
|---|---|
| 最终状态 / detail | |
| 抬升高度、保持时长 | |
| `max_slip` / `max_angle` | |
| duty（四指） | |
| 分阶段最大滑移（LIFT/HOLD/TRANSPORT/LOWER，左/右） | |
| 闭合窗口（样本数、通过数、gap spread） | |
| 接触法向力（分阶段均值/最小值） | |
| 最终杆位姿、x/y 误差、偏航误差 | |
| 抬升/保持/运输/落回/开爪耗时 | |
| weld / attach / force_hold 状态 | |

`contact_probe.csv`、`gripper_fingers_mj_left/right.csv`、`contact_probe_contacts.csv`
均为原始证据，保留不动。

## 5. 边界

- 不改代码、不改接触参数、不打开柔顺、不跑 CHOMP、不提交、不推送。
- 实验 headless；只有用户主动要求观看的验证轮才用 `--use-rviz`。
- 保留所有 run 目录与原始 CSV。

## 6. 交付

1. `git status` 与 `git diff --stat`（本轮应无新增代码改动）；
2. 构建结果；
3. run 目录（有效/无效分别列出）；
4. 关键日志行原文（rigid profile、internal compliance disabled、LOAD_VALIDATED、RELEASED、RESULT）；
5. 第 4 节指标表；
6. 明确结论：流程是否跑通、是否可作为 S3 刚性基线；
7. 任何偏离本任务书之处。
