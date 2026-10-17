# P4/S2b 运输计时（2× 拉伸）诊断与执行——执行器提示词

日期：2026-10-17。状态：待执行。执行人：执行器。
本文是任务书，不是结论；完成后由规划器判断是否保留改动。

---

## 0. 背景：一个必须澄清的计时事实

`contact_load_probe.cpp` 的 `makePlan` 在时间参数化之后有一段“拉伸”：

```cpp
double stretch = std::max(min_total, min_total / points.back().time);
for (...) stretch = std::max(stretch, 0.02 / segment_dt);
time *= stretch;
```

`min_total` 实际上不是“最短总时长”，而是**拉伸系数下限**：

- 抬升 `min_total=1.0` → 不额外拉伸；
- 运输 `min_total=2.0` → `stretch ≥ 2.0`，即把 IPTP 的时长**整体乘 2**；
- 落回 `min_total=0.5` → 由每段 0.02 s 的下限主导。

已观察到：运输计划时长 2.232 s，正好是 IPTP 原始时长约 **1.116 s × 2**。
而 S2 对照里 CHOMP 的运输计划是 **1.083 s**——两者几乎相同。

**结论：S2 里“CHOMP 比基线快一倍”不是规划器差异，而是基线的 2× 拉伸。**
本任务书先加诊断确认这一点，再用同一接触流程执行一次 1× 拉伸的运输，
看接触是否守得住。这是“拆开几何路径与时间参数化”的直接实验。

## 1. 允许的修改（仅这些）

### 1.1 诊断日志（不改变行为）

在 `makePlan` 的 `scaleTimedTrajectory` 之后、拉伸计算处，打印一行
`[CONTACT_PLAN_TIMING]`：

- `t_scaled`（拉伸前时长）、`scale`（`report.scale`）；
- `stretch_min_total = min_total` 与 `min_total/t_scaled` 两项；
- `stretch_segment = max(0.02/segment_dt)`；
- 最终 `stretch`、最终时长、以及是哪个来源决定（`min_total` / `segment` / `none`）。

需要知道阶段名：给 `makePlan` 增加一个 `const char* label` 参数（`lift` / `transport` /
`lower`，creep 若保留），并更新全部调用点。

### 1.2 运输拉伸可配置（默认不变）

在 `contact_load_probe.cpp` 读取环境变量 `CONTACT_TRANSPORT_STRETCH`（默认 `2.0`），
只用于运输那一次 `makePlan` 的 `min_total` 实参；把实际取值打印在
`[CONTACT_PROBE_PLAN]` 行或新的 timing 行里。默认值不变，普通运行行为完全不变。

不改：接触参数、夹持力、速度缩放、滑移窗口、模型、ACM、碰撞检查、验收判据。

## 2. 运行

```bash
source /opt/ros/humble/setup.bash
export MUJOCO_DIR=/home/lidio/Libraries/mujoco
cd /home/lidio/franka_ws_1
colcon build --packages-select dual_arm_carry_task

# 唯一一次执行运行：运输 1× 拉伸
CONTACT_TRANSPORT_STRETCH=1.0 ./scripts/run_contact_probe_local.sh --headless
```

- 一次运行；启动失败最多重试一次；物理/接触失败不重试。
- 必须确认日志中：`CONTACT_TRANSPORT_STRETCH=1.0` 生效、对应
  `[CONTACT_PLAN_TIMING] label=transport` 的最终时长约 1.0–1.3 s。
- 若运输计划时长没有明显下降（仍约 2.2 s），说明不是该项主导，停止并报告，
  不要继续改其他参数。

## 3. 指标与对照

刚性基线 `..._192912`（运输计划 2.232 s / 执行 2.44 s、运输滑移 L/R 5.89/4.41 mm、
max_slip 9.045 mm、落点 −3.34/−4.41 mm、−1.14°）。

本轮记录：

1. `[CONTACT_PLAN_TIMING]` 全部字段（lift/transport/lower）；
2. 运输计划时长、运输执行时长；
3. 运输段滑移 L/R、全流程 `max_slip`；
4. `[CONTACT_LOAD_VALIDATED]`、`[CONTACT_PROBE_RELEASED]`、最终落点与偏航误差；
5. `state`（DONE / 失败码）。

## 4. 判读

| 观察 | 含义 | 动作 |
|---|---|---|
| DONE，运输约 1.1–1.3 s，滑移 ≤20 mm，落点同量级 | 2× 拉伸是保守策略，可以去掉 | 记录为“运输时长减半”的 S2 结果 |
| DONE 但滑移/落点明显变差 | 拉伸对接触稳定性有保护作用 | 记录；默认保留 2.0 |
| 失败（掉杆/超时/碰撞） | 1× 提速不安全 | 保留失败目录，恢复默认，报告 |
| 时长没有下降 | 计时不由该参数主导 | 停止；报告诊断数据，由规划器决定下一步 |

不调其他参数来“救”这次实验；一次运行就停。

## 5. 边界

- 不跑 CHOMP；不改 S2 对照代码；不改柔顺开关（本轮刚性）；
- 不提交、不推送；保留所有 run 目录与原始 CSV；
- headless。

## 6. 交付

1. 修改文件与 `git diff --stat`；
2. 构建/测试结果；
3. run 目录；
4. `[CONTACT_PLAN_TIMING]` 原文（至少 transport 一行）；
5. 第 3 节指标与 `..._192912` 的对照；
6. 明确结论：运输时长是否真的减半、接触是否守住；
7. 任何偏离本任务书之处。
