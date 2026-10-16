# 真实接触抓取阶段交接说明（2026-10-16）

本文面向下一位继续开发者，记录本轮代码、诊断证据、当前边界和下一步计划。
公共架构与项目定位仍以 [`PROJECT_GUIDE.md`](../PROJECT_GUIDE.md) 为准。

## 已完成的工作

- `franka_dual_arm` 已初始化为独立 Git 仓库，`origin` 为 `Estara10/franka_ws_1`。
- 保留并编译通过 `ideal_weld` 基线；该入口仍使用原始 weld/equality 逻辑，未改义。
- 新增 `contact_probe` 无 weld 实验入口：双侧接触闭合、force-hold、5 cm 抬升、2 s 保持、下降和释放。
- 建立了从 MuJoCo 物理线程到任务 CSV/日志的接触诊断链，记录：
  - 指位、命令、force-hold 请求；
  - 每侧指垫接触数和接触布尔量；
  - 法向/切向力、杆竖直受力；
  - feedback sequence、仿真时间、weld/equality 状态。
- 修复了左右夹爪 trace 混写问题，当前输出为：
  - `gripper_fingers_mj_left.csv`
  - `gripper_fingers_mj_right.csv`
- 接触 action 的时间预算、接近速度和 force-hold 结果等待已调整；当前可成功进入 LIFT。
- `contact_probe` 默认关闭内部 MuJoCo 自适应柔顺并使用刚性位置验证配置，避免与摩擦承载诊断混杂。
- 抬升轨迹已改为 2.5 mm 物体空间步长、每段至少 1 s，减少轨迹离散造成的接触冲击。
- `contact_probe` 已增加运行时单主指垫 profile：辅助指垫碰撞关闭，主指垫尺寸和法向接触时间常数做有界调整；`ideal_weld` 不启用该 profile。

## 关键实验事实

截至 `franz_contact_probe_fix8_20261016_222657`：

- A1 P2a/P2b、预抓取和下降均通过；
- `ideal_weld` 路径此前可以完成理想刚性抓持；
- contact-probe 的 single-primary-pad profile 已生效（禁用 16 个辅助 pad，调整 4 个主 pad）；
- `contact_probe` 已通过双侧有效力接触建立：
  - `bilateral_force_established`
  - 左右约 5–8 N 法向力；
  - weld/equality 均为关闭；
- fix8 的 LIFT 仍失败，状态为：
  - `E_CONTACT_SLIP_OR_DROP`；
  - 接触 action 已报告 `bilateral_force_established`，所以不是闭合阶段失败；
  - LIFT 中杆最高约只到 `z=0.02070 m`，基本没有脱离支撑面；
  - 最终左右滑移约 `5.0 mm`；
  - 接触数量通常为 8，但失败前会降到 3，且一侧接触布尔量变为 `(0,1)`；
  - 失败时出现单侧法向力接近 0、另一侧法向力/切向力突增的接触切换。

因此当前已证明：

```text
真实接触闭合和有限 force-hold 可以建立
但无 weld 的摩擦承载抬升尚未通过
```

当前不能声称真实接触搬运完成，也不能把 `CONTACT_PROBE` 失败归因于状态机未连通。
状态机已进入 `LIFT`，失败发生在物理承载阶段。

## 当前核心问题

当前最重要的技术问题不是简单的“夹持力不够”，而是抬升过程中接触稳定性不足：

1. 主指垫仍会出现单侧接触或接触数量变化；
2. 夹爪与杆之间的有效接触法向会突变；
3. 杆仍受到支撑面约束，初始脱离阶段对摩擦和冲击最敏感；
4. 5 N/指已经达到当前 force-hold 实现的上限，继续无依据增大不可取；
5. 当前杆模型约 0.64 kg，与目标报告中的 0.5 kg 不同，实验结论必须注明模型质量。

日志中的视觉/MoveIt/关闭期进程错误（如 octomap 未配置、关闭时 process died）不是本轮真实接触首因；
应与任务过程错误分开记录。

## 下一步建议顺序

### 1. 先做单主指垫 profile 的 fix8/fix9 对比

确认日志包含：

```text
single-primary-pad profile: disabled 16 auxiliary pad geoms, tuned 4 primary pads
bilateral_force_established
```

并比较 LIFT 中：

- `rod_pad_contact_count` 是否稳定；
- 左右法向力是否同时非零；
- `rod_force_z_n` 是否具有持续向上的合力；
- 5 mm 滑移发生的仿真时间和杆高度。

### 2. fix8 后的当前判断：主 pad 接触仍会丢失

fix8 说明仅减少辅助 pad 还不够；主 pad 在 LIFT 中仍会出现单侧接触和冲击。下一步应增加每个杆—pad 接触的 geom 名称、接触法向方向和接触点位置记录，确认当前主 pad 是否真正位于杆的相对两侧，还是只擦到杆边缘。不要先无限增加摩擦。

### 3. 再做有限摩擦对比

只有在几何接触方向确认正确后，才做小范围、可解释的对比，例如保持杆质量不变，仅比较：

```text
friction[0] = 3.0（当前基线）
friction[0] = 4.0（单一对比）
```

每次只改一个物理因素，不混合不同质量、不同 pad profile 和不同轨迹数据。

### 4. 若仍不能抬升，改成“脱离支撑面”两阶段动作

可研究先极小幅、低冲击的预抬升/预压动作，确认杆底面脱离支撑后，再进入 5 cm 连续抬升。
但任何预抬升仍必须由真实接触和物理真值验证，不能用 set_body_state、weld 或瞬移替代。

## 代码和实验边界

- `ideal_weld` 必须持续作为理想模型回归入口保留。
- `contact_probe` 的 `CONTACT_LOAD_VALIDATED` 仍需同时满足抬升高度、保持时间、滑移、受力占空比和释放条件。
- 不得把接触 action 成功、双指闭合或单次接触帧当作承载成功。
- 不得直接把摩擦无限增大、减轻杆质量、放宽滑移阈值来制造成功。
- 所有 ROS/MuJoCo/RViz 运行和实验由用户执行；Codex 只提供命令、编译、静态检查和日志分析。
