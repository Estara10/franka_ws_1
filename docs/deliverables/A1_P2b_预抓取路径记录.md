# A1 P2b 记录：联合预抓取路径规划与校验（不执行）

对应阶段计划“当前执行器提示词：P2b，联合预抓取路径规划与校验（不执行）”。准则以 `PROJECT_GUIDE.md` 为准。
**本轮只规划，未执行任何运动，未开启 weld/attach，未进入 P3，未提交/推送 Git。**

## 完成度

- ✅ **完成**：从**实际起始状态**到**本次选定的同一双臂预抓取解**的真实 MoveIt 联合规划，
  并对**全部航点与相邻段插值状态**在 **P2a 判据模型**上完整复检。
- ✅ 非零 yaw 样本（+10°）下 `geometry=passed ik=passed collision=passed path_validation=passed`，
  **`p2_acceptance=true`**（语义 = 通过本次静态与预抓取路径检查，**不是**搬运 DONE）。
- ✅ 中段障碍**注入负例**成立：端点无碰撞、中段被检出 ⇒ 路径内部确实被检查。
- ✅ 无运动边界：执行/夹爪/weld/attach 计数全为 0，终态 `PREPARED`。
- ❌ **未做（本轮范围外）**：下降抓取、夹爪闭合、weld/attach、时间参数化与运动执行链路、完整搬运 —— 均属 P3。

## 一、新增实现

| 动作 | 文件 |
|---|---|
| 修改 | `dual_arm_controller.{hpp,cpp}`：新增 `runP2bPathCheck()`；抽出 `A1VerifyContext` 承载 P2a 判据模型/场景/ACM/起止状态；接触统计抽为共享 `tallyContactsIn`/`describeTally` |
| 修改 | `a1_sample_validator.{hpp,cpp}`：新增 `subdivisionsFor()`、`interpolateJoints()`（路径段采样，纯逻辑可离线测试） |
| 修改 | `carry_task_context.hpp`、`carry_task_state_machine.cpp`、`launch/…launch.py`：新增 P2b 参数（默认关闭） |
| 修改 | `scripts/a1_run_scene.sh`：新增 `--p2b-check`；抽取 `p2b_path.csv` / `p2b_path_summary.txt`；退出码跟随 `p2_acceptance` |
| 修改 | `test/test_a1_sample_validator.cpp`：新增路径段采样用例（25 → 29 → **31**） |

**判据复用**：P2b 直接使用 P2a 建立的 `A1VerifyContext` —— **同一个** `mujoco_hand_aabb` 模型、
**同一个**独立验证场景（含杆、支撑面）、**同一套** ACM。**不在路径中切换模型**。

## 二、调用规划器的证据

```
[P2B] 调用规划器: planner=PTP, time=10.0s, attempts=5, 起点=当前实际状态, 终点=本次选定预抓取解
[move_group_interface]: MoveGroup action client/server ready
[move_group_interface]: Planning request accepted
[move_group_interface]: Planning request complete!
[move_group_interface]: time taken to generate plan: ...
```

这是**真实规划器调用**，不是自造的线性关节插值。起点用 `setStartStateToCurrentState()`，
**未替换**为机器人默认/ready 起点；记录值与 live 当前状态最大关节差 **0.000000 rad**。

## 三、路径校验项与结果

| 检查 | 结果 |
|---|---|
| 航点数 / 关节数 | 24 / 14（双臂 14 关节齐全，两臂状态都保留） |
| 起点一致 | 与记录起始状态最大差 **0.000000 rad** |
| 逐航点关节限位 | 全部通过（判据模型 limits） |
| 数值有限 / 名称与维数 | 每点位置数 == 关节名数，无 NaN/Inf，否则 `E_P2B_DIM_MISMATCH`/`E_P2B_NOT_FINITE` |
| 相邻航点最大单关节变化 | **0.1088 rad** |
| 终点 FK 残差（对**本次选定**预抓取目标） | 左 **0.0000 mm / 0.0000°**，右 **0.0000 mm / 0.0000°** |
| 插值分辨率 | 最大单关节步长 **0.05 rad** ⇒ 细分 **55** 段，检查 **55** 个插值状态 |
| 插值状态碰撞（杆/地面/跨臂/自碰撞/其他） | **全部为 0** |

**口径声明**：这是**离散采样**检查（55 个状态），**不是**连续碰撞的数学保证。
自碰撞结果是在**既有 SRDF/ACM 豁免**下的结果，含同臂 `hand↔link6`(Never)/`link7`/`link8` 等继承项；
跨臂允许项已在验证副本中关闭。预抓取路径**不允许**手指/hand 穿杆，故全程使用严格 ACM（未启用手指-杆豁免）。

## 四、中段障碍注入负例

要求：证明校验覆盖路径**内部**。第一次实现直接把障碍放在中间航点的左手处，
**被断言拒绝**——因为手掌主要是**原地换向**而非平移，中间点与终点的掌部位姿重叠，
障碍同时压住端点，**不是严格内部遮挡**（扫描 `[9..19]` 全部"终≠0"）。

改为**搜索严格内部位置**：在候选连杆 {link5, link6, link7, link8} × 航点 20%–80% 上，
逐个注入 0.05 m 障碍并验证 `起点=0 且 终点=0 且 中间>0`，取第一个满足者。

```
[P2B][中段负例-注入] 候选扫描:  [mj_left_link5: 无严格内部候选] [mj_left_link6: 无严格内部候选]
                                  [mj_left_link7@9:起0/终0/中3]
[P2B][中段负例-注入] 选用航点 10/24 的 mj_left_link7 位置注入 0.05 m 障碍
     中间航点碰撞=3
     注入后整条路径上有 43 个采样状态被检出碰撞（原为 0）
✓ 端点无碰撞、中段被检出 ⇒ 路径内部确实被检查
```

⇒ **端点无碰撞、中段被检出**：校验确实覆盖路径内部，且远多于端点（43 个采样状态）。
若搜索不到严格内部候选，代码按**失败**处理并如实报告，不静默跳过
（`E_P2B_MIDPATH_NEGATIVE_NOT_CONSTRUCTIBLE`）。注入是**验证注入**，非规划器结果、非实际运动。

## 五、真实运行

```bash
./scripts/a1_run_scene.sh --prepare-mode --p2b-check --x 0.48 --y 0.03 --yaw-deg 10 --run-id a1_p2b_path_yawp10d
```

```
[P2B_RESULT] geometry=passed ik=passed collision=passed path_validation=passed p2_acceptance=true
```

数据路径：`experiment_results/a1_runs/a1_p2b_path_yawp10d/`
- `p2b_path.csv` —— **关节路径本体**（表头 + 24 航点 × 14 关节）
- `p2b_path_summary.txt` —— `model=mujoco_hand_aabb start_source=live_current_state
  goal=this_round_selected_pregrasp planner=PTP waypoints=24 joints=14`
- `task_launch.log`、`prepare_status.json`（含 `p2b_result_line`、`p2_acceptance`）、`manifest.json` 等

路径首点 `(0.0, -0.78499, 0.0, -2.356, 0.0, 1.570998, 0.785)`（两臂相同），
即 MuJoCo 实际起始位形，与记录起点一致。

**规划失败不被当作接受**：`E_P2B_PLAN_FAILED` / 超时 / 起点无效 / 起点被移动
（`E_P2B_START_MOVED`，容差 1e-3 rad）都按失败返回，不会输出 `p2_acceptance=true`。

**无运动证据**：`execute`、`状态: APPROACH`、`状态: GRASP`、`双臂规划成功`、`双臂移动完成`、
`Weld约束已激活`、`Object attached`、夹爪指令出现次数**均为 0**；终态 `PREPARED`。

保留的失败运行：`a1_p2b_path_yawp10`（中段负例设计不当，被断言拒绝）、
`…b`（扫描证明无严格内部候选）、`…c`（加入连杆搜索后首次通过）。**全部保留，未删除、未改写。**

## 六、测试与构建

```bash
export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
colcon build --packages-select dual_arm_carry_task --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test  --packages-select dual_arm_carry_task --ctest-args -R "test_a1_"
```

**5 个套件 130 例全部通过，0 失败**，构建 0 warning。
新增用例：段细分数（含零长段、向上取整、`max_step<=0`、NaN 输入不得产生 0 段）、
插值端点/夹取/t 越界钳制、维数不匹配返回空而不是错长度状态。
已有的错误关节数据、非有限、超限、尺寸不匹配、q/−q 残差等负例继续覆盖接口拒绝路径。

## 七、遗留问题（分级）

| 级别 | 项 |
|---|---|
| 允许后续改进 | 插值检查是**离散采样**（0.05 rad 步长，55 个状态），不构成连续碰撞保证；步长可在参数中收紧。 |
| 允许后续改进 | 规划用 **live move_group** 生成候选路径，再在**独立判据场景**完整复检。两者差异：live 场景不含支撑面、且用 URDF 手掌近似；因此路径须经复检才被接受（本轮即如此）。 |
| 允许后续改进 | PTP 生成的是关节空间同步运动；本轮只校验几何可行性，**未**做时间参数化、速度/加速度/jerk 检查。 |
| 允许后续改进 | 自碰撞沿用既有 SRDF 同臂豁免（`hand↔link6` 为 `Never`），是该规则下的结果。 |
| 允许后续改进 | 验证支撑面为 **6×6×2.0 m 的深 BOX**（顶面 z=0），比之前的 2 cm 薄板覆盖更深，但仍是有限域，不是无限地面。 |
| 允许后续改进 | 既有启动崩溃（`spin_some`/`move_group` segfault）本轮亦出现，失败运行原样保留、无隐藏重试。 |
| 当前不需要解决 | `p2_acceptance` **只**表示通过本次静态与预抓取路径检查；**不表示**可搬运、不表示真实硬件安全。下降/闭合/抓持/完整搬运与时间参数化属 P3。 |

## 八、更正

P2a 记录的两处表述更正（地面负例机理、手掌负例障碍位姿）已补记在
`docs/deliverables/A1_P2a_静态有效性检查记录.md` 末尾，本轮未重跑已通过的 P2a 正向检查；
其中障碍位姿已按更正意见改为放在 AABB 真实中心，最近一次运行仍通过。
