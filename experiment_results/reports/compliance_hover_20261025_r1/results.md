# 柔顺控制补充实验结果

本批独立于原80次ABCD、2026-10-20悬空专项及其他批次。样本均按首次预抓取运动请求获接受计数；正式动作中止或失败保留，不替换。组间区间为独立样本Welch t区间；区组只用于平衡顺序，不作同物理起点配对。曲线使用全部有效运行及共同覆盖范围，不外推尾段。

## 正式尝试和完成情况

| 条件 | 正式尝试 | DONE | 主指标有效样本 |
|---|---:|---:|---:|
| F0 | 10 | 10 | 10 |
| F1 | 10 | 10 | 10 |
| K400-N | 10 | 10 | 10 |
| K400-F | 10 | 10 | 10 |
| K200-N | 10 | 8 | 9 |
| K200-F | 10 | 10 | 10 |

正式动作前异常记录：7；正式任务未完成：2。计划外正式运行：1，原始数据保留但排除主分析。逐条见 `attempts.csv`/`run_index.csv`。

失败阶段与缺失测量：
- 动作前 F0_01 / compliance_hover_20261025_r1_feedforward_F0_b1_a1: E_SIM_COMPONENT_DIED_AT_STARTUP_DOMAIN_RANGE；不计正式次数。
- 动作前 F0_02 / compliance_hover_20261025_r1_feedforward_F0_b2_a1: E_MMC_STARTUP_PREFLIGHT_REJECTED；不计正式次数。
- 动作前 F0_06 / compliance_hover_20261025_r1_feedforward_F0_b6_a1: E_P2B_PLAN_FAILED；不计正式次数。
- 动作前 F1_10 / compliance_hover_20261025_r1_feedforward_F1_b10_a1: E_P2B_PLAN_FAILED；不计正式次数。
- 动作前 K200-F_05 / compliance_hover_20261025_r1_directional_K200_F_b5_a1: E_MMC_STARTUP_PREFLIGHT_REJECTED；不计正式次数。
- 动作前 K400-F_06 / compliance_hover_20261025_r1_directional_K400_F_b6_a1: E_P2B_PLAN_FAILED；不计正式次数。
- 动作前 K200-F_09 / compliance_hover_20261025_r1_directional_K200_F_b9_a1: E_P2B_PLAN_FAILED；不计正式次数。
- 正式 K200-N_04 / compliance_hover_20261025_r1_directional_K200_N_b4_a1: E_CONTACT_DISPATCH_START_MOVED；对应测量是否有效见 `runs.csv`。
- 正式 K200-N_05 / compliance_hover_20261025_r1_directional_K200_N_b5_a1: E_CONTACT_SLIP_OR_DROP；对应测量是否有效见 `runs.csv`。

K200-N_04在接触抬升派发时检测到起点移动2.12 mm，超过既有2.00 mm限制，未进入MMC窗口；K200-N_05已完成完整5 s窗口并保留测量，随后在下降阶段报接触滑移/掉落，任务未完成。没有放宽阈值或补跑。

## 载荷前馈

主要指标为从首次确认 MMC 参考后的观测入口到 3.5 s 内最后一个有效样本的杆 world-Z 变化（负值表示下沉）。不把低于3.5 s的缺失尾段外推。

| 条件 | 有效n | 均值 ± 样本SD (mm) | 范围 (mm) | 完成/正式尝试 |
|---|---:|---:|---:|---:|
| F0 | 10 | -9.084 ± 0.665 | -10.114 to -8.347 | 10/10 |
| F1 | 10 | -2.997 ± 0.538 | -3.665 to -2.223 | 10/10 |

F0−F1 的 rod_drop_3p5_mm 均值差=-6.086 mm，95% Welch区间 [-6.656, -5.516] mm（有效n=10/10）。该有符号位移差为负时，表示F1比F0下沉少（或相对上移）。

F0−F1 的 rod_delta_0p5_to_end_mm 均值差=-3.505 mm，95% Welch区间 [-3.879, -3.131] mm（有效n=10/10）。该有符号位移差为负时，表示F1比F0下沉少（或相对上移）。

实发核对：F0 10次的双臂MMC trace全窗最大值均为0 N；F1 10次均达到每臂3.1392 N。F1 trace从首个非零样本到达到目标99%的观测间隔为0.516–0.522 s（均值0.519 s），与冻结的0.5 s渐增一致到采样分辨率。逐次发布曲线见 `runs.csv` 和原始 `mmc_hold_trace.csv`。

| 条件 | 杆0.5–3.5 s ΔZ (mm) | 左手ΔZ (mm) | 右手ΔZ (mm) | 杆姿态变化 (°) |
|---|---:|---:|---:|---:|
| F0 | -3.290 ± 0.424 (n=10) | -1.626 ± 0.169 (n=10) | -1.497 ± 0.112 (n=10) | 0.362 ± 0.061 (n=10) |
| F1 | 0.215 ± 0.369 (n=10) | 1.727 ± 0.216 (n=10) | 1.991 ± 0.225 (n=10) | 0.179 ± 0.029 (n=10) |

## X向刚度和外力

具备有效MMC窗口的运行日志确认了实际增益：K400使用Kdiag=(400,400,400,20,20,20)，K200使用(200,400,400,20,20,20)；两臂阻尼比0.8、零空间刚度10，按现有在线 operational-inertia 方法计算阻尼。任务坐标为两臂 `mj_*_link0`，其朝向与world对齐。逐次实录值见 `runs.csv`。

曲线对齐实际物理事件 `rod_force_events.csv:pulse_start`；20次有力运行的起始时刻为首参考后1.000–1.002 s，脉冲持续1.000–1.002 s，总力2 N并在结束事件清零。N组按首参考后1.0 s和同一1.0 s时长作虚拟区间。图中位移相对各运行脉冲起点位置，未作线性背景外推。响应估计为同刚度施力组减无外力组的组均值差；交互估计为(K200施力−K200无力)−(K400施力−K400无力)。这属于独立组均值对照估计，不是直接测得的净外力响应。共同可用的撤力后末端为2.891 s；所有组都以该实测时点计算，不外推到准确3 s。

| 刚度 | 条件 | 杆X脉冲末均值±SD (mm) | 脉冲末范围 (mm) | 撤力+1s均值±SD (mm) | 撤力近+2.891s均值±SD (mm) |
|---|---|---:|---:|---:|---:|
| 400 N/m | K400-N | 0.201 ± 0.085 (n=10) | 0.107 to 0.386 | 0.287 ± 0.089 (n=10) | 0.375 ± 0.127 (n=10) |
| 400 N/m | K400-F | 1.690 ± 0.105 (n=10) | 1.517 to 1.814 | 0.843 ± 0.102 (n=10) | 0.620 ± 0.104 (n=10) |
| 200 N/m | K200-N | 0.277 ± 0.092 (n=9) | 0.157 to 0.395 | 0.456 ± 0.107 (n=9) | 0.629 ± 0.109 (n=9) |
| 200 N/m | K200-F | 2.282 ± 0.110 (n=10) | 2.102 to 2.459 | 1.452 ± 0.134 (n=10) | 1.071 ± 0.131 (n=10) |

### 组间估计

| 指标 | 对照 | 均值差 (mm) | 95% Welch区间 (mm) | n |
|---|---|---:|---:|---:|
| rod_x_pulse_end_mm | K400 force - no force | 1.490 | [1.399, 1.580] | 10/10 |
| rod_x_pulse_end_mm | K200 force - no force | 2.004 | [1.907, 2.102] | 10/9 |
| rod_x_pulse_end_mm | (K200 F-N) - (K400 F-N) | 0.515 | [0.387, 0.643] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| rod_x_withdrawal_plus1_mm | K400 force - no force | 0.557 | [0.467, 0.647] | 10/10 |
| rod_x_withdrawal_plus1_mm | K200 force - no force | 0.996 | [0.879, 1.113] | 10/9 |
| rod_x_withdrawal_plus1_mm | (K200 F-N) - (K400 F-N) | 0.439 | [0.297, 0.581] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| rod_x_withdrawal_near3_mm | K400 force - no force | 0.245 | [0.136, 0.354] | 10/10 |
| rod_x_withdrawal_near3_mm | K200 force - no force | 0.442 | [0.326, 0.559] | 10/9 |
| rod_x_withdrawal_near3_mm | (K200 F-N) - (K400 F-N) | 0.197 | [0.043, 0.351] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_hand_x_pulse_end_mm | K400 force - no force | 1.020 | [0.979, 1.060] | 10/10 |
| left_hand_x_pulse_end_mm | K200 force - no force | 1.411 | [1.332, 1.491] | 10/9 |
| left_hand_x_pulse_end_mm | (K200 F-N) - (K400 F-N) | 0.392 | [0.305, 0.479] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_hand_x_withdrawal_plus1_mm | K400 force - no force | 0.336 | [0.272, 0.399] | 10/10 |
| left_hand_x_withdrawal_plus1_mm | K200 force - no force | 0.758 | [0.622, 0.893] | 10/9 |
| left_hand_x_withdrawal_plus1_mm | (K200 F-N) - (K400 F-N) | 0.422 | [0.276, 0.567] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_hand_x_withdrawal_near3_mm | K400 force - no force | 0.055 | [-0.111, 0.221] | 10/10 |
| left_hand_x_withdrawal_near3_mm | K200 force - no force | 0.490 | [0.309, 0.670] | 10/9 |
| left_hand_x_withdrawal_near3_mm | (K200 F-N) - (K400 F-N) | 0.435 | [0.200, 0.670] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_hand_x_pulse_end_mm | K400 force - no force | 1.255 | [1.169, 1.341] | 10/10 |
| right_hand_x_pulse_end_mm | K200 force - no force | 1.657 | [1.535, 1.778] | 10/9 |
| right_hand_x_pulse_end_mm | (K200 F-N) - (K400 F-N) | 0.401 | [0.257, 0.545] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_hand_x_withdrawal_plus1_mm | K400 force - no force | 0.680 | [0.604, 0.756] | 10/10 |
| right_hand_x_withdrawal_plus1_mm | K200 force - no force | 0.990 | [0.817, 1.163] | 10/9 |
| right_hand_x_withdrawal_plus1_mm | (K200 F-N) - (K400 F-N) | 0.310 | [0.126, 0.494] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_hand_x_withdrawal_near3_mm | K400 force - no force | 0.474 | [0.355, 0.592] | 10/10 |
| right_hand_x_withdrawal_near3_mm | K200 force - no force | 0.545 | [0.360, 0.729] | 10/9 |
| right_hand_x_withdrawal_near3_mm | (K200 F-N) - (K400 F-N) | 0.071 | [-0.140, 0.282] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_rel_x_pulse_end_mm | K400 force - no force | 0.470 | [0.373, 0.566] | 10/10 |
| left_rel_x_pulse_end_mm | K200 force - no force | 0.593 | [0.467, 0.719] | 10/9 |
| left_rel_x_pulse_end_mm | (K200 F-N) - (K400 F-N) | 0.123 | [-0.030, 0.276] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_rel_x_withdrawal_plus1_mm | K400 force - no force | 0.221 | [0.131, 0.312] | 10/10 |
| left_rel_x_withdrawal_plus1_mm | K200 force - no force | 0.238 | [0.035, 0.442] | 10/9 |
| left_rel_x_withdrawal_plus1_mm | (K200 F-N) - (K400 F-N) | 0.017 | [-0.200, 0.235] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| left_rel_x_withdrawal_near3_mm | K400 force - no force | 0.190 | [-0.011, 0.391] | 10/10 |
| left_rel_x_withdrawal_near3_mm | K200 force - no force | -0.047 | [-0.269, 0.174] | 10/9 |
| left_rel_x_withdrawal_near3_mm | (K200 F-N) - (K400 F-N) | -0.238 | [-0.525, 0.050] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_rel_x_pulse_end_mm | K400 force - no force | 0.234 | [0.156, 0.312] | 10/10 |
| right_rel_x_pulse_end_mm | K200 force - no force | 0.348 | [0.267, 0.428] | 10/9 |
| right_rel_x_pulse_end_mm | (K200 F-N) - (K400 F-N) | 0.114 | [0.005, 0.222] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_rel_x_withdrawal_plus1_mm | K400 force - no force | -0.123 | [-0.207, -0.039] | 10/10 |
| right_rel_x_withdrawal_plus1_mm | K200 force - no force | 0.006 | [-0.112, 0.124] | 10/9 |
| right_rel_x_withdrawal_plus1_mm | (K200 F-N) - (K400 F-N) | 0.129 | [-0.011, 0.269] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |
| right_rel_x_withdrawal_near3_mm | K400 force - no force | -0.229 | [-0.349, -0.108] | 10/10 |
| right_rel_x_withdrawal_near3_mm | K200 force - no force | -0.102 | [-0.251, 0.046] | 10/9 |
| right_rel_x_withdrawal_near3_mm | (K200 F-N) - (K400 F-N) | 0.126 | [-0.057, 0.309] | 10/9/10/10 (K200-F/K200-N/K400-F/K400-N) |

### 竖向、姿态和完成情况

| 条件 | 完成/正式 | 杆ΔZ (mm) | 左手ΔZ (mm) | 右手ΔZ (mm) | 杆姿态变化 (°) |
|---|---:|---:|---:|---:|---:|
| K400-N | 10/10 | -3.016 ± 0.595 (n=10) | -1.158 ± 0.302 (n=10) | -0.529 ± 0.248 (n=10) | 0.237 ± 0.027 (n=10) |
| K400-F | 10/10 | -2.422 ± 0.875 (n=10) | -1.110 ± 0.942 (n=10) | -0.362 ± 0.809 (n=10) | 0.253 ± 0.017 (n=10) |
| K200-N | 8/10 | -3.047 ± 0.376 (n=9) | -1.554 ± 0.377 (n=9) | -0.788 ± 0.311 (n=9) | 0.244 ± 0.046 (n=9) |
| K200-F | 10/10 | -2.811 ± 0.461 (n=10) | -1.281 ± 0.582 (n=10) | -0.502 ± 0.451 (n=10) | 0.251 ± 0.039 (n=10) |

## 双手、姿态、竖向保持和限制

逐次杆与双手世界位移、杆减双手相对位移变化、杆姿态、实发前馈、实际脉冲事件和任务状态均见 `runs.csv`。CSV未覆盖的指标为空值，不以0代替。杆—手相对运动只能说明两者的相对位置变化，不能直接称为指垫滑移。每个运行原始曲线、接触与诊断记录保存在 `experiment_results/a1_runs/<run_id>/`；目录索引见 `run_index.csv`。

### 图表

1. `figures/feedforward_vertical_displacement.png`：F0/F1杆和双手竖向曲线，所有有效单次曲线、组均值和范围。
2. `figures/feedforward_endpoint_runs.png`：前馈主要指标逐次散点、均值和样本SD。
3. `figures/directional_x_pulse_aligned.png`：四条件X向曲线与1 s实际/虚拟脉冲时段。
4. `figures/directional_withdrawal_runs.png`：脉冲末、撤力后1 s与共同可用近3 s的逐次让步/残余。

区间用于表达该批有限样本的不确定性，不作显著性检验结论、总体成功率或真机能力外推。
