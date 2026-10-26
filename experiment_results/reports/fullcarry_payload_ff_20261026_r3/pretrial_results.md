# M0单次预试验核对

## 运行

- M0 run：`fullcarry_ff_r3_20261026_pretrial_M0_a5`，状态 `FAILED`，完整流程 `0/1`。
- 对照参考：当前版本此前三次M1完整成功运行 `fullcarry_ff_truth_cbgroup_20261026_M1_01, fullcarry_ff_truth_cbgroup_20261026_M1_02, fullcarry_ff_truth_cbgroup_20261026_M1_03`。这些M1只用于核对版本、条件与参考，不计入r3预试验或正式样本。
- 杆载荷前馈：M0主段每臂发布值 0.000000 N，trace绝对峰值 0.000000000 N；有效trace行数 1222。
- 启动包装脚本对C组固定打印“payload feedforward: enabled”，该行不区分M0/M1；实际开关以任务日志`payload_enabled=0`及前馈trace全零为准。
- 运输主段计划/实际时长：3.040/3.036 s；单独起点衔接 0.500 s。
- 有效运输数据：153个仿真时间对齐样本，覆盖 1.000；真值请求/响应间隔中位数/最大值 10.000/126.000 ms。
- 运输指标：`e_z(t₀)` -10.687 mm；总体均值/RMS -8.758/8.793 mm；相对起点误差变化 `Δe_z` 末值/RMS 2.195/2.084 mm。`Δe_z`保留符号，不将其较小解释为新增下沉减少。
- 释放测量：未产生（运行未进入释放阶段）。


## 失败阶段与直接原因

运行在`LOWER`阶段以`E_CONTACT_SLIP_OR_DROP`中止。末个接触记录的杆—手位置残差为左20.204 mm、右12.239 mm；角残差为左4.114°、右3.249°。该行两侧接触标志仍有效，杆中心高度为60.6 mm（支撑面为0 mm）。控制器切换、运输执行及真值对齐正常；直接失败是下降阶段由杆真值与手部FK计算的相对位姿残差越过既有20 mm保护限值，指向承载过程中的杆—手相对运动，不是启动、通信或控制器软件阻断。该指标不是直接的夹爪垫面滑移测量；一次M0预试验也不能证明关闭前馈是唯一原因。


## 与现有M1参考的比较

| M1记录 | 共同参考起点位置差 (mm) | 起点偏航差 (°) | 参考Z最大差 (mm) | 参考XY最大差 (mm) | 参考偏航最大差 (°) | M1主段前馈 (N/臂) |
|---|---:|---:|---:|---:|---:|---:|
| `fullcarry_ff_truth_cbgroup_20261026_M1_01` | 0.150 | 0.0135 | 0.0059 | 0.1500 | 0.0135 | 3.1392 |
| `fullcarry_ff_truth_cbgroup_20261026_M1_02` | 0.179 | 0.0542 | 0.0103 | 0.1790 | 0.0542 | 3.1392 |
| `fullcarry_ff_truth_cbgroup_20261026_M1_03` | 0.196 | 0.0615 | 0.0519 | 0.1891 | 0.0615 | 3.1392 |

## 进入正式实验的核对

| 核对项 | 结果 |
|---|---|
| M0_full_flow_complete | FAIL |
| M0_payload_zero | PASS |
| M0_common_reference_and_duration | PASS |
| M0_planning_checks | PASS |
| M0_transport_measurement_valid | PASS |
| M0_requested_scene_applied | PASS |
| M0_adopted_reference_matches_nominal | PASS |
| M0_trace_confirms_zero_force | PASS |
| M1_existing_runs_are_valid_current_references | PASS |
| references_comparable_to_existing_M1 | FAIL |

核对未通过：正式批次保持未启动；本次M0结果按实际情况保留。
