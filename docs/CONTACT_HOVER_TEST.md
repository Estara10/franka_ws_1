# 刚性无 weld 悬空实验：2026-10-19 修订

本轮只修实验链与数据解释，不改质量、摩擦、求解器、夹持力或手指接触几何。
不是“滑移已解决”，也没有将 MMC 接入接触承载；先建立 JTC 刚性短时基线。

## 判据与范围

- 固定场景 (0.50, 0, +10°)，10 N/指请求，默认抬升 50 mm、保持 3 s。
- 仅 `contact_hover_only=true`：LIFT 采样中曾达到杆真实上升 45 mm 后锁存。
  HOLD 不再逐帧复用 45 mm 阈值，避免将“已抬起后的下降”报成“未抬起”。
- 锁存不等于可以掉落：HOLD 的倾斜感知杆底净空须 >3 mm、相对 HOLD 起点下降须
  ≤20 mm；原抓取参考的位移残差 ≤20 mm / 姿态残差 ≤2° 始终保留。
  5 mm 仍是更严格的研究目标，不能将诊断完成等同于 5 mm 达标。
- 完整搬运的连续 45 mm 高度保护不受 hover 锁存影响。
- 输入限定高度 45–100 mm、保持 1–10 s；非法值在抓取前拒绝。
- 不再预先规划不用的 40 cm 路径。HOLD 后检查实测关节稳定再规划 LOWER，
  开爪前检查支撑面到位，开爪后确认释放并同步 MoveIt 场景。
- HOVER_LIFT 前增加 bilateral-contact gate：左右臂的两个指侧都须有几何接触，且四个
  聚合 pad 法向力均 ≥0.25 N，连续物理时间 0.25 s；3 s 内未建立则拒绝抬升。
  这是防止“未完成夹持就抬升”的时序保护，不是把夹持力/摩擦阈值改成成功。
- LOWER 可重新建立规划参考，但 hover 测量和反滑移检查始终保留闭合后的原参考。
  `[CONTACT_HOVER_RESULT]` 只证明 HOLD 完成；释放和最终 DONE 必须另行确认。

当前新增的 `CONTACT_PROBE_ADAPTIVE_FORCE_HOLD` 是**显式实验开关**，默认 `false`。
打开后，物理线程在已有位置保持目标和 ±2F actuator force cap 之内，根据左右 pad
法向力平均值对 tendon position target 做有界、小带宽修正；它不是理想力执行器，也
不是已经验证的防滑保证。刚性原始基线必须保持 `false`，用于 A/B 对照。

## 数据含义

`contact_probe.csv` 保持原有列。新 `hover_diagnostics.csv` 记录原始抓取参考下的
位移残差、期望杆中心减真实杆中心的 Z 差、杆姿态、倾斜感知净空、反馈时间差、
冻结杆固定点在当前手坐标系中的坐标。该固定点是运动学代理，不是局部 pad 触觉滑移。
杆中心残差含旋转杠杆效应，不能将其模长全部称为“竖直下滑”。

核对 `FrankaMjHardwareSystem::read()`：任务反馈的 `rod_force_xyz` 仅累计每臂
group=3 的 box 指垫与杆之间的接触，经 `contact.frame` 转置和 geom 次序符号映射到
世界系，不包含地面反力。每个接触若任一平移力分量非有限或绝对值 >150 N 则不累计；
这个过滤不限制仿真的实际受力，且多个保留接触之和仍可超过150 N。
高频 `contact_probe_contacts.csv` 保留原 wrench、世界力和 `force_included`，必要时
按同一 `feedback_seq` 聚合对照，不能把任务侧过滤合力当完整物体力平衡。

杆真值服务、关节反馈及左右指力并非同一物理步的联合快照；快照中某侧失联或
合力低于 mg 不能单独证明持续承载不足。当前不存在直接地面反力测量，几何净空仅作
净空证据。大力峰可能包含再接触冲击、求解或采样效应，本轮不裁剪峰值或认定单一根因。

## 运行与产物

```bash
export MUJOCO_DIR="$HOME/.local/mujoco-3.2.0"
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
unset CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY CONTACT_PLAN_COMPARE CONTACT_TRANSPORT_PLAN_ONLY
export CONTACT_PROBE_RIGID=true CONTACT_PROBE_HOVER_ONLY=1
export CONTACT_PROBE_HOVER_HEIGHT_M=0.05 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.0
export CONTACT_PROBE_RUN_ID="contact_hover_rigid_fix2_$(date +%Y%m%d_%H%M%S)"
bash scripts/run_contact_probe_local.sh --headless
```

wrapper 会在任务结束后（包括失败）生成 `hover_analysis.json` 和 `hover_analysis.txt`，
不改变运行退出码。统计包含各阶段接触采样占比、接触点计数变化、原始读数 p95/峰值、
杆和手的高度/横向变化、左右反馈时间差、重复序号。采样占比不是逐物理步时间占空比，
有限采样的峰值也不是保证捕获的真实最高峰；异常行保留，不自动删除。

离线分析也可手动运行（输出文件必须不存在）：

```bash
python3 scripts/analyze_contact_hover.py --run-dir "experiment_results/a1_runs/$CONTACT_PROBE_RUN_ID" \
  --output /tmp/hover_analysis_new.json
```

收尾将本轮 sim 进程组改为 TERM 优先（同本机空载测试），runner/task 的处理不变；
此路径尚需本次 hover 实测，不能从空载正常退出推断全部退出问题已经根除。

## 复验之后

先得到完整 LIFT/HOLD/LOWER/释放与失败数据，再决定是否接 MMC 的悬空 HOLD。
下一阶段不直接搬运、不同时更改接触参数；必要时检查同一物理步的力平衡与接触速度。
此前 elliptic/impratio 尝试已失败，不无依据重开扫描。当前功能完成不代表接触稳定性
或外部承载阻抗已验证。
