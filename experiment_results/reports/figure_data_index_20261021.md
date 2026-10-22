# 图表与数据索引（2026-10-21）

## 已有主图

| 用途 | 图文件 | 数据来源 | 状态 |
|---|---|---|---|
| S2 单样本离线指标 | `docs/figures/s2_offline_metrics_single_sample_20261020.png` | P6 S2 projection/time-profile records | 可用；单样本 |
| JTC/MMC 世界竖直位移 | `docs/figures/hover_ab_vertical_displacement_20261020.png` | `hover_ab_20261020/hover_ab_summary.csv` | 可用；n=3/组 |
| 悬空保持残差/姿态 | `docs/figures/hover_ab_residual_attitude_20261020.png` | 同上 | 可用；异步测量 |
| 杆重前馈对照 | `docs/figures/s2s3_extended_20261020/feedforward_hover_comparison.png` | `s2s3_extended_20261020/feedforward_summary.csv` | 可用；n=3/组 |
| 方向性长脉冲 | `docs/figures/s2s3_extended_20261020/directional_pulse_comparison.png` | `directional_summary.csv` | 可用；n=3/组 |
| S2/S3 规划比较 | `docs/figures/s2s3_extended_20261020/transport_plan_comparison.png` | `transport_plan_comparison.csv` | 可用；仅规划 |
| S2 位置×方法规划指标（2026-10-22） | `docs/figures/s2_time_tradeoff_20261022/s2_position_method_metrics.png` | `s2_time_tradeoff_20261022/s2_time_tradeoff.csv` | 可用；P1/P2及既有长平移；仅规划 |
| S2 时间—jerk 取舍（2026-10-22） | `docs/figures/s2_time_tradeoff_20261022/s2_time_jerk_tradeoff.png` | 同上 | 可用；共同时间与1.0/1.1/1.2伸缩 |
| 历史方向性脉冲 | `experiment_results/reports/hover_directional_perturbation_20261020/mmc_directional_perturbation_comparison.png` | `run_summary.csv` | 可用；原始曲线含漂移 |

## 数据缺口

本机未包含队友机器上的大型原始运行目录，因此以下脚本不能在本机重算完整时序图：

- `experiment_results/a1_runs/s2s3_plan_*`
- `experiment_results/a1_runs/contact_hover_ab_lidio_20261020_*`
- `experiment_results/a1_runs/contact_hover_directional_*`

已有汇总 CSV/JSON 和主图可以用于报告；不能据此恢复缺失的原始时序或重新计算未记录的指标。

## 统一口径

- S2 结果是有限离线样本，不表示执行收益。
- MMC 结果是无 weld 悬空 HOLD，不表示完整 MMC 搬运。
- `*_slip_m`、抓取参考残差和杆—手相对变化不是直接指垫切向滑移。
- 当前模型质量为 0.64 kg；杆重前馈为每臂约 3.1392 N。
- n=3 结果只报告逐次值、范围和中位数，不作显著性或普遍性结论。
