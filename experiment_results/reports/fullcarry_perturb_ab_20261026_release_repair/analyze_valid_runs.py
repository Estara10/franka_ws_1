#!/usr/bin/env python3
"""Descriptive performance analysis of ten complete runs per condition.

The original attempt-level completion statistics remain in their source batch.
Two B measurements lost to service timeouts are listed as technical exclusions.
"""
import importlib.util
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ORIGINAL = HERE.parent / "fullcarry_perturb_ab_20261026"
OUT = HERE / "valid_runs_analysis"


def main():
    spec = importlib.util.spec_from_file_location("perturb_analysis", ORIGINAL / "analyze_batch.py")
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)
    OUT.mkdir(exist_ok=True)
    original = analysis.read_csv(ORIGINAL / "formal_attempts.csv")
    followups = analysis.read_csv(HERE / "followup_metrics.csv")
    selected = [r for r in original if r["condition"] == "A" or r["flow_completed"] == "1"]
    selected += [r for r in followups if r["flow_completed"] == "1"]
    excluded = [r for r in original if r["condition"] == "B" and r["flow_completed"] != "1"]
    selected.sort(key=lambda r: (r["condition"], int(r["sequence"])))
    replacement_map = []
    for row in selected:
        row["analysis_slot"] = f"{row['condition']}{int(row['sequence']):02d}"
        prior = next((r for r in excluded if r["sequence"] == row["sequence"]), None)
        row["replaces_run_id"] = prior["run_id"] if prior and row["phase"] != "formal" else ""
        if row["replaces_run_id"]:
            replacement_map.append({"analysis_slot": row["analysis_slot"],
                                    "excluded_run_id": row["replaces_run_id"],
                                    "included_run_id": row["run_id"],
                                    "reason": "final_truth_measurement_timeout; post_repair_complete_run"})
    analysis.write_csv(OUT / "replacement_map.csv", replacement_map)
    for row in excluded:
        row["analysis_disposition"] = "technical_interruption_final_truth_request_timeout"
        row["release_measurement_valid"] = "0"
    analysis.write_csv(OUT / "technical_exclusions.csv", excluded)

    tracking, transport, metrics = [], [], []
    for attempt in selected:
        track, pulse, values = analysis.process_run(attempt)
        row = dict(attempt)
        row.update({key: value for key, value in values.items() if key != "text"})
        row["sample_source"] = "original_formal" if attempt["phase"] == "formal" else "post_repair_followup"
        row["analysis_population"] = "complete_runs_with_valid_measurements"
        metrics.append(row)
        tracking.extend(track)
        transport.extend(pulse)
    analysis.write_csv(OUT / "valid_runs.csv", metrics)
    analysis.write_csv(OUT / "aligned_tracking_samples.csv", tracking)
    analysis.write_csv(OUT / "disturbance_response_samples.csv", transport)
    comparisons = analysis.compare_main_references(selected)
    analysis.write_csv(OUT / "reference_comparison.csv", comparisons)
    metric_names = {
        "transport_rod_z_rms_mm": ("运输竖向误差RMS", "mm"),
        "transport_rod_x_rms_mm": ("运输X向误差RMS", "mm"),
        "transport_rod_position_rms_mm": ("运输三维位置误差RMS", "mm"),
        "transport_rod_orientation_mean_deg": ("运输姿态误差均值", "°"),
        "pulse_peak_abs_x_change_mm": ("施力期X向误差最大变化", "mm"),
        "x_change_1.0s_after_pulse_mm": ("撤力后1.0秒X向误差变化", "mm"),
        "x_change_1.4s_after_pulse_mm": ("撤力后1.4秒X向误差变化", "mm"),
        "release_position_error_mm": ("释放水平落点误差", "mm"),
        "release_target_yaw_abs_error_deg": ("释放偏航误差绝对值", "°"),
        "task_wall_time": ("任务墙钟时长", "s"),
    }
    summary = []
    for group in ("A", "B"):
        rows = [r for r in metrics if r["condition"] == group]
        for key, (_, unit) in metric_names.items():
            analysis.add_stats(summary, group, key, [analysis.fnum(r[key]) for r in rows], unit)
    analysis.write_csv(OUT / "group_summary.csv", summary)

    # Reuse the established disturbance calculation and plotting convention.
    analysis.OUT = OUT
    analysis.plot_disturbance(transport, {r["run_id"]: r for r in metrics})
    plt, np = analysis.plt, analysis.np
    styles = ("-", "--", ":")
    for kind, ylabel, keys in (
        ("position", "位置误差 (mm)", ("rod_position_error_mm", "left_hand_position_error_mm", "right_hand_position_error_mm")),
        ("attitude", "姿态误差 (°)", ("rod_orientation_error_deg", "left_hand_orientation_error_deg", "right_hand_orientation_error_deg")),
    ):
        fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0))
        for ax, phase, phase_name in zip(axes, analysis.PHASES, ("抬升", "运输主段", "下降")):
            for group in ("A", "B"):
                for run in sorted({r["run_id"] for r in tracking if r["condition"] == group}):
                    rows = sorted((r for r in tracking if r["run_id"] == run and r["phase"] == phase),
                                  key=lambda r: r["time_rel_phase_s"])
                    xs = [r["time_rel_phase_s"] for r in rows]
                    for key, style in zip(keys, styles):
                        ax.plot(xs, [r[key] for r in rows], color=analysis.COLORS[group],
                                ls=style, lw=0.65, alpha=0.36)
            ax.set_xlabel(f"{phase_name}阶段仿真时间 (s)")
            ax.grid(True, alpha=0.22)
        axes[0].set_ylabel(ylabel)
        handles = [analysis.Line2D([0], [0], color=analysis.COLORS[g], lw=2, label=f"{g}组，n=10") for g in ("A", "B")]
        handles += [analysis.Line2D([0], [0], color="#444444", ls=style, label=name)
                    for style, name in zip(styles, ("杆", "左手", "右手"))]
        fig.legend(handles=handles, loc="upper center", ncol=5, frameon=False)
        fig.tight_layout(rect=(0, 0, 1, 0.91))
        fig.savefig(OUT / f"full_carry_{kind}_error.png", dpi=180, bbox_inches="tight")
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(8, 3.7))
    for ax, key, ylabel in zip(axes, ("release_position_error_mm", "release_target_yaw_abs_error_deg"),
                               ("释放水平落点误差 (mm)", "释放偏航误差绝对值 (°)")):
        for index, group in enumerate(("A", "B")):
            values = [r[key] for r in metrics if r["condition"] == group]
            ax.scatter(index + np.linspace(-0.15, 0.15, len(values)), values, color=analysis.COLORS[group], s=24)
            mean = float(np.mean(values))
            ax.plot([index - 0.19, index + 0.19], [mean, mean], color=analysis.COLORS[group], lw=2)
        ax.set_xticks([0, 1], labels=["A，n=10", "B，n=10"])
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.22)
    fig.tight_layout()
    fig.savefig(OUT / "placement_error_valid_runs.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    lookup = {(r["condition"], r["metric"]): r for r in summary}
    text = ["# 完整搬运性能对照：有效运行分析\n",
            "A组采用位置控制，B组采用独立笛卡尔阻抗与载荷前馈。下表和图按每组10次具有完整测量的运行汇总。",
            "B组有2次运行因释放后杆真值请求超时未获得完整测量，排除出本性能分析，并在修复测量连接后补充2次；",
            "补充运行保持控制参数、名义物体参考、运输时长及外力设置一致。排除和补充为本次排查后确定，不是预先设定的排除规则。",
            "补充运行对应有效样本表的B02、B07位置，真实运行ID和替换关系见replacement_map.csv。",
            "本分析描述有效完整运行的性能，不估计全部尝试的任务成功率，也不表示两组同一冻结版本、同一轮随机配对。\n",
            "| 指标 | A，均值±标准差 | B，均值±标准差 | 单位 |\n|---|---:|---:|---|",
            ]
    for key, (label, unit) in metric_names.items():
        a, b = lookup[("A", key)], lookup[("B", key)]
        text.append(f"| {label} | {a['mean']:.2f}±{a['sd']:.2f} | {b['mean']:.2f}±{b['sd']:.2f} | {unit} |")
    text += ["\nB组的竖向跟踪误差较小；撤力后1.0 s较接近施力前水平，1.4 s则出现较大的反向偏差。",
             "B组放置误差及任务时长较大。没有无扰动组，因此误差变化并非独立测得的外力净响应；",
             "B组收益属于阻抗与载荷前馈组合，不能归因于阻抗单一因素。\n",
             "图示均为上述有效运行。正式尝试的完整结果及原图仍保存在原批次；两次技术中断见technical_exclusions.csv，",
             "其任务状态未改为DONE。修复开发中的另外两次未完成动作仍单列于上级attempts.csv。\n",
             "报告可采用的简短方法说明：‘两组各分析10次有效完整运行。B组另有2次运行因释放测量超时未纳入性能分析，修复测量连接后补充2次。’",
             "不能将本表的样本数作为原批次100%任务完成率。\n",
             "![位置误差](full_carry_position_error.png)\n![姿态误差](full_carry_attitude_error.png)",
             "![受力响应](disturbance_x_response_and_recovery.png)\n![放置误差](placement_error_valid_runs.png)"]
    (OUT / "results.md").write_text("\n".join(text) + "\n", encoding="utf-8")
    print(f"valid performance sample counts: A={sum(r['condition']=='A' for r in metrics)}, "
          f"B={sum(r['condition']=='B' for r in metrics)}; technical exclusions={len(excluded)}")
    print(OUT / "results.md")


if __name__ == "__main__":
    main()
