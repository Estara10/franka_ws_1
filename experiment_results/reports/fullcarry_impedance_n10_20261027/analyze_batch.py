#!/usr/bin/env python3
"""Summarize all ten current B attempts and compare with the existing A data."""
import importlib.util
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent
ORIGINAL = OUT.parent / "fullcarry_perturb_ab_20261026"


def main():
    spec = importlib.util.spec_from_file_location("existing_perturb_analysis", ORIGINAL / "analyze_batch.py")
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)
    attempts = [r for r in analysis.read_csv(OUT / "attempts.csv") if r["action_started"] == "1"]
    attempts.sort(key=lambda r: int(r["sequence"]))
    baseline = [r for r in analysis.read_csv(ORIGINAL / "formal_attempts.csv") if r["condition"] == "A"]
    tracking, transport, current, comparison = [], [], [], []
    for attempt in baseline + attempts:
        track, pulse, values = analysis.process_run(attempt)
        row = dict(attempt)
        row.update({key: value for key, value in values.items() if key != "text"})
        row["data_source"] = "current_B" if attempt["condition"] == "B" else "existing_A"
        row["release_measurement_valid"] = int(math.isfinite(row["release_position_error_mm"]))
        row["retreat_completed"] = int(attempt["flow_completed"] == "1")
        row["completed_task_wall_time"] = row["task_wall_time"] if row["retreat_completed"] else float("nan")
        comparison.append(row)
        if attempt["condition"] == "B":
            current.append(row)
        tracking.extend(track)
        transport.extend(pulse)
    analysis.write_csv(OUT / "formal_attempts.csv", current)
    analysis.write_csv(OUT / "comparison_runs.csv", comparison)
    supplemental = []
    for attempt in analysis.read_csv(OUT / "supplement" / "attempts.csv"):
        if attempt["action_started"] != "1":
            continue
        _, _, values = analysis.process_run(attempt)
        row = dict(attempt)
        row.update({key: value for key, value in values.items() if key != "text"})
        row["data_source"] = "additional_B_after_original_ten"
        supplemental.append(row)
    if supplemental:
        analysis.write_csv(OUT / "supplement_metrics.csv", supplemental)
    completed = [r for r in current + supplemental if r["flow_completed"] == "1"]
    analysis.write_csv(OUT / "completed_B_runs.csv", completed)
    analysis.write_csv(OUT / "aligned_tracking_samples.csv", tracking)
    analysis.write_csv(OUT / "disturbance_response_samples.csv", transport)
    startup = [r for r in analysis.read_csv(OUT / "attempts.csv") if r["action_started"] != "1"]
    analysis.write_csv(OUT / "pre_action_exceptions.csv", startup,
                       list(startup[0]) if startup else ["run_id", "direct_reason"])
    metrics = {
        "transport_rod_z_rms_mm": ("运输竖向位置误差RMS", "mm"),
        "transport_rod_x_rms_mm": ("运输X向位置误差RMS", "mm"),
        "transport_rod_position_rms_mm": ("运输三维位置误差RMS", "mm"),
        "transport_rod_orientation_mean_deg": ("运输姿态误差均值", "°"),
        "pulse_peak_abs_x_change_mm": ("施力期X向误差最大变化", "mm"),
        "x_change_1.0s_after_pulse_mm": ("撤力后1.0 s的X向变化", "mm"),
        "x_change_1.4s_after_pulse_mm": ("撤力后1.4 s的X向变化", "mm"),
        "release_position_error_mm": ("释放水平落点误差", "mm"),
        "release_target_yaw_abs_error_deg": ("释放偏航误差绝对值", "°"),
        "task_wall_time": ("任务尝试墙钟时长", "s"),
        "completed_task_wall_time": ("完成任务墙钟时长", "s"),
    }
    summary = []
    for group in ("A", "B"):
        rows = [r for r in comparison if r["condition"] == group]
        for key, (_, unit) in metrics.items():
            analysis.add_stats(summary, group, key, [r[key] for r in rows], unit)
    analysis.write_csv(OUT / "group_summary.csv", summary)
    references = analysis.compare_main_references(baseline + attempts)
    analysis.write_csv(OUT / "reference_comparison.csv", references)
    analysis.OUT = OUT
    analysis.plot_disturbance(transport, {r["run_id"]: r for r in comparison})
    analysis.plot_placement(comparison)

    plt, np = analysis.plt, analysis.np
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.7))
    for ax, key, label in zip(axes, ("transport_rod_z_rms_mm", "transport_rod_orientation_mean_deg"),
                              ("运输竖向误差RMS (mm)", "运输姿态误差均值 (°)")):
        for index, group in enumerate(("A", "B")):
            vals = [r[key] for r in comparison if r["condition"] == group and math.isfinite(r[key])]
            ax.scatter(index + np.linspace(-0.15, 0.15, len(vals)), vals, color=analysis.COLORS[group], s=25)
            if vals:
                mean = float(np.mean(vals))
                ax.plot([index - 0.19, index + 0.19], [mean, mean], color=analysis.COLORS[group], lw=2)
            ax.text(index, 0.98, f"n={len(vals)}", transform=ax.get_xaxis_transform(), ha="center", va="top")
        ax.set_xticks([0, 1], labels=["A：位置控制", "B：阻抗＋前馈"])
        ax.set_ylabel(label)
        ax.grid(True, axis="y", alpha=0.22)
    fig.tight_layout()
    fig.savefig(OUT / "transport_tracking_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    done = sum(r["flow_completed"] == "1" for r in current)
    lookup = {(r["condition"], r["metric"]): r for r in summary}
    text = ["# 完整搬运阻抗与载荷前馈：10次重复结果\n",
            "> **存档批次，未采用。** 用户于2026-10-27决定原定10次和追加B11仅存档，不纳入当前报告、选用图表或主统计。详见[存档状态](ARCHIVE_STATUS.md)。\n",
            f"本次计划10次，已接受动作{len(current)}次，完整完成{done}次；动作前异常{len(startup)}次另列。",
            "原定10次统计全部计入，未混入旧B样本或后加补充运行，未替换失败。Word未修改。\n",
            "场景为(0.50,0,+10°)至(0.40,0,+20°)，抬升0.40 m。共同运输主段3.040 s，",
            "主段开始1.0 s后施加world +X总力2 N、持续0.5 s。独立阻抗、载荷前馈和原放置流程保持不变。\n",
            "## 逐次结果\n",
            "| 编号 | 完整结果 | 运输竖向RMS/mm | 放置XY误差/mm | 偏航绝对误差/° | 直接中止原因 |",
            "|---|---|---:|---:|---:|---|"]
    def fmt(value):
        return f"{value:.3f}" if math.isfinite(value) else "—"
    for r in current:
        text.append(f"| B{int(r['sequence']):02d} | {r['task_state']} | {r['transport_rod_z_rms_mm']:.3f} | "
                    f"{fmt(r['release_position_error_mm'])} | {fmt(r['release_target_yaw_abs_error_deg'])} | {r['direct_reason']} |")
    text += ["\n## 性能对照\n",
             "A为已有位置控制10次数据，B为本次新执行的10次。以下为描述性均值±标准差，括号内为有效测量数。",
             "| 指标 | A：位置控制 | B：阻抗＋前馈 | 单位 |\n|---|---:|---:|---|"]
    for key, (label, unit) in metrics.items():
        a, b = lookup[("A", key)], lookup[("B", key)]
        text.append(f"| {label} | {a['mean']:.2f}±{a['sd']:.2f}（{a['n']}） | "
                    f"{b['mean']:.2f}±{b['sd']:.2f}（{b['n']}） | {unit} |")
    if supplemental:
        additional_done = sum(r["flow_completed"] == "1" for r in supplemental)
        text += ["\n## 另行补充运行\n",
                 f"原定10次之后，用户授权追加{len(supplemental)}次，同参数完成{additional_done}次。原定结果未删除或覆盖。",
                 f"全部本轮动作共{len(current)+len(supplemental)}次，完整{len(completed)}次；累计有效放置测量见completed_B_runs.csv。"]
        for r in supplemental:
            text.append(f"追加B{int(r['sequence']):02d}：{r['task_state']}，水平放置误差{r['release_position_error_mm']:.3f} mm，"
                        f"偏航绝对误差{r['release_target_yaw_abs_error_deg']:.3f}°，中止原因：{r['direct_reason'] or '无'}。")
        valid_xy = analysis.metric_stats([r["release_position_error_mm"] for r in completed])
        valid_yaw = analysis.metric_stats([r["release_target_yaw_abs_error_deg"] for r in completed])
        text.append(f"全部完成任务的放置水平误差均值{valid_xy['mean']:.3f} mm（n={valid_xy['n']}），"
                    f"偏航绝对误差均值{valid_yaw['mean']:.3f}°（n={valid_yaw['n']}）。这只描述完成任务，不替代原定十次的总体统计。")
    text += ["\n## 结果解释\n",
             "本次B组运输竖向误差明显小于位置控制参考数据，但水平跟踪和最终放置精度并未同时改善。",
             "撤力后1.0 s时B更接近施力前水平，随后出现较大的反向偏差，不能概括为整体恢复更好。",
             "B06在下降阶段左侧杆—手位置残差达到20.433 mm，超过既有20 mm条件；",
             "最后记录角残差约3.90°/2.64°，低于5°。未进入放置释放，未取得释放误差；",
             "这是位姿残差触发的任务中止，不是释放真值服务超时，也不单凭错误码认定物理掉落。",
             "表中的尝试耗时包含未完成运行，完成任务耗时另外统计，避免将提前中止当作更快完成。"]
    text += ["\n当前对照评价阻抗与前馈组合，不将差异单独归因于阻抗。没有无扰动搬运组，",
             "施力前后的误差变化含运动背景，不等于独立测得的外力净响应。正负变化应保留，不把反向偏差称为完全恢复。",
             "放置误差只统计已取得有效测量的样本，缺失不补零。全部运行索引在attempts.csv。\n",
             "![运输跟踪](transport_tracking_comparison.png)\n![受力响应](disturbance_x_response_and_recovery.png)",
             "![放置与完成](placement_error_and_completion.png)"]
    (OUT / "results.md").write_text("\n".join(text) + "\n", encoding="utf-8")
    print(f"B complete={done}/{len(current)}; startup_exceptions={len(startup)}")
    print(OUT / "results.md")


if __name__ == "__main__":
    main()
