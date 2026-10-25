#!/usr/bin/env python3
"""Analyze the separate matched-wait experiment without changing the n=20 study."""
import csv
import json
import math
import re
import statistics
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_abcd_s0_today import parse_run
from analyze_abcd_n20_20261025 import apply_release_wait_marker
from scientific_style import configure_scientific_style

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "experiment_results/reports/explanatory_20261025"
COLORS = {"P": "#0C5DA5", "T": "#C43D4B", "I0": "#E69F00", "I1": "#2A9D8F"}
configure_scientific_style()


def read(path):
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def match(pattern, text):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else None


def numeric(rows, key):
    return [float(r[key]) for r in rows if r.get(key) not in (None, "") and math.isfinite(float(r[key]))]


def main():
    rows, curves = [], {}
    for item in read(OUT / "attempts.csv"):
        if item["formal"] != "1":
            continue
        run_dir = ROOT / item["run_dir"]
        log = (run_dir / "task_launch.log").read_text(errors="replace")
        row = parse_run({"P": "A", "T": "B", "I0": "C", "I1": "C"}[item["condition"]], item["run_id"])
        apply_release_wait_marker(row, log)
        row = {k: v for k, v in row.items() if not k.startswith("_")}
        row.update(item)
        row["candidate_selected"] = match(r"\[ABCD_TIME_PROFILE_ACCEPTANCE\].*selected_candidate=(\d)", log)
        row["matched_wait_sim_s"] = match(r"\[ABCD_MATCHED_WAIT\].*actual_sim_sec=([\d.]+)", log)
        row["post_lift_to_dispatch_sim_s"] = match(r"\[TRANSPORT_EVENT\] event=start .*post_lift_sim_sec=([\d.]+)", log)
        row["actual_payload_force_max_n"] = match(r"\[ABCD_PAYLOAD_REFERENCE\].*max_abs_per_arm_n=([\d.]+)", log)
        start = match(r"\[TRANSPORT_EVENT\] event=start sim_time=([\d.]+)", log)
        end = match(r"\[TRANSPORT_EVENT\] event=end sim_time=([\d.]+)", log)
        row["transport_event_sim_s"] = end-start if start is not None and end is not None else None
        samples = []
        seen = set()
        for s in read(run_dir / "contact_probe.csv"):
            if s["phase"] != "TRANSPORT":
                continue
            a, b = float(s["truth_request_sim_time"]), float(s["truth_received_sim_time"])
            identity = (a, b)
            if identity in seen:
                continue
            seen.add(identity)
            samples.append(((a+b)/2, float(s["rod_z"])*1000, b-a))
        samples.sort()
        if samples and start is not None and end is not None:
            ts, zs, widths = map(np.array, zip(*samples))
            left, right = max(start, ts[0]), min(end, ts[-1])
            grid = np.arange(left, right+1e-9, .05)
            if len(grid) > 1:
                z = np.interp(grid, ts, zs)
                row.update(transport_height_net_mm=float(z[-1]-z[0]),
                           transport_height_range_mm=float(np.ptp(z)),
                           transport_sample_span_s=float(grid[-1]-grid[0]),
                           truth_interval_max_s=float(max(widths)),
                           transport_unique_truth_samples=len(samples))
                curves[item["sample"]] = (item["condition"], grid-grid[0], z-z[0])
        rows.append(row)
    write(OUT / "runs.csv", rows)
    metrics = ["matched_wait_sim_s", "post_lift_to_dispatch_sim_s", "transport_event_sim_s",
               "transport_height_net_mm", "transport_height_range_mm", "release_xy_error_mm",
               "release_yaw_error_deg", "truth_interval_max_s", "actual_payload_force_max_n",
               "continuous_jerk_rms_change_pct_vs_same_run", "time_profile_solve_sec"]
    summary = {}
    for condition in COLORS:
        group = [r for r in rows if r["condition"] == condition]
        stats = {}
        for key in metrics:
            values = numeric(group, key)
            stats[key] = {"n": len(values), "mean": statistics.mean(values),
                          "median": statistics.median(values), "min": min(values), "max": max(values)} if values else None
        summary[condition] = {"attempts": len(group), "completed": sum(r["state"] == "DONE" for r in group), "metrics": stats}
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    lines = ["# 相同运输前保持条件下的补充对照", "", "本批独立于80次正式主实验。四条件各3次；所有条件均计算候选，P/I0/I1仍选基础参考。运输前以抬升完成为起点设置8 s仿真窗口。物理参数、控制增益和既有保护不变。", "",
             "P=基础＋JTC；T=时间优化＋JTC；I0=基础＋阻抗、物体前馈关闭；I1=基础＋同阻抗、前馈开启。", "",
             "| 条件 | 完成/尝试 | 运输高度净变化中位数/mm | 高度范围中位数/mm | 释放位置误差均值/mm | 偏航误差均值/° |", "|---|---|---|---|---|---|"]
    def val(s, key, field):
        x=s["metrics"][key]
        return f"{x[field]:.3f}" if x else "未测"
    for c, s in summary.items():
        lines.append(f"| {c} | {s['completed']}/{s['attempts']} | {val(s,'transport_height_net_mm','median')} | {val(s,'transport_height_range_mm','median')} | {val(s,'release_xy_error_mm','mean')} | {val(s,'release_yaw_error_deg','mean')} |")
    lines += ["", "## 结果解释", "",
              "P、T各完成3/3。T选用候选的连续参考jerk RMS相对同次基础参考平均下降"
              f"{-summary['T']['metrics']['continuous_jerk_rms_change_pct_vs_same_run']['mean']:.2f}%；"
              "运输高度净变化、变化范围及释放误差未显示一致优势。跨次规划由各自实测起点生成，不能声称P/T跨运行的关节路径完全相同。",
              "I0三次在运输派发起点偏移检查中停止；I1三次在起点连接的抓持参考检查中停止。两条件均已抬升并经过8秒窗口，但没有有效运输窗口，因此没有完整运输的前馈因果比较。停止不等于已证明物体掉落；也不能把该长保持工况与原80次通常流程的完成率混合。",
              "I0实际发布的每臂物体前馈最大值为0 N，I1为3.1392 N；机器人本体补偿、阻抗增益和夹持参数未改变。"]
    lines += ["", "## 时间与采样", "", "GetBodyState响应没有物理采样时间戳，记录了请求发送与响应接收的仿真时间。本分析以二者中点估计采样时刻，并报告该区间宽度；未将其称为精确物理步。仅对实际运输阶段的不同真值样本，在覆盖范围内按50 ms插值，不外推。高度范围不是目标误差、振荡幅值或实测jerk。", "", "本轮是探索性小样本对照，不能据此估计总体可靠率或声称统计显著。正常部署中候选计算仍是算法开销，匹配保持不消除这一成本。P/I0/I1计算的候选未选用，其候选jerk变化只能描述计算结果，不能写成执行优化收益。", "", "## 逐次状态", ""]
    for r in rows:
        lines.append(f"- {r['sample']}: {r['state']}；run={r['run_id']}；{r.get('failure_reason','')}")
    (OUT / "results.md").write_text("\n".join(lines)+"\n")
    fig, axes = plt.subplots(2, 2, figsize=(8, 5.2), sharex=True, sharey=True, constrained_layout=True)
    for ax, c in zip(axes.flat, COLORS):
        for sample, (condition, t, z) in curves.items():
            if condition == c:
                ax.plot(t, z, color=COLORS[c], alpha=.75, label=sample)
        ax.axhline(0, lw=.7, color="gray")
        if ax.lines and ax.get_legend_handles_labels()[0]:
            ax.legend(frameon=False)
        else:
            ax.text(.5, .5, f"{c}：无有效运输曲线", transform=ax.transAxes,
                    ha="center", va="center")
        ax.set_xlabel("运输阶段时间 (s)")
        ax.set_ylabel("杆高度相对变化 (mm)")
    fig.savefig(OUT / "matched_transport_height.png", dpi=300)
    plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
