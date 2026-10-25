#!/usr/bin/env python3
"""Analyze the independent 20-per-group fixed-scene ABCD batch."""
import csv
import math
import re
import statistics
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_abcd_s0_today as parser
from scientific_style import GROUP_COLORS, configure_scientific_style

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "experiment_results/reports/abcd_n20_20261025"
RUNS = ROOT / "experiment_results/a1_runs"
FIG = OUT / "figures"
configure_scientific_style()


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def formal_dispatch(log):
    marker = log.find("[P3A-1] 下发轨迹")
    if marker < 0:
        return False
    return "Execute request accepted" in log[marker:]


def apply_release_wait_marker(parsed, log):
    """Use the per-run runtime marker; older parser defaults do not fit this batch."""
    match = re.search(
        r"\[ABCD_RELEASE_MEASUREMENT\] physics_wait_sec=([0-9.]+)", log
    )
    if match:
        parsed["release_wait_protocol"] = "measured_about_2.0s_simulation"
        parsed["release_wait_target_s"] = "2.000"
        parsed["release_wait_actual_s"] = match.group(1)
        parsed["release_wait_source"] = "task_launch_ABCD_RELEASE_MEASUREMENT"
    else:
        parsed["release_wait_protocol"] = ""
        parsed["release_wait_target_s"] = ""
        parsed["release_wait_actual_s"] = ""
        parsed["release_wait_source"] = ""
    return parsed


def classify_startup(log, wrapper, run_id, wrapper_exit):
    if "E_P2B_PLAN_FAILED" in log:
        return "pregrasp_PTP_plan_refused_before_goal"
    if "MMC_ACTIVE_VERIFY" in log and "request_response_timeout" in log:
        return "controller_list_response_timeout_before_goal"
    if "E_P3A_ARM_CONTROLLER_NOT_ACTIVE" in log:
        return "arm_controller_not_active_before_goal"
    if "clock/source observation failed" in wrapper:
        return "joint_state_clock_source_unavailable_before_motion"
    if "E_SIM_INFO_UNAVAILABLE" in (wrapper + log):
        return "get_sim_info_unavailable_before_truth"
    if "E_INSTANCE_ALREADY_PRESENT" in (wrapper + log) or "already answers on ROS_DOMAIN_ID" in wrapper:
        return "existing_instance_mutex_block"
    if "Operation not permitted" in wrapper or "All whitelist interfaces were filtered out" in wrapper:
        return "DDS_local_transport_permission"
    if "PREPARED" in log and "P2A_COLLISION" in log:
        return "static_collision_reject_before_goal"
    if wrapper_exit == "130" or run_id.endswith("C07") or "runner exited before publishing truth" in wrapper:
        return "batch_interruption_before_task"
    if not log:
        return "task_not_started"
    return "pregrasp_not_dispatched; inspect run log"


def formal_index():
    schedule = read_csv(OUT / "schedule.csv")
    ledger = read_csv(OUT / "attempts.csv")
    replacement_path = OUT / "replacement_map.csv"
    replacements = {r["sample"]: r for r in read_csv(replacement_path)} if replacement_path.exists() else {}
    by_sample = {}
    for row in ledger:
        by_sample.setdefault(row["sample"], []).append(row)

    formal = []
    startup = []
    superseded = []
    for slot in schedule:
        sample, group = slot["sample"], slot["group"]
        found = None
        sample_runs = by_sample.get(sample, [])
        replacement = replacements.get(sample)
        if replacement:
            original = next((r for r in sample_runs if r["run_id"] == replacement["original_run_id"]), None)
            selected = next((r for r in sample_runs if r["run_id"] == replacement["replacement_run_id"]), None)
            if original is None or selected is None:
                raise RuntimeError(f"Replacement ledger rows missing for {sample}")
            original_path = RUNS / original["run_id"]
            old = parser.parse_run(group, original["run_id"])
            old = apply_release_wait_marker(old, (original_path / "task_launch.log").read_text(errors="replace"))
            old.pop("_trace", None); old.pop("_release_center", None)
            old.update(round=slot["round"], sample=sample, group=group,
                       run_dir=str(original_path.relative_to(ROOT)), wrapper_exit=original["wrapper_exit"],
                       failure_stage="post-release truth/feedback validation; retreat not dispatched",
                       analysis_disposition="superseded by successful replacement", replacement_run_id=selected["run_id"])
            superseded.append(old)
            path = RUNS / selected["run_id"]
            log = (path / "task_launch.log").read_text(errors="replace") if (path / "task_launch.log").exists() else ""
            if not formal_dispatch(log):
                raise RuntimeError(f"Replacement did not reach formal start for {sample}")
            found = (selected, path, log)
            skipped_ids = {original["run_id"], selected["run_id"]}
        else:
            skipped_ids = set()
        for run in sample_runs:
            if run["run_id"] in skipped_ids:
                continue
            path = RUNS / run["run_id"]
            log = (path / "task_launch.log").read_text(errors="replace") if (path / "task_launch.log").exists() else ""
            wrapper_path = OUT / f"{run['run_id']}_wrapper.log"
            wrapper = wrapper_path.read_text(errors="replace") if wrapper_path.exists() else ""
            if formal_dispatch(log):
                found = (run, path, log)
                break
            startup.append({
                "round": slot["round"], "sample": sample, "group": group,
                "run_id": run["run_id"], "wrapper_exit": run["wrapper_exit"],
                "classification": classify_startup(log, wrapper, run["run_id"], run["wrapper_exit"]),
                "detail": next((line.strip() for line in reversed((wrapper + "\n" + log).splitlines())
                                if any(key in line for key in ("E_", "planning returned error", "Operation not permitted"))), ""),
                "run_dir": str(path.relative_to(ROOT)),
            })
        if found is None:
            raise RuntimeError(f"No formal pre-grasp dispatch found for {sample}")
        run, path, log = found
        parsed = parser.parse_run(group, run["run_id"])
        parsed = apply_release_wait_marker(parsed, log)
        parsed.pop("_trace", None)
        parsed.pop("_release_center", None)
        failure_stage = ""
        failure_evidence = ""
        if parsed["state"] != "DONE":
            if "[ABCD_RELEASE_MEASUREMENT]" in log:
                failure_stage = "post-release truth/feedback verification after measurement wait"
            elif "[P3A-1]" in log:
                failure_stage = "after first pre-grasp goal dispatch; inspect task log"
            diagnostics = [line.strip() for line in log.splitlines() if "[CONTACT_DIAGNOSTIC]" in line]
            if diagnostics:
                failure_evidence = "; ".join(diagnostics[-2:]).replace("|", "/")
        parsed.update(round=slot["round"], sample=sample, group=group,
                      run_dir=str(path.relative_to(ROOT)), wrapper_exit=run["wrapper_exit"],
                      formal_start_marker="P3A-1 first MoveGroup Execute request accepted",
                      failure_stage=failure_stage, failure_evidence=failure_evidence)
        if replacement:
            parsed.update(replaces_run_id=replacement["original_run_id"],
                          analysis_role="successful replacement selected per user clarification")
        formal.append(parsed)
    write_csv(OUT / "formal_attempts.csv", formal)
    write_csv(OUT / "startup_failures.csv", startup)
    superseded_path = OUT / "superseded_formal_attempts.csv"
    if replacements:
        write_csv(superseded_path, superseded)
    elif superseded_path.exists():
        superseded_path.unlink()
    run_index = [dict(sample=r["sample"], group=r["group"], run_id=r["run_id"],
                      classification="formal_attempt_replacement" if r.get("analysis_role") else "formal_attempt",
                      result=r["state"], run_dir=r["run_dir"])
                 for r in formal]
    run_index += [dict(sample=r["sample"], group=r["group"], run_id=r["run_id"],
                       classification="superseded_formal_attempt", result=r["state"], run_dir=r["run_dir"])
                  for r in superseded]
    run_index += [dict(sample=r["sample"], group=r["group"], run_id=r["run_id"],
                       classification=r["classification"], result="startup_exception", run_dir=r["run_dir"])
                  for r in startup]
    indexed_ids = {r["run_id"] for r in run_index}
    for run in ledger:
        if run["run_id"] in indexed_ids:
            continue
        log_path = RUNS / run["run_id"] / "task_launch.log"
        log = log_path.read_text(errors="replace") if log_path.exists() else ""
        result_line = next((line for line in reversed(log.splitlines()) if "[CONTACT_PROBE_RESULT]" in line), "")
        result = result_line.split("state=", 1)[1].split()[0] if "state=" in result_line else "no_task_result"
        run_index.append(dict(sample=run["sample"], group=run["group"], run_id=run["run_id"],
                              classification=run["kind"], result=result, run_dir=run["run_dir"]))
        indexed_ids.add(run["run_id"])
    write_csv(OUT / "raw_run_index.csv", run_index)
    return formal, startup


def hand_yaw(qx, qy, qz, qw):
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def representative_and_trace(row):
    rows = parser.read_csv(RUNS / row["run_id"] / "contact_probe.csv")
    carry = [r for r in rows if r.get("phase") in {"LIFT", "HOLD", "TRANSPORT", "ROTATE", "LOWER"}]
    if not carry:
        return [], {}
    t0 = min((float(r["left_sim_time"]) + float(r["right_sim_time"])) / 2 for r in carry)
    trace = []
    for r in carry:
        t = (float(r["left_sim_time"]) + float(r["right_sim_time"])) / 2 - t0
        trace.append({
            "t": t, "rod_z_mm": float(r["rod_z"]) * 1000,
            "rod_x_mm": float(r["rod_x"]) * 1000, "rod_y_mm": float(r["rod_y"]) * 1000,
            "left_hand_yaw_deg": hand_yaw(*(float(r[f"left_hand_{k}"]) for k in ("qx", "qy", "qz", "qw"))),
            "phase": r["phase"],
        })
    summary = {
        "rod_height_range_mm": max(r["rod_z_mm"] for r in trace) - min(r["rod_z_mm"] for r in trace),
        "rod_x_range_mm": max(r["rod_x_mm"] for r in trace) - min(r["rod_x_mm"] for r in trace),
        "rod_y_range_mm": max(r["rod_y_mm"] for r in trace) - min(r["rod_y_mm"] for r in trace),
        "left_hand_yaw_change_range_deg": max(r["left_hand_yaw_deg"] for r in trace) - min(r["left_hand_yaw_deg"] for r in trace),
    }
    return trace, summary


def num(row, key):
    try:
        v = row.get(key, "")
        return float(v) if v not in ("", None) else None
    except (TypeError, ValueError):
        return None


def stats(rows, key):
    values = [num(r, key) for r in rows]
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"n": 0, "mean": "", "sd": "", "median": "", "min": "", "max": ""}
    return {"n": len(values), "mean": statistics.mean(values),
            "sd": statistics.stdev(values) if len(values) > 1 else 0.0,
            "median": statistics.median(values), "min": min(values), "max": max(values)}


def summary_rows(rows):
    fields = [
        "release_wait_actual_s", "release_xy_error_mm", "release_yaw_error_deg", "transport_plan_s",
        "transport_observed_s", "time_profile_solve_sec", "continuous_jerk_rms_change_pct_vs_same_run",
        "continuous_jerk_peak_change_pct_vs_same_run", "max_grasp_reference_residual_mm",
        "max_grasp_reference_angle_deg", "rod_height_range_mm", "rod_x_range_mm",
        "rod_y_range_mm", "left_hand_yaw_change_range_deg",
    ]
    result = []
    for group in "ABCD":
        group_rows = [r for r in rows if r["group"] == group]
        completed = [r for r in group_rows if r["state"] == "DONE"]
        item = {"group": group, "formal_attempts": len(group_rows), "completed": len(completed),
                "completion_rate": len(completed) / len(group_rows) if group_rows else ""}
        for field in fields:
            subset = completed if field in {"release_xy_error_mm", "release_yaw_error_deg"} else group_rows
            s = stats(subset, field)
            for k, v in s.items():
                item[f"{field}_{k}"] = v
        item["transport_time_definition"] = "JTC MoveGroup wall time" if group in "AB" else "MMC reference playback time"
        item["failures"] = "; ".join(f"{r['sample']}:{r['failure_reason']}" for r in group_rows if r["state"] != "DONE")
        result.append(item)
    write_csv(OUT / "group_summary.csv", result)
    return result


def plot_figures(rows, representatives):
    FIG.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.labelsize": 9, "legend.fontsize": 8,
                         "axes.unicode_minus": False, "svg.fonttype": "none"})
    groups = list("ABCD")

    fig, ax = plt.subplots(figsize=(6.2, 3.6), layout="constrained")
    done_counts = []
    for i, group in enumerate(groups):
        subset = [r for r in rows if r["group"] == group]
        done = sum(r["state"] == "DONE" for r in subset)
        done_counts.append(done)
        ax.bar(i, done, width=.55, color=GROUP_COLORS[group])
        ax.text(i, done + .25, f"{done}/20", ha="center", va="bottom")
    ax.set(xticks=range(4), xticklabels=groups, xlabel="Group", ylabel="Complete full tasks", ylim=(0, 22))
    ax.set_yticks(range(0, 21, 5)); ax.grid(axis="y", alpha=.22)
    fig.savefig(FIG / "01_completion.png", dpi=300); fig.savefig(FIG / "01_completion.svg"); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.6), layout="constrained")
    for ax, key, ylabel in ((axes[0], "release_xy_error_mm", "Release position error (mm)"),
                            (axes[1], "release_yaw_error_deg", "Release yaw error (deg)")):
        data = [[num(r, key) for r in rows if r["group"] == g and r["state"] == "DONE" and num(r, key) is not None]
                for g in groups]
        bp = ax.boxplot(data, positions=range(4), widths=.48, patch_artist=True, showfliers=False)
        for patch, group in zip(bp["boxes"], groups):
            patch.set_facecolor(GROUP_COLORS[group]); patch.set_alpha(.22)
        for i, (group, values) in enumerate(zip(groups, data)):
            if values:
                jitter = np.linspace(-.10, .10, len(values)) if len(values) > 1 else np.array([0.0])
                ax.scatter(i + jitter, values, color=GROUP_COLORS[group], s=18, zorder=3)
        ns = [len(values) for values in data]
        ax.set(xticks=range(4), xticklabels=[f"{g}\nn={n}" for g, n in zip(groups, ns)], xlabel="Group", ylabel=ylabel)
        ax.set_ylim(bottom=0)
        ax.grid(axis="y", alpha=.22)
    fig.savefig(FIG / "02_release_errors.png", dpi=300); fig.savefig(FIG / "02_release_errors.svg"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.4, 3.8), layout="constrained")
    for i, group in enumerate(groups):
        field = "transport_action_wall_s" if group in "AB" else "transport_reference_playback_s"
        values = [num(r, field) for r in rows if r["group"] == group and num(r, field) is not None]
        if not values:
            continue
        jitter = np.linspace(-.10, .10, len(values)) if len(values) > 1 else np.array([0.0])
        ax.scatter(i + jitter, values, color=GROUP_COLORS[group], s=20)
        ax.hlines(statistics.median(values), i - .18, i + .18, color=GROUP_COLORS[group], lw=1.5)
    time_ns = [sum(num(r, "transport_action_wall_s" if g in "AB" else "transport_reference_playback_s") is not None
                   for r in rows if r["group"] == g) for g in groups]
    ax.set(xticks=range(4), xticklabels=[f"{g}\nn={n}" for g, n in zip(groups, time_ns)], xlabel="Group (A/B: JTC action wall time; C/D: MMC reference playback)",
           ylabel="Transport phase time (s)")
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", alpha=.22)
    fig.savefig(FIG / "03_transport_time.png", dpi=300); fig.savefig(FIG / "03_transport_time.svg"); plt.close(fig)

    opt = [r for r in rows if r["group"] in "BD" and num(r, "continuous_jerk_rms_change_pct_vs_same_run") is not None]
    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.6), layout="constrained")
    for ax, key, label in ((axes[0], "continuous_jerk_rms_change_pct_vs_same_run", "Continuous reference jerk RMS change (%)"),
                           (axes[1], "continuous_jerk_peak_change_pct_vs_same_run", "Continuous reference jerk peak change (%)")):
        for i, group in enumerate("BD"):
            values = [num(r, key) for r in opt if r["group"] == group and num(r, key) is not None]
            if values:
                jitter = np.linspace(-.10, .10, len(values)) if len(values) > 1 else np.array([0.0])
                ax.scatter(i + jitter, values, color=GROUP_COLORS[group], s=20)
                ax.hlines(statistics.mean(values), i - .17, i + .17, color=GROUP_COLORS[group], lw=1.5)
        ax.axhline(0, color="black", lw=.8, alpha=.6)
        ax.set(xticks=range(2), xticklabels=[f"{g}\nn={sum(r['group'] == g and num(r, key) is not None for r in opt)}" for g in "BD"], xlabel="Optimized group", ylabel=label)
        ax.grid(axis="y", alpha=.22)
    fig.savefig(FIG / "04_reference_jerk_change.png", dpi=300); fig.savefig(FIG / "04_reference_jerk_change.svg"); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.8), layout="constrained")
    for group in groups:
        row = representatives[group]
        trace = row["_trace_data"]
        if not trace:
            continue
        t = [v["t"] for v in trace]
        axes[0].plot(t, [v["rod_z_mm"] for v in trace], color=GROUP_COLORS[group], label=group)
        yaw0 = trace[0]["left_hand_yaw_deg"]
        axes[1].plot(t, [((v["left_hand_yaw_deg"] - yaw0 + 180) % 360) - 180 for v in trace],
                     color=GROUP_COLORS[group], label=group)
    axes[0].set(xlabel="Carry time (s)", ylabel="Rod center height (mm)")
    axes[1].set(xlabel="Carry time (s)", ylabel="Left hand yaw change (deg; posture proxy)")
    for ax in axes:
        ax.grid(alpha=.2); ax.legend(frameon=False)
    fig.savefig(FIG / "05_representative_motion.png", dpi=300); fig.savefig(FIG / "05_representative_motion.svg"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 3.5), layout="constrained")
    for group in groups:
        trace = representatives[group]["_trace_data"]
        if trace:
            ax.plot([v["rod_x_mm"] for v in trace], [v["rod_y_mm"] for v in trace],
                    color=GROUP_COLORS[group], label=group)
    ax.set(xlabel="Rod X (mm)", ylabel="Rod Y (mm)", aspect="equal")
    ax.grid(alpha=.2); ax.legend(frameon=False, loc="lower center", bbox_to_anchor=(.5, 1.02), ncol=4)
    fig.savefig(FIG / "06_representative_rod_xy.png", dpi=300); fig.savefig(FIG / "06_representative_rod_xy.svg"); plt.close(fig)


def choose_representatives(rows):
    result, rep_rows = {}, []
    for group in "ABCD":
        candidates = [r for r in rows if r["group"] == group and r["state"] == "DONE"]
        fields = ["release_xy_error_mm", "release_yaw_error_deg",
                  "transport_action_wall_s" if group in "AB" else "transport_reference_playback_s"]
        medians = {key: statistics.median([num(r, key) for r in candidates if num(r, key) is not None])
                   for key in fields if any(num(r, key) is not None for r in candidates)}
        scales = {key: max([abs(num(r, key) - medians[key]) for r in candidates if num(r, key) is not None] or [1.0])
                  for key in medians}
        def distance(r):
            terms = [((num(r, key) - median) / scales[key]) ** 2 for key, median in medians.items()
                     if num(r, key) is not None]
            return sum(terms) / len(terms) if terms else 0
        selected = min(candidates, key=distance)
        trace, trace_summary = representative_and_trace(selected)
        item = dict(group=group, sample=selected["sample"], run_id=selected["run_id"],
                    criterion="nearest multimetric group median among completed runs", **trace_summary)
        rep_rows.append(item)
        result[group] = {"_trace_data": trace, **item}
        selected["rod_height_range_mm"] = trace_summary.get("rod_height_range_mm", "")
    write_csv(OUT / "representative_runs.csv", rep_rows)
    return result


def write_results(rows, startup, summaries, representatives):
    lines = ["# 固定场景 ABCD 扩大样本对照实验结果", "",
             "本批次独立统计；不合并历史每组 3 次样本。固定场景为 `(0.50, 0, +10°) → (0.40, 0, +20°)`。每组 20 次正式尝试，正式起点定义为首次预抓取轨迹的 MoveGroup 执行请求被接受。动作前规划或启动异常保留在 `startup_failures.csv`，不进入 20 次正式分母。", "",
             "A 为基础轨迹＋JTC；B 为时间优化轨迹＋JTC；C 为基础轨迹＋外部笛卡尔阻抗及载荷前馈；D 为时间优化轨迹＋同一阻抗及前馈。C/D 组合结果不单独归因于阻抗。所有 jerk 数值均来自连续时间参考曲线，不是实测机械臂 jerk。", "",
             "## 完成情况", "", "| 组别 | 完整任务 | 正式尝试 | 完成率 |", "|---|---:|---:|---:|"]
    for s in summaries:
        lines.append(f"| {s['group']} | {s['completed']}/20 | {s['formal_attempts']} | {s['completed']/20:.1%} |")
    lines += ["", "## 连续指标描述统计", "", "释放后四组使用相同的 2.0 s 仿真时间测量等待；80 次正式运行日志均记录了 2.000–2.062 s 的实际等待。均值、样本标准差、中位数及范围按实际有值的正式运行计算。释放位置与偏航误差仅统计完整释放测量任务；缺失值未补零。JTC 实际运输阶段为 MoveGroup wall-clock；MMC 为参考播放仿真时间，两者定义不同，不能直接等同。运输时间图显示逐次数据，横线为中位数。", "",
              "| 组别 | 指标 | n | 均值 | SD | 中位数 | 范围 |", "|---|---|---:|---:|---:|---:|---:|"]
    metrics = [
        ("release_xy_error_mm", "释放位置误差 (mm)", True),
        ("release_yaw_error_deg", "释放角度误差 (°)", True),
        ("transport_plan_s", "运输计划时长 (s)", False),
        ("transport_observed_s", "实际运输时间 (s)", False),
        ("time_profile_solve_sec", "时间优化计算耗时 (s)", False),
        ("continuous_jerk_rms_change_pct_vs_same_run", "连续参考 jerk RMS 相对自身基线变化 (%)", False),
        ("continuous_jerk_peak_change_pct_vs_same_run", "连续参考 jerk 峰值相对自身基线变化 (%)", False),
        ("max_grasp_reference_residual_mm", "承载阶段抓取位置参考残差 (mm)", False),
        ("max_grasp_reference_angle_deg", "承载阶段抓取姿态参考残差 (°)", False),
        ("rod_height_range_mm", "承载阶段杆中心高度变化范围 (mm)", False),
        ("rod_x_range_mm", "承载阶段杆中心 X 变化范围 (mm)", False),
        ("rod_y_range_mm", "承载阶段杆中心 Y 变化范围 (mm)", False),
        ("left_hand_yaw_change_range_deg", "承载阶段左手偏航姿态代理变化范围 (°)", False),
    ]
    for group in "ABCD":
        group_rows = [r for r in rows if r["group"] == group]
        completed = [r for r in group_rows if r["state"] == "DONE"]
        for key, label, done_only in metrics:
            vals = stats(completed if done_only else group_rows, key)
            if vals["n"] == 0:
                continue
            lines.append(f"| {group} | {label} | {vals['n']} | {vals['mean']:.4f} | {vals['sd']:.4f} | {vals['median']:.4f} | [{vals['min']:.4f}, {vals['max']:.4f}] |")
    lines += ["", "## 未完成正式尝试", "", "| 样本 | 组别 | 最后阶段 | 直接原因 |", "|---|---|---|---|"]
    failed = [r for r in rows if r["state"] != "DONE"]
    if failed:
        for r in failed:
            evidence = r.get("failure_evidence", "") or r.get("failure_reason", "") or r["state"]
            lines.append(f"| {r['sample']} | {r['group']} | {r.get('failure_stage', '')} | {evidence} |")
    else:
        lines.append("| 无 | — | — | — |")
    startup_counts = Counter(r["classification"] for r in startup)
    lines += ["", "## 动作前异常", "", f"共记录 {len(startup)} 条动作前异常运行；均保留独立目录，不占正式尝试分母。逐条运行号、退出码、日志摘录及目录见 `startup_failures.csv`；正式和异常原始目录索引见 `raw_run_index.csv`。", "",
              "| 异常分类 | 次数 |", "|---|---:|"]
    for classification, count in sorted(startup_counts.items()):
        lines.append(f"| {classification} | {count} |")
    replacement_path = OUT / "replacement_map.csv"
    superseded_path = OUT / "superseded_formal_attempts.csv"
    if replacement_path.exists():
        replacement_rows = read_csv(replacement_path)
        lines += ["", "## C04 替补与原始记录", "", "原始 C04 运行完成抓取、运输、下降和开爪，但在统一释放 truth/feedback 检查处退出，未下发撤退；该原始失败保留在 `superseded_formal_attempts.csv`。随后新增运行已完成释放和撤退，按用户澄清的完整物体搬运完成口径，作为 C04 样本纳入主比较，C 组为 20/20。总计有81条达到正式起点的动作运行记录：原 C04 失败仍保留，但由成功替补用于80样本主比较。这项替换不表示已证明原错误属于随机因素。外层 wrapper 在任务日志 DONE 后、清理阶段收到 SIGINT，故其退出码为130。替补映射见 `replacement_map.csv`。"]
        for repl in replacement_rows:
            lines.append(f"- `{repl['sample']}`：纳入 `{repl['replacement_run_id']}`（state=DONE），替代原运行 `{repl['original_run_id']}`。")
    lines += ["", "## 各组比较", ""]
    for group in "ABCD":
        summary = next(s for s in summaries if s["group"] == group)
        lines.append(f"- {group}：{summary['completed']}/20 完整任务；{summary['transport_time_definition']}。落点与连续指标见逐次 CSV。")
    summaries_by_group = {s["group"]: s for s in summaries}
    def mean_text(group, key, precision=2):
        value = summaries_by_group[group].get(f"{key}_mean", "")
        return f"{value:.{precision}f}" if isinstance(value, (float, int)) else "—"
    lines += ["", "## 观察到的差异", "",
              f"在本固定场景下，B 相对 A 的释放位置误差均值为 {mean_text('B', 'release_xy_error_mm')} mm 与 {mean_text('A', 'release_xy_error_mm')} mm，偏航误差均值为 {mean_text('B', 'release_yaw_error_deg')}° 与 {mean_text('A', 'release_yaw_error_deg')}°；描述统计未显示时间优化改善落点精度。B 与 A 的计划运输时长均值接近（{mean_text('B', 'transport_plan_s')} s 与 {mean_text('A', 'transport_plan_s')} s），B 的优化计算平均耗时 {mean_text('B', 'time_profile_solve_sec', 3)} s。B 的候选连续参考 jerk RMS 均值较自身基础参考低 {abs(summaries_by_group['B']['continuous_jerk_rms_change_pct_vs_same_run_mean']):.2f}%。",
              f"D 相对 C 的释放位置误差均值为 {mean_text('D', 'release_xy_error_mm')} mm 与 {mean_text('C', 'release_xy_error_mm')} mm，偏航误差均值为 {mean_text('D', 'release_yaw_error_deg')}° 与 {mean_text('C', 'release_yaw_error_deg')}°；描述统计未显示优化后放置精度改善。D 的候选连续参考 jerk RMS/峰值均值相对自身基础参考变化为 {mean_text('D', 'continuous_jerk_rms_change_pct_vs_same_run')}% / {mean_text('D', 'continuous_jerk_peak_change_pct_vs_same_run')}%，两项平均均下降；优化计算平均耗时 {mean_text('D', 'time_profile_solve_sec', 3)} s。",
              f"C/D 的释放位置误差均值为 {mean_text('C', 'release_xy_error_mm')}–{mean_text('D', 'release_xy_error_mm')} mm，A/B 为 {mean_text('A', 'release_xy_error_mm')}–{mean_text('B', 'release_xy_error_mm')} mm；抓取参考位置残差在 C/D 也较高（{mean_text('C', 'max_grasp_reference_residual_mm')}–{mean_text('D', 'max_grasp_reference_residual_mm')} mm，A/B 为 {mean_text('A', 'max_grasp_reference_residual_mm')}–{mean_text('B', 'max_grasp_reference_residual_mm')} mm）。C/D 同时改变了控制方式并启用载荷前馈，因此不能将差异单独归因于阻抗。C04 原始运行在释放 truth/feedback 检查阶段未完成，按未完成正式尝试计入 C 组。",
              "A/B 实测运输时间采用 JTC MoveGroup wall-clock，C/D 采用 MMC 参考播放时间；测量定义不同，不能作直接组间速度比较。C/D 播放时间数据呈高低两个时间簇，均值不代表单一集中区。A/B 的实际 wall-clock 均值略高于计划运输时长均值；这批数据没有显示时间优化带来一致的运输时长缩短。",
              "每组正式分母 n=20，以描述统计为主，不作显著性推断。jerk 为连续时间参考曲线指标，不是实测机械臂 jerk；结论范围限于该固定起终场景。C/D 杆姿态曲线使用左手 yaw 作为姿态代理，contact_probe.csv 未记录杆四元数时间序列。", "",
              "## 图表与复现", "", "- `figures/01_completion.png`：完成率。", "- `figures/02_release_errors.png`：逐次释放位置/角度误差及分布。", "- `figures/03_transport_time.png`：逐次运输阶段时间，横线为中位数；区分 JTC wall time 与 MMC 参考播放时间。", "- `figures/04_reference_jerk_change.png`：B/D 候选相对自身基线的连续参考 jerk 变化。", "- `figures/05_representative_motion.png`：组内多指标接近中位数的代表运行，包含杆高度和左手 yaw 姿态代理曲线。", "- `figures/06_representative_rod_xy.png`：同一代表运行的杆中心平面轨迹。", "- `formal_attempts.csv`：80 次正式尝试逐次数据；`startup_failures.csv`：动作前异常；`raw_run_index.csv`：正式、启动异常及后补未计入运行的目录索引；`group_summary.csv`：组统计。", "- [统计核对与指标说明](统计核对与指标说明.md)；[结题报告实验结果分析](结题报告实验结果分析.md)。", "- 分析命令：`python3 scripts/analyze_abcd_n20_20261025.py`。", "- 续跑命令：`bash scripts/run_abcd_n20_20261025.sh`。Word 报告未修改。"]
    (OUT / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows, startup = formal_index()
    for row in rows:
        group = row["group"]
        observed = row.get("transport_action_wall_s") if group in "AB" else row.get("transport_reference_playback_s")
        row["transport_observed_s"] = observed
        base = num(row, "baseline_continuous_jerk_max")
        cand = num(row, "candidate_continuous_jerk_max")
        row["continuous_jerk_peak_change_pct_vs_same_run"] = 100 * (cand / base - 1) if base and cand is not None else ""
        _, trace_summary = representative_and_trace(row)
        for key, value in trace_summary.items():
            row[key] = value
    representatives = choose_representatives(rows)
    write_csv(OUT / "formal_attempts.csv", rows)
    summaries = summary_rows(rows)
    plot_figures(rows, representatives)
    write_results(rows, startup, summaries, representatives)
    print(f"formal={len(rows)} startup_exceptions={len(startup)}")
    for s in summaries:
        print(f"{s['group']}: DONE {s['completed']}/20 formal={s['formal_attempts']}")


if __name__ == "__main__":
    main()
