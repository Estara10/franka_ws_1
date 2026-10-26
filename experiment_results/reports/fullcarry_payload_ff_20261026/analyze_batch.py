#!/usr/bin/env python3
"""Analyze full-carry payload feedforward runs using actual sim-time coverage."""
from __future__ import annotations

import csv
import math
import os
import re
import statistics
from bisect import bisect_right
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
RUNS = ROOT / "experiment_results/a1_runs"
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplconfig-fullcarry-payload-ff-20261026")


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields=None):
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def fnum(row, key, default=float("nan")):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else default
    except (KeyError, ValueError, TypeError):
        return default


def interp(xs, ys, t):
    if not xs or t < xs[0] - 1e-10 or t > xs[-1] + 1e-10:
        return None
    if t <= xs[0]:
        return ys[0]
    if t >= xs[-1]:
        return ys[-1]
    i = bisect_right(xs, t)
    t0, t1 = xs[i - 1], xs[i]
    if t1 <= t0:
        return ys[i]
    alpha = (t - t0) / (t1 - t0)
    return ys[i - 1] + alpha * (ys[i] - ys[i - 1])


def parse_events(text):
    start = re.search(r"\[TRANSPORT_EVENT\] event=start sim_time=([0-9.eE+-]+)", text)
    end = re.search(r"\[TRANSPORT_EVENT\] event=end sim_time=([0-9.eE+-]+)", text)
    anchor = re.search(r"\[ABCD_PAYLOAD_RAMP_ANCHOR\].*sim_time=([0-9.eE+-]+)", text)
    active = re.search(r"\[ABCD_MMC_ACTIVE\].*payload_enabled=([01])", text)
    release = re.search(r"\[CONTACT_PROBE_RELEASED\] xyz=\(([^,]+),([^,]+),([^\)]+)\)", text)
    result = re.search(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+) detail=([^\r\n]+)", text)
    def val(m, i=1):
        return float(m.group(i)) if m else float("nan")
    return {
        "transport_start_s": val(start), "transport_end_s": val(end),
        "transport_actual_duration_s": val(end) - val(start),
        "payload_anchor_s": val(anchor), "payload_enabled": int(active.group(1)) if active else -1,
        "release_x_m": val(release, 1), "release_y_m": val(release, 2), "release_z_m": val(release, 3),
        "task_state": result.group(1) if result else "", "task_detail": result.group(2) if result else "",
    }


def load_run(row):
    run_id = row["run_id"]
    path = RUNS / run_id
    log = (path / "task_launch.log").read_text(errors="replace")
    events = parse_events(log)
    contact = read_csv(path / "contact_probe.csv")
    ref_all = read_csv(path / "abcd_reference.csv")
    refs = [r for r in ref_all if r["phase"] == "TRANSPORT"]
    refs.sort(key=lambda r: fnum(r, "sim_time_s"))
    forces = read_csv(path / "abcd_payload_trace.csv")
    meta = {r["key"]: r["value"] for r in read_csv(path / "abcd_reference_meta.csv")}

    samples = []
    for r in contact:
        request = fnum(r, "truth_request_sim_time")
        response = fnum(r, "truth_received_sim_time")
        z = fnum(r, "rod_z")
        if math.isfinite(request) and math.isfinite(response) and response >= request and math.isfinite(z):
            samples.append({"t": 0.5 * (request + response), "z": z, "width": response - request,
                            "x": fnum(r, "rod_x"), "y": fnum(r, "rod_y"),
                            "qx": fnum(r, "rod_qx"), "qy": fnum(r, "rod_qy"),
                            "qz": fnum(r, "rod_qz"), "qw": fnum(r, "rod_qw"), "phase": r["phase"]})
    samples.sort(key=lambda r: r["t"])
    dedup = {}
    for r in samples:
        dedup[r["t"]] = r
    samples = [dedup[t] for t in sorted(dedup)]
    tx, tz = [r["t"] for r in samples], [r["z"] for r in samples]
    tr = [r for r in samples if events["transport_start_s"] <= r["t"] <= events["transport_end_s"]]
    nominal_z = fnum(refs[0], "common_nominal_z_m") if refs else fnum(meta, "common_nominal_transport_z_m")
    t0, t1 = events["transport_start_s"], events["transport_end_s"]
    grid_end = min(t1, tx[-1] if tx else float("nan"))
    if not tx or not refs or not (math.isfinite(t0) and math.isfinite(t1)) or t0 < tx[0] or t0 > tx[-1]:
        grid = []
    else:
        # Contact rows are ~10 ms; use a fixed 20 ms grid to keep alignment independent of row count.
        n = max(1, int(math.floor((grid_end - t0) / 0.02)))
        grid = [t0 + 0.02 * i for i in range(n + 1)]
        if grid[-1] < grid_end - 1e-8:
            grid.append(grid_end)
    curve = []
    e0 = interp(tx, tz, t0) - nominal_z if grid else float("nan")
    e_values = []
    de_values = []
    for t in grid:
        z = interp(tx, tz, t)
        e = z - nominal_z
        de = e - e0
        curve.append({"run_id": run_id, "condition": row["condition"], "sim_time_s": t,
                      "elapsed_from_transport_start_s": t - t0, "actual_rod_z_m": z,
                      "common_nominal_z_m": nominal_z, "e_z_common_mm": 1000 * e,
                      "delta_e_z_mm": 1000 * de})
        e_values.append(e)
        de_values.append(de)

    rx = [fnum(r, "sim_time_s") for r in refs]
    rz = [fnum(r, "rod_ref_z_m") for r in refs]
    adopted_grid = [t for t in grid if rx and rx[0] <= t <= rx[-1]]
    adopted_errors = []
    reference_offsets = []
    for t in adopted_grid:
        actual = interp(tx, tz, t)
        adopted = interp(rx, rz, t)
        adopted_errors.append(actual - adopted)
        reference_offsets.append(adopted - nominal_z)
    plan_durations = sorted({round(fnum(r, "plan_duration_s"), 9) for r in refs})
    interval_widths = [r["width"] for r in tr]
    forces_sorted = sorted(forces, key=lambda r: fnum(r, "sim_time_s"))
    force_times = [fnum(r, "sim_time_s") for r in forces_sorted]
    ff_at_transport = [r for r in forces_sorted if t0 <= fnum(r, "sim_time_s") <= t1]
    force_t0 = None
    if ff_at_transport:
        nearest = min(ff_at_transport, key=lambda r: abs(fnum(r, "sim_time_s") - t0))
        force_t0 = fnum(nearest, "left_published_ff_z_n")
    force_peak = max((fnum(r, "left_published_ff_z_n") for r in ff_at_transport), default=float("nan"))
    task_complete = int(events["task_state"] == "DONE" and "[CONTACT_PROBE_RETREAT]" in log)

    target_x, target_y = fnum(row, "target_x_m"), fnum(row, "target_y_m")
    release_error_xy = (math.hypot(events["release_x_m"] - target_x, events["release_y_m"] - target_y)
                        if math.isfinite(events["release_x_m"]) else float("nan"))
    def rms(vals):
        return math.sqrt(sum(x * x for x in vals) / len(vals)) if vals else float("nan")
    metrics = {
        "phase": row["phase"], "sequence": row["sequence"], "block": row["block"],
        "condition": row["condition"], "run_id": run_id, "task_complete": task_complete,
        "task_state": events["task_state"], "direct_reason": row.get("direct_reason", ""),
        "transport_start_s": t0, "transport_end_s": t1,
        "transport_actual_duration_s": events["transport_actual_duration_s"],
        "transport_plan_duration_s": plan_durations[-1] if plan_durations else float("nan"),
        "common_nominal_z_m": nominal_z, "e_z_start_mm": 1000 * e0,
        "e_z_mean_mm": 1000 * statistics.mean(e_values) if e_values else float("nan"),
        "e_z_rms_mm": 1000 * rms(e_values), "e_z_end_mm": 1000 * e_values[-1] if e_values else float("nan"),
        "delta_e_z_rms_mm": 1000 * rms(de_values),
        "delta_e_z_end_mm": 1000 * de_values[-1] if de_values else float("nan"),
        "e_z_sample_count": len(e_values), "adopted_ref_sample_count": len(adopted_errors),
        "adopted_ref_error_rms_mm": 1000 * rms(adopted_errors),
        "adopted_ref_offset_start_mm": 1000 * (rz[0] - nominal_z) if rz else float("nan"),
        "adopted_ref_offset_end_mm": 1000 * (rz[-1] - nominal_z) if rz else float("nan"),
        "ref_time_gap_at_transport_start_ms": 1000 * (rx[0] - t0) if rx else float("nan"),
        "truth_interval_median_ms": 1000 * statistics.median(interval_widths) if interval_widths else float("nan"),
        "truth_interval_max_ms": 1000 * max(interval_widths) if interval_widths else float("nan"),
        "truth_transport_samples": len(tr), "payload_enabled": events["payload_enabled"],
        "payload_anchor_s": events["payload_anchor_s"],
        "payload_ramp_elapsed_at_transport_s": t0 - events["payload_anchor_s"],
        "published_ff_per_arm_at_transport_start_n": force_t0,
        "published_ff_per_arm_transport_peak_n": force_peak,
        "release_x_m": events["release_x_m"], "release_y_m": events["release_y_m"],
        "release_z_m": events["release_z_m"], "release_xy_error_mm": 1000 * release_error_xy,
        "release_yaw_error_deg": float("nan"),
        "transport_ref_rows": len(refs), "transport_ref_start_x_m": fnum(refs[0], "rod_ref_x_m") if refs else float("nan"),
        "transport_ref_start_y_m": fnum(refs[0], "rod_ref_y_m") if refs else float("nan"),
        "transport_ref_start_z_m": rz[0] if rz else float("nan"),
        "transport_ref_end_x_m": fnum(refs[-1], "rod_ref_x_m") if refs else float("nan"),
        "transport_ref_end_y_m": fnum(refs[-1], "rod_ref_y_m") if refs else float("nan"),
        "transport_ref_end_z_m": rz[-1] if rz else float("nan"),
        "left_right_inferred_rod_residual_max_mm": max((
            1000 * math.sqrt(sum((fnum(r, f"left_rod_ref_{axis}_m") - fnum(r, f"right_rod_ref_{axis}_m")) ** 2
                                  for axis in ("x", "y", "z"))) for r in refs), default=float("nan")),
    }
    return metrics, curve, refs


def yaw_from_quaternion(row, prefix=""):
    qx, qy, qz, qw = (fnum(row, prefix + "rod_ref_qx"), fnum(row, prefix + "rod_ref_qy"),
                      fnum(row, prefix + "rod_ref_qz"), fnum(row, prefix + "rod_ref_qw"))
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def circular_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def normalized_yaw(rows, u):
    rows = sorted(rows, key=lambda r: fnum(r, "plan_elapsed_s"))
    duration = fnum(rows[0], "plan_duration_s")
    xs = [fnum(r, "plan_elapsed_s") / duration for r in rows]
    yaw = []
    for r in rows:
        qx, qy, qz, qw = (fnum(r, "rod_ref_qx"), fnum(r, "rod_ref_qy"),
                          fnum(r, "rod_ref_qz"), fnum(r, "rod_ref_qw"))
        raw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        yaw.append(raw if not yaw else yaw[-1] + circular_diff(raw, yaw[-1]))
    return interp(xs, yaw, u)


def normalized_reference(rows, key, u):
    rows = sorted(rows, key=lambda r: fnum(r, "plan_elapsed_s"))
    duration = fnum(rows[0], "plan_duration_s")
    xs = [fnum(r, "plan_elapsed_s") / duration for r in rows]
    ys = [fnum(r, key) for r in rows]
    return interp(xs, ys, u)


def pretrial_reference_difference(by_condition):
    if set(by_condition) != {"M0", "M1"}:
        return [], {}
    u_grid = [i / 200 for i in range(201)]
    detail = []
    for u in u_grid:
        r0 = by_condition["M0"]
        r1 = by_condition["M1"]
        row = {"normalized_transport_progress": u}
        for key in ("rod_ref_x_m", "rod_ref_y_m", "rod_ref_z_m",
                    "left_hand_ref_x_m", "left_hand_ref_y_m", "left_hand_ref_z_m",
                    "right_hand_ref_x_m", "right_hand_ref_y_m", "right_hand_ref_z_m"):
            a, b = normalized_reference(r0, key, u), normalized_reference(r1, key, u)
            row[f"M0_{key}"] = a
            row[f"M1_{key}"] = b
            row[f"M1_minus_M0_{key}"] = b - a if a is not None and b is not None else float("nan")
        yaw0 = normalized_yaw(r0, u)
        yaw1 = normalized_yaw(r1, u)
        row["M0_rod_ref_yaw_rad"] = yaw0
        row["M1_rod_ref_yaw_rad"] = yaw1
        row["M1_minus_M0_rod_ref_yaw_deg"] = math.degrees(circular_diff(yaw1, yaw0))
        detail.append(row)
    m0_duration = fnum(by_condition["M0"][0], "plan_duration_s")
    m1_duration = fnum(by_condition["M1"][0], "plan_duration_s")
    offsets_z = [r["M1_minus_M0_rod_ref_z_m"] * 1000 for r in detail]
    offsets_xy = [math.hypot(r["M1_minus_M0_rod_ref_x_m"] * 1000,
                             r["M1_minus_M0_rod_ref_y_m"] * 1000) for r in detail]
    offsets_yaw = [abs(r["M1_minus_M0_rod_ref_yaw_deg"]) for r in detail]
    summary = {
        "M0_transport_plan_duration_s": m0_duration,
        "M1_transport_plan_duration_s": m1_duration,
        "transport_plan_duration_difference_s_M1_minus_M0": m1_duration - m0_duration,
        "transport_plan_duration_relative_difference_pct_vs_M0":
            100 * (m1_duration - m0_duration) / m0_duration if m0_duration else float("nan"),
        "reference_z_difference_mean_mm_M1_minus_M0": statistics.mean(offsets_z),
        "reference_z_difference_min_mm": min(offsets_z), "reference_z_difference_max_mm": max(offsets_z),
        "reference_xy_difference_max_mm": max(offsets_xy),
        "reference_yaw_difference_max_deg": max(offsets_yaw),
    }
    return detail, summary


def fmt(x, digits=3):
    return "NA" if not isinstance(x, (float, int)) or not math.isfinite(float(x)) else f"{x:.{digits}f}"


def main():
    ledger = read_csv(OUT / "attempts.csv")
    write_csv(OUT / "raw_run_index.csv", [
        {"phase": r["phase"], "sequence": r["sequence"], "block": r["block"],
         "condition": r["condition"], "launch_attempt": r["launch_attempt"],
         "run_id": r["run_id"], "action_started": r["action_started"], "transport_started": r["transport_started"],
         "status": r["status"], "direct_reason": r["direct_reason"], "run_dir": r["run_dir"],
         "launcher_log": r["launcher_log"]} for r in ledger
    ])
    selected = [r for r in ledger if r["phase"] == "pretrial" and r["status"] == "pretrial_completed"]
    metrics, curves, ref_by_condition = [], [], {}
    for row in selected:
        m, c, refs = load_run(row)
        metrics.append(m)
        curves.extend(c)
        ref_by_condition[row["condition"]] = refs
    write_csv(OUT / "pretrial_metrics.csv", metrics)
    write_csv(OUT / "pretrial_transport_curves.csv", curves)
    ref_diff, ref_summary = pretrial_reference_difference(ref_by_condition)
    write_csv(OUT / "pretrial_reference_difference.csv", ref_diff)

    # This gate is deliberately conservative: actual total transport time must be comparable,
    # in addition to both complete workflows and valid common-reference coverage.
    successful = len(metrics) == 2 and all(m["task_complete"] == 1 for m in metrics)
    coverage = len(metrics) == 2 and all(m["e_z_sample_count"] > 10 and m["adopted_ref_sample_count"] > 10 for m in metrics)
    duration_pct = abs(ref_summary.get("transport_plan_duration_relative_difference_pct_vs_M0", float("inf")))
    duration_difference_s = abs(ref_summary.get("transport_plan_duration_difference_s_M1_minus_M0", float("inf")))
    ref_height_mm = abs(ref_summary.get("reference_z_difference_mean_mm_M1_minus_M0", float("inf")))
    duration_comparable = duration_difference_s <= 0.01  # one 100 Hz reference-publication interval
    reference_gate_pass = successful and coverage and duration_comparable

    if metrics:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        colors = {"M0": "#3b6fb6", "M1": "#c44e52"}
        fig, ax = plt.subplots(figsize=(7.2, 4.3), constrained_layout=True)
        for cond in ("M0", "M1"):
            c = [r for r in curves if r["condition"] == cond]
            if not c:
                continue
            ax.plot([r["elapsed_from_transport_start_s"] for r in c],
                    [r["e_z_common_mm"] for r in c], color=colors[cond], linewidth=1.8,
                    label=f"{cond} (pretrial, n=1)")
            ax.plot([r["elapsed_from_transport_start_s"] for r in c],
                    [r["delta_e_z_mm"] for r in c], color=colors[cond], linewidth=1.1,
                    linestyle="--", alpha=0.8)
        ax.axhline(0, color="black", linewidth=0.7)
        ax.set_xlabel("Transport elapsed time (s)")
        ax.set_ylabel("Vertical error (mm); dashed: change from start")
        ax.legend(frameon=False)
        fig.savefig(OUT / "pretrial_transport_error.png", dpi=180)
        plt.close(fig)

        if ref_diff:
            fig, axs = plt.subplots(2, 1, figsize=(7.2, 6.0), sharex=True, constrained_layout=True)
            us = [r["normalized_transport_progress"] for r in ref_diff]
            axs[0].plot(us, [r["M1_minus_M0_rod_ref_x_m"] * 1000 for r in ref_diff], label="X")
            axs[0].plot(us, [r["M1_minus_M0_rod_ref_y_m"] * 1000 for r in ref_diff], label="Y")
            axs[0].plot(us, [r["M1_minus_M0_rod_ref_z_m"] * 1000 for r in ref_diff], label="Z")
            axs[0].set_ylabel("M1 − M0 reference (mm)")
            axs[0].legend(frameon=False, ncol=3)
            axs[1].plot(us, [r["M1_minus_M0_left_hand_ref_x_m"] * 1000 for r in ref_diff], label="Left hand X")
            axs[1].plot(us, [r["M1_minus_M0_right_hand_ref_x_m"] * 1000 for r in ref_diff], label="Right hand X")
            axs[1].set_xlabel("Normalized transport plan progress")
            axs[1].set_ylabel("M1 − M0 hand ref X (mm)")
            axs[1].legend(frameon=False, ncol=2)
            fig.savefig(OUT / "pretrial_reference_difference.png", dpi=180)
            plt.close(fig)

    m_by = {m["condition"]: m for m in metrics}
    m0, m1 = m_by.get("M0", {}), m_by.get("M1", {})
    m0_start, m1_start = m0.get("e_z_start_mm", float("nan")), m1.get("e_z_start_mm", float("nan"))
    improvement_before = m1_start - m0_start
    infra = [r for r in ledger if r["status"] == "startup_exception"]
    text = f"""# 完整搬运载荷前馈预试验结果

## 预试验记录

M0与M1均从名义场景完成抓取、抬升、运输、下降、释放和撤退，各1/1次。预试验不纳入正式样本。M0在MMC激活后杆前馈关闭；M1在首个有效MMC参考确认后开启线性渐增，两臂在运输起点的已发布竖直力为 {fmt(m1.get('published_ff_per_arm_at_transport_start_n'))} N/臂，渐增已持续 {fmt(m1.get('payload_ramp_elapsed_at_transport_s'))} s（设定0.5 s）。运输起点距MMC激活约4.4 s，没有额外承载等待。前馈坐标+Z为世界竖直向上，两个仿真基座方向一致；M0/M1本体补偿相同。

共同竖向名义参考以夹持后、前馈尚未激活的杆中心高度加0.40 m定义。杆模型原点为对称杆的几何中心且与质心重合。实际数据使用异步杆真值请求/响应仿真时间中点；本次运输区间的请求响应间隔中位数分别为 {fmt(m0.get('truth_interval_median_ms'))} ms（M0）和 {fmt(m1.get('truth_interval_median_ms'))} ms（M1），最大间隔分别为 {fmt(m0.get('truth_interval_max_ms'))} ms、{fmt(m1.get('truth_interval_max_ms'))} ms。所报指标只在覆盖范围内插值，没有外推。

| 指标 | M0（前馈关） | M1（前馈开） |
|---|---:|---:|
| 完整流程 | {m0.get('task_complete', 0)}/1 | {m1.get('task_complete', 0)}/1 |
| 运输计划时长（s） | {fmt(m0.get('transport_plan_duration_s'))} | {fmt(m1.get('transport_plan_duration_s'))} |
| 实际运输时长（s） | {fmt(m0.get('transport_actual_duration_s'))} | {fmt(m1.get('transport_actual_duration_s'))} |
| 运输起点误差 e_z(t₀)（mm） | {fmt(m0_start)} | {fmt(m1_start)} |
| 运输平均误差（mm） | {fmt(m0.get('e_z_mean_mm'))} | {fmt(m1.get('e_z_mean_mm'))} |
| 运输总体误差 RMS（mm） | {fmt(m0.get('e_z_rms_mm'))} | {fmt(m1.get('e_z_rms_mm'))} |
| 运输新增误差 RMS（mm） | {fmt(m0.get('delta_e_z_rms_mm'))} | {fmt(m1.get('delta_e_z_rms_mm'))} |
| 运输新增误差末值 Δe_z（mm） | {fmt(m0.get('delta_e_z_end_mm'))} | {fmt(m1.get('delta_e_z_end_mm'))} |
| 运输起点实际杆参考相对共同名义高度（mm） | {fmt(m0.get('adopted_ref_offset_start_mm'))} | {fmt(m1.get('adopted_ref_offset_start_mm'))} |
| 释放平面位置误差（mm） | {fmt(m0.get('release_xy_error_mm'))} | {fmt(m1.get('release_xy_error_mm'))} |

预试验的运输起点共同名义误差差值 M1−M0 为 **{fmt(improvement_before)} mm**，即该次M1起点误差较小。这只是每条件单次预试验，不能作为正式效应估计。运输过程的M0与M1实际计划分别从各自抬升后的实测状态重建：计划时长差为 {fmt(ref_summary.get('transport_plan_duration_difference_s_M1_minus_M0'))} s，M1相对M0变化 {fmt(ref_summary.get('transport_plan_duration_relative_difference_pct_vs_M0'))}%；按归一化进度比较的两组实际物体参考高度差平均约 {fmt(ref_summary.get('reference_z_difference_mean_mm_M1_minus_M0'))} mm，杆XY参考最大间距约 {fmt(ref_summary.get('reference_xy_difference_max_mm'))} mm，偏航参考最大差约 {fmt(ref_summary.get('reference_yaw_difference_max_deg'))}°。杆参考高度差反映各条件运输前实际承载状态，不能用它重新置零共同名义误差。

## 正式实验门槛判断

- 两条件完整流程：通过（各1/1）。
- 前馈开关、时序和发布力：通过；M0发布力为零，M1到运输开始前已达到设定值。
- 共同名义误差及实际反馈时间覆盖：通过，可计算运输起点、总体误差和新增误差。
- 实际运输参考及总时长可比：**未通过**。相同的运输赋时配置下，状态依赖的关节规划仍产生约{fmt(duration_pct,1)}%的计划时间差，差异远超100 Hz参考发布周期0.01 s，且运输起点杆高相差约{fmt(ref_height_mm)} mm。运输期间的新增沉降无法与运行时长差清楚区分。

因此本批次**未启动正式实验，正式计数为0/20**；没有因预试验效果差而改参。需要先通过允许在两条件间冻结共同物体路径和同一运输时长的规划/赋时实现，使该参考经过现有闭链、碰撞和状态连续性检查；不应通过回放与实时起点不匹配的旧关节轨迹解决。修订后应重新做M0/M1各一次预试验，满足进入条件后再解锁10个正式区组。当前 `PRETRIAL_GATE_PASS` 文件不存在，正式运行器会拒绝启动。

## 启动异常

M0首两次动作前启动因受限环境禁止FastDDS枚举接口及创建UDP socket，服务发现120 s超时；第三次受限启动在未下发动作前停止。使用允许本机本地DDS通信的执行权限后服务正常发现，M0第四次启动及M1第一次启动均完成预试验。启动异常不计为正式搬运动作；原始目录和wrapper日志均保留在 `attempts.csv` 与 `raw_run_index.csv`。

现有任务日志的释放记录包含XYZ位置但未保存释放时杆姿态四元数，因此这两次预试验的释放偏航误差未报告；不得用释放前状态代替释放测量。若后续形成新的可比批次，应在不改控制的前提下补充该记录。

图表：`pretrial_transport_error.png`展示两次预试验的共同名义竖向误差（实线）和相对运输起点的新增误差（虚线）；`pretrial_reference_difference.png`展示按计划归一化进度对齐的实际物体及双手参考差异。两图仅用于预试验门槛审查，不代表正式组统计。
"""
    (OUT / "pretrial_results.md").write_text(text, encoding="utf-8")
    if reference_gate_pass:
        (OUT / "PRETRIAL_GATE_PASS").write_text("both pretrials passed; reference comparison reviewed\n", encoding="utf-8")
    else:
        (OUT / "PRETRIAL_GATE_PASS").unlink(missing_ok=True)
    print(f"pretrials={len(metrics)} complete={sum(m['task_complete'] for m in metrics)} "
          f"reference_gate_pass={reference_gate_pass} duration_diff_pct={duration_pct:.2f}")


if __name__ == "__main__":
    main()
