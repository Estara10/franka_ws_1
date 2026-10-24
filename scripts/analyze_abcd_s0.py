#!/usr/bin/env python3
"""Summarize the fixed S0 ABCD runs from their existing run logs and CSVs."""

import csv
import json
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "experiment_results/a1_runs"
OUT = ROOT / "experiment_results/reports/abcd_s0_20261022"
FIG = ROOT / "docs/figures/历史归档/旧版统计"
ORDER = ["Ar1", "Cr1", "Br1", "Dr1", "Dr2", "Br2", "Cr2", "Ar2", "Br3", "Ar3", "Dr3", "Cr3"]
TARGET = (0.40, 0.0, 0.020654)
TARGET_YAW = math.radians(20.0)


def floats(text):
    return [float(v.strip()) for v in text.split(",")]


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrapped_angle(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def parse_run(tag):
    run_id = f"abcd_s0_{tag}_20261022"
    path = RUNS / run_id
    log = (path / "task_launch.log").read_text(errors="replace")
    state = json.loads((path / "prepare_status.json").read_text()).get("task_state", "UNKNOWN")
    result = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+) detail=([^\n]+)", log)
    final_state, reason = result[-1] if result else (state, "")
    group = tag[0]
    release = re.findall(
        r"读回原始值: object_frame='base_link'.*?pose=\(([^)]+)\) quat=\(([^)]+)\)", log
    )
    release_xyz = release_yaw = pos_error = yaw_error = ""
    if final_state == "DONE" and release:
        xyz = floats(release[-1][0])
        q = floats(release[-1][1])
        release_yaw_rad = yaw_from_quaternion(*q)
        pos_error = math.sqrt(sum((xyz[i] - TARGET[i]) ** 2 for i in range(3))) * 1000
        yaw_error = abs(math.degrees(wrapped_angle(release_yaw_rad - TARGET_YAW)))
        release_xyz = ";".join(f"{v:.6f}" for v in xyz)
        release_yaw = f"{math.degrees(release_yaw_rad):.3f}"

    plans = re.findall(
        r"\[CONTACT_PROBE_PLAN\] delta=([0-9.]+)m points=(\d+).*?IPTP duration=([0-9.]+)s", log
    )
    plan_times = [float(row[2]) for row in plans]
    lift_plan = plan_times[0] if len(plan_times) >= 1 else ""
    transport_plan = plan_times[1] if len(plan_times) >= 2 else ""
    lower_plan = plan_times[2] if len(plan_times) >= 3 else ""
    retreat_plan = plan_times[3] if len(plan_times) >= 4 else ""
    optimized = re.findall(
        r"\[ABCD_TRAJECTORY\].*?duration=([0-9.]+) exported_jerk_rms=([0-9.]+)", log
    )
    opt_duration, opt_jerk = optimized[-1] if optimized else ("", "")
    dispatch = re.findall(
        r"\[CONTACT_DISPATCH\] phase=TRANSPORT joint=\S+ measured=([-0-9.]+) plan_start=([-0-9.]+) delta=([0-9.]+) tolerance=([0-9.]+)",
        log,
    )
    start_delta = dispatch[-1][2] if dispatch else ""
    start_tolerance = dispatch[-1][3] if dispatch else ""

    residual_pos_mm = residual_angle_deg = ""
    csv_path = path / "contact_probe.csv"
    if csv_path.exists():
        with csv_path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        carry_rows = [r for r in rows if r.get("phase") in {"LIFT", "HOLD", "TRANSPORT", "ROTATE", "LOWER"}]
        if carry_rows:
            vals = []
            for side in ("left", "right"):
                for row in carry_rows:
                    try:
                        vals.append(abs(float(row[f"{side}_slip_m"])) * 1000)
                    except (ValueError, KeyError):
                        pass
            if vals:
                residual_pos_mm = max(vals)
            ang = []
            for side in ("left", "right"):
                for row in carry_rows:
                    try:
                        ang.append(abs(math.degrees(float(row[f"{side}_angle_rad"]))))
                    except (ValueError, KeyError):
                        pass
            if ang:
                residual_angle_deg = max(ang)

    phase_lines = re.findall(
        r"\[ABCD_MMC_PHASE_COMPLETE\] group=\w+ phase=(LIFT|TRANSPORT|LOWER) duration=([0-9.]+)", log
    )
    phase_actual = {k.lower() + "_actual_s": v for k, v in phase_lines}
    stamps = re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[P3A_RESULT\]", log)
    result_stamps = re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_RESULT\]", log)
    route_elapsed = ""
    if stamps and result_stamps:
        route_elapsed = max(0.0, float(result_stamps[-1]) - float(stamps[0]))

    return {
        "run_id": run_id,
        "group": group,
        "state": final_state,
        "failure": reason if final_state != "DONE" else "",
        "release_xyz_m": release_xyz,
        "release_yaw_deg": release_yaw,
        "release_position_error_mm": f"{pos_error:.3f}" if pos_error != "" else "",
        "release_yaw_error_deg": f"{yaw_error:.3f}" if yaw_error != "" else "",
        "max_grasp_reference_residual_mm": f"{residual_pos_mm:.3f}" if residual_pos_mm != "" else "",
        "max_grasp_reference_angle_deg": f"{residual_angle_deg:.3f}" if residual_angle_deg != "" else "",
        "lift_plan_s": lift_plan,
        "transport_plan_s": transport_plan,
        "optimized_transport_s": opt_duration,
        "optimized_export_jerk_rms": opt_jerk,
        "lower_plan_s": lower_plan,
        "retreat_plan_s": retreat_plan,
        "transport_start_delta_rad": start_delta,
        "transport_start_tolerance_rad": start_tolerance,
        "P3A_to_DONE_or_FAIL_s": f"{route_elapsed:.3f}" if route_elapsed != "" else "",
        **phase_actual,
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    records = [parse_run(tag) for tag in ORDER]
    fields = list(dict.fromkeys(key for row in records for key in row))
    with (OUT / "abcd_s0_run_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    groups = ["A", "B", "C", "D"]
    complete = {g: sum(r["group"] == g and r["state"] == "DONE" for r in records) for g in groups}
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    colors = {"A": "#4477AA", "B": "#CC6677", "C": "#228833", "D": "#AA3377"}
    axes[0].bar(groups, [complete[g] for g in groups], color=[colors[g] for g in groups])
    axes[0].set_ylim(0, 3.25)
    axes[0].set_yticks([0, 1, 2, 3])
    axes[0].set_title("Full route completion")
    axes[0].set_ylabel("runs completed (out of 3)")
    for i, g in enumerate(groups):
        axes[0].text(i, complete[g] + 0.08, f"{complete[g]}/3", ha="center")

    for i, g in enumerate(groups):
        vals = [float(r["release_position_error_mm"]) for r in records if r["group"] == g and r["release_position_error_mm"]]
        if vals:
            axes[1].scatter([i] * len(vals), vals, color=colors[g], s=45, zorder=3)
            axes[1].plot([i - 0.15, i + 0.15], [sum(vals) / len(vals)] * 2, color=colors[g], linewidth=2)
    axes[1].set_xticks(range(4), groups)
    axes[1].set_title("Release position error")
    axes[1].set_ylabel("3D distance to target (mm)")
    axes[1].grid(axis="y", alpha=0.25)

    for i, g in enumerate(groups):
        vals = [float(r["release_yaw_error_deg"]) for r in records if r["group"] == g and r["release_yaw_error_deg"]]
        if vals:
            axes[2].scatter([i] * len(vals), vals, color=colors[g], s=45, zorder=3)
            axes[2].plot([i - 0.15, i + 0.15], [sum(vals) / len(vals)] * 2, color=colors[g], linewidth=2)
    axes[2].set_xticks(range(4), groups)
    axes[2].set_title("Release yaw error")
    axes[2].set_ylabel("absolute error to +20° (deg)")
    axes[2].grid(axis="y", alpha=0.25)

    fig.suptitle("S0 no-weld ABCD transport — 3 runs per group, descriptive only")
    fig.tight_layout()
    fig.savefig(FIG / "20261022_上一批搬运结果.png", dpi=180)
    plt.close(fig)

    print(f"wrote {OUT / 'abcd_s0_run_summary.csv'}")
    print(f"wrote {FIG / 'abcd_s0_completion_and_release_error.png'}")
    for g in groups:
        group_rows = [r for r in records if r["group"] == g]
        pos = [float(r["release_position_error_mm"]) for r in group_rows if r["release_position_error_mm"]]
        yaw = [float(r["release_yaw_error_deg"]) for r in group_rows if r["release_yaw_error_deg"]]
        print(g, f"{complete[g]}/3", "position_mm=" + (f"{sum(pos)/len(pos):.3f} mean" if pos else "n/a"),
              "yaw_deg=" + (f"{sum(yaw)/len(yaw):.3f} mean" if yaw else "n/a"))


if __name__ == "__main__":
    main()
