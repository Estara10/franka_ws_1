#!/usr/bin/env python3
"""Offline summary and figures for the 2026-10-22 fixed-S0 ABCD batch."""

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
OUT = ROOT / "experiment_results/reports/abcd_s0_today_20261022"
FIG = ROOT / "docs/figures/abcd_s0_today_20261022"
FORMAL = [
    ("A", "abcd_s0_today_A1_20261022_210600"),
    ("C", "abcd_s0_today_C1_20261022_210700"),
    ("B", "abcd_s0_today_B1_20261022_210800"),
    ("D", "abcd_s0_today_D1_20261022_211200"),
    ("D", "abcd_s0_today_D2_20261022_211300"),
    ("B", "abcd_s0_today_B2_20261022_211400"),
    ("C", "abcd_s0_today_C2_20261022_211500"),
    ("A", "abcd_s0_today_A2_20261022_211600"),
    ("B", "abcd_s0_today_B3_20261022_211700"),
    ("A", "abcd_s0_today_A3_20261022_211800"),
    ("D", "abcd_s0_today_D3_20261022_211900"),
    ("C", "abcd_s0_today_C3_20261022_212000"),
]
VALIDATION = [
    ("B", "abcd_s0_Bconnectionv1_20261022_161500_retryinfra"),
    ("B", "abcd_s0_Bconnectionv2_20261022_211000"),
]
COLORS = {"A": "#4477AA", "B": "#CC6677", "C": "#228833", "D": "#AA3377"}
TARGET_XY = (0.40, 0.0)
SUPPORT_Z = 0.000654
ROD_DIMS = (0.04, 0.40, 0.04)


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrapped(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def bottom_extent(q):
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
    r20 = 2 * (xz - wy)
    r21 = 2 * (yz + wx)
    r22 = 1 - 2 * (xx + yy)
    return 0.5 * (abs(r20) * ROD_DIMS[0] + abs(r21) * ROD_DIMS[1] + abs(r22) * ROD_DIMS[2])


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def parse_run(group, run_id, formal=True):
    path = RUNS / run_id
    log_path = path / "task_launch.log"
    log = log_path.read_text(errors="replace") if log_path.exists() else ""
    status_path = path / "prepare_status.json"
    status = json.loads(status_path.read_text()).get("task_state", "UNKNOWN") if status_path.exists() else "UNKNOWN"
    result = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+) detail=([^\n]+)", log)
    state, reason = result[-1] if result else (status, "")
    if state != "DONE" and not reason:
        failures = re.findall(r"\[CONTACT_PROBE_FAILURE\] ([^\n]+)", log)
        reason = failures[-1].strip() if failures else ""

    # The release readback is the final A1 object read immediately before the
    # existing CONTACT_PROBE_RELEASED record; its wait is common across groups.
    release_lines = list(re.finditer(
        r"\[A1\] 读回原始值: object_frame='base_link'.*?pose=\(([^)]+)\) quat=\(([^)]+)\)", log
    ))
    released = re.search(r"\[CONTACT_PROBE_RELEASED\] xyz=\(([^)]+)\)", log)
    release_xyz = release_yaw = xy_error = yaw_error = z_error = gap = ""
    release_center = None
    release_quaternion = None
    if released and release_lines:
        before = [match for match in release_lines if match.start() < released.start()]
        if before:
            match = before[-1]
            release_center = [float(v.strip()) for v in match.group(1).split(",")]
            release_quaternion = [float(v.strip()) for v in match.group(2).split(",")]
            # Pose quat order in the log is x,y,z,w.
            yaw = quat_yaw(release_quaternion)
            xy_error = math.hypot(release_center[0] - TARGET_XY[0], release_center[1] - TARGET_XY[1]) * 1000
            yaw_error = abs(math.degrees(wrapped(yaw - math.radians(20.0))))
            z_error = (release_center[2] - (SUPPORT_Z + 0.020)) * 1000
            gap = (release_center[2] - bottom_extent(release_quaternion) - SUPPORT_Z) * 1000
            release_xyz = ";".join(f"{v:.6f}" for v in release_center)
            release_yaw = math.degrees(yaw)

    plans = {m.group(1): float(m.group(2)) for m in re.finditer(
        r"\[CONTACT_PLAN_TIMING\] label=(lift|transport|lower|retreat).*?final=([0-9.]+)", log
    )}
    mmc_phases = {m.group(1): float(m.group(2)) for m in re.finditer(
        r"\[ABCD_MMC_PHASE_COMPLETE\] group=\w+ phase=(LIFT|TRANSPORT|LOWER) duration=([0-9.]+)", log
    )}
    mmc_phases = {k.lower(): v for k, v in mmc_phases.items()}
    accepted = [float(v) for v in re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[move_group_interface\]: Execute request accepted", log)]
    succeeded = [float(v) for v in re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[move_group_interface\]: Execute request success!", log)]
    action_durations = [s - a for a, s in zip(accepted, succeeded) if s >= a]
    if group in ("A", "B"):
        # The two pre-grasp/grasp actions precede the four carrying actions.
        phase_actual = dict(zip(("lift", "transport", "lower", "retreat"), action_durations[2:6]))
    else:
        phase_actual = dict(mmc_phases)
        if action_durations:
            phase_actual["retreat"] = action_durations[-1]

    stamps_start = re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_CONFIG\]", log)
    stamps_end = re.findall(r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_RESULT\]", log)
    total_elapsed = float(stamps_end[-1]) - float(stamps_start[0]) if stamps_start and stamps_end else ""
    release_delay = ""
    if released and stamps_start:
        stamp = re.search(r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_RELEASED\]", log)
        if stamp:
            release_delay = float(stamp.group(1)) - float(stamps_start[0])

    connection = re.search(r"\[ABCD_JTC_START_CONNECTION\] samples=(\d+) duration_sec=([0-9.]+) max_joint_delta_rad=([0-9.]+)", log)
    mmc_connection = re.search(r"\[ABCD_MMC_START_CONNECTION\] phase=TRANSPORT samples=(\d+) max_joint_delta_rad=([0-9.]+)", log)
    if connection:
        connection_samples, connection_sec, connection_delta = connection.group(1), float(connection.group(2)), float(connection.group(3))
    elif mmc_connection:
        connection_samples, connection_sec, connection_delta = mmc_connection.group(1), "", float(mmc_connection.group(2))
    else:
        connection_samples = connection_sec = connection_delta = ""

    metrics = read_csv(path / "abcd_transport_time_profile.csv")
    baseline = next((r for r in metrics if r.get("plan") == "baseline"), {})
    candidate = next((r for r in metrics if r.get("plan") == "candidate"), {})
    transport_dispatch = read_csv(path / "abcd_transport_dispatch.csv")
    conn_plan = read_csv(path / "abcd_transport_start_connection.csv")
    baseline_traj = read_csv(path / "abcd_transport_baseline.csv")
    candidate_traj = read_csv(path / "abcd_transport_optimized.csv")
    # writeTrajectoryCsv stores one row per joint per timestamp; use the max t.
    def duration(rows):
        return max((float(r["time_from_start"]) for r in rows), default="") if rows else ""

    trace = read_csv(path / "contact_probe.csv")
    carry = [r for r in trace if r.get("phase") in {"LIFT", "HOLD", "TRANSPORT", "ROTATE", "LOWER"}]
    residual = []
    angles = []
    for row in carry:
        for side in ("left", "right"):
            try:
                residual.append(abs(float(row[f"{side}_slip_m"])) * 1000)
                angles.append(abs(math.degrees(float(row[f"{side}_angle_rad"]))))
            except (KeyError, ValueError):
                pass

    if group in ("C", "D"):
        connection_policy = "MMC_checked_start_connection"
    elif run_id in {
        "abcd_s0_today_A1_20261022_210600",
        "abcd_s0_today_B1_20261022_210800",
        "abcd_s0_Bconnectionv1_20261022_161500_retryinfra",
    }:
        connection_policy = "JTC_connection_gate_5mrad"
    else:
        connection_policy = "JTC_connection_gate_12mrad"

    return {
        "run_id": run_id,
        "batch": "formal" if formal else "validation",
        "connection_policy": connection_policy,
        "group": group,
        "state": state,
        "completed_release_retreat": int(state == "DONE"),
        "failure_reason": "" if state == "DONE" else reason,
        "release_xyz_m": release_xyz,
        "release_yaw_deg": f"{release_yaw:.4f}" if release_yaw != "" else "",
        "release_xy_error_mm": f"{xy_error:.4f}" if xy_error != "" else "",
        "release_yaw_error_deg": f"{yaw_error:.4f}" if yaw_error != "" else "",
        "release_center_z_error_mm": f"{z_error:.4f}" if z_error != "" else "",
        "rod_bottom_gap_mm": f"{gap:.4f}" if gap != "" else "",
        "release_after_contact_start_s": f"{release_delay:.3f}" if release_delay != "" else "",
        "lift_plan_s": f"{plans['lift']:.4f}" if "lift" in plans else "",
        "lift_actual_s": f"{phase_actual['lift']:.4f}" if "lift" in phase_actual else "",
        "transport_plan_s": f"{plans['transport']:.4f}" if "transport" in plans else "",
        "transport_actual_s": f"{phase_actual['transport']:.4f}" if "transport" in phase_actual else "",
        "lower_plan_s": f"{plans['lower']:.4f}" if "lower" in plans else "",
        "lower_actual_s": f"{phase_actual['lower']:.4f}" if "lower" in phase_actual else "",
        "retreat_plan_s": f"{plans['retreat']:.4f}" if "retreat" in plans else "",
        "retreat_actual_s": f"{phase_actual['retreat']:.4f}" if "retreat" in phase_actual else "",
        "contact_start_to_terminal_s": f"{total_elapsed:.3f}" if total_elapsed != "" else "",
        "max_grasp_reference_residual_mm": f"{max(residual):.4f}" if residual else "",
        "max_grasp_reference_angle_deg": f"{max(angles):.4f}" if angles else "",
        "start_connection_samples": connection_samples,
        "start_connection_s": f"{connection_sec:.4f}" if connection_sec != "" else "",
        "start_connection_max_joint_delta_rad": f"{connection_delta:.9f}" if connection_delta != "" else "",
        "baseline_transport_duration_s": f"{duration(baseline_traj):.6f}" if baseline_traj else "",
        "candidate_transport_duration_s": f"{duration(candidate_traj):.6f}" if candidate_traj else "",
        "dispatch_transport_duration_s": f"{duration(transport_dispatch):.6f}" if transport_dispatch else "",
        "connection_trajectory_duration_s": f"{duration(conn_plan):.6f}" if conn_plan else "",
        "baseline_continuous_jerk_rms": baseline.get("continuous_jerk_rms", ""),
        "candidate_continuous_jerk_rms": candidate.get("continuous_jerk_rms", ""),
        "baseline_continuous_jerk_max": baseline.get("continuous_jerk_max", ""),
        "candidate_continuous_jerk_max": candidate.get("continuous_jerk_max", ""),
        "baseline_exported_jerk_rms": baseline.get("exported_jerk_rms", ""),
        "candidate_exported_jerk_rms": candidate.get("exported_jerk_rms", ""),
        "baseline_exported_jerk_max": baseline.get("exported_jerk_max", ""),
        "candidate_exported_jerk_max": candidate.get("exported_jerk_max", ""),
        "max_export_deviation_rad": candidate.get("max_export_deviation_rad", ""),
        "time_profile_solve_sec": candidate.get("solve_sec", ""),
        "transport_executed": int(bool(transport_dispatch) and state == "DONE") if group in ("B", "D") else int(state == "DONE"),
        "_trace": trace,
        "_release_center": release_center,
    }


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def mean_range(rows, key):
    vals = [number(row, key) for row in rows]
    vals = [v for v in vals if v is not None]
    if not vals:
        return ""
    return f"{sum(vals)/len(vals):.4f} [{min(vals):.4f},{max(vals):.4f}]"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    records = [parse_run(g, run_id, True) for g, run_id in FORMAL]
    validations = [parse_run(g, run_id, False) for g, run_id in VALIDATION]
    public_fields = [k for k in records[0] if not k.startswith("_")]
    for name, rows in (("abcd_s0_formal_runs.csv", records), ("abcd_s0_validation_runs.csv", validations)):
        with (OUT / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=public_fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    groups = ["A", "B", "C", "D"]
    summary = []
    for group in groups:
        rows = [r for r in records if r["group"] == group]
        completed_rows = [r for r in rows if r["state"] == "DONE"]
        optimized_rows = [r for r in rows if number(r, "candidate_continuous_jerk_rms") is not None]
        done = sum(r["state"] == "DONE" for r in rows)
        def avg(key, subset):
            vals = [number(r, key) for r in subset]
            vals = [v for v in vals if v is not None]
            return sum(vals) / len(vals) if vals else None

        base_cont = avg("baseline_continuous_jerk_rms", optimized_rows)
        cand_cont = avg("candidate_continuous_jerk_rms", optimized_rows)
        base_export = avg("baseline_exported_jerk_rms", optimized_rows)
        cand_export = avg("candidate_exported_jerk_rms", optimized_rows)
        summary.append({
            "group": group,
            "runs": len(rows),
            "full_done": done,
            "failed": len(rows) - done,
            "completion": f"{done}/{len(rows)}",
            "release_xy_error_mm_mean_range": mean_range(rows, "release_xy_error_mm"),
            "release_yaw_error_deg_mean_range": mean_range(rows, "release_yaw_error_deg"),
            "rod_bottom_gap_mm_mean_range": mean_range(rows, "rod_bottom_gap_mm"),
            "completed_contact_start_to_terminal_s_mean_range": mean_range(completed_rows, "contact_start_to_terminal_s"),
            "all_attempts_contact_start_to_terminal_s_mean_range": mean_range(rows, "contact_start_to_terminal_s"),
            "completed_transport_actual_s_mean_range": mean_range(completed_rows, "transport_actual_s"),
            "max_grasp_residual_mm_mean_range": mean_range(rows, "max_grasp_reference_residual_mm"),
            "optimized_runs_generated": len(optimized_rows),
            "baseline_continuous_jerk_rms_mean": f"{base_cont:.6f}" if base_cont is not None else "",
            "candidate_continuous_jerk_rms_mean": f"{cand_cont:.6f}" if cand_cont is not None else "",
            "continuous_jerk_rms_change_pct": f"{100*(cand_cont/base_cont-1):.3f}" if base_cont else "",
            "baseline_exported_jerk_rms_mean": f"{base_export:.6f}" if base_export is not None else "",
            "candidate_exported_jerk_rms_mean": f"{cand_export:.6f}" if cand_export is not None else "",
            "exported_jerk_rms_change_pct": f"{100*(cand_export/base_export-1):.3f}" if base_export else "",
        })
    with (OUT / "abcd_s0_group_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)

    # Figure 1: completion plus same-release placement errors and support gap.
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes[0, 0]
    done_counts = [sum(r["group"] == g and r["state"] == "DONE" for r in records) for g in groups]
    ax.bar(groups, done_counts, color=[COLORS[g] for g in groups])
    ax.set_ylim(0, 3.35); ax.set_yticks(range(4)); ax.set_ylabel("complete release + retreat (of 3)")
    ax.set_title("Full-route completion")
    for i, val in enumerate(done_counts): ax.text(i, val + 0.05, f"{val}/3", ha="center")
    for ax, key, title, ylabel in (
        (axes[0, 1], "release_xy_error_mm", "Release XY error", "mm"),
        (axes[1, 0], "release_yaw_error_deg", "Release yaw error", "degrees"),
        (axes[1, 1], "rod_bottom_gap_mm", "Rod bottom gap at release readback", "mm"),
    ):
        for i, group in enumerate(groups):
            vals = [number(r, key) for r in records if r["group"] == group]
            vals = [v for v in vals if v is not None]
            if vals:
                ax.scatter([i] * len(vals), vals, color=COLORS[group], s=45, zorder=3)
                ax.plot([i - .18, i + .18], [sum(vals) / len(vals)] * 2, color=COLORS[group], lw=2)
        ax.set_xticks(range(4), groups); ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(axis="y", alpha=.25)
    fig.suptitle("Fixed S0 no-weld ABCD — every formal attempt shown; descriptive n=3")
    fig.tight_layout()
    fig.savefig(FIG / "abcd_s0_completion_and_placement.png", dpi=180)
    plt.close(fig)

    # Figure 2: same-path before/after metrics for each B/D time-profile run.
    opt_rows = [r for r in records if r["group"] in ("B", "D") and r["candidate_continuous_jerk_rms"]]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    xlabels = [re.search(r"_([ABCD][123])_", r["run_id"]).group(1) for r in opt_rows]
    x = list(range(len(opt_rows)))
    for ax, base_key, cand_key, title, ylabel in (
        (axes[0], "baseline_continuous_jerk_rms", "candidate_continuous_jerk_rms", "Native continuous jerk RMS", "rad/s³"),
        (axes[1], "baseline_exported_jerk_rms", "candidate_exported_jerk_rms", "Exported trajectory jerk RMS", "rad/s³"),
    ):
        width = .36
        baseline = [number(r, base_key) for r in opt_rows]
        candidate = [number(r, cand_key) for r in opt_rows]
        ax.bar([v - width / 2 for v in x], baseline, width, label="baseline", color="#8da0cb")
        ax.bar([v + width / 2 for v in x], candidate, width, label="time-profile candidate", color="#fc8d62")
        ax.set_xticks(x, xlabels, rotation=35, ha="right"); ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(axis="y", alpha=.25)
        ax.legend(frameon=False)
    ax = axes[2]
    width = .36
    base_duration = [number(r, "baseline_transport_duration_s") for r in opt_rows]
    cand_duration = [number(r, "candidate_transport_duration_s") for r in opt_rows]
    ax.bar([v - width / 2 for v in x], base_duration, width, label="baseline", color="#8da0cb")
    ax.bar([v + width / 2 for v in x], cand_duration, width, label="time-profile candidate", color="#fc8d62")
    ax.set_xticks(x, xlabels, rotation=35, ha="right"); ax.set_ylabel("s")
    ax.set_title("Matched transport duration"); ax.grid(axis="y", alpha=.25); ax.legend(frameon=False)
    fig.suptitle("Same-run, same geometric path; B1 candidate was generated but not executed")
    fig.tight_layout()
    fig.savefig(FIG / "abcd_s0_time_profile_jerk.png", dpi=180)
    plt.close(fig)

    # Figure 3: measured rod-center space tracks from the existing phase CSVs.
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    for row in records:
        run_id, group = row["run_id"], row["group"]
        trace = row["_trace"]
        pts = []
        for sample in trace:
            if sample.get("phase") not in {"LIFT", "HOLD", "TRANSPORT", "ROTATE", "LOWER", "RETREAT"}:
                continue
            try:
                pts.append((float(sample["rod_x"]), float(sample["rod_y"]), float(sample["rod_z"])))
            except (KeyError, ValueError):
                continue
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts],
                    color=COLORS[group], alpha=.55, lw=1.3,
                    label=f"{group} {re.search(r'_([ABCD][123])_', run_id).group(1)}")
    ax.scatter([.50], [0], [.020654], color="black", marker="o", s=45, label="initial")
    ax.scatter([.40], [0], [.020654], color="black", marker="x", s=55, label="target center")
    ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)"); ax.set_zlabel("world z (m)")
    ax.set_title("Measured rod-center paths during lift, transport, lower and retreat")
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    fig.tight_layout()
    fig.savefig(FIG / "abcd_s0_rod_space_trajectories.png", dpi=180)
    plt.close(fig)

    print(f"formal={OUT / 'abcd_s0_formal_runs.csv'}")
    print(f"groups={OUT / 'abcd_s0_group_summary.csv'}")
    print(f"validation={OUT / 'abcd_s0_validation_runs.csv'}")
    for item in summary:
        print(item)
    for name in ("abcd_s0_completion_and_placement.png", "abcd_s0_time_profile_jerk.png", "abcd_s0_rod_space_trajectories.png"):
        print(FIG / name)


if __name__ == "__main__":
    main()
