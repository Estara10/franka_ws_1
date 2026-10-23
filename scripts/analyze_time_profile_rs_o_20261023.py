#!/usr/bin/env python3
"""Build the R/S/O offline time-smoothness comparison.

R is the saved original transport reference. S is the same geometry stretched
by 1.2 in time. O is the corrected time-profile optimizer run against S at the
same 1.2T0 budget. This script is offline-only and never starts ROS or MuJoCo.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import statistics
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scientific_style import GROUP_COLORS, configure_scientific_style

configure_scientific_style()

ROOT = Path(__file__).resolve().parents[1]
FORMAL_RUNS = ROOT / "experiment_results/reports/final_tradeoff_20261022/abcd_s0_formal_runs.csv"
OUT_DIR = ROOT / "experiment_results/reports/final_tradeoff_20261022/time_profile_rs_o"
FIG = ROOT / "docs/figures/final_tradeoff_20261022/time_profile_rs_o.png"
DEFAULT_REPLAY = (
    ROOT
    / "install/dual_arm_carry_task/lib/dual_arm_carry_task/time_profile_replay"
)
SCALE = 1.2
THEORETICAL_JERK_RATIO = 1.0 / (SCALE**3)
S_RMS_RATIO_TOLERANCE = 0.01
# At a fixed 1 ms sampling grid, the peak can be sampled at a different phase
# after time stretching even when the analytic curve scales exactly.
S_PEAK_RATIO_TOLERANCE = 0.06


def alignment_module():
    path = Path(__file__).with_name("analyze_time_profile_alignment_20261023.py")
    spec = importlib.util.spec_from_file_location("time_profile_alignment", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def as_float(value: str) -> float:
    return float(value)


def bool_int(value: object) -> int:
    return 1 if str(value).strip() in {"1", "true", "True"} else 0


def percent_change(value: float, baseline: float) -> float:
    return 100.0 * (value / baseline - 1.0) if baseline != 0.0 else 0.0


def read_trajectory_points(path: Path) -> dict[int, dict[str, float]]:
    points: dict[int, dict[str, float]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            index = int(row["point_index"])
            points.setdefault(index, {})[row["joint_name"]] = float(row["position"])
    return points


def endpoint_difference(r_path: Path, s_path: Path) -> float:
    r_points = read_trajectory_points(r_path)
    s_points = read_trajectory_points(s_path)
    if not r_points or not s_points:
        return float("inf")
    difference = 0.0
    for r_index, s_index in (
        (min(r_points), min(s_points)),
        (max(r_points), max(s_points)),
    ):
        r_point = r_points[r_index]
        s_point = s_points[s_index]
        if set(r_point) != set(s_point):
            return float("inf")
        for joint in r_point:
            difference = max(difference, abs(r_point[joint] - s_point[joint]))
    return difference


def run_replay(replay_bin: Path, trajectory: Path, limits: Path, out_dir: Path,
               duration_scale: float) -> dict[str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(replay_bin),
        "--trajectory",
        str(trajectory),
        "--limits-csv",
        str(limits),
        "--out-dir",
        str(out_dir),
        "--duration-scale",
        str(duration_scale),
        "--max-evaluations",
        "100",
    ]
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    (out_dir / "command.txt").write_text(" ".join(command) + "\n", encoding="utf-8")
    (out_dir / "stdout.txt").write_text(completed.stdout, encoding="utf-8")
    (out_dir / "stderr.txt").write_text(completed.stderr, encoding="utf-8")
    if completed.returncode != 0:
        reason = completed.stderr.strip().splitlines()[-1] if completed.stderr.strip() else "replay failed"
        raise RuntimeError(f"replay exit={completed.returncode}: {reason}")
    summary_path = out_dir / "time_profile_summary.csv"
    if not summary_path.exists():
        raise RuntimeError(f"missing {summary_path}")
    rows = read_csv(summary_path)
    if len(rows) != 1:
        raise RuntimeError(f"expected one summary row in {summary_path}")
    return rows[0]


def group_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    summaries: list[dict[str, object]] = []
    for group in "ABCD":
        values = [row for row in rows if row["group"] == group]
        if not values:
            continue
        rms = [float(row["o_vs_s_rms_pct"]) for row in values]
        peak = [float(row["o_vs_s_peak_pct"]) for row in values]
        summaries.append(
            {
                "group": group,
                "n": len(values),
                "formal_done_n": sum(
                    str(row["formal_state"]) == "DONE" for row in values
                ),
                "o_accepted_n": sum(int(row["o_accepted"]) for row in values),
                "o_vs_s_rms_pct_mean": statistics.mean(rms),
                "o_vs_s_rms_pct_median": statistics.median(rms),
                "o_vs_s_rms_pct_min": min(rms),
                "o_vs_s_rms_pct_max": max(rms),
                "o_vs_s_peak_pct_mean": statistics.mean(peak),
                "o_vs_s_peak_pct_median": statistics.median(peak),
                "o_vs_s_peak_pct_min": min(peak),
                "o_vs_s_peak_pct_max": max(peak),
            }
        )
    return summaries


def make_figure(rows: list[dict[str, object]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    group_colors = GROUP_COLORS
    x = np.arange(len(rows))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.4))

    for i, row in enumerate(rows):
        color = group_colors[str(row["group"])]
        axes[0].plot(
            [i - 0.16, i, i + 0.16],
            [float(row["rms_R"]), float(row["rms_S"]), float(row["rms_O"])],
            color=color,
            marker="o",
            linewidth=1.2,
            markersize=4,
        )
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([str(row["run_id"]).split("_2026")[0] for row in rows], rotation=75, ha="right")
    axes[0].set_ylabel("Continuous jerk RMS (rad/s³)")
    axes[0].set_title("Absolute reference smoothness")
    axes[0].grid(axis="y", alpha=0.25)

    sr = [float(row["s_vs_r_rms_pct"]) for row in rows]
    osr = [float(row["o_vs_s_rms_pct"]) for row in rows]
    axes[1].axhline(-42.13, color="#666666", linestyle="--", linewidth=1.0, label="1.2^-3 theory")
    axes[1].scatter(x, sr, c=[group_colors[str(row["group"])] for row in rows], s=30)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([str(row["run_id"]).split("_2026")[0] for row in rows], rotation=75, ha="right")
    axes[1].set_ylabel("RMS change vs R (%)")
    axes[1].set_title("S: uniform 1.2T stretch")
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend(frameon=False)

    axes[2].axhline(-5.0, color="#B22222", linestyle="--", linewidth=1.0, label="-5% gate")
    axes[2].scatter(x, osr, c=[group_colors[str(row["group"])] for row in rows], s=30)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels([str(row["run_id"]).split("_2026")[0] for row in rows], rotation=75, ha="right")
    axes[2].set_ylabel("RMS change vs S (%)")
    axes[2].set_title("O: optimization within 1.2T")
    axes[2].grid(axis="y", alpha=0.25)
    axes[2].legend(frameon=False)

    fig.suptitle("R/S/O offline continuous reference comparison; not measured arm jerk", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def write_report(path: Path, gate: dict[str, object], groups: list[dict[str, object]]) -> None:
    lines = [
        "# R/S/O 离线比较结果",
        "",
        "本结果基于正式 12 个 `abcd_transport_baseline.csv`，使用同一几何路径和 1 ms 连续评价。",
        "",
        f"- gate_pass: `{gate['gate_pass']}`",
        f"- processed: `{gate['processed_runs']}/{gate['formal_runs']}`",
        f"- median O/S RMS: `{gate['median_o_vs_s_rms_pct']}` %",
        f"- blocking runs: `{gate['blocking_runs'] or 'none'}`",
        "",
        "| 组 | n | O 接受 | O/S RMS 中位数 | O/S RMS 范围 |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in groups:
        lines.append(
            f"| {row['group']} | {row['n']} | {row['o_accepted_n']} | "
            f"{row['o_vs_s_rms_pct_median']:.4f}% | "
            f"{row['o_vs_s_rms_pct_min']:.4f}% ～ {row['o_vs_s_rms_pct_max']:.4f}% |"
        )
    lines += [
        "",
        "S 是时间统一伸时；O 是同一 1.2T 预算下的时间剖面优化。指标是计划参考曲线指标，不是机械臂实测 jerk。",
        "B1 的正式失败状态必须与离线候选接受分开解释。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-bin", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--figure", type=Path, default=FIG)
    args = parser.parse_args()

    if not args.replay_bin.exists():
        raise FileNotFoundError(f"missing replay executable: {args.replay_bin}")
    alignment = alignment_module()
    formal = alignment.formal_rows()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    limits_path = args.out_dir / "used_joint_limits.csv"
    limit_rows = alignment.build_limit_rows()
    write_csv(
        limits_path,
        [
            "joint_name",
            "min_position",
            "max_position",
            "min_velocity",
            "max_velocity",
            "min_acceleration",
            "max_acceleration",
        ],
        limit_rows,
    )

    rows: list[dict[str, object]] = []
    missing: list[dict[str, object]] = []
    failed: list[dict[str, object]] = []

    for formal_row in formal:
        run_id = formal_row["run_id"]
        group = formal_row["group"]
        run_dir = ROOT / "experiment_results/a1_runs" / run_id
        trajectory = run_dir / "abcd_transport_baseline.csv"
        if not run_dir.is_dir() or not trajectory.is_file():
            missing.append(
                {
                    "run_id": run_id,
                    "group": group,
                    "path": str(trajectory),
                    "reason": "run directory or baseline trajectory missing",
                }
            )
            continue

        run_out = args.out_dir / run_id
        r_out = run_out / "R"
        so_out = run_out / "S_O"
        try:
            r_summary = run_replay(args.replay_bin, trajectory, limits_path, r_out, 1.0)
            so_summary = run_replay(args.replay_bin, trajectory, limits_path, so_out, SCALE)
        except Exception as exc:
            failed.append({"run_id": run_id, "group": group, "reason": str(exc)})
            continue

        rms_r = as_float(r_summary["baseline_rms"])
        rms_s = as_float(so_summary["baseline_rms"])
        rms_o = as_float(so_summary["candidate_rms"])
        peak_r = as_float(r_summary["baseline_peak"])
        peak_s = as_float(so_summary["baseline_peak"])
        peak_o = as_float(so_summary["candidate_peak"])
        duration_r = as_float(r_summary["target_duration_s"])
        duration_s = as_float(so_summary["target_duration_s"])
        duration_o = duration_s
        endpoint_diff = endpoint_difference(
            r_out / "time_profile_baseline.csv", so_out / "time_profile_baseline.csv"
        )
        s_rms_ratio = rms_s / rms_r if rms_r != 0.0 else 0.0
        s_peak_ratio = peak_s / peak_r if peak_r != 0.0 else 0.0
        s_rms_scale_ok = abs(s_rms_ratio - THEORETICAL_JERK_RATIO) <= S_RMS_RATIO_TOLERANCE
        s_peak_scale_ok = abs(s_peak_ratio - THEORETICAL_JERK_RATIO) <= S_PEAK_RATIO_TOLERANCE
        s_scale_ok = s_rms_scale_ok and s_peak_scale_ok
        s_duration_ok = abs(duration_s / duration_r - SCALE) <= 1e-9
        endpoints_ok = endpoint_diff <= 1e-10
        o_accepted = bool_int(so_summary["accepted"])
        o_constraints_ok = bool_int(so_summary["constraints_ok"])
        o_rms_improved = bool_int(so_summary["rms_improved"])
        o_peak_ok = bool_int(so_summary["peak_ok"])
        o_export_ok = bool_int(so_summary["export_fidelity_ok"])
        o_gate_ok = (
            o_accepted == 1
            and o_constraints_ok == 1
            and o_rms_improved == 1
            and o_peak_ok == 1
            and o_export_ok == 1
            and s_scale_ok
            and s_duration_ok
            and endpoints_ok
        )
        blocking = []
        if not s_rms_scale_ok:
            blocking.append("S_rms_scale_ratio")
        if not s_peak_scale_ok:
            blocking.append("S_peak_scale_ratio")
        if not s_duration_ok:
            blocking.append("S_duration")
        if not endpoints_ok:
            blocking.append("S_endpoints")
        if o_accepted != 1:
            blocking.append("O_not_accepted")
        if o_constraints_ok != 1:
            blocking.append("O_constraints")
        if o_rms_improved != 1:
            blocking.append("O_rms_not_reduced")
        if o_peak_ok != 1:
            blocking.append("O_peak")
        if o_export_ok != 1:
            blocking.append("O_export_fidelity")

        rows.append(
            {
                "run_id": run_id,
                "group": group,
                "formal_state": formal_row.get("state", ""),
                "completed_release_retreat": formal_row.get("completed_release_retreat", ""),
                "T_R_s": duration_r,
                "T_S_s": duration_s,
                "T_O_s": duration_o,
                "rms_R": rms_r,
                "rms_S": rms_s,
                "rms_O": rms_o,
                "peak_R": peak_r,
                "peak_S": peak_s,
                "peak_O": peak_o,
                "s_vs_r_rms_pct": percent_change(rms_s, rms_r),
                "s_vs_r_peak_pct": percent_change(peak_s, peak_r),
                "o_vs_s_rms_pct": percent_change(rms_o, rms_s),
                "o_vs_s_peak_pct": percent_change(peak_o, peak_s),
                "s_rms_ratio": s_rms_ratio,
                "s_peak_ratio": s_peak_ratio,
                "s_duration_ratio": duration_s / duration_r if duration_r != 0.0 else 0.0,
                "s_rms_scale_ok": int(s_rms_scale_ok),
                "s_peak_scale_ok": int(s_peak_scale_ok),
                "s_scale_ok": int(s_scale_ok),
                "s_duration_ok": int(s_duration_ok),
                "s_endpoint_difference_rad": endpoint_diff,
                "s_endpoints_ok": int(endpoints_ok),
                "r_command_interp_rms": r_summary["baseline_command_interp_rms"],
                "r_command_interp_peak": r_summary["baseline_command_interp_peak"],
                "s_command_interp_rms": so_summary["baseline_command_interp_rms"],
                "s_command_interp_peak": so_summary["baseline_command_interp_peak"],
                "o_command_interp_rms": so_summary["candidate_command_interp_rms"],
                "o_command_interp_peak": so_summary["candidate_command_interp_peak"],
                "o_accepted": o_accepted,
                "o_selected": int(so_summary["selected_plan"] == "candidate"),
                "o_constraints_ok": o_constraints_ok,
                "o_rms_improved": o_rms_improved,
                "o_peak_ok": o_peak_ok,
                "o_export_fidelity_ok": o_export_ok,
                "o_export_spacing_used": so_summary["export_spacing_used"],
                "o_export_deviation_rad": so_summary["candidate_export_deviation_rad"],
                "r_solve_sec": r_summary["solve_sec"],
                "s_o_solve_sec": so_summary["solve_sec"],
                "fallback_reason": so_summary["fallback_reason"],
                "o_gate_ok": int(o_gate_ok),
                "blocking_reason": ";".join(blocking),
                "replay_R_dir": str(r_out),
                "replay_S_O_dir": str(so_out),
            }
        )

    rows.sort(key=lambda row: (str(row["group"]), str(row["run_id"])))
    fields = [
        "run_id", "group", "formal_state", "completed_release_retreat",
        "T_R_s", "T_S_s", "T_O_s", "rms_R", "rms_S", "rms_O",
        "peak_R", "peak_S", "peak_O", "s_vs_r_rms_pct", "s_vs_r_peak_pct",
        "o_vs_s_rms_pct", "o_vs_s_peak_pct", "s_rms_ratio", "s_peak_ratio",
        "s_duration_ratio", "s_rms_scale_ok", "s_peak_scale_ok", "s_scale_ok", "s_duration_ok",
        "s_endpoint_difference_rad", "s_endpoints_ok",
        "r_command_interp_rms", "r_command_interp_peak",
        "s_command_interp_rms", "s_command_interp_peak",
        "o_command_interp_rms", "o_command_interp_peak",
        "o_accepted", "o_selected", "o_constraints_ok", "o_rms_improved",
        "o_peak_ok", "o_export_fidelity_ok", "o_export_spacing_used",
        "o_export_deviation_rad", "r_solve_sec", "s_o_solve_sec",
        "fallback_reason", "o_gate_ok", "blocking_reason",
        "replay_R_dir", "replay_S_O_dir",
    ]
    write_csv(args.out_dir / "rs_o_by_run.csv", fields, rows)
    groups = group_summary(rows)
    write_csv(
        args.out_dir / "rs_o_group_summary.csv",
        list(groups[0].keys()) if groups else ["group", "n"],
        groups,
    )
    write_csv(
        args.out_dir / "missing_runs.csv",
        ["run_id", "group", "path", "reason"],
        missing,
    )
    write_csv(
        args.out_dir / "failed_runs.csv",
        ["run_id", "group", "reason"],
        failed,
    )

    o_values = [float(row["o_vs_s_rms_pct"]) for row in rows]
    median_o = statistics.median(o_values) if o_values else float("nan")
    min_o = min(o_values) if o_values else float("nan")
    max_o = max(o_values) if o_values else float("nan")
    all_inputs = len(rows) == 12 and not missing and not failed
    all_s_rms_scale = all(int(row["s_rms_scale_ok"]) == 1 for row in rows)
    all_s_peak_scale = all(int(row["s_peak_scale_ok"]) == 1 for row in rows)
    all_s_duration = all(int(row["s_duration_ok"]) == 1 for row in rows)
    all_s_endpoints = all(int(row["s_endpoints_ok"]) == 1 for row in rows)
    all_scale = all_s_rms_scale and all_s_peak_scale and all_s_duration and all_s_endpoints
    all_o = all(int(row["o_gate_ok"]) == 1 for row in rows)
    gate_pass = all_inputs and all_scale and all_o and median_o <= -5.0
    blockers = []
    if missing:
        blockers.extend(f"missing:{row['run_id']}" for row in missing)
    if failed:
        blockers.extend(f"failed:{row['run_id']}" for row in failed)
    blockers.extend(
        f"{row['run_id']}:{row['blocking_reason']}"
        for row in rows
        if row["blocking_reason"]
    )
    if all_inputs and all_scale and all_o and median_o > -5.0:
        blockers.append("median_O_vs_S_above_threshold")
    gate_reason = "passed" if gate_pass else ";".join(blockers) or "gate_not_satisfied"
    gate = {
        "gate_pass": int(gate_pass),
        "reason": gate_reason,
        "formal_runs": len(formal),
        "processed_runs": len(rows),
        "missing_runs": len(missing),
        "failed_runs": len(failed),
        "all_s_rms_scale_ok": int(all_s_rms_scale),
        "all_s_peak_scale_ok": int(all_s_peak_scale),
        "all_s_duration_ok": int(all_s_duration),
        "all_s_endpoints_ok": int(all_s_endpoints),
        "all_s_scale_ok": int(all_scale),
        "all_o_candidates_ok": int(all_o),
        "median_o_vs_s_rms_pct": median_o,
        "min_o_vs_s_rms_pct": min_o,
        "max_o_vs_s_rms_pct": max_o,
        "s_theoretical_ratio": THEORETICAL_JERK_RATIO,
        "s_rms_ratio_tolerance": S_RMS_RATIO_TOLERANCE,
        "s_peak_ratio_tolerance": S_PEAK_RATIO_TOLERANCE,
        "blocking_runs": ";".join(blockers),
    }
    write_csv(args.out_dir / "rs_o_gate.csv", list(gate.keys()), [gate])
    write_report(args.out_dir / "rs_o_report.md", gate, groups)
    make_figure(rows, args.figure)

    print(f"formal runs: {len(formal)}")
    print(f"processed: {len(rows)}")
    print(f"missing: {len(missing)}")
    print(f"failed: {len(failed)}")
    print(f"all S/R scale checks: {int(all_scale)}")
    print(f"all O candidates: {int(all_o)}")
    print(f"median O/S RMS change: {median_o:.4f}%")
    print(f"gate_pass: {int(gate_pass)}")
    print(f"by-run: {args.out_dir / 'rs_o_by_run.csv'}")
    print(f"gate: {args.out_dir / 'rs_o_gate.csv'}")
    print(f"figure: {args.figure}")
    # A closed gate is a valid experimental result, not an execution error.
    return 1 if (missing or failed) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
