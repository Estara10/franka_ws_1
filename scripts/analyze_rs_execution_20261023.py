#!/usr/bin/env python3
"""Summarize the four supervised R/S full-carry executions.

The four expected tags are R1, S1, S2 and R2. The script reads only saved run
artifacts and never starts ROS or MuJoCo.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import statistics
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_rs_execution_20261023")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scientific_style import NATURE, configure_scientific_style

configure_scientific_style()

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "experiment_results/a1_runs"
OUT = ROOT / "experiment_results/reports/final_tradeoff_20261022/rs_execution"
FIG = ROOT / "docs/figures/辅助图/02_正常与慢速搬运对照.png"
TAGS = ("R1", "S1", "S2", "R2")
TARGET_XY = (0.40, 0.0)
TARGET_YAW_DEG = 20.0
ROD_DIMS = (0.04, 0.40, 0.04)
SUPPORT_Z = 0.0


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def wrapped(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def bottom_extent(q: list[float]) -> float:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)
    return 0.5 * (
        abs(r20) * ROD_DIMS[0]
        + abs(r21) * ROD_DIMS[1]
        + abs(r22) * ROD_DIMS[2]
    )


def find_run(tag: str) -> Path:
    matches = sorted(RUNS.glob(f"today_rs_{tag}_*"))
    if not matches:
        raise FileNotFoundError(f"missing run directory for {tag}")
    return matches[-1]


def read_meta(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def action_durations(log: str) -> list[float]:
    accepted = [
        float(value)
        for value in re.findall(
            r"\[INFO\] \[([0-9.]+)\].*?\[move_group_interface\]: Execute request accepted",
            log,
        )
    ]
    succeeded = [
        float(value)
        for value in re.findall(
            r"\[INFO\] \[([0-9.]+)\].*?\[move_group_interface\]: Execute request success!",
            log,
        )
    ]
    return [end - start for start, end in zip(accepted, succeeded) if end >= start]


def parse_run(tag: str) -> dict[str, object]:
    run_dir = find_run(tag)
    log = (run_dir / "task_launch.log").read_text(errors="replace")
    status = json.loads((run_dir / "prepare_status.json").read_text(encoding="utf-8"))
    meta = read_meta(run_dir / "contact_transport_meta.txt")
    probe_rows = read_csv(run_dir / "contact_probe.csv")

    plans = {
        match.group(1): float(match.group(2))
        for match in re.finditer(
            r"\[CONTACT_PLAN_TIMING\] label=(lift|transport|lower|retreat).*?final=([0-9.]+)",
            log,
        )
    }
    durations = action_durations(log)
    action_transport = durations[3] if len(durations) >= 4 else ""
    action_lift = durations[2] if len(durations) >= 3 else ""
    action_lower = durations[4] if len(durations) >= 5 else ""
    action_retreat = durations[5] if len(durations) >= 6 else ""

    result_matches = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+)", log)
    probe_state = result_matches[-1] if result_matches else status.get("task_state", "UNKNOWN")
    release_matches = list(
        re.finditer(
            r"\[A1\] 读回原始值: object_frame='base_link'.*?pose=\(([^)]+)\) quat=\(([^)]+)\)",
            log,
        )
    )
    released_match = re.search(r"\[CONTACT_PROBE_RELEASED\] xyz=\(([^)]+)\)", log)
    released_line = re.search(r"\[CONTACT_PROBE_RELEASED\].*", log)
    release_center = None
    release_quat = None
    if released_match and release_matches:
        before = [match for match in release_matches if match.start() < released_match.start()]
        if before:
            release_center = [float(value.strip()) for value in before[-1].group(1).split(",")]
            release_quat = [float(value.strip()) for value in before[-1].group(2).split(",")]

    release_xy_error = ""
    release_yaw_error = ""
    bottom_gap = ""
    release_xyz = ""
    release_yaw = ""
    if release_center is not None and release_quat is not None:
        release_xy_error = math.hypot(
            release_center[0] - TARGET_XY[0],
            release_center[1] - TARGET_XY[1],
        ) * 1000.0
        release_yaw = math.degrees(quaternion_yaw(*release_quat))
        release_yaw_error = abs(
            math.degrees(wrapped(math.radians(release_yaw) - math.radians(TARGET_YAW_DEG)))
        )
        bottom_gap = (
            release_center[2] - bottom_extent(release_quat) - SUPPORT_Z
        ) * 1000.0
        release_xyz = ";".join(f"{value:.6f}" for value in release_center)

    phases: dict[str, list[dict[str, str]]] = {}
    for row in probe_rows:
        phases.setdefault(row["phase"], []).append(row)
    hold_rows = phases.get("HOLD", [])
    transport_rows = phases.get("TRANSPORT", [])
    lift_rows = phases.get("LIFT", [])
    lower_rows = phases.get("LOWER", [])
    hold_dz = (
        float(hold_rows[-1]["rod_z"]) - float(hold_rows[0]["rod_z"])
        if hold_rows
        else ""
    )
    lift_height = (
        float(lift_rows[-1]["rod_z"]) - float(lift_rows[0]["rod_z"])
        if lift_rows
        else ""
    )
    transport_actual_dx = (
        float(transport_rows[-1]["rod_x"]) - float(transport_rows[0]["rod_x"])
        if transport_rows
        else ""
    )
    transport_actual_dy = (
        float(transport_rows[-1]["rod_y"]) - float(transport_rows[0]["rod_y"])
        if transport_rows
        else ""
    )
    lower_end_z = float(lower_rows[-1]["rod_z"]) if lower_rows else ""

    max_slip_match = re.search(r"\[CONTACT_LOAD_VALIDATED\].*?max_slip=([0-9.]+)mm", log)
    max_angle_match = re.search(r"\[CONTACT_LOAD_VALIDATED\].*?max_angle=([0-9.]+)deg", log)
    config_match = re.search(r"\[CONTACT_PROBE_CONFIG\].*$", log, re.MULTILINE)
    result_match = re.search(
        r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_RESULT\]", log
    )
    config_stamp = re.search(
        r"\[INFO\] \[([0-9.]+)\].*?\[CONTACT_PROBE_CONFIG\]", log
    )
    total_elapsed = (
        float(result_match.group(1)) - float(config_stamp.group(1))
        if result_match and config_stamp
        else ""
    )
    last = probe_rows[-1] if probe_rows else {}
    weld_ok = (
        str(last.get("weld_left", "0")) == "0"
        and str(last.get("weld_right", "0")) == "0"
        and str(last.get("rod_constraint", "0")) == "0"
    )
    released_ok = (
        released_line is not None
        and "open=true" in released_line.group(0)
        and "weld=OFF" in released_line.group(0)
        and "attach=none" in released_line.group(0)
    )
    retreat_ok = "[CONTACT_PROBE_RETREAT] hands lifted 0.150 m" in log and "state=retreated" in log
    completion_ok = probe_state == "DONE" and status.get("task_state") == "DONE"
    planned_duration = float(meta.get("planned_duration_s", "nan"))
    ratio = (
        float(action_transport) / planned_duration
        if action_transport != "" and planned_duration > 0.0
        else ""
    )

    return {
        "tag": tag,
        "mode": "R" if tag.startswith("R") else "S",
        "run_id": run_dir.name,
        "run_dir": str(run_dir),
        "task_state": status.get("task_state", "UNKNOWN"),
        "probe_state": probe_state,
        "transport_time_scale": float(meta.get("transport_time_scale", "nan")),
        "transport_stretch": float(meta.get("transport_stretch", "nan")),
        "base_plan_duration_s": float(meta.get("base_plan_duration_s", "nan")),
        "planned_duration_s": planned_duration,
        "transport_plan_s": plans.get("transport", ""),
        "transport_action_wall_s": action_transport,
        "transport_action_vs_plan_ratio": ratio,
        "lift_action_wall_s": action_lift,
        "lower_action_wall_s": action_lower,
        "retreat_action_wall_s": action_retreat,
        "total_elapsed_wall_s": total_elapsed,
        "transport_actual_dx_m": transport_actual_dx,
        "transport_actual_dy_m": transport_actual_dy,
        "hold_dz_mm": hold_dz * 1000.0 if hold_dz != "" else "",
        "lift_height_m": lift_height,
        "lower_end_z_m": lower_end_z,
        "release_xyz_m": release_xyz,
        "release_yaw_deg": release_yaw,
        "release_xy_error_mm": release_xy_error,
        "release_yaw_error_deg": release_yaw_error,
        "rod_bottom_gap_mm": bottom_gap,
        "max_hold_slip_mm": float(max_slip_match.group(1)) if max_slip_match else "",
        "max_hold_angle_deg": float(max_angle_match.group(1)) if max_angle_match else "",
        "weld_constraint_off": int(weld_ok),
        "release_ok": int(released_ok),
        "retreat_ok": int(retreat_ok),
        "completion_ok": int(completion_ok),
    }


def group_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for mode in ("R", "S"):
        values = [row for row in rows if row["mode"] == mode]
        result.append(
            {
                "mode": mode,
                "n": len(values),
                "completed_n": sum(int(row["completion_ok"]) for row in values),
                "transport_plan_s_mean": statistics.mean(
                    float(row["planned_duration_s"]) for row in values
                ),
                "transport_action_wall_s_mean": statistics.mean(
                    float(row["transport_action_wall_s"]) for row in values
                ),
                "transport_action_wall_s_median": statistics.median(
                    float(row["transport_action_wall_s"]) for row in values
                ),
                "release_xy_error_mm_mean": statistics.mean(
                    float(row["release_xy_error_mm"]) for row in values
                ),
                "release_yaw_error_deg_mean": statistics.mean(
                    float(row["release_yaw_error_deg"]) for row in values
                ),
                "rod_bottom_gap_mm_mean": statistics.mean(
                    float(row["rod_bottom_gap_mm"]) for row in values
                ),
                "max_hold_slip_mm_mean": statistics.mean(
                    float(row["max_hold_slip_mm"]) for row in values
                ),
                "total_elapsed_wall_s_mean": statistics.mean(
                    float(row["total_elapsed_wall_s"]) for row in values
                ),
            }
        )
    return result


def make_figure(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    tags = [str(row["tag"]) for row in rows]
    x = list(range(len(rows)))
    axes[0].bar(
        x,
        [float(row["transport_action_wall_s"]) for row in rows],
        color=[NATURE["blue"] if row["mode"] == "R" else NATURE["red"] for row in rows],
    )
    axes[0].plot(
        x,
        [float(row["planned_duration_s"]) for row in rows],
        color="#222222",
        marker="x",
        linestyle="--",
        label="planned transport",
    )
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(tags)
    axes[0].set_ylabel("Transport time (s)")
    axes[0].set_title("Planned vs MoveGroup action wall time")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)

    axes[1].scatter(
        [float(row["release_xy_error_mm"]) for row in rows],
        [float(row["release_yaw_error_deg"]) for row in rows],
        c=[NATURE["blue"] if row["mode"] == "R" else NATURE["red"] for row in rows],
        s=45,
    )
    for x_value, y_value, tag in zip(
        [float(row["release_xy_error_mm"]) for row in rows],
        [float(row["release_yaw_error_deg"]) for row in rows],
        tags,
    ):
        axes[1].annotate(tag, (x_value, y_value), xytext=(4, 4), textcoords="offset points")
    axes[1].set_xlabel("Release XY error (mm)")
    axes[1].set_ylabel("Release yaw error (deg)")
    axes[1].set_title("Release placement")
    axes[1].grid(alpha=0.25)

    fig.suptitle("R/S full-carry execution; descriptive n=2 per mode", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    rows = [parse_run(tag) for tag in TAGS]
    OUT.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    write_csv(OUT / "rs_execution.csv", fields, rows)
    summary = group_summary(rows)
    write_csv(OUT / "rs_execution_group_summary.csv", list(summary[0].keys()), summary)
    make_figure(rows, FIG)
    print(f"runs: {len(rows)}")
    for row in rows:
        print(
            row["tag"],
            "state=", row["probe_state"],
            "scale=", row["transport_time_scale"],
            "plan_s=", row["planned_duration_s"],
            "action_s=", row["transport_action_wall_s"],
            "release_xy_mm=", row["release_xy_error_mm"],
            "release_yaw_deg=", row["release_yaw_error_deg"],
            "gap_mm=", row["rod_bottom_gap_mm"],
        )
    print(f"csv: {OUT / 'rs_execution.csv'}")
    print(f"summary: {OUT / 'rs_execution_group_summary.csv'}")
    print(f"figure: {FIG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
