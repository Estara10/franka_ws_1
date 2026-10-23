#!/usr/bin/env python3
"""Run the corrected time-profile replay over the 12 formal ABCD runs.

This script is intentionally offline-only. It reads the saved baseline
transport plans and the existing joint-limit configuration, invokes the
installed C++ replay tool once per formal run, and writes a new aggregate under
final_tradeoff_20261022/time_profile_alignment/.
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
FORMAL_RUNS = ROOT / "experiment_results/reports/final_tradeoff_20261022/abcd_s0_formal_runs.csv"
OUT_DIR = ROOT / "experiment_results/reports/final_tradeoff_20261022/time_profile_alignment"
LIMITS_YAML = ROOT / "src/multipanda_ros2/franka_moveit_config/config/joint_limits.yaml"
URDF = ROOT / "src/multipanda_ros2/franka_description/robots/common/panda_arm.xacro"
DEFAULT_REPLAY = (
    ROOT
    / "install/dual_arm_carry_task/lib/dual_arm_carry_task/time_profile_replay"
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def formal_rows() -> list[dict[str, str]]:
    if not FORMAL_RUNS.exists():
        raise FileNotFoundError(f"missing formal run index: {FORMAL_RUNS}")
    rows = [row for row in read_csv(FORMAL_RUNS) if row.get("batch") == "formal"]
    if len(rows) != 12:
        raise ValueError(f"expected 12 formal runs, found {len(rows)} in {FORMAL_RUNS}")
    return sorted(rows, key=lambda row: (row["group"], row["run_id"]))


def parse_panda_position_limits() -> dict[str, tuple[float, float]]:
    root = ET.parse(URDF).getroot()
    positions: dict[str, tuple[float, float]] = {}
    for joint in root.iter("joint"):
        name = joint.attrib.get("name", "")
        # This source is a xacro macro with names such as
        # "${arm_id}_joint1"; the instantiated MoveIt names are
        # mj_left_joint1 / mj_right_joint1.
        match = re.search(r"_joint([1-7])$", name)
        if not match:
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise ValueError(f"missing position limit for {name}")
        positions[f"panda_joint{match.group(1)}"] = (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
    if len(positions) != 7:
        raise ValueError(f"expected 7 Panda position limits, found {len(positions)}")
    return positions


def build_limit_rows() -> list[dict[str, object]]:
    position_limits = parse_panda_position_limits()
    with LIMITS_YAML.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    configured = config.get("joint_limits", {})
    rows: list[dict[str, object]] = []
    for side in ("left", "right"):
        for index in range(1, 8):
            name = f"mj_{side}_joint{index}"
            panda_name = f"panda_joint{index}"
            limits = configured.get(name)
            if limits is None:
                raise ValueError(f"missing joint_limits entry for {name}")
            if panda_name not in position_limits:
                raise ValueError(f"missing URDF position limits for {panda_name}")
            min_position, max_position = position_limits[panda_name]
            rows.append(
                {
                    "joint_name": name,
                    "min_position": min_position,
                    "max_position": max_position,
                    "min_velocity": -float(limits["max_velocity"]),
                    "max_velocity": float(limits["max_velocity"]),
                    "min_acceleration": -float(limits["max_acceleration"]),
                    "max_acceleration": float(limits["max_acceleration"]),
                }
            )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-bin", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--max-evaluations", type=int, default=100)
    args = parser.parse_args()

    if not args.replay_bin.exists():
        raise FileNotFoundError(
            f"missing replay executable: {args.replay_bin}; build dual_arm_carry_task first"
        )

    formal = formal_rows()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    limit_rows = build_limit_rows()
    limits_path = args.out_dir / "used_joint_limits.csv"
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

    aggregate: list[dict[str, object]] = []
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

        replay_dir = args.out_dir / run_id
        replay_dir.mkdir(parents=True, exist_ok=True)
        command = [
            str(args.replay_bin),
            "--trajectory",
            str(trajectory),
            "--limits-csv",
            str(limits_path),
            "--out-dir",
            str(replay_dir),
            "--max-evaluations",
            str(args.max_evaluations),
        ]
        completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
        (replay_dir / "command.txt").write_text(" ".join(command) + "\n", encoding="utf-8")
        (replay_dir / "stdout.txt").write_text(completed.stdout, encoding="utf-8")
        (replay_dir / "stderr.txt").write_text(completed.stderr, encoding="utf-8")
        if completed.returncode != 0:
            failed.append(
                {
                    "run_id": run_id,
                    "group": group,
                    "returncode": completed.returncode,
                    "reason": completed.stderr.strip().splitlines()[-1]
                    if completed.stderr.strip()
                    else "replay failed",
                }
            )
            continue

        summary_path = replay_dir / "time_profile_summary.csv"
        if not summary_path.exists():
            failed.append(
                {
                    "run_id": run_id,
                    "group": group,
                    "returncode": completed.returncode,
                    "reason": "time_profile_summary.csv missing",
                }
            )
            continue
        summary = read_csv(summary_path)[0]
        aggregate.append(
            {
                "run_id": run_id,
                "group": group,
                "formal_state": formal_row.get("state", ""),
                "completed_release_retreat": formal_row.get(
                    "completed_release_retreat", ""
                ),
                "accepted": summary.get("accepted", ""),
                "selected_plan": summary.get("selected_plan", ""),
                "baseline_rms": summary.get("baseline_rms", ""),
                "candidate_rms": summary.get("candidate_rms", ""),
                "selected_rms": summary.get("selected_rms", ""),
                "baseline_peak": summary.get("baseline_peak", ""),
                "candidate_peak": summary.get("candidate_peak", ""),
                "selected_peak": summary.get("selected_peak", ""),
                "selected_command_interp_rms": summary.get(
                    "selected_command_interp_rms", ""
                ),
                "selected_command_interp_peak": summary.get(
                    "selected_command_interp_peak", ""
                ),
                "constraints_ok": summary.get("constraints_ok", ""),
                "rms_improved": summary.get("rms_improved", ""),
                "peak_ok": summary.get("peak_ok", ""),
                "export_fidelity_ok": summary.get("export_fidelity_ok", ""),
                "export_spacing_used": summary.get("export_spacing_used", ""),
                "baseline_export_deviation_rad": summary.get(
                    "baseline_export_deviation_rad", ""
                ),
                "candidate_export_deviation_rad": summary.get(
                    "candidate_export_deviation_rad", ""
                ),
                "evaluations": summary.get("evaluations", ""),
                "solve_sec": summary.get("solve_sec", ""),
                "fallback_reason": summary.get("fallback_reason", ""),
                "replay_dir": str(replay_dir),
            }
        )

    aggregate.sort(key=lambda row: (str(row["group"]), str(row["run_id"])))
    summary_fields = [
        "run_id",
        "group",
        "formal_state",
        "completed_release_retreat",
        "accepted",
        "selected_plan",
        "baseline_rms",
        "candidate_rms",
        "selected_rms",
        "baseline_peak",
        "candidate_peak",
        "selected_peak",
        "selected_command_interp_rms",
        "selected_command_interp_peak",
        "constraints_ok",
        "rms_improved",
        "peak_ok",
        "export_fidelity_ok",
        "export_spacing_used",
        "baseline_export_deviation_rad",
        "candidate_export_deviation_rad",
        "evaluations",
        "solve_sec",
        "fallback_reason",
        "replay_dir",
    ]
    write_csv(args.out_dir / "time_profile_alignment.csv", summary_fields, aggregate)
    write_csv(args.out_dir / "missing_runs.csv", ["run_id", "group", "path", "reason"], missing)
    write_csv(
        args.out_dir / "failed_runs.csv",
        ["run_id", "group", "returncode", "reason"],
        failed,
    )

    print(f"formal runs: {len(formal)}")
    print(f"processed: {len(aggregate)}")
    print(f"missing: {len(missing)}")
    print(f"failed: {len(failed)}")
    print(f"summary: {args.out_dir / 'time_profile_alignment.csv'}")
    if missing:
        print(f"missing file: {args.out_dir / 'missing_runs.csv'}")
    if failed:
        print(f"failed file: {args.out_dir / 'failed_runs.csv'}")
        return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
