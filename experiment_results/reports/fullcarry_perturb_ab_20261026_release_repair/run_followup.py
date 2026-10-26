#!/usr/bin/env python3
"""Two predeclared B follow-ups; never replaces the original formal attempts."""
import csv
import importlib.util
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
ORIGINAL = OUT.parent / "fullcarry_perturb_ab_20261026"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    runner = load("original_perturb_runner", ORIGINAL / "run_batch.py")
    runner.OUT = OUT
    runner.LEDGER = OUT / "attempts.csv"
    runner.PREFIX = OUT.name + "_validated"
    with (ORIGINAL / "schedule.csv").open(newline="") as handle:
        selected = [r for r in csv.DictReader(handle) if r["sequence"] in {"2", "7"}]
    for item in selected:
        previous = [r for r in runner.rows() if r["phase"] == "validation"
                    and r["sequence"] == item["sequence"]]
        if any(r["action_started"] == "1" for r in previous):
            continue
        row = runner.run_one("validation", item, len(previous) + 1)
        if row["action_started"] != "1":
            return 2
    analyzer = load("original_perturb_analyzer", ORIGINAL / "analyze_batch.py")
    rows = []
    for attempt in runner.rows():
        if attempt["phase"] != "validation" or attempt["action_started"] != "1":
            continue
        _, _, metrics = analyzer.process_run(attempt)
        row = dict(attempt)
        row.update({key: value for key, value in metrics.items() if key != "text"})
        rows.append(row)
    analyzer.write_csv(OUT / "followup_metrics.csv", rows)
    # This file describes available placement measurements, not a success rate.
    original = analyzer.read_csv(ORIGINAL / "formal_attempts.csv")
    measurements = [r for r in original + rows if r["condition"] == "B"
                    and r["flow_completed"] == "1"]
    fields = ["phase", "run_id", "flow_completed", "release_position_error_mm",
              "release_target_yaw_abs_error_deg", "task_wall_time"]
    analyzer.write_csv(OUT / "b_completed_placement_measurements.csv", measurements, fields)
    print(f"FOLLOWUP completed={sum(r['flow_completed'] == '1' for r in rows)}/{len(rows)}; "
          f"original_B=8/10; total_B={len(measurements)}/{10 + len(rows)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
