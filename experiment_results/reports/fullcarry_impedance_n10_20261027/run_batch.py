#!/usr/bin/env python3
"""Run the ten fixed impedance/payload full-carry attempts sequentially."""
import csv
import importlib.util
from pathlib import Path

OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent / "fullcarry_perturb_ab_20261026" / "run_batch.py"


def main():
    spec = importlib.util.spec_from_file_location("existing_carry_runner", SOURCE)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.OUT = OUT
    runner.LEDGER = OUT / "attempts.csv"
    runner.PREFIX = OUT.name
    with (OUT / "schedule.csv").open(newline="") as handle:
        schedule = list(csv.DictReader(handle))
    for item in schedule:
        previous = [r for r in runner.rows() if r["sequence"] == item["sequence"]]
        if any(r["action_started"] == "1" for r in previous):
            continue
        if any(r["status"] == "reserved" for r in previous):
            raise RuntimeError("An interrupted launch needs its actual result recorded before resuming")
        launches = len(previous)
        while launches < 3:
            launches += 1
            result = runner.run_one("formal", item, launches)
            if result["action_started"] == "1":
                break
            print("No accepted motion; keeping startup record and continuing the same scheduled slot", flush=True)
        else:
            return 2
    rows = [r for r in runner.rows() if r["action_started"] == "1"]
    done = sum(r["flow_completed"] == "1" for r in rows)
    print(f"B batch complete: {done}/{len(rows)} complete full-carry tasks", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
