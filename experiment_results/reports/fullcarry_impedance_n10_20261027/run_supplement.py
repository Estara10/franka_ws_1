#!/usr/bin/env python3
"""One additional run authorized after B06 stopped; keep the original ten."""
import importlib.util
from pathlib import Path

BASE = Path(__file__).resolve().parent
OUT = BASE / "supplement"
SOURCE = BASE.parent / "fullcarry_perturb_ab_20261026" / "run_batch.py"


def main():
    OUT.mkdir(exist_ok=True)
    spec = importlib.util.spec_from_file_location("existing_carry_runner", SOURCE)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.OUT = OUT
    runner.LEDGER = OUT / "attempts.csv"
    runner.PREFIX = BASE.name + "_supplement"
    if runner.rows():
        print("The one additional run is already recorded; no automatic replacement")
        return 0
    item = dict(sequence="11", block="supplement", condition="B", internal_group="C",
                start_x_m="0.500", start_y_m="0.000", start_yaw_deg="10.0",
                target_x_m="0.400", target_y_m="0.000", target_yaw_deg="20.0")
    row = runner.run_one("supplement", item, 1)
    return 0 if row["flow_completed"] == "1" else 1


if __name__ == "__main__":
    raise SystemExit(main())
