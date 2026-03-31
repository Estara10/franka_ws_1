#!/usr/bin/env python3
import argparse
import csv
import datetime
import os
from typing import Dict, Tuple


def read_metric_csv(path: str) -> Dict[str, str]:
    metrics: Dict[str, str] = {}
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None or len(header) < 2 or header[0] != "metric":
            raise ValueError(f"Invalid metric csv format: {path}")
        for row in reader:
            if len(row) < 2:
                continue
            metrics[row[0].strip()] = row[1].strip()
    return metrics


def parse_float(metrics: Dict[str, str], key: str) -> float:
    value = metrics.get(key, "nan")
    try:
        return float(value)
    except ValueError:
        return float("nan")


def bool_pass(expr: bool) -> str:
    return "PASS" if expr else "FAIL"


def write_metric_csv(path: str, rows: Tuple[Tuple[str, str], ...]) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        for k, v in rows:
            writer.writerow([k, v])


def write_metric_md(path: str, rows: Tuple[Tuple[str, str], ...], title: str) -> None:
    with open(path, "w") as f:
        f.write(f"# {title}\n\n")
        f.write("| Metric | Value |\n")
        f.write("|---|---:|\n")
        for k, v in rows:
            f.write(f"| {k} | {v} |\n")


def main() -> None:
    default_output_dir = os.path.join(os.getcwd(), "experiment_results", "reports")
    parser = argparse.ArgumentParser(description="Compare dual-arm task metrics reports.")
    parser.add_argument("--baseline", required=True, help="Baseline metrics summary csv")
    parser.add_argument("--optimized", required=True, help="Optimized metrics summary csv")
    parser.add_argument("--output-dir", default=default_output_dir, help="Output directory")
    parser.add_argument("--prefix", default="benchmark_comparison", help="Output file prefix")
    parser.add_argument("--sync-target-mm", type=float, default=5.0, help="Sync error threshold")
    parser.add_argument("--ee-target-mm", type=float, default=2.0, help="EE error threshold")
    parser.add_argument("--energy-improve-target-pct", type=float, default=10.0,
                        help="Energy improvement threshold in percent")
    args = parser.parse_args()

    baseline = read_metric_csv(args.baseline)
    optimized = read_metric_csv(args.optimized)

    baseline_energy = parse_float(baseline, "energy_proxy_j")
    optimized_energy = parse_float(optimized, "energy_proxy_j")
    optimized_sync_p95 = parse_float(optimized, "sync_error_p95_mm")
    optimized_ee = parse_float(optimized, "ee_terminal_error_mm")

    energy_reduction_pct = float("nan")
    if baseline_energy > 0.0 and optimized_energy == optimized_energy:
        energy_reduction_pct = (baseline_energy - optimized_energy) / baseline_energy * 100.0

    sync_pass = (optimized_sync_p95 == optimized_sync_p95) and (optimized_sync_p95 <= args.sync_target_mm)
    ee_pass = (optimized_ee == optimized_ee) and (optimized_ee <= args.ee_target_mm)
    energy_pass = (energy_reduction_pct == energy_reduction_pct) and (
        energy_reduction_pct >= args.energy_improve_target_pct
    )
    overall_pass = sync_pass and ee_pass and energy_pass

    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(args.output_dir, exist_ok=True)
    out_csv = os.path.join(args.output_dir, f"{args.prefix}_{now}_summary.csv")
    out_md = os.path.join(args.output_dir, f"{args.prefix}_{now}_summary.md")

    rows = (
        ("baseline_summary", args.baseline),
        ("optimized_summary", args.optimized),
        ("baseline_energy_proxy_j", f"{baseline_energy:.6f}" if baseline_energy == baseline_energy else "nan"),
        ("optimized_energy_proxy_j", f"{optimized_energy:.6f}" if optimized_energy == optimized_energy else "nan"),
        ("energy_reduction_pct", f"{energy_reduction_pct:.3f}" if energy_reduction_pct == energy_reduction_pct else "nan"),
        ("energy_reduction_target_pct", f"{args.energy_improve_target_pct:.3f}"),
        ("energy_reduction_pass", bool_pass(energy_pass)),
        ("optimized_sync_error_p95_mm", f"{optimized_sync_p95:.3f}" if optimized_sync_p95 == optimized_sync_p95 else "nan"),
        ("sync_target_mm", f"{args.sync_target_mm:.3f}"),
        ("sync_pass", bool_pass(sync_pass)),
        ("optimized_ee_terminal_error_mm", f"{optimized_ee:.3f}" if optimized_ee == optimized_ee else "nan"),
        ("ee_target_mm", f"{args.ee_target_mm:.3f}"),
        ("ee_pass", bool_pass(ee_pass)),
        ("overall_pass", bool_pass(overall_pass)),
    )

    write_metric_csv(out_csv, rows)
    write_metric_md(out_md, rows, "Task Metrics Benchmark Comparison")

    print("Benchmark comparison generated:")
    print(f"  CSV: {out_csv}")
    print(f"  MD:  {out_md}")
    print(f"  Overall: {bool_pass(overall_pass)}")


if __name__ == "__main__":
    main()
