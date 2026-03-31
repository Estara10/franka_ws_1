#!/bin/bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="${WORKSPACE_DIR}/experiment_results"
REPORT_DIR="${RESULTS_DIR}/reports"

MODE="${1:-rigid}"
TARGET_MS="${2:-100.0}"
ENABLE_ROTATE="${ENABLE_ROTATE:-false}"

mode_to_selection() {
    case "$1" in
        rigid) echo "1" ;;
        compliant) echo "2" ;;
        chomp_only) echo "3" ;;
        compliant_chomp) echo "4" ;;
        *)
            echo "Unsupported mode: $1" >&2
            echo "Supported modes: rigid | compliant | chomp_only | compliant_chomp" >&2
            exit 1
            ;;
    esac
}

mkdir -p "$REPORT_DIR"

selection="$(mode_to_selection "$MODE")"

echo "========================================"
echo "Latency benchmark run"
echo "  mode: ${MODE}"
echo "  target_latency_ms: ${TARGET_MS}"
echo "========================================"

(
    cd "$WORKSPACE_DIR"
    printf "A\n%s\n" "$selection" | ENABLE_SENSOR_GUI=false ENABLE_ROTATE="$ENABLE_ROTATE" ./start_dual_arm_task.sh
)

latency_summary_csv=$(ls -t "${REPORT_DIR}"/comm_latency_*_summary.csv 2>/dev/null | head -1 || true)
latency_summary_md=$(ls -t "${REPORT_DIR}"/comm_latency_*_summary.md 2>/dev/null | head -1 || true)

if [ -z "$latency_summary_csv" ]; then
    echo "Failed to find latency summary csv in ${REPORT_DIR}" >&2
    exit 1
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
validation_csv="${REPORT_DIR}/latency_validation_${MODE}_${timestamp}_summary.csv"
validation_md="${REPORT_DIR}/latency_validation_${MODE}_${timestamp}_summary.md"

python3 - "$latency_summary_csv" "$TARGET_MS" "$validation_csv" "$validation_md" "$MODE" "$latency_summary_md" <<'PY'
import csv
import math
import sys

latency_csv = sys.argv[1]
target_ms = float(sys.argv[2])
out_csv = sys.argv[3]
out_md = sys.argv[4]
mode = sys.argv[5]
latency_md = sys.argv[6]

metrics = {}
with open(latency_csv, "r", newline="") as f:
    reader = csv.reader(f)
    header = next(reader, None)
    if header is None or len(header) < 2:
        raise RuntimeError(f"Invalid latency summary file: {latency_csv}")
    for row in reader:
        if len(row) < 2:
            continue
        metrics[row[0].strip()] = row[1].strip()

def get_float(name: str) -> float:
    try:
        return float(metrics.get(name, "nan"))
    except ValueError:
        return float("nan")

p95 = get_float("p95_ms")
p99 = get_float("p99_ms")
mean = get_float("mean_ms")
max_v = get_float("max_ms")
sample_count = metrics.get("sample_count", "0")

p95_pass = (not math.isnan(p95)) and (p95 <= target_ms)
max_pass = (not math.isnan(max_v)) and (max_v <= target_ms)
overall_pass = p95_pass and max_pass

rows = [
    ("mode", mode),
    ("latency_summary_csv", latency_csv),
    ("latency_summary_md", latency_md if latency_md else "N/A"),
    ("sample_count", sample_count),
    ("target_latency_ms", f"{target_ms:.3f}"),
    ("mean_ms", f"{mean:.3f}" if not math.isnan(mean) else "nan"),
    ("p95_ms", f"{p95:.3f}" if not math.isnan(p95) else "nan"),
    ("p99_ms", f"{p99:.3f}" if not math.isnan(p99) else "nan"),
    ("max_ms", f"{max_v:.3f}" if not math.isnan(max_v) else "nan"),
    ("p95_within_target", "PASS" if p95_pass else "FAIL"),
    ("max_within_target", "PASS" if max_pass else "FAIL"),
    ("overall_pass", "PASS" if overall_pass else "FAIL"),
]

with open(out_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["metric", "value"])
    writer.writerows(rows)

with open(out_md, "w", encoding="utf-8") as f:
    f.write("# Latency Validation Summary\n\n")
    f.write("| Metric | Value |\n")
    f.write("|---|---:|\n")
    for k, v in rows:
        f.write(f"| {k} | {v} |\n")

print("Latency validation generated:")
print(f"  CSV: {out_csv}")
print(f"  MD:  {out_md}")
print(f"  Overall: {'PASS' if overall_pass else 'FAIL'}")
PY

echo ""
echo "Latency benchmark finished."
echo "  Monitor report:    ${latency_summary_csv}"
echo "  Validation report: ${validation_csv}"
