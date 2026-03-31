#!/bin/bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="${WORKSPACE_DIR}/experiment_results"
REPORT_DIR="${RESULTS_DIR}/reports"

BASELINE_MODE="${1:-rigid}"
OPTIMIZED_MODE="${2:-compliant_chomp}"
ENABLE_ROTATE="${ENABLE_ROTATE:-false}"

mode_to_selection() {
    case "$1" in
        rigid) echo "1" ;;
        compliant) echo "2" ;;
        chomp_only) echo "3" ;;
        compliant_chomp) echo "4" ;;
        *)
            echo "Unsupported mode: $1" >&2
            exit 1
            ;;
    esac
}

run_single_mode() {
    local mode="$1"
    local selection
    selection="$(mode_to_selection "$mode")"

    echo "========================================"
    echo "Running mode: ${mode}"
    echo "========================================"

    (
        cd "$WORKSPACE_DIR"
        printf "A\n%s\n" "$selection" | ENABLE_SENSOR_GUI=false ENABLE_ROTATE="$ENABLE_ROTATE" ./start_dual_arm_task.sh
    )

    local latest
    latest=$(ls -t "${REPORT_DIR}"/task_metrics_*_"${mode}"_*_summary.csv 2>/dev/null | head -1 || true)
    if [ -z "$latest" ]; then
        echo "Failed to find metrics summary for mode ${mode}" >&2
        exit 1
    fi

    echo "Metrics summary for ${mode}: ${latest}"
    MODE_SUMMARY_CSV="$latest"
}

mkdir -p "$REPORT_DIR"

run_single_mode "$BASELINE_MODE"
BASELINE_SUMMARY_CSV="$MODE_SUMMARY_CSV"

run_single_mode "$OPTIMIZED_MODE"
OPTIMIZED_SUMMARY_CSV="$MODE_SUMMARY_CSV"

(
    cd "$WORKSPACE_DIR"
    python3 compare_task_metrics.py \
        --baseline "$BASELINE_SUMMARY_CSV" \
        --optimized "$OPTIMIZED_SUMMARY_CSV" \
        --output-dir "$REPORT_DIR" \
        --prefix "task_metrics_${BASELINE_MODE}_vs_${OPTIMIZED_MODE}"
)

echo ""
echo "Benchmark pipeline finished."
echo "  Baseline summary:  $BASELINE_SUMMARY_CSV"
echo "  Optimized summary: $OPTIMIZED_SUMMARY_CSV"
