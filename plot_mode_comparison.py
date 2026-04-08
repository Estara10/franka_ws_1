#!/usr/bin/env python3
import argparse
import csv
import datetime
import glob
import os
import re
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


MODE_ORDER = ["rigid", "compliant", "chomp_only", "compliant_chomp"]
MODE_LABEL = {
    "rigid": "Rigid",
    "compliant": "Compliant",
    "chomp_only": "CHOMP-only",
    "compliant_chomp": "Compliant+CHOMP",
}


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    low = int(k)
    high = min(low + 1, len(sorted_vals) - 1)
    frac = k - low
    return sorted_vals[low] * (1.0 - frac) + sorted_vals[high] * frac


def find_latest_mode_csv(data_dir: str, mode: str) -> str:
    # Use strict filename matching to avoid prefix collisions like
    # 'experiment_compliant_*.csv' accidentally matching
    # 'experiment_compliant_chomp_*.csv'.
    all_files = glob.glob(os.path.join(data_dir, "experiment_*.csv"))
    name_re = re.compile(rf"^experiment_{re.escape(mode)}_\d{{8}}_\d{{6}}\.csv$")
    files = [p for p in all_files if name_re.match(os.path.basename(p))]
    if not files:
        return ""
    return max(files, key=os.path.getctime)


def load_mode_metrics(csv_path: str, active_force_threshold: float, stress_threshold: float) -> Dict[str, float]:
    times: List[float] = []
    left: List[float] = []
    right: List[float] = []
    stress: List[float] = []
    accel: List[float] = []

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row.get("Time", "0")))
                ly = float(row.get("Left_Y_Force", "0"))
                ry = float(row.get("Right_Y_Force", "0"))
                left.append(ly)
                right.append(ry)
                stress.append(float(row.get("Internal_Stress", "0")))
                accel.append(float(row.get("Joint_Accel_Norm", "0")))
            except ValueError:
                continue

    if not times:
        return {
            "rows": 0.0,
            "duration_s": 0.0,
            "active_rows": 0.0,
            "active_ratio": 0.0,
            "stress_mean_active": 0.0,
            "stress_p95_active": 0.0,
            "stress_over_thr_ratio_active": 0.0,
            "accel_mean_active": 0.0,
            "accel_p95_active": 0.0,
            "asymmetry_mean_active": 0.0,
        }

    active_indices: List[int] = []
    for i in range(len(times)):
        if abs(left[i]) > active_force_threshold or abs(right[i]) > active_force_threshold:
            active_indices.append(i)

    if not active_indices:
        # Fallback to all rows, avoid empty stats
        active_indices = list(range(len(times)))

    stress_active = [stress[i] for i in active_indices]
    accel_active = [accel[i] for i in active_indices]
    asym_active = [abs(left[i] - right[i]) for i in active_indices]
    stress_over = [1.0 for s in stress_active if s > stress_threshold]

    return {
        "rows": float(len(times)),
        "duration_s": float(max(times)),
        "active_rows": float(len(active_indices)),
        "active_ratio": float(len(active_indices) / max(1, len(times))),
        "stress_mean_active": float(sum(stress_active) / max(1, len(stress_active))),
        "stress_p95_active": float(percentile(stress_active, 0.95)),
        "stress_over_thr_ratio_active": float(len(stress_over) / max(1, len(stress_active))),
        "accel_mean_active": float(sum(accel_active) / max(1, len(accel_active))),
        "accel_p95_active": float(percentile(accel_active, 0.95)),
        "asymmetry_mean_active": float(sum(asym_active) / max(1, len(asym_active))),
    }


def compute_scores(mode_metrics: Dict[str, Dict[str, float]]) -> Dict[str, float]:
    components: List[Tuple[str, float]] = [
        ("accel_p95_active", 0.35),
        ("accel_mean_active", 0.25),
        ("stress_over_thr_ratio_active", 0.20),
        ("asymmetry_mean_active", 0.20),
    ]

    mins: Dict[str, float] = {}
    for key, _ in components:
        vals = [mode_metrics[m][key] for m in mode_metrics]
        mins[key] = min(vals) if vals else 1.0

    scores: Dict[str, float] = {}
    for mode, metrics in mode_metrics.items():
        score = 0.0
        for key, weight in components:
            base = mins[key]
            value = metrics[key]
            if base <= 1e-9:
                norm = 1.0 if value <= 1e-9 else 1.0 + value
            else:
                norm = value / base
            score += weight * norm
        scores[mode] = score
    return scores


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Four-mode experiment comparison (active-window metrics + composite score)")
    parser.add_argument("--data-dir", default="experiment_results/data", help="Input experiment CSV directory")
    parser.add_argument("--plot-dir", default="experiment_results/plots", help="Output plots directory")
    parser.add_argument("--active-force-threshold", type=float, default=0.3, help="Active-window threshold for |Fy| (N)")
    parser.add_argument("--stress-threshold", type=float, default=5.0, help="Stress threshold (N)")
    parser.add_argument("--output-prefix", default="mode_comparison", help="Output filename prefix")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(args.plot_dir, exist_ok=True)

    mode_files: Dict[str, str] = {}
    missing: List[str] = []
    for mode in MODE_ORDER:
        path = find_latest_mode_csv(args.data_dir, mode)
        if not path:
            missing.append(mode)
        else:
            mode_files[mode] = path

    if missing:
        print("WARNING: missing CSV files, cannot run full four-mode comparison:")
        for mode in missing:
            print(f"  - {mode} (expected: experiment_{mode}_*.csv)")
        print(f"  data directory: {args.data_dir}")
        return 0

    mode_metrics: Dict[str, Dict[str, float]] = {}
    for mode in MODE_ORDER:
        mode_metrics[mode] = load_mode_metrics(
            mode_files[mode], args.active_force_threshold, args.stress_threshold
        )

    scores = compute_scores(mode_metrics)
    ranking = sorted(scores.items(), key=lambda kv: kv[1])

    rigid_score = scores.get("rigid", 0.0)
    improve_vs_rigid: Dict[str, float] = {}
    for mode, score in scores.items():
        if rigid_score > 1e-9:
            improve_vs_rigid[mode] = (rigid_score - score) / rigid_score * 100.0
        else:
            improve_vs_rigid[mode] = 0.0

    labels = [MODE_LABEL[m] for m in MODE_ORDER]
    colors = ["#8f8f8f", "#4c78a8", "#f58518", "#54a24b"]

    accel_p95_vals = [mode_metrics[m]["accel_p95_active"] for m in MODE_ORDER]
    stress_over_vals = [mode_metrics[m]["stress_over_thr_ratio_active"] * 100.0 for m in MODE_ORDER]
    asym_vals = [mode_metrics[m]["asymmetry_mean_active"] for m in MODE_ORDER]
    score_vals = [scores[m] for m in MODE_ORDER]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    def draw_bar(ax, values: List[float], title: str, ylabel: str, lower_better: bool = True):
        bars = ax.bar(labels, values, color=colors, edgecolor="black", alpha=0.88)
        for i, bar in enumerate(bars):
            y = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2, y + (max(values) * 0.02 if max(values) > 0 else 0.02),
                    f"{y:.2f}", ha="center", va="bottom", fontsize=10, fontweight="bold")
            if MODE_ORDER[i] != "rigid":
                delta = improve_vs_rigid[MODE_ORDER[i]]
                ax.text(bar.get_x() + bar.get_width() / 2, y * 0.6,
                        f"vs rigid {delta:+.1f}%", ha="center", va="center", fontsize=9, color="white")

        ax.set_title(title + (" (lower is better)" if lower_better else ""))
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", linestyle="--", alpha=0.6)

    draw_bar(axes[0, 0], accel_p95_vals, "Active-window Acceleration P95", "Norm")
    draw_bar(axes[0, 1], stress_over_vals, "Active-window Stress Over-threshold Ratio", "Ratio (%)")
    draw_bar(axes[1, 0], asym_vals, "Active-window Force Asymmetry |Ly-Ry|", "Mean absolute diff (N)")
    draw_bar(axes[1, 1], score_vals, "Composite Stability Score", "Score")

    best_mode = ranking[0][0]
    fig.suptitle(
        "Four-mode Comparison (Active Window)\n"
        f"Best mode: {MODE_LABEL.get(best_mode, best_mode)} | "
        f"Compliant+CHOMP improvement vs rigid: {improve_vs_rigid.get('compliant_chomp', 0.0):+.1f}%",
        fontsize=15,
        fontweight="bold",
    )

    plt.tight_layout(rect=[0, 0.02, 1, 0.94])

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png_path = os.path.join(args.plot_dir, f"{args.output_prefix}_{ts}.png")
    csv_path = os.path.join(args.plot_dir, f"{args.output_prefix}_{ts}.csv")

    plt.savefig(png_path, dpi=160, bbox_inches="tight")
    plt.close()

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "mode",
            "label",
            "source_csv",
            "duration_s",
            "active_ratio",
            "stress_mean_active",
            "stress_p95_active",
            "stress_over_thr_ratio_active",
            "accel_mean_active",
            "accel_p95_active",
            "asymmetry_mean_active",
            "composite_score",
            "improve_vs_rigid_pct",
        ])
        for mode in MODE_ORDER:
            m = mode_metrics[mode]
            writer.writerow([
                mode,
                MODE_LABEL[mode],
                mode_files[mode],
                f"{m['duration_s']:.6f}",
                f"{m['active_ratio']:.6f}",
                f"{m['stress_mean_active']:.6f}",
                f"{m['stress_p95_active']:.6f}",
                f"{m['stress_over_thr_ratio_active']:.6f}",
                f"{m['accel_mean_active']:.6f}",
                f"{m['accel_p95_active']:.6f}",
                f"{m['asymmetry_mean_active']:.6f}",
                f"{scores[mode]:.6f}",
                f"{improve_vs_rigid[mode]:.3f}",
            ])

        writer.writerow([])
        writer.writerow(["ranking", "mode", "score"])
        for rank, (mode, score) in enumerate(ranking, 1):
            writer.writerow([rank, mode, f"{score:.6f}"])

    print("Done: four-mode comparison files generated:")
    print(f"  PNG: {png_path}")
    print(f"  CSV: {csv_path}")
    print("\nComposite score ranking (lower is better):")
    for rank, (mode, score) in enumerate(ranking, 1):
        print(f"  {rank}. {mode:<16} score={score:.4f}  vs-rigid={improve_vs_rigid.get(mode, 0.0):+.1f}%")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
