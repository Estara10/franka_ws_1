#!/usr/bin/env python3
"""Summarize 2026-10-22 S2 transport timing tradeoffs from saved trajectories."""

import csv
import math
import os
import re
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_s2_tradeoff_20261022")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]


ROOT = Path(__file__).resolve().parents[1]
CASES = [
    ("P1 30 cm", "experiment_results/a1_runs/s2_20261022_p1_d30_check3"),
    ("P2 30 cm", "experiment_results/a1_runs/s2_20261022_p2_d30_retry"),
    ("Existing 18 cm", "experiment_results/a1_runs/s2s3_plan_long_20261020"),
]
METHOD_FILES = {
    "B": "transport_baseline_trajectory.csv",
    "G candidate": "transport_optimized_trajectory.csv",
    "TP": "time_profile_candidate.csv",
}
REPORT = ROOT / "experiment_results/reports/s2_time_tradeoff_20261022"
FIGURES = ROOT / "docs/figures/s2_time_tradeoff_20261022"


def load_trajectory(path):
    joints = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            joint = joints.setdefault(row["joint_name"], {})
            joint[int(row["point_index"])] = (
                float(row["time_from_start"]),
                float(row["position"]),
                float(row["velocity"]),
                float(row["acceleration"]),
            )
    return joints


def continuous_jerk(path):
    joints = load_trajectory(path)
    quadrature_x, quadrature_w = np.polynomial.legendre.leggauss(48)
    sum_squared_integral = 0.0
    total_duration = None
    peak = 0.0
    for points in joints.values():
        ordered = [points[i] for i in sorted(points)]
        total_duration = ordered[-1][0]
        for start, end in zip(ordered, ordered[1:]):
            t0, q0, v0, a0 = start
            t1, q1, v1, a1 = end
            duration = t1 - t0
            c0 = q0
            c1 = v0 * duration
            c2 = 0.5 * a0 * duration * duration
            d = q1 - c0 - c1 - c2
            v = v1 * duration - c1 - 2.0 * c2
            a = a1 * duration * duration - 2.0 * c2
            c3 = 10.0 * d - 4.0 * v + 0.5 * a
            c4 = -15.0 * d + 7.0 * v - a
            c5 = 6.0 * d - 3.0 * v + 0.5 * a
            u = 0.5 * (quadrature_x + 1.0)
            jerk = (6.0 * c3 + 24.0 * c4 * u + 60.0 * c5 * u * u) / duration**3
            sum_squared_integral += 0.5 * duration * float(np.dot(quadrature_w, jerk * jerk))
            peak = max(peak, float(np.max(np.abs(jerk))))
    rms = math.sqrt(sum_squared_integral / (total_duration * len(joints)))
    return total_duration, rms, peak


def read_summary(path):
    with path.open(newline="") as stream:
        return {row["plan"]: row for row in csv.DictReader(stream)}


def read_profile_log(path):
    pattern = re.compile(
        r"\[TIME_PROFILE_OPT\].*continuous_jerk_rms=([0-9.eE+-]+)->([0-9.eE+-]+) "
        r"\(([-0-9.eE+]+)%\).*continuous_jerk_max=([0-9.eE+-]+)->([0-9.eE+-]+)"
    )
    match = pattern.search(path.read_text())
    if not match:
        raise RuntimeError(f"missing [TIME_PROFILE_OPT] row in {path}")
    return tuple(float(value) for value in match.groups())


def main():
    REPORT.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)
    results = []
    for scene, relative_dir in CASES:
        run_dir = ROOT / relative_dir
        summary = read_summary(run_dir / "transport_plan_summary.csv")
        tp_native = read_profile_log(run_dir / "task_launch.log")
        loaded = {}
        for method, filename in METHOD_FILES.items():
            duration, rms, peak = continuous_jerk(run_dir / filename)
            row_name = {"B": "baseline", "G candidate": "geometry_candidate", "TP": "time_profile_candidate"}[method]
            row = summary[row_name]
            loaded[method] = {
                "duration": duration,
                "jerk_rms": rms,
                "jerk_max": peak,
                "path_error_mm": float(row["max_rod_pos_mm"]),
                "pass": int(row["pass"]),
                "run_dir": relative_dir,
            }
        common_duration = max(loaded["B"]["duration"], loaded["G candidate"]["duration"])
        for method, values in loaded.items():
            common_factor = common_duration / values["duration"]
            row = {
                "scene": scene,
                "run_dir": values["run_dir"],
                "method": method,
                "pass": values["pass"],
                "duration_raw_s": values["duration"],
                "jerk_rms_raw_rad_s3": values["jerk_rms"],
                "jerk_max_raw_rad_s3": values["jerk_max"],
                "path_error_mm": values["path_error_mm"],
                "tp_profile_continuous_jerk_rms_rad_s3": (
                    tp_native[1]
                    if method == "TP" else ""
                ),
                "tp_profile_continuous_jerk_max_rad_s3": (
                    tp_native[4]
                    if method == "TP" else ""
                ),
                "tp_profile_baseline_jerk_rms_rad_s3": tp_native[0] if method == "TP" else "",
                "tp_profile_native_change_pct": tp_native[2] if method == "TP" else "",
                "tp_max_export_deviation_rad": (
                    float(summary["time_profile_candidate"]["max_export_deviation_rad"])
                    if method == "TP" else ""
                ),
                "T_common_s": common_duration,
                "stretch_to_common": common_factor,
                "jerk_rms_common_rad_s3": values["jerk_rms"] / common_factor**3,
                "jerk_max_common_rad_s3": values["jerk_max"] / common_factor**3,
            }
            for c in (1.0, 1.1, 1.2):
                row[f"duration_{c:.1f}x_s"] = common_duration * c
                row[f"jerk_rms_{c:.1f}x_rad_s3"] = values["jerk_rms"] / (common_factor * c) ** 3
                row[f"jerk_max_{c:.1f}x_rad_s3"] = values["jerk_max"] / (common_factor * c) ** 3
            results.append(row)

    fieldnames = list(results[0])
    with (REPORT / "s2_time_tradeoff.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    scenes = [case[0] for case in CASES]
    methods = list(METHOD_FILES)
    colors = {"B": "#355f8a", "G candidate": "#dc8d27", "TP": "#b64b52"}
    values_by_scene = {scene: {row["method"]: row for row in results if row["scene"] == scene} for scene in scenes}

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    metric_specs = [
        ("duration_raw_s", "Duration (s)"),
        ("jerk_rms_raw_rad_s3", "Exported quintic jerk RMS (rad/s³)"),
        ("path_error_mm", "Max rod path error (mm)"),
    ]
    x = np.arange(len(scenes))
    width = 0.24
    for ax, (metric, ylabel) in zip(axes, metric_specs):
        for i, method in enumerate(methods):
            vals = [values_by_scene[s][method][metric] for s in scenes]
            bars = ax.bar(x + (i - 1) * width, vals, width, label=method, color=colors[method])
            ax.bar_label(bars, fmt="%.2f", fontsize=8, padding=2)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x, scenes)
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False)
    fig.suptitle("Coordinated transport planning: position × method", fontsize=14)
    fig.tight_layout()
    fig.savefig(FIGURES / "s2_position_method_metrics.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.5, 7))
    markers = {"P1 30 cm": "o", "P2 30 cm": "s", "Existing 18 cm": "^"}
    for scene in scenes:
        for method in methods:
            row = values_by_scene[scene][method]
            ax.scatter(row["duration_raw_s"], row["jerk_rms_raw_rad_s3"],
                       marker=markers[scene], s=82, facecolors="none",
                       edgecolors=colors[method], linewidths=1.7)
            xs = [row[f"duration_{c:.1f}x_s"] for c in (1.0, 1.1, 1.2)]
            ys = [row[f"jerk_rms_{c:.1f}x_rad_s3"] for c in (1.0, 1.1, 1.2)]
            ax.plot(xs, ys, color=colors[method], alpha=0.45,
                    linestyle={"B": "-", "G candidate": "--", "TP": ":"}[method],
                    marker=".", markersize=5)
    ax.set_xlabel("Transport duration (s)")
    ax.set_ylabel("Exported quintic jerk RMS (rad/s³)")
    ax.set_title("Time–smoothness tradeoff: raw plans and Tcommon × 1.0/1.1/1.2 stretches")
    ax.grid(alpha=0.25)
    for method, color in colors.items():
        ax.plot([], [], color=color, label=method)
    method_legend = ax.legend(frameon=False, ncol=3, loc="upper right")
    ax.add_artist(method_legend)
    for scene, marker in markers.items():
        ax.scatter([], [], marker=marker, facecolors="none", edgecolors="#444444", label=scene)
    ax.legend(frameon=False, ncol=3, loc="upper left")
    fig.tight_layout()
    fig.savefig(FIGURES / "s2_time_jerk_tradeoff.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
