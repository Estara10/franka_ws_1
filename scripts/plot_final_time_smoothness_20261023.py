#!/usr/bin/env python3
"""Compose the final time-smoothness figure from saved R/S/O and R/S data."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scientific_style import GROUP_COLORS, NATURE, configure_scientific_style


ROOT = Path(__file__).resolve().parents[1]
OFFLINE = ROOT / "experiment_results/reports/final_tradeoff_20261022/time_profile_rs_o/rs_o_by_run.csv"
EXECUTION = ROOT / "experiment_results/reports/final_tradeoff_20261022/rs_execution/rs_execution.csv"
OUT_DIR = ROOT / "docs/figures/历史归档/技术分析版"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def short_run_id(value: str) -> str:
    return value.replace("abcd_s0_today_", "").replace("_20261022", "")


def main() -> None:
    configure_scientific_style()
    offline = read_csv(OFFLINE)
    execution = read_csv(EXECUTION)
    if not offline or not execution:
        raise RuntimeError("missing R/S/O or R/S execution data")

    fig, axes = plt.subplots(2, 2, figsize=(10.6, 8.0))
    ax_a, ax_b, ax_c, ax_d = axes.ravel()

    # A: S relative to R for every formal path.
    x = np.arange(len(offline))
    groups = [row["group"] for row in offline]
    colors = [GROUP_COLORS[group] for group in groups]
    s_change = [float(row["s_vs_r_rms_pct"]) for row in offline]
    ax_a.axhline(-42.13, color=NATURE["gray"], linestyle="--", linewidth=0.9, label=r"$1/1.2^3$ theory")
    ax_a.scatter(x, s_change, c=colors, s=30, edgecolors="white", linewidths=0.4, zorder=3)
    ax_a.set_ylim(-43.1, -41.2)
    ax_a.set_xticks(x)
    ax_a.set_xticklabels([short_run_id(row["run_id"]) for row in offline], rotation=60, ha="right")
    ax_a.set_ylabel("RMS change vs R (%)")
    ax_a.set_title("S: uniform 1.2T stretch")
    ax_a.legend(frameon=False, loc="lower right")

    # B: O relative to S, with the pre-registered action gate.
    o_change = [float(row["o_vs_s_rms_pct"]) for row in offline]
    ax_b.axhline(-5.0, color=NATURE["red"], linestyle="--", linewidth=0.9, label="-5% action gate")
    ax_b.scatter(x, o_change, c=colors, s=30, edgecolors="white", linewidths=0.4, zorder=3)
    ax_b.set_ylim(min(o_change) - 0.8, max(o_change) + 0.8)
    ax_b.set_xticks(x)
    ax_b.set_xticklabels([short_run_id(row["run_id"]) for row in offline], rotation=60, ha="right")
    ax_b.set_ylabel("RMS change vs S (%)")
    ax_b.set_title("O: optimization within 1.2T")
    ax_b.legend(frameon=False, loc="lower right")

    # C: planned and observed transport time for the four R/S runs.
    exec_x = np.arange(len(execution))
    action = [float(row["transport_action_wall_s"]) for row in execution]
    plan = [float(row["planned_duration_s"]) for row in execution]
    exec_colors = [NATURE["blue"] if row["mode"] == "R" else NATURE["red"] for row in execution]
    ax_c.bar(exec_x, action, color=exec_colors, width=0.62, alpha=0.80, label="MoveGroup action")
    ax_c.plot(exec_x, plan, color=NATURE["gray"], marker="D", linestyle="none",
              markersize=4.5, label="planned transport", zorder=3)
    ax_c.set_xticks(exec_x)
    ax_c.set_xticklabels([row["tag"] for row in execution])
    ax_c.set_ylabel("Transport time (s)")
    ax_c.set_title("R/S execution time")
    ax_c.set_ylim(0.0, max(action) * 1.16)
    ax_c.legend(frameon=False, loc="upper left")

    # D: release placement; annotate the small execution sample explicitly.
    xy = [float(row["release_xy_error_mm"]) for row in execution]
    yaw = [float(row["release_yaw_error_deg"]) for row in execution]
    ax_d.scatter(xy, yaw, c=exec_colors, s=48, edgecolors="white", linewidths=0.5, zorder=3)
    for x_value, y_value, row in zip(xy, yaw, execution):
        ax_d.annotate(row["tag"], (x_value, y_value), xytext=(4, 4),
                      textcoords="offset points", fontsize=7)
    ax_d.set_xlabel("Release XY error (mm)")
    ax_d.set_ylabel("Release yaw error (deg)")
    ax_d.set_title("R/S release placement (2/2 each complete)")
    ax_d.set_xlim(min(xy) - 0.35, max(xy) + 0.45)
    ax_d.set_ylim(min(yaw) - 0.025, max(yaw) + 0.025)

    for axis, label in zip(axes.ravel(), ("A", "B", "C", "D")):
        axis.text(-0.13, 1.04, label, transform=axis.transAxes, fontsize=11,
                  fontweight="bold", va="top")
    fig.suptitle("Time–smoothness trade-off: reference trajectories and R/S execution", fontsize=12)
    fig.text(
        0.5,
        0.018,
        "图注：A、B 为 12 条基础路径的离线连续参考指标；C、D 为 R/S 四次完整执行（每组 n=2）。"
        "jerk 为计划参考曲线指标，不是实测机械臂 jerk；O 未执行。",
        ha="center",
        va="bottom",
        fontsize=7.5,
    )
    fig.subplots_adjust(bottom=0.17)
    fig.tight_layout(rect=(0.0, 0.07, 1.0, 0.95))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "svg"):
        fig.savefig(OUT_DIR / f"02_运输时间与平滑度.{suffix}", dpi=300)
    plt.close(fig)
    print(OUT_DIR / "02_运输时间与平滑度.png")
    print(OUT_DIR / "02_运输时间与平滑度.svg")


if __name__ == "__main__":
    main()
