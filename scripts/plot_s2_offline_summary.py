#!/usr/bin/env python3
"""Plot the two S2 offline before/after metrics from their frozen result records.

These are single reconstructed offline planning samples, not execution trials.
The values and source records are kept explicit here to make the thesis figure
reproducible without implying a run-to-run range that was never measured.
"""

from pathlib import Path

import matplotlib.pyplot as plt


METRICS = [
    {
        "title": "S2 geometry objective",
        "unit": "normalized objective J (lower is better)",
        "baseline": 1.100000000,
        "optimized": 1.083784970,
        "change": "−1.47%",
        "source": "P6_S2_projection_optimizer_result_20261017.md, r11",
    },
    {
        "title": "S2 time-profile jerk RMS",
        "unit": "rad/s³ (continuous-time metric; lower is better)",
        "baseline": 29.7753,
        "optimized": 28.8372,
        "change": "−3.15%",
        "source": "P6_S2_time_profile_result_20261017.md, continuous mapping",
    },
]


def main():
    out = Path("docs/figures/历史归档/离线探索/20261020_单样本离线优化.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.6), constrained_layout=True)
    colors = {"baseline": "#557A95", "optimized": "#E1812C"}
    labels = {"baseline": "Before", "optimized": "After"}

    for ax, metric in zip(axes, METRICS):
        vals = [metric["baseline"], metric["optimized"]]
        ax.plot(vals, [0, 0], color="#9AA6AC", linewidth=2.0, zorder=1)
        for key, y in (("baseline", 0), ("optimized", 0)):
            value = metric[key]
            ax.scatter(value, y, s=105, color=colors[key], edgecolor="white",
                       linewidth=0.8, zorder=2, label=labels[key])
            ax.annotate(f"{value:.4f}" if value < 10 else f"{value:.3f}",
                        (value, y), xytext=(0, 15 if key == "baseline" else -23),
                        textcoords="offset points", ha="center", fontsize=10)
        ax.set_title(metric["title"])
        ax.set_xlabel(metric["unit"])
        ax.set_yticks([])
        span = max(vals) - min(vals)
        pad = max(span * 0.9, max(vals) * 0.025)
        ax.set_xlim(min(vals) - pad, max(vals) + pad)
        ax.grid(axis="x", alpha=0.25)
        ax.text(0.5, 0.84, metric["change"], transform=ax.transAxes,
                ha="center", va="center", fontsize=12, fontweight="bold")

    axes[0].legend(loc="lower center", bbox_to_anchor=(0.5, -0.32), ncol=2, frameon=False)
    fig.suptitle("S2 offline before/after metrics — one reconstructed sample per branch", fontsize=14)
    fig.text(0.5, -0.06,
             "No execution trials or repeated samples are available for S2; no range is implied. "
             "The jerk metric is the continuous-time result (not the 21-point export interpolation value).",
             ha="center", fontsize=9)
    fig.savefig(out, dpi=240, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
