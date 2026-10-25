#!/usr/bin/env python3
"""Generate title-free report plots from saved experiment data; no simulation."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scientific_style import GROUP_COLORS, NATURE, configure_scientific_style
from analyze_s2s3_extended import DIR_RUNS, FF_RUNS, directional_run, ff_run


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "experiment_results/reports/final_tradeoff_20261022"
RUNS = ROOT / "experiment_results/a1_runs"
OUT = ROOT / "docs/figures/结题主图"
B_REPORT = ROOT / "experiment_results/reports/b_formal_20261025"
B_RUN_IDS = [f"b_formal_20261025_{i:02d}" for i in range(1, 4)]
BLUE, GREEN, ORANGE = (NATURE[key] for key in ("blue", "green", "orange"))
EXPORTS = []


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def values(rows, key):
    return np.array([float(row[key]) for row in rows])


def setup():
    configure_scientific_style()
    plt.rcParams.update({
        "font.size": 10, "axes.labelsize": 10, "xtick.labelsize": 9,
        "ytick.labelsize": 9, "legend.fontsize": 9, "axes.linewidth": 0.7,
        "lines.linewidth": 1.4, "axes.unicode_minus": False, "svg.fonttype": "none",
    })
    OUT.mkdir(parents=True, exist_ok=True)


def save(fig, name, sources, metrics):
    fig.savefig(OUT / f"{name}.png", dpi=300)
    fig.savefig(OUT / f"{name}.svg")
    plt.close(fig)
    EXPORTS.append({"figure": name, "sources": sources, "metrics": metrics})
    print(OUT / f"{name}.png")


def plot_transport():
    source = B_REPORT / "abcd_display_runs.csv"
    rows = read_csv(source)
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.5), layout="constrained")
    counts, means, yaws = [], [], []
    for i, group in enumerate("ABCD"):
        completed = [r for r in rows if r["group"] == group and r["state"] == "DONE"]
        counts.append(len(completed))
        axes[0].bar(i, len(completed), width=0.56, color=GROUP_COLORS[group])
        error = values(completed, "release_xy_error_mm")
        means.append(float(np.mean(error)) if len(error) else None)
        yaws.append(float(np.mean(values(completed, "release_yaw_error_deg"))) if completed else None)
        axes[1].scatter(i + np.linspace(-0.07, 0.07, len(error)), error,
                        color=GROUP_COLORS[group], s=26, zorder=3)
        if len(error):
            axes[1].hlines(means[-1], i - 0.19, i + 0.19, color=GROUP_COLORS[group], lw=1.6)
    axes[0].set(ylim=(0, 3.6), yticks=range(4), ylabel="完整完成次数")
    for i, count in enumerate(counts):
        axes[0].text(i, count + 0.10, f"{count}/3", ha="center", fontsize=9)
    axes[1].set(ylim=(0, 12), ylabel="释放后平面位置误差（mm）")
    for ax in axes:
        ax.set_xticks(range(4), list("ABCD"))
        ax.set_xlabel("实验组别")
        ax.grid(axis="x", visible=False)
    save(fig, "01_完整搬运对照", [str(source.relative_to(ROOT))],
         {"groups": list("ABCD"), "completed": counts, "attempts_each": 3,
          "release_xy_mean_mm": means, "release_yaw_mean_deg": yaws,
          "batches": {"B": "20261025", "A_C_D": "20261022"}})


def plot_b_formal():
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.5), layout="constrained")
    sources, spans = [], []
    for i, (run_id, color) in enumerate(zip(B_RUN_IDS, (BLUE, GREEN, ORANGE)), 1):
        source = RUNS / run_id / "contact_probe.csv"
        rows = read_csv(source)
        time = (values(rows, "left_sim_time") + values(rows, "right_sim_time")) / 2
        time -= time[0]
        axes[0].plot(time, values(rows, "rod_z") * 1000, label=f"B{i}", color=color)
        axes[1].plot(values(rows, "rod_x") * 1000, values(rows, "rod_z") * 1000,
                     label=f"B{i}", color=color)
        sources.append(str(source.relative_to(ROOT)))
        spans.append(float(time[-1]))
    axes[0].set(xlabel="接触夹稳后的时间（s）", ylabel="铝条Z坐标（mm）")
    axes[1].set(xlabel="铝条X坐标（mm）", ylabel="铝条Z坐标（mm）")
    for ax in axes:
        ax.legend(frameon=False)
    out = ROOT / "docs/figures/辅助图/04_B组完整搬运曲线"
    fig.savefig(out.with_suffix(".png"), dpi=300)
    fig.savefig(out.with_suffix(".svg"))
    plt.close(fig)
    EXPORTS.append({"figure": str(out.relative_to(ROOT)),
                    "sources": sources,
                    "metrics": {"runs": B_RUN_IDS, "span_sec": spans,
                                "coordinates": "MuJoCo rod world XZ; feedback-time axis"}})
    print(out.with_suffix(".png"))


def plot_b_jerk():
    source = B_REPORT / "b_runs.csv"
    rows = read_csv(source)
    fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.5), layout="constrained")
    x = np.arange(len(rows))
    for ax, suffix, label in (
        (axes[0], "rms", "连续参考jerk RMS（rad/s³）"),
        (axes[1], "max", "连续参考jerk峰值（rad/s³）"),
    ):
        ax.bar(x - 0.17, values(rows, f"baseline_continuous_jerk_{suffix}"),
               width=0.32, color=BLUE, label="基础参考")
        ax.bar(x + 0.17, values(rows, f"candidate_continuous_jerk_{suffix}"),
               width=0.32, color=GREEN, label="时间优化参考")
        ax.set_xticks(x, [r["sample"] for r in rows])
        ax.set(xlabel="B组运行编号", ylabel=label, ylim=(0, None))
        ax.grid(axis="x", visible=False)
        ax.legend(frameon=False)
    out = ROOT / "docs/figures/辅助图/05_B组运输参考平滑度"
    fig.savefig(out.with_suffix(".png"), dpi=300)
    fig.savefig(out.with_suffix(".svg"))
    plt.close(fig)
    EXPORTS.append({"figure":str(out.relative_to(ROOT)),
                    "sources":[str(source.relative_to(ROOT))],
                    "metrics":{"comparison":"same run, same geometry and duration; planned jerk"}})
    print(out.with_suffix(".png"))


def plot_time_tradeoff():
    offline_path = REPORT / "time_profile_rs_o/rs_o_by_run.csv"
    execution_path = REPORT / "rs_execution/rs_execution.csv"
    offline, execution = read_csv(offline_path), read_csv(execution_path)
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.5), layout="constrained")
    x = np.arange(1, len(offline) + 1)
    for key, color, marker, label in (
        ("rms_R", BLUE, "o", "R：原时长"),
        ("rms_S", GREEN, "s", "S：1.2倍时长"),
        ("rms_O", ORANGE, "^", "O：1.2倍时长再优化（离线）"),
    ):
        axes[0].plot(x, values(offline, key), marker=marker, color=color,
                     markersize=3.5, linewidth=1.0, label=label)
    axes[0].set(xlabel="基础路径编号", ylabel="连续参考jerk RMS（rad/s³）", ylim=(0, None))
    axes[0].set_xticks(x)
    axes[0].legend(frameon=False, loc="lower left", fontsize=8)

    x = np.arange(len(execution))
    axes[1].bar(x, values(execution, "transport_action_wall_s"), width=0.54,
                color=[BLUE if r["mode"] == "R" else GREEN for r in execution],
                label="动作墙钟耗时")
    axes[1].plot(x, values(execution, "planned_duration_s"), linestyle="none",
                 marker="D", color="#333333", markersize=4, label="计划时长", zorder=3)
    axes[1].set_xticks(x, [r["tag"] for r in execution])
    axes[1].set(xlabel="运行编号", ylabel="运输时间（s）", ylim=(0, 4))
    axes[1].grid(axis="x", visible=False)
    axes[1].legend(frameon=False, loc="upper left")
    normalized = [np.full(len(offline), 100.0),
                  values(offline, "rms_S") / values(offline, "rms_R") * 100,
                  values(offline, "rms_O") / values(offline, "rms_R") * 100]
    xy = {mode: float(np.mean(values([r for r in execution if r["mode"] == mode],
                                  "release_xy_error_mm"))) for mode in ("R", "S")}
    save(fig, "02_运输时间与平滑度",
         [str(p.relative_to(ROOT)) for p in (offline_path, execution_path)],
         {"paths": len(offline), "normalized_jerk_median_pct": [float(np.median(v)) for v in normalized],
          "slow_reduction_median_pct": 100 - float(np.median(normalized[1])),
          "optimization_extra_median_pct": -float(np.median(values(offline, "o_vs_s_rms_pct"))),
          "release_xy_mean_mm": xy, "O_executed": False})


def plot_feedforward():
    records = {group: [ff_run(RUNS / run, 0.0) for run in ids] for group, ids in FF_RUNS.items()}
    fig, ax = plt.subplots(figsize=(6.5, 3.8), layout="constrained")
    styles = ("-", "--", ":")
    for group, color, label in (("off", BLUE, "未补偿"), ("on", GREEN, "载荷补偿")):
        for i, rec in enumerate(records[group]):
            ax.plot(rec["series"]["t"], rec["series"]["rod"], color=color,
                    linestyle=styles[i], linewidth=1.5, label=f"{label}{i + 1}")
    ax.axhline(0, color="#888888", linewidth=0.6)
    ax.set(xlim=(0, 3), ylim=(-10, 0.5), xlabel="MMC保持开始后的时间（s）",
           ylabel="铝条竖向位移（mm）")
    ax.legend(frameon=False, ncol=2, loc="lower left", fontsize=8)
    medians = {g: float(np.median([r["rod_dz_mm"] for r in recs])) for g, recs in records.items()}
    save(fig, "03_载荷补偿减少下沉",
         [f"experiment_results/a1_runs/{run}/{name}" for ids in FF_RUNS.values() for run in ids
          for name in ("contact_probe.csv", "mmc_hold_trace.csv", "prepare_status.json")],
         {"n_each": 3, "rod_dz_median_mm": medians,
          "descent_reduction_pct": (1 - abs(medians["on"]) / abs(medians["off"])) * 100,
          "sampling_span_sec": [r["window_span_sec"] for recs in records.values() for r in recs]})


def plot_directional():
    records = {group: [directional_run(RUNS / run) for run in ids] for group, ids in DIR_RUNS.items()}
    fig, ax = plt.subplots(figsize=(6.5, 3.8), layout="constrained")
    styles = ("-", "--", ":")
    for group, color, label in (("fixed", BLUE, "400 N/m"), ("soft", ORANGE, "200 N/m")):
        for i, rec in enumerate(records[group]):
            t, position = np.array(rec["pulse_t"]), np.array(rec["fields"]["rod_x"])
            y = position - np.interp(0.0, t, position)
            ax.plot(t, y, color=color, linestyle=styles[i], linewidth=1.5,
                    label=f"{label}，第{i + 1}次")
    ax.axvspan(0, 1, color="#ADB5BD", alpha=0.18)
    ax.axhline(0, color="#888888", linewidth=0.6)
    ax.set(xlim=(-0.5, 4), xlabel="相对施力开始的时间（s）", ylabel="铝条X向位移（mm）")
    ax.legend(frameon=False, ncol=2, loc="lower right", fontsize=8)
    medians = {group: [float(np.median([r[key] for r in recs]))
                       for key in ("rod_x_pulse_end_mm", "rod_x_plus_3s_mm")]
               for group, recs in records.items()}
    save(fig, "04_方向性柔顺与受力让步",
         [f"experiment_results/a1_runs/{run}/{name}" for ids in DIR_RUNS.values() for run in ids
          for name in ("contact_probe.csv", "mmc_hold_trace.csv", "rod_force_events.csv", "prepare_status.json")],
         {"n_each": 3, "pulse_end_and_residual_median_mm": medians,
          "pulse_end_displacement_increase_pct": (medians["soft"][0] / medians["fixed"][0] - 1) * 100})


def main():
    setup()
    plot_transport()
    plot_b_formal()
    plot_b_jerk()
    plot_time_tradeoff()
    plot_feedforward()
    plot_directional()
    path = B_REPORT / "report_figures_20261025.json"
    path.write_text(json.dumps(EXPORTS, ensure_ascii=False, indent=2) + "\n")
    print(path)


if __name__ == "__main__":
    main()
