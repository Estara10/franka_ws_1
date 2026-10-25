#!/usr/bin/env python3
"""Report figures from the frozen compliance batch; no experiments or resampling."""
import ast
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scientific_style import configure_scientific_style

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "experiment_results/reports/compliance_hover_20261025_r1"
OUT = ROOT / "docs/figures/柔顺专项_20261025"
OUT.mkdir(exist_ok=True)
configure_scientific_style()
plt.rcParams.update({"font.size": 10, "axes.labelsize": 10, "legend.fontsize": 9})


def read(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


rows = [r for r in read(SRC / "runs.csv") if r["analysis_include"] == "1"]
valid = lambda r: all(r[k] == "1" for k in
                      ("valid_observation_window", "valid_payload_feedforward", "valid_force_pulse"))
stats = read(SRC / "group_statistics.csv")
contrasts = read(SRC / "contrast_estimates.csv")
colors = {"F0": "#0C5DA5", "F1": "#2A9D8F", "K400-N": "#0C5DA5",
          "K400-F": "#0C5DA5", "K200-N": "#E69F00", "K200-F": "#E69F00"}


def save(fig, name):
    fig.savefig(OUT / (name + ".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT / (name + ".svg"), bbox_inches="tight")
    plt.close(fig)


feed = []
for r in rows:
    if r["study"] != "feedforward" or not valid(r):
        continue
    t0 = float(r["first_reference_sim_time"])
    points = []
    for s in read(ROOT / r["run_dir"] / "contact_probe.csv"):
        if s["phase"] != "MMC_HOLD":
            continue
        t = .5 * (float(s["left_sim_time"]) + float(s["right_sim_time"])) - t0
        if 0 <= t <= 3.5:
            points.append([t] + [float(s[k]) for k in ("rod_z", "left_hand_z", "right_hand_z")])
    a = np.array(sorted(points))
    a[:, 1:] = (a[:, 1:] - a[0, 1:]) * 1000
    feed.append((r["condition"], a))
lo, hi = max(a[0, 0] for _, a in feed), min(a[-1, 0] for _, a in feed)
grid = np.arange(lo, hi + 1e-9, .05)
fig, axes = plt.subplots(3, 1, figsize=(6.5, 6.5), sharex=True, constrained_layout=True)
for j, (ax, label) in enumerate(zip(axes, ("杆", "左手", "右手")), 1):
    for c, legend in (("F0", "无载荷前馈，n=10"), ("F1", "有载荷前馈，n=10")):
        mat = np.array([np.interp(grid, a[:, 0], a[:, j]) for cc, a in feed if cc == c])
        for y in mat:
            ax.plot(grid, y, color=colors[c], lw=.65, alpha=.22)
        ax.fill_between(grid, mat.min(0), mat.max(0), color=colors[c], alpha=.10)
        ax.plot(grid, mat.mean(0), color=colors[c], lw=1.7, label=legend)
    ax.set_ylabel(label + "竖向位移 (mm)")
    ax.axhline(0, color=".5", lw=.6)
axes[0].legend(frameon=False)
axes[-1].set_xlabel("首条有效阻抗参考确认后的时间 (s)")
save(fig, "载荷前馈_竖向曲线")

fig, ax = plt.subplots(figsize=(6.0, 3.6), constrained_layout=True)
for i, c in enumerate(("F0", "F1")):
    vals = [float(r["rod_drop_3p5_mm"]) for r in rows if r["condition"] == c and valid(r)]
    ax.scatter(i + np.linspace(-.09, .09, len(vals)), vals, s=25, color=colors[c], zorder=3)
    ax.errorbar(i, np.mean(vals), yerr=np.std(vals, ddof=1), fmt="D", ms=4, color="black", capsize=5)
ax.set_xticks([0, 1], ["无载荷前馈（n=10）", "有载荷前馈（n=10）"])
ax.set_ylabel("观测末次有效样本的杆竖向位移 (mm)")
save(fig, "载荷前馈_逐次下沉")

directional = [r for r in rows if r["study"] == "directional" and valid(r)]
lo, hi = -.4, min(ast.literal_eval(r["rod_series_t_s"])[-1] for r in directional)
grid = np.arange(lo, hi + 1e-9, .05)
fig, axes = plt.subplots(3, 1, figsize=(6.5, 6.8), sharex=True, constrained_layout=True)
for ax, body, label in zip(axes, ("rod", "left_hand", "right_hand"), ("杆", "左手", "右手")):
    for c in ("K400-N", "K400-F", "K200-N", "K200-F"):
        rr = [r for r in directional if r["condition"] == c]
        mat = []
        for r in rr:
            t = ast.literal_eval(r[body + "_series_t_s"])
            x = ast.literal_eval(r[body + "_series_x_mm"])
            assert grid[0] >= t[0] and grid[-1] <= t[-1]
            mat.append(np.interp(grid, t, x))
        mat = np.array(mat)
        ls = "--" if c.endswith("-N") else "-"
        for y in mat:
            ax.plot(grid, y, color=colors[c], ls=ls, alpha=.13, lw=.6)
        ax.plot(grid, mat.mean(0), color=colors[c], ls=ls, lw=1.6,
                label=f"{c}，n={len(rr)}")
    ax.axvspan(0, 1, color=".5", alpha=.09)
    ax.axhline(0, color=".5", lw=.6)
    ax.set_ylabel(label + "X向位移 (mm)")
axes[0].legend(frameon=False, ncol=2)
axes[-1].set_xlabel("相对实际或虚拟施力开始的时间 (s)")
save(fig, "方向性阻抗_原始位移")

fig, ax = plt.subplots(figsize=(6.3, 3.8), constrained_layout=True)
metrics = ("rod_x_pulse_end_mm", "rod_x_withdrawal_plus1_mm", "rod_x_withdrawal_near3_mm")
for c, offset in (("K400", -.06), ("K200", .06)):
    rr = [next(x for x in contrasts if x["metric"] == m and x["contrast"] == c + " force - no force") for m in metrics]
    means = np.array([float(r["estimate"]) for r in rr])
    low, high = [np.array([float(r[k]) for r in rr]) for k in ("ci_low", "ci_high")]
    ax.errorbar(np.arange(3) + offset, means, yerr=[means-low, high-means],
                fmt="o-", lw=1.2, capsize=4, color=colors[c + "-F"],
                label="X向刚度" + c[1:] + " N/m")
ax.set_xticks([0, 1, 2], ["脉冲结束", "撤力后1 s", "撤力后2.891 s"])
ax.set_ylabel("施力组减无外力组的杆位移估计 (mm)")
ax.legend(frameon=False)
save(fig, "方向性阻抗_响应与残余")
print(OUT)
