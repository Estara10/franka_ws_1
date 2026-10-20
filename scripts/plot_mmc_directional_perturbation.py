#!/usr/bin/env python3
"""Summarize the fixed/directional MMC +X pulse comparison from run CSVs."""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


FIXED_RUNS = [
    "contact_hover_directional_fixed_01_retry1",
    "contact_hover_directional_fixed_02",
    "contact_hover_directional_fixed_03",
]
DIRECTIONAL_RUNS = [
    "contact_hover_directional_soft_01",
    "contact_hover_directional_soft_02",
    "contact_hover_directional_soft_03",
]


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def value(row, key):
    return float(row[key])


def quaternion_angle_deg(rows):
    q0 = np.array([value(rows[0], f"rod_q{axis}") for axis in "xyzw"], dtype=float)
    q0 /= np.linalg.norm(q0)
    out = []
    for row in rows:
        q = np.array([value(row, f"rod_q{axis}") for axis in "xyzw"], dtype=float)
        q /= np.linalg.norm(q)
        dot = min(1.0, max(-1.0, abs(float(np.dot(q0, q)))))
        out.append(math.degrees(2.0 * math.acos(dot)))
    return np.asarray(out)


def load_run(run_id, root):
    run_dir = root / run_id
    force_events = read_rows(run_dir / "rod_force_events.csv")
    event = {row["event"]: float(row["sim_time"]) for row in force_events}
    probe = [row for row in read_rows(run_dir / "contact_probe.csv") if row["phase"] == "MMC_HOLD"]
    diagnostic = [row for row in read_rows(run_dir / "hover_diagnostics.csv") if row["phase"] == "MMC_HOLD"]
    mmc_trace = read_rows(run_dir / "mmc_hold_trace.csv")
    if len(probe) < 5 or len(diagnostic) < 5 or not {"armed", "pulse_start", "pulse_end"}.issubset(event):
        raise RuntimeError(f"{run_id}: missing MMC trace or physical force event rows")

    trace_start = float(mmc_trace[0]["sim_absolute_sec"])
    hold_span = float(mmc_trace[-1]["sim_elapsed_sec"]) - float(mmc_trace[0]["sim_elapsed_sec"])
    sim_t = np.array([(value(r, "left_sim_time") + value(r, "right_sim_time")) / 2.0 for r in probe])
    pulse_t = sim_t - event["pulse_start"]
    hold_t = sim_t - trace_start
    within_hold = (hold_t >= 0.0) & (hold_t <= hold_span)
    sim_t, pulse_t, hold_t = sim_t[within_hold], pulse_t[within_hold], hold_t[within_hold]
    probe = [row for row, keep in zip(probe, within_hold) if keep]
    diagnostic_times = np.array([(value(r, "left_sim_time") + value(r, "right_sim_time")) / 2.0 - trace_start
                                 for r in diagnostic])
    diag_keep = (diagnostic_times >= 0.0) & (diagnostic_times <= hold_span)
    diagnostic = [row for row, keep in zip(diagnostic, diag_keep) if keep]
    diag_hold_t = diagnostic_times[diag_keep]
    q_angle = quaternion_angle_deg(diagnostic)
    skew_ms = np.array([abs(value(r, "feedback_skew_sec")) * 1000.0 for r in diagnostic])
    fields = {
        "rod_x": np.array([value(r, "rod_x") for r in probe]) * 1000.0,
        "left_x": np.array([value(r, "left_hand_x") for r in probe]) * 1000.0,
        "right_x": np.array([value(r, "right_hand_x") for r in probe]) * 1000.0,
        "rod_z": np.array([value(r, "rod_z") for r in probe]) * 1000.0,
        "left_z": np.array([value(r, "left_hand_z") for r in probe]) * 1000.0,
        "right_z": np.array([value(r, "right_hand_z") for r in probe]) * 1000.0,
        "rod_left_rel_x": (np.array([value(r, "rod_x") - value(r, "left_hand_x") for r in probe]) * 1000.0),
        "rod_right_rel_x": (np.array([value(r, "rod_x") - value(r, "right_hand_x") for r in probe]) * 1000.0),
    }
    # Fit a local pre-pulse linear trend over [-0.5, -0.05] s. The detrended
    # response assumes this background drift would continue linearly after
    # force onset; raw pulse-aligned traces remain the primary observation.
    pre = (pulse_t >= -0.5) & (pulse_t <= -0.05)
    pre_count = int(np.count_nonzero(pre))
    pre_span = float(np.max(pulse_t[pre]) - np.min(pulse_t[pre])) if pre_count else 0.0
    if pre_count < 6 or pre_span < 0.3:
        raise RuntimeError(f"{run_id}: insufficient pre-pulse window for local drift fit ({pre_count} samples, {pre_span:.3f}s)")
    drift_slopes = {}
    detrended_fields = {}
    for key in ("rod_x", "left_x", "right_x"):
        slope, _ = np.polyfit(pulse_t[pre], fields[key][pre], 1)
        x_at_force_start = float(np.interp(0.0, pulse_t, fields[key]))
        drift_slopes[key] = float(slope)
        detrended_fields[key] = fields[key] - x_at_force_start - slope * pulse_t
    # Quaternions and robot-state feedback are sampled independently; keep the
    # finite recorded samples and do not describe them as synchronized.
    task_log = (run_dir / "task_launch.log").read_text(encoding="utf-8", errors="replace")
    result = re.search(r"\[CONTACT_HOVER_RESULT\].*", task_log)
    released = "[CONTACT_HOVER_RELEASED]" in task_log
    return {
        "run_id": run_id,
        "arm_time_sim_sec": event["armed"],
        "pulse_start_sim_sec": event["pulse_start"],
        "pulse_end_sim_sec": event["pulse_end"],
        "pulse_delay_sec": event["pulse_start"] - event["armed"],
        "pulse_duration_sec": event["pulse_end"] - event["pulse_start"],
        "hold_span_sec": hold_span,
        "monitor_span_sec": float(sim_t[-1] - sim_t[0]),
        "max_feedback_skew_ms": float(np.max(skew_ms)),
        "pre_pulse_fit_samples": pre_count,
        "pre_pulse_fit_span_sec": pre_span,
        "pre_pulse_drift_slope_mm_s": drift_slopes,
        "max_rod_attitude_change_deg": float(np.max(q_angle)),
        "rod_dz_hold_mm": float(fields["rod_z"][-1] - fields["rod_z"][0]),
        "left_hand_dz_hold_mm": float(fields["left_z"][-1] - fields["left_z"][0]),
        "right_hand_dz_hold_mm": float(fields["right_z"][-1] - fields["right_z"][0]),
        "task_result": result.group(0) if result else "missing",
        "released": released,
        "pulse_t": pulse_t,
        "hold_t": hold_t,
        "diag_hold_t": diag_hold_t,
        "q_angle": q_angle,
        "fields": fields,
        "detrended_fields": detrended_fields,
    }


def interpolate(series, time, grid):
    order = np.argsort(time)
    return np.interp(grid, time[order], series[order])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-root", type=Path, default=Path("experiment_results/a1_runs"))
    parser.add_argument("--output-dir", type=Path,
                        default=Path("experiment_results/reports/hover_directional_perturbation_20261020"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    fixed = [load_run(run_id, args.runs_root) for run_id in FIXED_RUNS]
    directional = [load_run(run_id, args.runs_root) for run_id in DIRECTIONAL_RUNS]
    groups = [
        {"label": "Fixed gain", "color": "#2878B5", "runs": fixed},
        {"label": "Directional Kx=200", "color": "#E1812C", "runs": directional},
    ]

    pulse_grid = np.linspace(-0.5, 1.5, 101)
    hold_grid = np.linspace(0.0, 3.0, 121)
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.3), constrained_layout=True)
    fig.suptitle("MMC hover response to a world +X rod-force pulse", fontsize=17, fontweight="semibold")

    ax = axes[0, 0]
    for group in groups:
        for key, label in [("rod_x", "rod"), ("left_x", "left hand"), ("right_x", "right hand")]:
            # Keep a distinct line style for the three bodies; each group gets
            # the same visual mapping in all panels.
            runs = group["runs"]
            traces = []
            for run in runs:
                baseline_t = -0.1
                time = run["pulse_t"]
                s = run["fields"][key]
                base = float(np.interp(baseline_t, np.sort(time), s[np.argsort(time)]))
                tr = interpolate(s - base, time, pulse_grid)
                traces.append(tr)
                ax.plot(pulse_grid, tr, color=group["color"], alpha=0.15, linewidth=0.7,
                        linestyle={"rod": "-", "left hand": "--", "right hand": ":"}[label])
            med = np.median(np.asarray(traces), axis=0)
            ax.plot(pulse_grid, med, color=group["color"], linewidth=2.0,
                    linestyle={"rod": "-", "left hand": "--", "right hand": ":"}[label],
                    label=f"{group['label']} — {label}")
    ax.axvspan(0, 0.2, color="#333333", alpha=0.1, label="force on")
    ax.axvline(0.2, color="#333333", linewidth=0.8, linestyle="--")
    ax.axhline(0, color="#333333", linewidth=0.7)
    ax.set_title("World-X displacement, pulse-aligned")
    ax.set_xlabel("Time from measured pulse start (s)")
    ax.set_ylabel("ΔX from 0.1 s pre-pulse (mm)")
    ax.grid(True, alpha=0.24)
    ax.legend(fontsize=8, ncol=2, frameon=True)

    ax = axes[0, 1]
    for group in groups:
        for key, label, style in [("rod_z", "rod", "-"), ("left_z", "left hand", "--"), ("right_z", "right hand", ":")]:
            traces = []
            for run in group["runs"]:
                time = run["hold_t"]
                s = run["fields"][key]
                baseline = float(np.interp(0.0, time, s))
                trace = interpolate(s - baseline, time, hold_grid)
                traces.append(trace)
                ax.plot(hold_grid, trace, color=group["color"], alpha=0.14,
                        linewidth=0.7, linestyle=style)
            ax.plot(hold_grid, np.median(np.asarray(traces), axis=0), color=group["color"],
                    linewidth=2.0, linestyle=style, label=f"{group['label']} — {label}")
    ax.axvspan(1.0, 1.2, color="#333333", alpha=0.1, label="scheduled pulse")
    ax.axhline(0, color="#333333", linewidth=0.7)
    ax.set_title("Vertical drift during MMC HOLD")
    ax.set_xlabel("Time from MMC HOLD arm (s)")
    ax.set_ylabel("ΔZ from HOLD entry (mm)")
    ax.grid(True, alpha=0.24)
    ax.legend(fontsize=8, ncol=2, frameon=True)

    ax = axes[1, 0]
    for group in groups:
        for key, label, style in [("rod_left_rel_x", "rod − left hand", "-"),
                                  ("rod_right_rel_x", "rod − right hand", "--")]:
            traces = []
            for run in group["runs"]:
                time = run["pulse_t"]
                s = run["fields"][key]
                baseline_t = -0.1
                baseline = float(np.interp(baseline_t, np.sort(time), s[np.argsort(time)]))
                trace = interpolate(s - baseline, time, pulse_grid)
                traces.append(trace)
                ax.plot(pulse_grid, trace, color=group["color"], alpha=0.15,
                        linewidth=0.7, linestyle=style)
            ax.plot(pulse_grid, np.median(np.asarray(traces), axis=0), color=group["color"],
                    linewidth=2.0, linestyle=style, label=f"{group['label']} — {label}")
    ax.axvspan(0, 0.2, color="#333333", alpha=0.1)
    ax.axhline(0, color="#333333", linewidth=0.7)
    ax.set_title("Rod motion relative to each hand")
    ax.set_xlabel("Time from measured pulse start (s)")
    ax.set_ylabel("Δ relative X (mm; asynchronous samples)")
    ax.grid(True, alpha=0.24)
    ax.legend(fontsize=8, ncol=2, frameon=True)

    ax = axes[1, 1]
    for group in groups:
        for run in group["runs"]:
            ax.plot(run["diag_hold_t"], run["q_angle"], color=group["color"], alpha=0.18, linewidth=0.7)
        stacked_t = np.vstack([np.interp(hold_grid, r["diag_hold_t"], r["q_angle"]) for r in group["runs"]])
        ax.plot(hold_grid, np.median(stacked_t, axis=0), color=group["color"], linewidth=2.5,
                label=f"{group['label']} median (n=3)")
    ax.axvspan(1.0, 1.2, color="#333333", alpha=0.1)
    ax.set_title("Rod attitude change from HOLD entry")
    ax.set_xlabel("Time from MMC HOLD arm (s)")
    ax.set_ylabel("Rod orientation change (deg)")
    ax.grid(True, alpha=0.24)
    ax.legend(frameon=True)

    fig.text(0.5, -0.01,
             "Force is total 2 N at the rod COM in world +X for 0.2 s of simulation time; "
             "curves are descriptive medians of three runs, with individual runs faint. "
             "Samples are finite-rate and left/right feedback is asynchronous.",
             ha="center", fontsize=9)
    image_path = args.output_dir / "mmc_directional_perturbation_comparison.png"
    fig.savefig(image_path, dpi=300, bbox_inches="tight")
    plt.close(fig)

    detrended_fig, detrended_ax = plt.subplots(figsize=(10.5, 5.5), constrained_layout=True)
    for group in groups:
        for key, label, style in [("rod_x", "rod", "-"), ("left_x", "left hand", "--"),
                                  ("right_x", "right hand", ":")]:
            traces = []
            for run in group["runs"]:
                trace = interpolate(run["detrended_fields"][key], run["pulse_t"], pulse_grid)
                traces.append(trace)
                detrended_ax.plot(pulse_grid, trace, color=group["color"], alpha=0.18,
                                  linewidth=0.8, linestyle=style)
            detrended_ax.plot(pulse_grid, np.median(np.asarray(traces), axis=0),
                              color=group["color"], linewidth=2.2, linestyle=style,
                              label=f"{group['label']} — {label} median")
    detrended_ax.axvspan(0.0, 0.2, color="#333333", alpha=0.1, label="2 N force on")
    detrended_ax.axhline(0.0, color="#333333", linewidth=0.7)
    detrended_ax.set_title("Pulse-aligned world-X response after local linear-drift subtraction")
    detrended_ax.set_xlabel("Time from measured pulse start (s)")
    detrended_ax.set_ylabel("ΔX minus extrapolated pre-pulse drift (mm)")
    detrended_ax.grid(True, alpha=0.24)
    detrended_ax.legend(fontsize=9, ncol=2)
    detrended_fig.text(0.5, -0.015,
                       "Per-run linear fit over −0.5…−0.05 s (9 samples each, 0.397–0.412 s); assumes local drift continues linearly after onset. "
                       "Finite-rate asynchronous measurements; not a direct isolated force response.",
                       ha="center", fontsize=9)
    detrended_path = args.output_dir / "mmc_directional_pulse_detrended.png"
    detrended_fig.savefig(detrended_path, dpi=300, bbox_inches="tight")
    plt.close(detrended_fig)

    summary = []
    for group_name, runs in [("fixed", fixed), ("directional", directional)]:
        for run in runs:
            pulse_t = run["pulse_t"]
            fields = run["fields"]
            row = {k: v for k, v in run.items() if k not in {"pulse_t", "hold_t", "diag_hold_t", "q_angle", "fields", "detrended_fields"}}
            for key in ["rod_x", "left_x", "right_x", "rod_left_rel_x", "rod_right_rel_x"]:
                series = fields[key]
                base = float(np.interp(-0.1, np.sort(pulse_t), series[np.argsort(pulse_t)]))
                row[f"{key}_delta_at_pulse_end_mm"] = float(np.interp(0.2, np.sort(pulse_t), series[np.argsort(pulse_t)]) - base)
                row[f"{key}_delta_1s_after_end_mm"] = float(np.interp(1.2, np.sort(pulse_t), series[np.argsort(pulse_t)]) - base)
            row["group"] = group_name
            for key in ["rod_x", "left_x", "right_x"]:
                row[f"{key}_pre_pulse_drift_slope_mm_s"] = run["pre_pulse_drift_slope_mm_s"][key]
                values = run["detrended_fields"][key]
                row[f"{key}_detrended_at_pulse_end_mm"] = float(np.interp(0.2, np.sort(pulse_t), values[np.argsort(pulse_t)]))
                row[f"{key}_detrended_1s_after_end_mm"] = float(np.interp(1.2, np.sort(pulse_t), values[np.argsort(pulse_t)]))
            summary.append(row)

    csv_path = args.output_dir / "run_summary.csv"
    keys = list(summary[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(summary)
    json_path = args.output_dir / "run_summary.json"
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"saved {image_path}")
    print(f"saved {detrended_path}")
    print(f"saved {csv_path}")
    for group in ["fixed", "directional"]:
        rows = [r for r in summary if r["group"] == group]
        print(group, "n=", len(rows), "hold span median=", round(float(np.median([r["hold_span_sec"] for r in rows])), 3),
              "rod dx end median=", round(float(np.median([r["rod_x_delta_at_pulse_end_mm"] for r in rows])), 3),
              "rod dx +1s median=", round(float(np.median([r["rod_x_delta_1s_after_end_mm"] for r in rows])), 3),
              "rod dz median=", round(float(np.median([r["rod_dz_hold_mm"] for r in rows])), 3),
              "attitude max median=", round(float(np.median([r["max_rod_attitude_change_deg"] for r in rows])), 3))


if __name__ == "__main__":
    main()
