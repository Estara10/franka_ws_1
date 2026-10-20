#!/usr/bin/env python3
"""S2/S3 extended-scenario, rod-payload-feedforward and directional comparison.

Reads only the saved run CSVs/logs. It does not modify any historical run or
figure: every output goes to --report-dir / --fig-dir.

Windows
  FF: MMC window = 3.0 s from the first mmc_hold_trace row (first accepted
      reference). The post-wait window is 0.5..3.0 s because the requested
      hold was 3.0 s; the design intent was 0.5 s wait + 3.0 s, so only
      2.42-2.46 s remain after the wait. Both are reported.
  Directional: 5.0 s MMC hold; 1.0 s after arm the rod receives world +X 2 N
      for 1.0 s; the response is evaluated at pulse end, +1 s and +3 s.
"""
import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCENARIOS = [
    ('original 0.10 m/+10 deg', 's2s3_plan_orig_20261020'),
    ('long 0.18 m/+10 deg', 's2s3_plan_long_20261020'),
    ('rot 0.15 m/+25 deg', 's2s3_plan_rot_retry2_20261020'),
    ('rot backup 0.15 m/+20 deg', 's2s3_plan_rot_backup_20261020'),
]
FF_RUNS = {
    'off': ['s2s3_ff_off_01_20261020', 's2s3_ff_off_02_20261020',
            's2s3_ff_off_03_20261020'],
    'on': ['s2s3_ff_on_01_retry1_20261020', 's2s3_ff_on_02_20261020',
           's2s3_ff_on_03_20261020'],
}
DIR_RUNS = {
    'fixed': ['s2s3_dir_fixed_01_20261020', 's2s3_dir_fixed_02_20261020',
              's2s3_dir_fixed_03_20261020'],
    'soft': ['s2s3_dir_soft_01_retry1_20261020', 's2s3_dir_soft_02_20261020',
             's2s3_dir_soft_03_20261020'],
}
COLORS = {'baseline': '#4C72B0', 'geometry': '#DD8452', 'time_profile': '#55A868',
          'off': '#4C72B0', 'on': '#C44E52',
          'fixed': '#2878B5', 'soft': '#E1812C'}
METHOD_LABEL = {'baseline': 'baseline', 'geometry': 'geometry projection',
                'time_profile': 'time profile'}


def rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def fnum(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def med(values):
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def rng(values):
    values = [v for v in values if v is not None]
    return (min(values), max(values)) if values else (None, None)


# --------------------------------------------------------------------------- #
# 1. Transport planning comparison
# --------------------------------------------------------------------------- #
def transport_run(run_dir):
    summary = {r['plan']: r for r in rows(run_dir / 'transport_plan_summary.csv')}
    log = (run_dir / 'task_launch.log').read_text(errors='replace')
    opt = re.search(r'\[TRANSPORT_OPT\] label=transport_optimized status=(\w+)'
                    r'.*?reduction=([0-9.]+).*?optimize_sec=([0-9.]+)', log)
    tp = re.search(r'\[TIME_PROFILE_OPT\] duration=([0-9.]+).*?solve_sec=([0-9.]+)\s+'
                   r'continuous_jerk_rms=([0-9.]+)->([0-9.]+)\s+\(([-+0-9.]+)%\)'
                   r'\s+continuous_jerk_max=([0-9.]+)->([0-9.]+)', log)
    out = {'plans': summary, 'geometry_status': None, 'geometry_reduction': None,
           'geometry_solve_sec': None,
           'time_profile': None}
    if opt:
        out['geometry_status'] = opt.group(1)
        out['geometry_reduction'] = float(opt.group(2))
        out['geometry_solve_sec'] = float(opt.group(3))
    if tp:
        out['time_profile'] = {
            'duration': float(tp.group(1)), 'solve_sec': float(tp.group(2)),
            'continuous_baseline': float(tp.group(3)),
            'continuous_candidate': float(tp.group(4)),
            'continuous_change_pct': float(tp.group(5)),
            'continuous_max_baseline': float(tp.group(6)),
            'continuous_max_candidate': float(tp.group(7)),
        }
    return out


def plot_transport(data, out_path):
    labels = [label for label, _ in SCENARIOS]
    x = np.arange(len(labels))
    width = 0.25
    methods = [('baseline', 'baseline', 'baseline'),
               ('geometry_candidate', 'geometry', 'geometry projection'),
               ('time_profile_candidate', 'time_profile', 'time profile')]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    ax = axes[0][0]
    for i, (plan_key, color_key, label) in enumerate(methods):
        values = [float(data[label_]['plans'][plan_key]['jerk_rms']) for label_ in labels]
        ax.bar(x + (i - 1) * width, values, width, color=COLORS[color_key],
               label=label)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.set_ylabel('dense quintic jerk RMS (rad/s3)')
    ax.set_title('Dense jerk after export interpolation (20 ms resampling)')
    ax.grid(axis='y', alpha=0.25)
    ax.legend(fontsize=8)

    ax = axes[0][1]
    for i, (plan_key, color_key, label) in enumerate(methods):
        values = [float(data[label_]['plans'][plan_key]['duration_s']) for label_ in labels]
        ax.bar(x + (i - 1) * width, values, width, color=COLORS[color_key],
               label=label)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.set_ylabel('duration (s)')
    ax.set_title('Resulting duration from the same timing pipeline')
    ax.grid(axis='y', alpha=0.25)

    ax = axes[1][0]
    base = [data[label]['time_profile']['continuous_baseline'] for label in labels]
    cand = [data[label]['time_profile']['continuous_candidate'] for label in labels]
    ax.bar(x - width / 2, base, width, color=COLORS['baseline'], label='baseline')
    ax.bar(x + width / 2, cand, width, color=COLORS['time_profile'], label='time profile')
    for i, label in enumerate(labels):
        pct = data[label]['time_profile']['continuous_change_pct']
        ax.text(i, max(base[i], cand[i]) * 1.02, f'{pct:+.2f}%', ha='center', fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.set_ylabel('continuous jerk RMS (rad/s3)')
    ax.set_title('Time profile: reliable (continuous) jerk, not the export artifact')
    ax.grid(axis='y', alpha=0.25)
    ax.legend(fontsize=8)

    ax = axes[1][1]
    red = [100.0 * data[label]['geometry_reduction'] for label in labels]
    ax.bar(x, red, 0.5, color=COLORS['geometry'])
    for i, value in enumerate(red):
        ax.text(i, value + 0.02, f'{value:.2f}%', ha='center', fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha='right')
    ax.set_ylabel('objective reduction (%)')
    ax.set_title('Geometry projection: J0 -> J1 (30-iteration budget)')
    ax.grid(axis='y', alpha=0.25)

    fig.suptitle('S2 transport planning comparison by scenario '
                 '(baseline / geometry / fixed-geometry time profile)')
    fig.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close(fig)


# --------------------------------------------------------------------------- #
# 2. Payload feedforward
# --------------------------------------------------------------------------- #
HOLD_WINDOW = 3.0
WAIT_SEC = 0.5


def ff_run(run_dir, wait):
    trace = rows(run_dir / 'mmc_hold_trace.csv')
    if not trace:
        raise RuntimeError(f'{run_dir.name}: empty MMC trace')
    t0 = float(trace[0]['sim_absolute_sec'])
    probe = [r for r in rows(run_dir / 'contact_probe.csv')
             if r['phase'] == 'MMC_HOLD']
    selected = []
    for row in probe:
        t = fnum(row, 'right_sim_time')
        if t is None:
            continue
        rel = t - t0
        if wait - 1e-9 <= rel <= HOLD_WINDOW + 1e-9:
            selected.append((rel, row))
    if len(selected) < 2:
        raise RuntimeError(f'{run_dir.name}: fewer than 2 samples in window')
    t = np.array([item[0] for item in selected])
    series = {
        'rod': np.array([fnum(r, 'rod_z') * 1000.0 for _, r in selected]),
        'left': np.array([fnum(r, 'left_hand_z') * 1000.0 for _, r in selected]),
        'right': np.array([fnum(r, 'right_hand_z') * 1000.0 for _, r in selected]),
    }
    entry = selected[0][1]
    end = selected[-1][1]
    status = json.loads((run_dir / 'prepare_status.json').read_text())
    return {
        'run_id': run_dir.name,
        'wait_sec': wait,
        'window_start_from_hold_sec': float(t[0]),
        'window_end_from_hold_sec': float(t[-1]),
        'window_span_sec': float(t[-1] - t[0]),
        'samples': len(selected),
        'series': {'t': t.tolist(),
                   **{key: (value - value[0]).tolist() for key, value in series.items()}},
        'rod_dz_mm': float(series['rod'][-1] - series['rod'][0]),
        'left_hand_dz_mm': float(series['left'][-1] - series['left'][0]),
        'right_hand_dz_mm': float(series['right'][-1] - series['right'][0]),
        'entry_grasp_residual_l_mm': fnum(entry, 'left_slip_m') * 1000.0,
        'entry_grasp_residual_r_mm': fnum(entry, 'right_slip_m') * 1000.0,
        'residual_increment_l_mm': (fnum(end, 'left_slip_m') -
                                    fnum(entry, 'left_slip_m')) * 1000.0,
        'residual_increment_r_mm': (fnum(end, 'right_slip_m') -
                                    fnum(entry, 'right_slip_m')) * 1000.0,
        'completed': status.get('task_state') == 'DONE',
    }


def plot_feedforward(off, on, out_path):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    series = [('rod', '-', 'rod'), ('left', '--', 'left hand'), ('right', ':', 'right hand')]
    for col, (results, group, wait) in enumerate(
            [(off, 'off', WAIT_SEC), (on, 'on', WAIT_SEC)]):
        ax = axes[0][col]
        for key, style, label in series:
            for rec in results:
                ax.plot(rec['series']['t'], rec['series'][key], color=COLORS[group],
                        linestyle=style, linewidth=1.0, alpha=0.45)
            first_common = max(rec['series']['t'][0] for rec in results)
            last_common = min(rec['series']['t'][-1] for rec in results)
            grid = np.linspace(first_common, last_common, 60)
            med_curve = np.median(
                [np.interp(grid, rec['series']['t'], rec['series'][key]) for rec in results],
                axis=0)
            ax.plot(grid, med_curve, color=COLORS[group], linestyle=style, linewidth=2.2,
                    label=label)
        ax.axhline(0.0, color='0.5', linewidth=0.7)
        ax.set_xlim(wait, HOLD_WINDOW)
        ax.set_xlabel('time from first MMC reference (s)')
        ax.set_ylabel('world ΔZ from window start (mm)')
        observed_spans = [rec['post_window_span_sec'] for rec in results]
        ax.set_title(f'payload feedforward {"ON" if group == "on" else "OFF"}: '
                     f'target t={wait:.1f}-{HOLD_WINDOW:.1f} s; '
                     f'observed span {min(observed_spans):.2f}-{max(observed_spans):.2f} s')
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)

    ax = axes[1][0]
    values = {group: [r['rod_dz_mm'] for r in results]
              for group, results in [('off', off), ('on', on)]}
    positions = [0, 1]
    for i, group in enumerate(['off', 'on']):
        for j, value in enumerate(values[group]):
            ax.scatter(i + (j - 1) * 0.08, value, color=COLORS[group], s=36)
        ax.hlines(med(values[group]), i - 0.22, i + 0.22, color=COLORS[group], linewidth=3)
    ax.set_xticks(positions)
    ax.set_xticklabels(['FF off', 'FF on'])
    ax.set_ylabel('rod world ΔZ over the window (mm)')
    ax.set_title('Per-run rod vertical change (line = median)')
    off_med = med([r['full_rod_dz_mm'] for r in off])
    on_med = med([r['full_rod_dz_mm'] for r in on])
    off_range = rng([r['full_rod_dz_mm'] for r in off])
    on_range = rng([r['full_rod_dz_mm'] for r in on])
    ax.text(0.02, 0.03,
            f'Nominal full 0-3 s: off {off_med:+.2f} [{off_range[0]:+.2f}, {off_range[1]:+.2f}] mm; '
            f'on {on_med:+.2f} [{on_range[0]:+.2f}, {on_range[1]:+.2f}] mm (n=3 each)',
            transform=ax.transAxes, fontsize=8, va='bottom',
            bbox={'facecolor': 'white', 'alpha': 0.8, 'edgecolor': 'none'})
    ax.grid(axis='y', alpha=0.25)

    ax = axes[1][1]
    full_off = [r['full_rod_dz_mm'] for r in off]
    post_off = [r['rod_dz_mm'] for r in off]
    full_on = [r['full_rod_dz_mm'] for r in on]
    post_on = [r['rod_dz_mm'] for r in on]
    bars = [('off nominal 0-3 s', med(full_off), COLORS['off'], 0.45),
            ('off post-ramp', med(post_off), COLORS['off'], 1.0),
            ('on nominal 0-3 s', med(full_on), COLORS['on'], 0.45),
            ('on post-ramp', med(post_on), COLORS['on'], 1.0)]
    for i, (label, value, color, alpha) in enumerate(bars):
        ax.bar(i, value, 0.7, color=color, alpha=alpha, label=label)
        ax.text(i, value + (0.12 if value >= 0 else -0.35), f'{value:+.2f}',
                ha='center', fontsize=8)
    ax.axhline(0.0, color='0.5', linewidth=0.7)
    ax.set_xticks(range(4))
    ax.set_xticklabels([b[0] for b in bars], rotation=12, ha='right')
    ax.set_ylabel('median rod world ΔZ (mm)')
    descent_reduction_pct = (abs(med(full_off)) - abs(med(full_on))) / abs(med(full_off)) * 100.0
    post_spans = [r['post_window_span_sec'] for r in off + on]
    ax.set_ylim(top=max(2.5, ax.get_ylim()[1]))
    ax.set_title('Observed full-HOLD samples vs post-ramp samples')
    ax.text(0.02, 0.98,
            f'|median ΔZ| reduced {descent_reduction_pct:.1f}% from group medians (nominal 0-3 s). '
            f'Post-ramp t=0.5-3 s contains only {min(post_spans):.2f}-{max(post_spans):.2f} s sampled.',
            transform=ax.transAxes, fontsize=8, va='top',
            bbox={'facecolor': 'white', 'alpha': 0.88, 'edgecolor': 'none'})
    ax.grid(axis='y', alpha=0.25)
    ax.legend(fontsize=7)

    fig.suptitle('Rod payload feedforward: fixed-gain MMC hover, n=3/group; '
                 '0.5 s ramp is inside the 3.0 s hold, not followed by another 3 s window')
    fig.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close(fig)


# --------------------------------------------------------------------------- #
# 3. Directional compliance
# --------------------------------------------------------------------------- #
PULSE_DURATION = 1.0


def directional_run(run_dir):
    events = {r['event']: float(r['sim_time'])
              for r in rows(run_dir / 'rod_force_events.csv')}
    trace = rows(run_dir / 'mmc_hold_trace.csv')
    t0 = float(trace[0]['sim_absolute_sec'])
    hold_span = float(trace[-1]['sim_elapsed_sec']) - float(trace[0]['sim_elapsed_sec'])
    probe = [r for r in rows(run_dir / 'contact_probe.csv')
             if r['phase'] == 'MMC_HOLD']
    selected = []
    for row in probe:
        t = (fnum(row, 'left_sim_time') + fnum(row, 'right_sim_time')) / 2.0
        rel = t - t0
        if 0.0 - 1e-9 <= rel <= hold_span + 1e-9:
            selected.append((rel, row))
    if len(selected) < 10:
        raise RuntimeError(f'{run_dir.name}: too few MMC_HOLD samples')
    hold_t = np.array([item[0] for item in selected])
    sim_t = hold_t + t0
    pulse_t = sim_t - events['pulse_start']
    fields = {
        'rod_x': np.array([fnum(r, 'rod_x') for _, r in selected]) * 1000.0,
        'left_x': np.array([fnum(r, 'left_hand_x') for _, r in selected]) * 1000.0,
        'right_x': np.array([fnum(r, 'right_hand_x') for _, r in selected]) * 1000.0,
        'rod_z': np.array([fnum(r, 'rod_z') for _, r in selected]) * 1000.0,
        'left_z': np.array([fnum(r, 'left_hand_z') for _, r in selected]) * 1000.0,
        'right_z': np.array([fnum(r, 'right_hand_z') for _, r in selected]) * 1000.0,
        'rod_left_rel_x': (np.array([fnum(r, 'rod_x') - fnum(r, 'left_hand_x')
                                     for _, r in selected]) * 1000.0),
        'rod_right_rel_x': (np.array([fnum(r, 'rod_x') - fnum(r, 'right_hand_x')
                                      for _, r in selected]) * 1000.0),
    }
    pre = (pulse_t >= -0.5) & (pulse_t <= -0.05)
    if pre.sum() < 6 or (pulse_t[pre].max() - pulse_t[pre].min()) < 0.3:
        raise RuntimeError(f'{run_dir.name}: insufficient pre-pulse window')
    detrended = {}
    slopes = {}
    for key in ('rod_x', 'left_x', 'right_x'):
        slope, _ = np.polyfit(pulse_t[pre], fields[key][pre], 1)
        base = float(np.interp(0.0, pulse_t, fields[key]))
        slopes[key] = float(slope)
        detrended[key] = fields[key] - base - slope * pulse_t
    entry, end = selected[0][1], selected[-1][1]
    status = json.loads((run_dir / 'prepare_status.json').read_text())
    result = {
        'run_id': run_dir.name,
        'task_state': status.get('task_state'),
        'pulse_start_sim_sec': events['pulse_start'],
        'pulse_end_sim_sec': events['pulse_end'],
        'pulse_duration_sec': events['pulse_end'] - events['pulse_start'],
        'hold_span_sec': hold_span,
        'pre_pulse_samples': int(pre.sum()),
        'pre_pulse_span_sec': float(pulse_t[pre].max() - pulse_t[pre].min()),
        'pre_pulse_drift_slope_mm_s': slopes,
        'entry_grasp_residual_l_mm': fnum(entry, 'left_slip_m') * 1000.0,
        'entry_grasp_residual_r_mm': fnum(entry, 'right_slip_m') * 1000.0,
        'residual_increment_l_mm': (fnum(end, 'left_slip_m') -
                                    fnum(entry, 'left_slip_m')) * 1000.0,
        'residual_increment_r_mm': (fnum(end, 'right_slip_m') -
                                    fnum(entry, 'right_slip_m')) * 1000.0,
        'rod_dz_hold_mm': float(fields['rod_z'][-1] - fields['rod_z'][0]),
        'left_hand_dz_hold_mm': float(fields['left_z'][-1] - fields['left_z'][0]),
        'right_hand_dz_hold_mm': float(fields['right_z'][-1] - fields['right_z'][0]),
        'pulse_t': pulse_t.tolist(),
        'hold_t': hold_t.tolist(),
        'fields': {key: value.tolist() for key, value in fields.items()},
        'detrended': {key: value.tolist() for key, value in detrended.items()},
    }
    for key in ('rod_x', 'left_x', 'right_x'):
        base = float(np.interp(0.0, pulse_t, fields[key]))
        for label, at in (('pulse_end', PULSE_DURATION),
                          ('plus_1s', PULSE_DURATION + 1.0),
                          ('plus_3s', PULSE_DURATION + 3.0)):
            result[f'{key}_{label}_mm'] = float(np.interp(at, pulse_t, fields[key]) - base)
            result[f'{key}_{label}_detrended_mm'] = float(
                np.interp(at, pulse_t, detrended[key]))
    return result


def plot_directional(fixed, soft, out_path):
    grid = np.linspace(-0.5, 1.0 + 3.0, 141)
    hold_grid = np.linspace(0.0, 5.0, 121)
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    for group, results, color in [('fixed', fixed, COLORS['fixed']),
                                  ('soft', soft, COLORS['soft'])]:
        ax = axes[0][0]
        for rec in results:
            ax.plot(rec['pulse_t'], [
                value - float(np.interp(0.0, rec['pulse_t'], rec['fields']['rod_x']))
                for value in rec['fields']['rod_x']],
                color=color, alpha=0.25, linewidth=0.8)
        median_curve = np.median(
            [np.interp(grid, rec['pulse_t'],
                       [v - np.interp(0.0, rec['pulse_t'], rec['fields']['rod_x'])
                        for v in rec['fields']['rod_x']]) for rec in results], axis=0)
        ax.plot(grid, median_curve, color=color, linewidth=2.2, label=group)

        ax = axes[1][0]
        for key, style, label in [('left_x', '--', 'left hand'), ('right_x', ':', 'right hand')]:
            for rec in results:
                ax.plot(rec['pulse_t'], [
                    v - float(np.interp(0.0, rec['pulse_t'], rec['fields'][key]))
                    for v in rec['fields'][key]],
                    color=color, linestyle=style, alpha=0.2, linewidth=0.7)
            med_curve = np.median(
                [np.interp(grid, rec['pulse_t'],
                           [v - np.interp(0.0, rec['pulse_t'], rec['fields'][key])
                            for v in rec['fields'][key]]) for rec in results], axis=0)
            ax.plot(grid, med_curve, color=color, linestyle=style, linewidth=2.0,
                    label=f'{group} {label}')

        ax = axes[1][1]
        for rec in results:
            ax.plot(rec['hold_t'], [
                v - rec['fields']['rod_z'][0] for v in rec['fields']['rod_z']],
                color=color, alpha=0.22, linewidth=0.8)
        med_curve = np.median(
            [np.interp(hold_grid, rec['hold_t'],
                       [v - rec['fields']['rod_z'][0] for v in rec['fields']['rod_z']])
             for rec in results], axis=0)
        ax.plot(hold_grid, med_curve, color=color, linewidth=2.2, label=group)

    ax = axes[0][0]
    ax.axvspan(0.0, PULSE_DURATION, color='0.2', alpha=0.10, label='+X 2 N on')
    ax.axvline(PULSE_DURATION, color='0.3', linewidth=0.8, linestyle='--')
    ax.axhline(0.0, color='0.5', linewidth=0.7)
    ax.set_xlim(-0.5, 4.0)
    ax.set_xlabel('time from pulse start (s)')
    ax.set_ylabel('rod world ΔX from pulse start (mm)')
    ax.set_title('Raw rod X response (thin = run, thick = median)')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    ax = axes[1][0]
    ax.axvspan(0.0, PULSE_DURATION, color='0.2', alpha=0.10)
    ax.axhline(0.0, color='0.5', linewidth=0.7)
    ax.set_xlim(-0.5, 4.0)
    ax.set_xlabel('time from pulse start (s)')
    ax.set_ylabel('hand world ΔX (mm)')
    ax.set_title('Hand X response')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7)

    ax = axes[1][1]
    ax.axvspan(1.0, 2.0, color='0.2', alpha=0.10, label='pulse (1-2 s)')
    ax.axhline(0.0, color='0.5', linewidth=0.7)
    ax.set_xlim(0.0, 5.0)
    ax.set_xlabel('time from MMC HOLD arm (s)')
    ax.set_ylabel('rod world ΔZ from HOLD entry (mm)')
    ax.set_title('Vertical hold with feedforward enabled')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # Second panel: bars at the requested times.
    ax = axes[0][1]
    labels = ['pulse end', '+1 s', '+3 s']
    xpos = np.arange(3)
    for offset, group, results in [(-0.18, 'fixed', fixed), (0.18, 'soft', soft)]:
        medians = [med([r[f'rod_x_{key}_mm'] for r in results])
                   for key in ('pulse_end', 'plus_1s', 'plus_3s')]
        ax.bar(xpos + offset, medians, 0.34, color=COLORS[group], label=group)
        for i, key in enumerate(('pulse_end', 'plus_1s', 'plus_3s')):
            for value in [r[f'rod_x_{key}_mm'] for r in results]:
                ax.scatter(i + offset, value, color='k', s=10, zorder=3)
    ax.set_xticks(xpos)
    ax.set_xticklabels(labels)
    ax.set_ylabel('rod world ΔX from pulse start (mm)')
    ax.set_title('Raw response at fixed times (bars = group medians; dots = runs)')
    ax.grid(axis='y', alpha=0.25)
    ax.legend(fontsize=8)
    pulse_end_fixed = med([r['rod_x_pulse_end_mm'] for r in fixed])
    pulse_end_soft = med([r['rod_x_pulse_end_mm'] for r in soft])
    residual_fixed = med([r['rod_x_plus_3s_mm'] for r in fixed])
    residual_soft = med([r['rod_x_plus_3s_mm'] for r in soft])
    pulse_pct = (pulse_end_soft / pulse_end_fixed - 1.0) * 100.0
    residual_pct = (residual_soft / residual_fixed - 1.0) * 100.0
    ax.set_ylim(0.0, max(3.2, ax.get_ylim()[1]))
    ax.text(0.5, 0.97,
            f'From group medians: pulse-end displacement +{pulse_pct:.1f}%; '
            f'+3 s residual +{residual_pct:.1f}% (n=3/group; descriptive only)',
            transform=ax.transAxes, fontsize=8, ha='center', va='top',
            bbox={'facecolor': 'white', 'alpha': 0.88, 'edgecolor': 'none'})

    fig.suptitle('Directional MMC: lower X stiffness increases displacement and residual; '
                 'same payload feedforward, world +X total 2 N for 1 s, 5 s hold')
    fig.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close(fig)


def plot_directional_detrended(fixed, soft, out_path):
    grid = np.linspace(-0.5, 4.0, 141)
    fig, ax = plt.subplots(figsize=(11, 5.6), constrained_layout=True)
    for group, results, color in [('fixed', fixed, COLORS['fixed']),
                                  ('soft', soft, COLORS['soft'])]:
        for key, style in [('rod_x', '-'), ('left_x', '--'), ('right_x', ':')]:
            for rec in results:
                ax.plot(rec['pulse_t'], rec['detrended'][key], color=color,
                        linestyle=style, alpha=0.2, linewidth=0.7)
            med_curve = np.median(
                [np.interp(grid, rec['pulse_t'], rec['detrended'][key])
                 for rec in results], axis=0)
            ax.plot(grid, med_curve, color=color, linestyle=style, linewidth=2.0,
                    label=f'{group} {key.replace("_x", "")}')
    ax.axvspan(0.0, PULSE_DURATION, color='0.2', alpha=0.10, label='+X 2 N on')
    ax.axhline(0.0, color='0.5', linewidth=0.7)
    ax.set_xlim(-0.5, 4.0)
    ax.set_xlabel('time from pulse start (s)')
    ax.set_ylabel('ΔX after pre-pulse linear-drift subtraction (mm)')
    ax.set_title('Auxiliary de-trended response (assumes the pre-pulse drift continues)')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    fig.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close(fig)


def write_csv(path, records, fields):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key) for key in fields})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root', type=Path,
                        default=Path('experiment_results/a1_runs'))
    parser.add_argument('--report-dir', type=Path,
                        default=Path('experiment_results/reports/s2s3_extended_20261020'))
    parser.add_argument('--fig-dir', type=Path,
                        default=Path('docs/figures/s2s3_extended_20261020'))
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    args.fig_dir.mkdir(parents=True, exist_ok=True)

    # 1. Transport
    transport = {}
    transport_rows = []
    for label, run_id in SCENARIOS:
        run_dir = args.runs_root / run_id
        data = transport_run(run_dir)
        transport[label] = data
        for method in ('baseline', 'geometry_candidate', 'time_profile_candidate'):
            plan = data['plans'][method]
            transport_rows.append({
                'scenario': label, 'run_id': run_id, 'plan': method,
                'target_x': plan['scenario_target_x'],
                'target_y': plan['scenario_target_y'],
                'target_yaw_deg': plan['scenario_target_yaw_deg'],
                'transport_dx': plan['transport_dx'],
                'transport_dyaw_deg': plan['transport_dyaw_deg'],
                'points': plan['points'], 'duration_s': plan['duration_s'],
                'planning_sec': plan['planning_sec'],
                'collisions': plan['collisions'],
                'limit_violation': plan['limit_violation'],
                'vel_violation': plan['vel_violation'],
                'acc_violation': plan['acc_violation'],
                'jerk_rms': plan['jerk_rms'], 'jerk_max': plan['jerk_max'],
                'continuous_jerk_rms': plan['continuous_jerk_rms'],
                'continuous_jerk_max': plan['continuous_jerk_max'],
                'exported_jerk_rms': plan['exported_jerk_rms'],
                'exported_jerk_max': plan['exported_jerk_max'],
                'max_export_deviation_rad': plan['max_export_deviation_rad'],
                'max_sample_dual_pos_mm': plan['max_sample_dual_pos_mm'],
                'max_rod_pos_mm': plan['max_rod_pos_mm'], 'pass': plan['pass'],
                'geometry_reduction': data['geometry_reduction'],
                'time_profile_continuous_change_pct':
                    data['time_profile']['continuous_change_pct'],
            })
    write_csv(args.report_dir / 'transport_plan_comparison.csv', transport_rows,
              list(transport_rows[0].keys()))
    plot_transport(transport, args.fig_dir / 'transport_plan_comparison.png')

    # 2. Feedforward
    ff = {}
    for group, run_ids in FF_RUNS.items():
        ff[group] = []
        for run_id in run_ids:
            run_dir = args.runs_root / run_id
            full = ff_run(run_dir, 0.0)
            post = ff_run(run_dir, WAIT_SEC)
            full['full_rod_dz_mm'] = full['rod_dz_mm']
            full['full_left_hand_dz_mm'] = full['left_hand_dz_mm']
            full['full_right_hand_dz_mm'] = full['right_hand_dz_mm']
            full['post_window_start_from_hold_sec'] = post['window_start_from_hold_sec']
            full['post_window_end_from_hold_sec'] = post['window_end_from_hold_sec']
            full['rod_dz_mm'] = post['rod_dz_mm']
            full['left_hand_dz_mm'] = post['left_hand_dz_mm']
            full['right_hand_dz_mm'] = post['right_hand_dz_mm']
            full['post_window_span_sec'] = post['window_span_sec']
            full['post_samples'] = post['samples']
            full['series'] = post['series']
            full['group'] = group
            ff[group].append(full)
    ff_fields = ['group', 'run_id', 'task_state', 'wait_sec',
                 'window_start_from_hold_sec', 'window_end_from_hold_sec', 'window_span_sec',
                 'post_window_start_from_hold_sec', 'post_window_end_from_hold_sec',
                 'post_window_span_sec', 'samples', 'post_samples',
                 'full_rod_dz_mm', 'rod_dz_mm', 'full_left_hand_dz_mm',
                 'left_hand_dz_mm', 'full_right_hand_dz_mm', 'right_hand_dz_mm',
                 'entry_grasp_residual_l_mm', 'entry_grasp_residual_r_mm',
                 'residual_increment_l_mm', 'residual_increment_r_mm', 'completed']
    ff_records = []
    for group in ('off', 'on'):
        for rec in ff[group]:
            rec['task_state'] = 'DONE' if rec['completed'] else 'FAILED'
            ff_records.append({key: rec.get(key) for key in ff_fields})
    write_csv(args.report_dir / 'feedforward_summary.csv', ff_records, ff_fields)
    plot_feedforward(ff['off'], ff['on'], args.fig_dir / 'feedforward_hover_comparison.png')
    (args.report_dir / 'feedforward_summary.json').write_text(
        json.dumps({group: [{k: v for k, v in rec.items() if k != 'series'}
                            for rec in ff[group]] for group in ff},
                   indent=2, ensure_ascii=False) + '\n')

    # 3. Directional
    directional = {}
    dir_records = []
    for group, run_ids in DIR_RUNS.items():
        directional[group] = []
        for run_id in run_ids:
            rec = directional_run(args.runs_root / run_id)
            rec['group'] = group
            directional[group].append(rec)
            dir_records.append(rec)
    dir_fields = ['group', 'run_id', 'task_state', 'pulse_start_sim_sec',
                  'pulse_end_sim_sec', 'pulse_duration_sec', 'hold_span_sec',
                  'pre_pulse_samples', 'pre_pulse_span_sec',
                  'rod_dz_hold_mm', 'left_hand_dz_hold_mm', 'right_hand_dz_hold_mm',
                  'entry_grasp_residual_l_mm', 'entry_grasp_residual_r_mm',
                  'residual_increment_l_mm', 'residual_increment_r_mm']
    for key in ('rod_x', 'left_x', 'right_x'):
        for label in ('pulse_end', 'plus_1s', 'plus_3s'):
            dir_fields += [f'{key}_{label}_mm', f'{key}_{label}_detrended_mm']
    dir_fields += ['rod_x_pre_pulse_drift_slope_mm_s',
                   'left_x_pre_pulse_drift_slope_mm_s',
                   'right_x_pre_pulse_drift_slope_mm_s']
    for rec in dir_records:
        for key in ('rod_x', 'left_x', 'right_x'):
            rec[f'{key}_pre_pulse_drift_slope_mm_s'] = rec['pre_pulse_drift_slope_mm_s'][key]
    write_csv(args.report_dir / 'directional_summary.csv', dir_records, dir_fields)
    plot_directional(directional['fixed'], directional['soft'],
                     args.fig_dir / 'directional_pulse_comparison.png')
    plot_directional_detrended(directional['fixed'], directional['soft'],
                               args.fig_dir / 'directional_pulse_detrended.png')

    print('transport:', args.report_dir / 'transport_plan_comparison.csv')
    for group in ('off', 'on'):
        recs = ff[group]
        print(f'FF {group}: full rod_dz median {med([r["full_rod_dz_mm"] for r in recs]):+.3f} mm, '
              f'post-wait {med([r["rod_dz_mm"] for r in recs]):+.3f} mm, '
              f'completed {sum(r["completed"] for r in recs)}/{len(recs)}')
    for group in ('fixed', 'soft'):
        recs = directional[group]
        print(f'Directional {group}: rod_x end {med([r["rod_x_pulse_end_mm"] for r in recs]):+.3f} mm, '
              f'+1s {med([r["rod_x_plus_1s_mm"] for r in recs]):+.3f} mm, '
              f'+3s {med([r["rod_x_plus_3s_mm"] for r in recs]):+.3f} mm, '
              f'rod dz {med([r["rod_dz_hold_mm"] for r in recs]):+.3f} mm')
    print('figures:', args.fig_dir)


if __name__ == '__main__':
    main()
