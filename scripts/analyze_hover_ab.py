#!/usr/bin/env python3
"""Read-only A/B hover comparison: JTC hold vs fixed-gain MMC hold.

Window definition
  A (JTC): first HOVER_HOLD sample after lift; 3.0 s from that sample.
  B (MMC): first mmc_hold_trace sample after the MMC first reference; 3.0 s
           from that absolute simulation time.

The task's whole MMC_HOLD phase also contains the gate/switch and switch-back;
those are excluded from the 3 s window and reported separately.

All values are read from contact_probe.csv; rod truth, joint feedback and
contact forces are asynchronous samples, not one synchronized measurement.
The task's legacy ``*_slip_m`` values are reported as grasp-reference residuals:
inferred rod-centre mismatch from the original grasp transform. They are not
direct measurements of local pad/tip tangential contact slip.
"""
import argparse
import csv
import re
import json
import math
import statistics
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DEFAULT_RUNS = [
    ('A', 'contact_hover_ab_lidio_20261020_A1_retry2'),
    ('A', 'contact_hover_ab_lidio_20261020_A2'),
    ('A', 'contact_hover_ab_lidio_20261020_A3'),
    ('B', 'contact_hover_ab_lidio_20261020_B1'),
    ('B', 'contact_hover_ab_lidio_20261020_B2'),
    ('B', 'contact_hover_ab_lidio_20261020_B3_retry1'),
]
WINDOW_SEC = 3.0
COLORS = {'A': '#1f77b4', 'B': '#d62728'}
SERIES = [('rod', 'rod', '-'), ('left hand', 'left_hand', '--'),
          ('right hand', 'right_hand', ':')]


def num(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def quat_angle_deg(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def load_window(run_dir, group):
    rows = list(csv.DictReader((run_dir / 'contact_probe.csv').open()))
    if group == 'A':
        phase = [r for r in rows if r['phase'] == 'HOVER_HOLD']
        if not phase:
            raise ValueError(f'{run_dir.name}: no HOVER_HOLD samples')
        t0 = num(phase[0], 'right_sim_time')
        selected = [r for r in phase if num(r, 'right_sim_time') - t0 <= WINDOW_SEC + 1e-9]
        switch_sec = 0.0
    else:
        phase = [r for r in rows if r['phase'] == 'MMC_HOLD']
        trace = list(csv.DictReader((run_dir / 'mmc_hold_trace.csv').open()))
        if not phase or not trace:
            raise ValueError(f'{run_dir.name}: missing MMC_HOLD/trace samples')
        t0 = num(trace[0], 'sim_absolute_sec')
        selected = [r for r in phase
                    if 0.0 <= num(r, 'right_sim_time') - t0 <= WINDOW_SEC + 1e-9]
        # Gate release to first accepted reference, parsed from the task log.
        switch_sec = None
        log = (run_dir / 'task_launch.log').read_text(errors='replace').splitlines()
        gate_ts = ref_ts = None
        for line in log:
            if '[MMC_HOLD_BRIDGE] start gate released' in line and gate_ts is None:
                match = re.search(r'\[(\d+\.\d+)\]', line)
                gate_ts = float(match.group(1)) if match else None
            if '[MMC_FIRST_REFERENCE] result=accepted' in line and ref_ts is None:
                match = re.search(r'\[(\d+\.\d+)\]', line)
                ref_ts = float(match.group(1)) if match else None
        if gate_ts is not None and ref_ts is not None:
            switch_sec = ref_ts - gate_ts
    if len(selected) < 2:
        raise ValueError(f'{run_dir.name}: fewer than 2 samples in the window')
    return t0, selected, switch_sec


def load_rod_quats(run_dir, group, t0):
    """Rod attitude is recorded in hover_diagnostics.csv, not contact_probe.csv."""
    phase = 'HOVER_HOLD' if group == 'A' else 'MMC_HOLD'
    rows = [r for r in csv.DictReader((run_dir / 'hover_diagnostics.csv').open())
            if r['phase'] == phase and
            -0.1 <= num(r, 'right_sim_time') - t0 <= WINDOW_SEC + 0.1]
    rows.sort(key=lambda r: num(r, 'right_sim_time'))
    if len(rows) < 2:
        raise ValueError(f'{run_dir.name}: fewer than 2 rod attitude samples in the window')
    times = [num(r, 'right_sim_time') for r in rows]
    quats = [[num(r, 'rod_q' + ax) for ax in ('x', 'y', 'z', 'w')] for r in rows]

    def nearest(t):
        return min(range(len(times)), key=lambda i: abs(times[i] - t))

    return times, quats, nearest


def summarize(run_dir, group):
    t0, rows, switch_sec = load_window(run_dir, group)
    entry, end = rows[0], rows[-1]
    rel_entry = {side: [num(entry, 'rod_' + ax) - num(entry, side + '_hand_' + ax)
                        for ax in ('x', 'y', 'z')] for side in ('left', 'right')}
    rel_end = {side: [num(end, 'rod_' + ax) - num(end, side + '_hand_' + ax)
                      for ax in ('x', 'y', 'z')] for side in ('left', 'right')}
    diag_times, diag_quats, nearest_diag = load_rod_quats(run_dir, group, t0)
    rod_q0 = diag_quats[nearest_diag(num(entry, 'right_sim_time'))]
    rod_q1 = diag_quats[nearest_diag(num(end, 'right_sim_time'))]
    data = {
        'group': group,
        'run_id': run_dir.name,
        'window_t0_sim_sec': t0,
        'window_rows': len(rows),
        'window_span_sec': num(end, 'right_sim_time') - num(entry, 'right_sim_time'),
        'post_lift_switch_sec': switch_sec,
        'entry': {
            'rod_world_xyz_mm': [num(entry, 'rod_' + ax) * 1000.0 for ax in ('x', 'y', 'z')],
            'left_hand_world_xyz_mm': [num(entry, 'left_hand_' + ax) * 1000.0 for ax in ('x', 'y', 'z')],
            'right_hand_world_xyz_mm': [num(entry, 'right_hand_' + ax) * 1000.0 for ax in ('x', 'y', 'z')],
            'rod_quat_xyzw': rod_q0,
            'left_hand_quat_xyzw': [num(entry, 'left_hand_q' + ax) for ax in ('x', 'y', 'z', 'w')],
            'right_hand_quat_xyzw': [num(entry, 'right_hand_q' + ax) for ax in ('x', 'y', 'z', 'w')],
            'left_grasp_reference_residual_mm': num(entry, 'left_slip_m') * 1000.0,
            'right_grasp_reference_residual_mm': num(entry, 'right_slip_m') * 1000.0,
        },
        'end': {
            'left_grasp_reference_residual_mm': num(end, 'left_slip_m') * 1000.0,
            'right_grasp_reference_residual_mm': num(end, 'right_slip_m') * 1000.0,
        },
        'delta': {
            'rod_xyz_mm': [(num(end, 'rod_' + ax) - num(entry, 'rod_' + ax)) * 1000.0
                           for ax in ('x', 'y', 'z')],
            'left_hand_xyz_mm': [(num(end, 'left_hand_' + ax) - num(entry, 'left_hand_' + ax)) * 1000.0
                                 for ax in ('x', 'y', 'z')],
            'right_hand_xyz_mm': [(num(end, 'right_hand_' + ax) - num(entry, 'right_hand_' + ax)) * 1000.0
                                  for ax in ('x', 'y', 'z')],
            'left_grasp_reference_residual_mm': (num(end, 'left_slip_m') - num(entry, 'left_slip_m')) * 1000.0,
            'right_grasp_reference_residual_mm': (num(end, 'right_slip_m') - num(entry, 'right_slip_m')) * 1000.0,
            'left_rod_hand_xyz_mm': [(rel_end['left'][i] - rel_entry['left'][i]) * 1000.0
                                     for i in range(3)],
            'right_rod_hand_xyz_mm': [(rel_end['right'][i] - rel_entry['right'][i]) * 1000.0
                                      for i in range(3)],
            'left_rod_hand_norm_mm': math.dist(rel_entry['left'], rel_end['left']) * 1000.0,
            'right_rod_hand_norm_mm': math.dist(rel_entry['right'], rel_end['right']) * 1000.0,
            'rod_attitude_deg': quat_angle_deg(rod_q0, rod_q1),
        },
        'window_max': {
            'left_grasp_reference_residual_mm': max(num(r, 'left_slip_m') for r in rows) * 1000.0,
            'right_grasp_reference_residual_mm': max(num(r, 'right_slip_m') for r in rows) * 1000.0,
            'left_angle_deg': max(num(r, 'left_angle_rad') for r in rows) * 180.0 / math.pi,
            'right_angle_deg': max(num(r, 'right_angle_rad') for r in rows) * 180.0 / math.pi,
        },
        'all_four_contact_sample_percent': 100.0 * sum(
            all(num(r, s + '_contact_' + f) == 1 for s in ('left', 'right') for f in ('1', '2'))
            for r in rows) / len(rows),
        'series': [
            {'t': num(r, 'right_sim_time') - num(entry, 'right_sim_time'),
             'rod': (num(r, 'rod_z') - num(entry, 'rod_z')) * 1000.0,
             'left_hand': (num(r, 'left_hand_z') - num(entry, 'left_hand_z')) * 1000.0,
             'right_hand': (num(r, 'right_hand_z') - num(entry, 'right_hand_z')) * 1000.0,
             'left_grasp_reference_residual_mm': num(r, 'left_slip_m') * 1000.0,
             'right_grasp_reference_residual_mm': num(r, 'right_slip_m') * 1000.0,
             'rod_attitude_deg': quat_angle_deg(
                 rod_q0, diag_quats[nearest_diag(num(r, 'right_sim_time'))])}
            for r in rows
        ],
        'completed': None,
    }
    status_path = run_dir / 'prepare_status.json'
    if status_path.exists():
        status = json.loads(status_path.read_text())
        text = (run_dir / 'task_launch.log').read_text(errors='replace')
        data['completed'] = status.get('task_state') == 'DONE' and \
            '[CONTACT_HOVER_RELEASED]' in text
    return data


def group_range(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {'min': min(values), 'median': statistics.median(values), 'max': max(values)}


def plot_vertical(results, out):
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.0), sharex=True)
    for row, group in enumerate(('A', 'B')):
        group_results = [r for r in results if r['group'] == group]
        for col, (label, key, style) in enumerate(SERIES):
            ax = axes[row][col]
            for rec in group_results:
                ax.plot([p['t'] for p in rec['series']], [p[key] for p in rec['series']],
                        color=COLORS[group], linestyle=style, linewidth=1.0, alpha=0.45)
            # Group mean on the common 0.05 s grid.
            grid = sorted({round(p['t'], 3) for rec in group_results for p in rec['series']})
            mean = []
            for t in grid:
                vals = [p[key] for rec in group_results for p in rec['series']
                        if abs(p['t'] - t) < 0.026]
                mean.append(statistics.median(vals) if vals else float('nan'))
            ax.plot(grid, mean, color=COLORS[group], linewidth=2.4)
            ax.axhline(0.0, color='0.6', linewidth=0.8)
            ax.grid(alpha=0.25)
            ax.set_title(f'{"JTC (A)" if group == "A" else "MMC (B)"} - {label}', fontsize=10)
            if col == 0:
                ax.set_ylabel('world-frame ΔZ from window entry (mm)')
            if row == 1:
                ax.set_xlabel('time from window entry (s)')
    # Common scale per series so the A/B magnitudes are directly comparable;
    # per-panel autoscaling would make JTC ripple look as large as the MMC drop.
    for col, (_, key, _) in enumerate(SERIES):
        values = [p[key] for rec in results for p in rec['series']]
        low, high = min(values), max(values)
        pad = 0.08 * max(1e-6, high - low)
        for row in range(2):
            axes[row][col].set_ylim(low - pad, high + pad)
    fig.suptitle('World-frame vertical displacement from 3 s HOLD window entry (thin = run, thick = group median)')
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches='tight')
    plt.close(fig)


def plot_residual_attitude(results, out):
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.0), sharex=True)
    for col, group in enumerate(('A', 'B')):
        group_results = [r for r in results if r['group'] == group]
        ax = axes[0][col]
        for rec in group_results:
            for side, style in (('left', '-'), ('right', '--')):
                ax.plot([p['t'] for p in rec['series']],
                        [p[side + '_grasp_reference_residual_mm'] for p in rec['series']],
                        color=COLORS[group], linestyle=style, linewidth=1.0, alpha=0.45)
        grid = sorted({round(p['t'], 3) for rec in group_results for p in rec['series']})
        for side, style in (('left', '-'), ('right', '--')):
            mean = [statistics.median([p[side + '_grasp_reference_residual_mm'] for rec in group_results
                                       for p in rec['series'] if abs(p['t'] - t) < 0.026] or [float('nan')])
                    for t in grid]
            ax.plot(grid, mean, color=COLORS[group], linestyle=style, linewidth=2.2,
                    label=f'{side} residual')
        ax.set_title(f'{"JTC (A)" if group == "A" else "MMC (B)"} - grasp reference residual')
        ax.set_ylabel('inferred grasp-reference residual (mm)')
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)

        ax = axes[1][col]
        for rec in group_results:
            ax.plot([p['t'] for p in rec['series']],
                    [p['rod_attitude_deg'] for p in rec['series']],
                    color=COLORS[group], linewidth=1.0, alpha=0.45)
        mean = [statistics.median([p['rod_attitude_deg'] for rec in group_results
                                   for p in rec['series'] if abs(p['t'] - t) < 0.026] or [float('nan')])
                for t in grid]
        ax.plot(grid, mean, color=COLORS[group], linewidth=2.4)
        ax.set_title(f'{"JTC (A)" if group == "A" else "MMC (B)"} - rod attitude change')
        ax.set_ylabel('angle from entry (deg)')
        ax.set_xlabel('time from window entry (s)')
        ax.grid(alpha=0.25)
    fig.suptitle('Grasp reference residual and rod attitude over the 3 s window')
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches='tight')
    plt.close(fig)


def plot_summary(results, out):
    runs = [r for r in results]
    labels = [r['sample'] for r in runs]
    colors = [COLORS[r['group']] for r in runs]
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.0))
    ax = axes[0]
    drops = [-r['delta']['rod_xyz_mm'][2] for r in runs]
    ax.bar(range(len(runs)), drops, color=colors)
    ax.axhline(0.0, color='0.5', linewidth=0.8)
    ax.set_xticks(range(len(runs)))
    ax.set_xticklabels(labels, rotation=30, ha='right')
    ax.set_ylabel('rod world-frame ΔZ, entry to end (mm; + = down)')
    ax.set_title('Rod world displacement over the 3 s window')
    ax.grid(axis='y', alpha=0.25)

    ax = axes[1]
    width = 0.36
    idx = range(len(runs))
    left = [r['delta']['left_rod_hand_norm_mm'] for r in runs]
    right = [r['delta']['right_rod_hand_norm_mm'] for r in runs]
    ax.bar([i - width / 2 for i in idx], left, width, label='left', color='#2ca02c')
    ax.bar([i + width / 2 for i in idx], right, width, label='right', color='#9467bd')
    ax.set_xticks(list(idx))
    ax.set_xticklabels(labels, rotation=30, ha='right')
    ax.set_ylabel('|change in rod-minus-hand vector| (mm)')
    ax.set_title('Change in rod-hand relative pose position')
    ax.legend(fontsize=8)
    ax.grid(axis='y', alpha=0.25)
    for i, rec in enumerate(runs):
        if not rec['completed']:
            ax.text(i, 0, 'FAIL', ha='center', va='bottom', fontsize=8, color='red')
    fig.suptitle('Per-run 3 s window summary (A = JTC, B = MMC; relative vector is not direct contact slip)')
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root', type=Path, default=Path('experiment_results/a1_runs'))
    parser.add_argument('--out-dir', type=Path, default=Path('experiment_results/reports/hover_ab_20261020'))
    parser.add_argument('--fig-dir', type=Path, default=Path('docs/figures/历史归档/早期实验'))
    parser.add_argument('--runs', default=None,
                        help='Comma-separated GROUP:run_id entries; defaults to the six reported runs')
    args = parser.parse_args()
    runs = DEFAULT_RUNS
    if args.runs:
        runs = [tuple(item.split(':', 1)) for item in args.runs.split(',')]
    results = []
    for group, run_id in runs:
        run_dir = args.runs_root / run_id
        if not (run_dir / 'contact_probe.csv').exists():
            raise SystemExit(f'missing run: {run_dir}')
        results.append(summarize(run_dir, group))
    counts = {'A': 0, 'B': 0}
    for rec in results:
        counts[rec['group']] += 1
        rec['sample'] = rec['group'] + str(counts[rec['group']])
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.fig_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        'window_sec': WINDOW_SEC,
        'window_definition': {
            'A': 'first HOVER_HOLD sample after lift; 3.0 s from that sample',
            'B': 'first mmc_hold_trace sample after the MMC first reference; 3.0 s from that absolute sim time',
        },
        'note': ('Asynchronous physical samples. Rod and hand deltas are world-frame displacement from the '
                 'window-entry sample. Legacy task *_slip_m fields are inferred grasp-reference rod-centre '
                 'residuals, not local pad tangential contact slip. Rod-minus-hand vector change is a '
                 'relative pose-position change, not direct contact slip.'),
        'runs': results,
        'group_ranges': {},
    }
    for group in ('A', 'B'):
        rs = [r for r in results if r['group'] == group]
        summary['group_ranges'][group] = {
            'rod_world_delta_z_mm': group_range([r['delta']['rod_xyz_mm'][2] for r in rs]),
            'left_grasp_reference_residual_increment_mm': group_range([r['delta']['left_grasp_reference_residual_mm'] for r in rs]),
            'right_grasp_reference_residual_increment_mm': group_range([r['delta']['right_grasp_reference_residual_mm'] for r in rs]),
            'left_rod_hand_norm_mm': group_range([r['delta']['left_rod_hand_norm_mm'] for r in rs]),
            'right_rod_hand_norm_mm': group_range([r['delta']['right_rod_hand_norm_mm'] for r in rs]),
            'rod_attitude_deg': group_range([r['delta']['rod_attitude_deg'] for r in rs]),
            'completed': sum(1 for r in rs if r['completed']),
        }
    (args.out_dir / 'hover_ab_summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n')

    fields = ['sample', 'group', 'run_id', 'window_span_sec', 'post_lift_switch_sec',
              'entry_left_grasp_reference_residual_mm', 'entry_right_grasp_reference_residual_mm',
              'end_left_grasp_reference_residual_mm', 'end_right_grasp_reference_residual_mm',
              'left_grasp_reference_residual_increment_mm', 'right_grasp_reference_residual_increment_mm',
              'rod_world_dx_mm', 'rod_world_dy_mm', 'rod_world_dz_mm',
              'left_hand_world_dz_mm', 'right_hand_world_dz_mm',
              'rod_minus_left_hand_delta_x_mm', 'rod_minus_left_hand_delta_y_mm', 'rod_minus_left_hand_delta_z_mm',
              'rod_minus_right_hand_delta_x_mm', 'rod_minus_right_hand_delta_y_mm', 'rod_minus_right_hand_delta_z_mm',
              'rod_minus_left_hand_delta_norm_mm', 'rod_minus_right_hand_delta_norm_mm', 'rod_attitude_deg',
              'window_max_left_grasp_reference_residual_mm', 'window_max_right_grasp_reference_residual_mm',
              'window_max_left_angle_deg', 'window_max_right_angle_deg',
              'all_four_contact_sample_percent', 'completed']
    with (args.out_dir / 'hover_ab_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for r in results:
            d = r['delta']
            writer.writerow({
                'sample': r['sample'], 'group': r['group'], 'run_id': r['run_id'],
                'window_span_sec': r['window_span_sec'],
                'post_lift_switch_sec': r['post_lift_switch_sec'],
                'entry_left_grasp_reference_residual_mm': r['entry']['left_grasp_reference_residual_mm'],
                'entry_right_grasp_reference_residual_mm': r['entry']['right_grasp_reference_residual_mm'],
                'end_left_grasp_reference_residual_mm': r['end']['left_grasp_reference_residual_mm'],
                'end_right_grasp_reference_residual_mm': r['end']['right_grasp_reference_residual_mm'],
                'left_grasp_reference_residual_increment_mm': d['left_grasp_reference_residual_mm'],
                'right_grasp_reference_residual_increment_mm': d['right_grasp_reference_residual_mm'],
                'rod_world_dx_mm': d['rod_xyz_mm'][0], 'rod_world_dy_mm': d['rod_xyz_mm'][1],
                'rod_world_dz_mm': d['rod_xyz_mm'][2],
                'left_hand_world_dz_mm': d['left_hand_xyz_mm'][2],
                'right_hand_world_dz_mm': d['right_hand_xyz_mm'][2],
                'rod_minus_left_hand_delta_x_mm': d['left_rod_hand_xyz_mm'][0],
                'rod_minus_left_hand_delta_y_mm': d['left_rod_hand_xyz_mm'][1],
                'rod_minus_left_hand_delta_z_mm': d['left_rod_hand_xyz_mm'][2],
                'rod_minus_right_hand_delta_x_mm': d['right_rod_hand_xyz_mm'][0],
                'rod_minus_right_hand_delta_y_mm': d['right_rod_hand_xyz_mm'][1],
                'rod_minus_right_hand_delta_z_mm': d['right_rod_hand_xyz_mm'][2],
                'rod_minus_left_hand_delta_norm_mm': d['left_rod_hand_norm_mm'],
                'rod_minus_right_hand_delta_norm_mm': d['right_rod_hand_norm_mm'],
                'rod_attitude_deg': d['rod_attitude_deg'],
                'window_max_left_grasp_reference_residual_mm': r['window_max']['left_grasp_reference_residual_mm'],
                'window_max_right_grasp_reference_residual_mm': r['window_max']['right_grasp_reference_residual_mm'],
                'window_max_left_angle_deg': r['window_max']['left_angle_deg'],
                'window_max_right_angle_deg': r['window_max']['right_angle_deg'],
                'all_four_contact_sample_percent': r['all_four_contact_sample_percent'],
                'completed': r['completed'],
            })

    plot_vertical(results, args.fig_dir / '20261020_悬空保持高度变化.png')
    plot_residual_attitude(results, args.fig_dir / '20261020_悬空保持残差与姿态.png')
    plot_summary(results, args.fig_dir / '20261020_悬空保持逐次汇总.png')

    for r in results:
        d = r['delta']
        print(f'{r["group"]} {r["run_id"]}: span={r["window_span_sec"]:.3f}s '
              f'switch={r["post_lift_switch_sec"] if r["post_lift_switch_sec"] is None else round(r["post_lift_switch_sec"], 3)} '
              f'rod_world_dz={d["rod_xyz_mm"][2]:+.3f}mm '
              f'grasp_residual_inc L/R={d["left_grasp_reference_residual_mm"]:+.3f}/{d["right_grasp_reference_residual_mm"]:+.3f}mm '
              f'rel_norm L/R={d["left_rod_hand_norm_mm"]:.3f}/{d["right_rod_hand_norm_mm"]:.3f}mm '
              f'att={d["rod_attitude_deg"]:.3f}deg all_four_contact_samples={r["all_four_contact_sample_percent"]:.1f}% '
              f'completed={r["completed"]}')
    print('summary:', args.out_dir / 'hover_ab_summary.csv')
    print('figures:', args.fig_dir)


if __name__ == '__main__':
    main()
