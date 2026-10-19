#!/usr/bin/env python3
"""Read-only hover diagnostics. No ROS, no simulated-state writes, no force clipping.

Task CSV values are asynchronous, subsampled, and force-filtered by hardware.
Occupancy below is sample-count based, NOT physical-time contact duty.
"""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from statistics import median


def number(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else None
    except (KeyError, TypeError, ValueError):
        return None


def stats(values):
    vs = sorted(v for v in values if v is not None)
    if not vs:
        return None
    return dict(n=len(vs), min=vs[0], median=median(vs),
                p95=vs[max(0, math.ceil(0.95 * len(vs)) - 1)], max=vs[-1])


def analyze(run):
    path = run / 'contact_probe.csv'
    rows = list(csv.DictReader(path.open())) if path.exists() else []
    diagnostic = run / 'hover_diagnostics.csv'
    ds = list(csv.DictReader(diagnostic.open())) if diagnostic.exists() else []
    task = (run / 'task_launch.log').read_text(errors='replace') if (run / 'task_launch.log').exists() else ''
    status_path = run / 'prepare_status.json'
    status = json.loads(status_path.read_text()) if status_path.exists() else {}
    result = dict(run_id=run.name, task_state=status.get('task_state'),
                  result_line=status.get('contact_probe_result_line'),
                  hover_hold_reported='[CONTACT_HOVER_RESULT]' in task,
                  release_reported='[CONTACT_HOVER_RELEASED]' in task,
                  phases={}, limitations=[
                      'Readout statistics, not a pass/fail substitute for task result.',
                      'Contact occupancy is sample-count based; missing/duplicate snapshots are reported.',
                      'Rod truth, joint feedback and left/right force feedback are not one synchronized snapshot.',
                      'Task forces exclude individual contacts with nonfinite or >150 N force components.',
                      'Large finite aggregate forces are retained; they do not identify a unique physical cause.',
                      'No direct ground reaction force in task CSV; clearance is geometric, not a ground force measurement.',
                      'Existing slip is inferred rod-centre pose residual, not measured local pad tangential displacement.',
                  ])
    for phase in dict.fromkeys(r['phase'] for r in rows):
        rs = [r for r in rows if r['phase'] == phase]
        ts = [number(r, 'left_sim_time') for r in rs]
        times = [t for t in ts if t is not None]
        data = dict(rows=len(rs), span_sim_s=max(times)-min(times) if times else None)
        pairs = [(r.get('left_seq'), r.get('right_seq')) for r in rs]
        data['duplicate_feedback_pairs'] = len(pairs) - len(set(pairs))
        data['contact_count_changes_between_samples'] = {side: sum(
            a.get(side+'_pad_contact_count') != b.get(side+'_pad_contact_count')
            for a, b in zip(rs, rs[1:])) for side in ('left', 'right')}
        data['feedback_skew_sec'] = stats(
            abs(number(r, 'left_sim_time')-number(r, 'right_sim_time'))
            for r in rs if number(r, 'left_sim_time') is not None and number(r, 'right_sim_time') is not None)
        for side in ('left', 'right'):
            data[side + '_contact_point_counts'] = dict(Counter(r.get(side+'_pad_contact_count', '?') for r in rs))
            for finger in ('1', '2'):
                field = side + '_contact_' + finger
                valid = [number(r, field) for r in rs if number(r, field) is not None]
                data[field+'_sample_fraction'] = sum(v > 0 for v in valid)/len(valid) if valid else None
        keys = ['rod_z', 'left_hand_z', 'right_hand_z', 'left_slip_m', 'right_slip_m',
                'left_angle_rad', 'right_angle_rad']
        keys += [s+'_'+f for s in ('left', 'right') for f in (
            'normal1_n', 'normal2_n', 'tangent1_n', 'tangent2_n',
            'rod_force_x_n', 'rod_force_y_n', 'rod_force_z_n')]
        data['readout_statistics'] = {key: stats(number(r, key) for r in rs) for key in keys}
        data['large_force_readout_rows_over_100N'] = sum(any(
            number(r, key) is not None and abs(number(r, key)) > 100
            for key in keys if key.endswith('_n')) for r in rs)
        data['first_to_last_delta_m'] = {key: number(rs[-1], key)-number(rs[0], key)
            for key in ('rod_x', 'rod_y', 'rod_z', 'left_hand_x', 'left_hand_y', 'left_hand_z',
                        'right_hand_x', 'right_hand_y', 'right_hand_z')
            if number(rs[-1], key) is not None and number(rs[0], key) is not None}
        valid_contact_rows = [r for r in rs if all(number(r, s+'_contact_'+f) is not None
            for s in ('left', 'right') for f in ('1', '2'))]
        data['both_arms_geometric_contact_sample_fraction'] = sum(
            all(number(r, s+'_contact_'+f) == 1 for s in ('left', 'right') for f in ('1', '2'))
            for r in valid_contact_rows)/len(valid_contact_rows) if valid_contact_rows else None
        phase_ds = [r for r in ds if r['phase'] == phase]
        if phase_ds:
            data['additional_diagnostics'] = {key: stats(number(r, key) for r in phase_ds)
                for key in phase_ds[0] if key not in ('phase', 'wall_steady_sec')}
            data['diagnostic_first_to_last_delta_m'] = {
                key: number(phase_ds[-1], key)-number(phase_ds[0], key)
                for key in phase_ds[0] if key.endswith('_m') and
                number(phase_ds[-1], key) is not None and number(phase_ds[0], key) is not None}
        result['phases'][phase] = data
    result['constraints_active_samples'] = sum(any(number(r, k) not in (0, None)
        for k in ('weld_left', 'weld_right', 'rod_constraint')) for r in rows)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir', type=Path, required=True)
    ap.add_argument('--output', type=Path, help='Optional NEW JSON file; refuses overwriting')
    args = ap.parse_args()
    if not args.run_dir.is_dir():
        ap.error('run directory does not exist')
    result = analyze(args.run_dir)
    output = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:
            stream.write(output + '\n')
    print('run:', result['run_id'], 'task_state:', result['task_state'])
    print('result:', result['result_line'])
    print('hold_reported:', result['hover_hold_reported'], 'release_reported:', result['release_reported'])
    if not result['phases']:
        print('No contact samples; inspect pregrasp/P2B result, not a slip experiment.')
    for phase, data in result['phases'].items():
        print(phase, 'rows=', data['rows'], 'span_s=', data['span_sim_s'],
              'both_arms_contact_fraction=', data['both_arms_geometric_contact_sample_fraction'])
        print('  first_to_last_delta_m:', data['first_to_last_delta_m'])
        print('  slip_m:', {s: data['readout_statistics'][s+'_slip_m'] for s in ('left', 'right')})
        print('  feedback_skew_sec:', data['feedback_skew_sec'])
    for note in result['limitations']:
        print('NOTE:', note)
    if args.output:
        print('Full statistics:', args.output)


if __name__ == '__main__':
    main()
