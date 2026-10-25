#!/usr/bin/env python3
"""Summarize the three predeclared B attempts; preserve historical ABCD data."""
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_abcd_s0_today import parse_run, read_csv

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'experiment_results/reports/b_formal_20261025'
IDS = [f'b_formal_20261025_{i:02d}' for i in range(1, 4)]


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def stats(rows, field):
    values = [float(row[field]) for row in rows if row.get(field) not in ('', None)]
    if not values:
        return None
    return {'n':len(values), 'mean':statistics.mean(values),
            'median':statistics.median(values), 'min':min(values), 'max':max(values)}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    current = []
    for index, run_id in enumerate(IDS, 1):
        run = ROOT / 'experiment_results/a1_runs' / run_id
        if not (run / 'task_launch.log').exists():
            raise RuntimeError(f'No task record for predeclared attempt {run_id}')
        row = {k:v for k,v in parse_run('B', run_id).items() if not k.startswith('_')}
        row['batch'] = 'B_formal_20261025'
        row['sample'] = f'B{index}'
        row['optimizer_version'] = 'continuous_jerk_peak_fidelity_checked'
        row['connection_policy'] = 'JTC_12mrad_stationary_zero_velocity'
        metrics = read_csv(run / 'abcd_transport_time_profile.csv')
        candidate = next((r for r in metrics if r['plan']=='candidate'), {})
        row['candidate_selected'] = candidate.get('selected', '')
        row['candidate_accepted'] = candidate.get('accepted', '')
        row['continuous_jerk_peak_change_pct'] = (
            100*(float(row['candidate_continuous_jerk_max']) /
                 float(row['baseline_continuous_jerk_max']) - 1)
            if row['baseline_continuous_jerk_max'] else '')
        trace = read_csv(run / 'contact_probe.csv')
        row['constraint_records'] = len(trace)
        row['weld_and_rod_constraint_off'] = int(bool(trace) and all(
            float(r[k]) == 0 for r in trace for k in ('weld_left','weld_right','rod_constraint')))
        current.append(row)
    completed = [r for r in current if r['state']=='DONE']
    summary = {'group':'B', 'batch':'20261025', 'attempts':len(current),
               'completed':len(completed), 'failed':len(current)-len(completed),
               'run_ids':IDS, 'old_B_replaced_in_presentation_only':True,
               'historical_data_modified':False,
               'metrics': {key:stats(completed, key) for key in (
                   'release_xy_error_mm','release_yaw_error_deg',
                   'candidate_transport_duration_s','transport_action_wall_s',
                   'continuous_jerk_rms_change_pct_vs_same_run',
                   'continuous_jerk_peak_change_pct','time_profile_solve_sec',
                   'max_grasp_reference_residual_mm','max_grasp_reference_angle_deg')},
               'notes':['Jerk is a planned reference metric, not measured robot jerk.',
                        'B is the new batch; A/C/D retain their historical batch.',
                        'JTC action wall time includes the checked 0.5 s start connection.']}
    write_csv(OUT/'b_runs.csv', current)
    group_row = {'group':'B', 'batch':'20261025', 'attempts':len(current),
                 'completed':len(completed), 'failed':len(current)-len(completed)}
    for metric, result in summary['metrics'].items():
        if result:
            group_row.update({f'{metric}_{key}':value for key,value in result.items()})
    write_csv(OUT/'b_group_summary.csv', [group_row])
    historical = read_csv(ROOT/'experiment_results/reports/final_tradeoff_20261022/abcd_s0_formal_runs.csv')
    for row in historical:
        row['batch'] = 'ABCD_formal_20261022'
    display = [r for r in historical if r['group']!='B'] + current
    write_csv(OUT/'abcd_display_runs.csv', display)
    (OUT/'b_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
