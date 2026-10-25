#!/usr/bin/env python3
"""Analyze every scheduled attempt in the final unified ABCD experiment."""
import csv
import json
import re
import statistics
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from analyze_abcd_s0_today import parse_run, read_csv
from scientific_style import GROUP_COLORS, NATURE, configure_scientific_style

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'experiment_results/reports/final_abcd_20261025'
FIG=ROOT/'docs/figures/最终实验_20261025'
ORDER='A1 C1 B1 D1 D2 B2 C2 A2 B3 A3 D3 C3'.split()
METRICS=['release_xy_error_mm','release_yaw_error_deg','rod_bottom_gap_mm',
         'transport_plan_s','transport_action_wall_s','transport_reference_playback_s',
         'contact_start_to_terminal_s','max_grasp_reference_residual_mm',
         'max_grasp_reference_angle_deg','time_profile_solve_sec',
         'continuous_jerk_rms_change_pct_vs_same_run','continuous_jerk_peak_change_pct',
         'release_wait_actual_s']


def write_csv(path,rows):
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)


def stats(rows,key):
    v=[float(r[key]) for r in rows if r.get(key) not in ('',None)]
    if not v:return None
    return dict(n=len(v),mean=statistics.mean(v),median=statistics.median(v),
                sd=statistics.stdev(v) if len(v)>1 else 0,min=min(v),max=max(v))


def summarize():
    selection=OUT/'valid_attempts.csv'
    exits={r['sample']:r for r in read_csv(selection if selection.exists() else OUT/'attempt_exits.csv')}
    rows=[]
    for tag in ORDER:
        if tag not in exits:raise RuntimeError(f'Scheduled attempt unfinished: {tag}')
        if exits[tag]['wrapper_exit']=='pending':raise RuntimeError(f'Attempt still running: {tag}')
        run_id=exits[tag]['run_id']
        run=ROOT/'experiment_results/a1_runs'/run_id
        parsed=parse_run(tag[0],run_id)
        r={k:v for k,v in parsed.items() if not k.startswith('_')}
        r.update(sample=tag,batch='unified_final_20261025',wrapper_exit=exits[tag]['wrapper_exit'])
        log=(run/'task_launch.log').read_text(errors='replace') if (run/'task_launch.log').exists() else ''
        wait=re.search(r'\[ABCD_RELEASE_MEASUREMENT\] physics_wait_sec=([0-9.]+)',log)
        r.update(release_wait_protocol='common_2s_simulation',release_wait_target_s=2.0,
                 release_wait_actual_s=wait.group(1) if wait else '',
                 release_wait_source='ABCD_RELEASE_MEASUREMENT')
        profile=read_csv(run/'abcd_transport_time_profile.csv')
        candidate=next((v for v in profile if v.get('plan')=='candidate'),{})
        r['candidate_accepted']=candidate.get('accepted','')
        r['candidate_selected']=candidate.get('selected','')
        r['continuous_jerk_peak_change_pct']=(
            100*(float(r['candidate_continuous_jerk_max'])/float(r['baseline_continuous_jerk_max'])-1)
            if r['baseline_continuous_jerk_max'] not in ('',None) else '')
        trace=parsed['_trace']
        r['constraint_samples']=len(trace)
        r['weld_and_rod_constraint_off']=(int(all(
            float(v[k])==0 for v in trace for k in ['weld_left','weld_right','rod_constraint'])) if trace else '')
        if not log:r['failure_reason']='task_not_started; see wrapper and simulator logs'
        rows.append(r)
    write_csv(OUT/'runs.csv',rows)
    summaries={}
    flat=[]
    for group in 'ABCD':
        scheduled=[r for r in rows if r['group']==group]
        done=[r for r in scheduled if r['state']=='DONE']
        s=dict(attempts=len(scheduled),completed=len(done),failed=len(scheduled)-len(done),
               metrics={key:stats(done,key) for key in METRICS},
               failures=[dict(sample=r['sample'],reason=r['failure_reason']) for r in scheduled if r['state']!='DONE'])
        summaries[group]=s
        f=dict(group=group,attempts=s['attempts'],completed=s['completed'],failed=s['failed'])
        for key,metric in s['metrics'].items():
            if metric:f.update({f'{key}_{k}':v for k,v in metric.items()})
        flat.append(f)
    write_csv(OUT/'group_summary.csv',flat)
    (OUT/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
    return rows,summaries


def save(fig,name):
    fig.savefig(FIG/f'{name}.png',dpi=300)
    fig.savefig(FIG/f'{name}.svg')
    plt.close(fig)
    print(FIG/f'{name}.png')


def figures(rows):
    family=configure_scientific_style()
    plt.rcParams.update({'font.size':10,'axes.labelsize':10,'legend.fontsize':8,
                         'axes.unicode_minus':False,'svg.fonttype':'none',
                         'font.family':['Times New Roman',family]})
    FIG.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(1,2,figsize=(8.5,3.5),layout='constrained')
    for i,g in enumerate('ABCD'):
        done=[r for r in rows if r['group']==g and r['state']=='DONE']
        axes[0].bar(i,len(done),width=.56,color=GROUP_COLORS[g])
        axes[0].text(i,len(done)+.1,f'{len(done)}/3',ha='center')
        y=[float(r['release_xy_error_mm']) for r in done]
        axes[1].scatter(i+np.linspace(-.07,.07,len(y)),y,color=GROUP_COLORS[g],s=25)
        if y:axes[1].hlines(statistics.mean(y),i-.18,i+.18,color=GROUP_COLORS[g],lw=1.5)
    axes[0].set(ylabel='有效搬运完成次数',ylim=(0,3.6),yticks=range(4))
    axes[1].set(ylabel='释放后平面位置误差（mm）',ylim=(0,None))
    for ax in axes:
        ax.set_xticks(range(4),list('ABCD'));ax.set_xlabel('实验组别');ax.grid(axis='x',visible=False)
    save(fig,'01_四组完整搬运与放置误差')

    fig,axes=plt.subplots(2,2,figsize=(8.5,6.0),layout='constrained',sharex=True,sharey=True)
    for ax,g in zip(axes.flat,'ABCD'):
        for r,ls in zip([r for r in rows if r['group']==g],['-','--',':']):
            trace=read_csv(ROOT/'experiment_results/a1_runs'/r['run_id']/'contact_probe.csv')
            if not trace:continue
            t=np.array([(float(v['left_sim_time'])+float(v['right_sim_time']))/2 for v in trace]);t-=t[0]
            ax.plot(t,[float(v['rod_z'])*1000 for v in trace],color=GROUP_COLORS[g],ls=ls,label=r['sample'])
        ax.set(xlabel='接触夹稳后的时间（s）',ylabel='铝条Z坐标（mm）');ax.legend(frameon=False)
    save(fig,'02_四组铝条实际高度曲线')

    opt=[r for r in rows if r['group'] in 'BD' and r['baseline_continuous_jerk_rms'] not in ('',None)]
    opt.sort(key=lambda r:(r['group'],r['sample']))
    fig,axes=plt.subplots(1,2,figsize=(9,3.5),layout='constrained')
    for ax,suffix,label in [(axes[0],'rms','连续参考jerk RMS（rad/s³）'),(axes[1],'max','连续参考jerk峰值（rad/s³）')]:
        x=np.arange(len(opt));base=[float(r[f'baseline_continuous_jerk_{suffix}']) for r in opt]
        selected=[float(r[f'candidate_continuous_jerk_{suffix}']) if r['candidate_selected']=='1' else b for r,b in zip(opt,base)]
        ax.bar(x-.17,base,width=.32,color=NATURE['blue'],label='基础参考')
        ax.bar(x+.17,selected,width=.32,color=NATURE['green'],label='采用参考')
        ax.set_xticks(x,[r['sample'] for r in opt]);ax.set(xlabel='优化组运行编号',ylabel=label,ylim=(0,None))
        ax.legend(frameon=False,loc='lower center',bbox_to_anchor=(.5,1.01),ncol=2)
        ax.grid(axis='x',visible=False)
    save(fig,'03_运输参考平滑度对照')

    # Replot existing independent experiments from their actual source data.
    import plot_report_figures_20261024 as historical
    plt.rcParams['font.family']=['Times New Roman',family]
    historical.OUT=FIG
    historical.plot_time_tradeoff()
    historical.plot_feedforward()
    historical.plot_directional()
    (OUT/'historical_figure_sources.json').write_text(json.dumps(historical.EXPORTS,ensure_ascii=False,indent=2)+'\n')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,summary=summarize()
    figures(rows)
    text=['# 统一ABCD实验统计结果','',
          '| 样本 | 完整结果 | 运输计划/s | 连续参考RMS变化 | 连续参考峰值变化 | 释放平面误差/mm | 偏航误差/° |',
          '|---|---|---:|---:|---:|---:|---:|']
    def fmt(v,n=2):return f'{float(v):.{n}f}' if v not in ('',None) else '—'
    for r in rows:
        text.append('| '+' | '.join([r['sample'],r['state'],fmt(r['transport_plan_s'],4),
          fmt(r['continuous_jerk_rms_change_pct_vs_same_run'])+('%' if r['continuous_jerk_rms_change_pct_vs_same_run']!='' else ''),
          fmt(r['continuous_jerk_peak_change_pct'])+('%' if r['continuous_jerk_peak_change_pct']!='' else ''),
          fmt(r['release_xy_error_mm'],4),fmt(r['release_yaw_error_deg'],4)])+' |')
    text+=['','| 组别 | 完整次数 | 释放平面误差/mm（均值±SD） | 偏航误差/°（均值±SD） |',
           '|---|---|---:|---:|']
    for g,s in summary.items():
        def ms(key):
            st=s['metrics'][key]
            return f'{st["mean"]:.3f}±{st["sd"]:.3f}' if st else '—'
        text.append(f'| {g} | {s["completed"]}/{s["attempts"]} | {ms("release_xy_error_mm")} | {ms("release_yaw_error_deg")} |')
    text+=['','各组均在释放后约2秒仿真时刻测量。当前有效搬运样本清单见valid_attempts.csv；初始化中止另行保留，不改写原始运行状态。',
           '参考jerk按每次自身的同路径、同时长基础参考计算。C/D采用阻抗与物体载荷前馈组合，不单独归因于阻抗。',
           'JTC动作确认耗时与MMC参考播放时间分别保存，不混合为同一执行时间。图中散点为逐次结果，短横线为均值，n=3不作总体可靠率或显著性推断。',
           'C1首次启动因/get_sim_info服务响应超时，原始目录保留在startup_failures.csv。D3原运行在闭爪后、承载控制激活前因启动预算不一致中止，记入excluded_initializations.csv；修复后重新执行的实际结果另存，不将原记录改成DONE。',
           '有效搬运评价从承载控制准备完成开始；开始承载运动后的掉落、碰撞或中止仍计入搬运失败。']
    (OUT/'results.md').write_text('\n'.join(text)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
