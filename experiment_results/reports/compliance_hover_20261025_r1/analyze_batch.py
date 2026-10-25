#!/usr/bin/env python3
"""Frozen descriptive analysis for compliance_hover_20261025_r1."""
import csv
import json
import math
import re
import statistics
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import t as student_t

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUNS = ROOT / "experiment_results/a1_runs"
FIG = OUT / "figures"
FIG.mkdir(exist_ok=True)
COLORS = {"F0": "#2878B5", "F1": "#D55E00", "K400-N": "#2878B5",
          "K400-F": "#4C9F70", "K200-N": "#CC79A7", "K200-F": "#E69F00"}
DIR_CONDS = ("K400-N", "K400-F", "K200-N", "K200-F")


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def fnum(row, key):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else None
    except (KeyError, TypeError, ValueError):
        return None


def read_start(log):
    m = re.search(r"\[MMC_HOVER_WINDOW\] first_reference_sim_time=([0-9.eE+-]+)", log)
    return float(m.group(1)) if m else None


def mean_ci(values, alpha=0.05):
    x = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if len(x) == 0:
        return {"n": 0, "mean": None, "sd": None, "min": None, "max": None,
                "ci_low": None, "ci_high": None}
    m = float(np.mean(x)); sd = float(np.std(x, ddof=1)) if len(x) > 1 else None
    half = float(student_t.ppf(1 - alpha / 2, len(x) - 1) * sd / math.sqrt(len(x))) if len(x) > 1 else None
    return {"n": len(x), "mean": m, "sd": sd, "min": float(np.min(x)), "max": float(np.max(x)),
            "ci_low": m-half if half is not None else None,
            "ci_high": m+half if half is not None else None}


def welch(a, b):
    """Independent-sample estimate a-b with two-sided Welch 95% t interval."""
    a = np.asarray([x for x in a if x is not None and math.isfinite(x)], dtype=float)
    b = np.asarray([x for x in b if x is not None and math.isfinite(x)], dtype=float)
    if len(a) < 2 or len(b) < 2:
        return {"n_a": len(a), "n_b": len(b), "estimate": None, "se": None,
                "df": None, "ci_low": None, "ci_high": None}
    va, vb = np.var(a, ddof=1), np.var(b, ddof=1)
    aa, bb = va/len(a), vb/len(b)
    se2 = aa + bb
    df = se2**2 / (aa**2/(len(a)-1) + bb**2/(len(b)-1)) if se2 else len(a)+len(b)-2
    estimate = float(np.mean(a)-np.mean(b)); se = math.sqrt(se2)
    half = float(student_t.ppf(.975, df) * se)
    return {"n_a": len(a), "n_b": len(b), "estimate": estimate, "se": se,
            "df": float(df), "ci_low": estimate-half, "ci_high": estimate+half}


def interp_at(times, values, target):
    if not times or target < times[0] or target > times[-1]:
        return None
    return float(np.interp(target, times, values))


def quat_angle(q0, q1):
    if any(x is None for x in q0 + q1):
        return None
    a=np.asarray(q0,dtype=float); b=np.asarray(q1,dtype=float)
    na=np.linalg.norm(a); nb=np.linalg.norm(b)
    if na == 0 or nb == 0: return None
    dot=min(1.0,max(-1.0,abs(float(np.dot(a/na,b/nb)))))
    return math.degrees(2*math.acos(dot))


def event_times(path):
    if not path.exists(): return {}
    return {r["event"]: float(r["sim_time"]) for r in read_csv(path) if r.get("sim_time")}


def load_run(attempt):
    d=RUNS/attempt["run_id"]
    log_path=d/"task_launch.log"
    log=log_path.read_text(errors="replace") if log_path.exists() else ""
    state=""
    try: state=json.loads((d/"prepare_status.json").read_text()).get("task_state","")
    except Exception: pass
    row={k:attempt.get(k,"").strip() for k in ("study","block","condition","sample","run_id","wrapper_exit","formal","failure_stage","run_dir")}
    row["task_state"]=state
    row["completed"]=int(state=="DONE" and "[CONTACT_HOVER_RELEASED]" in log)
    row["first_reference_sim_time"]=read_start(log)
    row["requested_feedforward_enabled"]=int(attempt["condition"] in ("F1","K400-N","K400-F","K200-N","K200-F"))
    row["requested_force"]=int(attempt["condition"].endswith("-F"))
    row["requested_kx_n_m"]=200 if attempt["condition"].startswith("K200") else 400
    variant=re.search(r"\[MMC_CONTROL_VARIANT\] (fixed|directional).*?Kdiag=\(([^)]+)\).*?(?:damping_ratio|zeta)=([0-9.eE+-]+) nullspace=([0-9.eE+-]+)",log)
    row["actual_impedance_variant"]=variant.group(1) if variant else None
    if variant:
        gains=[float(x) for x in variant.group(2).split(",")]
        for key,value in zip(("actual_kx_n_m","actual_ky_n_m","actual_kz_n_m","actual_krx_nm_rad","actual_kry_nm_rad","actual_krz_nm_rad"),gains):row[key]=value
        row["actual_damping_ratio"]=float(variant.group(3));row["actual_nullspace_stiffness"]=float(variant.group(4))
        row["actual_impedance_frame"]="mj_left_link0 / mj_right_link0 (identity orientation, world-aligned)"
    row["measured_window_span_s"]=None
    row["valid_observation_window"]=0
    row["valid_payload_feedforward"]=0
    row["valid_force_pulse"]=int(attempt["study"]=="feedforward")
    row["physical_pulse_start_sim_time"]=None
    row["physical_pulse_end_sim_time"]=None
    if row["first_reference_sim_time"] is None or not (d/"contact_probe.csv").exists():
        return row, None
    t0=row["first_reference_sim_time"]
    required_span=3.5 if attempt["study"]=="feedforward" else 5.0
    probe=[r for r in read_csv(d/"contact_probe.csv") if r.get("phase")=="MMC_HOLD"]
    series=[]
    for r in probe:
        ls=fnum(r,"left_sim_time"); rs=fnum(r,"right_sim_time")
        if ls is None or rs is None: continue
        tm=(ls+rs)/2-t0
        if tm<0 or tm>required_span: continue
        vals={k:fnum(r,k) for k in ("rod_x","rod_y","rod_z","left_hand_x","left_hand_y","left_hand_z",
                                      "right_hand_x","right_hand_y","right_hand_z")}
        vals.update({"t":tm,"left_rel_x":(vals["rod_x"]-vals["left_hand_x"]) if vals["rod_x"] is not None and vals["left_hand_x"] is not None else None,
                     "right_rel_x":(vals["rod_x"]-vals["right_hand_x"]) if vals["rod_x"] is not None and vals["right_hand_x"] is not None else None,
                     "left_rel_z":(vals["rod_z"]-vals["left_hand_z"]) if vals["rod_z"] is not None and vals["left_hand_z"] is not None else None,
                     "right_rel_z":(vals["rod_z"]-vals["right_hand_z"]) if vals["rod_z"] is not None and vals["right_hand_z"] is not None else None})
        series.append(vals)
    series.sort(key=lambda x:x["t"])
    if not series: return row, None
    row["measured_window_span_s"]=series[-1]["t"]-series[0]["t"]
    row["measurement_samples"]=len(series)
    row["valid_observation_window"]=int("[MMC_HOVER_WINDOW]" in log and series[-1]["t"] >= required_span-0.12)
    row["rod_drop_3p5_mm"]=None
    row["rod_delta_0p5_to_end_mm"]=None
    for prefix in ("rod","left_hand","right_hand"):
        key=prefix+"_z"; vals=[s[key] for s in series]
        valid=[(s["t"],s[key]) for s in series if s[key] is not None]
        if not valid: continue
        tt=[p[0] for p in valid]; vv=[p[1] for p in valid]
        if prefix=="rod": row["rod_z_entry_m"]=vv[0]
        entry=vv[0]
        # Use the last measured sample at or before the frozen 3.5 s horizon.
        end_i=max((i for i,t in enumerate(tt) if t<=3.5),default=None)
        if end_i is not None:
            row[prefix+"_delta_end_mm"]=(vv[end_i]-entry)*1000
            row[prefix+"_end_actual_t_s"]=tt[end_i]
        start05=interp_at(tt,vv,.5)
        if start05 is not None and end_i is not None:
            row[prefix+"_delta_0p5_to_end_mm"]=(vv[end_i]-start05)*1000
    row["rod_drop_3p5_mm"]=row.get("rod_delta_end_mm")
    requested_span=3.5 if attempt["study"]=="feedforward" else 5.0
    diag=[r for r in read_csv(d/"hover_diagnostics.csv") if r.get("phase")=="MMC_HOLD"] if (d/"hover_diagnostics.csv").exists() else []
    diag=sorted(diag,key=lambda r:((fnum(r,"left_sim_time") or 0)+(fnum(r,"right_sim_time") or 0))/2)
    diag=[r for r in diag if -0.05 <= ((fnum(r,"left_sim_time") or 0)+(fnum(r,"right_sim_time") or 0))/2-t0 <= requested_span]
    if len(diag)>=2:
        q=lambda r:[fnum(r,"rod_q"+a) for a in "xyzw"]
        q0=q(diag[0]); qend=q(diag[-1]); row["rod_attitude_change_deg"]=quat_angle(q0,qend)
        row["rod_attitude_at_end_deg"]=quat_angle(q0,qend)
    for side in ("left","right"):
        relkey=side+"_rel_z"
        vals=[s[relkey] for s in series if s[relkey] is not None and s["t"]<=requested_span]
        if vals: row[side+"_rod_hand_relative_z_change_mm"]=(vals[-1]-vals[0])*1000
    ff=re.search(r"\[PAYLOAD_FF\] enabled=(\d).*target_per_arm_n=([0-9.eE+-]+)",log)
    row["logged_feedforward_enabled"]=int(ff.group(1)) if ff else None
    row["logged_feedforward_target_per_arm_n"]=float(ff.group(2)) if ff else None
    trace_path=d/"mmc_hold_trace.csv"
    if trace_path.exists():
        ff_rows=read_csv(trace_path)
        for side in ("left","right"):
            vals=[fnum(x,side+"_payload_ff_force_n") for x in ff_rows]
            vals=[x for x in vals if x is not None]
            if vals:
                row[side+"_payload_ff_min_n"]=min(vals)
                row[side+"_payload_ff_max_n"]=max(vals)
                row[side+"_payload_ff_nonzero_samples"]=sum(abs(x)>1e-9 for x in vals)
    if all(row.get(side+"_payload_ff_max_n") is not None for side in ("left","right")):
        if attempt["condition"]=="F0":
            row["valid_payload_feedforward"]=int(row.get("logged_feedforward_enabled")==0 and
                all(abs(row[side+"_payload_ff_max_n"])<1e-9 and abs(row[side+"_payload_ff_min_n"])<1e-9 for side in ("left","right")))
        else:
            row["valid_payload_feedforward"]=int(row.get("logged_feedforward_enabled")==1 and
                all(abs(row[side+"_payload_ff_max_n"]-3.1392)<.02 for side in ("left","right")))
    p=event_times(d/"rod_force_events.csv")
    if "pulse_start" in p: row["physical_pulse_start_sim_time"]=p["pulse_start"]
    if "pulse_end" in p: row["physical_pulse_end_sim_time"]=p["pulse_end"]
    # X displacement is raw relative motion from force start (or the frozen 1 s virtual start).
    if attempt["study"]=="directional":
        onset=(p.get("pulse_start",t0+1)-t0) if attempt["condition"].endswith("-F") else 1.0
        duration=(p["pulse_end"]-p["pulse_start"]) if "pulse_end" in p and "pulse_start" in p else 1.0
        if attempt["condition"].endswith("-F") and "pulse_start" in p and "pulse_end" in p:
            events={r["event"]:r for r in read_csv(d/"rod_force_events.csv")}
            row["valid_force_pulse"]=int(abs(onset-1.0)<.05 and abs(duration-1.0)<.02 and
                abs(float(events["pulse_start"].get("force_x_n",0))-2.0)<1e-9 and
                abs(float(events["pulse_end"].get("force_x_n",1)))<1e-9)
        else:
            row["valid_force_pulse"]=int(not attempt["condition"].endswith("-F"))
        row["pulse_onset_relative_to_first_reference_s"]=onset
        row["pulse_duration_s"]=duration if attempt["condition"].endswith("-F") else None
        rel_t=[s["t"]-onset for s in series]
        pulse_end=duration
        for body in ("rod","left_hand","right_hand","left_rel","right_rel"):
            key=body+"_x" if body in ("rod","left_hand","right_hand") else body
            vv=[s.get(key) for s in series]
            valid=[(t,v) for t,v in zip(rel_t,vv) if v is not None]
            if not valid: continue
            tt=[x[0] for x in valid]; xx=[x[1] for x in valid]
            base=interp_at(tt,xx,0.0)
            row[body+"_x_at_pulse_start_mm"]=0.0 if base is not None else None
            if base is None: continue
            for label,target in (("pulse_end",pulse_end),("withdrawal_plus1",pulse_end+1.0)):
                x=interp_at(tt,xx,target)
                row[body+"_x_"+label+"_mm"]=(x-base)*1000 if x is not None else None
            row[body+"_series_t_s"]=tt
            row[body+"_series_x_mm"]=[(x-base)*1000 for x in xx]
            row[body+"_series_pulse_end_s"]=pulse_end
    return row, series


def group_stats(rows, metric_keys):
    out=[]
    for group in ("F0","F1",*DIR_CONDS):
        rr=[r for r in rows if r["condition"]==group and r.get("analysis_include")==1]
        for metric in metric_keys:
            relevant=rr
            if metric in ("rod_drop_3p5_mm","rod_delta_0p5_to_end_mm","left_hand_delta_end_mm","right_hand_delta_end_mm",
                          "left_hand_delta_0p5_to_end_mm","right_hand_delta_0p5_to_end_mm","rod_attitude_at_end_deg",
                          "left_rod_hand_relative_z_change_mm","right_rod_hand_relative_z_change_mm"):
                relevant=[r for r in rr if r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1]
            elif metric.endswith(("pulse_end_mm","withdrawal_plus1_mm","withdrawal_near3_mm")):
                relevant=[r for r in rr if r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and r.get("valid_force_pulse")==1]
            stat=mean_ci([r.get(metric) for r in relevant])
            out.append({"condition":group,"metric":metric,**stat})
    return out


def grid_summary(run_data, condition_set, metric, lo, hi, step=.05, coverage_runs=None):
    selected=[x for x in run_data if x[0]["condition"] in condition_set and x[1]]
    if not selected: return np.array([]),[],[]
    # Strict common coverage: every included run must cover the entire grid.
    coverage=coverage_runs or selected
    common_lo=max(ss[0]["t"] for _,ss in coverage)
    common_hi=min(ss[-1]["t"] for _,ss in coverage)
    lo=max(lo,common_lo); hi=min(hi,common_hi)
    if hi<=lo:return np.array([]),[],[]
    grid=np.arange(lo,hi+1e-9,step)
    curves=[]
    for _,ss in selected:
        tt=[s["t"] for s in ss]; vv=[s.get(metric) for s in ss]
        keep=[i for i,v in enumerate(vv) if v is not None]
        series=np.asarray([vv[i] for i in keep],dtype=float)
        if metric.endswith("_z") and len(series): series=(series-series[0])*1000.0
        curves.append(np.interp(grid,[tt[i] for i in keep],series))
    return grid,curves,selected


def save_figures(rows, run_data):
    rows=[r for r in rows if r.get("analysis_include")==1]
    run_data=[x for x in run_data if x[0].get("analysis_include")==1]
    # 1) Payload feedforward: rod and both hands, complete common valid window.
    frows=[x for x in run_data if x[0]["study"]=="feedforward" and x[0].get("valid_observation_window")==1 and x[0].get("valid_payload_feedforward")==1 and x[1]]
    fig,axes=plt.subplots(1,3,figsize=(12,3.6),sharex=True)
    for ax,metric,label in zip(axes,("rod_z","left_hand_z","right_hand_z"),("Rod","Left hand","Right hand")):
        for cond in ("F0","F1"):
            g,curves,_=grid_summary(frows,[cond],metric,0,3.5,coverage_runs=frows)
            if not len(g):continue
            mat=np.asarray(curves)
            for y in mat: ax.plot(g,y,color=COLORS[cond],alpha=.22,lw=.7)
            ax.plot(g,mat.mean(axis=0),color=COLORS[cond],lw=1.8,label=cond)
            if len(mat)>1:ax.fill_between(g,mat.min(axis=0),mat.max(axis=0),color=COLORS[cond],alpha=.12)
        ax.axhline(0,color="0.45",lw=.7);ax.set_title(label);ax.set_xlabel("Time from first accepted MMC reference (s)")
        ax.grid(alpha=.2)
    axes[0].set_ylabel("World vertical displacement from window entry (mm)")
    axes[-1].legend(frameon=False)
    fig.tight_layout();fig.savefig(FIG/"feedforward_vertical_displacement.png",dpi=240);plt.close(fig)

    # 2) Per-run primary endpoint, points plus group mean and sample SD.
    fig,ax=plt.subplots(figsize=(5.4,3.8))
    for x,cond in enumerate(("F0","F1")):
        vals=[r["rod_drop_3p5_mm"] for r in rows if r["condition"]==cond and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and r.get("rod_drop_3p5_mm") is not None]
        jitter=np.linspace(-.08,.08,len(vals)) if vals else []
        ax.scatter(np.full(len(vals),x)+jitter,vals,color=COLORS[cond],s=24,zorder=3,label=cond)
        if vals:
            st=mean_ci(vals); ax.errorbar(x,st["mean"],yerr=st["sd"] or 0,color="black",capsize=4,fmt="D",ms=4,lw=1)
    ax.axhline(0,color="0.45",lw=.7);ax.set_xticks([0,1],["F0","F1"]);ax.set_ylabel("Rod ΔZ at last sample ≤ 3.5 s (mm)")
    ax.grid(axis="y",alpha=.2);fig.tight_layout();fig.savefig(FIG/"feedforward_endpoint_runs.png",dpi=240);plt.close(fig)

    # 3) Raw X displacement, aligned to measured physical pulse or virtual 1 s point.
    drows=[r for r in rows if r["study"]=="directional"]
    valid=[r for r in drows if r.get("analysis_include")==1 and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and r.get("valid_force_pulse")==1 and r.get("first_reference_sim_time") is not None]
    if valid:
        max_after=min((r.get("measured_window_span_s") or 0)-(r.get("pulse_onset_relative_to_first_reference_s") or 1) for r in valid)
        hi=min(4.1,max_after)
        grid=np.arange(-.4,hi+1e-9,.05) if hi>0 else np.array([])
    else:grid=np.array([])
    fig,axes=plt.subplots(1,3,figsize=(12,3.7),sharex=True)
    for ax,body,label in zip(axes,("rod","left_hand","right_hand"),("Rod","Left hand","Right hand")):
        for cond in DIR_CONDS:
            rs=[r for r in valid if r["condition"]==cond and r.get(body+"_series_t_s")]
            curves=[]
            for r in rs:
                tt=r[body+"_series_t_s"];xx=r[body+"_series_x_mm"]
                if len(grid) and grid[0]>=tt[0] and grid[-1]<=tt[-1]:
                    curves.append(np.interp(grid,tt,xx))
                    ax.plot(grid,curves[-1],color=COLORS[cond],alpha=.18,lw=.7)
            if curves:
                mat=np.asarray(curves);ax.plot(grid,mat.mean(0),color=COLORS[cond],lw=1.7,label=cond)
                if len(mat)>1:ax.fill_between(grid,mat.min(0),mat.max(0),color=COLORS[cond],alpha=.10)
        ax.axvspan(0,1,color="0.4",alpha=.08);ax.axvline(0,color="0.3",ls="--",lw=.7);ax.axvline(1,color="0.3",ls=":",lw=.7)
        ax.axhline(0,color="0.45",lw=.7);ax.set_title(label);ax.set_xlabel("Time from actual / virtual pulse start (s)");ax.grid(alpha=.2)
    axes[0].set_ylabel("Raw world-X displacement from pulse-start position (mm)");axes[-1].legend(frameon=False,fontsize=8)
    fig.tight_layout();fig.savefig(FIG/"directional_x_pulse_aligned.png",dpi=240);plt.close(fig)

    # 4) Per-run force/no-force displacement at pulse end and after withdrawal.
    fig,axes=plt.subplots(2,3,figsize=(13,6.3),sharex=True)
    times=("pulse_end","withdrawal_plus1","withdrawal_near3")
    for ax,body,label in zip(axes.flat,("rod","left_hand","right_hand","left_rel","right_rel"),("Rod","Left hand","Right hand","Rod − left hand","Rod − right hand")):
        for cond in DIR_CONDS:
            color=COLORS[cond]
            for ti,tname in enumerate(times):
                vals=[]
                for r in valid:
                    if r["condition"]!=cond:continue
                    val=r.get(body+"_x_"+tname+"_mm")
                    if val is not None:vals.append(val)
                x=ti+({"K400-N":-.24,"K400-F":-.08,"K200-N":.08,"K200-F":.24}[cond])
                if vals:
                    ax.scatter(np.full(len(vals),x),vals,s=12,color=color,alpha=.55,label=cond if ti==0 else None)
                    ax.plot(x,np.mean(vals),marker="D",ms=3,color=color)
        ax.axhline(0,color="0.45",lw=.7);ax.set_title(label);ax.set_ylabel("ΔX from pulse-start position (mm)");ax.grid(axis="y",alpha=.18)
    for ax in axes[-1]:ax.set_xticks([0,1,2],["Pulse\nend","Withdrawal\n+1 s","Common near\n+3 s"])
    axes.flat[-1].set_visible(False)
    axes[0,0].legend(frameon=False,fontsize=8)
    fig.tight_layout();fig.savefig(FIG/"directional_withdrawal_runs.png",dpi=240);plt.close(fig)


def fmt(x, digits=3):
    return "未测" if x is None else f"{x:.{digits}f}"


def write_results(attempts, rows, summary, contrasts):
    lines=["# 柔顺控制补充实验结果", "",
           "本批独立于原80次ABCD、2026-10-20悬空专项及其他批次。样本均按首次预抓取运动请求获接受计数；正式动作中止或失败保留，不替换。组间区间为独立样本Welch t区间；区组只用于平衡顺序，不作同物理起点配对。曲线使用全部有效运行及共同覆盖范围，不外推尾段。", "",
           "## 正式尝试和完成情况", "",
           "| 条件 | 正式尝试 | DONE | 主指标有效样本 |", "|---|---:|---:|---:|"]
    for c in ("F0","F1",*DIR_CONDS):
        rr=[r for r in rows if r["condition"]==c]
        m="rod_drop_3p5_mm" if c.startswith("F") else "rod_x_pulse_end_mm"
        n=summary[c]["valid_metrics"][m]["n"]
        lines.append(f"| {c} | {summary[c]['formal_attempts']} | {summary[c]['completed']} | {n} |")
    nonformal=[a for a in attempts if a.get("formal")!="1"]
    formal_fail=[a for a in attempts if a.get("formal")=="1" and a.get("task_state")!="DONE" and "EXTRA_FORMAL_RUN" not in a.get("failure_stage","")]
    extras=[a for a in attempts if "EXTRA_FORMAL_RUN" in a.get("failure_stage","")]
    lines += ["", f"正式动作前异常记录：{len(nonformal)}；正式任务未完成：{len(formal_fail)}。计划外正式运行：{len(extras)}，原始数据保留但排除主分析。逐条见 `attempts.csv`/`run_index.csv`。", ""]
    if nonformal or formal_fail:
        lines.append("失败阶段与缺失测量：")
        for a in nonformal:
            lines.append(f"- 动作前 {a.get('sample')} / {a.get('run_id')}: {a.get('failure_stage') or '启动异常，未进入正式动作'}；不计正式次数。")
        for a in formal_fail:
            lines.append(f"- 正式 {a.get('sample')} / {a.get('run_id')}: {a.get('failure_stage') or a.get('task_state')}；对应测量是否有效见 `runs.csv`。")
        lines.append("")
        lines += ["K200-N_04在接触抬升派发时检测到起点移动2.12 mm，超过既有2.00 mm限制，未进入MMC窗口；K200-N_05已完成完整5 s窗口并保留测量，随后在下降阶段报接触滑移/掉落，任务未完成。没有放宽阈值或补跑。", ""]
    lines += [
              "## 载荷前馈", "",
              "主要指标为从首次确认 MMC 参考后的观测入口到 3.5 s 内最后一个有效样本的杆 world-Z 变化（负值表示下沉）。不把低于3.5 s的缺失尾段外推。", "",
              "| 条件 | 有效n | 均值 ± 样本SD (mm) | 范围 (mm) | 完成/正式尝试 |", "|---|---:|---:|---:|---:|"]
    for c in ("F0","F1"):
        s=summary[c]["valid_metrics"]["rod_drop_3p5_mm"]
        lines.append(f"| {c} | {s['n']} | {fmt(s['mean'])} ± {fmt(s['sd'])} | {fmt(s['min'])} to {fmt(s['max'])} | {summary[c]['completed']}/{summary[c]['formal_attempts']} |")
    for c in ("rod_drop_3p5_mm","rod_delta_0p5_to_end_mm"):
        con=next(x for x in contrasts if x["study"]=="feedforward" and x["metric"]==c)
        lines.append("")
        lines.append(f"F0−F1 的 {c} 均值差={fmt(con['estimate'])} mm，95% Welch区间 [{fmt(con['ci_low'])}, {fmt(con['ci_high'])}] mm（有效n={con['n_a']}/{con['n_b']}）。该有符号位移差为负时，表示F1比F0下沉少（或相对上移）。")
    lines += ["", "实发核对：F0 10次的双臂MMC trace全窗最大值均为0 N；F1 10次均达到每臂3.1392 N。F1 trace从首个非零样本到达到目标99%的观测间隔为0.516–0.522 s（均值0.519 s），与冻结的0.5 s渐增一致到采样分辨率。逐次发布曲线见 `runs.csv` 和原始 `mmc_hold_trace.csv`。", "",
              "| 条件 | 杆0.5–3.5 s ΔZ (mm) | 左手ΔZ (mm) | 右手ΔZ (mm) | 杆姿态变化 (°) |", "|---|---:|---:|---:|---:|"]
    for c in ("F0","F1"):
        s=summary[c]["valid_metrics"]; cells=[]
        for m in ("rod_delta_0p5_to_end_mm","left_hand_delta_0p5_to_end_mm","right_hand_delta_0p5_to_end_mm","rod_attitude_at_end_deg"):
            q=s[m]
            cells.append(f"{fmt(q['mean'])} ± {fmt(q['sd'])} (n={q['n']})")
        lines.append(f"| {c} | {cells[0]} | {cells[1]} | {cells[2]} | {cells[3]} |")
    lines += ["",
              "## X向刚度和外力", "",
              "具备有效MMC窗口的运行日志确认了实际增益：K400使用Kdiag=(400,400,400,20,20,20)，K200使用(200,400,400,20,20,20)；两臂阻尼比0.8、零空间刚度10，按现有在线 operational-inertia 方法计算阻尼。任务坐标为两臂 `mj_*_link0`，其朝向与world对齐。逐次实录值见 `runs.csv`。", "",
              "曲线对齐实际物理事件 `rod_force_events.csv:pulse_start`；20次有力运行的起始时刻为首参考后1.000–1.002 s，脉冲持续1.000–1.002 s，总力2 N并在结束事件清零。N组按首参考后1.0 s和同一1.0 s时长作虚拟区间。图中位移相对各运行脉冲起点位置，未作线性背景外推。响应估计为同刚度施力组减无外力组的组均值差；交互估计为(K200施力−K200无力)−(K400施力−K400无力)。这属于独立组均值对照估计，不是直接测得的净外力响应。共同可用的撤力后末端为2.891 s；所有组都以该实测时点计算，不外推到准确3 s。", "",
              "| 刚度 | 条件 | 杆X脉冲末均值±SD (mm) | 脉冲末范围 (mm) | 撤力+1s均值±SD (mm) | 撤力近+2.891s均值±SD (mm) |", "|---|---|---:|---:|---:|---:|"]
    for k in (400,200):
        for suffix in ("N","F"):
            c=f"K{k}-{suffix}";s=summary[c]["valid_metrics"]
            cells=[]
            for stem in ("rod_x_pulse_end_mm","rod_x_withdrawal_plus1_mm","rod_x_withdrawal_near3_mm"):
                q=s[stem];cells.append(f"{fmt(q['mean'])} ± {fmt(q['sd'])} (n={q['n']})")
            pulse=s["rod_x_pulse_end_mm"]
            span=f"{fmt(pulse['min'])} to {fmt(pulse['max'])}"
            lines.append(f"| {k} N/m | {c} | {cells[0]} | {span} | {cells[1]} | {cells[2]} |")
    lines += ["", "### 组间估计", "", "| 指标 | 对照 | 均值差 (mm) | 95% Welch区间 (mm) | n |", "|---|---|---:|---:|---:|"]
    for c in contrasts:
        if c["study"]!="directional":continue
        ns=(f"{c['n_k200_f']}/{c['n_k200_n']}/{c['n_k400_f']}/{c['n_k400_n']} (K200-F/K200-N/K400-F/K400-N)" if c.get("n_k200_f") is not None else f"{c['n_a']}/{c['n_b']}")
        lines.append(f"| {c['metric']} | {c['contrast']} | {fmt(c['estimate'])} | [{fmt(c['ci_low'])}, {fmt(c['ci_high'])}] | {ns} |")
    lines += ["", "### 竖向、姿态和完成情况", "", "| 条件 | 完成/正式 | 杆ΔZ (mm) | 左手ΔZ (mm) | 右手ΔZ (mm) | 杆姿态变化 (°) |", "|---|---:|---:|---:|---:|---:|"]
    for c in DIR_CONDS:
        s=summary[c]["valid_metrics"]
        cells=[]
        for m in ("rod_drop_3p5_mm","left_hand_delta_end_mm","right_hand_delta_end_mm","rod_attitude_at_end_deg"):
            q=s[m]
            cells.append(f"{fmt(q['mean'])} ± {fmt(q['sd'])} (n={q['n']})")
        lines.append(f"| {c} | {summary[c]['completed']}/{summary[c]['formal_attempts']} | {cells[0]} | {cells[1]} | {cells[2]} | {cells[3]} |")
    lines += ["", "## 双手、姿态、竖向保持和限制", "",
              "逐次杆与双手世界位移、杆减双手相对位移变化、杆姿态、实发前馈、实际脉冲事件和任务状态均见 `runs.csv`。CSV未覆盖的指标为空值，不以0代替。杆—手相对运动只能说明两者的相对位置变化，不能直接称为指垫滑移。每个运行原始曲线、接触与诊断记录保存在 `experiment_results/a1_runs/<run_id>/`；目录索引见 `run_index.csv`。", "",
              "### 图表", "",
              "1. `figures/feedforward_vertical_displacement.png`：F0/F1杆和双手竖向曲线，所有有效单次曲线、组均值和范围。",
              "2. `figures/feedforward_endpoint_runs.png`：前馈主要指标逐次散点、均值和样本SD。",
              "3. `figures/directional_x_pulse_aligned.png`：四条件X向曲线与1 s实际/虚拟脉冲时段。",
              "4. `figures/directional_withdrawal_runs.png`：脉冲末、撤力后1 s与共同可用近3 s的逐次让步/残余。", "",
              "区间用于表达该批有限样本的不确定性，不作显著性检验结论、总体成功率或真机能力外推。"]
    (OUT/"results.md").write_text("\n".join(lines)+"\n",encoding="utf-8")


def main():
    attempts=read_csv(OUT/"attempts.csv")
    schedule=read_csv(OUT/"schedule.csv")
    scheduled={r["sample"] for r in schedule}
    formal_by_sample={}
    for a in attempts:
        if a.get("formal")=="1" and a.get("sample") in scheduled:
            formal_by_sample.setdefault(a["sample"],a)
    run_rows=[];run_data=[]
    for a in attempts:
        if a.get("formal")!="1": continue
        row,series=load_run(a)
        row["analysis_include"]=int(a.get("sample") in scheduled and formal_by_sample.get(a.get("sample"),{}).get("run_id")==a.get("run_id"))
        row["attempt_class"]="scheduled_primary" if row["analysis_include"] else "extra_formal_excluded"
        run_rows.append(row);run_data.append((row,series))
    # Common near-3 s withdrawal endpoint, without extrapolating any run tail.
    directional=[r for r in run_rows if r.get("analysis_include")==1 and r["study"]=="directional" and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and r.get("valid_force_pulse")==1 and r.get("pulse_onset_relative_to_first_reference_s") is not None]
    if directional:
        post_spans=[]
        for r in directional:
            end_rel=(r.get("pulse_duration_s") if r.get("requested_force") else 1.0) or 1.0
            onset=r["pulse_onset_relative_to_first_reference_s"]
            post_spans.append((r.get("measured_window_span_s") or 0)-onset-end_rel)
        near_post=max(0.0,min(3.0,min(post_spans)))
        for r in directional:
            d=RUNS/r["run_id"]
            probe=[x for x in read_csv(d/"contact_probe.csv") if x.get("phase")=="MMC_HOLD"]
            t0=r["first_reference_sim_time"]; onset=r["pulse_onset_relative_to_first_reference_s"]
            end_rel=(r.get("pulse_duration_s") if r.get("requested_force") else 1.0) or 1.0
            times=[];vals={k:[] for k in ("rod_x","left_hand_x","right_hand_x","left_rel_x","right_rel_x")}
            for x in probe:
                ref_tm=(float(x["left_sim_time"])+float(x["right_sim_time"]))/2-t0
                if ref_tm<0 or ref_tm>5.0: continue
                tm=ref_tm-onset
                times.append(tm)
                for k in ("rod_x","left_hand_x","right_hand_x"):
                    vals[k].append(float(x[k]) if x.get(k) else float("nan"))
                vals["left_rel_x"].append(float(x["rod_x"])-float(x["left_hand_x"]))
                vals["right_rel_x"].append(float(x["rod_x"])-float(x["right_hand_x"]))
            for k in vals:
                arr=np.asarray(vals[k],dtype=float); base=interp_at(times,arr.tolist(),0.0)
                for lab,post in (("pulse_end",0.0),("withdrawal_plus1",1.0),("withdrawal_near3",near_post)):
                    target=end_rel+post; value=interp_at(times,arr.tolist(),target)
                    r[k+"_"+lab+"_mm"]=(value-base)*1000 if base is not None and value is not None else None
            r["withdrawal_near3_actual_after_end_s"]=near_post
    write_csv(OUT/"runs.csv",run_rows)
    # Run index preserves failures/nonformal records too.
    write_csv(OUT/"run_index.csv",attempts)
    metrics=["rod_drop_3p5_mm","rod_delta_0p5_to_end_mm","left_hand_delta_end_mm","right_hand_delta_end_mm",
             "left_hand_delta_0p5_to_end_mm","right_hand_delta_0p5_to_end_mm","rod_attitude_at_end_deg",
             "left_rod_hand_relative_z_change_mm","right_rod_hand_relative_z_change_mm",
             "rod_x_pulse_end_mm","rod_x_withdrawal_plus1_mm","rod_x_withdrawal_near3_mm",
             "left_hand_x_pulse_end_mm","left_hand_x_withdrawal_plus1_mm","left_hand_x_withdrawal_near3_mm",
             "right_hand_x_pulse_end_mm","right_hand_x_withdrawal_plus1_mm","right_hand_x_withdrawal_near3_mm",
             "left_rel_x_pulse_end_mm","left_rel_x_withdrawal_plus1_mm","left_rel_x_withdrawal_near3_mm",
             "right_rel_x_pulse_end_mm","right_rel_x_withdrawal_plus1_mm","right_rel_x_withdrawal_near3_mm"]
    stats=group_stats(run_rows,metrics)
    write_csv(OUT/"group_statistics.csv",stats)
    contrasts=[]
    for metric in ("rod_drop_3p5_mm","rod_delta_0p5_to_end_mm"):
        a=[r.get(metric) for r in run_rows if r.get("analysis_include")==1 and r["condition"]=="F0" and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1]
        b=[r.get(metric) for r in run_rows if r.get("analysis_include")==1 and r["condition"]=="F1" and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1]
        contrasts.append({"study":"feedforward","metric":metric,"contrast":"F0 - F1 (lower is less sinking)",**welch(a,b)})
    for metric in ("rod_x_pulse_end_mm","rod_x_withdrawal_plus1_mm","rod_x_withdrawal_near3_mm",
                   "left_hand_x_pulse_end_mm","left_hand_x_withdrawal_plus1_mm","left_hand_x_withdrawal_near3_mm",
                   "right_hand_x_pulse_end_mm","right_hand_x_withdrawal_plus1_mm","right_hand_x_withdrawal_near3_mm",
                   "left_rel_x_pulse_end_mm","left_rel_x_withdrawal_plus1_mm","left_rel_x_withdrawal_near3_mm",
                   "right_rel_x_pulse_end_mm","right_rel_x_withdrawal_plus1_mm","right_rel_x_withdrawal_near3_mm"):
        valid=lambda r:r.get("analysis_include")==1 and r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and r.get("valid_force_pulse")==1
        a=[r.get(metric) for r in run_rows if r["condition"]=="K400-F" and valid(r)]
        b=[r.get(metric) for r in run_rows if r["condition"]=="K400-N" and valid(r)]
        c=[r.get(metric) for r in run_rows if r["condition"]=="K200-F" and valid(r)]
        d=[r.get(metric) for r in run_rows if r["condition"]=="K200-N" and valid(r)]
        contrasts.append({"study":"directional","metric":metric,"contrast":"K400 force - no force",**welch(a,b)})
        contrasts.append({"study":"directional","metric":metric,"contrast":"K200 force - no force",**welch(c,d)})
        k400=welch(a,b);k200=welch(c,d)
        if all(k.get("estimate") is not None for k in (k400,k200)):
            # Independent four-cell difference-in-differences, Welch-Satterthwaite df.
            groups=[[r.get(metric) for r in run_rows if r["condition"]==cnd and valid(r)] for cnd in DIR_CONDS]
            xx=[np.asarray([v for v in g if v is not None],dtype=float) for g in groups]
            if all(len(x)>1 for x in xx):
                vars=[np.var(x,ddof=1)/len(x) for x in xx];se2=sum(vars)
                df=se2**2/sum(v*v/(len(x)-1) for v,x in zip(vars,xx)) if se2 else sum(len(x) for x in xx)-4
                estimate=k200["estimate"]-k400["estimate"]; se=math.sqrt(se2); half=student_t.ppf(.975,df)*se
                contrasts.append({"study":"directional","metric":metric,"contrast":"(K200 F-N) - (K400 F-N)",
                                  "n_a":sum(len(x) for x in xx),"n_b":0,
                                  "n_k200_f":len(xx[3]),"n_k200_n":len(xx[2]),"n_k400_f":len(xx[1]),"n_k400_n":len(xx[0]),
                                  "estimate":estimate,"se":se,"df":df,
                                  "ci_low":estimate-half,"ci_high":estimate+half})
    write_csv(OUT/"contrast_estimates.csv",contrasts)
    save_figures(run_rows,run_data)
    # Structured roll-up
    summary={}
    for c in ("F0","F1",*DIR_CONDS):
        rr=[r for r in run_rows if r["condition"]==c and r.get("analysis_include")==1]
        valid=[r for r in rr if r.get("valid_observation_window")==1 and r.get("valid_payload_feedforward")==1 and
               (r.get("valid_force_pulse")==1 or r.get("study")!="directional")]
        sample_ids={s["sample"] for s in schedule if s["condition"]==c}
        summary[c]={"formal_attempts":sum(1 for a in attempts if a["sample"] in sample_ids and a["formal"]=="1"),
                    "completed":sum(r["completed"] for r in rr),"valid_metrics":{m:mean_ci([r.get(m) for r in valid]) for m in metrics}}
    (OUT/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    write_results(attempts,run_rows,summary,contrasts)
    print(json.dumps({c:{"attempts":s["formal_attempts"],"completed":s["completed"]} for c,s in summary.items()},ensure_ascii=False))


if __name__=="__main__":main()
