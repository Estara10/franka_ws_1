#!/usr/bin/env python3
"""Summarize and plot the frozen full-carry disturbance A/B batch."""
from __future__ import annotations

import csv
import math
import os
import re
import statistics
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/franka_ws_1_mplconfig")
import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
font_manager.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUNS = ROOT / "experiment_results/a1_runs"
LEDGER = OUT / "attempts.csv"
COLORS = {"A": "#245a9b", "B": "#d17a00"}
PHASES = ("LIFT", "TRANSPORT", "LOWER")
MAIN_PLAN_DURATION_S = 3.040

matplotlib.rcParams["font.family"] = [font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc").get_name(), "DejaVu Sans"]


def read_csv(path: Path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows, fields=None):
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def fnum(value, default=float("nan")):
    try:
        v = float(value)
        return v if math.isfinite(v) else default
    except (TypeError, ValueError):
        return default


def parse_event_times(text, token, event):
    pattern = re.compile(r"\[[^\]]*\]\s*\[([^\]]+)\].*?" + re.escape(token) +
                        r"[^\r\n]*event=" + re.escape(event) + r"[^\r\n]*?sim_time=([0-9.eE+-]+)")
    found = []
    for m in pattern.finditer(text):
        found.append((fnum(m.group(1)), fnum(m.group(2))))
    return found


def parse_log(run_dir: Path):
    path = run_dir / "task_launch.log"
    text = path.read_text(errors="replace") if path.is_file() else ""
    main_start = parse_event_times(text, "[TRANSPORT_MAIN_EVENT]", "start")
    main_end = parse_event_times(text, "[TRANSPORT_MAIN_EVENT]", "end")
    transport_start = parse_event_times(text, "[TRANSPORT_EVENT]", "start")
    transport_end = parse_event_times(text, "[TRANSPORT_EVENT]", "end")
    release = re.search(r"\[CONTACT_PROBE_RELEASE_COMPARISON\][^\r\n]*", text)
    released = re.search(r"\[CONTACT_PROBE_RELEASED\][^\r\n]*", text)
    connection = re.search(r"\[CONTACT_TRANSPORT_COMMON_CONNECTION\][^\r\n]*", text)
    fail = re.findall(r"\[CONTACT_PROBE_FAILURE\] ([^\r\n]+)", text)
    if not fail:
        fail = re.findall(r"\[CONTACT_PROBE_RESULT\] state=FAILED detail=([^\r\n]+)", text)
    result = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+)", text)
    first_pregrasp = None
    accepted = None
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if "[P3A-1] 下发轨迹" in line:
            mt = re.search(r"\[([0-9]+\.[0-9]+)\]", line)
            if mt:
                first_pregrasp = float(mt.group(1))
            for later in lines[i+1:]:
                if "Execute request accepted" in later:
                    mt = re.search(r"\[([0-9]+\.[0-9]+)\]", later)
                    if mt:
                        accepted = float(mt.group(1))
                    break
            break
    terminal_time = None
    for line in reversed(lines):
        if "[CONTACT_PROBE_RESULT]" in line:
            mt = re.search(r"\[([0-9]+\.[0-9]+)\]", line)
            if mt:
                terminal_time = float(mt.group(1))
            break
    connection_offset = float("nan")
    if connection:
        mt = re.search(r"object_offset_mm=([0-9.eE+-]+)", connection.group(0))
        if mt:
            connection_offset = fnum(mt.group(1))
    release_position = release_yaw = float("nan")
    if release:
        mt = re.search(r"landing_xy_error_mm=([0-9.eE+-]+)", release.group(0))
        if mt:
            release_position = fnum(mt.group(1))
        mt = re.search(r"angle_deg=([0-9.eE+-]+)", release.group(0))
        if mt:
            # This is the release attitude change, not target yaw error.
            release_yaw = fnum(mt.group(1))
    target_yaw_error = float("nan")
    if released:
        mt = re.search(r"yaw_error_rad=([0-9.eE+-]+)", released.group(0))
        if mt:
            target_yaw_error = math.degrees(fnum(mt.group(1)))
    observed_main_end = main_end[-1][1] if main_end else float("nan")
    main_end_source = "observed_event" if math.isfinite(observed_main_end) else "planned_duration_fallback"
    analysis_main_end = observed_main_end
    if not math.isfinite(analysis_main_end) and main_start:
        # Keep an explicit nominal-window fallback for time-aligned metrics only.
        # The measured event duration below remains missing when the end event is absent.
        analysis_main_end = main_start[-1][1] + MAIN_PLAN_DURATION_S
    return {
        "text": text,
        "main_start": main_start[-1][1] if main_start else float("nan"),
        "main_end": observed_main_end,
        "analysis_main_end": analysis_main_end,
        "main_end_source": main_end_source,
        "transport_start": transport_start[-1][1] if transport_start else float("nan"),
        "transport_end": transport_end[-1][1] if transport_end else float("nan"),
        "release_position": release_position,
        "release_attitude_change": release_yaw,
        "target_yaw_error": target_yaw_error,
        "connection_offset": connection_offset,
        "failure": fail[-1] if fail else "",
        "terminal_state": result[-1] if result else "",
        "task_wall_time": terminal_time-accepted if terminal_time and accepted else float("nan"),
    }


def quat_from(row, names):
    q = np.array([fnum(row[n]) for n in names], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        return None
    return q/norm


def quat_slerp(q0, q1, u):
    q1 = q1.copy()
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 *= -1.0
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = q0 + u*(q1-q0)
        return q/np.linalg.norm(q)
    theta = math.acos(dot)
    s = math.sin(theta)
    return (math.sin((1-u)*theta)/s)*q0 + (math.sin(u*theta)/s)*q1


def interpolate_reference(rows, time_s):
    if not rows or not math.isfinite(time_s):
        return None
    ordered = sorted(rows, key=lambda r: fnum(r.get("sim_time_s")))
    dedup = []
    for row in ordered:
        t = fnum(row.get("sim_time_s"))
        if not math.isfinite(t):
            continue
        if dedup and abs(t-fnum(dedup[-1]["sim_time_s"])) < 1e-9:
            dedup[-1] = row
        else:
            dedup.append(row)
    if not dedup or time_s < fnum(dedup[0]["sim_time_s"])-1e-8 or time_s > fnum(dedup[-1]["sim_time_s"])+1e-8:
        return None
    if time_s <= fnum(dedup[0]["sim_time_s"]):
        return dedup[0]
    if time_s >= fnum(dedup[-1]["sim_time_s"]):
        return dedup[-1]
    hi = next(i for i, r in enumerate(dedup) if fnum(r["sim_time_s"]) >= time_s)
    a, b = dedup[hi-1], dedup[hi]
    ta, tb = fnum(a["sim_time_s"]), fnum(b["sim_time_s"])
    u = (time_s-ta)/(tb-ta) if tb > ta else 0.0
    out = {"sim_time_s": str(time_s)}
    for prefix in ("rod_ref", "left_hand_ref", "right_hand_ref"):
        pa = np.array([fnum(a[f"{prefix}_{axis}_m"]) for axis in "xyz"])
        pb = np.array([fnum(b[f"{prefix}_{axis}_m"]) for axis in "xyz"])
        p = pa + u*(pb-pa)
        for axis, value in zip("xyz", p):
            out[f"{prefix}_{axis}_m"] = value
        qnames = [f"{prefix}_q{axis}" for axis in "xyzw"]
        qa, qb = quat_from(a, qnames), quat_from(b, qnames)
        if qa is None or qb is None:
            return None
        q = quat_slerp(qa, qb, u)
        for axis, value in zip("xyzw", q):
            out[f"{prefix}_q{axis}"] = value
    return out


def position_error(actual, reference):
    if actual is None or reference is None:
        return float("nan"), [float("nan")]*3
    delta = np.array(actual)-np.array(reference)
    return float(np.linalg.norm(delta))*1000.0, list(delta)


def pose_error(actual_q, ref_q):
    if actual_q is None or ref_q is None:
        return float("nan")
    dot = min(1.0, max(-1.0, abs(float(np.dot(actual_q, ref_q)))))
    return math.degrees(2.0*math.acos(dot))


def actual_pose(row, kind):
    if kind == "rod":
        pkeys = ["rod_x", "rod_y", "rod_z"]
        qkeys = [f"rod_q{x}" for x in "xyzw"]
    elif kind == "left":
        pkeys = [f"left_hand_{x}" for x in "xyz"]
        qkeys = [f"left_hand_q{x}" for x in "xyzw"]
    else:
        pkeys = [f"right_hand_{x}" for x in "xyz"]
        qkeys = [f"right_hand_q{x}" for x in "xyzw"]
    p = [fnum(row[k]) for k in pkeys]
    q = quat_from(row, qkeys)
    return p, q


def event_rows(path):
    return read_csv(path)


def metric_stats(values):
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"n": 0, "mean": float("nan"), "sd": float("nan"), "median": float("nan"),
                "min": float("nan"), "max": float("nan")}
    return {"n": len(vals), "mean": statistics.mean(vals),
            "sd": statistics.stdev(vals) if len(vals)>1 else 0.0,
            "median": statistics.median(vals), "min": min(vals), "max": max(vals)}


def interp_series(rows, t, xkey, ykey):
    if not rows or t < rows[0][xkey] or t > rows[-1][xkey]:
        return float("nan")
    xs = [r[xkey] for r in rows]
    i = int(np.searchsorted(xs, t, side="left"))
    if i == 0:
        return rows[0][ykey]
    if i >= len(rows):
        return rows[-1][ykey]
    a, b = rows[i-1], rows[i]
    u = (t-a[xkey])/(b[xkey]-a[xkey]) if b[xkey]>a[xkey] else 0.0
    return a[ykey]+u*(b[ykey]-a[ykey])


def process_run(attempt):
    run_id = attempt["run_id"]
    run_dir = RUNS / run_id
    logs = parse_log(run_dir)
    contacts = read_csv(run_dir / "contact_probe.csv")
    references = read_csv(run_dir / "abcd_reference.csv")
    force_events = event_rows(run_dir / "rod_force_events.csv")
    payload_trace = read_csv(run_dir / "abcd_payload_trace.csv")
    refs_by_phase = defaultdict(list)
    for row in references:
        if row.get("phase") in PHASES:
            refs_by_phase[row["phase"]].append(row)
    force_times = {r["event"]: fnum(r["sim_time"]) for r in force_events}
    payload_transport = [r for r in payload_trace if r.get("phase") == "TRANSPORT"]
    pulse_start = force_times.get("pulse_start", float("nan"))
    pulse_end = force_times.get("pulse_end", float("nan"))
    main_start, main_end = logs["main_start"], logs["analysis_main_end"]
    tracking = []
    for row in contacts:
        phase = row.get("phase", "")
        if phase not in PHASES:
            continue
        obs_t = fnum(row.get("observation_sim_time"))
        truth_req = fnum(row.get("truth_request_sim_time"))
        truth_recv = fnum(row.get("truth_received_sim_time"))
        truth_t = (truth_req+truth_recv)/2.0 if math.isfinite(truth_req) and math.isfinite(truth_recv) else float("nan")
        if phase == "TRANSPORT":
            if not (math.isfinite(main_start) and math.isfinite(main_end) and main_start-1e-6 <= truth_t <= main_end+1e-6):
                continue
        refs = refs_by_phase.get(phase, [])
        ref_rod = interpolate_reference(refs, truth_t)
        ref_hands = interpolate_reference(refs, obs_t)
        if ref_rod is None or ref_hands is None:
            continue
        ar_p, ar_q = actual_pose(row, "rod")
        al_p, al_q = actual_pose(row, "left")
        rr_p = [fnum(ref_rod[f"rod_ref_{x}_m"]) for x in "xyz"]
        rr_q = quat_from(ref_rod, [f"rod_ref_q{x}" for x in "xyzw"])
        lh_ref_p = [fnum(ref_hands[f"left_hand_ref_{x}_m"]) for x in "xyz"]
        lh_ref_q = quat_from(ref_hands, [f"left_hand_ref_q{x}" for x in "xyzw"])
        rh_p, rh_q = actual_pose(row, "right")
        rh_ref_p = [fnum(ref_hands[f"right_hand_ref_{x}_m"]) for x in "xyz"]
        rh_ref_q = quat_from(ref_hands, [f"right_hand_ref_q{x}" for x in "xyzw"])
        rod_mm, rod_vec = position_error(ar_p, rr_p)
        left_mm, left_vec = position_error(al_p, lh_ref_p)
        right_mm, right_vec = position_error(rh_p, rh_ref_p)
        if phase == "TRANSPORT":
            phase_origin = main_start
        else:
            phase_origin = min((fnum(r.get("sim_time_s")) for r in refs), default=obs_t)
        tracking.append({
            "sequence": attempt["sequence"], "condition": attempt["condition"], "run_id": run_id,
            "phase": phase, "truth_time_s": truth_t, "observation_time_s": obs_t,
            "time_rel_phase_s": obs_t-phase_origin, "time_rel_main_s": truth_t-main_start if phase=="TRANSPORT" else "",
            "rod_ref_x_m": rr_p[0], "rod_ref_y_m": rr_p[1], "rod_ref_z_m": rr_p[2],
            "rod_x_error_m": rod_vec[0], "rod_y_error_m": rod_vec[1], "rod_z_error_m": rod_vec[2],
            "rod_position_error_mm": rod_mm, "rod_orientation_error_deg": pose_error(ar_q, rr_q),
            "left_hand_position_error_mm": left_mm, "left_hand_orientation_error_deg": pose_error(al_q, lh_ref_q),
            "right_hand_position_error_mm": right_mm, "right_hand_orientation_error_deg": pose_error(rh_q, rh_ref_q),
            "left_rod_hand_residual_mm": fnum(row.get("left_slip_m"))*1000.0,
            "left_rod_hand_residual_deg": math.degrees(fnum(row.get("left_angle_rad"))),
            "right_rod_hand_residual_mm": fnum(row.get("right_slip_m"))*1000.0,
            "right_rod_hand_residual_deg": math.degrees(fnum(row.get("right_angle_rad"))),
            "pulse_start_rel_s": pulse_start-main_start if math.isfinite(pulse_start) and math.isfinite(main_start) else "",
            "pulse_end_rel_s": pulse_end-main_start if math.isfinite(pulse_end) and math.isfinite(main_start) else "",
        })
    transport = sorted((r for r in tracking if r["phase"]=="TRANSPORT"), key=lambda r:r["truth_time_s"])
    baseline = metric_stats([r["rod_x_error_m"]*1000 for r in transport
                             if math.isfinite(pulse_start) and main_start <= r["truth_time_s"] < pulse_start])
    for r in transport:
        r["rod_x_error_mm"] = r["rod_x_error_m"]*1000.0
        r["rod_x_change_from_preforce_median_mm"] = r["rod_x_error_mm"]-baseline["median"] if baseline["n"] else float("nan")
    main_span = main_end-main_start if math.isfinite(main_end) and math.isfinite(main_start) else float("nan")
    pulse_rel_start = pulse_start-main_start if math.isfinite(pulse_start) and math.isfinite(main_start) else float("nan")
    pulse_rel_end = pulse_end-main_start if math.isfinite(pulse_end) and math.isfinite(main_start) else float("nan")
    during = [r for r in transport if pulse_start <= r["truth_time_s"] <= pulse_end]
    late_recovery = [r for r in transport if math.isfinite(pulse_end) and r["truth_time_s"] >= pulse_end+0.5]
    delta_during = [r["rod_x_change_from_preforce_median_mm"] for r in during]
    late_delta = [r["rod_x_change_from_preforce_median_mm"] for r in late_recovery]
    end_candidates = [r for r in transport if math.isfinite(pulse_end) and r["truth_time_s"] <= pulse_end]
    pulse_end_rel = pulse_end-main_start if math.isfinite(pulse_end) and math.isfinite(main_start) else float("nan")
    recovery_snapshots = {
        f"x_change_{offset:.1f}s_after_pulse_mm": interp_series(
            transport, pulse_end_rel+offset, "time_rel_main_s", "rod_x_change_from_preforce_median_mm")
        if math.isfinite(pulse_end_rel) else float("nan")
        for offset in (0.5, 1.0, 1.4)
    }
    metrics = {
        **logs,
        "main_plan_duration_s": MAIN_PLAN_DURATION_S,
        "main_event_duration_s": logs["main_end"]-main_start if math.isfinite(logs["main_end"]) and math.isfinite(main_start) else float("nan"),
        "main_end_source": logs["main_end_source"],
        "transport_event_duration_s": logs["transport_end"]-logs["transport_start"] if math.isfinite(logs["transport_end"]) and math.isfinite(logs["transport_start"]) else float("nan"),
        "pulse_delay_after_main_s": pulse_start-main_start if math.isfinite(pulse_start) and math.isfinite(main_start) else float("nan"),
        "pulse_duration_s": pulse_end-pulse_start if math.isfinite(pulse_end) and math.isfinite(pulse_start) else float("nan"),
        "pulse_start_sim_s": pulse_start, "pulse_end_sim_s": pulse_end,
        "preforce_x_error_median_mm": baseline["median"], "preforce_x_error_n": baseline["n"],
        "pulse_peak_abs_x_change_mm": max((abs(v) for v in delta_during if math.isfinite(v)), default=float("nan")),
        "pulse_peak_signed_x_change_mm": (max(delta_during, key=abs) if delta_during else float("nan")),
        "pulse_end_x_change_mm": end_candidates[-1]["rod_x_change_from_preforce_median_mm"] if end_candidates else float("nan"),
        "late_recovery_x_change_mean_mm": statistics.mean(late_delta) if late_delta else float("nan"),
        "late_recovery_x_change_last_mm": late_delta[-1] if late_delta else float("nan"),
        "late_recovery_n": len(late_delta),
        **recovery_snapshots,
        "transport_rod_x_rms_mm": math.sqrt(statistics.mean([r["rod_x_error_m"]**2 for r in transport]))*1000 if transport else float("nan"),
        "transport_rod_z_rms_mm": math.sqrt(statistics.mean([r["rod_z_error_m"]**2 for r in transport]))*1000 if transport else float("nan"),
        "transport_rod_position_rms_mm": math.sqrt(statistics.mean([(r["rod_position_error_mm"]/1000.0)**2 for r in transport]))*1000 if transport else float("nan"),
        "transport_rod_orientation_mean_deg": statistics.mean([r["rod_orientation_error_deg"] for r in transport]) if transport else float("nan"),
        "transport_hand_position_mean_mm": statistics.mean([v for r in transport for v in (r["left_hand_position_error_mm"],r["right_hand_position_error_mm"]) if math.isfinite(v)]) if transport else float("nan"),
        "transport_hand_orientation_mean_deg": statistics.mean([v for r in transport for v in (r["left_hand_orientation_error_deg"],r["right_hand_orientation_error_deg"]) if math.isfinite(v)]) if transport else float("nan"),
        "transport_left_rod_hand_residual_mean_mm": statistics.mean([r["left_rod_hand_residual_mm"] for r in transport]) if transport else float("nan"),
        "transport_right_rod_hand_residual_mean_mm": statistics.mean([r["right_rod_hand_residual_mm"] for r in transport]) if transport else float("nan"),
        "tracking_sample_n": len(tracking),
        "release_position_error_mm": logs["release_position"],
        "release_attitude_change_deg": logs["release_attitude_change"],
        "release_target_yaw_error_deg": logs["target_yaw_error"],
        "release_target_yaw_abs_error_deg": abs(logs["target_yaw_error"]) if math.isfinite(logs["target_yaw_error"]) else float("nan"),
        "connection_object_offset_mm": logs["connection_offset"],
        "feedforward_enabled": "true" if attempt["condition"]=="B" else "false",
        "feedforward_target_per_arm_n": 3.1392 if attempt["condition"]=="B" else 0.0,
        "feedforward_left_transport_start_n": fnum(payload_transport[0].get("left_published_ff_z_n")) if payload_transport else 0.0,
        "feedforward_right_transport_start_n": fnum(payload_transport[0].get("right_published_ff_z_n")) if payload_transport else 0.0,
        "feedforward_left_transport_max_n": max((fnum(r.get("left_published_ff_z_n")) for r in payload_transport), default=0.0),
        "feedforward_right_transport_max_n": max((fnum(r.get("right_published_ff_z_n")) for r in payload_transport), default=0.0),
        "force_event_armed": int("armed" in force_times),
        "force_event_started": int("pulse_start" in force_times),
        "force_event_ended": int("pulse_end" in force_times),
    }
    for phase in PHASES:
        phase_rows = [r for r in tracking if r["phase"] == phase]
        prefix = phase.lower()
        phase_metrics = (
            ("rod_position_error_mean_mm", "rod_position_error_mm"),
            ("rod_orientation_error_mean_deg", "rod_orientation_error_deg"),
            ("left_hand_position_error_mean_mm", "left_hand_position_error_mm"),
            ("right_hand_position_error_mean_mm", "right_hand_position_error_mm"),
            ("left_hand_orientation_error_mean_deg", "left_hand_orientation_error_deg"),
            ("right_hand_orientation_error_mean_deg", "right_hand_orientation_error_deg"),
            ("left_rod_hand_residual_mean_mm", "left_rod_hand_residual_mm"),
            ("right_rod_hand_residual_mean_mm", "right_rod_hand_residual_mm"),
        )
        for output_key, source_key in phase_metrics:
            values = [fnum(r[source_key]) for r in phase_rows if math.isfinite(fnum(r[source_key]))]
            metrics[f"{prefix}_{output_key}"] = statistics.mean(values) if values else float("nan")
    return tracking, transport, metrics


def compare_main_references(attempts):
    by_group = {"A": [], "B": []}
    for attempt in attempts:
        refs = [r for r in read_csv(RUNS/attempt["run_id"]/"abcd_reference.csv")
                if r.get("phase") == "TRANSPORT"]
        refs.sort(key=lambda r: fnum(r.get("plan_elapsed_s")))
        times = np.array([fnum(r.get("plan_elapsed_s")) for r in refs])
        positions = np.array([[fnum(r.get(f"rod_ref_{axis}_m")) for axis in "xyz"] for r in refs])
        quats = np.array([[fnum(r.get(f"rod_ref_q{axis}")) for axis in "xyzw"] for r in refs])
        valid = np.isfinite(times) & np.isfinite(positions).all(axis=1) & np.isfinite(quats).all(axis=1)
        times, positions, quats = times[valid], positions[valid], quats[valid]
        if len(times) < 2:
            continue
        keep = np.r_[True, np.diff(times) > 1e-9]
        by_group[attempt["condition"]].append((attempt["run_id"], times[keep], positions[keep], quats[keep]))
    comparisons = []
    for run_a, time_a, pos_a, quat_a in by_group["A"]:
        for run_b, time_b, pos_b, quat_b in by_group["B"]:
            lo, hi = max(time_a[0], time_b[0]), min(time_a[-1], time_b[-1])
            if hi <= lo:
                continue
            grid = np.linspace(lo, hi, 301)
            pa = np.column_stack([np.interp(grid, time_a, pos_a[:, i]) for i in range(3)])
            pb = np.column_stack([np.interp(grid, time_b, pos_b[:, i]) for i in range(3)])
            qa = np.column_stack([np.interp(grid, time_a, quat_a[:, i]) for i in range(4)])
            qb = np.column_stack([np.interp(grid, time_b, quat_b[:, i]) for i in range(4)])
            p_error = np.linalg.norm(pa-pb, axis=1)*1000.0
            qa /= np.linalg.norm(qa, axis=1)[:, None]
            qb /= np.linalg.norm(qb, axis=1)[:, None]
            dots = np.clip(np.abs(np.sum(qa*qb, axis=1)), 0.0, 1.0)
            q_error = np.degrees(2.0*np.arccos(dots))
            comparisons.append({
                "run_A": run_a, "run_B": run_b, "overlap_plan_elapsed_s": hi-lo,
                "max_position_difference_mm": float(np.max(p_error)),
                "rms_position_difference_mm": float(np.sqrt(np.mean(p_error**2))),
                "max_orientation_difference_deg": float(np.max(q_error)),
                "mean_orientation_difference_deg": float(np.mean(q_error)),
            })
    return comparisons


def add_stats(summary, group, metric, values, unit):
    s = metric_stats(values)
    summary.append({"condition": group, "metric": metric, "n": s["n"], "unit": unit,
                    "mean": s["mean"], "sd": s["sd"], "median": s["median"],
                    "min": s["min"], "max": s["max"]})


def stats_text(s, digits=2):
    if not s["n"]:
        return "无有效测量"
    return (f"n={s['n']}，均值 {s['mean']:.{digits}f}，标准差 {s['sd']:.{digits}f}，"
            f"中位数 {s['median']:.{digits}f}，范围 {s['min']:.{digits}f}–{s['max']:.{digits}f}")


def plot_tracking(tracking):
    phase_names = {"LIFT": "抬升", "TRANSPORT": "运输主段", "LOWER": "下降"}
    metrics = [
        ("position", "位置误差 (mm)", ["rod_position_error_mm", "left_hand_position_error_mm", "right_hand_position_error_mm"]),
        ("attitude", "姿态误差 (deg)", ["rod_orientation_error_deg", "left_hand_orientation_error_deg", "right_hand_orientation_error_deg"]),
    ]
    line_styles = ["-", "--", ":"]
    names = ["杆", "左手", "右手"]
    for name, ylabel, keys in metrics:
        fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.1), sharey=False)
        for ax, phase in zip(axes, PHASES):
            subset = [r for r in tracking if r["phase"]==phase]
            for group in ("A", "B"):
                group_rows = [r for r in subset if r["condition"]==group]
                for run in sorted({r["run_id"] for r in group_rows}):
                    run_rows = sorted([r for r in group_rows if r["run_id"]==run], key=lambda r:r["time_rel_phase_s"])
                    xs = [r["time_rel_phase_s"] for r in run_rows]
                    for key, style in zip(keys, line_styles):
                        ys = [r[key] for r in run_rows]
                        if any(math.isfinite(y) for y in ys):
                            ax.plot(xs, ys, color=COLORS[group], linestyle=style, linewidth=0.65, alpha=0.20)
                # Median curves are calculated over fixed elapsed-time bins; raw run curves stay visible.
                if group_rows:
                    xmax = max(r["time_rel_phase_s"] for r in group_rows if math.isfinite(r["time_rel_phase_s"]))
                    bins = np.linspace(0.0, xmax, 45)
                    centers = 0.5*(bins[:-1]+bins[1:])
                    for key, style in zip(keys, line_styles):
                        vals = []
                        for lo, hi in zip(bins[:-1], bins[1:]):
                            bucket = [r[key] for r in group_rows if lo <= r["time_rel_phase_s"] < hi and math.isfinite(r[key])]
                            vals.append(float(np.median(bucket)) if bucket else float("nan"))
                        ax.plot(centers, vals, color=COLORS[group], linestyle=style, linewidth=1.8)
            ax.set_title(phase_names[phase], fontsize=10)
            ax.set_xlabel("阶段内仿真时间 (s)")
            ax.grid(True, alpha=0.22)
        axes[0].set_ylabel(ylabel)
        handles = [Line2D([0],[0], color=COLORS[g], lw=2, label=f"{g}组") for g in ("A","B")]
        handles += [Line2D([0],[0], color="#444444", linestyle=style, lw=1.8, label=label)
                    for style,label in zip(line_styles,names)]
        fig.legend(handles=handles, loc="upper center", ncol=5, frameon=False, bbox_to_anchor=(0.5,1.02))
        fig.tight_layout(rect=(0,0,1,0.94))
        fig.savefig(OUT/f"full_carry_{name}_error.png", dpi=180, bbox_inches="tight")
        plt.close(fig)


def plot_disturbance(samples, metrics_by_run):
    fig, axes = plt.subplots(2,1,figsize=(9.5,7.2),sharex=True)
    for group in ("A","B"):
        group_runs = sorted({r["run_id"] for r in samples if r["condition"]==group})
        for run in group_runs:
            rows = sorted((r for r in samples if r["run_id"]==run),key=lambda r:r["time_rel_main_s"])
            xs = [r["time_rel_main_s"] for r in rows]
            axes[0].plot(xs,[r["rod_x_error_mm"] for r in rows],color=COLORS[group],alpha=0.18,lw=0.7)
            axes[1].plot(xs,[r["rod_x_change_from_preforce_median_mm"] for r in rows],color=COLORS[group],alpha=0.18,lw=0.7)
        grid=np.linspace(0,3.04,80)
        for ax, key in zip(axes,["rod_x_error_mm","rod_x_change_from_preforce_median_mm"]):
            curves=[]
            for run in group_runs:
                rows=sorted((r for r in samples if r["run_id"]==run),key=lambda r:r["time_rel_main_s"])
                xs=np.array([r["time_rel_main_s"] for r in rows],dtype=float)
                ys=np.array([r[key] for r in rows],dtype=float)
                valid=np.isfinite(xs)&np.isfinite(ys)
                if valid.sum()>=2:
                    lo,hi=xs[valid].min(),xs[valid].max()
                    curves.append(np.interp(grid[(grid>=lo)&(grid<=hi)],xs[valid],ys[valid]))
            if curves:
                # Recompute per-grid medians with coverage retained.
                vals=[]
                for t in grid:
                    at=[]
                    for run in group_runs:
                        rows=sorted((r for r in samples if r["run_id"]==run),key=lambda r:r["time_rel_main_s"])
                        at.append(interp_series(rows,t,"time_rel_main_s",key))
                    finite=[v for v in at if math.isfinite(v)]
                    vals.append(float(np.median(finite)) if finite else float("nan"))
                ax.plot(grid,vals,color=COLORS[group],lw=2.2,label=f"{group}组中位数")
    for ax in axes:
        ax.axvspan(1.0,1.5,color="#888888",alpha=0.15)
        ax.axhline(0,color="#555555",lw=0.8)
        ax.grid(True,alpha=0.22)
    axes[0].set_ylabel("原始 X 位置误差 (mm)")
    axes[1].set_ylabel("相对施力前中位数变化 (mm)")
    axes[1].set_xlabel("运输主段起点后的仿真时间 (s)")
    axes[0].legend(frameon=False,ncol=2)
    fig.tight_layout()
    fig.savefig(OUT/"disturbance_x_response_and_recovery.png",dpi=180,bbox_inches="tight")
    plt.close(fig)


def plot_placement(formal):
    fig, axes = plt.subplots(1,3,figsize=(11.5,4.2),gridspec_kw={"width_ratios":[1,1,0.8]})
    jitter={}
    for group_i, group in enumerate(("A","B")):
        rows=[r for r in formal if r["condition"]==group]
        xs=[]; ys=[]; yaw=[]
        for i,r in enumerate(rows):
            offset=(i-4.5)*0.035
            jitter[r["run_id"]]=group_i+offset
            if math.isfinite(r["release_position_error_mm"]):
                xs.append(group_i+offset); ys.append(r["release_position_error_mm"])
            if math.isfinite(r["release_target_yaw_error_deg"]):
                yaw.append((group_i+offset,r["release_target_yaw_error_deg"]))
        axes[0].scatter(xs,ys,s=24,color=COLORS[group],label=f"{group}组",zorder=3)
        axes[1].scatter([x for x,_ in yaw],[y for _,y in yaw],s=24,color=COLORS[group],zorder=3)
        done=sum(r["flow_completed"]=="1" for r in rows)
        axes[2].bar(group_i,done,color=COLORS[group],width=0.58)
        axes[2].text(group_i,done+0.15,f"{done}/10",ha="center",va="bottom",fontsize=9)
        npos=sum(math.isfinite(r["release_position_error_mm"]) for r in rows)
        nyaw=sum(math.isfinite(r["release_target_yaw_error_deg"]) for r in rows)
        axes[0].text(group_i,0.98,f"n={npos}",transform=axes[0].get_xaxis_transform(),ha="center",va="top",fontsize=8)
        axes[1].text(group_i,0.98,f"n={nyaw}",transform=axes[1].get_xaxis_transform(),ha="center",va="top",fontsize=8)
    for ax in axes[:2]:
        ax.set_xticks([0,1],labels=["A","B"])
        ax.grid(True,axis="y",alpha=0.22)
    axes[0].set_ylabel("释放后水平落点误差 (mm)")
    axes[1].set_ylabel("释放后偏航目标误差 (deg)")
    axes[2].set_xticks([0,1],labels=["A","B"])
    axes[2].set_ylim(0,10.7)
    axes[2].set_ylabel("完整任务数 (次/10)")
    axes[2].grid(True,axis="y",alpha=0.22)
    fig.tight_layout()
    fig.savefig(OUT/"placement_error_and_completion.png",dpi=180,bbox_inches="tight")
    plt.close(fig)


def main():
    ledger = read_csv(LEDGER)
    formal_attempts = [r for r in ledger if r["phase"]=="formal" and r["action_started"]=="1"]
    if len(formal_attempts)!=20 or {c:sum(r["condition"]==c for r in formal_attempts) for c in ("A","B")}!={"A":10,"B":10}:
        raise SystemExit(f"expected 20 formal action attempts, got {len(formal_attempts)}")
    formal_attempts.sort(key=lambda r:int(r["sequence"]))
    all_tracking=[]; all_transport=[]; enriched=[]; per_run={}
    for attempt in formal_attempts:
        tracking,transport,metrics=process_run(attempt)
        all_tracking.extend(tracking); all_transport.extend(transport)
        row=dict(attempt)
        row.update({k:v for k,v in metrics.items() if k!="text"})
        row["release_measurement_valid"] = int(math.isfinite(fnum(metrics["release_position_error_mm"])))
        row["retreat_completed"] = int(attempt["flow_completed"] == "1")
        if attempt["flow_completed"] == "1":
            row["failure_stage"] = ""
            row["completed_stages"] = "抓取、抬升、运输、下降、支撑、释放、撤退"
        elif attempt["direct_reason"] == "E_CONTACT_RELEASE_TRUTH_OR_FEEDBACK":
            row["failure_stage"] = "释放后真值/反馈校验（已开爪，未进入撤退）"
            row["completed_stages"] = "抓取、抬升、运输、下降、支撑、开爪"
        else:
            row["failure_stage"] = attempt["direct_reason"] or "未定位"
            row["completed_stages"] = ""
        enriched.append(row); per_run[attempt["run_id"]]=row
    reference_comparisons = compare_main_references(formal_attempts)
    write_csv(OUT/"reference_comparison.csv", reference_comparisons)
    tracking_fields=list(all_tracking[0]) if all_tracking else []
    write_csv(OUT/"aligned_tracking_samples.csv",all_tracking,tracking_fields)
    write_csv(OUT/"disturbance_response_samples.csv",all_transport,list(all_transport[0]) if all_transport else [])
    write_csv(OUT/"formal_attempts.csv",enriched)

    summary=[]
    numerical_metrics = {
        "main_event_duration_s": "s", "transport_event_duration_s": "s", "task_wall_time": "s",
        "connection_object_offset_mm": "mm", "transport_rod_x_rms_mm": "mm",
        "transport_rod_z_rms_mm": "mm", "transport_rod_position_rms_mm": "mm",
        "transport_rod_orientation_mean_deg": "deg", "transport_hand_position_mean_mm": "mm",
        "transport_hand_orientation_mean_deg": "deg", "transport_left_rod_hand_residual_mean_mm": "mm",
        "transport_right_rod_hand_residual_mean_mm": "mm", "release_position_error_mm": "mm",
        "release_target_yaw_error_deg": "deg", "release_target_yaw_abs_error_deg": "deg",
        "release_attitude_change_deg": "deg",
        "pulse_delay_after_main_s": "s", "pulse_duration_s": "s",
        "feedforward_left_transport_start_n": "N", "feedforward_right_transport_start_n": "N",
        "feedforward_left_transport_max_n": "N", "feedforward_right_transport_max_n": "N",
        "preforce_x_error_median_mm": "mm", "pulse_peak_abs_x_change_mm": "mm",
        "pulse_peak_signed_x_change_mm": "mm", "pulse_end_x_change_mm": "mm",
        "late_recovery_x_change_mean_mm": "mm", "late_recovery_x_change_last_mm": "mm",
        "x_change_0.5s_after_pulse_mm": "mm", "x_change_1.0s_after_pulse_mm": "mm",
        "x_change_1.4s_after_pulse_mm": "mm",
    }
    for group in ("A","B"):
        records=[r for r in enriched if r["condition"]==group]
        summary.append({"condition":group,"metric":"full_task_completion","n":10,"unit":"count/10",
                        "mean":sum(r["flow_completed"]=="1" for r in records),"sd":"","median":"","min":"","max":""})
        for metric,unit in numerical_metrics.items():
            add_stats(summary,group,metric,[fnum(r.get(metric)) for r in records],unit)
        for phase in ("lift", "transport", "lower"):
            for suffix, unit in (
                ("rod_position_error_mean_mm", "mm"),
                ("rod_orientation_error_mean_deg", "deg"),
                ("left_hand_position_error_mean_mm", "mm"),
                ("right_hand_position_error_mean_mm", "mm"),
                ("left_hand_orientation_error_mean_deg", "deg"),
                ("right_hand_orientation_error_mean_deg", "deg"),
                ("left_rod_hand_residual_mean_mm", "mm"),
                ("right_rod_hand_residual_mean_mm", "mm"),
            ):
                metric = f"{phase}_{suffix}"
                add_stats(summary,group,metric,[fnum(r.get(metric)) for r in records],unit)
    write_csv(OUT/"group_summary.csv",summary)

    pre_action=[r for r in ledger if r["phase"]=="formal" and r["action_started"]!="1"]
    pre_rows=[]
    for r in pre_action:
        wrapper=ROOT/r["launcher_log"]
        wrapper_text=wrapper.read_text(errors="replace") if wrapper.exists() else ""
        pre_rows.append({**r,"classification":"pre-action MoveIt path-planning rejection" if "P2b path check did NOT pass" in wrapper_text else "pre-action launch exception"})
    write_csv(OUT/"pre_action_exceptions.csv",pre_rows)

    plot_tracking(all_tracking)
    plot_disturbance(all_transport,enriched)
    plot_placement(enriched)

    def summary_for(group, metric):
        vals=[fnum(r.get(metric)) for r in enriched if r["condition"]==group]
        return metric_stats(vals)
    counts={g:sum(r["flow_completed"]=="1" for r in enriched if r["condition"]==g) for g in ("A","B")}
    reference_position_max = max((r["max_position_difference_mm"] for r in reference_comparisons), default=float("nan"))
    reference_orientation_max = max((r["max_orientation_difference_deg"] for r in reference_comparisons), default=float("nan"))
    text=[]
    text.append("# 完整搬运受力让步对照：结果分析\n")
    text.append("## 实验范围与方法\n")
    text.append("本批次在固定仿真场景中比较两套完整控制方案：A为JTC位置控制；B为独立笛卡尔阻抗及杆载荷前馈。B包含阻抗和载荷前馈两项实现，组间差异评价的是完整方案，不能将差异单独归因于阻抗算法。两组采用相同的共同物体参考、3.040 s运输主段和正常放置流程；正式动作各10次，预试验不纳入统计。B预试验完成全流程；A首次预试验在下降阶段触发杆支撑穿透检查，统一两组下降终端姿态后，A复核运行完成全流程。正式日志确认A组前馈为0 N，B组10次在运输段开始时两臂均已达到3.1392 N。每次正式运行均在运输主段开始1.0 s后，对杆质心施加世界+X方向2 N、持续0.5 s的单次扰动；正式运行20/20均记录了施力起止事件。实际施力起止以`rod_force_events.csv`中的仿真时间为准。没有无扰动组，因此施力前后误差变化不能解释为外力的净响应。\n")
    text.append("\n## 任务完成与放置\n")
    text.append(f"A组完成{counts['A']}/10，B组完成{counts['B']}/10。两次B组未完成均发生在释放后的最终真值/反馈合并检查，夹爪打开和约2 s释放测量等待已完成，但未进入撤退；该检查的合并错误码未区分真值服务失败和反馈校验失败。两次均保留为未完成任务，不替换。动作前另有{len(pre_rows)}次P2b预抓取路径规划未通过，没有接受预抓取运动请求，作为动作前异常单列。\n")
    for group in ("A","B"):
        xy=summary_for(group,"release_position_error_mm")
        yaw=summary_for(group,"release_target_yaw_error_deg")
        yaw_abs=summary_for(group,"release_target_yaw_abs_error_deg")
        text.append(f"- {group}组释放测量：水平落点误差 {stats_text(xy)} mm；有符号偏航目标误差 {stats_text(yaw)} deg，偏航误差绝对值 {stats_text(yaw_abs)} deg。仅使用成功完成释放测量的样本，缺失值不补零。\n")
    text.append("\n## 共同运输与轨迹跟踪\n")
    for group in ("A","B"):
        dur=summary_for(group,"main_event_duration_s")
        xr=summary_for(group,"transport_rod_x_rms_mm")
        zr=summary_for(group,"transport_rod_z_rms_mm")
        pos=summary_for(group,"transport_rod_position_rms_mm")
        attitude=summary_for(group,"transport_rod_orientation_mean_deg")
        hand=summary_for(group,"transport_hand_position_mean_mm")
        wall=summary_for(group,"task_wall_time")
        text.append(f"- {group}组运输主段实测事件时长 {stats_text(dur)} s（计划3.040 s；事件时长受观测循环采样影响），任务墙钟时长 {stats_text(wall)} s。杆X向误差RMS {stats_text(xr)} mm、竖向误差RMS {stats_text(zr)} mm、三维位置误差RMS {stats_text(pos)} mm、姿态误差均值 {stats_text(attitude)} deg；双手相对各自参考的平均位置误差 {stats_text(hand)} mm。\n")
    text.append(f"共同参考规划起点连接单独保留，不并入3.040 s主段误差。A/B组逐次实际运输物体参考在共同时间区间内插值比较，跨组逐次组合的最大位置差为{reference_position_max:.3f} mm，最大姿态差为{reference_orientation_max:.3f} deg（逐组合结果见`reference_comparison.csv`），参考路径相近但不完全相同。曲线按各条仿真时间戳插值对齐，未按数组下标配对，也未外推。杆真值服务记录请求与响应时间，杆真值对齐时间采用两者中点作为近似；双手反馈按观测记录时刻对齐。A组两次JTC日志未写入主段结束事件，误差曲线按记录的主段起点加计划3.040 s确定分析窗口，实际事件时长统计仍按有效结束事件计（n=8）。杆—手相对残差是由手位姿和冻结抓取变换推得的几何残差，不等同于直接测得的接触滑移。\n")
    text.append("\n## 受力让步与恢复\n")
    for group in ("A","B"):
        peak=summary_for(group,"pulse_peak_abs_x_change_mm")
        end=summary_for(group,"pulse_end_x_change_mm")
        late=summary_for(group,"late_recovery_x_change_mean_mm")
        after05=summary_for(group,"x_change_0.5s_after_pulse_mm")
        after10=summary_for(group,"x_change_1.0s_after_pulse_mm")
        after14=summary_for(group,"x_change_1.4s_after_pulse_mm")
        text.append(f"- {group}组：施力期间相对施力前X向误差中位数的最大绝对变化 {stats_text(peak)} mm；脉冲结束附近变化 {stats_text(end)} mm。撤力后0.5/1.0/1.4 s时的误差变化均值分别为{after05['mean']:.2f}/{after10['mean']:.2f}/{after14['mean']:.2f} mm（有效样本数{after05['n']}/{after10['n']}/{after14['n']}）；撤力后0.5 s至主段末的时间窗平均变化为 {stats_text(late)} mm。原始误差与变化曲线同时给出。\n")
    peak_a=summary_for("A","pulse_peak_abs_x_change_mm"); peak_b=summary_for("B","pulse_peak_abs_x_change_mm")
    late_a=summary_for("A","late_recovery_x_change_mean_mm"); late_b=summary_for("B","late_recovery_x_change_mean_mm")
    b14=summary_for("B","x_change_1.4s_after_pulse_mm"); a14=summary_for("A","x_change_1.4s_after_pulse_mm")
    text.append(f"按每次运行自身施力前X误差中位数中心化，B组施力期最大变化均值比A组高{peak_b['mean']-peak_a['mean']:.2f} mm；B组误差变化均值从撤力后0.5 s的{summary_for('B','x_change_0.5s_after_pulse_mm')['mean']:.2f} mm降至1.0 s的{summary_for('B','x_change_1.0s_after_pulse_mm')['mean']:.2f} mm，并在1.4 s时为{b14['mean']:.2f} mm；A组对应为{summary_for('A','x_change_0.5s_after_pulse_mm')['mean']:.2f}/{summary_for('A','x_change_1.0s_after_pulse_mm')['mean']:.2f}/{a14['mean']:.2f} mm。B组较快接近施力前水平，随后出现更大的反向偏差，不能概括为持续稳定回零。上述响应包括完整控制系统的轨迹跟踪、承载和场景演化；由于缺少无扰动对照，不将全部误差变化归因于这次外力。让步较大需结合恢复残差、放置误差和完成情况判断，不能直接称为更优。\n")
    text.append("\n## 结果解释与适用范围\n")
    text.append("综合而言，B组的竖向跟踪误差较小；在本扰动时序下，施力后X向偏差更快接近施力前水平，但到撤力1.4 s时平均已转为−2.80 mm，表现出反向超调，不能称为稳定回零。代价是水平让步更大，完成释放测量的B样本落点误差和偏航误差绝对值均值高于A组，且两次释放后校验未完成。A组完成率为10/10，B组为8/10；这些结果支持在承载姿态跟踪和短时扰动响应需求下继续考察B方案，同时要求改进释放后验证与落点表现。B组两次未完成的直接物理原因未由合并检查记录区分，不能据此判定为控制或搬运失败。本实验仅适用于固定起终点、共同名义物体路径和指定2 N短时扰动下的两套完整控制方案，不外推至任意起点、角度或载荷。均值和标准差为描述性统计，未用不同编号的独立初始化运行冒充重复物理配对。\n")
    text.append("\n## 图表\n")
    text.append("![物体与双手位置误差](full_carry_position_error.png)\n\n图1　杆和双手在抬升、运输主段及下降阶段的参考位置误差。细线为逐次运行，粗线为组内时间分箱中位数。\n")
    text.append("![物体与双手姿态误差](full_carry_attitude_error.png)\n\n图2　杆和双手相对各自姿态参考的角误差。细线为逐次运行，粗线为组内时间分箱中位数。\n")
    text.append("![运输扰动响应与恢复](disturbance_x_response_and_recovery.png)\n\n图3　杆的X向原始参考误差及相对本次运行施力前中位数的变化。灰色区域为计划施力区间；细线为逐次运行，粗线为组内中位数。\n")
    text.append("![释放误差与任务完成](placement_error_and_completion.png)\n\n图4　释放后水平落点误差、带符号偏航目标误差及完整任务次数。散点表示全部有效逐次测量。\n")
    text.append("逐次测量位于`formal_attempts.csv`、`aligned_tracking_samples.csv`和`disturbance_response_samples.csv`，组统计见`group_summary.csv`；逐次共同参考差异见`reference_comparison.csv`。\n")
    (OUT/"results.md").write_text("\n".join(text),encoding="utf-8")
    print(f"formal={len(enriched)} A={counts['A']}/10 B={counts['B']}/10 tracking_samples={len(all_tracking)}")


if __name__=="__main__":
    main()
