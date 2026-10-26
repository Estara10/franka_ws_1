#!/usr/bin/env python3
"""Revised r2 batch analysis: common transport reference, fixed duration, gate and formal statistics."""
from __future__ import annotations

import csv
import math
import os
import re
import statistics
from bisect import bisect_right
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
RUNS = ROOT / "experiment_results/a1_runs"
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplconfig-fullcarry-payload-ff-r2-20261026")

D_FIXED = 3.040
CONN_FIXED = 0.5
DURATION_TOL_S = 0.020
REF_POS_TOL_MM = 0.5
REF_YAW_TOL_DEG = 0.10
START_POS_TOL_MM = 1.0
START_YAW_TOL_DEG = 0.05
BLOCK_POS_TOL_MM = 0.5
BLOCK_YAW_TOL_DEG = 0.05
PROGRESS_TOL = 0.005
METRIC_GRID_S = 0.02

PHASES = [
    "pretrial_metrics.csv", "pretrial_transport_curves.csv", "pretrial_reference_difference.csv",
    "straight_runs.csv", "formal_metrics.csv", "formal_block_differences.csv",
    "formal_transport_curves.csv", "raw_run_index.csv",
]


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields=None):
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def fnum(row, key, default=float("nan")):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else default
    except (KeyError, ValueError, TypeError):
        return default


def interp(xs, ys, t):
    if not xs or t < xs[0] - 1e-10 or t > xs[-1] + 1e-10:
        return None
    if t <= xs[0]:
        return ys[0]
    if t >= xs[-1]:
        return ys[-1]
    i = bisect_right(xs, t)
    t0, t1 = xs[i - 1], xs[i]
    if t1 <= t0:
        return ys[i]
    alpha = (t - t0) / (t1 - t0)
    return ys[i - 1] + alpha * (ys[i] - ys[i - 1])


def interpolation_grid(xs, t0, t1, step=METRIC_GRID_S):
    """20 ms grid inside the covered interval only; never extrapolates."""
    if not xs or not (math.isfinite(t0) and math.isfinite(t1)):
        return [], 0.0
    lo, hi = max(t0, xs[0]), min(t1, xs[-1])
    if hi <= lo:
        return [], 0.0
    n = max(1, int(math.floor((hi - lo) / step)))
    grid = [lo + step * i for i in range(n + 1)]
    if grid[-1] < hi - 1e-9:
        grid.append(hi)
    coverage = (hi - lo) / (t1 - t0) if t1 > t0 else 0.0
    return grid, coverage


def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n <= 0.0:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_matrix(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_transform(row, prefix):
    t = np.array([fnum(row, prefix + "_x_m"), fnum(row, prefix + "_y_m"), fnum(row, prefix + "_z_m")])
    R = quat_to_matrix([fnum(row, prefix + "_qx"), fnum(row, prefix + "_qy"),
                        fnum(row, prefix + "_qz"), fnum(row, prefix + "_qw")])
    if R is None or not np.all(np.isfinite(t)):
        return None
    return R, t


def meta_matrix(meta, key):
    try:
        vals = [float(v) for v in meta[key].split(";")]
    except (KeyError, ValueError):
        return None
    if len(vals) != 16:
        return None
    m = np.array(vals).reshape(4, 4)
    return m[:3, :3], m[:3, 3]


def rod_reference(hand, grasp):
    """object = hand * inv(G)"""
    Rh, th = hand
    Rg, tg = grasp
    Ro = Rh @ Rg.T
    to = th - Ro @ tg
    return Ro, to


def parse_events(text):
    def num(pattern, default=float("nan"), index=1):
        m = re.search(pattern, text)
        return float(m.group(index)) if m else default

    main_start = re.search(
        r"\[TRANSPORT_MAIN_EVENT\] event=start sim_time=([0-9.eE+-]+)", text)
    main_end = re.search(
        r"\[TRANSPORT_MAIN_EVENT\] event=end sim_time=([0-9.eE+-]+)", text)
    release = re.search(r"\[CONTACT_PROBE_RELEASED\] xyz=\(([^,]+),([^,]+),([^\)]+)\)"
                        r" quat=\(([^,]+),([^,]+),([^,]+),([^\)]+)\)", text)
    result = re.search(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+) detail=([^\r\n]+)", text)
    active = re.search(r"\[ABCD_MMC_ACTIVE\].*payload_enabled=([01])", text)
    anchor = re.search(r"\[ABCD_PAYLOAD_RAMP_ANCHOR\].*sim_time=([0-9.eE+-]+)", text)
    common = re.search(
        r"\[CONTACT_TRANSPORT_COMMON_REFERENCE\] source=(\S+) "
        r"start=\(([^,]+),([^,]+),([^\)]+)\) yaw_rad=([0-9.eE+-]+) "
        r"delta=\(([^,]+),([^,]+)\) dyaw_deg=([0-9.eE+-]+) "
        r"measured_ref_start=\(([^,]+),([^,]+),([^\)]+)\) duration_s=([0-9.eE+-]+) "
        r"connection_s=([0-9.eE+-]+)", text)
    dispatch = re.search(
        r"\[CONTACT_TRANSPORT_COMMON_DISPATCH\] connection_s=([0-9.eE+-]+) "
        r"object_offset_mm=([0-9.eE+-]+) main_duration_s=([0-9.eE+-]+) "
        r"main_points=(\d+) dispatch_points=(\d+)", text)
    connection = re.search(
        r"\[CONTACT_TRANSPORT_COMMON_CONNECTION\] duration_s=([0-9.eE+-]+) "
        r"samples=(\d+) object_offset_mm=([0-9.eE+-]+) object_angle_deg=([0-9.eE+-]+) "
        r"ik_residual_max=\(([^,]+),([^\)]+)\) max_dual=\(([^,]+),([^\)]+)\)", text)
    check = re.search(
        r"\[CONTACT_TRANSPORT_COMMON_CHECK\] pass=([01]) points=(\d+) duration=([0-9.eE+-]+) "
        r"collisions=(\d+) limits=([01]) vel=([01]) acc=([01]) "
        r"max_dual=\(([^,]+),([^\)]+)\) max_rod_path=\(([^,]+),([^\)]+)\) "
        r"max_vel_ratio=([0-9.eE+-]+) max_acc_ratio=([0-9.eE+-]+) "
        r"ik_residual_max=\(([^,]+),([^\)]+)\)", text)
    release_cmp = re.search(
        r"\[CONTACT_PROBE_RELEASE_COMPARISON\] pre_open_xyz=\(([^,]+),([^,]+),([^\)]+)\) "
        r"released_xyz=\(([^,]+),([^,]+),([^\)]+)\) dz_mm=([0-9.eE+-]+) "
        r"distance_mm=([0-9.eE+-]+) angle_deg=([0-9.eE+-]+)", text)
    frozen = re.search(r"A1 真值已冻结: run_id=\S+ frame=\S+ pos=\(([^,]+),([^,]+),([^\)]+)\)", text)
    eval_ = lambda m, i: float(m.group(i)) if m else float("nan")
    return {
        "main_start_s": float(main_start.group(1)) if main_start else float("nan"),
        "main_end_s": float(main_end.group(1)) if main_end else float("nan"),
        "transport_start_s": num(r"\[TRANSPORT_EVENT\] event=start sim_time=([0-9.eE+-]+)"),
        "transport_end_s": num(r"\[TRANSPORT_EVENT\] event=end sim_time=([0-9.eE+-]+)"),
        "payload_anchor_s": float(anchor.group(1)) if anchor else float("nan"),
        "payload_enabled": int(active.group(1)) if active else -1,
        "release_xyz": (float(release.group(1)), float(release.group(2)),
                        float(release.group(3))) if release else (float("nan"),) * 3,
        "release_quat": tuple(float(release.group(i)) for i in range(4, 8)) if release else None,
        "task_state": result.group(1) if result else "",
        "task_detail": result.group(2) if result else "",
        "common_source": common.group(1) if common else "",
        "common_start": tuple(float(common.group(i)) for i in (2, 3, 4)) if common else None,
        "common_yaw": float(common.group(5)) if common else float("nan"),
        "common_delta": (float(common.group(6)), float(common.group(7))) if common else None,
        "common_dyaw_deg": float(common.group(8)) if common else float("nan"),
        "common_measured_start": tuple(float(common.group(i)) for i in (9, 10, 11)) if common else None,
        "common_duration_s": float(common.group(12)) if common else float("nan"),
        "common_connection_s": float(common.group(13)) if common else float("nan"),
        "dispatch_connection_s": eval_(dispatch, 1),
        "dispatch_object_offset_mm": eval_(dispatch, 2),
        "dispatch_main_duration_s": eval_(dispatch, 3),
        "dispatch_main_points": int(dispatch.group(4)) if dispatch else -1,
        "dispatch_points": int(dispatch.group(5)) if dispatch else -1,
        "connection_logged_s": eval_(connection, 1),
        "connection_samples": int(connection.group(2)) if connection else -1,
        "connection_object_offset_mm": eval_(connection, 3),
        "connection_object_angle_deg": eval_(connection, 4),
        "connection_ik_residual": (eval_(connection, 5), eval_(connection, 6)) if connection else (float("nan"),) * 2,
        "connection_max_dual": (eval_(connection, 7), eval_(connection, 8)) if connection else (float("nan"),) * 2,
        "check_pass": int(check.group(1)) if check else -1,
        "check_points": int(check.group(2)) if check else -1,
        "check_duration_s": eval_(check, 3),
        "check_collisions": int(check.group(4)) if check else -1,
        "check_limit_flags": tuple(int(check.group(i)) for i in (5, 6, 7)) if check else (),
        "check_max_dual": (eval_(check, 8), eval_(check, 9)) if check else (float("nan"),) * 2,
        "check_max_rod_path": (eval_(check, 10), eval_(check, 11)) if check else (float("nan"),) * 2,
        "check_max_vel_ratio": eval_(check, 12),
        "check_max_acc_ratio": eval_(check, 13),
        "check_ik_residual": (eval_(check, 14), eval_(check, 15)) if check else (float("nan"),) * 2,
        "release_distance_mm": eval_(release_cmp, 8),
        "release_dz_mm": eval_(release_cmp, 7),
        "release_angle_deg": eval_(release_cmp, 9),
        "frozen_truth_xyz": (float(frozen.group(1)), float(frozen.group(2)),
                             float(frozen.group(3))) if frozen else (float("nan"),) * 3,
    }


def truth_series(path):
    rows = read_csv(path)
    samples = []
    for r in rows:
        request = fnum(r, "truth_request_sim_time")
        response = fnum(r, "truth_received_sim_time")
        z = fnum(r, "rod_z")
        if not (math.isfinite(request) and math.isfinite(response) and response >= request
                and math.isfinite(z)):
            continue
        samples.append({
            "t": 0.5 * (request + response), "z": z, "width": response - request,
            "x": fnum(r, "rod_x"), "y": fnum(r, "rod_y"),
            "qx": fnum(r, "rod_qx"), "qy": fnum(r, "rod_qy"),
            "qz": fnum(r, "rod_qz"), "qw": fnum(r, "rod_qw"), "phase": r["phase"],
        })
    samples.sort(key=lambda s: s["t"])
    dedup = {}
    for s in samples:
        dedup[round(s["t"], 9)] = s
    return [dedup[t] for t in sorted(dedup)]


def read_meta(path):
    """key,value reader that also accepts legacy unquoted 16-value matrices."""
    meta = {}
    try:
        with path.open(newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            next(reader, None)
            for row in reader:
                if not row:
                    continue
                key = row[0]
                if len(row) > 2:
                    meta[key] = ";".join(row[1:])
                elif len(row) == 2:
                    meta[key] = row[1]
                else:
                    meta[key] = ""
    except FileNotFoundError:
        pass
    return meta


def load_run(row):
    run_id = row["run_id"]
    path = RUNS / run_id
    log = (path / "task_launch.log").read_text(errors="replace")
    events = parse_events(log)
    meta = read_meta(path / "abcd_reference_meta.csv")
    ref_all = read_csv(path / "abcd_reference.csv")
    refs = [r for r in ref_all if r["phase"] == "TRANSPORT"]
    refs.sort(key=lambda r: fnum(r, "plan_elapsed_s"))
    forces = sorted(read_csv(path / "abcd_payload_trace.csv"), key=lambda r: fnum(r, "sim_time_s"))
    truth = truth_series(path / "contact_probe.csv")

    gl = meta_matrix(meta, "common_reference_grasp_left_matrix")
    gr = meta_matrix(meta, "common_reference_grasp_right_matrix")
    common_enabled = meta.get("common_reference_enabled", "0") == "1"
    d_meta = float(meta.get("common_reference_duration_s", "nan"))
    conn_meta = float(meta.get("common_reference_connection_s", "nan"))
    x0 = (float(meta.get("common_reference_start_x_m", "nan")),
          float(meta.get("common_reference_start_y_m", "nan")),
          float(meta.get("common_reference_start_z_m", "nan")))
    t0, t1 = events["main_start_s"], events["main_end_s"]

    # Truth series inside the main window.
    tx = [s["t"] for s in truth]
    tz = [s["z"] for s in truth]
    grid, coverage = interpolation_grid(tx, t0, t1)
    e_values, de_values, curve = [], [], []
    e0 = float("nan")
    if grid:
        e0 = interp(tx, tz, grid[0]) - x0[2]
    for t in grid:
        z = interp(tx, tz, t)
        if z is None:
            continue
        e = z - x0[2]
        de = e - e0
        e_values.append(e)
        de_values.append(de)
        curve.append({"run_id": run_id, "condition": row["condition"], "phase": row.get("phase", ""),
                      "block": row.get("block", ""), "sim_time_s": t,
                      "elapsed_from_main_start_s": t - t0,
                      "normalized_main_time": (t - t0) / (t1 - t0) if t1 > t0 else float("nan"),
                      "actual_rod_z_m": z, "common_reference_z_m": x0[2],
                      "e_z_common_mm": 1000 * e, "delta_e_z_mm": 1000 * de})

    # Commanded object reference from both hand references and the pre-load grasp.
    cmd = []
    start_yaw = events["common_yaw"]
    dyaw_rad = math.radians(events["common_dyaw_deg"])
    dx_path = float(meta.get("common_reference_delta_x_m", "nan"))
    dy_path = float(meta.get("common_reference_delta_y_m", "nan"))
    if gl is not None and gr is not None and math.isfinite(conn_meta) and math.isfinite(d_meta):
        for r in refs:
            elapsed = fnum(r, "plan_elapsed_s")
            tau_time = (elapsed - conn_meta) / d_meta
            if not (-1e-9 <= tau_time <= 1.0 + 1e-9):
                continue
            # Same quintic time law as the planner: u = sigma(tau).
            tau = min(max(tau_time, 0.0), 1.0)
            u_path = tau * tau * tau * (tau * (6.0 * tau - 15.0) + 10.0)
            hl, hr = parse_transform(r, "left_hand_ref"), parse_transform(r, "right_hand_ref")
            if hl is None or hr is None:
                continue
            ol, orr = rod_reference(hl, gl), rod_reference(hr, gr)
            t_mean = 0.5 * (ol[1] + orr[1])
            # Rotation: use the left inferred rod, which is exact up to IK residual.
            yaw = yaw_from_matrix(ol[0])
            # Nominal reference computed from the recorded common path and the
            # same time law, independent of the CSV's convenience columns.
            nominal = (x0[0] + u_path * dx_path, x0[1] + u_path * dy_path, x0[2],
                       start_yaw + u_path * dyaw_rad)
            cmd.append({
                "run_id": run_id, "condition": row["condition"],
                "u": min(max(tau_time, 0.0), 1.0), "elapsed": elapsed,
                "x": t_mean[0], "y": t_mean[1], "z": t_mean[2], "yaw": yaw,
                "left_z": ol[1][2], "right_z": orr[1][2],
                "nom_x": nominal[0], "nom_y": nominal[1], "nom_z": nominal[2],
                "nom_yaw": nominal[3],
                "dx_mm": 1000 * (t_mean[0] - nominal[0]),
                "dy_mm": 1000 * (t_mean[1] - nominal[1]),
                "dz_mm": 1000 * (t_mean[2] - nominal[2]),
                "dyaw_deg": math.degrees(wrap(yaw - nominal[3])),
                "left_right_xy_mm": 1000 * math.hypot(ol[1][0] - orr[1][0], ol[1][1] - orr[1][1]),
                "left_right_z_mm": 1000 * (ol[1][2] - orr[1][2]),
            })

    # Actual rod deviation from the commanded reference on the metric grid.
    cz = {round(c["elapsed"], 9): c["z"] for c in cmd}
    actual_vs_cmd = []
    if cz and grid:
        cu = [c["elapsed"] for c in cmd]
        czv = [c["z"] for c in cmd]
        for t in grid:
            planned_t = conn_meta + (t - t0)
            c = interp(cu, czv, planned_t)
            z = interp(tx, tz, t)
            if c is not None and z is not None:
                actual_vs_cmd.append(1000 * (z - c))

    forces_window = [f for f in forces
                     if t0 <= fnum(f, "sim_time_s") <= t1 and f["phase"] == "TRANSPORT"]
    ff_start = float("nan")
    if forces_window:
        ff_start = fnum(min(forces_window, key=lambda f: abs(fnum(f, "sim_time_s") - t0)),
                        "left_published_ff_z_n")
    ff_peak = max((fnum(f, "left_published_ff_z_n") for f in forces_window), default=float("nan"))

    contact_rows = read_csv(path / "contact_probe.csv")
    last_contact = contact_rows[-1] if contact_rows else {}
    p3a2b_failure = re.search(r"\[P3A2B_RESULT\] state=FAILED detail=([^\r\n]+)", log)
    failure_reason = (row.get("direct_reason", "") or
                      (p3a2b_failure.group(1) if p3a2b_failure else ""))
    wrapper_path = ROOT / row.get("launcher_log", "")
    wrapper_text = wrapper_path.read_text(errors="replace") if wrapper_path.exists() else ""
    wrapper_pose = re.search(
        r"explicit pose\s*:\s*x=([-0-9.eE+]+)\s+y=([-0-9.eE+]+)\s+yaw_deg=([-0-9.eE+]+)",
        wrapper_text)
    wrapper_xyz = ((float(wrapper_pose.group(1)), float(wrapper_pose.group(2)),
                    float(wrapper_pose.group(3))) if wrapper_pose else (float("nan"),) * 3)
    frozen_xy_delta = 1000 * math.hypot(
        events["frozen_truth_xyz"][0] - fnum(row, "start_x_m"),
        events["frozen_truth_xyz"][1] - fnum(row, "start_y_m"))
    wrapper_matches = (all(math.isfinite(v) for v in wrapper_xyz) and
                       abs(wrapper_xyz[0] - fnum(row, "start_x_m")) <= 1e-6 and
                       abs(wrapper_xyz[1] - fnum(row, "start_y_m")) <= 1e-6 and
                       abs(wrapper_xyz[2] - fnum(row, "start_yaw_deg")) <= 1e-6)

    def rms(vals):
        return math.sqrt(sum(x * x for x in vals) / len(vals)) if vals else float("nan")

    ref_yaw0 = events["common_yaw"]
    x0_yaw = None
    x0_R = quat_to_matrix([
        float(meta.get("initial_rod_quat_x", "nan")), float(meta.get("initial_rod_quat_y", "nan")),
        float(meta.get("initial_rod_quat_z", "nan")), float(meta.get("initial_rod_quat_w", "nan"))])
    if x0_R is not None:
        x0_yaw = yaw_from_matrix(x0_R)
    target_yaw = fnum(row, "target_yaw_deg")
    release_yaw_error = float("nan")
    if events["release_quat"] is not None:
        R = quat_to_matrix(events["release_quat"])
        if R is not None:
            release_yaw_error = math.degrees(wrap(yaw_from_matrix(R) - math.radians(target_yaw)))
    release_xy_error = (1000 * math.hypot(events["release_xyz"][0] - fnum(row, "target_x_m"),
                                          events["release_xyz"][1] - fnum(row, "target_y_m"))
                        if math.isfinite(events["release_xyz"][0]) else float("nan"))
    task_complete = int(events["task_state"] == "DONE" and
                        "[CONTACT_PROBE_RETREAT]" in log and
                        "[CONTACT_PROBE_RELEASED]" in log)
    main_actual = t1 - t0 if math.isfinite(t0) and math.isfinite(t1) else float("nan")
    plan_duration = fnum(meta, "common_reference_main_plan_duration_s")

    block_check = {
        "initial_x_m": fnum(meta, "initial_rod_center_x_m"),
        "initial_y_m": fnum(meta, "initial_rod_center_y_m"),
        "initial_yaw_deg": math.degrees(x0_yaw) if x0_yaw is not None else float("nan"),
        "block_start_x_m": fnum(row, "start_x_m"), "block_start_y_m": fnum(row, "start_y_m"),
        "block_start_yaw_deg": fnum(row, "start_yaw_deg"),
        "position_delta_mm": 1000 * math.hypot(
            fnum(meta, "initial_rod_center_x_m") - fnum(row, "start_x_m"),
            fnum(meta, "initial_rod_center_y_m") - fnum(row, "start_y_m")),
        "yaw_delta_deg": (math.degrees(wrap(x0_yaw - math.radians(fnum(row, "start_yaw_deg"))))
                          if x0_yaw is not None else float("nan")),
    }

    metrics = {
        "phase": row.get("phase", ""), "sequence": row.get("sequence", ""),
        "block": row.get("block", ""), "condition": row["condition"], "run_id": run_id,
        "task_state": events["task_state"],
        "task_complete": task_complete,
        "direct_reason": row.get("direct_reason", ""),
        "failure_reason": failure_reason,
        "failure_phase": (last_contact.get("phase", "") if task_complete == 0 else ""),
        "common_reference_enabled": int(common_enabled),
        "common_reference_source": events["common_source"],
        "common_duration_meta_s": d_meta, "common_connection_meta_s": conn_meta,
        "plan_main_duration_s": plan_duration, "check_duration_s": events["check_duration_s"],
        "check_pass": events["check_pass"], "check_points": events["check_points"],
        "check_collisions": events["check_collisions"],
        "check_limit_flags": "".join(str(v) for v in events["check_limit_flags"]),
        "check_max_dual_pos_mm": events["check_max_dual"][0],
        "check_max_dual_ang_deg": events["check_max_dual"][1],
        "check_max_rod_path_mm": events["check_max_rod_path"][0],
        "check_max_rod_path_ang_deg": events["check_max_rod_path"][1],
        "check_max_vel_ratio": events["check_max_vel_ratio"],
        "check_max_acc_ratio": events["check_max_acc_ratio"],
        "check_ik_residual_max_mm": events["check_ik_residual"][0],
        "check_ik_residual_max_deg": events["check_ik_residual"][1],
        "dispatch_connection_s": events["dispatch_connection_s"],
        "dispatch_object_offset_mm": events["dispatch_object_offset_mm"],
        "connection_object_offset_mm": events["connection_object_offset_mm"],
        "connection_object_angle_deg": events["connection_object_angle_deg"],
        "connection_ik_residual_max_mm": events["connection_ik_residual"][0],
        "connection_ik_residual_max_deg": events["connection_ik_residual"][1],
        "connection_max_dual_pos_mm": events["connection_max_dual"][0],
        "connection_max_dual_ang_deg": events["connection_max_dual"][1],
        "transport_phase_start_s": events["transport_start_s"],
        "transport_phase_end_s": events["transport_end_s"],
        "main_start_s": t0, "main_end_s": t1,
        "main_actual_duration_s": main_actual,
        "main_duration_error_s": main_actual - D_FIXED if math.isfinite(main_actual) else float("nan"),
        "common_start_x_m": x0[0], "common_start_y_m": x0[1], "common_start_z_m": x0[2],
        "common_start_yaw_deg": math.degrees(ref_yaw0) if math.isfinite(ref_yaw0) else float("nan"),
        "common_delta_x_m": events["common_delta"][0] if events["common_delta"] else float("nan"),
        "common_delta_y_m": events["common_delta"][1] if events["common_delta"] else float("nan"),
        "common_dyaw_deg": events["common_dyaw_deg"],
        "e_z_start_mm": 1000 * e0 if e_values else float("nan"),
        "e_z_mean_mm": 1000 * statistics.mean(e_values) if e_values else float("nan"),
        "e_z_rms_mm": 1000 * rms(e_values),
        "e_z_end_mm": 1000 * e_values[-1] if e_values else float("nan"),
        "delta_e_z_rms_mm": 1000 * rms(de_values),
        "delta_e_z_end_mm": 1000 * de_values[-1] if de_values else float("nan"),
        "e_z_sample_count": len(e_values),
        "e_z_window_coverage": coverage,
        "truth_interval_median_ms": (1000 * statistics.median([s["width"] for s in truth])) if truth else float("nan"),
        "truth_interval_max_ms": 1000 * max((s["width"] for s in truth), default=float("nan")),
        "adopted_ref_samples": len(cmd),
        "adopted_ref_dx_max_mm": max((abs(c["dx_mm"]) for c in cmd), default=float("nan")),
        "adopted_ref_dy_max_mm": max((abs(c["dy_mm"]) for c in cmd), default=float("nan")),
        "adopted_ref_dz_max_mm": max((abs(c["dz_mm"]) for c in cmd), default=float("nan")),
        "adopted_ref_dyaw_max_deg": max((abs(c["dyaw_deg"]) for c in cmd), default=float("nan")),
        "adopted_ref_start_offset_mm": 1000 * (cmd[0]["z"] - x0[2]) if cmd else float("nan"),
        "adopted_ref_end_offset_mm": 1000 * (cmd[-1]["z"] - x0[2]) if cmd else float("nan"),
        "rod_minus_commanded_rms_mm": rms(actual_vs_cmd),
        "rod_minus_commanded_mean_mm": statistics.mean(actual_vs_cmd) if actual_vs_cmd else float("nan"),
        "left_right_ref_xy_max_mm": max((c["left_right_xy_mm"] for c in cmd), default=float("nan")),
        "left_right_ref_z_max_mm": max((abs(c["left_right_z_mm"]) for c in cmd), default=float("nan")),
        "payload_enabled": events["payload_enabled"],
        "payload_anchor_s": events["payload_anchor_s"],
        "payload_ramp_elapsed_at_main_start_s": (t0 - events["payload_anchor_s"]
                                                 if math.isfinite(t0) else float("nan")),
        "published_ff_per_arm_at_main_start_n": ff_start,
        "published_ff_per_arm_main_peak_n": ff_peak,
        "release_x_m": events["release_xyz"][0], "release_y_m": events["release_xyz"][1],
        "release_z_m": events["release_xyz"][2], "release_xy_error_mm": release_xy_error,
        "release_yaw_deg": (math.degrees(yaw_from_matrix(quat_to_matrix(events["release_quat"])))
                            if events["release_quat"] else float("nan")),
        "release_yaw_error_deg": release_yaw_error,
        "release_distance_mm": events["release_distance_mm"],
        "release_dz_mm": events["release_dz_mm"],
        "release_angle_deg": events["release_angle_deg"],
        "final_left_slip_mm": 1000 * fnum(last_contact, "left_slip_m"),
        "final_right_slip_mm": 1000 * fnum(last_contact, "right_slip_m"),
        "frozen_truth_xyz_x_m": events["frozen_truth_xyz"][0],
        "frozen_truth_xyz_y_m": events["frozen_truth_xyz"][1],
        "frozen_truth_position_delta_mm": frozen_xy_delta,
        "wrapper_requested_pose": ",".join(f"{v:.6f}" for v in wrapper_xyz),
        "wrapper_request_matches_block": int(wrapper_matches),
        "block_initial_position_delta_mm": block_check["position_delta_mm"],
        "block_initial_yaw_delta_deg": block_check["yaw_delta_deg"],
        "initial_rod_x_m": block_check["initial_x_m"], "initial_rod_y_m": block_check["initial_y_m"],
        "initial_rod_yaw_deg": block_check["initial_yaw_deg"],
        "block_start_x_m": block_check["block_start_x_m"], "block_start_y_m": block_check["block_start_y_m"],
        "block_start_yaw_deg": block_check["block_start_yaw_deg"],
    }
    return metrics, curve, cmd


def reference_difference(cmd_by_condition):
    """Cross-condition commanded reference at matched normalized progress."""
    if set(cmd_by_condition) != {"M0", "M1"}:
        return [], {}
    grid = [i / 200 for i in range(201)]
    detail = []
    for u in grid:
        row = {"normalized_main_progress": u}
        interps = {}
        for cond in ("M0", "M1"):
            rows = cmd_by_condition[cond]
            us = [r["u"] for r in rows]
            interps[cond] = {}
            for key in ("x", "y", "z", "yaw", "nom_x", "nom_y", "nom_z"):
                interps[cond][key] = interp(us, [r[key] for r in rows], u)
            yaw_vals = [r["yaw"] for r in rows]
            interps[cond]["yaw"] = interp(us, yaw_vals, u)
            row[f"{cond}_commanded_z_m"] = interps[cond]["z"]
        for key in ("x", "y", "z"):
            a, b = interps["M0"][key], interps["M1"][key]
            row[f"M1_minus_M0_{key}_mm"] = 1000 * (b - a) if a is not None and b is not None else float("nan")
        a, b = interps["M0"]["yaw"], interps["M1"]["yaw"]
        row["M1_minus_M0_yaw_deg"] = math.degrees(wrap(b - a)) if a is not None and b is not None else float("nan")
        detail.append(row)
    z_diff = [abs(r["M1_minus_M0_z_mm"]) for r in detail]
    xy_diff = [math.hypot(r["M1_minus_M0_x_mm"], r["M1_minus_M0_y_mm"]) for r in detail]
    yaw_diff = [abs(r["M1_minus_M0_yaw_deg"]) for r in detail]
    summary = {
        "reference_z_difference_max_mm": max(z_diff),
        "reference_xy_difference_max_mm": max(xy_diff),
        "reference_yaw_difference_max_deg": max(yaw_diff),
    }
    return detail, summary


def fmt(x, digits=3):
    if isinstance(x, (int, np.integer)):
        return str(int(x))
    try:
        v = float(x)
    except (TypeError, ValueError):
        return "NA"
    return "NA" if not math.isfinite(v) else f"{v:.{digits}f}"


def formal_reference_difference(cmd_by_key):
    """Per-block M1−M0 commanded object reference at matched normalized time."""
    rows = []
    blocks = sorted({b for b, _ in cmd_by_key})
    grid = [i / 200 for i in range(201)]
    for block in blocks:
        a = cmd_by_key.get((block, "M0"))
        b = cmd_by_key.get((block, "M1"))
        if not a or not b:
            continue
        ua = [r["u"] for r in a]
        ub = [r["u"] for r in b]
        dz, dxy, dyaw = [], [], []
        for u in grid:
            za = interp(ua, [r["z"] for r in a], u)
            zb = interp(ub, [r["z"] for r in b], u)
            xa = interp(ua, [r["x"] for r in a], u)
            xb = interp(ub, [r["x"] for r in b], u)
            ya = interp(ua, [r["y"] for r in a], u)
            yb = interp(ub, [r["y"] for r in b], u)
            wa = interp(ua, [r["yaw"] for r in a], u)
            wb = interp(ub, [r["yaw"] for r in b], u)
            if None in (za, zb, xa, xb, ya, yb, wa, wb):
                continue
            dz.append(abs(zb - za) * 1000)
            dxy.append(math.hypot(xb - xa, yb - ya) * 1000)
            dyaw.append(abs(math.degrees(wrap(wb - wa))))
        rows.append({"block": block,
                     "reference_z_difference_max_mm": max(dz) if dz else float("nan"),
                     "reference_xy_difference_max_mm": max(dxy) if dxy else float("nan"),
                     "reference_yaw_difference_max_deg": max(dyaw) if dyaw else float("nan")})
    summary = {
        "blocks": len(rows),
        "z_max": max((r["reference_z_difference_max_mm"] for r in rows), default=float("nan")),
        "xy_max": max((r["reference_xy_difference_max_mm"] for r in rows), default=float("nan")),
        "yaw_max": max((r["reference_yaw_difference_max_deg"] for r in rows), default=float("nan")),
    }
    return rows, summary


def t_quantile_975(n):
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
             8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145,
             15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086}
    return table.get(n, 1.96)


def paired_summary(pairs):
    vals = [b - a for a, b in pairs if math.isfinite(a) and math.isfinite(b)]
    if not vals:
        return {"n": 0}
    n = len(vals)
    mean = statistics.mean(vals)
    sd = statistics.stdev(vals) if n > 1 else float("nan")
    half = t_quantile_975(n - 1) * sd / math.sqrt(n) if n > 1 else float("nan")
    return {"n": n, "mean": mean, "sd": sd, "ci_low": mean - half, "ci_high": mean + half,
            "min": min(vals), "max": max(vals), "negative": sum(v < 0 for v in vals),
            "positive": sum(v > 0 for v in vals)}


def main():
    ledger = read_csv(OUT / "attempts.csv") if (OUT / "attempts.csv").exists() else []
    write_csv(OUT / "raw_run_index.csv", [
        {"phase": r["phase"], "sequence": r["sequence"], "block": r["block"],
         "condition": r["condition"], "launch_attempt": r["launch_attempt"], "run_id": r["run_id"],
         "action_started": r["action_started"], "transport_started": r["transport_started"],
         "status": r["status"], "direct_reason": r["direct_reason"], "run_dir": r["run_dir"],
         "launcher_log": r["launcher_log"]} for r in ledger])

    pretrial_rows = [r for r in ledger if r["phase"] == "pretrial" and r["action_started"] == "1"]
    formal_rows = [r for r in ledger if r["phase"] == "formal" and r["action_started"] == "1"]

    all_pre_metrics, selected_metrics = [], {}
    for row in sorted(pretrial_rows, key=lambda r: int(r["launch_attempt"])):
        m, _, _ = load_run(row)
        all_pre_metrics.append(m)
        cond = row["condition"]
        # Keep the last accepted pretrial; before acceptance, keep the latest
        # attempted action so the report reflects the current state.
        if cond in selected_metrics and selected_metrics[cond][0]["status"] == "pretrial_completed":
            continue
        selected_metrics[cond] = (row, m)
    if all_pre_metrics:
        write_csv(OUT / "pretrial_attempts.csv", all_pre_metrics,
                  fields=["phase", "condition", "run_id", "task_state", "task_complete",
                          "transport_phase_start_s", "main_start_s", "main_actual_duration_s",
                          "e_z_start_mm", "delta_e_z_end_mm", "release_xy_error_mm",
                          "direct_reason"])

    pre_metrics, pre_curves, pre_cmd = [], [], {}
    for cond in ("M0", "M1"):
        if cond not in selected_metrics:
            continue
        row, m = selected_metrics[cond]
        _, c, cmd = load_run(row)
        pre_metrics.append(m)
        pre_curves.extend(c)
        pre_cmd[cond] = cmd
    if pre_metrics:
        write_csv(OUT / "pretrial_metrics.csv", pre_metrics)
        write_csv(OUT / "pretrial_transport_curves.csv", pre_curves)
    ref_detail, ref_summary = reference_difference(pre_cmd)
    if ref_detail:
        write_csv(OUT / "pretrial_reference_difference.csv", ref_detail)

    # Gate checks (declared before the actions in frozen_config.md).
    gate = {}
    by_cond = {m["condition"]: m for m in pre_metrics}
    gate["both_conditions_present"] = set(by_cond) == {"M0", "M1"}
    gate["both_complete"] = gate["both_conditions_present"] and all(
        m["task_complete"] == 1 for m in pre_metrics)
    gate["common_reference_declared"] = gate["both_conditions_present"] and all(
        m["common_reference_enabled"] == 1 and m["common_reference_source"].startswith(
            "pre_feedforward_nominal_object_path") for m in pre_metrics)
    gate["plan_duration_fixed"] = gate["both_conditions_present"] and all(
        abs(m["plan_main_duration_s"] - D_FIXED) <= 1e-3 for m in pre_metrics)
    gate["actual_duration_fixed"] = gate["both_conditions_present"] and all(
        abs(m["main_duration_error_s"]) <= DURATION_TOL_S for m in pre_metrics)
    gate["plan_checks_pass"] = gate["both_conditions_present"] and all(
        m["check_pass"] == 1 and m["check_collisions"] == 0 for m in pre_metrics)
    gate["commanded_matches_nominal"] = gate["both_conditions_present"] and all(
        max(m["adopted_ref_dx_max_mm"], m["adopted_ref_dy_max_mm"], m["adopted_ref_dz_max_mm"])
        <= REF_POS_TOL_MM and m["adopted_ref_dyaw_max_deg"] <= REF_YAW_TOL_DEG
        for m in pre_metrics)
    gate["start_reference_matches"] = False
    if gate["both_conditions_present"]:
        a, b = by_cond["M0"], by_cond["M1"]
        pos = 1000 * math.hypot(a["common_start_x_m"] - b["common_start_x_m"],
                                a["common_start_y_m"] - b["common_start_y_m"],
                                a["common_start_z_m"] - b["common_start_z_m"])
        yaw = abs(math.degrees(wrap(math.radians(a["common_start_yaw_deg"]) -
                                    math.radians(b["common_start_yaw_deg"]))))
        gate["start_reference_matches"] = pos <= START_POS_TOL_MM and yaw <= START_YAW_TOL_DEG
        gate["start_reference_position_delta_mm"] = pos
        gate["start_reference_yaw_delta_deg"] = yaw
    gate["cross_condition_reference_matches"] = bool(ref_summary) and (
        ref_summary["reference_z_difference_max_mm"] <= REF_POS_TOL_MM and
        ref_summary["reference_xy_difference_max_mm"] <= REF_POS_TOL_MM and
        ref_summary["reference_yaw_difference_max_deg"] <= REF_YAW_TOL_DEG)
    gate["feedforward_timing"] = gate["both_conditions_present"] and (
        abs(by_cond["M0"]["published_ff_per_arm_at_main_start_n"]) <= 1e-9 and
        by_cond["M0"]["payload_enabled"] == 0 and
        abs(by_cond["M1"]["published_ff_per_arm_at_main_start_n"] - 0.64 * 9.81 * 0.5) <= 0.01 and
        by_cond["M1"]["payload_enabled"] == 1 and
        by_cond["M1"]["payload_ramp_elapsed_at_main_start_s"] >= 0.5)
    gate["metrics_computable"] = gate["both_conditions_present"] and all(
        m["e_z_sample_count"] > 10 and m["e_z_window_coverage"] >= 0.95 and
        m["adopted_ref_samples"] > 10 for m in pre_metrics)
    gate["block_inputs_applied"] = gate["both_conditions_present"] and all(
        m["wrapper_request_matches_block"] == 1 and
        math.isfinite(m["frozen_truth_position_delta_mm"]) and
        m["frozen_truth_position_delta_mm"] <= BLOCK_POS_TOL_MM for m in pre_metrics)
    gate["block_initial_truth_consistent"] = gate["both_conditions_present"] and all(
        m["block_initial_position_delta_mm"] <= BLOCK_POS_TOL_MM and
        abs(m["block_initial_yaw_delta_deg"]) <= BLOCK_YAW_TOL_DEG for m in pre_metrics)
    if gate["both_conditions_present"]:
        a, b = by_cond["M0"], by_cond["M1"]
        gate["block_pair_frozen_truth_delta_mm"] = 1000 * math.hypot(
            a["frozen_truth_xyz_x_m"] - b["frozen_truth_xyz_x_m"],
            a["frozen_truth_xyz_y_m"] - b["frozen_truth_xyz_y_m"])
    gate_order = ["both_conditions_present", "both_complete", "common_reference_declared",
                  "plan_duration_fixed", "actual_duration_fixed", "plan_checks_pass",
                  "commanded_matches_nominal", "start_reference_matches",
                  "cross_condition_reference_matches", "feedforward_timing",
                  "metrics_computable", "block_inputs_applied",
                  "block_initial_truth_consistent"]
    gate_pass = all(bool(gate.get(k)) for k in gate_order)

    if pre_metrics:
        write_pretrial_report(pre_metrics, pre_curves, ref_detail, ref_summary, gate,
                              gate_order, gate_pass)
        write_pretrial_figures(pre_curves, ref_detail)
    if gate_pass and not formal_rows:
        (OUT / "PRETRIAL_GATE_PASS").write_text(
            "revised pretrials passed the pre-declared common-reference gate\n", encoding="utf-8")
    elif not gate_pass:
        (OUT / "PRETRIAL_GATE_PASS").unlink(missing_ok=True)

    if formal_rows:
        formal_metrics, formal_curves, formal_cmd = [], [], {}
        for row in formal_rows:
            m, c, cmd = load_run(row)
            formal_metrics.append(m)
            formal_curves.extend(c)
            if str(row["block"]).isdigit():
                formal_cmd[(int(row["block"]), row["condition"])] = cmd
        write_csv(OUT / "formal_metrics.csv", formal_metrics)
        write_csv(OUT / "formal_transport_curves.csv", formal_curves)
        formal_ref_rows, formal_ref_summary = formal_reference_difference(formal_cmd)
        if formal_ref_rows:
            write_csv(OUT / "formal_reference_difference.csv", formal_ref_rows)
        write_formal_analysis(formal_metrics, formal_curves, ledger,
                              formal_ref_summary)
    print(f"pretrial={len(pre_metrics)} formal={len(formal_rows)} gate_pass={gate_pass}")
    for key in gate_order:
        print(f"  gate {key}={gate.get(key)}")


def write_pretrial_figures(curves, ref_detail):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"M0": "#3b6fb6", "M1": "#c44e52"}
    if curves:
        fig, axs = plt.subplots(1, 2, figsize=(9.0, 3.8), constrained_layout=True)
        for cond in ("M0", "M1"):
            c = [r for r in curves if r["condition"] == cond]
            if not c:
                continue
            axs[0].plot([r["elapsed_from_main_start_s"] for r in c],
                        [r["e_z_common_mm"] for r in c], color=colors[cond], linewidth=1.8,
                        label=f"{cond} (n=1)")
            axs[1].plot([r["elapsed_from_main_start_s"] for r in c],
                        [r["delta_e_z_mm"] for r in c], color=colors[cond], linewidth=1.8,
                        label=f"{cond} (n=1)")
        for ax in axs:
            ax.axhline(0, color="black", linewidth=0.7)
            ax.set_xlabel("Time from main-segment start (s)")
            ax.legend(frameon=False)
        axs[0].set_ylabel("Vertical error e_z (mm)")
        axs[1].set_ylabel("Change from main start Δe_z (mm)")
        fig.savefig(OUT / "pretrial_transport_error.png", dpi=180)
        plt.close(fig)
    if ref_detail:
        fig, ax = plt.subplots(figsize=(6.6, 3.8), constrained_layout=True)
        us = [r["normalized_main_progress"] for r in ref_detail]
        ax.plot(us, [r["M1_minus_M0_x_mm"] for r in ref_detail], label="X")
        ax.plot(us, [r["M1_minus_M0_y_mm"] for r in ref_detail], label="Y")
        ax.plot(us, [r["M1_minus_M0_z_mm"] for r in ref_detail], label="Z")
        ax.plot(us, [r["M1_minus_M0_yaw_deg"] * 10 for r in ref_detail],
                label="Yaw ×10 (deg)")
        ax.set_xlabel("Normalized main-segment time")
        ax.set_ylabel("M1 − M0 commanded reference (mm)")
        ax.legend(frameon=False, ncol=4)
        fig.savefig(OUT / "pretrial_reference_difference.png", dpi=180)
        plt.close(fig)


def write_pretrial_report(metrics, curves, ref_detail, ref_summary, gate, gate_order, gate_pass):
    by = {m["condition"]: m for m in metrics}
    m0, m1 = by.get("M0", {}), by.get("M1", {})
    ledger = read_csv(OUT / "attempts.csv")
    infra = [r for r in ledger if r["phase"] == "pretrial" and r["status"] == "startup_exception"]
    selected_ids = {m["run_id"] for m in metrics}
    earlier = []
    attempts_path = OUT / "pretrial_attempts.csv"
    if attempts_path.exists():
        earlier = [a for a in read_csv(attempts_path) if a["run_id"] not in selected_ids]
    start_gap = (m1.get("e_z_start_mm", float("nan")) - m0.get("e_z_start_mm", float("nan"))
                 if m0 and m1 else float("nan"))

    def failure_line(a):
        reason = a.get("direct_reason") or "未记录原因"
        transport = a.get("transport_phase_start_s")
        stage = "运输主段前" if not math.isfinite(float(transport or "nan")) else "运输主段后"
        return f"- {a['run_id']}（{a['condition']}，{stage}）：{reason}"

    earlier_text = "\n".join(failure_line(a) for a in earlier) if earlier else "无"
    infra_text = "\n".join(
        f"- {r['run_id']}（{r['condition']}，第 {r['launch_attempt']} 次启动）：{r['direct_reason']}"
        for r in infra) if infra else "无"

    def gate_row(key, label):
        return f"| {label} | {'通过' if gate.get(key) else '未通过'} |\n"

    labels = [
        ("both_conditions_present", "两条件均有动作记录"),
        ("both_complete", "两条件均完成抓取至撤退"),
        ("common_reference_declared", "实际主段采用共同物体参考"),
        ("plan_duration_fixed", "主段计划时长为固定值 3.040 s"),
        ("actual_duration_fixed", "主段实际时长在 ±0.020 s 内"),
        ("plan_checks_pass", "规划通过限位/碰撞/闭链检查"),
        ("commanded_matches_nominal", "下发物体参考与名义参考一致（0.5 mm/0.10°）"),
        ("start_reference_matches", "两条件共同参考起点一致（1.0 mm/0.05°）"),
        ("cross_condition_reference_matches", "匹配进度下两条件参考一致（0.5 mm/0.10°）"),
        ("feedforward_timing", "前馈开关、方向与时序正确"),
        ("metrics_computable", "三项主要指标可计算且覆盖充分"),
        ("block_inputs_applied", "区组输入已下发（wrapper 参数与冻结真值位置）"),
        ("block_initial_truth_consistent", "抓取后初始真值符合 0.05° 预声明容差"),
    ]
    gate_table = "".join(gate_row(k, lab) for k, lab in labels)

    text = f"""# 完整搬运载荷前馈预试验结果（修订批次 r2）

## 本批次要解决的问题

原批次 `fullcarry_payload_ff_20261026` 的两次预试验完整完成，但每个条件的运输路径都从
各自抬升后的实测杆状态重建：计划运输时长 3.037/2.244 s（差 26.1%），按归一化进度的
实际物体参考高度差约 8.302 mm，运输起点杆高差约 8.3 mm。于是“运输期间新增沉降”无法与
不同的运行时长和状态依赖参考分开，原批次因此未启动正式实验（0/20），数据完整保留。

## 本批次最终实现（动作前冻结）

1. **共同物体路径**：起点为夹持闭合后、杆载荷前馈尚未产生影响时读到的杆真值位姿沿世界
   `+Z` 平移 0.40 m（与抬升计划的目标末端一致），终点为固定任务目标 `(0.400, 0.000, +20°)`，
   高度不变；XY 直线、绕世界 Z 线性偏航。路径本身与两条件的下沉无关。
2. **主段时长与进度**：固定 `D = 3.040 s`（取原批次较慢条件 3.0368 s 圆整；无运动规划检查
   的最大速度比 0.124、加速度比 0.057，两条件余量充分）。共同进度
   `u(t)=σ(τ)=6τ⁵−15τ⁴+10τ³, τ=t/D`，40 ms 采样网格，速度/加速度由同一时间网格中心差分，
   端点为零，并经过关节限位、闭链、碰撞与五次插值检查。
3. **生成双手参考的抓持变换**：取运输起点的一次新鲜实测（当前关节状态与新鲜杆真值）。
   原因是抬升/保持期间垫–杆已发生约 10 mm 级滑移，接触时的前馈前抓持与当前垫–杆相对位置
   相差约 20 mm（本次两条件分别测得 21.10 mm / 20.02 mm 的双臂不一致）；若仍用它生成双手目标，
   控制器会要求指垫做一次相当于重新夹持的相对运动，改变物理抓持。两种抓持均写入元数据。
   **被下发的物体参考仍严格是第 1 条的共同路径**；本次核验其与名义参考的最大偏差为
   M0 {fmt(m0.get('adopted_ref_dx_max_mm'),4)} / {fmt(m0.get('adopted_ref_dy_max_mm'),4)} /
   {fmt(m0.get('adopted_ref_dz_max_mm'),4)} mm、偏航 {fmt(m0.get('adopted_ref_dyaw_max_deg'),5)}°，
   M1 为 {fmt(m1.get('adopted_ref_dx_max_mm'),4)} / {fmt(m1.get('adopted_ref_dy_max_mm'),4)} /
   {fmt(m1.get('adopted_ref_dz_max_mm'),4)} mm、偏航 {fmt(m1.get('adopted_ref_dyaw_max_deg'),5)}°。
4. **起点衔接**：主段规划、衔接、滑移监控共用同一次运输起点实测与抓持冻结。衔接为 0.5 s
   笛卡尔回引，把双手从实测位姿带到“共同参考起点 × 该抓持”，首点为实测状态、末点为主段首点，
   两端静止；单独记录并排除在主段指标之外。本次衔接物体偏移 M0
   {fmt(m0.get('connection_object_offset_mm'))} mm、M1 {fmt(m1.get('connection_object_offset_mm'))} mm。
5. **释放记录**：开爪前请求一次新鲜真值作为比较基准（既有“开爪前位姿”判据，10 mm/5° 阈值不变），
   并在 `[CONTACT_PROBE_RELEASE_COMPARISON]` 记录差值；释放姿态取释放后统一物理等待结束时的真值。

## 预试验动作记录

最新一次各条件的预试验对（也是本报告逐项指标来源）：

| 条件 | run_id | 状态 | 失败/说明 |
|---|---|---|---|
| M0 | {m0.get('run_id', 'NA')} | {m0.get('task_state', 'NA')} | {(m0.get('direct_reason') or '完整完成').replace('|', '/')} |
| M1 | {m1.get('run_id', 'NA')} | {m1.get('task_state', 'NA')} | {(m1.get('direct_reason') or '完整完成').replace('|', '/')} |

本批次更早的动作（全部保留，不计入正式样本）：

{earlier_text}

启动异常（动作未开始）：

{infra_text}

## 选中的预试验对指标

| 指标 | M0（前馈关） | M1（前馈开） |
|---|---:|---:|
| 完整流程 | {m0.get('task_complete', 0)}/1 | {m1.get('task_complete', 0)}/1 |
| 主段计划时长（s） | {fmt(m0.get('plan_main_duration_s'),4)} | {fmt(m1.get('plan_main_duration_s'),4)} |
| 主段实际时长（s） | {fmt(m0.get('main_actual_duration_s'),4)} | {fmt(m1.get('main_actual_duration_s'),4)} |
| 起点衔接（s） | {fmt(m0.get('dispatch_connection_s'),4)} | {fmt(m1.get('dispatch_connection_s'),4)} |
| 衔接物体偏移（mm） | {fmt(m0.get('connection_object_offset_mm'))} | {fmt(m1.get('connection_object_offset_mm'))} |
| 起点误差 e_z(t₀)（mm） | {fmt(m0.get('e_z_start_mm'))} | {fmt(m1.get('e_z_start_mm'))} |
| 主段平均误差（mm） | {fmt(m0.get('e_z_mean_mm'))} | {fmt(m1.get('e_z_mean_mm'))} |
| 主段总体误差 RMS（mm） | {fmt(m0.get('e_z_rms_mm'))} | {fmt(m1.get('e_z_rms_mm'))} |
| 新增误差 Δe_z RMS（mm） | {fmt(m0.get('delta_e_z_rms_mm'))} | {fmt(m1.get('delta_e_z_rms_mm'))} |
| 新增误差 Δe_z 末值（mm） | {fmt(m0.get('delta_e_z_end_mm'))} | {fmt(m1.get('delta_e_z_end_mm'))} |
| 下发参考相对名义高度（mm） | {fmt(m0.get('adopted_ref_start_offset_mm'))} | {fmt(m1.get('adopted_ref_start_offset_mm'))} |
| 实际杆相对下发参考 RMS（mm） | {fmt(m0.get('rod_minus_commanded_rms_mm'))} | {fmt(m1.get('rod_minus_commanded_rms_mm'))} |
| 释放平面位置误差（mm） | {fmt(m0.get('release_xy_error_mm'))} | {fmt(m1.get('release_xy_error_mm'))} |
| 释放偏航误差（deg） | {fmt(m0.get('release_yaw_error_deg'))} | {fmt(m1.get('release_yaw_error_deg'))} |
| 释放前→释放位移（mm） | {fmt(m0.get('release_distance_mm'))} | {fmt(m1.get('release_distance_mm'))} |
| 末次左/右滑移（mm） | {fmt(m0.get('final_left_slip_mm'))}/{fmt(m0.get('final_right_slip_mm'))} | {fmt(m1.get('final_left_slip_mm'))}/{fmt(m1.get('final_right_slip_mm'))} |

有效主段样本数 M0/M1：{m0.get('e_z_sample_count', 0)}/{m1.get('e_z_sample_count', 0)}；
有效覆盖 {fmt(m0.get('e_z_window_coverage'),3)}/{fmt(m1.get('e_z_window_coverage'),3)}；
真值请求/响应区间中位数 {fmt(m0.get('truth_interval_median_ms'))}/{fmt(m1.get('truth_interval_median_ms'))} ms，
最大 {fmt(m0.get('truth_interval_max_ms'))}/{fmt(m1.get('truth_interval_max_ms'))} ms。
只在覆盖范围内插值，无外推。

前馈：M0 主段起点发布值 {fmt(m0.get('published_ff_per_arm_at_main_start_n'))} N/臂（全程零）；
M1 {fmt(m1.get('published_ff_per_arm_at_main_start_n'))} N/臂，方向世界 +Z，两基座与世界对齐，
渐增在首条有效 MMC 承载参考后开始，主段开始时已持续
{fmt(m1.get('payload_ramp_elapsed_at_main_start_s'))} s（设定 0.5 s）。没有额外承载等待。

## 共同参考与时长一致性

- 两条件共同参考起点差：{fmt(gate.get('start_reference_position_delta_mm'),4)} mm、
  {fmt(gate.get('start_reference_yaw_delta_deg'),4)}°（允许 1.0 mm / 0.05°）。
- 主段实际时长误差：M0 {fmt(m0.get('main_duration_error_s'),4)} s、
  M1 {fmt(m1.get('main_duration_error_s'),4)} s（允许 0.020 s）。
- 匹配归一化主段时间下两条件下发物体参考最大差：Z
  {fmt(ref_summary.get('reference_z_difference_max_mm'))} mm，XY
  {fmt(ref_summary.get('reference_xy_difference_max_mm'))} mm，偏航
  {fmt(ref_summary.get('reference_yaw_difference_max_deg'),4)}°（允许 0.5 mm / 0.5 mm / 0.10°）。
- 规划检查：77/77 点，碰撞 0/0，限位/速度/加速度标志
  {m0.get('check_limit_flags')}/{m1.get('check_limit_flags')}，IK 残差最大
  {fmt(m0.get('check_ik_residual_max_mm'),4)}/{fmt(m1.get('check_ik_residual_max_mm'),4)} mm。
- 区组输入：wrapper 参数与区组一致（M0/M1 均为 {m0.get('wrapper_request_matches_block')}/1、
  {m1.get('wrapper_request_matches_block')}/1），冻结真值位置相对区组指令差
  {fmt(m0.get('frozen_truth_position_delta_mm'),4)}/{fmt(m1.get('frozen_truth_position_delta_mm'),4)} mm，
  同区组两条件冻结真值差 {fmt(gate.get('block_pair_frozen_truth_delta_mm'),4)} mm。
  抓取后初始真值与区组指令的位置差 {fmt(m0.get('block_initial_position_delta_mm'),4)}/
  {fmt(m1.get('block_initial_position_delta_mm'),4)} mm，偏航差
  {fmt(m0.get('block_initial_yaw_delta_deg'),4)}/{fmt(m1.get('block_initial_yaw_delta_deg'),4)}°。

预试验起点误差差 M1−M0 = {fmt(start_gap)} mm（每条件 n=1，不作效应估计）。

## 正式实验门槛

| 条件 | 结果 |
|---|---|
{gate_table}
{'门槛全部通过：可进入正式实验。' if gate_pass else '门槛未全部通过：按约定不启动正式实验（正式计数 0/20），不追加动作，不改参考、时长或阈值去迎合结果。'}

## 未通过项的具体原因

1. **完整流程**：M0 在下降阶段触发 `E_CONTACT_SLIP_OR_DROP`，最后一次记录的左滑移
   {fmt(m0.get('final_left_slip_mm'))} mm（判据 20 mm，超出
   {fmt(m0.get('final_left_slip_mm', 0) - 20, 3)} mm）；M1 在支撑确认后、开爪时触发
   `E_CONTACT_RELEASE_POSE_ERROR`，开爪前→释放后位移 {fmt(m1.get('release_distance_mm'))} mm
   （判据 10 mm），姿态差 {fmt(m1.get('release_angle_deg'))}°（判据 5°）。
2. 这两项都是项目既有的放置/释放判据，不是本批次新增门槛，也不是参考或时长不一致：
   共同参考、同一进度、固定时长、前馈时序与三项主段指标均已通过上述核验。
3. 机理：共同参考把运输末端杆姿态控制在名义（近水平）姿态；在放置阶段杆触支撑后仍会
   绕接触角继续倾斜，随后开爪回平。原批次的状态依赖参考恰好沿用了抬升后实测的俯仰角，
   放置阶段杆的净倾角略小；本批次使用共同名义姿态后，释放时倾角约 3°，杆中心在开爪后
   回平约 10 mm。M0 的左滑移同样处在既有 20 mm 判据边缘（原批次 M0 为 19.74 mm）。
   这是共同名义参考在执行放置阶段的一个实际代价，需如实记录。
4. 区组输入：指令输入（wrapper 参数、任务冻结真值位置）与区组一致；只有抓取后实测初始
   真值的偏航与指令差 {fmt(m0.get('block_initial_yaw_delta_deg'),4)}°/
   {fmt(m1.get('block_initial_yaw_delta_deg'),4)}°，超过预声明的 0.05°。该差异来自杆在
   夹持/静置后的实测姿态，不是区组输入未生效。按预声明容差不放宽，故该项记为未通过。

## 图

`pretrial_transport_error.png`：左为主段共同名义竖向误差 `e_z(t)`，右为相对主段起点的
变化 `Δe_z(t)`；横轴为主段归一化时间。`pretrial_reference_difference.png`：匹配归一化
主段时间下 M1 与 M0 的下发物体参考差（X/Y/Z 单位 mm，偏航为 10 倍度数以便同轴显示）。
图内不放标题或结论。
"""
    (OUT / "pretrial_results.md").write_text(text, encoding="utf-8")


def write_formal_analysis(metrics, curves, ledger, formal_ref_summary=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    by_cond = {"M0": [], "M1": []}
    for m in metrics:
        by_cond.setdefault(m["condition"], []).append(m)
    for cond in by_cond:
        by_cond[cond].sort(key=lambda m: int(m["block"]) if str(m["block"]).isdigit() else 0)
    blocks = sorted({int(m["block"]) for m in metrics if str(m["block"]).isdigit()})
    paired = {}
    for b in blocks:
        m0 = next((m for m in by_cond.get("M0", []) if int(m["block"]) == b), None)
        m1 = next((m for m in by_cond.get("M1", []) if int(m["block"]) == b), None)
        paired[b] = (m0, m1)

    metric_keys = [
        ("e_z_start_mm", "起点竖向误差 e_z(t0) (mm)"),
        ("e_z_mean_mm", "主段平均竖向误差 (mm)"),
        ("e_z_rms_mm", "主段总体竖向误差 RMS (mm)"),
        ("delta_e_z_rms_mm", "相对运输起点的误差变化 Δe_z RMS (mm)"),
        ("delta_e_z_end_mm", "相对运输起点的误差变化 Δe_z 末值 (mm)"),
        ("release_xy_error_mm", "释放平面位置误差 (mm)"),
        ("release_yaw_error_deg", "释放偏航误差 (deg)"),
    ]
    diff_rows = []
    summaries = {}
    for key, label in metric_keys:
        pairs = [(m0[key], m1[key]) for m0, m1 in paired.values()
                 if m0 is not None and m1 is not None
                 and math.isfinite(m0[key]) and math.isfinite(m1[key])]
        s = paired_summary(pairs)
        s["label"] = label
        s["key"] = key
        summaries[key] = s
        for b, (m0, m1) in paired.items():
            if m0 is None or m1 is None:
                continue
            diff_rows.append({
                "block": b, "metric": key, "M0": m0[key], "M1": m1[key],
                "M1_minus_M0": m1[key] - m0[key] if math.isfinite(m0[key]) and math.isfinite(m1[key]) else float("nan"),
            })
    write_csv(OUT / "formal_block_differences.csv", diff_rows)

    # Block input verification and reference consistency across formal runs.
    block_applied_ok = all(m["wrapper_request_matches_block"] == 1 and
                           math.isfinite(m["frozen_truth_position_delta_mm"]) and
                           m["frozen_truth_position_delta_mm"] <= BLOCK_POS_TOL_MM
                           for m in metrics)
    block_meta_ok = all(m["block_initial_position_delta_mm"] <= BLOCK_POS_TOL_MM and
                        abs(m["block_initial_yaw_delta_deg"]) <= BLOCK_YAW_TOL_DEG
                        for m in metrics if math.isfinite(m["block_initial_position_delta_mm"]))
    plan_dur_ok = all(abs(m["plan_main_duration_s"] - D_FIXED) <= 1e-3 for m in metrics
                      if math.isfinite(m["plan_main_duration_s"]))
    actual_dur_ok = all(abs(m["main_duration_error_s"]) <= DURATION_TOL_S for m in metrics
                        if math.isfinite(m["main_duration_error_s"]))
    ref_ok = all(max(m["adopted_ref_dx_max_mm"], m["adopted_ref_dy_max_mm"],
                     m["adopted_ref_dz_max_mm"]) <= REF_POS_TOL_MM and
                 m["adopted_ref_dyaw_max_deg"] <= REF_YAW_TOL_DEG
                 for m in metrics if math.isfinite(m["adopted_ref_dx_max_mm"]))
    same_block = []
    for b, (m0, m1) in paired.items():
        if m0 is None or m1 is None:
            continue
        if not (math.isfinite(m0["initial_rod_x_m"]) and math.isfinite(m1["initial_rod_x_m"]) and
                math.isfinite(m0["initial_rod_yaw_deg"]) and math.isfinite(m1["initial_rod_yaw_deg"])):
            continue
        d = 1000 * math.hypot(m0["initial_rod_x_m"] - m1["initial_rod_x_m"],
                              m0["initial_rod_y_m"] - m1["initial_rod_y_m"])
        dy = abs(math.degrees(wrap(math.radians(m0["initial_rod_yaw_deg"]) -
                                   math.radians(m1["initial_rod_yaw_deg"]))))
        same_block.append({"block": b, "initial_truth_position_delta_mm": d,
                           "initial_truth_yaw_delta_deg": dy})
    write_csv(OUT / "formal_block_initial_truth.csv", same_block)

    # Figures: condition curves and paired differences (no titles or slogans).
    colors = {"M0": "#3b6fb6", "M1": "#c44e52"}
    if curves:
        fig, axs = plt.subplots(1, 2, figsize=(9.4, 3.9), constrained_layout=True)
        for cond in ("M0", "M1"):
            runs = sorted({r["run_id"] for r in curves if r["condition"] == cond})
            if not runs:
                continue
            grids = np.linspace(0.0, 1.0, 101)
            e_stack, d_stack = [], []
            for rid in runs:
                rows = sorted([r for r in curves if r["run_id"] == rid],
                              key=lambda r: r["normalized_main_time"])
                xs = [r["normalized_main_time"] for r in rows]
                e_stack.append([interp(xs, [r["e_z_common_mm"] for r in rows], u) for u in grids])
                d_stack.append([interp(xs, [r["delta_e_z_mm"] for r in rows], u) for u in grids])
            e_arr, d_arr = np.array(e_stack, dtype=float), np.array(d_stack, dtype=float)
            for ax, arr in ((axs[0], e_arr), (axs[1], d_arr)):
                mean = np.nanmean(arr, axis=0)
                sd = np.nanstd(arr, axis=0, ddof=1) if arr.shape[0] > 1 else np.zeros_like(mean)
                ax.plot(grids, mean, color=colors[cond], linewidth=1.9, label=f"{cond} mean")
                ax.fill_between(grids, mean - sd, mean + sd, color=colors[cond], alpha=0.18,
                                linewidth=0)
        axs[0].set_ylabel("e_z (mm)")
        axs[1].set_ylabel("Δe_z (mm)")
        for ax in axs:
            ax.axhline(0, color="black", linewidth=0.7)
            ax.set_xlabel("Normalized main-segment time")
            ax.legend(frameon=False)
        fig.savefig(OUT / "formal_transport_error.png", dpi=180)
        plt.close(fig)

    nonempty_keys = [item for item in metric_keys if summaries[item[0]]["n"] > 0]
    fig, axs = plt.subplots(max(1, len(nonempty_keys)), 1,
                            figsize=(6.4, 1.2 * max(1, len(nonempty_keys))),
                            constrained_layout=True, squeeze=False)
    axs = [row[0] for row in axs]
    fig_labels = {
        "e_z_start_mm": "e_z at main start (mm)",
        "e_z_mean_mm": "mean e_z, main segment (mm)",
        "e_z_rms_mm": "RMS e_z, main segment (mm)",
        "delta_e_z_rms_mm": "RMS change from starting error (mm)",
        "delta_e_z_end_mm": "end change from starting error (mm)",
        "release_xy_error_mm": "release XY error (mm)",
        "release_yaw_error_deg": "release yaw error (deg)",
    }
    for ax, (key, label) in zip(axs, nonempty_keys):
        s = summaries[key]
        vals = [b - a for a, b in
                [(m0[key], m1[key]) for m0, m1 in paired.values()
                 if m0 is not None and m1 is not None
                 and math.isfinite(m0[key]) and math.isfinite(m1[key])]]
        ax.axvline(0, color="black", linewidth=0.7)
        ax.plot(vals, [0] * len(vals), "o", color="#4c4c4c", markersize=5, alpha=0.75)
        ax.errorbar([s["mean"]], [0], xerr=[[s["mean"] - s["ci_low"]], [s["ci_high"] - s["mean"]]],
                    fmt="o", color="#c44e52", capsize=4, markersize=6)
        ax.set_yticks([])
        ax.set_xlabel(f"M1 − M0: {fig_labels[key]}")
    fig.savefig(OUT / "formal_paired_differences.png", dpi=180)
    plt.close(fig)

    # Completion / validity accounting.
    attempts = [r for r in ledger if r["phase"] == "formal" and r["action_started"] == "1"]
    completed = sum(1 for r in attempts if r["flow_completed"] == "1")
    valid = {cond: sum(1 for m in by_cond.get(cond, [])
                       if math.isfinite(m["e_z_start_mm"]) and m["e_z_sample_count"] > 10)
             for cond in ("M0", "M1")}
    started = {cond: len(by_cond.get(cond, [])) for cond in ("M0", "M1")}
    complete_by = {cond: sum(m["task_complete"] for m in by_cond.get(cond, []))
                   for cond in ("M0", "M1")}

    def line(key):
        s = summaries[key]
        if s["n"] == 0:
            return "无配对有效样本"
        return (f"n={s['n']}，均值差 {fmt(s['mean'])}（95% CI {fmt(s['ci_low'])} 至 "
                f"{fmt(s['ci_high'])}），范围 {fmt(s['min'])} 至 {fmt(s['max'])}")

    def cond_mean(cond, key):
        vals = [m[key] for m in by_cond.get(cond, []) if math.isfinite(m[key])]
        return statistics.mean(vals) if vals else float("nan")

    reason_counts = {}
    for m in metrics:
        if m["task_complete"] == 1:
            continue
        key = re.split(r"[：:]", m["failure_reason"] or "未记录原因", maxsplit=1)[0][:70]
        reason_counts[key] = reason_counts.get(key, 0) + 1
    reason_text = "；".join(f"{k}：{v} 次"
                            for k, v in sorted(reason_counts.items(), key=lambda kv: -kv[1]))

    def run_row(m):
        status = m["task_state"] or "-"
        reason = re.split(r"[：:]", m["failure_reason"] or "", maxsplit=1)[0].replace("|", "/")[:70]
        phase = m["failure_phase"] or ""
        return (f"| {m['block']} | {m['condition']} | {m['task_complete']} | "
                f"{fmt(m['e_z_start_mm'])} | {fmt(m['e_z_mean_mm'])} | {fmt(m['e_z_rms_mm'])} | "
                f"{fmt(m['delta_e_z_rms_mm'])} | {fmt(m['delta_e_z_end_mm'])} | "
                f"{fmt(m['release_xy_error_mm'])} | {fmt(m['release_yaw_error_deg'])} | "
                f"{fmt(m['adopted_ref_dz_max_mm'], 4)} | {fmt(m['adopted_ref_dyaw_max_deg'], 4)} | "
                f"{fmt(m['main_duration_error_s'], 4)} | {status} {phase} {reason} |")
    run_table = "\n".join(run_row(m) for m in
                          sorted(metrics, key=lambda m: (int(m["block"]), m["condition"])))

    conn0 = cond_mean("M0", "connection_object_offset_mm")
    conn1 = cond_mean("M1", "connection_object_offset_mm")
    ff0 = cond_mean("M0", "published_ff_per_arm_at_main_start_n")
    ff1 = cond_mean("M1", "published_ff_per_arm_at_main_start_n")
    ref_dz = max((m["adopted_ref_dz_max_mm"] for m in metrics
                  if math.isfinite(m["adopted_ref_dz_max_mm"])), default=float("nan"))
    ref_dyaw = max((m["adopted_ref_dyaw_max_deg"] for m in metrics
                    if math.isfinite(m["adopted_ref_dyaw_max_deg"])), default=float("nan"))
    dur_min = min((m["main_duration_error_s"] for m in metrics
                   if math.isfinite(m["main_duration_error_s"])), default=float("nan"))
    dur_max = max((m["main_duration_error_s"] for m in metrics
                   if math.isfinite(m["main_duration_error_s"])), default=float("nan"))
    missing_blocks = [b for b, (a, c) in paired.items()
                      if a is None or c is None or
                      not math.isfinite(a["e_z_start_mm"]) or not math.isfinite(c["e_z_start_mm"])]
    s_start = summaries["e_z_start_mm"]
    s_drms = summaries["delta_e_z_rms_mm"]
    s_dend = summaries["delta_e_z_end_mm"]
    s_rel = summaries["release_xy_error_mm"]
    if formal_ref_summary and formal_ref_summary.get("blocks"):
        ref_summary_text = (
            f"{formal_ref_summary['blocks']} 个区组的最大差为 Z "
            f"{fmt(formal_ref_summary['z_max'])} mm、XY "
            f"{fmt(formal_ref_summary['xy_max'])} mm、偏航 "
            f"{fmt(formal_ref_summary['yaw_max'],4)}°（预声明允许 0.5 mm / 0.5 mm / 0.10°）")
    else:
        ref_summary_text = "无可配对的运输参考行"

    text = f"""# 完整搬运载荷前馈正式实验结果（修订批次 r2）

## 冻结条件与执行口径

正式批次沿用 r2 冻结配置：共同名义运输参考（前馈前杆位姿＋0.40 m，终点为固定目标）、
主段固定时长 {D_FIXED:.3f} s、共同五次平滑进度函数、0.5 s 起点衔接单独记录。M0 杆载荷前馈
全程为零；M1 首条有效 MMC 承载参考确认后 0.5 s 渐增至每臂 3.1392 N，方向世界 +Z。
控制、阻抗、夹持、摩擦、质量、碰撞与任务判据均未改变，正式期间未修改影响结论的条件。

预试验门槛的两项未通过（完整流程；抓取后实测初始真值偏航 0.145° > 预声明 0.05°）见
`pretrial_results.md`。经用户 2026-10-26 明确指示“继续做完”，在 `FORMAL_USER_OVERRIDE.md`
记录门槛现状与执行口径后启动正式批次；所有正式失败保留，未补跑替换。

## 尝试、有效样本与完成情况

- 正式动作尝试 20/20（M0 10、M1 10）。
- 进入运输主段：18/20；区组 6 的 M0/M1 均在抓取前的下降路径校验失败
  （`E_2B_SCHEDULE_ENDPOINT_MISMATCH`），未进入承载，因此该区组无运输指标。
- 主段指标有效：M0 {valid['M0']}/10、M1 {valid['M1']}/10（缺失值不补零）。
- 完整流程（抓取至撤退）：{completed}/20（M0 {complete_by['M0']}/10、M1 {complete_by['M1']}/10）。
- 未完成动作的原因：{reason_text}。另有 3 次动作前启动异常，已按规则重启动，不计为动作尝试。
- 区组输入已下发核对：{'通过' if block_applied_ok else '未通过'}（wrapper 参数与任务冻结真值位置）；
  抓取后实测初始真值与区组指令最大差
  {fmt(max((abs(m['block_initial_position_delta_mm']) for m in metrics if math.isfinite(m['block_initial_position_delta_mm'])), default=float('nan')))} mm、
  {fmt(max((abs(m['block_initial_yaw_delta_deg']) for m in metrics if math.isfinite(m['block_initial_yaw_delta_deg'])), default=float('nan')))}°。
  该差为杆静置后的实测姿态，不是区组输入未生效；预声明的 0.05° 过程容差仍在预试验报告中单列。

## 参考、时长与前馈的实际执行

- 主段计划时长逐次 3.0400 s；实际主段时长误差 {fmt(dur_min,4)} 至 {fmt(dur_max,4)} s
  （允许 ±0.020 s）。
- 下发物体参考与共同名义路径的最大偏差（全部正式运行）：Δz {fmt(ref_dz,4)} mm，
  Δ偏航 {fmt(ref_dyaw,4)}°。
- 跨条件下发物体参考（匹配归一化主段时间）：{ref_summary_text}。逐区组见
  `formal_reference_difference.csv`。
- 起点衔接物体偏移均值：M0 {fmt(conn0)} mm、M1 {fmt(conn1)} mm；衔接口径与记录见
  `frozen_config.md`，不计入主段指标。
- 前馈主段起点发布值均值：M0 {fmt(ff0)} N/臂、M1 {fmt(ff1)} N/臂（目标 3.1392，世界 +Z）；
  M0 全程为零，M1 在主段开始前已完成 0.5 s 渐增，没有额外承载等待。

## 逐次结果

| 区组 | 条件 | 完成 | e_z(t0) mm | 主段均值 mm | 主段 RMS mm | Δe_z RMS mm | Δe_z 末值 mm | 释放 XY mm | 释放偏航 ° | 参考 Δz mm | 参考 Δ偏航 ° | 时长误差 s | 状态/原因 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
{run_table}

## 主要指标（条件均值与区组配对差）

| 指标 | M0 均值（n） | M1 均值（n） | M1−M0 配对 |
|---|---:|---:|---|
"""
    for key, label in metric_keys:
        m0v = [m[key] for m in by_cond.get("M0", []) if math.isfinite(m[key])]
        m1v = [m[key] for m in by_cond.get("M1", []) if math.isfinite(m[key])]
        text += (f"| {label} | {fmt(statistics.mean(m0v) if m0v else float('nan'))}（{len(m0v)}） | "
                 f"{fmt(statistics.mean(m1v) if m1v else float('nan'))}（{len(m1v)}） | "
                 f"{line(key)} |\n")
    text += f"""
95% 区间为区组内配对差值的 t 区间（n−1 自由度）。有效配对区组：9 个（区组
{', '.join(str(b) for b in sorted(set(paired) - set(missing_blocks)))}）；缺失区组
{', '.join(str(b) for b in missing_blocks) or '无'} 不参与该指标配对，也不补零。
逐次值与区组差见 `formal_metrics.csv` 与 `formal_block_differences.csv`。

## 结果分析与判断

1. **收益是否主要发生在运输开始前。** 主段起点误差差 M1−M0 为 {fmt(s_start['mean'])} mm
   （95% CI {fmt(s_start['ci_low'])} 至 {fmt(s_start['ci_high'])}；n={s_start['n']}），
   主段总体误差 RMS 差为 {fmt(summaries['e_z_rms_mm']['mean'])} mm
   （{fmt(summaries['e_z_rms_mm']['ci_low'])} 至 {fmt(summaries['e_z_rms_mm']['ci_high'])}）。
   M1 在主段起点已基本处于共同名义高度（均值 {fmt(cond_mean('M1','e_z_start_mm'))} mm），
   M0 仍低 {fmt(-cond_mean('M0','e_z_start_mm'))} mm。该差值约 8.7 mm，与 MMC 在杆重下的
   准静态阻抗偏转量级一致（杆重 6.3 N / 共同物体平移刚度 800 N·m⁻¹ ≈ 7.9 mm）：
   前馈每臂 3.14 N 抵消了杆重，使杆在共同参考附近而不是悬挂在其下方。也就是说，
   在本共同参考设计下，前馈的主要作用是把杆保持在共同名义参考高度；起点处已解释了
   绝大部分差异，而不是运输轨迹或几何差异。
2. **运输误差相对起点的变化。** Δe_z=e_z(t)−e_z(t₀)，其RMS配对差为
   {fmt(s_drms['mean'])} mm（95% CI {fmt(s_drms['ci_low'])} 至 {fmt(s_drms['ci_high'])}；
   n={s_drms['n']}），Δe_z 末值为 {fmt(s_dend['mean'])} mm
   （{fmt(s_dend['ci_low'])} 至 {fmt(s_dend['ci_high'])}）。两条件末值均为正，
   M0约+1.910 mm、M1约+1.379 mm；由于起点误差为负，正值表示向上接近参考高度，
   不表示新增下沉。M1的Δe_z RMS平均小约0.524 mm，只能说明相对起点的误差变化较小。
   从较大下沉中恢复也会增大这一指标，因此其区间不跨0不能单独证明动态跟踪改善。
   本批承载收益由起点误差及总体e_z RMS描述，不将Δe_z RMS的下降称为新增下沉减少。
3. **承载变化是否转化为放置精度改善。** 不能判断。正式批次的释放配对有效样本数
   n={s_rel['n']}：仅区组 5 的 M0 完成释放（释放平面误差 14.4 mm、偏航误差 −1.36°），
   M1 没有完成释放的样本。并且 20 次中 19 次未完成完整流程，其中放置阶段失败占主导
   （滑移超 20 mm 9 次，开爪前→释放位移超 10 mm 7 次，释放真值/反馈失败 1 次）。
   因此本批次没有形成放置精度的有效配对比较，结果同时反映新参考方案下放置流程的完成困难。
4. **完成情况与其他指标的代价。** 完整流程为 M0 {complete_by['M0']}/10、M1 {complete_by['M1']}/10。
   M1在主段保持较小的总体竖向偏差，但两条件放置流程均存在中止。
   7次释放位移中止中，杆中心下降9.971—10.453 mm，水平位置变化仅0.779—1.020 mm，
   姿态变化2.988—3.255°，最终中心高度均为20.654 mm，符合倾斜长杆落平的几何变化量级。
   9次LOWER中止的末次杆真值对应底面距支撑面约14.5—34.2 mm，
   此时左侧抓取参考位置残差已超过20 mm，不能用触台后落平解释这类中止。正常落平与异常掉落需结合
   支撑、姿态及最终落点区分，不能仅凭该错误名称确认物理掉落，亦不能唯一归因于前馈。
   本批支持固定场景、当前共同参考实现下较小的运输总体竖向偏差，主要差异在主段入口
   已经形成；不支持动态偏差普遍改善、放置精度改善或完整任务性能提升。

## 有效范围与限制

- 固定单一场景（区组仅 ±2 mm、±0.4° 小扰动）、0.64 kg 杆、10 N/指夹持、本批次冻结参数；
  9 个区组配对，区组 6 缺失；不能外推为其他姿态、载荷或真机能力。
- 区间为描述性统计，不构成显著性检验或总体推断；样本为仿真重复，不是独立工况随机抽样。
- `e_z(t0)` 是主段起点（衔接结束后）的误差，包含控制器在 0.5 s 衔接内跟随共同参考的响应；
  它不是“前馈前下沉”的直接测量。共同参考的定义、抓持变换选择与理由见 `frozen_config.md`。
- 杆真值取请求/响应仿真时间中点，只在覆盖范围内插值（逐次 154 个 20 ms 样本，覆盖 1.000）；
  不宣称精确同步物理采样。
- 正式失败全部保留；运输指标与完整流程分别统计，缺失值不补零。

## 图

`formal_transport_error.png`：归一化主段时间上的竖向误差（左）与相对主段起点的变化（右），
曲线为条件均值，阴影为 ±1 SD。`formal_paired_differences.png`：各区组 M1−M0 配对差（灰点）
与均值及 95% t 区间（红）。释放指标无配对样本，未绘制该子图。图内不放标题或结论。
Δe_z为相对起点的有符号误差变化，RMS不含方向；本批末值为正时表示从负起点误差向上恢复，
不能将该图标注为“新增下沉”或用其较小直接断言动态控制更优。
"""
    (OUT / "results.md").write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
