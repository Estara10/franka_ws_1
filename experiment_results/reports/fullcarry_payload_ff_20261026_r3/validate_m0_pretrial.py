#!/usr/bin/env python3
"""Check the single r3 M0 pretrial against the three current-version M1 validations."""
import csv
import math
from pathlib import Path

import analyze_batch as analysis

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
RUNS = ROOT / "experiment_results/a1_runs"
REPAIR_LOGS = ROOT / "experiment_results/reports/fullcarry_payload_ff_20261026_repair"
M1_IDS = [
    "fullcarry_ff_truth_cbgroup_20261026_M1_01",
    "fullcarry_ff_truth_cbgroup_20261026_M1_02",
    "fullcarry_ff_truth_cbgroup_20261026_M1_03",
]
EXPECTED = {"start_x_m": "0.500", "start_y_m": "0.000", "start_yaw_deg": "10.0",
            "target_x_m": "0.400", "target_y_m": "0.000", "target_yaw_deg": "20.0"}


def rows(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path, data):
    if not data:
        return
    fields = list(data[0])
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(data)


def load_payload_trace(run_id):
    samples = rows(RUNS / run_id / "abcd_payload_trace.csv")
    forces = []
    for r in samples:
        for key in ("left_published_ff_z_n", "right_published_ff_z_n"):
            try:
                value = float(r[key])
            except (KeyError, ValueError):
                continue
            if math.isfinite(value):
                forces.append(value)
    return samples, forces


def load_metric(row):
    metric, curve, command = analysis.load_run(row)
    return metric, curve, command


def main():
    ledger = rows(OUT / "attempts.csv")
    m0_rows = [r for r in ledger if r["phase"] == "pretrial" and r["condition"] == "M0"
               and r["action_started"] == "1"]
    if len(m0_rows) != 1:
        raise SystemExit(f"expected exactly one action-started M0 pretrial, found {len(m0_rows)}")
    m0_row = m0_rows[0]
    m0, m0_curve, m0_cmd = load_metric(m0_row)
    m0_contact = rows(RUNS / m0["run_id"] / "contact_probe.csv")
    last_contact = m0_contact[-1] if m0_contact else {}

    m1_data = []
    for i, run_id in enumerate(M1_IDS, 1):
        wrapper = f"experiment_results/reports/fullcarry_payload_ff_20261026_repair/truth_cbgroup_M1_0{i}_wrapper.log"
        row = {"phase": "existing_m1_reference", "sequence": "0", "block": "pre",
               "condition": "M1", "run_id": run_id, "launcher_log": wrapper,
               "direct_reason": "Existing current-version M1 success; reference only, not an r3 action",
               **EXPECTED}
        metric, curve, command = load_metric(row)
        m1_data.append((row, metric, curve, command))

    comparison = []
    reference_details = []
    gate = {
        "M0_full_flow_complete": m0["task_complete"] == 1 and m0["task_state"] == "DONE",
        "M0_payload_zero": m0["payload_enabled"] == 0,
        "M0_common_reference_and_duration": (m0["common_reference_enabled"] == 1 and
            m0["common_reference_source"].startswith("pre_feedforward_nominal_object_path") and
            abs(m0["plan_main_duration_s"] - 3.040) <= 0.001 and
            abs(m0["dispatch_connection_s"] - 0.500) <= 0.001 and
            abs(m0["main_duration_error_s"]) <= 0.020),
        "M0_planning_checks": m0["check_pass"] == 1 and m0["check_collisions"] == 0,
        "M0_transport_measurement_valid": (m0["e_z_sample_count"] > 10 and
            m0["e_z_window_coverage"] >= 0.95),
        "M0_requested_scene_applied": (m0["wrapper_request_matches_block"] == 1 and
            math.isfinite(m0["frozen_truth_position_delta_mm"]) and
            m0["frozen_truth_position_delta_mm"] <= 0.5),
        "M0_adopted_reference_matches_nominal": (max(m0["adopted_ref_dx_max_mm"],
            m0["adopted_ref_dy_max_mm"], m0["adopted_ref_dz_max_mm"]) <= 0.5 and
            m0["adopted_ref_dyaw_max_deg"] <= 0.10),
    }
    m0_trace, m0_forces = load_payload_trace(m0["run_id"])
    gate["M0_trace_confirms_zero_force"] = bool(m0_trace) and max(map(abs, m0_forces), default=math.inf) <= 1e-9

    for _, m1, _, m1_cmd in m1_data:
        refs, ref_summary = analysis.reference_difference({"M0": m0_cmd, "M1": m1_cmd})
        dx = 1000 * (m0["common_start_x_m"] - m1["common_start_x_m"])
        dy = 1000 * (m0["common_start_y_m"] - m1["common_start_y_m"])
        dz = 1000 * (m0["common_start_z_m"] - m1["common_start_z_m"])
        yaw = abs(math.degrees(analysis.wrap(math.radians(m0["common_start_yaw_deg"] -
                                                            m1["common_start_yaw_deg"]))))
        pos = math.sqrt(dx * dx + dy * dy + dz * dz)
        comparison.append({
            "m1_reference_run_id": m1["run_id"], "m0_run_id": m0["run_id"],
            "start_position_delta_mm": f"{pos:.6f}", "start_yaw_delta_deg": f"{yaw:.6f}",
            "reference_z_max_difference_mm": f"{ref_summary.get('reference_z_difference_max_mm', math.nan):.6f}",
            "reference_xy_max_difference_mm": f"{ref_summary.get('reference_xy_difference_max_mm', math.nan):.6f}",
            "reference_yaw_max_difference_deg": f"{ref_summary.get('reference_yaw_difference_max_deg', math.nan):.6f}",
            "m1_complete": m1["task_complete"], "m1_payload_enabled": m1["payload_enabled"],
            "m1_force_at_main_start_n": f"{m1['published_ff_per_arm_at_main_start_n']:.6f}",
            "m1_ramp_elapsed_at_main_start_s": f"{m1['payload_ramp_elapsed_at_main_start_s']:.6f}",
        })
        for row in refs:
            row["m1_reference_run_id"] = m1["run_id"]
            reference_details.append(row)

    gate["M1_existing_runs_are_valid_current_references"] = all(
        m["task_complete"] == 1 and m["payload_enabled"] == 1 and
        abs(m["plan_main_duration_s"] - 3.040) <= 0.001 and
        abs(m["main_duration_error_s"]) <= 0.020 and
        abs(m["published_ff_per_arm_at_main_start_n"] - 3.1392) <= 0.01 and
        m["payload_ramp_elapsed_at_main_start_s"] >= 0.5 and
        m["e_z_sample_count"] > 10 and m["e_z_window_coverage"] >= 0.95
        for _, m, _, _ in m1_data)
    gate["references_comparable_to_existing_M1"] = all(
        float(r["start_position_delta_mm"]) <= 1.0 and
        float(r["start_yaw_delta_deg"]) <= 0.05 and
        float(r["reference_z_max_difference_mm"]) <= 0.5 and
        float(r["reference_xy_max_difference_mm"]) <= 0.5 and
        float(r["reference_yaw_max_difference_deg"]) <= 0.10
        for r in comparison)

    metrics = [{"record_role": "single_r3_M0_pretrial", **m0}]
    metrics.extend({"record_role": "existing_M1_validation_reference", **m}
                   for _, m, _, _ in m1_data)
    write_csv(OUT / "pretrial_metrics.csv", metrics)
    curve_rows = [{"record_role": "single_r3_M0_pretrial", **r} for r in m0_curve]
    for _, _, curve, _ in m1_data:
        curve_rows.extend({"record_role": "existing_M1_validation_reference", **r} for r in curve)
    write_csv(OUT / "pretrial_transport_curves.csv", curve_rows)
    write_csv(OUT / "pretrial_reference_comparison.csv", comparison)
    write_csv(OUT / "pretrial_reference_difference.csv", reference_details)

    checks = "\n".join(f"| {key} | {'PASS' if value else 'FAIL'} |" for key, value in gate.items())
    all_pass = all(gate.values())
    release_text = (f"XY误差 {m0['release_xy_error_mm']:.3f} mm，偏航误差 "
                    f"{m0['release_yaw_error_deg']:.3f}°" if
                    math.isfinite(m0["release_xy_error_mm"]) and
                    math.isfinite(m0["release_yaw_error_deg"]) else "未产生（运行未进入释放阶段）")
    failure_detail = ""
    if m0["task_complete"] == 0:
        failure_detail = f"""

## 失败阶段与直接原因

运行在`{m0['failure_phase'] or '未知'}`阶段以`{m0['failure_reason'] or m0['direct_reason']}`中止。末个接触记录的杆—手位置残差为左{1000*float(last_contact.get('left_slip_m','nan')):.3f} mm、右{1000*float(last_contact.get('right_slip_m','nan')):.3f} mm；角残差为左{math.degrees(float(last_contact.get('left_angle_rad','nan'))):.3f}°、右{math.degrees(float(last_contact.get('right_angle_rad','nan'))):.3f}°。该行两侧接触标志仍有效，杆中心高度为{1000*float(last_contact.get('rod_z','nan')):.1f} mm（支撑面为0 mm）。控制器切换、运输执行及真值对齐正常；直接失败是下降阶段由杆真值与手部FK计算的相对位姿残差越过既有20 mm保护限值，指向承载过程中的杆—手相对运动，不是启动、通信或控制器软件阻断。该指标不是直接的夹爪垫面滑移测量；一次M0预试验也不能证明关闭前馈是唯一原因。
"""
    report = f"""# M0单次预试验核对

## 运行

- M0 run：`{m0['run_id']}`，状态 `{m0['task_state']}`，完整流程 `{m0['task_complete']}/1`。
- 对照参考：当前版本此前三次M1完整成功运行 `{', '.join(M1_IDS)}`。这些M1只用于核对版本、条件与参考，不计入r3预试验或正式样本。
- 杆载荷前馈：M0主段每臂发布值 {m0['published_ff_per_arm_at_main_start_n']:.6f} N，trace绝对峰值 {max(map(abs, m0_forces), default=math.nan):.9f} N；有效trace行数 {len(m0_trace)}。
- 运输主段计划/实际时长：{m0['plan_main_duration_s']:.3f}/{m0['main_actual_duration_s']:.3f} s；单独起点衔接 {m0['dispatch_connection_s']:.3f} s。
- 有效运输数据：{m0['e_z_sample_count']}个仿真时间对齐样本，覆盖 {m0['e_z_window_coverage']:.3f}；真值请求/响应间隔中位数/最大值 {m0['truth_interval_median_ms']:.3f}/{m0['truth_interval_max_ms']:.3f} ms。
- 运输指标：`e_z(t₀)` {m0['e_z_start_mm']:.3f} mm；总体均值/RMS {m0['e_z_mean_mm']:.3f}/{m0['e_z_rms_mm']:.3f} mm；相对起点误差变化 `Δe_z` 末值/RMS {m0['delta_e_z_end_mm']:.3f}/{m0['delta_e_z_rms_mm']:.3f} mm。`Δe_z`保留符号，不将其较小解释为新增下沉减少。
- 释放测量：{release_text}。
{failure_detail}

## 与现有M1参考的比较

| M1记录 | 共同参考起点位置差 (mm) | 起点偏航差 (°) | 参考Z最大差 (mm) | 参考XY最大差 (mm) | 参考偏航最大差 (°) | M1主段前馈 (N/臂) |
|---|---:|---:|---:|---:|---:|---:|
""" + "\n".join(
        f"| `{r['m1_reference_run_id']}` | {float(r['start_position_delta_mm']):.3f} | "
        f"{float(r['start_yaw_delta_deg']):.4f} | {float(r['reference_z_max_difference_mm']):.4f} | "
        f"{float(r['reference_xy_max_difference_mm']):.4f} | {float(r['reference_yaw_max_difference_deg']):.4f} | "
        f"{float(r['m1_force_at_main_start_n']):.4f} |" for r in comparison) + f"""

## 进入正式实验的核对

| 核对项 | 结果 |
|---|---|
{checks}

{'核对通过：冻结当前版本和配置，可执行预定的M0/M1各10次正式批次。' if all_pass else '核对未通过：正式批次保持未启动；本次M0结果按实际情况保留。'}
"""
    (OUT / "pretrial_results.md").write_text(report, encoding="utf-8")
    gate_path = OUT / "PRETRIAL_GATE_PASS"
    if all_pass:
        gate_path.write_text("M0 single pretrial and comparison with existing current-version M1 runs passed.\n",
                             encoding="utf-8")
    else:
        gate_path.unlink(missing_ok=True)
    print(f"M0 pretrial checks: {'PASS' if all_pass else 'FAIL'}")
    for key, value in gate.items():
        print(f"  {key}={value}")
    if not all_pass:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
