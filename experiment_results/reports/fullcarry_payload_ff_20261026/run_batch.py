#!/usr/bin/env python3
"""Sequential, resumable runner for the 2026-10-26 payload-FF full-carry batch."""
from __future__ import annotations

import csv
import os
import re
import signal
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "experiment_results/reports/fullcarry_payload_ff_20261026"
RUNS = ROOT / "experiment_results/a1_runs"
LEDGER = OUT / "attempts.csv"
FIELDS = [
    "phase", "sequence", "block", "condition", "launch_attempt", "run_id", "domain_id",
    "start_x_m", "start_y_m", "start_yaw_deg", "target_x_m", "target_y_m", "target_yaw_deg",
    "wrapper_exit", "action_started", "transport_started", "flow_completed", "task_state",
    "status", "direct_reason", "run_dir", "launcher_log",
]


def read_rows():
    if not LEDGER.exists():
        return []
    with LEDGER.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_rows(rows):
    tmp = LEDGER.with_suffix(".csv.tmp")
    with tmp.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(LEDGER)


def marker_status(run_dir: Path):
    log = run_dir / "task_launch.log"
    s = log.read_text(errors="replace") if log.exists() else ""
    p = s.find("[P3A-1] 下发轨迹")
    action = bool(p >= 0 and "Execute request accepted" in s[p:])
    transport = "[TRANSPORT_EVENT] event=start" in s
    done = ("[CONTACT_PROBE_RESULT] state=DONE" in s and
            "[CONTACT_PROBE_RELEASED]" in s and "[CONTACT_PROBE_RETREAT]" in s)
    state_m = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+)", s)
    reason_m = re.findall(r"\[CONTACT_PROBE_FAILURE\] ([^\r\n]+)", s)
    if not reason_m:
        reason_m = re.findall(r"\[CONTACT_PROBE_RESULT\] state=FAILED detail=([^\r\n]+)", s)
    return action, transport, done, state_m[-1] if state_m else "", reason_m[-1] if reason_m else "", s


def startup_reason(launcher: Path, task_log: str):
    data = launcher.read_text(errors="replace") if launcher.exists() else ""
    for line in data.splitlines():
        if "error:" in line.lower() or "failed" in line.lower() or "timeout" in line.lower():
            return line.strip()[:240]
    for line in task_log.splitlines():
        if "preflight" in line.lower() or "startup" in line.lower() or "timeout" in line.lower():
            return line.strip()[:240]
    return "wrapper returned before accepted pre-grasp motion"


def persist(row):
    rows = read_rows()
    for i, old in enumerate(rows):
        if old["run_id"] == row["run_id"]:
            rows[i] = row
            write_rows(rows)
            return
    rows.append(row)
    write_rows(rows)


def count_started(rows, phase):
    return sum(r["phase"] == phase and r["action_started"] == "1" for r in rows)


def recover_interrupted(rows):
    changed = False
    for row in rows:
        if row["status"] != "reserved":
            continue
        run_dir = Path(row["run_dir"])
        action, transport, done, state, reason, text = marker_status(run_dir)
        if action:
            row.update(action_started="1", transport_started=str(int(transport)),
                       flow_completed=str(int(done)), task_state=state,
                       status=("pretrial_completed" if row["phase"] == "pretrial" and done else
                               "pretrial_incomplete" if row["phase"] == "pretrial" else
                               "completed" if done else "incomplete"),
                       direct_reason=reason or ("interrupted after formal action start" if not done else ""))
        else:
            sim_log = run_dir / "sim_dual_moveit.log"
            startup_text = sim_log.read_text(errors="replace") if sim_log.exists() else ""
            diagnosis = ("FastDDS getifaddrs/UDP socket denied by sandbox; no action started"
                         if "getifaddrs: Operation not permitted" in startup_text or
                            "Error creating socket: Operation not permitted" in startup_text else
                         "recovered reservation without accepted pre-grasp motion")
            row.update(action_started="0", transport_started="0", flow_completed="0",
                       status="startup_exception",
                       direct_reason=diagnosis)
        persist(row)
        changed = True
    return read_rows() if changed else rows


def load_schedule(phase):
    if phase == "pretrial":
        return [
            {"sequence": 0, "block": "pre", "condition": "M0", "payload_feedforward": "0",
             "start_x_m": "0.500", "start_y_m": "0.000", "start_yaw_deg": "10.0",
             "target_x_m": "0.400", "target_y_m": "0.000", "target_yaw_deg": "20.0"},
            {"sequence": 0, "block": "pre", "condition": "M1", "payload_feedforward": "1",
             "start_x_m": "0.500", "start_y_m": "0.000", "start_yaw_deg": "10.0",
             "target_x_m": "0.400", "target_y_m": "0.000", "target_yaw_deg": "20.0"},
        ]
    with (OUT / "schedule.csv").open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def prior_transport_failures(rows, condition):
    prior = [r for r in rows if r["phase"] == "formal" and r["condition"] == condition and
             r["action_started"] == "1"]
    prior.sort(key=lambda r: (int(r["sequence"]), int(r["launch_attempt"])))
    return len(prior) >= 2 and all(r["transport_started"] == "0" for r in prior[-2:])


def run_one(phase, item, launch_attempt):
    condition = item["condition"]
    sequence = int(item["sequence"])
    if phase == "pretrial":
        label = condition
        domain = 120 + (0 if condition == "M0" else 5) + launch_attempt
    else:
        label = f"B{int(item['block']):02d}_{condition}"
        domain = 140 + sequence * 2 + launch_attempt
    run_id = f"fullcarry_ff_20261026_{phase}_{label}_a{launch_attempt}"
    run_dir = RUNS / run_id
    launcher = OUT / f"{run_id}_wrapper.log"
    if run_dir.exists() or (ROOT / "launch_logs" / run_id).exists():
        raise RuntimeError(f"Refusing to reuse existing run directory or ROS log: {run_id}")

    row = {k: "" for k in FIELDS}
    row.update(phase=phase, sequence=str(sequence), block=str(item["block"]), condition=condition,
               launch_attempt=str(launch_attempt), run_id=run_id, domain_id=str(domain),
               start_x_m=item["start_x_m"], start_y_m=item["start_y_m"],
               start_yaw_deg=item["start_yaw_deg"], target_x_m=item["target_x_m"],
               target_y_m=item["target_y_m"], target_yaw_deg=item["target_yaw_deg"],
               wrapper_exit="", action_started="0", transport_started="0", flow_completed="0",
               status="reserved", direct_reason="", run_dir=str(run_dir.relative_to(ROOT)),
               launcher_log=str(launcher.relative_to(ROOT)))
    persist(row)

    env = os.environ.copy()
    env.update({
        "CONTACT_PROBE_DOMAIN_ID": str(domain), "CONTACT_PROBE_RUN_ID": run_id,
        "CONTACT_PROBE_START_X": item["start_x_m"], "CONTACT_PROBE_START_Y": item["start_y_m"],
        "CONTACT_PROBE_START_YAW_DEG": item["start_yaw_deg"],
        "CONTACT_PROBE_TARGET_X": item["target_x_m"], "CONTACT_PROBE_TARGET_Y": item["target_y_m"],
        "CONTACT_PROBE_TARGET_YAW_DEG": item["target_yaw_deg"],
        "CONTACT_PROBE_ABCD_GROUP": "C", "CONTACT_ABCD_PAYLOAD_FEEDFORWARD": item["payload_feedforward"],
        "CONTACT_PROBE_PAYLOAD_FEEDFORWARD": "0", "CONTACT_PROBE_HOVER_CONTROL_MODE": "jtc",
        "CONTACT_PROBE_MMC_CONTROL_VARIANT": "fixed", "CONTACT_PROBE_ROD_DISTURBANCE": "0",
        "CONTACT_PROBE_ADAPTIVE_FORCE_HOLD": "false", "CONTACT_ABCD_MATCHED_WAIT_SEC": "0",
        "CONTACT_TRANSPORT_STRETCH": "2.0", "CONTACT_TRANSPORT_TIME_SCALE": "1.0",
        "CONTACT_TRANSPORT_PLAN_ONLY": "0", "CONTACT_PROBE_HOVER_ONLY": "0",
        "CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY": "0",
    })
    env.pop("CONTACT_TRANSPORT_PAIRING", None)
    env.pop("CONTACT_PROBE_RIGID", None)
    print(f"START phase={phase} sequence={sequence} block={item['block']} condition={condition} run={run_id}",
          flush=True)
    with launcher.open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(["bash", "scripts/run_contact_probe_local.sh", "--headless"],
                                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            rc = proc.wait()
        except KeyboardInterrupt:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=10)
            raise
    action, transport, done, state, reason, task_text = marker_status(run_dir)
    row.update(wrapper_exit=str(rc), action_started=str(int(action)),
               transport_started=str(int(transport)), flow_completed=str(int(done)), task_state=state)
    if action:
        row["status"] = (("pretrial_completed" if done else "pretrial_incomplete")
                         if phase == "pretrial" else ("completed" if done else "incomplete"))
        row["direct_reason"] = reason
    else:
        row["status"] = "startup_exception"
        row["direct_reason"] = startup_reason(launcher, task_text)
    persist(row)
    print(f"END phase={phase} sequence={sequence} condition={condition} rc={rc} action={int(action)} "
          f"transport={int(transport)} complete={int(done)} status={row['status']}", flush=True)
    return row


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in {"pretrial", "formal"}:
        print("usage: run_batch.py pretrial|formal", file=sys.stderr)
        return 2
    phase = sys.argv[1]
    if phase == "formal" and not (OUT / "PRETRIAL_GATE_PASS").is_file():
        print("formal run blocked: pretrial acceptance file is absent", file=sys.stderr)
        return 3
    rows = recover_interrupted(read_rows())
    schedule = load_schedule(phase)
    for item in schedule:
        rows = read_rows()
        condition = item["condition"]
        if phase == "formal":
            seq = int(item["sequence"])
            if any(r["phase"] == "formal" and int(r["sequence"]) == seq and r["action_started"] == "1"
                   for r in rows):
                continue
            if prior_transport_failures(rows, condition):
                print(f"PAUSED: last two formal {condition} attempts did not enter transport", file=sys.stderr)
                return 4
            prior_infra = [r for r in rows if r["phase"] == phase and int(r["sequence"]) == seq and
                           r["action_started"] == "0"]
        else:
            prior = [r for r in rows if r["phase"] == phase and r["condition"] == condition]
            if any(r["action_started"] == "1" for r in prior):
                continue
            prior_infra = [r for r in prior if r["action_started"] == "0"]

        # After three identical infrastructure errors, one corrected launch may be
        # attempted only after its cause and correction are documented in this batch.
        max_launches = 4 if (OUT / "STARTUP_DIAGNOSIS.md").is_file() else 3
        launch = len(prior_infra) + 1
        if launch > max_launches:
            print(f"PAUSED after three startup exceptions for {condition}", file=sys.stderr)
            return 5
        while launch <= max_launches:
            row = run_one(phase, item, launch)
            if row["status"] != "startup_exception":
                break
            rows = read_rows()
            consecutive = []
            for old in reversed([r for r in rows if r["phase"] == phase]):
                if old["status"] != "startup_exception":
                    break
                consecutive.append(old["direct_reason"])
            if len(consecutive) >= 3 and len(set(consecutive[:3])) == 1:
                print(f"PAUSED after three consecutive same startup issue: {consecutive[0]}",
                      file=sys.stderr)
                return 5
            infra = [r for r in rows if r["phase"] == phase and
                     ((phase == "pretrial" and r["condition"] == condition) or
                      (phase == "formal" and int(r["sequence"]) == int(item["sequence"]))) and
                     r["action_started"] == "0"]
            same_issue = sum(r["direct_reason"] == row["direct_reason"] for r in infra[-3:])
            if same_issue >= 3:
                print(f"PAUSED after three same startup exceptions: {row['direct_reason']}",
                      file=sys.stderr)
                return 5
            launch += 1
        if launch > max_launches and row["status"] == "startup_exception":
            print(f"PAUSED after three startup exceptions for {condition}: {row['direct_reason']}",
                  file=sys.stderr)
            return 5
        # A formal failure after the first accepted pre-grasp remains the sample; continue by design.

    rows = read_rows()
    if phase == "pretrial":
        pre = [r for r in rows if r["phase"] == "pretrial" and r["action_started"] == "1"]
        passed = (len({r["condition"] for r in pre}) == 2 and
                  all(r["status"] == "pretrial_completed" for r in pre) and
                  all((RUNS / r["run_id"] / "abcd_reference.csv").is_file() for r in pre))
        if passed:
            print("PRETRIAL_RUNS_COMPLETE: analyze reference match, force timing and metric coverage before gate.")
            return 0
        print("PRETRIAL_GATE_NOT_MET: inspect both pretrials; formal mode remains blocked.", file=sys.stderr)
        return 6
    started = [r for r in rows if r["phase"] == "formal" and r["action_started"] == "1"]
    print(f"FORMAL_ACTION_ATTEMPTS={len(started)}/20; completed={sum(r['flow_completed']=='1' for r in started)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
