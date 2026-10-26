#!/usr/bin/env python3
"""Sequential runner for the fixed full-carry A/B disturbance comparison."""
from __future__ import annotations

import csv
import os
import re
import signal
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUNS = ROOT / "experiment_results/a1_runs"
LEDGER = OUT / "attempts.csv"
PREFIX = "fullcarry_perturb_ab_20261026"
COMMON_DURATION = "3.040"
COMMON_CONNECTION = "0.5"
FIELDS = [
    "phase", "sequence", "block", "condition", "internal_group", "launch_attempt", "run_id",
    "domain_id", "start_x_m", "start_y_m", "start_yaw_deg", "target_x_m", "target_y_m",
    "target_yaw_deg", "wrapper_exit", "action_started", "transport_started", "main_started",
    "pulse_started", "pulse_ended", "flow_completed", "task_state", "status", "direct_reason",
    "run_dir", "launcher_log",
]


def rows():
    if not LEDGER.exists():
        return []
    with LEDGER.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def save(row):
    all_rows = rows()
    all_rows.append(row)
    tmp = LEDGER.with_suffix(".csv.tmp")
    with tmp.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(all_rows)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(LEDGER)


def inspect_run(run_dir: Path):
    log = run_dir / "task_launch.log"
    content = log.read_text(errors="replace") if log.exists() else ""
    lines = content.splitlines()
    action_line = next((i for i, line in enumerate(lines) if "[P3A-1] 下发轨迹" in line), None)
    action = action_line is not None and any("Execute request accepted" in line
                                             for line in lines[action_line:])
    transport = "[TRANSPORT_EVENT] event=start" in content
    main = "[TRANSPORT_MAIN_EVENT] event=start" in content
    done = ("[CONTACT_PROBE_RESULT] state=DONE" in content and
            "[CONTACT_PROBE_RELEASED]" in content and "[CONTACT_PROBE_RETREAT]" in content)
    states = re.findall(r"\[CONTACT_PROBE_RESULT\] state=([^ ]+)", content)
    reasons = re.findall(r"\[CONTACT_PROBE_FAILURE\] ([^\r\n]+)", content)
    if not reasons:
        reasons = re.findall(r"\[CONTACT_PROBE_RESULT\] state=FAILED detail=([^\r\n]+)", content)
    force_file = run_dir / "rod_force_events.csv"
    force_content = force_file.read_text(errors="replace") if force_file.exists() else ""
    return (action, transport, main, "pulse_start" in force_content, "pulse_end" in force_content,
            done, states[-1] if states else "", reasons[-1] if reasons else "", content)


def startup_reason(wrapper: Path, task_text: str):
    data = wrapper.read_text(errors="replace") if wrapper.exists() else ""
    for line in data.splitlines():
        lower = line.lower()
        if "error:" in lower or "failed" in lower or "timeout" in lower:
            return line.strip()[:240]
    for line in task_text.splitlines():
        lower = line.lower()
        if "preflight" in lower or "startup" in lower or "timeout" in lower:
            return line.strip()[:240]
    return "wrapper returned before accepted pre-grasp motion"


def run_one(phase: str, item: dict[str, str], launch_attempt: int):
    sequence = int(item["sequence"])
    condition = item["condition"]
    label = condition if phase.startswith("pretrial") else f"{condition}{sequence:02d}"
    run_id = f"{PREFIX}_{phase}_{label}_i{launch_attempt}"
    run_dir = RUNS / run_id
    wrapper = OUT / f"{run_id}_wrapper.log"
    ros_logs = ROOT / "launch_logs" / run_id
    if run_dir.exists() or ros_logs.exists():
        raise RuntimeError(f"refusing to reuse existing run output: {run_id}")
    domain = 60 + len(rows())
    row = {key: "" for key in FIELDS}
    row.update(
        phase=phase, sequence=str(sequence), block=item.get("block", "pre"),
        condition=condition, internal_group=item["internal_group"],
        launch_attempt=str(launch_attempt), run_id=run_id, domain_id=str(domain),
        start_x_m=item["start_x_m"], start_y_m=item["start_y_m"],
        start_yaw_deg=item["start_yaw_deg"], target_x_m=item["target_x_m"],
        target_y_m=item["target_y_m"], target_yaw_deg=item["target_yaw_deg"],
        wrapper_exit="", action_started="0", transport_started="0", main_started="0",
        pulse_started="0", pulse_ended="0", flow_completed="0", task_state="",
        status="reserved", direct_reason="", run_dir=str(run_dir.relative_to(ROOT)),
        launcher_log=str(wrapper.relative_to(ROOT)),
    )
    # Reserve before launching so an interrupted session cannot reuse this directory.
    save(row)
    env = os.environ.copy()
    env.update({
        "CONTACT_PROBE_DOMAIN_ID": str(domain), "CONTACT_PROBE_RUN_ID": run_id,
        "CONTACT_PROBE_START_X": item["start_x_m"], "CONTACT_PROBE_START_Y": item["start_y_m"],
        "CONTACT_PROBE_START_YAW_DEG": item["start_yaw_deg"],
        "CONTACT_PROBE_TARGET_X": item["target_x_m"], "CONTACT_PROBE_TARGET_Y": item["target_y_m"],
        "CONTACT_PROBE_TARGET_YAW_DEG": item["target_yaw_deg"],
        "CONTACT_PROBE_ABCD_GROUP": item["internal_group"],
        "CONTACT_ABCD_PAYLOAD_FEEDFORWARD": "1" if condition == "B" else "0",
        "CONTACT_PROBE_PAYLOAD_FEEDFORWARD": "0", "CONTACT_PROBE_HOVER_CONTROL_MODE": "jtc",
        "CONTACT_PROBE_MMC_CONTROL_VARIANT": "fixed", "CONTACT_PROBE_ROD_DISTURBANCE": "1",
        "CONTACT_PROBE_ROD_FORCE_DURATION_SEC": "0.5",
        "CONTACT_PROBE_ADAPTIVE_FORCE_HOLD": "false", "CONTACT_ABCD_MATCHED_WAIT_SEC": "0",
        "CONTACT_TRANSPORT_STRETCH": "2.0", "CONTACT_TRANSPORT_TIME_SCALE": "1.0",
        "CONTACT_TRANSPORT_PLAN_ONLY": "0", "CONTACT_PROBE_HOVER_ONLY": "0",
        "CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY": "0", "CONTACT_TRANSPORT_COMMON_REFERENCE": "1",
        "CONTACT_TRANSPORT_COMMON_DURATION_SEC": COMMON_DURATION,
        "CONTACT_TRANSPORT_CONNECTION_SEC": COMMON_CONNECTION,
    })
    env.pop("CONTACT_TRANSPORT_PAIRING", None)
    env.pop("CONTACT_PROBE_RIGID", None)
    print(f"START phase={phase} sequence={sequence} condition={condition} run={run_id}", flush=True)
    with wrapper.open("w", encoding="utf-8") as output:
        proc = subprocess.Popen(["bash", "scripts/run_contact_probe_local.sh", "--headless"],
                                cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try:
            rc = proc.wait()
        except KeyboardInterrupt:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=10)
            raise
    action, transport, main, pulse_start, pulse_end, done, state, reason, task_text = inspect_run(run_dir)
    row.update(wrapper_exit=str(rc), action_started=str(int(action)),
               transport_started=str(int(transport)), main_started=str(int(main)),
               pulse_started=str(int(pulse_start)), pulse_ended=str(int(pulse_end)),
               flow_completed=str(int(done)), task_state=state,
               status=("completed" if done else "incomplete") if action else "startup_exception",
               direct_reason=reason if action else startup_reason(wrapper, task_text))
    # Replace the reservation row with the final record.
    all_rows = rows()
    for index in range(len(all_rows)-1, -1, -1):
        if all_rows[index]["run_id"] == run_id:
            all_rows[index] = row
            break
    tmp = LEDGER.with_suffix(".csv.tmp")
    with tmp.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(all_rows)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(LEDGER)
    print(f"END phase={phase} sequence={sequence} condition={condition} action={int(action)} "
          f"main={int(main)} pulse={int(pulse_start and pulse_end)} done={int(done)} "
          f"status={row['status']}", flush=True)
    return row


def schedule(phase):
    path = OUT / ("pretrial_recheck_schedule.csv" if phase == "pretrial_recheck" else
                  "pretrial_schedule.csv" if phase == "pretrial" else "schedule.csv")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def execute_phase(phase):
    schedule_rows = schedule(phase)
    all_rows = rows()
    for item in schedule_rows:
        matching = [row for row in all_rows if row["phase"] == phase and
                    row["sequence"] == item["sequence"] and row["condition"] == item["condition"]]
        if any(row["action_started"] == "1" for row in matching):
            continue
        if any(row["status"] == "reserved" for row in matching):
            print(f"PAUSED: an earlier reservation is unresolved for {phase} {item['condition']} {item['sequence']}",
                  file=sys.stderr)
            return 3
        launches = sum(row["status"] == "startup_exception" for row in matching)
        while True:
            if launches >= 3:
                print(f"PAUSED after three pre-action infrastructure exceptions: {item['condition']} {item['sequence']}",
                      file=sys.stderr)
                return 4
            row = run_one(phase, item, launches+1)
            launches += int(row["status"] == "startup_exception")
            if row["status"] != "startup_exception":
                break
            latest = rows()
            last_three = [r["direct_reason"] for r in latest if r["phase"] == phase][-3:]
            if len(last_three) == 3 and len(set(last_three)) == 1:
                print(f"PAUSED after three identical startup exceptions: {last_three[0]}", file=sys.stderr)
                return 4
            if launches >= 3:
                print(f"PAUSED after three pre-action infrastructure exceptions: {item['condition']} {item['sequence']}",
                      file=sys.stderr)
                return 4
        all_rows = rows()
    return 0


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in {"pretrial", "pretrial_recheck", "formal"}:
        print("usage: python3 run_batch.py pretrial|pretrial_recheck|formal", file=sys.stderr)
        return 2
    phase = sys.argv[1]
    if phase == "formal":
        pre = [r for r in rows() if r["phase"].startswith("pretrial") and r["action_started"] == "1"]
        latest = {condition: next((r for r in reversed(pre) if r["condition"] == condition), None)
                  for condition in ("A", "B")}
        if any(r is None or r["status"] != "completed" or r["main_started"] != "1" or
               r["pulse_started"] != "1" or r["pulse_ended"] != "1" for r in latest.values()):
            print("formal mode requires complete, measured A/B pretrials with the force pulse recorded",
                  file=sys.stderr)
            return 3
        review = OUT / "pretrial_review.md"
        if not review.is_file():
            print("formal mode waits for the pretrial data review", file=sys.stderr)
            return 3
    return execute_phase(phase)


if __name__ == "__main__":
    raise SystemExit(main())
