#!/usr/bin/env bash
# Sequential, one-run-per-schedule-row launcher for this frozen hover batch.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$ROOT/experiment_results/reports/compliance_hover_20261025_r1"
cd "$ROOT"

export CONTACT_PROBE_START_X=0.50 CONTACT_PROBE_START_Y=0.00 CONTACT_PROBE_START_YAW_DEG=10
export CONTACT_PROBE_HOVER_ONLY=1 CONTACT_PROBE_HOVER_HEIGHT_M=0.05
export CONTACT_PROBE_HOVER_CONTROL_MODE=mmc_hold CONTACT_PROBE_RIGID=true
export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false CONTACT_PROBE_ABCD_GROUP=none
export CONTACT_PROBE_ROD_FORCE_DURATION_SEC=1.0
unset CONTACT_ABCD_MATCHED_WAIT_SEC CONTACT_ABCD_PAYLOAD_FEEDFORWARD

seq=0
while IFS=, read -r study block condition sample; do
  [[ "$study" == study || -z "$study" ]] && continue
  sample="${sample%$'\r'}"
  seq=$((seq + 1))
  if awk -F, -v s="$sample" 'NR>1 && $4==s && $7==1 {found=1} END {exit !found}' "$OUT/attempts.csv"; then
    continue
  fi
  prior=$(awk -F, -v s="$sample" 'NR>1 && $4==s {n++} END {print n+0}' "$OUT/attempts.csv")
  attempt=$((prior + 1))
  safe_condition="${condition//-/_}"
  run_id="compliance_hover_20261025_r1_${study}_${safe_condition}_b${block}_a${attempt}"
  run_dir="experiment_results/a1_runs/$run_id"
  if [[ -e "$run_dir" ]]; then echo "Refusing existing run directory: $run_dir" >&2; exit 2; fi

  case "$study:$condition" in
    feedforward:F0) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=0 CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed CONTACT_PROBE_ROD_DISTURBANCE=0 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.5 ;;
    feedforward:F1) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=1 CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed CONTACT_PROBE_ROD_DISTURBANCE=0 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.5 ;;
    directional:K400-N) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=1 CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed CONTACT_PROBE_ROD_DISTURBANCE=0 CONTACT_PROBE_HOVER_HOLD_SECONDS=5.0 ;;
    directional:K400-F) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=1 CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed CONTACT_PROBE_ROD_DISTURBANCE=1 CONTACT_PROBE_HOVER_HOLD_SECONDS=5.0 ;;
    directional:K200-N) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=1 CONTACT_PROBE_MMC_CONTROL_VARIANT=directional CONTACT_PROBE_ROD_DISTURBANCE=0 CONTACT_PROBE_HOVER_HOLD_SECONDS=5.0 ;;
    directional:K200-F) export CONTACT_PROBE_PAYLOAD_FEEDFORWARD=1 CONTACT_PROBE_MMC_CONTROL_VARIANT=directional CONTACT_PROBE_ROD_DISTURBANCE=1 CONTACT_PROBE_HOVER_HOLD_SECONDS=5.0 ;;
    *) echo "Unexpected frozen schedule row: $study/$condition" >&2; exit 2 ;;
  esac
  export CONTACT_PROBE_DOMAIN_ID=$((100 + (seq + attempt * 60) % 120))
  export CONTACT_PROBE_RUN_ID="$run_id"
  export ROS_HOME="/tmp/${run_id}_ros_home"
  log="$OUT/${run_id}_wrapper.log"
  echo "START study=$study block=$block condition=$condition sample=$sample run=$run_id"
  bash scripts/run_contact_probe_local.sh --headless > "$log" 2>&1
  rc=$?
  result=$(python3 - "$run_dir" "$study" "$condition" <<'PY'
import csv, json, re, sys
from pathlib import Path
d=Path(sys.argv[1]); study, condition=sys.argv[2:]
log=(d/'task_launch.log').read_text(errors='replace') if (d/'task_launch.log').exists() else ''
accepted=bool(re.search(r'\[P3A-1\].*下发轨迹.*\n.*Execute request accepted', log))
state=''
try: state=json.loads((d/'prepare_status.json').read_text()).get('task_state','')
except Exception: pass
errors=re.findall(r'(?:E_[A-Z0-9_]+|\[CONTACT_HOVER_RESULT\].*)', log)
failure=errors[-1] if errors and state not in ('DONE','done') else ''
trace=d/'mmc_hold_trace.csv'; covered=0.0
if trace.exists():
    try:
        rows=list(csv.DictReader(trace.open()))
        covered=float(rows[-1]['sim_elapsed_sec'])-float(rows[0]['sim_elapsed_sec']) if len(rows)>1 else 0.0
    except Exception: pass
required=3.5 if study=='feedforward' else 5.0
window=int('[MMC_HOVER_WINDOW]' in log and trace.exists() and covered >= required-0.12)
events=d/'rod_force_events.csv'; force='na'
if study=='directional' and condition.endswith('-F'):
    force=0
    if events.exists():
        names={r.get('event') for r in csv.DictReader(events.open())}
        force=int({'pulse_start','pulse_end'}.issubset(names))
payload=int('[PAYLOAD_FF] enabled=1' in log)
print(','.join(map(str,[int(accepted),state,failure.replace(',',';'),window,force,payload,covered])))
PY
)
  IFS=, read -r formal state failure window force payload covered <<< "$result"
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$study" "$block" "$condition" "$sample" "$run_id" "$rc" "$formal" "$state" "$failure" \
    "$window" "$force" "$run_dir" >> "$OUT/attempts.csv"
  echo "END sample=$sample exit=$rc formal=$formal state=$state window=$window force_events=$force coverage=${covered}s"

  # Stop after a pre-action infrastructure exception or a missing observation
  # window so its cause can be assessed before the next formal attempt.
  if [[ "$formal" != 1 ]]; then echo "STOP: pre-formal exception retained; inspect $run_dir and resume only after diagnosis."; exit 4; fi
  if [[ "$window" != 1 ]]; then
    previous=$(awk -F, -v current="$run_id" 'NR>1 && $7==1 && $5!=current {last=$9; last_window=$10} END {if (last_window==0) print last}' "$OUT/attempts.csv")
    if [[ -n "$previous" && "$previous" == "$failure" ]]; then
      echo "STOP: two consecutive formal runs lacked the MMC window for the same reason: $failure"; exit 5
    fi
    echo "RETAINED formal failure without MMC window; proceeding to next scheduled sample: $failure"
  fi
  if [[ "$study" == directional && "$condition" == *-F && "$force" != 1 ]]; then
    echo "STOP: physical pulse events are incomplete; inspect before continuing."; exit 6
  fi
done < "$OUT/schedule.csv"
echo ALL_SCHEDULED_RUNS_RECORDED
