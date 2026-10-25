#!/usr/bin/env bash
# Sequential, resumable headless A/B/C/D batch. Each invocation runs one simulator.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

out=experiment_results/reports/abcd_n20_20261025
mkdir -p "$out"
schedule="$out/schedule.csv"
ledger="$out/attempts.csv"
if [[ ! -f "$schedule" ]]; then
  echo 'round,position,group,sample' > "$schedule"
  python3 - <<'PY' >> "$schedule"
import random
rng = random.Random(20261025)
for rnd in range(1, 21):
    groups = list('ABCD')
    rng.shuffle(groups)
    for pos, group in enumerate(groups, 1):
        print(f'{rnd},{pos},{group},{group}{rnd:02d}')
PY
fi
if [[ ! -f "$ledger" ]]; then
  echo 'sample,group,run_id,wrapper_exit,run_dir,kind' > "$ledger"
fi

export CONTACT_PROBE_START_X=0.50 CONTACT_PROBE_START_Y=0.00 CONTACT_PROBE_START_YAW_DEG=10
export CONTACT_PROBE_TARGET_X=0.40 CONTACT_PROBE_TARGET_Y=0.00 CONTACT_PROBE_TARGET_YAW_DEG=20
export CONTACT_TRANSPORT_STRETCH=2.0 CONTACT_TRANSPORT_TIME_SCALE=1.0
export CONTACT_TRANSPORT_PLAN_ONLY=0 CONTACT_PROBE_HOVER_ONLY=0 CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY=0
export CONTACT_PROBE_HOVER_CONTROL_MODE=jtc CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed
export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false CONTACT_PROBE_PAYLOAD_FEEDFORWARD=0
export CONTACT_PROBE_ROD_DISTURBANCE=0
unset CONTACT_TRANSPORT_PAIRING CONTACT_PROBE_RIGID

while IFS=, read -r round position group sample; do
  [[ "$round" == round || -z "$round" ]] && continue
  if awk -F, -v s="$sample" 'NR>1 && $1==s && $6!="startup_exception" {found=1} END{exit !found}' "$ledger"; then
    continue
  fi
  export CONTACT_PROBE_ABCD_GROUP="$group"
  base_run_id="abcd_n20_20261025_${sample}"
  retry_no=$(awk -F, -v s="$sample" 'NR>1 && $1==s {n++} END{print n+0}' "$ledger")
  export CONTACT_PROBE_RUN_ID="$base_run_id"
  if (( retry_no > 0 )); then export CONTACT_PROBE_RUN_ID="${base_run_id}_startup$((retry_no + 1))"; fi
  if [[ -e "experiment_results/a1_runs/$CONTACT_PROBE_RUN_ID" ]]; then
    echo "Refusing to overwrite existing run directory for $sample: $CONTACT_PROBE_RUN_ID" >&2
    exit 2
  fi
  log="$out/${CONTACT_PROBE_RUN_ID}_wrapper.log"
  echo "[N20] START round=$round position=$position sample=$sample run_id=$CONTACT_PROBE_RUN_ID"
  bash scripts/run_contact_probe_local.sh --headless > "$log" 2>&1
  rc=$?
  run_dir="experiment_results/a1_runs/$CONTACT_PROBE_RUN_ID"
  kind=needs_classification
  if [[ -d "$run_dir" ]]; then kind=action_or_started; else kind=startup_or_wrapper; fi
  printf '%s,%s,%s,%s,%s,%s\n' "$sample" "$group" "$CONTACT_PROBE_RUN_ID" "$rc" "$run_dir" "$kind" >> "$ledger"
  echo "[N20] END round=$round sample=$sample wrapper_exit=$rc kind=$kind"
done < "$schedule"
echo '[N20] ALL_SCHEDULED_ATTEMPTS_HAVE_LEDGER_ENTRIES'
