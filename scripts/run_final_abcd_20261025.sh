#!/usr/bin/env bash
# Twelve predeclared visible attempts. No retries and no parallel simulators.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export CONTACT_PROBE_START_X=0.50 CONTACT_PROBE_START_Y=0.00 CONTACT_PROBE_START_YAW_DEG=10
export CONTACT_PROBE_TARGET_X=0.40 CONTACT_PROBE_TARGET_Y=0.00 CONTACT_PROBE_TARGET_YAW_DEG=20
export CONTACT_TRANSPORT_STRETCH=2.0 CONTACT_TRANSPORT_TIME_SCALE=1.0
export CONTACT_TRANSPORT_PLAN_ONLY=0 CONTACT_PROBE_HOVER_ONLY=0 CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY=0
export CONTACT_PROBE_HOVER_CONTROL_MODE=jtc CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed
export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false CONTACT_PROBE_PAYLOAD_FEEDFORWARD=0
export CONTACT_PROBE_ROD_DISTURBANCE=0
unset CONTACT_TRANSPORT_PAIRING
out=experiment_results/reports/final_abcd_20261025
mkdir -p "$out"
order=(A1 C1 B1 D1 D2 B2 C2 A2 B3 A3 D3 C3)
for tag in "${order[@]}"; do
  if [[ -f "$out/attempt_exits.csv" ]] && grep -q "^${tag}," "$out/attempt_exits.csv"; then continue; fi
  run_id="final_abcd_20261025_${tag}"
  if [[ "$tag" == C1 ]]; then run_id="${CONTACT_BATCH_C1_RUN_ID:-$run_id}"; fi
  if [[ -e "experiment_results/a1_runs/${run_id}" ]]; then
    echo "Existing run directory for $tag; refusing to overwrite or rerun." >&2
    exit 2
  fi
done
if [[ ! -f "$out/attempt_exits.csv" ]]; then
  printf 'sample,run_id,wrapper_exit\n' > "$out/attempt_exits.csv"
fi
for tag in "${order[@]}"; do
  if grep -q "^${tag}," "$out/attempt_exits.csv"; then continue; fi
  export CONTACT_PROBE_ABCD_GROUP="${tag:0:1}"
  export CONTACT_PROBE_RUN_ID="final_abcd_20261025_${tag}"
  if [[ "$tag" == C1 ]]; then export CONTACT_PROBE_RUN_ID="${CONTACT_BATCH_C1_RUN_ID:-$CONTACT_PROBE_RUN_ID}"; fi
  echo "[FINAL_BATCH] START $tag $CONTACT_PROBE_RUN_ID"
  bash scripts/run_contact_probe_local.sh --use-rviz > "$out/${CONTACT_PROBE_RUN_ID}_wrapper.log" 2>&1
  result=$?
  printf '%s,%s,%s\n' "$tag" "$CONTACT_PROBE_RUN_ID" "$result" >> "$out/attempt_exits.csv"
  echo "[FINAL_BATCH] END $tag wrapper_exit=$result"
done
echo '[FINAL_BATCH] ALL_TWELVE_ATTEMPTS_FINISHED'
