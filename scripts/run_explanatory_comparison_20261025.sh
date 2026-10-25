#!/usr/bin/env bash
# Small, separate matched-hold study; the original n=20 batch is untouched.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
out=experiment_results/reports/explanatory_20261025
mkdir -p "$out"
schedule="$out/schedule.csv"
ledger="$out/attempts.csv"
if [[ ! -f "$schedule" ]]; then
  cat > "$schedule" <<'EOF'
block,condition,sample,group,payload
1,P,P1,A,0
1,T,T1,B,0
1,I0,I01,C,0
1,I1,I11,C,1
2,I1,I12,C,1
2,I0,I02,C,0
2,T,T2,B,0
2,P,P2,A,0
3,T,T3,B,0
3,P,P3,A,0
3,I1,I13,C,1
3,I0,I03,C,0
EOF
fi
if [[ ! -f "$ledger" ]]; then
  echo 'block,condition,sample,run_id,wrapper_exit,formal,run_dir' > "$ledger"
fi
export CONTACT_PROBE_START_X=0.50 CONTACT_PROBE_START_Y=0.00 CONTACT_PROBE_START_YAW_DEG=10
export CONTACT_PROBE_TARGET_X=0.40 CONTACT_PROBE_TARGET_Y=0.00 CONTACT_PROBE_TARGET_YAW_DEG=20
export CONTACT_TRANSPORT_STRETCH=2.0 CONTACT_TRANSPORT_TIME_SCALE=1.0
export CONTACT_TRANSPORT_PLAN_ONLY=0 CONTACT_PROBE_HOVER_ONLY=0 CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY=0
export CONTACT_PROBE_HOVER_CONTROL_MODE=jtc CONTACT_PROBE_MMC_CONTROL_VARIANT=fixed
export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false CONTACT_PROBE_PAYLOAD_FEEDFORWARD=0
export CONTACT_PROBE_ROD_DISTURBANCE=0 CONTACT_ABCD_MATCHED_WAIT_SEC=8.0
unset CONTACT_TRANSPORT_PAIRING CONTACT_PROBE_RIGID
sequence=0
while IFS=, read -r block condition sample group payload; do
  [[ "$block" == block || -z "$block" ]] && continue
  sequence=$((sequence+1))
  if awk -F, -v s="$sample" 'NR>1 && $3==s && $6==1 {found=1} END {exit !found}' "$ledger"; then
    continue
  fi
  export CONTACT_PROBE_ABCD_GROUP="$group" CONTACT_ABCD_PAYLOAD_FEEDFORWARD="$payload"
  prior=$(awk -F, -v s="$sample" 'NR>1 && $3==s {n++} END {print n+0}' "$ledger")
  limit=$((prior+3))
  for ((attempt=prior+1;attempt<=limit;attempt++)); do
    export CONTACT_PROBE_DOMAIN_ID=$((140+sequence*4+attempt))
    export CONTACT_PROBE_RUN_ID="explanatory_20261025_${sample}_attempt${attempt}"
    run_dir="experiment_results/a1_runs/$CONTACT_PROBE_RUN_ID"
    [[ ! -e "$run_dir" ]] || { echo "Existing run: $run_dir" >&2; exit 2; }
    log="$out/${CONTACT_PROBE_RUN_ID}_wrapper.log"
    echo "START block=$block condition=$condition sample=$sample run=$CONTACT_PROBE_RUN_ID"
    bash scripts/run_contact_probe_local.sh --headless > "$log" 2>&1
    rc=$?
    formal=$(python3 - "$run_dir/task_launch.log" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]);s=p.read_text(errors="replace") if p.exists() else ""
i=s.find("[P3A-1] 下发轨迹")
print(int(i>=0 and "Execute request accepted" in s[i:]))
PY
)
    printf '%s,%s,%s,%s,%s,%s,%s\n' "$block" "$condition" "$sample" "$CONTACT_PROBE_RUN_ID" "$rc" "$formal" "$run_dir" >> "$ledger"
    echo "END sample=$sample exit=$rc formal=$formal"
    if [[ "$formal" == 1 ]]; then
      if rg -q 'E_ABCD_MATCHED_WAIT_OVERRUN|E_ABCD_MATCHED_WAIT_CLOCK_TIMEOUT' "$run_dir/task_launch.log"; then
        echo 'Matched timing is not valid; stop to inspect instead of changing the frozen wait.'
        exit 3
      fi
      break
    fi
    if ((attempt==limit)); then echo "Three startup attempts failed for $sample"; exit 4; fi
  done
done < "$schedule"
echo ALL_12_FORMAL_ATTEMPTS_RECORDED
