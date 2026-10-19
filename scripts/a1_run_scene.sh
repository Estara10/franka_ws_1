#!/usr/bin/env bash
#
# A1 P1a: non-interactive single-candidate scene runner.
#
# Applies ONE sampled rod pose to a freshly started simulation instance, waits
# for it to settle, and writes a manifest. It does not run the carrying task,
# does not publish a truth topic, and does not touch the planning scene.
#
# Ownership and record rules:
#   * a dedicated ROS domain is used, and the script refuses to start if an
#     instance already answers on it; it never cleans up processes it does not
#     own
#   * the run directory is reserved exclusively before any side effect, so an
#     existing run is never overwritten or truncated
#   * exactly one simulation instance is started
#   * the script owns every process it starts and signals ONLY its own process
#     group on exit; `pkill`/`killall` are never used
#   * every discovery command is bounded by a wall-clock timeout
#
# File ownership: the runner node writes `manifest.json` and
# `settle_samples.csv`; this script writes `sim_dual_moveit.log`, `runner.log`,
# `services_seen.txt`, `controllers.txt`, `sim_stack_deaths*.txt` and, when the
# script itself fails, `script_status.json`.
#
# Usage:
#   ./scripts/a1_run_scene.sh --dry-run --seed 20261011 --candidate-index 0
#   ./scripts/a1_run_scene.sh --seed 20261011 --candidate-index 0 --run-id a1_x
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CANDIDATE_INDEX=0
RUN_ID=""
DRY_RUN=0
SEED=""
# Explicit ROS session isolation: a dedicated domain, localhost only.
DOMAIN_ID="${A1_ROS_DOMAIN_ID:-77}"
USE_RVIZ="false"
HEADLESS="false"
OBSERVE_ON_ERROR=0
READY_TIMEOUT=120
NODE_TIMEOUT=180
HELPER_TIMEOUT=20
MANIFEST_DIR="${WS_DIR}/experiment_results/a1_runs"
PARAMS_FILE=""
PKG_SHARE=""
EX_X=""
EX_Y=""
EX_YAW=""
EXPLICIT_FIELDS=0
# A1 P1b: prepare-mode (no motion). Same simulation instance does apply ->
# publish truth -> task freezes and syncs the planning scene -> PREPARED.
PREPARE_MODE=0
TRUTH_TOPIC="/a1_rod_pose_truth"
HOLD_SEC=300
TASK_TIMEOUT=180
# A1 P2a: static validity checks (geometry / dual-arm IK / collision) run
# inside the prepare boundary. Off by default, so P1b behaviour is unchanged.
P2A_CHECK=0
# A1 P2b: joint pre-grasp path planning + verification. Planning only, no motion.
P2B_CHECK=0
# A1 P3a-1: execute the verified pre-grasp plan, then stop. Implies --p2b-check.
PREGRASP_EXECUTE=0
GRASP_DESCENT=0
GRASP_CONTACT_CLOSE=0
GRASP_MODEL="none"
CONTACT_HOLD_DIAGNOSTIC_ONLY="${CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY:-0}"
CONTACT_HOVER_ONLY=0
CONTACT_HOVER_HEIGHT_M="${CONTACT_PROBE_HOVER_HEIGHT_M:-0.05}"
CONTACT_HOVER_HOLD_SECONDS="${CONTACT_PROBE_HOVER_HOLD_SECONDS:-3.0}"
DESCENT_NEGATIVE_FINAL=""
DESCENT_NEGATIVE_TRUTH=""
# A1 P3a-2a: gripper interface check -- one empty-load open/close/open cycle.
GRIPPER_SELFTEST=0
P2A_WORKSPACE=""
P2A_WORKSPACE_SOURCE=""
TASK_TARGET_X=""
TASK_TARGET_Y=""
TASK_TARGET_YAW_DEG=""
TASK_TARGET_FIELDS=0

usage() {
  cat <<'EOF'
usage: a1_run_scene.sh [options]

  --candidate-index N   candidate index to apply (default 0)
  --run-id ID           run identifier; also the output directory name
  --dry-run             print candidates and write a manifest; never starts a
                        simulator and never calls a service
  --seed N              override the configured seed
  --x M --y M --yaw-deg DEG
                        apply this KNOWN pose instead of a sampled one (all
                        three required together). Used for verification, e.g.
                        yaw exactly 0, which the sampler cannot produce.
  --prepare-mode        A1 P1b: within this one simulation instance, apply the
                        pose, publish the settled truth, then run the task in
                        prepare-only mode until it reports PREPARED. No motion.
  --truth-topic T       truth topic (default /a1_rod_pose_truth)
  --hold-sec S          how long the truth publisher stays alive (default 300)
  --task-timeout SEC    wall-clock bound for the task reaching PREPARED (default 180)
  --p2a-check           A1 P2a: after the planning scene is synced, also run the
                        static geometry / dual-arm IK / combined-collision
                        checks on the frozen truth. Still no motion. The exit
                        code then reflects the P2a result, not just PREPARED.
  --p2a-workspace XMIN XMAX YMIN YMAX
                        configured research envelope for the complete rod
                        footprint (metres). The support surface in this scene is
                        an unbounded MuJoCo plane, so this is a chosen research
                        region, NOT a physical table edge.
  --p2a-workspace-source S
                        provenance text for the envelope above, recorded verbatim
  --gripper-selftest    A1 P3a-2a: start one simulator and run a single dual-gripper
                        open -> empty close -> open cycle at the arm's default start
                        pose. The arms are not commanded; no weld/attach.
  --descent-negative-final M   A1 P3a-2b verification only: position | collision.
                        Deliberately damages the FINAL trajectory so the rejection
                        path can be shown to fire. Nothing is executed.
  --descent-negative-truth M   A1 P3a-2b verification only: unavailable | offset.
                        Makes the pre-descent rod truth unavailable or shifted so
                        the refusal path can be shown to fire. Nothing is executed.
  --grasp-descent       A1 P3a-2b: after the pre-grasp is reached, open both
                        grippers and descend along world Z to the grasp pose,
                        then stop. Implies --pregrasp-execute. The gripper stays
                        OPEN: nothing is closed, welded, attached, lifted or
                        carried. Exit code follows the P3A2B result.
  --grasp-contact-close A1 P3a-2c: after GRASP_POSE_REACHED, close progressively and
                        require fresh bilateral finger-to-rod contact; no weld/attach/lift/carry.
  --grasp-model contact_probe
                        Contact-only 5cm lift, 2s hold, lower and release; no weld, no full carry.
  --contact-hover-only  Contact-only suspended hold: lift, hold, lower and release; no transport.
  --contact-hover-height M  suspended hold height (default 0.05 m)
  --contact-hover-hold-sec S suspended hold duration (default 3 s)
  --grasp-model ideal_weld
                        S1: explicit ideal rigid grasp, after A1 verified descent complete
                        lift/transport/rotate/place/release/retreat; friction grip is not validated.
  --target-x M --target-y M --target-yaw-deg DEG
                        Optional explicit task target forwarded to the task launch;
                        omitted values retain the launch defaults.
  --pregrasp-execute    A1 P3a-1: also EXECUTE the same plan P2b just verified and
                        stop on arrival (no descent, no grasp, no carry). Implies
                        --p2b-check. Exit code follows the P3A result.
  --p2b-check           A1 P2b: also plan the dual-arm joint pre-grasp path from
                        the actual start and re-verify every interpolated state on
                        the P2a criterion model. Planning only, nothing is
                        executed. The exit code then follows p2_acceptance.
  --domain-id N         ROS_DOMAIN_ID for this run (default 77)
  --use-rviz            start RViz with the simulator (default: no)
  --observe-on-error    on a task ERROR, keep this visible simulator open until Enter/Ctrl+C
  --headless            run the simulator headless
  --ready-timeout SEC   wall-clock seconds to wait for the services
  --node-timeout SEC    hard wall-clock bound on the runner node (default 180)
  --manifest-dir DIR    manifest root directory
  --params-file FILE    override the installed config path
  -h, --help            this message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --candidate-index) CANDIDATE_INDEX="$2"; shift 2 ;;
    --run-id)          RUN_ID="$2"; shift 2 ;;
    --dry-run)         DRY_RUN=1; shift ;;
    --seed)            SEED="$2"; shift 2 ;;
    --x)               EX_X="$2"; EXPLICIT_FIELDS=$((EXPLICIT_FIELDS + 1)); shift 2 ;;
    --y)               EX_Y="$2"; EXPLICIT_FIELDS=$((EXPLICIT_FIELDS + 1)); shift 2 ;;
    --yaw-deg)         EX_YAW="$2"; EXPLICIT_FIELDS=$((EXPLICIT_FIELDS + 1)); shift 2 ;;
    --prepare-mode)    PREPARE_MODE=1; shift ;;
    --truth-topic)     TRUTH_TOPIC="$2"; shift 2 ;;
    --hold-sec)        HOLD_SEC="$2"; shift 2 ;;
    --task-timeout)    TASK_TIMEOUT="$2"; shift 2 ;;
    --p2a-check)       P2A_CHECK=1; shift ;;
    --p2b-check)       P2B_CHECK=1; P2A_CHECK=1; shift ;;
    --pregrasp-execute) PREGRASP_EXECUTE=1; P2B_CHECK=1; P2A_CHECK=1; shift ;;
    --grasp-descent)    GRASP_DESCENT=1; PREGRASP_EXECUTE=1; P2B_CHECK=1; P2A_CHECK=1; shift ;;
    --grasp-contact-close) GRASP_CONTACT_CLOSE=1; GRASP_DESCENT=1; PREGRASP_EXECUTE=1; P2B_CHECK=1; P2A_CHECK=1; shift ;;
    --grasp-model) GRASP_MODEL="$2"; shift 2 ;;
    --contact-hover-only) CONTACT_HOVER_ONLY=1; shift ;;
    --contact-hover-height) CONTACT_HOVER_HEIGHT_M="$2"; shift 2 ;;
    --contact-hover-hold-sec) CONTACT_HOVER_HOLD_SECONDS="$2"; shift 2 ;;
  --contact-hold-diagnostic-only) CONTACT_HOLD_DIAGNOSTIC_ONLY=1; shift ;;
    --target-x) TASK_TARGET_X="$2"; TASK_TARGET_FIELDS=$((TASK_TARGET_FIELDS + 1)); shift 2 ;;
    --target-y) TASK_TARGET_Y="$2"; TASK_TARGET_FIELDS=$((TASK_TARGET_FIELDS + 1)); shift 2 ;;
    --target-yaw-deg) TASK_TARGET_YAW_DEG="$2"; TASK_TARGET_FIELDS=$((TASK_TARGET_FIELDS + 1)); shift 2 ;;
    --descent-negative-final) DESCENT_NEGATIVE_FINAL="$2"; shift 2 ;;
    --descent-negative-truth) DESCENT_NEGATIVE_TRUTH="$2"; shift 2 ;;
    --gripper-selftest) GRIPPER_SELFTEST=1; shift ;;
    --p2a-workspace)   P2A_WORKSPACE="$2 $3 $4 $5"; shift 5 ;;
    --p2a-workspace-source) P2A_WORKSPACE_SOURCE="$2"; shift 2 ;;
    --domain-id)       DOMAIN_ID="$2"; shift 2 ;;
    --use-rviz)        USE_RVIZ="true"; shift ;;
    --observe-on-error) OBSERVE_ON_ERROR=1; shift ;;
    --headless)        HEADLESS="true"; shift ;;
    --ready-timeout)   READY_TIMEOUT="$2"; shift 2 ;;
    --node-timeout)    NODE_TIMEOUT="$2"; shift 2 ;;
    --manifest-dir)    MANIFEST_DIR="$2"; shift 2 ;;
    --params-file)     PARAMS_FILE="$2"; shift 2 ;;
    -h|--help)         usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 3 ;;
  esac
done

if [[ "${GRASP_MODEL}" != "none" && "${GRASP_MODEL}" != "ideal_weld" && "${GRASP_MODEL}" != "contact_probe" ]]; then
  echo "error: --grasp-model must be none, ideal_weld or contact_probe" >&2; exit 3
fi
if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 && "${GRASP_MODEL}" != "contact_probe" ]]; then
  echo "error: --contact-hold-diagnostic-only requires --grasp-model contact_probe" >&2
  exit 2
fi
if [[ "${CONTACT_HOVER_ONLY}" -eq 1 && "${GRASP_MODEL}" != "contact_probe" ]]; then
  echo "error: --contact-hover-only requires --grasp-model contact_probe" >&2
  exit 2
fi
if [[ "${GRASP_MODEL}" == "ideal_weld" || "${GRASP_MODEL}" == "contact_probe" ]]; then
  if [[ "${CONTACT_TRANSPORT_PLAN_ONLY:-0}" -eq 1 ]]; then
    # S2 plan-only: the static checks are needed, the execution entries are not.
    P2A_CHECK=1; P2B_CHECK=0; PREGRASP_EXECUTE=0; GRASP_DESCENT=0; GRASP_CONTACT_CLOSE=0
  else
    GRASP_DESCENT=1; PREGRASP_EXECUTE=1; P2B_CHECK=1; P2A_CHECK=1
  fi
  if [[ "${GRASP_CONTACT_CLOSE}" -eq 1 || "${GRIPPER_SELFTEST}" -eq 1 ]]; then
    echo "error: ideal_weld/contact_probe is exclusive with contact-close and gripper selftest" >&2; exit 3
  fi
fi

if ! [[ "${CANDIDATE_INDEX}" =~ ^[0-9]+$ ]]; then
  echo "error: --candidate-index must be a non-negative integer" >&2
  exit 3
fi
if [[ -n "${SEED}" ]] && ! [[ "${SEED}" =~ ^[0-9]+$ ]]; then
  echo "error: --seed must be a non-negative integer" >&2
  exit 3
fi
if [[ "${EXPLICIT_FIELDS}" -ne 0 && "${EXPLICIT_FIELDS}" -ne 3 ]]; then
  echo "error: --x, --y and --yaw-deg must be supplied together" >&2
  exit 3
fi
if [[ "${TASK_TARGET_FIELDS}" -ne 0 && "${TASK_TARGET_FIELDS}" -ne 3 ]]; then
  echo "error: --target-x, --target-y and --target-yaw-deg must be supplied together" >&2
  exit 3
fi

# --- ROS session isolation ---------------------------------------------------
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1

# --- Environment -------------------------------------------------------------
if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "error: ${WS_DIR}/install/setup.bash not found; build the workspace first" >&2
  exit 3
fi
# The workspace setup scripts reference unset variables (e.g. COLCON_TRACE).
set +u
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"
set -u

PKG_SHARE="$(ros2 pkg prefix dual_arm_carry_task 2>/dev/null || true)"
if [[ -z "${PKG_SHARE}" ]]; then
  echo "error: dual_arm_carry_task is not visible after sourcing the workspace" >&2
  exit 3
fi
PKG_SHARE="${PKG_SHARE}/share/dual_arm_carry_task"

if [[ -z "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="${PKG_SHARE}/config/a1_scene_sampling.yaml"
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "error: configuration not found: ${PARAMS_FILE}" >&2
  echo "       build/install dual_arm_carry_task first" >&2
  exit 3
fi

FRANKA_DESC_PREFIX="$(ros2 pkg prefix franka_description 2>/dev/null || true)"
MODELS_DIR=""
if [[ -n "${FRANKA_DESC_PREFIX}" ]]; then
  MODELS_DIR="${FRANKA_DESC_PREFIX}/share/franka_description/mujoco/franka"
fi
MOVEIT_PREFIX="$(ros2 pkg prefix franka_moveit_config 2>/dev/null || true)"
SRDF_PATH=""
if [[ -n "${MOVEIT_PREFIX}" ]]; then
  SRDF_PATH="${MOVEIT_PREFIX}/share/franka_moveit_config/srdf/dual_panda_generated.srdf"
fi

GIT_COMMIT="$(git -C "${WS_DIR}" rev-parse HEAD 2>/dev/null || true)"
GIT_DIRTY=0
if [[ -n "$(git -C "${WS_DIR}" status --porcelain 2>/dev/null)" ]]; then
  GIT_DIRTY=1
fi

if [[ -z "${RUN_ID}" ]]; then
  RUN_ID="a1_$(date -u +%Y%m%d_%H%M%S)"
fi
if ! [[ "${RUN_ID}" =~ ^[A-Za-z0-9_.-]+$ ]] || [[ "${RUN_ID}" == "." || "${RUN_ID}" == ".." ]]; then
  echo "error: --run-id may only contain letters, digits, '_', '-' and '.', and must not be '.' or '..'" >&2
  exit 3
fi
RUN_DIR="${MANIFEST_DIR}/${RUN_ID}"

# Reserve the run directory EXCLUSIVELY before any side effect. `mkdir` without
# -p fails when the directory exists, so an earlier run is never reused,
# truncated or overwritten.
mkdir -p "${MANIFEST_DIR}"
if ! mkdir "${RUN_DIR}" 2>/dev/null; then
  echo "error: run directory already exists, refusing to reuse it: ${RUN_DIR}" >&2
  exit 3
fi
# Tell the runner node that this fresh directory was reserved for it, so it does
# not refuse a directory that already exists. The node still refuses any other
# existing path, including a direct invocation.
: > "${RUN_DIR}/.reserved_by_a1_run_scene"

SCRIPT_STATUS=""
record_status() {
  # $1 = outcome code, $2 = detail, $3 = stage
  local code="$1" detail="$2" stage="$3"
  SCRIPT_STATUS="${code}"
  {
    printf '{\n'
    printf '  "written_by": "scripts/a1_run_scene.sh",\n'
    printf '  "run_id": "%s",\n' "${RUN_ID}"
    printf '  "stage": "%s",\n' "${stage}"
    printf '  "outcome_code": "%s",\n' "${code}"
    printf '  "detail": "%s",\n' "$(printf '%s' "${detail}" | tr -d '"\\')"
    printf '  "ros_domain_id": "%s",\n' "${ROS_DOMAIN_ID}"
    printf '  "ros_localhost_only": "1",\n'
    printf '  "manifest_present": %s\n' "$([[ -f "${RUN_DIR}/manifest.json" ]] && echo true || echo false)"
    printf '}\n'
  } > "${RUN_DIR}/script_status.json" 2>/dev/null || true
}

# P1b prepare-mode record: what the prepare flow observed. Written by the
# script; the runner node owns manifest.json / settle_samples.csv and the task
# owns task_launch.log.
record_prepare_status() {
  local task_state="$1"
  {
    printf '{\n'
    printf '  "written_by": "scripts/a1_run_scene.sh",\n'
    printf '  "run_id": "%s",\n' "${RUN_ID}"
    printf '  "mode": "prepare",\n'
    printf '  "task_state": "%s",\n' "${task_state}"
    printf '  "truth_topic": "%s",\n' "${TRUTH_TOPIC}"
    printf '  "expected_run_id": "%s",\n' "${RUN_ID}"
    printf '  "ros_domain_id": "%s",\n' "${ROS_DOMAIN_ID}"
    printf '  "truth_published": %s,\n' "$(grep -q 'truth published' "${RUN_DIR}/runner.log" 2>/dev/null && echo true || echo false)"
    printf '  "manifest_present": %s,\n' "$([[ -f "${RUN_DIR}/manifest.json" ]] && echo true || echo false)"
    printf '  "p2a_check_enabled": %s,\n' "$([[ "${P2A_CHECK}" -eq 1 ]] && echo true || echo false)"
    printf '  "p2a_result_line": "%s",\n' "$(grep -m1 '\[P2A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[P2A_RESULT\]/[P2A_RESULT]/' | tr -d '"\\')"
    printf '  "p2a_static_passed": %s,\n' "$(p2a_static_passed && echo true || echo false)"
    printf '  "p2b_check_enabled": %s,\n' "$([[ "${P2B_CHECK}" -eq 1 ]] && echo true || echo false)"
    printf '  "p2b_result_line": "%s",\n' "$(grep -m1 '\[P2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[P2B_RESULT\]/[P2B_RESULT]/' | tr -d '"\\')"
    printf '  "p2_acceptance": %s,\n' "$(p2_acceptance_true && echo true || echo false)"
    printf '  "grasp_model": "%s",\n' "${GRASP_MODEL}"
    printf '  "contact_hold_diagnostic_only": %s,\n' "$([[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]] && echo true || echo false)"
    printf '  "contact_hover_only": %s,\n' "$([[ "${CONTACT_HOVER_ONLY}" -eq 1 ]] && echo true || echo false)"
    printf '  "contact_probe_result_line": "%s",\n' "$(grep -m1 '\[CONTACT_PROBE_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[CONTACT_PROBE_RESULT\]/[CONTACT_PROBE_RESULT]/' | tr -d '"\\')"
    printf '  "grasp_descent_enabled": %s,\n' "$([[ "${GRASP_DESCENT}" -eq 1 ]] && echo true || echo false)"
    printf '  "grasp_contact_close_enabled": %s,\n' "$([[ "${GRASP_CONTACT_CLOSE}" -eq 1 ]] && echo true || echo false)"
printf '  "p3a2b_result_line": "%s",\n' "$(grep -m1 '\[P3A2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[P3A2B_RESULT\]/[P3A2B_RESULT]/' | tr -d '"\\')"
    printf '  "p3a2c_result_line": "%s",\n' "$(grep -m1 '\[P3A2C_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[P3A2C_RESULT\]/[P3A2C_RESULT]/' | tr -d '"\\')"
printf '  "pregrasp_execute_enabled": %s,\n' "$([[ "${PREGRASP_EXECUTE}" -eq 1 ]] && echo true || echo false)"
    printf '  "p3a_result_line": "%s",\n' "$(grep -m1 '\[P3A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null | sed 's/.*\[P3A_RESULT\]/[P3A_RESULT]/' | tr -d '"\\')"
    printf '  "pregrasp_reached": %s,\n' "$(pregrasp_reached && echo true || echo false)"
    printf '  "task_log": "task_launch.log"\n'
    printf '}\n'
  } > "${RUN_DIR}/prepare_status.json" 2>/dev/null || true
}

# True only when the task actually emitted a P2a result with all three static
# checks passed. Absence of the line is NOT treated as a pass.
# True only when the task emitted p2_acceptance=true. Missing line is NOT a pass.
# True only when the task reported PREGRASP_REACHED. Absence is NOT arrival.
pregrasp_reached() {
  local line
  line="$(grep -m1 '\[P3A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null || true)"
  [[ -n "${line}" ]] || return 1
  [[ "${line}" == *"state=PREGRASP_REACHED"* ]]
}

grasp_pose_reached() {
  local line
  line="$(grep -m1 '\[P3A2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null || true)"
  [[ "${line}" == *"state=GRASP_POSE_REACHED"* ]]
}

grasp_contact_confirmed() {
  local line
  line="$(grep -m1 '\[P3A2C_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null || true)"
  [[ "${line}" == *"state=GRASP_CONTACT_CONFIRMED"* ]]
}

p2_acceptance_true() {
  local line
  line="$(grep -m1 '\[P2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null || true)"
  [[ -n "${line}" ]] || return 1
  [[ "${line}" == *"p2_acceptance=true"* ]]
}

p2a_static_passed() {
  local line
  line="$(grep -m1 '\[P2A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null || true)"
  [[ -n "${line}" ]] || return 1
  [[ "${line}" == *"geometry=passed"* && "${line}" == *"ik=passed"* && "${line}" == *"collision=passed"* ]]
}

# Any exit through this trap reports a failure that has not already been
# reported, so an aborted script still leaves a machine-readable record.
SIM_PID=""
SIM_PGID=""
RUNNER_PID=""
RUNNER_PGID=""
TASK_PID=""
TASK_PGID=""
SELF_PGID="$(ps -o pgid= -p $$ | tr -d ' ')"
cleanup_done=0

stop_group() {
  local label="$1" pgid="$2" pid="$3"
  [[ -z "${pgid}" || -z "${pid}" ]] && return 0
  local target="$pid"
  if [[ "${pgid}" != "${SELF_PGID}" ]]; then
    target="-${pgid}"
  else
    echo "[cleanup] ${label} shares this script's process group; signalling the PID only"
  fi
  kill -0 -- "${target}" 2>/dev/null || return 0
  # The no-load tests on this workstation verified TERM-first shutdown of the
  # embedded MuJoCo/controller-manager stack; INT-first hit executor teardown.
  local first_signal=INT
  [[ "${label}" == "sim" ]] && first_signal=TERM
  echo "[cleanup] SIG${first_signal} to ${label} (${target})"
  kill -"${first_signal}" -- "${target}" 2>/dev/null
  for _ in $(seq 1 40); do kill -0 -- "${target}" 2>/dev/null || break; sleep 0.25; done
  if kill -0 -- "${target}" 2>/dev/null; then
    echo "[cleanup] SIGTERM to ${label}"
    kill -TERM -- "${target}" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 -- "${target}" 2>/dev/null || break; sleep 0.25; done
  fi
  if kill -0 -- "${target}" 2>/dev/null; then
    echo "[cleanup] SIGKILL to ${label}"
    kill -KILL -- "${target}" 2>/dev/null
  fi
  wait "${pid}" 2>/dev/null
}

cleanup() {
  local rc=$?
  if [[ "${cleanup_done}" -eq 0 ]]; then
    cleanup_done=1
    stop_group runner "${RUNNER_PGID}" "${RUNNER_PID}"
    stop_group sim "${SIM_PGID}" "${SIM_PID}"
    echo "[cleanup] simulator reaped"
    # Final audit AFTER cleanup: crashes during shutdown are a known limitation
    # and are classified separately from the run itself.
    if [[ -f "${RUN_DIR}/sim_dual_moveit.log" ]]; then
      if grep -qE "process has died|Segmentation fault" "${RUN_DIR}/sim_dual_moveit.log"; then
        grep -E "process has died|Segmentation fault" "${RUN_DIR}/sim_dual_moveit.log" \
          > "${RUN_DIR}/sim_stack_deaths_after_cleanup.txt" 2>/dev/null || true
        echo "[cleanup] note: the launch stack reported process deaths during shutdown;"
        echo "          see sim_stack_deaths_after_cleanup.txt (known limitation, not a run failure)"
      fi
    fi
  fi
  if [[ "$rc" -ne 0 && -z "${SCRIPT_STATUS}" && -n "${RUN_DIR}" && -d "${RUN_DIR}" ]]; then
    record_status "E_SCRIPT_INTERRUPTED" "script exited with code ${rc}" "cleanup"
  fi
  return "${rc}"
}
trap cleanup EXIT
trap 'echo "[run] signal received; stopping" >&2; exit 130' INT TERM

observe_task_error_if_requested() {
  [[ "${OBSERVE_ON_ERROR}" -eq 1 && "${TASK_STATE:-}" == "ERROR" && "${RC:-0}" -ne 0 ]] || return 0
  [[ "${USE_RVIZ}" == "true" && "${HEADLESS}" != "true" ]] || return 0
  if ! grep -qE '\[CONTACT_PROBE_CONFIG\]|\[CONTACT_HOLD_MECHANISM_RESULT\]|\[P3A2C_RESULT\]' \
      "${RUN_DIR}/task_launch.log" 2>/dev/null; then
    echo "[observe] contact execution did not reach its task-stage marker; skip observation wait"
    return 0
  fi
  if [[ ! -t 0 || ! -t 1 ]]; then
    echo "[observe] non-interactive terminal; skip observation wait"
    return 0
  fi
  [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null || {
    echo "[observe] simulator is no longer running; skip observation wait"
    return 0
  }
  local services
  services="$(timeout "${HELPER_TIMEOUT}" ros2 service list --no-daemon 2>/dev/null || true)"
  if ! grep -qx "/get_sim_info" <<<"${services}"; then
    echo "[observe] live MuJoCo truth service unavailable; skip observation wait"
    return 0
  fi

  printf '%s\n' '任务已失败并停止运动，保留仿真供观察；按回车结束，Ctrl+C也可退出。'
  local observe_interrupted=0
  trap 'observe_interrupted=1' INT
  IFS= read -r _observe_input || true
  trap 'echo "[run] signal received; stopping" >&2; exit 130' INT TERM
}

echo "=============================================================="
echo " A1 P1a scene runner"
echo "   run_id            : ${RUN_ID}"
echo "   candidate index   : ${CANDIDATE_INDEX}"
echo "   seed override     : ${SEED:-<from config>}"
if [[ "${EXPLICIT_FIELDS}" -eq 3 ]]; then
  echo "   explicit pose     : x=${EX_X} y=${EX_Y} yaw_deg=${EX_YAW} (override)"
else
  echo "   explicit pose     : none (sampled)"
fi
echo "   dry run           : ${DRY_RUN}"
echo "   ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID}"
echo "   ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
echo "   params            : ${PARAMS_FILE}"
echo "   output dir        : ${RUN_DIR}"
if [[ "${TASK_TARGET_FIELDS}" -eq 3 ]]; then
  echo "   task target       : x=${TASK_TARGET_X} y=${TASK_TARGET_Y} world_yaw_deg=${TASK_TARGET_YAW_DEG}"
else
  echo "   task target       : launch defaults"
fi
echo "=============================================================="

NODE_ARGS=(
  --candidate-index "${CANDIDATE_INDEX}"
  --run-id "${RUN_ID}"
  --manifest-dir "${MANIFEST_DIR}"
  --domain-id "${ROS_DOMAIN_ID}"
  --localhost-only "1"
)
[[ -n "${SEED}" ]] && NODE_ARGS+=(--seed "${SEED}")
if [[ "${EXPLICIT_FIELDS}" -eq 3 ]]; then
  NODE_ARGS+=(--x "${EX_X}" --y "${EX_Y}" --yaw-deg "${EX_YAW}")
fi
[[ -n "${MODELS_DIR}" ]] && NODE_ARGS+=(--models-dir "${MODELS_DIR}")
[[ -n "${SRDF_PATH}" ]] && NODE_ARGS+=(--srdf-path "${SRDF_PATH}")
[[ -n "${GIT_COMMIT}" ]] && NODE_ARGS+=(--git-commit "${GIT_COMMIT}")
NODE_ARGS+=(--git-dirty "${GIT_DIRTY}")

# --- Dry run: no simulator, no service call ---------------------------------
if [[ "${DRY_RUN}" -eq 1 ]]; then
  NODE_ARGS+=(--dry-run)
  timeout "${NODE_TIMEOUT}" ros2 run dual_arm_carry_task a1_scene_runner_node \
    "${NODE_ARGS[@]}" \
    --ros-args --params-file "${PARAMS_FILE}" \
    2>&1 | tee "${RUN_DIR}/runner.log"
  RC=${PIPESTATUS[0]}
  if [[ "${RC}" -ne 0 ]]; then
    record_status "E_DRY_RUN_FAILED" "runner exited with code ${RC}" "dry_run"
  fi
  echo "runner exit code: ${RC}"
  exit "${RC}"
fi

# --- Ownership: refuse to run alongside an instance we do not own ------------
# Discovery calls are bounded: an unreachable or wedged graph must not hang.
# --no-daemon: the CLI discovery daemon can retain entries for an instance that
# already died (e.g. after a crash), which would make this guard refuse a run
# when nothing is actually listening. Live discovery is what this check needs.
EXISTING="$(timeout "${HELPER_TIMEOUT}" ros2 service list --no-daemon 2>/dev/null || true)"
if grep -qE "^/(set_body_state|get_body_state|get_sim_info)$" <<<"${EXISTING}"; then
  echo "error: a simulation instance already answers on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}:" >&2
  grep -E "^/(set_body_state|get_body_state|get_sim_info)$" <<<"${EXISTING}" >&2
  echo "       this script will not reuse or clean up an instance it did not start." >&2
  echo "       Stop that instance (or use another --domain-id) and retry." >&2
  record_status "E_INSTANCE_ALREADY_PRESENT" \
    "services already advertised on this ROS domain" "ownership_check"
  exit 4
fi

# --- Start exactly one simulation instance ----------------------------------
SIM_LOG="${RUN_DIR}/sim_dual_moveit.log"
# Contact-probe policy: real-contact load validation must not simultaneously
# soften the arm position loop with the internal adaptive-compliance baseline.
# Keep ideal_weld and all other modes unchanged; allow an explicit opt-out for
# controlled A/B experiments with CONTACT_PROBE_RIGID=false.
CONTACT_PROBE_RIGID="${CONTACT_PROBE_RIGID:-true}"
CONTACT_PROBE_ADAPTIVE_FORCE_HOLD="${CONTACT_PROBE_ADAPTIVE_FORCE_HOLD:-false}"
TASK_CONTROL_MODE="compliant"
if [[ "${GRASP_MODEL}" == "contact_probe" && "${CONTACT_PROBE_RIGID}" == "true" ]]; then
  export MJ_INTERNAL_COMPLIANCE_ENABLED=false
  TASK_CONTROL_MODE="rigid"
  echo "[contact_probe] rigid validation profile: internal MuJoCo compliance disabled"
fi
if [[ "${GRASP_MODEL}" == "contact_probe" ]]; then
  export CONTACT_PROBE_SINGLE_PAD=true
  export CONTACT_PROBE_CONTACT_TRACE_CSV="${RUN_DIR}/contact_probe_contacts.csv"
  echo "[contact_probe] single-primary-pad contact profile enabled"
  echo "[contact_probe] per-contact geom/force/actuator trace: ${CONTACT_PROBE_CONTACT_TRACE_CSV}"
  export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD
  echo "[contact_probe] adaptive normal-force hold: ${CONTACT_PROBE_ADAPTIVE_FORCE_HOLD}"
else
  # These are process-wide opt-in hooks used by the hardware plugin. Do not
  # let values inherited from the caller's shell affect ideal_weld/other runs.
  unset CONTACT_PROBE_SINGLE_PAD CONTACT_PROBE_CONTACT_TRACE_CSV
fi

# A1 P3a-2a only: have each simulated gripper node append a plain CSV of its own
# real finger feedback (q1, q2, total gap, sequence number, command, target).
# Off for every other mode, and read by the gripper nodes at construction, so it
# must be exported before the simulation launch below.
if [[ "${GRIPPER_SELFTEST}" -eq 1 || "${GRASP_CONTACT_CLOSE}" -eq 1 || "${GRASP_MODEL}" == "contact_probe" ]]; then
  export GRIPPER_TRACE_CSV="${RUN_DIR}/gripper_fingers"
  echo "[gripper] raw finger command/feedback trace enabled: ${GRIPPER_TRACE_CSV}_<robot>.csv"
fi
echo "[sim] starting one simulation instance (MoveIt launch includes MuJoCo)"
setsid ros2 launch franka_moveit_config sim_dual_moveit.launch.py \
  "use_rviz:=${USE_RVIZ}" "headless:=${HEADLESS}" \
  > "${SIM_LOG}" 2>&1 &
SIM_PID=$!
sleep 1
SIM_PGID="$(ps -o pgid= -p "${SIM_PID}" 2>/dev/null | tr -d ' ' || true)"
echo "[sim] pid=${SIM_PID} pgid=${SIM_PGID:-unknown} log=${SIM_LOG}"

# --- Wait for the JSON services ---------------------------------------------
echo "[sim] waiting up to ${READY_TIMEOUT}s for /set_body_state, /get_body_state, /get_sim_info"
DEADLINE=$((SECONDS + READY_TIMEOUT))
READY=0
while (( SECONDS < DEADLINE )); do
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "[sim] simulator exited early; see ${SIM_LOG}" >&2
    tail -n 40 "${SIM_LOG}" >&2 || true
    record_status "E_SIM_EXITED_DURING_STARTUP" "launch process died before services appeared" "startup"
    exit 2
  fi
  # A crash reported by the launch machinery during startup stops the run before
  # anything is written to the simulation.
  if grep -qE "process has died|Segmentation fault" "${SIM_LOG}" 2>/dev/null; then
    grep -E "process has died|Segmentation fault" "${SIM_LOG}" \
      > "${RUN_DIR}/sim_stack_deaths.txt" 2>/dev/null || true
    echo "[sim] a component died during startup; see sim_stack_deaths.txt" >&2
    record_status "E_SIM_COMPONENT_DIED_AT_STARTUP" "launch reported a dead component" "startup"
    exit 2
  fi
  SERVICES="$(timeout "${HELPER_TIMEOUT}" ros2 service list 2>/dev/null || true)"
  if grep -qx "/set_body_state" <<<"${SERVICES}" \
    && grep -qx "/get_body_state" <<<"${SERVICES}" \
    && grep -qx "/get_sim_info" <<<"${SERVICES}"; then
    READY=1
    break
  fi
  sleep 1
done
if [[ "${READY}" -ne 1 ]]; then
  echo "error: services were not available within ${READY_TIMEOUT}s" >&2
  timeout "${HELPER_TIMEOUT}" ros2 service list 2>/dev/null | sort > "${RUN_DIR}/services_seen.txt" || true
  tail -n 40 "${SIM_LOG}" >&2 || true
  record_status "E_SERVICES_UNAVAILABLE" "services did not appear within ${READY_TIMEOUT}s" "startup"
  exit 2
fi
echo "[sim] services available"
timeout "${HELPER_TIMEOUT}" ros2 service list 2>/dev/null | sort > "${RUN_DIR}/services_seen.txt" || true

# Controller inventory is captured by the script (the node does not query it).
if timeout "${HELPER_TIMEOUT}" ros2 control list_controllers \
    > "${RUN_DIR}/controllers.txt" 2>&1; then
  echo "[sim] controller inventory captured"
else
  echo "NOT_RUN: ros2 control list_controllers did not complete within ${HELPER_TIMEOUT}s" \
    >> "${RUN_DIR}/controllers.txt"
  echo "[sim] warning: controller inventory not captured (see controllers.txt)"
fi

# --- A1 P1b prepare mode ----------------------------------------------------
# Everything happens inside the single simulation instance started above: the
# runner applies the pose and publishes the settled truth, the task receives and
# freezes it and confirms the planning scene. No second simulation is started and
# the rod is never written twice.
# --- A1 P3a-2a: minimal explicit gripper empty-load cycle -------------------
if [[ "${GRIPPER_SELFTEST}" -eq 1 ]]; then
  echo "[gripper] empty-load cycle: open -> close -> open (arms idle)"
  # The node itself waits (bounded) for the action servers, so no CLI discovery
  # race is introduced here.
  setsid ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
    gripper_selftest:=true \
    control_mode:=compliant \
    enable_latency_monitor:=false enable_task_metrics_monitor:=false \
    > "${RUN_DIR}/gripper_selftest.log" 2>&1 &
  TASK_PID=$!
  sleep 1
  TASK_PGID="$(ps -o pgid= -p "${TASK_PID}" 2>/dev/null | tr -d ' ' || true)"

  DEADLINE=$((SECONDS + TASK_TIMEOUT))
  GRIPPER_STATE=""
  while (( SECONDS < DEADLINE )); do
    if grep -q '\[GRIPPER_RESULT\]' "${RUN_DIR}/gripper_selftest.log" 2>/dev/null; then
      GRIPPER_STATE="$(grep -m1 -o 'state=[A-Z_]*' "${RUN_DIR}/gripper_selftest.log" | head -1)"
      break
    fi
    if ! kill -0 "${TASK_PID}" 2>/dev/null; then GRIPPER_STATE="state=PROCESS_EXITED"; break; fi
    sleep 1
  done
  [[ -z "${GRIPPER_STATE}" ]] && GRIPPER_STATE="state=TIMEOUT"
  # Raw evidence: what the simulator actually publishes for the finger joints.
  timeout "${HELPER_TIMEOUT}" ros2 topic echo --once /joint_states \
    > "${RUN_DIR}/joint_states_after_cycle.txt" 2>/dev/null || true
  echo "[gripper] ${GRIPPER_STATE}"
  grep -m1 '\[GRIPPER_RESULT\]' "${RUN_DIR}/gripper_selftest.log" 2>/dev/null || true
  {
    printf '{\n  "written_by": "scripts/a1_run_scene.sh",\n'
    printf '  "run_id": "%s",\n  "mode": "gripper_selftest",\n' "${RUN_ID}"
    printf '  "result_line": "%s",\n' "$(grep -m1 '\[GRIPPER_RESULT\]' "${RUN_DIR}/gripper_selftest.log" 2>/dev/null | sed 's/.*\[GRIPPER_RESULT\]/[GRIPPER_RESULT]/' | tr -d '"\\')"
    printf '  "reached": %s\n' "$([[ "${GRIPPER_STATE}" == "state=GRIPPER_TEST_DONE" ]] && echo true || echo false)"
    printf '}\n'
  } > "${RUN_DIR}/gripper_selftest_status.json" 2>/dev/null || true

  if [[ "${GRIPPER_STATE}" == "state=GRIPPER_TEST_DONE" ]]; then RC=0; else RC=7; fi
  stop_group task "${TASK_PGID}" "${TASK_PID}"
  stop_group sim "${SIM_PGID}" "${SIM_PID}"
  SIM_PID=""; SIM_PGID=""
  echo "gripper selftest exit code: ${RC}"
  exit "${RC}"
fi

if [[ "${PREPARE_MODE}" -eq 1 ]]; then
  echo "[prepare] A1 P1b prepare mode: apply -> publish truth -> task freeze -> scene sync"
  RUNNER_ARGS=("${NODE_ARGS[@]}" --publish-truth --truth-topic "${TRUTH_TOPIC}" --hold-sec "${HOLD_SEC}")
  setsid ros2 run dual_arm_carry_task a1_scene_runner_node \
    "${RUNNER_ARGS[@]}" \
    --ros-args --params-file "${PARAMS_FILE}" -p service_call_timeout_sec:=5.0 \
    > "${RUN_DIR}/runner.log" 2>&1 &
  RUNNER_PID=$!
  sleep 1
  RUNNER_PGID="$(ps -o pgid= -p "${RUNNER_PID}" 2>/dev/null | tr -d ' ' || true)"
  echo "[prepare] runner pid=${RUNNER_PID} pgid=${RUNNER_PGID:-unknown}"

  # Wait for the settled truth to have been published. The runner publishes
  # before it writes the manifest, so the manifest is the readiness signal.
  PREP_DEADLINE=$((SECONDS + NODE_TIMEOUT))
  TRUTH_READY=0
  while (( SECONDS < PREP_DEADLINE )); do
    if ! kill -0 "${RUNNER_PID}" 2>/dev/null && [[ ! -f "${RUN_DIR}/manifest.json" ]]; then
      echo "[prepare] runner exited before publishing truth; see runner.log" >&2
      break
    fi
    if [[ -f "${RUN_DIR}/manifest.json" ]] && grep -q "truth published" "${RUN_DIR}/runner.log"; then
      TRUTH_READY=1
      break
    fi
    sleep 1
  done
  if [[ "${TRUTH_READY}" -ne 1 ]]; then
    echo "error: the settled truth was not published in time" >&2
    tail -n 20 "${RUN_DIR}/runner.log" >&2 || true
    record_status "E_TRUTH_NOT_PUBLISHED" "runner did not publish truth in time" "prepare"
    RC=2
  else
    echo "[prepare] truth published; starting the task in prepare-only mode"
    # The task must not move: rod_pose_source=a1_truth with prepare_only=true.
    P2A_ARGS=(p2a_check:="$([[ "${P2A_CHECK}" -eq 1 ]] && echo true || echo false)"
              p2b_check:="$([[ "${P2B_CHECK}" -eq 1 ]] && echo true || echo false)"
              pregrasp_execute:="$([[ "${PREGRASP_EXECUTE}" -eq 1 ]] && echo true || echo false)"
              grasp_descent:="$([[ "${GRASP_DESCENT}" -eq 1 ]] && echo true || echo false)")
    P2A_ARGS+=(grasp_contact_close:="$([[ "${GRASP_CONTACT_CLOSE}" -eq 1 ]] && echo true || echo false)")
    P2A_ARGS+=(grasp_model:="${GRASP_MODEL}")
    P2A_ARGS+=(contact_hold_diagnostic_only:="$([[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]] && echo true || echo false)")
    P2A_ARGS+=(contact_hover_only:="$([[ "${CONTACT_HOVER_ONLY}" -eq 1 ]] && echo true || echo false)" \
               contact_hover_height_m:="${CONTACT_HOVER_HEIGHT_M}" \
               contact_hover_hold_seconds:="${CONTACT_HOVER_HOLD_SECONDS}")
    # P4/S2 phase 1: planner comparison. Gated by an explicit environment
    # variable so the ordinary contact-probe path never changes.
    if [[ "${CONTACT_PLAN_COMPARE:-0}" -eq 1 ]]; then
      P2A_ARGS+=(contact_plan_compare:=true)
    fi
    # S2 transport projection comparison: planning only, never any motion.
    if [[ "${CONTACT_TRANSPORT_PLAN_ONLY:-0}" -eq 1 ]]; then
      P2A_ARGS+=(contact_transport_plan_only:=true)
    fi
    if [[ "${TASK_TARGET_FIELDS}" -eq 3 ]]; then
      P2A_ARGS+=(rod_target_x:="${TASK_TARGET_X}" rod_target_y:="${TASK_TARGET_Y}"
                 rod_target_yaw_deg:="${TASK_TARGET_YAW_DEG}")
    fi
    # Verification-only injections. ros2 launch rejects an empty value
    # ("name:="), so these are only passed when actually set.
    if [[ -n "${DESCENT_NEGATIVE_FINAL}" ]]; then
      P2A_ARGS+=(descent_negative_final:="${DESCENT_NEGATIVE_FINAL}")
    fi
    if [[ -n "${DESCENT_NEGATIVE_TRUTH}" ]]; then
      P2A_ARGS+=(descent_negative_truth:="${DESCENT_NEGATIVE_TRUTH}")
    fi
    if [[ -n "${P2A_WORKSPACE}" ]]; then
      read -r P2A_XMIN P2A_XMAX P2A_YMIN P2A_YMAX <<<"${P2A_WORKSPACE}"
      P2A_ARGS+=(p2a_workspace_x_min:="${P2A_XMIN}" p2a_workspace_x_max:="${P2A_XMAX}"
                 p2a_workspace_y_min:="${P2A_YMIN}" p2a_workspace_y_max:="${P2A_YMAX}")
    fi
    if [[ -n "${P2A_WORKSPACE_SOURCE}" ]]; then
      P2A_ARGS+=(p2a_workspace_source:="${P2A_WORKSPACE_SOURCE}")
    fi

    setsid ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
      rod_pose_source:=a1_truth prepare_only:=true expected_run_id:="${RUN_ID}" \
      a1_truth_topic:="${TRUTH_TOPIC}" use_vision:=false \
      "${P2A_ARGS[@]}" \
      control_mode:="${TASK_CONTROL_MODE}" \
      enable_latency_monitor:=false enable_task_metrics_monitor:=false \
      > "${RUN_DIR}/task_launch.log" 2>&1 &
    TASK_PID=$!
    sleep 1
    TASK_PGID="$(ps -o pgid= -p "${TASK_PID}" 2>/dev/null | tr -d ' ' || true)"
    echo "[prepare] task pid=${TASK_PID} pgid=${TASK_PGID:-unknown}"

    TASK_DEADLINE=$((SECONDS + TASK_TIMEOUT))
    TASK_STATE=""
    while (( SECONDS < TASK_DEADLINE )); do
      if [[ "${GRASP_MODEL}" == "contact_probe" ]] && grep -q '\[CONTACT_PROBE_RESULT\] state=PLAN_COMPARE_DONE' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="PLAN_COMPARE_DONE"; break
      fi
      if [[ "${GRASP_MODEL}" == "contact_probe" ]] && grep -q '\[CONTACT_PROBE_RESULT\] state=DONE' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="DONE"; break
      fi
      if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]] && grep -q '\[CONTACT_HOLD_MECHANISM_RESULT\] state=PASS' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="CONTACT_HOLD_MECHANISM_PASS"; break
      fi
      if grep -q '\[S1_IDEAL_WELD_RESULT\] state=DONE' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="DONE"; break
      fi
      # S2 transport plan-only: checked before the PREPARED marker, because the
      # comparison runs inside the prepare step and never publishes PREPARED.
      if [[ "${CONTACT_TRANSPORT_PLAN_ONLY:-0}" -eq 1 ]] && \
         grep -q '\[TRANSPORT_OPT_RESULT\] state=PLAN_ONLY_DONE' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="PLAN_ONLY_DONE"; break
      fi
      if grep -q "已发布准备完成状态(PREPARED)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="PREPARED"; break
      fi
      if grep -q "\[P3A2C_RESULT\] state=GRASP_CONTACT_CONFIRMED" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="GRASP_CONTACT_CONFIRMED"; break
      fi
      if grep -q "已发布任务失败状态(ERROR)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="ERROR"; break
      fi
      # A1 P3a-2b terminal state: the descent reached the grasp pose. Checked
      # BEFORE the pre-grasp marker, because with grasp_descent enabled the
      # pre-grasp is an intermediate step on the way here.
      if grep -q "已发布抓取位姿到位状态(GRASP_POSE_REACHED)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="GRASP_POSE_REACHED"; break
      fi
      # A1 P3a-1 terminal state: pre-grasp reached (its own stage result).
      if grep -q "已发布预抓取到位状态(PREGRASP_REACHED)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="PREGRASP_REACHED"; break
      fi
      if ! kill -0 "${TASK_PID}" 2>/dev/null; then
        TASK_STATE="TASK_PROCESS_EXITED"; break
      fi
      sleep 1
    done
    [[ -z "${TASK_STATE}" ]] && TASK_STATE="TIMEOUT"
    echo "[prepare] task state: ${TASK_STATE}"
    record_prepare_status "${TASK_STATE}"

    if [[ "${TASK_STATE}" == "DONE" && "${GRASP_MODEL}" == "ideal_weld" ]]; then
      RC=0
      echo "[prepare] S1 ideal_weld complete; rigid ideal grasp only (friction/slip unverified)"
    elif [[ "${TASK_STATE}" == "DONE" && "${GRASP_MODEL}" == "contact_probe" ]]; then
      RC=0
      echo "[prepare] contact_probe complete: no-weld lift/hold/lower/release only; full transport unverified"
    elif [[ "${TASK_STATE}" == "CONTACT_HOLD_MECHANISM_PASS" ]]; then
      RC=0
      echo "[prepare] contact hold mechanism only: fixed-arm hold and release passed; no lift/load claim"
    elif [[ "${TASK_STATE}" == "PLAN_COMPARE_DONE" ]]; then
      RC=0
      echo "[prepare] P4/S2 phase 1: planner comparison complete; no carry claim"
    elif [[ "${TASK_STATE}" == "PLAN_ONLY_DONE" ]]; then
      RC=0
      echo "[prepare] S2 transport projection comparison complete: planning only; no motion was sent"
    elif [[ "${TASK_STATE}" == "GRASP_CONTACT_CONFIRMED" ]]; then
      if grasp_contact_confirmed; then
        RC=0
        echo "[prepare] P3a-2c: GRASP_CONTACT_CONFIRMED (bilateral finger-rod contact; no weld/attach/lift/carry)"
      else
        record_status "E_P3A2C_NOT_CONFIRMED" "contact result missing or failed" "prepare"
        RC=9
      fi
    elif [[ "${TASK_STATE}" == "PREGRASP_REACHED" ]]; then
      RC=0
      if pregrasp_reached; then
        echo "[prepare] P3a-1: PREGRASP_REACHED (pre-grasp only; no grasp, no carry)"
      else
        echo "[prepare] P3a-1 reported a terminal state but no PREGRASP_REACHED result" >&2
        record_status "E_P3A_NOT_REACHED" \
          "pre-grasp execution ended without reporting arrival" "prepare"
        RC=6
      fi
    elif [[ "${TASK_STATE}" == "GRASP_POSE_REACHED" ]]; then
      if [[ "${GRASP_CONTACT_CLOSE}" -eq 1 ]]; then
        echo "[prepare] P3a-2c NOT CONFIRMED: contact close was requested, but task stopped at GRASP_POSE_REACHED" >&2
        record_status "E_P3A2C_NOT_CONFIRMED" \
          "contact close was requested but the task only reached GRASP_POSE_REACHED; no contact result was reported" \
          "prepare"
        RC=9
      elif grasp_pose_reached; then
        RC=0
        echo "[prepare] P3a-2b: GRASP_POSE_REACHED (arms only; gripper still OPEN; no grasp, no carry)"
      else
        echo "[prepare] P3a-2b reported the terminal state but no GRASP_POSE_REACHED result" >&2
        record_status "E_P3A2B_NOT_REACHED" \
          "the descent ended without reporting arrival" "prepare"
        RC=8
      fi
    elif [[ "${TASK_STATE}" == "PREPARED" ]]; then
      RC=0
      # PREPARED only means the truth was frozen and the scene synced. When the
      # P2a check is enabled, the exit code reflects the static validity result
      # so a rejected sample can never look like a passing run.
      if [[ "${P2A_CHECK}" -eq 1 ]]; then
        if [[ "${P2B_CHECK}" -eq 1 ]]; then
          if p2_acceptance_true; then
            echo "[prepare] P2b: static + joint pre-grasp path checks passed (planning only, not DONE)"
          else
            echo "[prepare] P2b path check did NOT pass; see task_launch.log" >&2
            grep -m1 '\[P2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null >&2 || true
            record_status "E_P2B_PATH_FAILED" \
              "the joint pre-grasp path did not pass; sample rejected, no rule or start was changed" \
              "prepare"
            RC=5
          fi
        elif [[ "${GRASP_DESCENT}" -eq 1 ]]; then
          if grasp_pose_reached; then
            echo "[prepare] P3a-2b: GRASP_POSE_REACHED (arms only, gripper still OPEN; no grasp, no carry)"
          else
            echo "[prepare] P3a-2b did NOT reach the grasp pose; see task_launch.log" >&2
            grep -m1 '\[P3A2B_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null >&2 || true
            record_status "E_P3A2B_NOT_REACHED" \
              "the descent did not reach the grasp pose; reported as failure, nothing retried or re-sampled" \
              "prepare"
            RC=8
          fi
        elif [[ "${PREGRASP_EXECUTE}" -eq 1 ]]; then
          if pregrasp_reached; then
            echo "[prepare] P3a-1: PREGRASP_REACHED (pre-grasp only, not a grasp or carry)"
          else
            echo "[prepare] P3a-1 did NOT reach the pre-grasp target; see task_launch.log" >&2
            grep -m1 '\[P3A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null >&2 || true
            record_status "E_P3A_NOT_REACHED" \
              "pre-grasp execution did not reach the target; reported as failure, nothing retried" \
              "prepare"
            RC=6
          fi
        elif p2a_static_passed; then
          echo "[prepare] P2a static checks: geometry/ik/collision all passed"
        else
          echo "[prepare] P2a static checks did NOT all pass; see task_launch.log" >&2
          grep -m1 '\[P2A_RESULT\]' "${RUN_DIR}/task_launch.log" 2>/dev/null >&2 || true
          record_status "E_P2A_STATIC_FAILED" \
            "P2a geometry/IK/collision did not all pass; sample rejected, no rule or tolerance was relaxed" \
            "prepare"
          RC=4
        fi
      fi
    else
      RC=2
    fi
    # P2b: extract the recorded joint path into a CSV next to the log.
    if [[ -f "${RUN_DIR}/task_launch.log" ]] && grep -q '\[P2B_PATH\]' "${RUN_DIR}/task_launch.log" 2>/dev/null; then
      {
        grep -m1 '\[P2B_PATH_NAMES\]' "${RUN_DIR}/task_launch.log" \
          | sed 's/.*\[P2B_PATH_NAMES\] //' | sed 's/^/waypoint,/'
        grep '\[P2B_PATH\]' "${RUN_DIR}/task_launch.log" | sed 's/.*\[P2B_PATH\] //'
      } > "${RUN_DIR}/p2b_path.csv" 2>/dev/null || true
      grep -m1 '\[P2B_START_END\]' "${RUN_DIR}/task_launch.log" 2>/dev/null \
        | sed 's/.*\[P2B_START_END\] //' > "${RUN_DIR}/p2b_path_summary.txt" 2>/dev/null || true
      echo "[prepare] wrote p2b_path.csv ($(grep -c '\[P2B_PATH\]' "${RUN_DIR}/task_launch.log") waypoints)"
    fi

    stop_group task "${TASK_PGID}" "${TASK_PID}"
  fi

  stop_group runner "${RUNNER_PGID}" "${RUNNER_PID}"
  RUNNER_PID=""; RUNNER_PGID=""
  observe_task_error_if_requested
  echo "prepare exit code: ${RC}"
  exit "${RC}"
fi

# --- Run the single-candidate runner (foreground, bounded) ------------------
echo "[run] applying candidate ${CANDIDATE_INDEX} (node timeout ${NODE_TIMEOUT}s)"
timeout --signal=TERM --kill-after=20 "${NODE_TIMEOUT}" \
  ros2 run dual_arm_carry_task a1_scene_runner_node \
  "${NODE_ARGS[@]}" \
  --ros-args --params-file "${PARAMS_FILE}" \
  2>&1 | tee "${RUN_DIR}/runner.log"
RC=${PIPESTATUS[0]}
if [[ "${RC}" -eq 124 ]]; then
  echo "runner exceeded ${NODE_TIMEOUT}s and was terminated" | tee -a "${RUN_DIR}/runner.log"
  record_status "E_NODE_TIMEOUT" "runner exceeded ${NODE_TIMEOUT}s" "run"
fi

# Component deaths seen during the run are part of the run verdict.
if grep -qE "process has died|Segmentation fault" "${SIM_LOG}" 2>/dev/null; then
  grep -E "process has died|Segmentation fault" "${SIM_LOG}" \
    > "${RUN_DIR}/sim_stack_deaths.txt" 2>&1 || true
  echo "[sim] warning: the launch stack reported process deaths during the run;"
  echo "      see sim_stack_deaths.txt"
  if [[ "${RC}" -eq 0 ]]; then
    RC=4
    record_status "E_SIM_COMPONENT_DIED_DURING_RUN" "launch reported a dead component" "run"
  fi
fi

echo "runner exit code: ${RC}"
if [[ -f "${RUN_DIR}/manifest.json" ]]; then
  echo "manifest: ${RUN_DIR}/manifest.json"
else
  echo "no manifest was written for this run"
  if [[ -z "${SCRIPT_STATUS}" ]]; then
    record_status "E_NO_MANIFEST" "the runner produced no manifest" "run"
  fi
fi
exit "${RC}"
