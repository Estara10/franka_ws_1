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
  --domain-id N         ROS_DOMAIN_ID for this run (default 77)
  --use-rviz            start RViz with the simulator (default: no)
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
    --domain-id)       DOMAIN_ID="$2"; shift 2 ;;
    --use-rviz)        USE_RVIZ="true"; shift ;;
    --headless)        HEADLESS="true"; shift ;;
    --ready-timeout)   READY_TIMEOUT="$2"; shift 2 ;;
    --node-timeout)    NODE_TIMEOUT="$2"; shift 2 ;;
    --manifest-dir)    MANIFEST_DIR="$2"; shift 2 ;;
    --params-file)     PARAMS_FILE="$2"; shift 2 ;;
    -h|--help)         usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 3 ;;
  esac
done

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
    printf '  "task_log": "task_launch.log"\n'
    printf '}\n'
  } > "${RUN_DIR}/prepare_status.json" 2>/dev/null || true
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
  kill -0 "${target}" 2>/dev/null || return 0
  echo "[cleanup] SIGINT to ${label} (${target})"
  kill -INT "${target}" 2>/dev/null
  for _ in $(seq 1 40); do kill -0 "${target}" 2>/dev/null || break; sleep 0.25; done
  if kill -0 "${target}" 2>/dev/null; then
    echo "[cleanup] SIGTERM to ${label}"
    kill -TERM "${target}" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "${target}" 2>/dev/null || break; sleep 0.25; done
  fi
  if kill -0 "${target}" 2>/dev/null; then
    echo "[cleanup] SIGKILL to ${label}"
    kill -KILL "${target}" 2>/dev/null
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
EXISTING="$(timeout "${HELPER_TIMEOUT}" ros2 service list 2>/dev/null || true)"
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
if [[ "${PREPARE_MODE}" -eq 1 ]]; then
  echo "[prepare] A1 P1b prepare mode: apply -> publish truth -> task freeze -> scene sync"
  RUNNER_ARGS=("${NODE_ARGS[@]}" --publish-truth --truth-topic "${TRUTH_TOPIC}" --hold-sec "${HOLD_SEC}")
  setsid ros2 run dual_arm_carry_task a1_scene_runner_node \
    "${RUNNER_ARGS[@]}" \
    --ros-args --params-file "${PARAMS_FILE}" \
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
    setsid ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
      rod_pose_source:=a1_truth prepare_only:=true expected_run_id:="${RUN_ID}" \
      a1_truth_topic:="${TRUTH_TOPIC}" use_vision:=false \
      control_mode:=compliant \
      enable_latency_monitor:=false enable_task_metrics_monitor:=false \
      > "${RUN_DIR}/task_launch.log" 2>&1 &
    TASK_PID=$!
    sleep 1
    TASK_PGID="$(ps -o pgid= -p "${TASK_PID}" 2>/dev/null | tr -d ' ' || true)"
    echo "[prepare] task pid=${TASK_PID} pgid=${TASK_PGID:-unknown}"

    TASK_DEADLINE=$((SECONDS + TASK_TIMEOUT))
    TASK_STATE=""
    while (( SECONDS < TASK_DEADLINE )); do
      if grep -q "已发布准备完成状态(PREPARED)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="PREPARED"; break
      fi
      if grep -q "已发布任务失败状态(ERROR)" "${RUN_DIR}/task_launch.log" 2>/dev/null; then
        TASK_STATE="ERROR"; break
      fi
      if ! kill -0 "${TASK_PID}" 2>/dev/null; then
        TASK_STATE="TASK_PROCESS_EXITED"; break
      fi
      sleep 1
    done
    [[ -z "${TASK_STATE}" ]] && TASK_STATE="TIMEOUT"
    echo "[prepare] task state: ${TASK_STATE}"
    record_prepare_status "${TASK_STATE}"

    if [[ "${TASK_STATE}" == "PREPARED" ]]; then RC=0; else RC=2; fi
    stop_group task "${TASK_PGID}" "${TASK_PID}"
  fi

  stop_group runner "${RUNNER_PGID}" "${RUNNER_PID}"
  RUNNER_PID=""; RUNNER_PGID=""
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
