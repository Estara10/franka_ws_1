#!/usr/bin/env bash
# One user-supervised, visible +10 degree contact-load probe on this workstation.
# This wrapper never builds, retries, or cleans up unrelated processes.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

OBSERVE_ON_ERROR=0
INTERNAL_COMPLIANCE=0
# Display mode: headless for experiments (default), --use-rviz to watch the
# simulation. Pass exactly one; --headless is the safe default.
DISPLAY_FLAG="--headless"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) DISPLAY_FLAG="--headless"; shift ;;
    --use-rviz) DISPLAY_FLAG="--use-rviz"; shift ;;
    --internal-compliance) INTERNAL_COMPLIANCE=1; shift ;;
    --observe-on-error) OBSERVE_ON_ERROR=1; shift ;;
    -h|--help)
      echo "usage: run_contact_probe_local.sh [--headless|--use-rviz] [--internal-compliance] [--observe-on-error]"
      exit 0
      ;;
    *) echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: ROS 2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "error: workspace install/setup.bash is missing; build the required packages first" >&2
  exit 2
fi

# Keep an explicitly selected local MuJoCo. Use the version already used by
# this workspace's CMake cache only when the caller has not selected one.
MUJOCO_DIR="${MUJOCO_DIR:-/home/lidio/Libraries/mujoco}"
if [[ ! -d "${MUJOCO_DIR}/lib" ]]; then
  echo "error: MUJOCO_DIR has no lib directory: ${MUJOCO_DIR}" >&2
  echo "       set MUJOCO_DIR to this workstation's existing MuJoCo installation" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export MUJOCO_DIR
case ":${LD_LIBRARY_PATH:-}:" in
  *":${MUJOCO_DIR}/lib:"*) ;;
  *) export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac

export ROS_DOMAIN_ID="${CONTACT_PROBE_DOMAIN_ID:-${ROS_DOMAIN_ID:-113}}"
export ROS_LOCALHOST_ONLY=1

# contact_probe's checked-in default is the rigid validation profile. Refuse
# an inherited A/B override so this one run does not silently change factors.
# The compliant A/B round must request the profile explicitly.
if [[ "${INTERNAL_COMPLIANCE}" -eq 1 ]]; then
  export CONTACT_PROBE_RIGID=false
  export MJ_INTERNAL_COMPLIANCE_ENABLED=true
else
  if [[ "${CONTACT_PROBE_RIGID:-true}" != "true" ]]; then
    echo "error: CONTACT_PROBE_RIGID is not true; unset the override for the baseline probe" >&2
    echo "       (use --internal-compliance for the explicit compliant A/B round)" >&2
    exit 2
  fi
  export CONTACT_PROBE_RIGID=true
fi
CONTACT_HOLD_DIAGNOSTIC_ONLY="${CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY:-0}"
CONTACT_HOVER_ONLY="${CONTACT_PROBE_HOVER_ONLY:-0}"
CONTACT_HOVER_HEIGHT_M="${CONTACT_PROBE_HOVER_HEIGHT_M:-0.05}"
CONTACT_HOVER_HOLD_SECONDS="${CONTACT_PROBE_HOVER_HOLD_SECONDS:-3.0}"
CONTACT_HOVER_CONTROL_MODE="${CONTACT_PROBE_HOVER_CONTROL_MODE:-jtc}"
CONTACT_PROBE_ABCD_GROUP="${CONTACT_PROBE_ABCD_GROUP:-none}"
CONTACT_PROBE_ADAPTIVE_FORCE_HOLD="${CONTACT_PROBE_ADAPTIVE_FORCE_HOLD:-false}"
CONTACT_PROBE_MMC_CONTROL_VARIANT="${CONTACT_PROBE_MMC_CONTROL_VARIANT:-fixed}"
CONTACT_PROBE_ROD_DISTURBANCE="${CONTACT_PROBE_ROD_DISTURBANCE:-0}"
CONTACT_PROBE_PAYLOAD_FEEDFORWARD="${CONTACT_PROBE_PAYLOAD_FEEDFORWARD:-0}"
CONTACT_PROBE_ROD_FORCE_DURATION_SEC="${CONTACT_PROBE_ROD_FORCE_DURATION_SEC:-0.2}"
if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" != "0" && "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" != "1" ]]; then
  echo "error: CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY must be 0 or 1" >&2
  exit 2
fi
if [[ "${CONTACT_HOVER_ONLY}" != "0" && "${CONTACT_HOVER_ONLY}" != "1" ]]; then
  echo "error: CONTACT_PROBE_HOVER_ONLY must be 0 or 1" >&2
  exit 2
fi
if [[ "${CONTACT_HOVER_ONLY}" -eq 1 && "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]]; then
  echo "error: hover-only and fixed-arm diagnostic modes are mutually exclusive" >&2
  exit 2
fi
case "${CONTACT_HOVER_CONTROL_MODE}" in
  jtc|mmc_hold) ;;
  *) echo "error: CONTACT_PROBE_HOVER_CONTROL_MODE must be jtc or mmc_hold" >&2; exit 2 ;;
esac
if [[ "${CONTACT_HOVER_ONLY}" -ne 1 && "${CONTACT_HOVER_CONTROL_MODE}" != "jtc" ]]; then
  echo "error: mmc_hold requires CONTACT_PROBE_HOVER_ONLY=1" >&2; exit 2
fi
case "${CONTACT_PROBE_ABCD_GROUP}" in
  none|A|B|C|D) ;;
  *) echo "error: CONTACT_PROBE_ABCD_GROUP must be none, A, B, C or D" >&2; exit 2 ;;
esac
if [[ "${CONTACT_PROBE_ABCD_GROUP}" != "none" &&
      ( "${CONTACT_HOVER_ONLY}" -eq 1 || "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ) ]]; then
  echo "error: ABCD mode is exclusive with hover and hold diagnostics" >&2
  exit 2
fi
export CONTACT_HOVER_CONTROL_MODE
export CONTACT_PROBE_ABCD_GROUP
case "${CONTACT_PROBE_ADAPTIVE_FORCE_HOLD}" in
  true|false|1|0) ;;
  *) echo "error: CONTACT_PROBE_ADAPTIVE_FORCE_HOLD must be true/false/1/0" >&2; exit 2 ;;
esac
export CONTACT_PROBE_ADAPTIVE_FORCE_HOLD
case "${CONTACT_PROBE_MMC_CONTROL_VARIANT}" in
  fixed|directional) ;;
  *) echo "error: CONTACT_PROBE_MMC_CONTROL_VARIANT must be fixed or directional" >&2; exit 2 ;;
esac
if [[ "${CONTACT_PROBE_ROD_DISTURBANCE}" != "0" && "${CONTACT_PROBE_ROD_DISTURBANCE}" != "1" ]]; then
  echo "error: CONTACT_PROBE_ROD_DISTURBANCE must be 0 or 1" >&2; exit 2
fi
if [[ "${CONTACT_PROBE_ROD_DISTURBANCE}" == "1" && "${CONTACT_HOVER_CONTROL_MODE}" != "mmc_hold" ]]; then
  echo "error: rod disturbance requires CONTACT_HOVER_CONTROL_MODE=mmc_hold" >&2; exit 2
fi
if [[ "${CONTACT_PROBE_PAYLOAD_FEEDFORWARD}" != "0" && "${CONTACT_PROBE_PAYLOAD_FEEDFORWARD}" != "1" ]]; then
  echo "error: CONTACT_PROBE_PAYLOAD_FEEDFORWARD must be 0 or 1" >&2; exit 2
fi
if [[ "${CONTACT_PROBE_PAYLOAD_FEEDFORWARD}" == "1" && "${CONTACT_HOVER_CONTROL_MODE}" != "mmc_hold" ]]; then
  echo "error: payload feedforward requires CONTACT_HOVER_CONTROL_MODE=mmc_hold" >&2; exit 2
fi
if ! awk -v d="${CONTACT_PROBE_ROD_FORCE_DURATION_SEC}" 'BEGIN { exit !(d > 0 && d <= 5) }'; then
  echo "error: CONTACT_PROBE_ROD_FORCE_DURATION_SEC must be in (0, 5] s" >&2; exit 2
fi
export CONTACT_PROBE_MMC_CONTROL_VARIANT CONTACT_PROBE_ROD_DISTURBANCE
export CONTACT_PROBE_PAYLOAD_FEEDFORWARD CONTACT_PROBE_ROD_FORCE_DURATION_SEC

if [[ "${INTERNAL_COMPLIANCE}" -eq 1 ]]; then
  RUN_ID="${CONTACT_PROBE_RUN_ID:-lidio_contact_probe_compliant_$(date +%Y%m%d_%H%M%S)}"
else
  RUN_ID="${CONTACT_PROBE_RUN_ID:-lidio_contact_probe_yaw10_$(date +%Y%m%d_%H%M%S)}"
fi
if ! [[ "${RUN_ID}" =~ ^[A-Za-z0-9_.-]+$ ]] || [[ "${RUN_ID}" == "." || "${RUN_ID}" == ".." ]]; then
  echo "error: CONTACT_PROBE_RUN_ID may only contain letters, digits, '_', '-' and '.'" >&2
  exit 2
fi
RUN_DIR="${WS_DIR}/experiment_results/a1_runs/${RUN_ID}"
if [[ -e "${RUN_DIR}" ]]; then
  echo "error: run directory already exists; refusing to overwrite: ${RUN_DIR}" >&2
  exit 2
fi
export ROS_LOG_DIR="${WS_DIR}/launch_logs/${RUN_ID}"
if [[ -e "${ROS_LOG_DIR}" ]]; then
  echo "error: ROS log directory already exists; refusing to reuse: ${ROS_LOG_DIR}" >&2
  exit 2
fi
mkdir -p "${ROS_LOG_DIR}"
export CONTACT_PROBE_RUN_DIR="${RUN_DIR}"

if [[ "${CONTACT_HOVER_ONLY}" -eq 1 ]]; then
  echo "contact_probe: RIGID suspended hover, lift=${CONTACT_HOVER_HEIGHT_M}m hold=${CONTACT_HOVER_HOLD_SECONDS}s mode=${CONTACT_HOVER_CONTROL_MODE}; no transport"
elif [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]]; then
  echo "contact_probe: HOLD-MECHANISM diagnostic only at (0.50, 0.00, +10 deg); no lift"
else
  echo "contact_probe: full fixed initial pose (${CONTACT_PROBE_START_X:-0.50}, ${CONTACT_PROBE_START_Y:-0.0}, +${CONTACT_PROBE_START_YAW_DEG:-10.0} deg), current default profile"
fi
if [[ "${INTERNAL_COMPLIANCE}" -eq 1 ]]; then
  echo "contact_probe profile: COMPLIANT (internal compliance enabled; CONTACT_PROBE_RIGID=false)"
else
  echo "contact_probe profile: RIGID (internal compliance disabled; CONTACT_PROBE_RIGID=true)"
fi
if [[ "${CONTACT_PROBE_ABCD_GROUP}" != "none" ]]; then
  echo "ABCD group: ${CONTACT_PROBE_ABCD_GROUP}; full lift/transport/lower/release/retreat"
fi
echo "MuJoCo: ${MUJOCO_DIR}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "ROS_HOME: ${ROS_HOME:-<ROS default>}"
echo "run id: ${RUN_ID}"
echo "run data: ${RUN_DIR}"
echo "ROS logs: ${ROS_LOG_DIR}"
echo "This invokes one run only; failures are preserved and are not retried."
if [[ "${OBSERVE_ON_ERROR}" -eq 1 ]]; then
  echo "Failure observation: enabled (interactive visible run only)"
fi

RUN_ARGS=(
  --run-id "${RUN_ID}" \
  --domain-id "${ROS_DOMAIN_ID}" \
  --prepare-mode \
  --grasp-model contact_probe \
  --x "${CONTACT_PROBE_START_X:-0.50}" --y "${CONTACT_PROBE_START_Y:-0.0}"
  --yaw-deg "${CONTACT_PROBE_START_YAW_DEG:-10.0}" \
  "${DISPLAY_FLAG}" \
  --ready-timeout 120 \
  --task-timeout 300 \
  --hold-sec 360 \
  --node-timeout 360
)
if [[ -n "${CONTACT_PROBE_TARGET_X:-}" && -n "${CONTACT_PROBE_TARGET_Y:-}" && -n "${CONTACT_PROBE_TARGET_YAW_DEG:-}" ]]; then
  RUN_ARGS+=(--target-x "${CONTACT_PROBE_TARGET_X}" --target-y "${CONTACT_PROBE_TARGET_Y}"
             --target-yaw-deg "${CONTACT_PROBE_TARGET_YAW_DEG}")
fi
if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]]; then
  RUN_ARGS+=(--contact-hold-diagnostic-only)
fi
if [[ "${CONTACT_HOVER_ONLY}" -eq 1 ]]; then
  RUN_ARGS+=(--contact-hover-only --contact-hover-height "${CONTACT_HOVER_HEIGHT_M}"
             --contact-hover-hold-sec "${CONTACT_HOVER_HOLD_SECONDS}"
             --contact-hover-control-mode "${CONTACT_HOVER_CONTROL_MODE}")
fi
if [[ "${OBSERVE_ON_ERROR}" -eq 1 ]]; then
  RUN_ARGS+=(--observe-on-error)
fi
if bash scripts/a1_run_scene.sh "${RUN_ARGS[@]}"; then
  RUN_STATUS=0
else
  RUN_STATUS=$?
fi

echo "runner exit=${RUN_STATUS}"
if [[ "${CONTACT_HOVER_ONLY}" -eq 1 && -d "${RUN_DIR}" ]]; then
  python3 scripts/analyze_contact_hover.py --run-dir "${RUN_DIR}" \
    --output "${RUN_DIR}/hover_analysis.json" > "${RUN_DIR}/hover_analysis.txt" 2>&1 || \
    echo "note: analysis failed; see hover_analysis.txt; original runner exit remains ${RUN_STATUS}"
  echo "hover diagnostics: hover_diagnostics.csv hover_analysis.json hover_analysis.txt"
fi
echo "Return these files from ${RUN_DIR}:"
echo "  prepare_status.json task_launch.log sim_dual_moveit.log contact_probe.csv"
echo "  gripper_fingers_mj_left.csv gripper_fingers_mj_right.csv controllers.txt runner.log"
echo "ROS logs: ${ROS_LOG_DIR}"
exit "${RUN_STATUS}"
