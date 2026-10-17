#!/usr/bin/env bash
# One user-supervised, visible +10 degree contact-load probe on this workstation.
# This wrapper never builds, retries, or cleans up unrelated processes.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

OBSERVE_ON_ERROR=0
# Display mode: headless for experiments (default), --use-rviz to watch the
# simulation. Pass exactly one; --headless is the safe default.
DISPLAY_FLAG="--headless"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) DISPLAY_FLAG="--headless"; shift ;;
    --use-rviz) DISPLAY_FLAG="--use-rviz"; shift ;;
    --observe-on-error) OBSERVE_ON_ERROR=1; shift ;;
    -h|--help)
      echo "usage: run_contact_probe_local.sh [--headless|--use-rviz] [--observe-on-error]"
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
if [[ "${CONTACT_PROBE_RIGID:-true}" != "true" ]]; then
  echo "error: CONTACT_PROBE_RIGID is not true; unset the override for the baseline probe" >&2
  exit 2
fi
export CONTACT_PROBE_RIGID=true
CONTACT_HOLD_DIAGNOSTIC_ONLY="${CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY:-0}"
if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" != "0" && "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" != "1" ]]; then
  echo "error: CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY must be 0 or 1" >&2
  exit 2
fi

RUN_ID="${CONTACT_PROBE_RUN_ID:-lidio_contact_probe_yaw10_$(date +%Y%m%d_%H%M%S)}"
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

if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]]; then
  echo "contact_probe: HOLD-MECHANISM diagnostic only at (0.50, 0.00, +10 deg); no lift"
else
  echo "contact_probe: full fixed initial pose (0.50, 0.00, +10 deg), current default profile"
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
  --x 0.50 --y 0.0 --yaw-deg 10.0 \
  "${DISPLAY_FLAG}" \
  --ready-timeout 120 \
  --task-timeout 300 \
  --hold-sec 360 \
  --node-timeout 360
)
if [[ "${CONTACT_HOLD_DIAGNOSTIC_ONLY}" -eq 1 ]]; then
  RUN_ARGS+=(--contact-hold-diagnostic-only)
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
echo "Return these files from ${RUN_DIR}:"
echo "  prepare_status.json task_launch.log sim_dual_moveit.log contact_probe.csv"
echo "  gripper_fingers_mj_left.csv gripper_fingers_mj_right.csv controllers.txt runner.log"
echo "ROS logs: ${ROS_LOG_DIR}"
exit "${RUN_STATUS}"
