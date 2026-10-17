#!/usr/bin/env bash
# No-load validation of the dual-arm Cartesian impedance chain.
#
# Starts ONE simulation instance (the MoveIt launch includes MuJoCo), loads the
# multi-mode controller inactive, and runs the bridge that performs:
#   switch to impedance -> hold 3 s -> both hands up 5 mm -> hold 3 s ->
#   switch back to the trajectory controller, seeded from the measured joints.
#
# No gripper command, no weld, no object interaction. Failures are preserved.
set -uo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

HEADLESS=true
USE_RVIZ=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS=true; USE_RVIZ=false; shift ;;
    --use-rviz) HEADLESS=false; USE_RVIZ=true; shift ;;
    *) echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

RUN_ID="${RUN_ID:-dual_impedance_no_load_$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="${WS_DIR}/experiment_results/a1_runs/${RUN_ID}"
if [[ -e "${RUN_DIR}" ]]; then
  echo "error: run directory already exists: ${RUN_DIR}" >&2
  exit 2
fi
mkdir -p "${RUN_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

MUJOCO_DIR="${MUJOCO_DIR:-/home/lidio/Libraries/mujoco}"
export MUJOCO_DIR
case ":${LD_LIBRARY_PATH:-}:" in
  *":${MUJOCO_DIR}/lib:"*) ;;
  *) export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-113}"
export ROS_LOCALHOST_ONLY=1

echo "run id : ${RUN_ID}"
echo "data   : ${RUN_DIR}"
echo "This starts one simulation instance and sends no gripper or weld command."

# headless does NOT disable RViz (that is a separate flag), and RViz does not
# always quit on SIGINT, which leaves the whole launch tree hanging.
setsid ros2 launch franka_moveit_config sim_dual_moveit.launch.py \
  headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" \
  > "${RUN_DIR}/sim_dual_moveit.log" 2>&1 &
SIM_PID=$!
cleanup() {
  kill -0 "${SIM_PID}" 2>/dev/null || return 0
  kill -INT -- "-${SIM_PID}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${SIM_PID}" 2>/dev/null || return 0
    sleep 0.5
  done
  echo "[run] launch did not exit on SIGINT; escalating" >&2
  kill -TERM -- "-${SIM_PID}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    kill -0 "${SIM_PID}" 2>/dev/null || return 0
    sleep 0.5
  done
  kill -KILL -- "-${SIM_PID}" 2>/dev/null || true
}
trap cleanup EXIT

READY=0
DEADLINE=$((SECONDS + 180))
while (( SECONDS < DEADLINE )); do
  STATE="$(timeout 10 ros2 control list_controllers 2>/dev/null || true)"
  if echo "${STATE}" | grep -q "dual_panda_arm_controller.*active"; then
    READY=1
    break
  fi
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "error: simulation exited during start-up; see sim_dual_moveit.log" >&2
    exit 2
  fi
  sleep 3
done
timeout 10 ros2 control list_controllers > "${RUN_DIR}/controllers_before.txt" 2>&1 || true
if (( READY != 1 )); then
  echo "error: dual_panda_arm_controller did not become active" >&2
  exit 2
fi
echo "[run] controller manager ready"

# Load + configure the multi-mode controller without activating it; activation
# is the switched step under test.
ros2 run controller_manager spawner sim_multi_mode_controller --inactive \
  > "${RUN_DIR}/spawn_multi_mode.log" 2>&1
SPAWN_RC=$?
echo "[run] spawn_multi_mode_controller exit=${SPAWN_RC}"

ros2 run dual_arm_carry_task dual_impedance_no_load_bridge --ros-args \
  -p hold_seconds:=3.0 \
  -p move_amplitude_m:=0.005 \
  -p move_seconds:=1.0 \
  -p final_hold_seconds:=3.0 \
  -p rate_hz:=100.0 \
  > "${RUN_DIR}/bridge.log" 2>&1
BRIDGE_RC=$?
echo "[run] bridge exit=${BRIDGE_RC}"

timeout 10 ros2 control list_controllers > "${RUN_DIR}/controllers_after.txt" 2>&1 || true
cp "${RUN_DIR}/bridge.log" "${RUN_DIR}/bridge_summary.log" 2>/dev/null || true

echo "return these files: sim_dual_moveit.log bridge.log controllers_before.txt controllers_after.txt"
exit "${BRIDGE_RC}"
