#!/usr/bin/env bash
# Build with the dependencies selected during the Ubuntu 22.04 migration.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export MUJOCO_DIR="${MUJOCO_DIR:-${HOME}/.local/mujoco-3.2.0}"
dqrobotics_prefix="${workspace_dir}/.deps/dqrobotics"
if [[ ! -f "${MUJOCO_DIR}/include/mujoco/mujoco.h" || ! -f "${MUJOCO_DIR}/lib/libmujoco.so" ]]; then
  echo "MuJoCo 3.2.0 installation missing: ${MUJOCO_DIR}" >&2
  exit 1
fi
if [[ ! -f "${dqrobotics_prefix}/include/dqrobotics/DQ.h" || ! -f "${dqrobotics_prefix}/lib/libdqrobotics.so" ]]; then
  echo "DQ Robotics installation missing; see docs/LOCAL_BUILD.md" >&2
  exit 1
fi
export CMAKE_PREFIX_PATH="${dqrobotics_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
cd -- "${workspace_dir}"
colcon build --symlink-install --parallel-workers 2 \
  "$@" --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" "-DMUJOCO_DIR=${MUJOCO_DIR}"
ln -sfn build/compile_commands.json compile_commands.json
echo "IntelliSense database: ${workspace_dir}/compile_commands.json"
