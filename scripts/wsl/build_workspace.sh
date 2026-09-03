#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
source_root="${project_root}/ros_ws/src"
runtime_root="${HOME}/loader_sim_runtime"

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p \
  "${runtime_root}/build" \
  "${runtime_root}/install" \
  "${runtime_root}/log"

rosdep install \
  --from-paths "${source_root}" \
  --ignore-src \
  --rosdistro jazzy \
  -y

colcon \
  --log-base "${runtime_root}/log" \
  build \
  --base-paths "${source_root}" \
  --build-base "${runtime_root}/build" \
  --install-base "${runtime_root}/install" \
  --symlink-install \
  --event-handlers console_cohesion+

printf 'PASS  Workspace built into %s\n' "${runtime_root}"
