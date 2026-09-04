#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
source_root="${project_root}/ros_ws/src"
runtime_root="${HOME}/loader_sim_runtime"

set +u
source /opt/ros/jazzy/setup.bash
set -u

# WSL imports the Windows PATH by default.  CMake can otherwise discover
# Windows-only SDKs (notably Anaconda's Protobuf) and combine their headers
# with Ubuntu / ROS libraries.  Keep project builds on the native WSL toolchain.
IFS=: read -ra inherited_path_entries <<< "${PATH}"
native_path_entries=()
for path_entry in "${inherited_path_entries[@]}"; do
  if [[ "${path_entry}" != /mnt/?/* ]]; then
    native_path_entries+=("${path_entry}")
  fi
done
PATH="$(IFS=:; printf '%s' "${native_path_entries[*]}")"
export PATH

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
