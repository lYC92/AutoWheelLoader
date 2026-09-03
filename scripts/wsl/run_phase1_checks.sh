#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

printf '%s\n' '=== Environment ==='
bash "${script_dir}/verify_environment.sh"

printf '%s\n' '=== ROS 2 DDS ==='
bash "${script_dir}/smoke_test_ros.sh"

# Let the WSLg D3D12 context used by glxinfo fully release before Gazebo starts.
sleep 5
printf '%s\n' '=== Gazebo camera ==='
if ! bash "${script_dir}/smoke_test_gazebo.sh"; then
  printf 'WARN  Gazebo check failed once; waiting 5 seconds before the final retry.\n' >&2
  sleep 5
  bash "${script_dir}/smoke_test_gazebo.sh"
fi

printf '%s\n' 'PASS: all phase-1 checks completed.'
