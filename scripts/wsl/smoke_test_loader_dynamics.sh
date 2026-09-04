#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
world_file="${project_root}/simulation/worlds/loader_kinematics.sdf"
xacro_file="${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro"
urdf_file="${runtime_root}/results/loader.dynamics.urdf"
server_log="${runtime_root}/log/loader_dynamics_smoke_gazebo.log"
test_log="${runtime_root}/results/loader_dynamics_smoke.txt"
plugin_dir="${runtime_root}/install/loader_dynamics/lib"

if [[ -f /etc/profile.d/loader-sim-wslg.sh ]]; then
  source /etc/profile.d/loader-sim-wslg.sh
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_dir}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${runtime_root}/log" "${runtime_root}/results"
xacro "${xacro_file}" model_fidelity:=nominal enable_dynamics:=true >"${urdf_file}"

server_pid=''
cleanup() {
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

gz sim -s -r "${world_file}" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_kinematics/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before entity creation became available.\n' >&2
    tail -n 100 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

spawn_output="$(ros2 run ros_gz_sim create \
  -world loader_kinematics \
  -file "${urdf_file}" \
  -name dynamics_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

python3 "${project_root}/tools/ros/test_loader_dynamics.py" | tee "${test_log}"

if grep -Eqi 'Failed to load system plugin|Failed to initialize ROS|missing joint' "${server_log}"; then
  printf 'FAIL  Gazebo log contains a loader dynamics plugin error.\n' >&2
  tail -n 100 "${server_log}" >&2
  exit 1
fi

printf 'PASS  Gazebo loader dynamics plugin integration test completed.\n'
printf 'Gazebo log: %s\n' "${server_log}"
printf 'Test result: %s\n' "${test_log}"
