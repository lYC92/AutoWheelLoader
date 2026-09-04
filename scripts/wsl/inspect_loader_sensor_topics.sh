#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
urdf_file="${runtime_root}/results/loader.sensors.urdf"
server_log="${runtime_root}/log/loader_sensors_inspect.log"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

mkdir -p "${runtime_root}/results" "${runtime_root}/log"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_lidar_imu:=true >"${urdf_file}"

server_pid=''
cleanup() {
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

gz sim -s -r "${project_root}/simulation/worlds/loader_sensors.sdf" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_sensors/create$'; then
    break
  fi
  sleep 1
done

ros2 run ros_gz_sim create \
  -world loader_sensors \
  -file "${urdf_file}" \
  -name sensor_loader \
  -z 0.20

sleep 5
mapfile -t topics < <(gz topic -l | grep -E '^/loader/sensors|^/clock$' || true)
if [[ ${#topics[@]} -eq 0 ]]; then
  printf 'FAIL  No loader sensor topics were created.\n' >&2
  tail -n 100 "${server_log}" >&2
  exit 1
fi

for topic in "${topics[@]}"; do
  printf 'TOPIC %s\n' "${topic}"
  gz topic -i -t "${topic}"
done
