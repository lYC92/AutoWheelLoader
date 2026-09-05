#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
urdf_file="${runtime_root}/results/loader.sensor_effects.urdf"
server_log="${runtime_root}/log/sensor_effects_smoke_gazebo.log"
bridge_log="${runtime_root}/log/sensor_effects_smoke_bridge.log"
effects_log="${runtime_root}/log/sensor_effects_smoke_node.log"
test_log="${runtime_root}/results/sensor_effects_smoke.txt"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

mkdir -p "${runtime_root}/results" "${runtime_root}/log"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_lidar_imu:=true \
  lidar_mount_offset_xyz:="0.01 0.0 -0.02" \
  imu_mount_offset_rpy:="0 0 0.005" >"${urdf_file}"

server_pid=''
bridge_pid=''
effects_pid=''
cleanup() {
  for pid in "${effects_pid}" "${bridge_pid}" "${server_pid}"; do
    if [[ -n ${pid} ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      wait "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

gz sim -s -r "${project_root}/simulation/worlds/loader_sensors.sdf" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_sensors/create$'; then
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
  -world loader_sensors \
  -file "${urdf_file}" \
  -name sensor_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

bridge_arguments=(
  '/loader/sensors/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
  '/loader/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
)
ros2 run ros_gz_bridge parameter_bridge "${bridge_arguments[@]}" >"${bridge_log}" 2>&1 &
bridge_pid=$!

ros2 run loader_sensor_effects lidar_effects_node --ros-args \
  -p use_sim_time:=true \
  -p dropout_probability:=0.10 \
  -p distortion_enabled:=true \
  -p random_seed:=7 >"${effects_log}" 2>&1 &
effects_pid=$!
sleep 2

python3 "${project_root}/tools/ros/test_sensor_effects.py" "${urdf_file}" | tee "${test_log}"

if grep -Eqi 'Unable to create|Failed to load|Segmentation fault|terminate called' \
    "${server_log}" "${bridge_log}" "${effects_log}"; then
  printf 'FAIL  a pipeline log contains a sensor error.\n' >&2
  tail -n 100 "${server_log}" >&2
  tail -n 100 "${bridge_log}" >&2
  tail -n 100 "${effects_log}" >&2
  exit 1
fi

printf 'PASS  Sensor effect channel, IMU noise, and mount perturbation smoke completed.\n'
printf 'Gazebo log: %s\n' "${server_log}"
printf 'Bridge log: %s\n' "${bridge_log}"
printf 'Effects node log: %s\n' "${effects_log}"
printf 'Test result: %s\n' "${test_log}"
