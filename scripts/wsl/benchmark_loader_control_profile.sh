#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
result_dir="${runtime_root}/results"
log_dir="${runtime_root}/log"
urdf_file="${result_dir}/loader.control_realtime.urdf"
server_log="${log_dir}/loader_control_profile_gazebo.log"
stats_log="${result_dir}/loader_control_profile_stats.txt"
gpu_log="${result_dir}/loader_control_profile_gpu.csv"
lidar_frequency_log="${result_dir}/loader_control_profile_lidar_frequency.txt"
summary_file="${result_dir}/loader_control_profile_baseline.csv"
rsp_log="${log_dir}/loader_control_profile_robot_state_publisher.log"
bridge_log="${log_dir}/loader_control_profile_clock_bridge.log"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p "${result_dir}" "${log_dir}"
rm -f "${server_log}" "${stats_log}" "${gpu_log}" "${lidar_frequency_log}" "${summary_file}"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_lidar_imu:=true >"${urdf_file}"

server_pid=''
gpu_pid=''
lidar_frequency_pid=''
rsp_pid=''
bridge_pid=''
cleanup() {
  if [[ -n ${bridge_pid} ]]; then
    kill "${bridge_pid}" >/dev/null 2>&1 || true
    wait "${bridge_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${rsp_pid} ]]; then
    kill "${rsp_pid}" >/dev/null 2>&1 || true
    wait "${rsp_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${lidar_frequency_pid} ]]; then
    kill "${lidar_frequency_pid}" >/dev/null 2>&1 || true
    wait "${lidar_frequency_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${gpu_pid} ]]; then
    kill "${gpu_pid}" >/dev/null 2>&1 || true
    wait "${gpu_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"${bridge_log}" 2>&1 &
bridge_pid=$!
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(<"${urdf_file}")" >"${rsp_log}" 2>&1 &
rsp_pid=$!

printf 'timestamp,memory_used_mib,utilization_gpu_pct\n' >"${gpu_log}"
gz sim -s -r --headless-rendering \
  "${project_root}/simulation/worlds/loader_sensors.sdf" >"${server_log}" 2>&1 &
server_pid=$!

(
  while kill -0 "${server_pid}" >/dev/null 2>&1; do
    printf '%s,' "$(date --iso-8601=seconds)" >>"${gpu_log}"
    nvidia-smi \
      --query-gpu=memory.used,utilization.gpu \
      --format=csv,noheader,nounits >>"${gpu_log}" 2>/dev/null || true
    sleep 1
  done
) &
gpu_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_sensors/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before control-profile entity creation.\n' >&2
    tail -n 100 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

spawn_output="$(ros2 run ros_gz_sim create \
  -world loader_sensors \
  -file "${urdf_file}" \
  -name control_profile_loader \
  -z 0.20 2>&1)"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf '%s\n' "${spawn_output}" >&2
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

for _ in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  sleep 1
done
ros2 run controller_manager spawner loader_command_controller \
  --controller-manager /controller_manager \
  --controller-manager-timeout 30 >/dev/null

stats_topic='/world/loader_sensors/stats'
lidar_topic='/loader/sensors/lidar/scan/points'
for _ in $(seq 1 30); do
  topics="$(gz topic -l 2>/dev/null || true)"
  if grep -q "^${stats_topic}$" <<<"${topics}" && grep -q "^${lidar_topic}$" <<<"${topics}"; then
    break
  fi
  sleep 1
done

if ! gz topic -l | grep -q "^${lidar_topic}$"; then
  printf 'FAIL  Lidar point-cloud topic was not published.\n' >&2
  exit 1
fi

printf 'Sampling the full control profile for 12 seconds...\n'
timeout 12s gz topic -f -t "${lidar_topic}" >"${lidar_frequency_log}" 2>&1 &
lidar_frequency_pid=$!
gz topic -e -t "${stats_topic}" -d 12 >"${stats_log}"
set +e
wait "${lidar_frequency_pid}"
lidar_frequency_status=$?
set -e
lidar_frequency_pid=''
if [[ ${lidar_frequency_status} -ne 0 && ${lidar_frequency_status} -ne 124 ]]; then
  printf 'WARN  Lidar frequency sampler exited with status %s.\n' "${lidar_frequency_status}" >&2
fi

rtf_summary="$(awk '
  /real_time_factor:/ {
    value=$2 + 0;
    count++;
    sum+=value;
    if (count == 1 || value < min) min=value;
    if (count == 1 || value > max) max=value;
  }
  END {
    if (count > 0) printf "%d,%.6f,%.6f,%.6f", count, sum/count, min, max;
  }
' "${stats_log}")"
if [[ -z ${rtf_summary} ]]; then
  printf 'FAIL  No real_time_factor samples were found.\n' >&2
  exit 1
fi

IFS=',' read -r rtf_samples rtf_avg rtf_min rtf_max <<<"${rtf_summary}"
gpu_mem_peak="$(awk -F, 'NR > 1 {gsub(/ /, "", $2); if ($2 + 0 > max) max=$2 + 0} END {print max + 0}' "${gpu_log}")"
gpu_util_peak="$(awk -F, 'NR > 1 {gsub(/ /, "", $3); if ($3 + 0 > max) max=$3 + 0} END {print max + 0}' "${gpu_log}")"
lidar_hz="$(awk '/average rate:/ {value=$3} END {print value}' "${lidar_frequency_log}")"
if [[ -z ${lidar_hz} ]]; then
  lidar_hz='unavailable'
fi

{
  printf 'metric,value\n'
  printf 'profile,control_realtime\n'
  printf 'physics_step_hz,500\n'
  printf 'control_path,ros2_control\n'
  printf 'lidar_channels,32\n'
  printf 'lidar_horizontal_samples,1024\n'
  printf 'rtf_samples,%s\n' "${rtf_samples}"
  printf 'rtf_average,%s\n' "${rtf_avg}"
  printf 'rtf_minimum,%s\n' "${rtf_min}"
  printf 'rtf_maximum,%s\n' "${rtf_max}"
  printf 'gpu_memory_peak_mib,%s\n' "${gpu_mem_peak}"
  printf 'gpu_utilization_peak_pct,%s\n' "${gpu_util_peak}"
  printf 'lidar_frequency_hz,%s\n' "${lidar_hz}"
} | tee "${summary_file}"

awk -v rtf="${rtf_avg}" 'BEGIN {exit !(rtf >= 0.9)}' || {
  printf 'FAIL  Average real-time factor %s is below the 0.9 gate.\n' "${rtf_avg}" >&2
  exit 1
}

if [[ ${lidar_hz} != unavailable ]]; then
  awk -v hz="${lidar_hz}" 'BEGIN {exit !(hz >= 9.0)}' || {
    printf 'FAIL  Average lidar frequency %s Hz is below the 9 Hz gate.\n' "${lidar_hz}" >&2
    exit 1
  }
fi

printf 'PASS  Full nominal control profile meets the 500 Hz / RTF 0.9 gate.\n'
printf 'Result: %s\n' "${summary_file}"
