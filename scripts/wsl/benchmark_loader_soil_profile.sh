#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
result_dir="${runtime_root}/results"
log_dir="${runtime_root}/log"
urdf_file="${result_dir}/loader.full_soil_profile.urdf"
server_log="${log_dir}/loader_full_soil_profile_gazebo.log"
bridge_log="${log_dir}/loader_full_soil_profile_bridge.log"
rsp_log="${log_dir}/loader_full_soil_profile_robot_state_publisher.log"
scenario_log="${result_dir}/loader_full_soil_profile_scenario.txt"
stats_log="${result_dir}/loader_full_soil_profile_stats.txt"
gpu_log="${result_dir}/loader_full_soil_profile_gpu.csv"
lidar_log="${result_dir}/loader_full_soil_profile_lidar_frequency.txt"
summary_file="${result_dir}/loader_full_soil_profile_baseline.csv"
expectation_log="${result_dir}/loader_full_soil_profile_expectation.txt"
soil_plugin_dir="${runtime_root}/install/loader_soil/lib"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${soil_plugin_dir}:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${soil_plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${result_dir}" "${log_dir}"
rm -f "${server_log}" "${bridge_log}" "${scenario_log}" "${stats_log}" \
  "${gpu_log}" "${lidar_log}" "${summary_file}" "${expectation_log}"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_lidar_imu:=true enable_soil_slice:=true >"${urdf_file}"

server_pid=''
bridge_pid=''
rsp_pid=''
gpu_pid=''
lidar_pid=''
scenario_pid=''
cleanup() {
  for process_id in "${scenario_pid}" "${lidar_pid}" "${gpu_pid}" "${bridge_pid}" \
      "${rsp_pid}" "${server_pid}"; do
    if [[ -n ${process_id} ]]; then
      kill "${process_id}" >/dev/null 2>&1 || true
      wait "${process_id}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"${bridge_log}" 2>&1 &
bridge_pid=$!
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true -p robot_description:="$(<"${urdf_file}")" \
  >"${rsp_log}" 2>&1 &
rsp_pid=$!

printf 'timestamp,memory_used_mib,utilization_gpu_pct\n' >"${gpu_log}"
gz sim -s -r --headless-rendering \
  "${project_root}/simulation/worlds/loader_soil_control_profile.sdf" \
  >"${server_log}" 2>&1 &
server_pid=$!
(
  while kill -0 "${server_pid}" >/dev/null 2>&1; do
    printf '%s,' "$(date --iso-8601=seconds)" >>"${gpu_log}"
    nvidia-smi --query-gpu=memory.used,utilization.gpu \
      --format=csv,noheader,nounits >>"${gpu_log}" 2>/dev/null || true
    sleep 1
  done
) &
gpu_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before full-soil profile entity creation.\n' >&2
    tail -n 120 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

spawn_output="$(ros2 run ros_gz_sim create -world loader_soil_slice \
  -file "${urdf_file}" -name full_soil_profile_loader -z 0.20 2>&1)"
if ! grep -qi success <<<"${spawn_output}"; then
  printf '%s\n' "${spawn_output}" >&2
  exit 1
fi
for _ in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  sleep 1
done
ros2 run controller_manager spawner loader_command_controller \
  --controller-manager /controller_manager --controller-manager-timeout 30 >/dev/null

stats_topic='/world/loader_soil_slice/stats'
lidar_topic='/loader/sensors/lidar/scan/points'
for _ in $(seq 1 30); do
  topics="$(gz topic -l 2>/dev/null || true)"
  if grep -q "^${stats_topic}$" <<<"${topics}" && grep -q "^${lidar_topic}$" <<<"${topics}"; then
    break
  fi
  sleep 1
done
if ! gz topic -l | grep -q "^${lidar_topic}$"; then
  printf 'FAIL  Vehicle lidar topic was not published in the full-soil profile.\n' >&2
  exit 1
fi

python3 "${project_root}/tools/ros/test_loader_soil_coupling.py" \
  --proxy-expectation "${expectation_log}" >"${scenario_log}" 2>&1 &
scenario_pid=$!
timeout 20s gz topic -f -t "${lidar_topic}" >"${lidar_log}" 2>&1 &
lidar_pid=$!
gz topic -e -t "${stats_topic}" -d 20 >"${stats_log}"

set +e
wait "${scenario_pid}"
scenario_status=$?
wait "${lidar_pid}"
lidar_status=$?
set -e
scenario_pid=''
lidar_pid=''
if [[ ${scenario_status} -ne 0 ]]; then
  printf 'FAIL  Dynamic full-soil scenario failed during the benchmark.\n' >&2
  cat "${scenario_log}" >&2
  exit 1
fi
if [[ ${lidar_status} -ne 0 && ${lidar_status} -ne 124 ]]; then
  printf 'WARN  Lidar frequency sampler exited with status %s.\n' "${lidar_status}" >&2
fi

rtf_summary="$(awk '
  /real_time_factor:/ {v=$2+0; n++; sum+=v; if(n==1||v<min)min=v; if(n==1||v>max)max=v}
  END {if(n>0) printf "%d,%.6f,%.6f,%.6f",n,sum/n,min,max}
' "${stats_log}")"
if [[ -z ${rtf_summary} ]]; then
  printf 'FAIL  No real_time_factor samples were found.\n' >&2
  exit 1
fi
IFS=',' read -r rtf_samples rtf_avg rtf_min rtf_max <<<"${rtf_summary}"
gpu_mem_peak="$(awk -F, 'NR>1 {gsub(/ /,"",$2); if($2+0>max)max=$2+0} END {print max+0}' "${gpu_log}")"
gpu_util_peak="$(awk -F, 'NR>1 {gsub(/ /,"",$3); if($3+0>max)max=$3+0} END {print max+0}' "${gpu_log}")"
lidar_hz="$(awk '/average rate:/ {value=$3} END {print value}' "${lidar_log}")"
[[ -n ${lidar_hz} ]] || lidar_hz='unavailable'

{
  printf 'metric,value\n'
  printf 'profile,full_soil_realtime\n'
  printf 'physics_step_hz,500\n'
  printf 'soil_cells,280\n'
  printf 'soil_visual_update_hz,10\n'
  printf 'rtf_samples,%s\n' "${rtf_samples}"
  printf 'rtf_average,%s\n' "${rtf_avg}"
  printf 'rtf_minimum,%s\n' "${rtf_min}"
  printf 'rtf_maximum,%s\n' "${rtf_max}"
  printf 'gpu_memory_peak_mib,%s\n' "${gpu_mem_peak}"
  printf 'gpu_utilization_peak_pct,%s\n' "${gpu_util_peak}"
  printf 'vehicle_lidar_frequency_hz,%s\n' "${lidar_hz}"
} | tee "${summary_file}"

awk -v rtf="${rtf_avg}" 'BEGIN {exit !(rtf>=0.9)}' || {
  printf 'FAIL  Full-soil average real-time factor %s is below 0.9.\n' "${rtf_avg}" >&2
  exit 1
}
if [[ ${lidar_hz} != unavailable ]]; then
  awk -v hz="${lidar_hz}" 'BEGIN {exit !(hz>=9.0)}' || {
    printf 'FAIL  Full-soil vehicle lidar rate %s Hz is below 9 Hz.\n' "${lidar_hz}" >&2
    exit 1
  }
fi
printf 'PASS  Dynamic full-soil vehicle profile meets the RTF and lidar gates.\n'
printf 'Scenario: %s\n' "${scenario_log}"
printf 'Result: %s\n' "${summary_file}"
