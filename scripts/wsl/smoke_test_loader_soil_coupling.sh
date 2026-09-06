#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
mode="${1:-physics}"
if [[ ${mode} != physics && ${mode} != perception ]]; then
  printf 'Usage: %s [physics|perception]\n' "$0" >&2
  exit 2
fi
suffix=""
world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
if [[ ${mode} == perception ]]; then
  suffix="_perception"
  world_file="${project_root}/simulation/worlds/loader_soil_perception.sdf"
fi
urdf_file="${runtime_root}/results/loader.soil_coupling.urdf"
server_log="${runtime_root}/log/loader_soil_coupling${suffix}_gazebo.log"
rsp_log="${runtime_root}/log/loader_soil_coupling${suffix}_robot_state_publisher.log"
bridge_log="${runtime_root}/log/loader_soil_coupling${suffix}_bridge.log"
test_log="${runtime_root}/results/loader_soil_coupling${suffix}.txt"
pose_log="${runtime_root}/results/loader_soil_coupling${suffix}_pose.txt"
proxy_pose_log="${runtime_root}/results/loader_soil_proxy${suffix}_column_pose.txt"
proxy_expectation_log="${runtime_root}/results/loader_soil_proxy${suffix}_expectation.txt"
soil_plugin_dir="${runtime_root}/install/loader_soil/lib"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${soil_plugin_dir}:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${soil_plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${runtime_root}/results" "${runtime_root}/log"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_soil_slice:=true >"${urdf_file}"

server_pid=''
rsp_pid=''
bridge_pid=''
cleanup() {
  for process_id in "${bridge_pid}" "${rsp_pid}" "${server_pid}"; do
    if [[ -n ${process_id} ]]; then
      kill "${process_id}" >/dev/null 2>&1 || true
      wait "${process_id}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

bridge_arguments=('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
# Reproduce slow GUI/sensor initialization: gravity must not determine whether
# the driving test can start. Phases always use simulation time on slow hosts.
test_arguments=(--proxy-expectation "${proxy_expectation_log}" --use-sim-time-for-phases --startup-settle-s 8)
if [[ ${mode} == perception ]]; then
  bridge_arguments+=(
    '/loader_soil/observer/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
  )
  test_arguments+=(--observer-topic /loader_soil/observer/scan/points)
fi
ros2 run ros_gz_bridge parameter_bridge "${bridge_arguments[@]}" >"${bridge_log}" 2>&1 &
bridge_pid=$!
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true -p robot_description:="$(<"${urdf_file}")" \
  >"${rsp_log}" 2>&1 &
rsp_pid=$!
gz sim -s -r "${world_file}" \
  >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before soil-coupling entity creation.\n' >&2
    tail -n 120 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

spawn_output="$(ros2 run ros_gz_sim create \
  -world loader_soil_slice \
  -file "${urdf_file}" \
  -name soil_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf 'FAIL  Soil loader entity creation did not report success.\n' >&2
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

python3 "${project_root}/tools/ros/test_loader_soil_coupling.py" \
  "${test_arguments[@]}" 2>&1 | tee "${test_log}"
gz model -m soil_loader -p >"${pose_log}"
: >"${proxy_pose_log}"
read -r proxy_index _ <"${proxy_expectation_log}"
printf -v proxy_name 'soil_column_%03d' "${proxy_index}"
gz model -m "${proxy_name}" -p >"${proxy_pose_log}"
python3 "${project_root}/tools/soil_slice/verify_soil_proxy_pose.py" \
  "${proxy_pose_log}" "${proxy_expectation_log}"

if grep -Eqi 'Failed to load|Could not load|exception|Segmentation fault|terminate called' \
    "${server_log}"; then
  printf 'FAIL  Gazebo log contains a loader-soil integration error.\n' >&2
  tail -n 160 "${server_log}" >&2
  exit 1
fi

printf 'PASS  Full loader, ros2_control, and nominal soil slice are coupled.\n'
printf 'Gazebo log: %s\n' "${server_log}"
printf 'Test result: %s\n' "${test_log}"
printf 'Final pose: %s\n' "${pose_log}"
printf 'Soil proxy poses: %s\n' "${proxy_pose_log}"
