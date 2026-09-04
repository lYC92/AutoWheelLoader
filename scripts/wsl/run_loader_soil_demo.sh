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

world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
suffix=""
if [[ ${mode} == perception ]]; then
  world_file="${project_root}/simulation/worlds/loader_soil_perception.sdf"
  suffix="_perception"
fi

gui_config="${project_root}/simulation/config/gui/loader_demo.config"
urdf_file="${runtime_root}/results/loader.soil_demo.urdf"
server_log="${runtime_root}/log/loader_soil_demo${suffix}_gazebo.log"
gui_log="${runtime_root}/log/loader_soil_demo${suffix}_gui.log"
rsp_log="${runtime_root}/log/loader_soil_demo${suffix}_robot_state_publisher.log"
bridge_log="${runtime_root}/log/loader_soil_demo${suffix}_bridge.log"
scenario_log="${runtime_root}/results/loader_soil_demo${suffix}.txt"
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

if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/'; then
  printf '%s\n' 'ERROR: a loader_soil_slice Gazebo server is already running.' >&2
  printf '%s\n' 'Close the existing Gazebo window, then run this launcher again.' >&2
  exit 3
fi

server_pid=''
gui_pid=''
rsp_pid=''
bridge_pid=''

stop_process() {
  local process_id="$1"
  [[ -z ${process_id} ]] && return 0
  if ! kill -0 "${process_id}" >/dev/null 2>&1; then
    wait "${process_id}" >/dev/null 2>&1 || true
    return 0
  fi
  kill -INT "${process_id}" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    kill -0 "${process_id}" >/dev/null 2>&1 || break
    sleep 0.1
  done
  if kill -0 "${process_id}" >/dev/null 2>&1; then
    kill -KILL "${process_id}" >/dev/null 2>&1 || true
  fi
  wait "${process_id}" >/dev/null 2>&1 || true
}

cleanup() {
  stop_process "${bridge_pid}"
  stop_process "${rsp_pid}"
  stop_process "${gui_pid}"
  stop_process "${server_pid}"
}
trap cleanup EXIT INT TERM

bridge_arguments=('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
test_arguments=(--use-sim-time-for-phases)
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

printf '%s\n' 'Starting the Gazebo server...'
gz sim -s -r "${world_file}" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf '%s\n' 'ERROR: Gazebo exited before the world was ready.' >&2
    tail -n 160 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

if ! gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/create$'; then
  printf '%s\n' 'ERROR: timed out waiting for the Gazebo world.' >&2
  tail -n 160 "${server_log}" >&2
  exit 1
fi

printf '%s\n' 'Opening the Gazebo window. Initial scene loading can take several seconds...'
(
  # On this Windows 10 / WSLg host, running Qt and the simulation server on the
  # same D3D12 device can make Qt lose its OpenGL context.  Keep NVIDIA for the
  # server and sensors, and render only the interactive GUI with llvmpipe.
  unset GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_QPA_PLATFORM=xcb
  exec gz sim -g --gui-config "${gui_config}"
) >"${gui_log}" 2>&1 &
gui_pid=$!

spawn_output="$(ros2 run ros_gz_sim create \
  -world loader_soil_slice \
  -file "${urdf_file}" \
  -name soil_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf '%s\n' 'ERROR: loader entity creation did not report success.' >&2
  exit 1
fi

for _ in $(seq 1 40); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf '%s\n' 'ERROR: Gazebo exited while loading ros2_control.' >&2
    tail -n 160 "${server_log}" >&2
    exit 1
  fi
  sleep 0.5
done

ros2 run controller_manager spawner loader_command_controller \
  --controller-manager /controller_manager --controller-manager-timeout 30 >/dev/null

for _ in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '^/gui/move_to$'; then
    break
  fi
  if ! kill -0 "${gui_pid}" >/dev/null 2>&1; then
    printf '%s\n' 'ERROR: the Gazebo GUI exited while creating its 3D scene.' >&2
    tail -n 160 "${gui_log}" >&2
    exit 1
  fi
  sleep 0.5
done

if ! gz service -l 2>/dev/null | grep -q '^/gui/move_to$'; then
  printf '%s\n' 'ERROR: timed out waiting for the Gazebo GUI 3D scene.' >&2
  tail -n 160 "${gui_log}" >&2
  exit 1
fi

printf '%s\n' 'Loader is ready. The automated dig cycle starts in 10 seconds.'
printf '%s\n' 'If needed, select soil_loader in Entity tree and press F to focus it.'
sleep 10

set +e
python3 "${project_root}/tools/ros/test_loader_soil_coupling.py" \
  "${test_arguments[@]}" | tee "${scenario_log}"
scenario_status=${PIPESTATUS[0]}
set -e

if [[ ${scenario_status} -eq 0 ]]; then
  printf '%s\n' 'Demo cycle complete. Gazebo will remain open.'
else
  printf 'Demo cycle failed with status %d. Gazebo will remain open for inspection.\n' \
    "${scenario_status}" >&2
fi
printf '%s\n' 'Close the Gazebo window or press Ctrl+C here to stop the simulation.'

set +e
wait "${gui_pid}"
gui_status=$?
set -e
gui_pid=''

stop_process "${server_pid}"
server_pid=''

if [[ ${scenario_status} -ne 0 ]]; then
  exit "${scenario_status}"
fi
if [[ ${gui_status} -eq 130 ]]; then
  # Ctrl+C is the documented, normal way to close an interactive demo.
  exit 0
fi
if [[ ${gui_status} -ne 0 ]]; then
  printf 'Gazebo GUI exited with status %d. See %s\n' "${gui_status}" "${gui_log}" >&2
  exit "${gui_status}"
fi
