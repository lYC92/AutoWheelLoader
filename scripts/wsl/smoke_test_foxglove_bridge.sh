#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
urdf_file="${runtime_root}/results/loader.foxglove_smoke.urdf"
server_log="${runtime_root}/log/foxglove_smoke_gazebo.log"
rsp_log="${runtime_root}/log/foxglove_smoke_robot_state_publisher.log"
bridge_log="${runtime_root}/log/foxglove_smoke_bridge.log"
foxglove_log="${runtime_root}/log/foxglove_smoke_foxglove.log"
manual_log="${runtime_root}/log/foxglove_smoke_manual_gateway.log"
test_log="${runtime_root}/results/foxglove_bridge_smoke.txt"
soil_plugin_dir="${runtime_root}/install/loader_soil/lib"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${soil_plugin_dir}:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${soil_plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${runtime_root}/log" "${runtime_root}/results"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_soil_slice:=true \
  enable_lidar_imu:=false >"${urdf_file}"

if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/'; then
  printf 'FAIL  a loader_soil_slice Gazebo server is already running.\n' >&2
  printf 'Close the existing Gazebo window, then run this test again.\n' >&2
  exit 3
fi

server_pid=''
rsp_pid=''
bridge_pid=''
foxglove_pid=''
manual_pid=''
cleanup() {
  for pid in "${manual_pid}" "${foxglove_pid}" "${bridge_pid}" "${rsp_pid}" "${server_pid}"; do
    if [[ -n ${pid} ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      wait "${pid}" >/dev/null 2>&1 || true
    fi
  done
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

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8765 use_sim_time:=true >"${foxglove_log}" 2>&1 &
foxglove_pid=$!

foxglove_ready=false
for _ in $(seq 1 40); do
  if (exec 9<>/dev/tcp/127.0.0.1/8765) 2>/dev/null; then
    exec 9>&-
    exec 9<&-
    foxglove_ready=true
    break
  fi
  if ! kill -0 "${foxglove_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Foxglove Bridge exited during startup.\n' >&2
    tail -n 120 "${foxglove_log}" >&2
    exit 1
  fi
  sleep 0.25
done
if [[ ${foxglove_ready} != true ]]; then
  printf 'FAIL  timed out waiting for Foxglove Bridge on port 8765.\n' >&2
  tail -n 120 "${foxglove_log}" >&2
  exit 1
fi

gz sim -s -r "${world_file}" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/create$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before entity creation became available.\n' >&2
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
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

for _ in $(seq 1 40); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited while loading ros2_control.\n' >&2
    tail -n 120 "${server_log}" >&2
    exit 1
  fi
  sleep 0.5
done

ros2 run controller_manager spawner loader_command_controller \
  --controller-manager /controller_manager --controller-manager-timeout 30 >/dev/null

python3 "${project_root}/tools/ros/loader_manual_gateway.py" \
  --ros-args -p use_sim_time:=true >"${manual_log}" 2>&1 &
manual_pid=$!
sleep 2

python3 "${project_root}/tools/ros/test_foxglove_bridge.py" | tee "${test_log}"

printf 'Gazebo log: %s\n' "${server_log}"
printf 'Foxglove Bridge log: %s\n' "${foxglove_log}"
printf 'Manual gateway log: %s\n' "${manual_log}"
printf 'Test result: %s\n' "${test_log}"
