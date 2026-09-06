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
export ROS_DOMAIN_ID="${LOADER_TEST_ROS_DOMAIN_ID:-83}"
export GZ_PARTITION="loader_foxglove_check_${$}"
world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
urdf_file="${runtime_root}/results/loader.foxglove_smoke.urdf"
server_log="${runtime_root}/log/foxglove_smoke_gazebo.log"
rsp_log="${runtime_root}/log/foxglove_smoke_robot_state_publisher.log"
bridge_log="${runtime_root}/log/foxglove_smoke_bridge.log"
foxglove_log="${runtime_root}/log/foxglove_smoke_foxglove.log"
manual_log="${runtime_root}/log/foxglove_smoke_manual_gateway.log"
test_log="${runtime_root}/results/foxglove_bridge_${mode}_smoke.txt"
soil_plugin_dir="${runtime_root}/install/loader_soil/lib"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${soil_plugin_dir}:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${soil_plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${runtime_root}/log" "${runtime_root}/results"
enable_lidar_imu=false
if [[ ${mode} == perception ]]; then
  world_file="${project_root}/simulation/worlds/loader_soil_perception.sdf"
  enable_lidar_imu=true
fi
# Use an isolated test dependency environment; never modify system Python.
test_python="${runtime_root}/venv/observability/bin/python"
if [[ ! -x ${test_python} ]]; then
  python3 -m venv --system-site-packages "${runtime_root}/venv/observability"
fi
if ! "${test_python}" -c 'import websockets' >/dev/null 2>&1; then
  "${test_python}" -m pip install 'websockets==15.0.1'
fi
if (exec 9<>/dev/tcp/127.0.0.1/8765) 2>/dev/null; then
  printf 'FAIL  Port 8765 is already in use; close the existing demo before testing.\n' >&2
  exit 3
fi
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_soil_slice:=true \
  enable_lidar_imu:="${enable_lidar_imu}" >"${urdf_file}"

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
effects_pid=''
lidar_tf_pid=''
imu_tf_pid=''
cleanup() {
  for pid in "${effects_pid}" "${lidar_tf_pid}" "${imu_tf_pid}" "${manual_pid}" "${foxglove_pid}" "${bridge_pid}" "${rsp_pid}" "${server_pid}"; do
    if [[ -n ${pid} ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      wait "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

bridge_arguments=('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')
if [[ ${mode} == perception ]]; then
  bridge_arguments+=(
    '/loader_soil/observer/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    '/loader/sensors/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    '/loader/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
  )
fi
ros2 run ros_gz_bridge parameter_bridge "${bridge_arguments[@]}" >"${bridge_log}" 2>&1 &
bridge_pid=$!

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(<"${urdf_file}")" >"${rsp_log}" 2>&1 &
rsp_pid=$!
if [[ ${mode} == perception ]]; then
  ros2 run tf2_ros static_transform_publisher --frame-id lidar_link \
    --child-frame-id soil_loader/lidar_link/loader_gpu_lidar \
    >"${runtime_root}/log/foxglove_smoke_lidar_tf.log" 2>&1 &
  lidar_tf_pid=$!
  ros2 run tf2_ros static_transform_publisher --frame-id imu_link \
    --child-frame-id soil_loader/imu_link/loader_imu \
    >"${runtime_root}/log/foxglove_smoke_imu_tf.log" 2>&1 &
  imu_tf_pid=$!
  ros2 run loader_sensor_effects lidar_effects_node --ros-args \
    --params-file "${project_root}/ros_ws/src/loader_sensor_effects/config/nominal.yaml" \
    >"${runtime_root}/log/foxglove_smoke_effects.log" 2>&1 &
  effects_pid=$!
fi

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
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager --controller-manager-timeout 30 >/dev/null

python3 "${project_root}/tools/ros/loader_manual_gateway.py" \
  --ros-args -p use_sim_time:=true >"${manual_log}" 2>&1 &
manual_pid=$!
sleep 2

"${test_python}" "${project_root}/tools/ros/test_foxglove_bridge.py" --mode "${mode}" | tee "${test_log}"

printf 'Gazebo log: %s\n' "${server_log}"
printf 'Foxglove Bridge log: %s\n' "${foxglove_log}"
printf 'Manual gateway log: %s\n' "${manual_log}"
printf 'Test result: %s\n' "${test_log}"
