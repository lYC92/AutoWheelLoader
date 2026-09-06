#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
mode="${1:-physics}"
control_mode="${2:-auto}"
localization="${3:-none}"
scenario="${4:-soil}"
if [[ ${scenario} != soil && ${scenario} != localization ]]; then
  printf 'ERROR: scenario must be soil or localization.\n' >&2
  exit 2
fi
if [[ ${scenario} == localization && ( ${localization} != kiss_icp || ${control_mode} != auto ) ]]; then
  printf 'ERROR: localization scenario requires auto / kiss_icp.\n' >&2
  exit 2
fi
headless="${LOADER_HEADLESS:-0}"
if [[ ${localization} != none && ${localization} != kiss_icp ]]; then
  printf 'ERROR: localization must be none or kiss_icp.\n' >&2
  exit 2
fi
if [[ ${localization} != none && ${mode} != perception ]]; then
  printf 'ERROR: localization requires perception mode.\n' >&2
  exit 2
fi
if [[ ${headless} == 1 && ${control_mode} != auto ]]; then
  printf 'ERROR: headless validation requires auto mode.\n' >&2
  exit 2
fi

if [[ ${mode} != physics && ${mode} != perception ]]; then
  printf 'Usage: %s [physics|perception] [auto|manual]\n' "$0" >&2
  exit 2
fi
if [[ ${control_mode} != auto && ${control_mode} != manual ]]; then
  printf 'Usage: %s [physics|perception] [auto|manual]\n' "$0" >&2
  exit 2
fi

world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
suffix=""
if [[ ${mode} == perception ]]; then
  world_file="${project_root}/simulation/worlds/loader_soil_perception.sdf"
  suffix="_perception"
fi

gui_config="${project_root}/simulation/config/gui/loader_demo.config"
run_dir="${runtime_root}/results/runs/${mode}_${scenario}_$(date +%Y%m%d_%H%M%S)_${$}"
urdf_file="${run_dir}/loader.urdf"
server_log="${runtime_root}/log/loader_soil_demo${suffix}_gazebo.log"
gui_log="${runtime_root}/log/loader_soil_demo${suffix}_gui.log"
rsp_log="${runtime_root}/log/loader_soil_demo${suffix}_robot_state_publisher.log"
bridge_log="${runtime_root}/log/loader_soil_demo${suffix}_bridge.log"
scenario_log="${runtime_root}/results/loader_soil_demo${suffix}.txt"
foxglove_log="${runtime_root}/log/loader_soil_demo${suffix}_foxglove.log"
manual_log="${runtime_root}/log/loader_soil_demo${suffix}_manual_gateway.log"
sensor_tf_log="${runtime_root}/log/loader_soil_demo${suffix}_sensor_tf.log"
effects_log="${runtime_root}/log/loader_soil_demo${suffix}_sensor_effects.log"
imu_tf_log="${runtime_root}/log/loader_soil_demo${suffix}_imu_tf.log"
soil_plugin_dir="${runtime_root}/install/loader_soil/lib"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
if [[ ${localization} == kiss_icp ]]; then
  if [[ ! -f ${runtime_root}/localization/install/setup.bash ]]; then
    printf 'ERROR: run scripts/wsl/bootstrap_localization.sh first.\n' >&2
    exit 2
  fi
  source "${runtime_root}/localization/install/setup.bash"
fi
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="${soil_plugin_dir}:/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${soil_plugin_dir}:${LD_LIBRARY_PATH:-}"

mkdir -p "${runtime_root}/results" "${runtime_root}/log" "${run_dir}"
if [[ ${scenario} == localization ]]; then
  world_file="${run_dir}/loader_localization.world.sdf"
  python3 "${project_root}/tools/ros/generate_localization_world.py" "${world_file}"
fi
enable_lidar_imu=false
if [[ ${mode} == perception ]]; then
  enable_lidar_imu=true
fi
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  enable_ros2_control:=true enable_soil_slice:=true \
  enable_lidar_imu:="${enable_lidar_imu}" \
  enable_ground_truth:="${enable_lidar_imu}" >"${urdf_file}"

if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/'; then
  printf '%s\n' 'ERROR: a loader_soil_slice Gazebo server is already running.' >&2
  printf '%s\n' 'Close the existing Gazebo window, then run this launcher again.' >&2
  exit 3
fi
if (exec 9<>/dev/tcp/127.0.0.1/18765) 2>/dev/null; then
  printf 'ERROR: Foxglove port 18765 is already in use. Close the existing demo first.\n' >&2
  exit 3
fi

server_pid=''
gui_pid=''
rsp_pid=''
bridge_pid=''
foxglove_pid=''
manual_pid=''
sensor_tf_pid=''
effects_pid=''
imu_tf_pid=''
localization_pid=''
localization_crop_pid=''
evaluation_pid=''

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
  stop_process "${evaluation_pid}"
  stop_process "${localization_pid}"
  stop_process "${localization_crop_pid}"
  stop_process "${effects_pid}"
  stop_process "${imu_tf_pid}"
  stop_process "${manual_pid}"
  stop_process "${sensor_tf_pid}"
  stop_process "${foxglove_pid}"
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
    '/loader/sensors/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    '/loader/sensors/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
    '/loader/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
  )
  test_arguments+=(--observer-topic /loader_soil/observer/scan/points)
fi

ros2 run ros_gz_bridge parameter_bridge "${bridge_arguments[@]}" >"${bridge_log}" 2>&1 &
bridge_pid=$!
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true -p robot_description:="$(<"${urdf_file}")" \
  >"${rsp_log}" 2>&1 &
rsp_pid=$!

if [[ ${mode} == perception ]]; then
  # Gazebo scopes sensor frame IDs below the sensor name, while the URDF TF tree
  # ends at lidar_link.  This identity alias lets Foxglove render the vehicle
  # point cloud and robot model in the same base_link frame.
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id lidar_link \
    --child-frame-id soil_loader/lidar_link/loader_gpu_lidar \
    --ros-args -p use_sim_time:=true >"${sensor_tf_log}" 2>&1 &
  sensor_tf_pid=$!
  ros2 run tf2_ros static_transform_publisher \
    --frame-id imu_link --child-frame-id soil_loader/imu_link/loader_imu \
    --ros-args -p use_sim_time:=true >"${imu_tf_log}" 2>&1 &
  imu_tf_pid=$!
  ros2 run loader_sensor_effects lidar_effects_node --ros-args \
    --params-file "${project_root}/ros_ws/src/loader_sensor_effects/config/nominal.yaml" \
    >"${effects_log}" 2>&1 &
  effects_pid=$!
  if [[ ${localization} == kiss_icp ]]; then
    python3 "${project_root}/tools/ros/filter_localization_cloud.py" --model-urdf "${urdf_file}" --ros-args \
      --params-file "${project_root}/simulation/config/localization/kiss_icp.yaml" \
      >"${runtime_root}/log/loader_localization_crop.log" 2>&1 &
    localization_crop_pid=$!
    ros2 run kiss_icp kiss_icp_node --ros-args \
      --params-file "${project_root}/simulation/config/localization/kiss_icp.yaml" \
      -r pointcloud_topic:=/loader/localization/points \
      -r kiss/odometry:=/loader/localization/odometry \
      >"${runtime_root}/log/loader_localization.log" 2>&1 &
    localization_pid=$!
  fi
fi

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=18765 use_sim_time:=true \
  >"${foxglove_log}" 2>&1 &
foxglove_pid=$!

foxglove_ready=false
for _ in $(seq 1 40); do
  if (exec 9<>/dev/tcp/127.0.0.1/18765) 2>/dev/null; then
    exec 9>&-
    exec 9<&-
    foxglove_ready=true
    break
  fi
  if ! kill -0 "${foxglove_pid}" >/dev/null 2>&1; then
    printf '%s\n' 'ERROR: Foxglove Bridge exited during startup.' >&2
    tail -n 120 "${foxglove_log}" >&2
    exit 1
  fi
  sleep 0.25
done
if [[ ${foxglove_ready} != true ]]; then
  printf '%s\n' 'ERROR: timed out waiting for Foxglove Bridge on port 18765.' >&2
  tail -n 120 "${foxglove_log}" >&2
  exit 1
fi

foxglove_url='https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A18765'
if [[ ${headless} != 1 && ${LOADER_OPEN_FOXGLOVE:-1} != 0 ]] && command -v powershell.exe >/dev/null 2>&1; then
  powershell.exe -NoProfile -NonInteractive -Command \
    "Start-Process '${foxglove_url}'" >/dev/null 2>&1 || true
fi
printf '%s\n' 'Foxglove telemetry: ws://localhost:18765'
printf 'Import this layout once: %s\n' \
  "${project_root}/foxglove/loader_simulation_layout.json"

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

if [[ ${headless} != 1 ]]; then
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
fi

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
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager --controller-manager-timeout 30 >/dev/null

if [[ ${control_mode} == manual ]]; then
  python3 "${project_root}/tools/ros/loader_manual_gateway.py" \
    --ros-args -p use_sim_time:=true >"${manual_log}" 2>&1 &
  manual_pid=$!
fi

if [[ ${headless} != 1 ]]; then
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

printf '%s\n' 'If needed, select soil_loader in Entity tree and press F to focus it.'
fi

if [[ ${scenario} == localization ]]; then
  python3 "${project_root}/tools/ros/evaluate_localization.py" \
    --output "${runtime_root}/results/localization" \
    --configuration "${project_root}/simulation/config/localization/kiss_icp.yaml" \
    --model-urdf "${urdf_file}" \
    >"${runtime_root}/log/localization_evaluation.log" 2>&1 &
  evaluation_pid=$!
  sleep 3
fi

scenario_status=0
if [[ ${control_mode} == auto ]]; then
  printf 'Loader is ready. Starting %s scenario.\n' "${scenario}"
  set +e
  if [[ ${scenario} == localization ]]; then
    python3 "${project_root}/tools/ros/run_localization_scenario.py" 2>&1 | tee "${scenario_log}"
  else
    python3 "${project_root}/tools/ros/test_loader_soil_coupling.py" \
      "${test_arguments[@]}" 2>&1 | tee "${scenario_log}"
  fi
  scenario_status=${PIPESTATUS[0]}
  set -e
else
  printf '%s\n' 'Loader is ready for manual control from the Foxglove Teleop panels.'
  printf '%s\n' 'Raise and curl the bucket before driving so the cutting edge clears the ground.'
  printf '%s\n' 'The gateway brakes automatically when a Teleop button is released.'
fi

if [[ -n ${evaluation_pid} ]]; then
  kill -INT "${evaluation_pid}" >/dev/null 2>&1 || true
  set +e
  wait "${evaluation_pid}"
  evaluation_status=$?
  set -e
  evaluation_pid=''
  cat "${runtime_root}/log/localization_evaluation.log"
  if [[ ${evaluation_status} -ne 0 ]]; then
    scenario_status=${evaluation_status}
  fi
fi
if [[ ${headless} == 1 ]]; then
  exit "${scenario_status}"
fi

if [[ ${control_mode} == auto && ${scenario_status} -eq 0 ]]; then
  printf '%s\n' 'Demo cycle complete. Gazebo will remain open.'
elif [[ ${scenario_status} -ne 0 ]]; then
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
