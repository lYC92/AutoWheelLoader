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
grid_mode="${LOADER_SOIL_3D:-false}"
approach_yaw="${LOADER_APPROACH_YAW:-0}"
transfer_test="${LOADER_TRANSFER_TEST:-false}"
ab_test="${LOADER_AB_TEST:-false}"
guards_test="${LOADER_SOIL_GUARDS:-false}"
slope_test="${LOADER_SLOPED_BLADE:-false}"
[[ ${slope_test} == true || ${LOADER_NAVIGATION_STOP:-false} == true ]] && ab_test=true
soil_capacity=3.0
[[ ${guards_test} == true ]] && soil_capacity=0.2
grid_config="${project_root}/ros_ws/src/loader_description/config/soil_heightfield.yaml"
if [[ ${ab_test} == true || ${guards_test} == true ]]; then
  grid_mode=true
  transfer_test=true
  grid_config="${project_root}/ros_ws/src/loader_description/config/soil_heightfield_ab.yaml"
fi
suffix=""
world_file="${project_root}/simulation/worlds/loader_soil_slice.sdf"
if [[ ${mode} == perception ]]; then
  suffix="_perception"
  world_file="${project_root}/simulation/worlds/loader_soil_perception.sdf"
fi
if [[ ${grid_mode} == true ]]; then
  suffix="${suffix}_3d"
  world_file="${runtime_root}/results/loader_soil_3d${suffix}.sdf"
  mkdir -p "${runtime_root}/results"
  generator_args=(--config "${grid_config}" --task-config "${LOADER_AB_CONFIG:-${project_root}/simulation/config/ab_task.yaml}")
  [[ ${slope_test} == true ]] && generator_args+=(--wheel-ramp)
  [[ ${LOADER_CAPTURE_YARD:-false} == true ]] && generator_args+=(--overview-camera)
  [[ ${mode} == perception ]] && generator_args+=(--observer-lidar)
  python3 "${project_root}/tools/soil_heightfield_3d/generate_gazebo_world.py" "${world_file}" "${generator_args[@]}"
fi
urdf_file="${runtime_root}/results/loader.soil_coupling${suffix}.urdf"
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

if gz service -l 2>/dev/null | grep -q '^/world/loader_soil_slice/'; then
  printf 'ERROR: an existing loader_soil_slice world is running; close it before this test.\n' >&2
  exit 3
fi

mkdir -p "${runtime_root}/results" "${runtime_root}/log"
xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  dynamic_payload:="${LOADER_DYNAMIC_PAYLOAD:-true}" soil_bucket_capacity_m3:="${soil_capacity}" enable_ros2_control:=true enable_soil_slice:=true enable_soil_3d:="${grid_mode}" enable_ground_truth:="${transfer_test}" soil_3d_config:="${grid_config}" >"${urdf_file}"

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
[[ ${LOADER_CAPTURE_YARD:-false} == true ]] && bridge_arguments+=('/loader/yard_camera@sensor_msgs/msg/Image[gz.msgs.Image')
if [[ ${transfer_test} == true ]]; then
  bridge_arguments+=('/loader/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
fi
# Reproduce slow GUI/sensor initialization: gravity must not determine whether
# the driving test can start. Phases always use simulation time on slow hosts.
test_arguments=(--proxy-expectation "${proxy_expectation_log}" --use-sim-time-for-phases --startup-settle-s 8)
[[ ${grid_mode} == true ]] && test_arguments+=(--heightfield-3d)
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

read -r spawn_x spawn_y < <(python3 -c 'import math,sys; a=float(sys.argv[1]); print(7-7*math.cos(a), -7*math.sin(a))' "${approach_yaw}")
spawn_z=0.20
[[ ${slope_test} == true ]] && spawn_z=0.35
spawn_output="$(ros2 run ros_gz_sim create \
  -world loader_soil_slice \
  -file "${urdf_file}" \
  -name soil_loader \
  -x "${spawn_x}" -y "${spawn_y}" -Y "${approach_yaw}" -z "${spawn_z}" 2>&1)"
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

if [[ ${LOADER_NAVIGATION_STOP:-false} == true ]]; then
  python3 "${project_root}/tools/ros/test_navigation_stop.py" --output "${runtime_root}/results/navigation_stop.json"
  exit $?
fi

if [[ ${slope_test} == true ]]; then
  ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager >/dev/null
  python3 "${project_root}/tools/ros/test_sloped_vehicle.py" --output "${runtime_root}/results/sloped_vehicle.json"
  exit $?
fi

if [[ ${guards_test} == true ]]; then
  python3 "${project_root}/tools/ros/test_soil_guards.py" --capacity "${soil_capacity}" \
    --output "${runtime_root}/results/soil_guards.json" 2>&1 | tee "${runtime_root}/results/soil_guards.txt"
  exit "${PIPESTATUS[0]}"
fi

if [[ ${ab_test} == true ]]; then
  python3 "${project_root}/tools/ros/run_ab_cycle.py" --pose-source ground_truth \
    --config "${LOADER_AB_CONFIG:-${project_root}/simulation/config/ab_task.yaml}" \
    --output "${runtime_root}/results/ab_cycle.json" 2>&1 | tee "${runtime_root}/results/ab_cycle.txt"
  exit "${PIPESTATUS[0]}"
fi

if [[ ${transfer_test} == true ]]; then
  python3 "${project_root}/tools/ros/run_transfer_scenario.py" --pose-source ground_truth \
    --output "${runtime_root}/results/transfer_validation.json" 2>&1 | tee "${runtime_root}/results/transfer_validation.txt"
  exit "${PIPESTATUS[0]}"
fi

python3 "${project_root}/tools/ros/test_loader_soil_coupling.py" \
  "${test_arguments[@]}" 2>&1 | tee "${test_log}"
gz model -m soil_loader -p >"${pose_log}"
: >"${proxy_pose_log}"
read -r proxy_index _ <"${proxy_expectation_log}"
printf -v proxy_name 'soil_column_%03d' "${proxy_index}"
if [[ ${grid_mode} == true ]]; then
  # Visual-only XY cells are verified by the observer lidar in perception mode.
  # They are no longer independent models addressable with gz model.
  gz model -m soil_grid -p >"${proxy_pose_log}"
else
  gz model -m "${proxy_name}" -p >"${proxy_pose_log}"
fi
if [[ ${grid_mode} != true ]]; then
  python3 "${project_root}/tools/soil_slice/verify_soil_proxy_pose.py" \
    "${proxy_pose_log}" "${proxy_expectation_log}"
fi

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
