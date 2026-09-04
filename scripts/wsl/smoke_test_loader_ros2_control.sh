#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
world_file="${project_root}/simulation/worlds/loader_kinematics.sdf"
xacro_file="${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro"
urdf_file="${runtime_root}/results/loader.ros2_control.urdf"
server_log="${runtime_root}/log/loader_ros2_control_smoke_gazebo.log"
test_log="${runtime_root}/results/loader_ros2_control_smoke.txt"
controller_log="${runtime_root}/results/loader_ros2_control_controllers.txt"
rsp_log="${runtime_root}/log/loader_ros2_control_robot_state_publisher.log"
bridge_log="${runtime_root}/log/loader_ros2_control_clock_bridge.log"

source /etc/profile.d/loader-sim-wslg.sh
set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

mkdir -p "${runtime_root}/log" "${runtime_root}/results"
xacro "${xacro_file}" model_fidelity:=nominal enable_ros2_control:=true >"${urdf_file}"

server_pid=''
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

gz sim -s -r "${world_file}" >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/world/loader_kinematics/create$'; then
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
  -world loader_kinematics \
  -file "${urdf_file}" \
  -name ros2_control_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

for _ in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before controller_manager became available.\n' >&2
    tail -n 100 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

ros2 run controller_manager spawner loader_command_controller \
  --controller-manager /controller_manager \
  --controller-manager-timeout 30
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager \
  --controller-manager-timeout 30

ros2 control list_controllers --controller-manager /controller_manager | tee "${controller_log}"
if ! grep -Eq '^loader_command_controller[[:space:]].*[[:space:]]active$' "${controller_log}"; then
  printf 'FAIL  loader_command_controller is not active.\n' >&2
  exit 1
fi
if ! grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' "${controller_log}"; then
  printf 'FAIL  joint_state_broadcaster is not active.\n' >&2
  exit 1
fi

topic_info="$(ros2 topic info /loader/command --verbose)"
if ! grep -q 'loader_command_controller' <<<"${topic_info}"; then
  printf 'FAIL  /loader/command is not subscribed by loader_command_controller.\n' >&2
  printf '%s\n' "${topic_info}" >&2
  exit 1
fi

python3 "${project_root}/tools/ros/test_loader_dynamics.py" | tee "${test_log}"

if ! timeout 5s ros2 topic echo /joint_states --once >/dev/null 2>&1; then
  printf 'FAIL  joint_state_broadcaster did not publish /joint_states.\n' >&2
  exit 1
fi

if grep -Eqi 'Failed to load|Could not load|exception|Segmentation fault|terminate called' \
    "${server_log}"; then
  printf 'FAIL  Gazebo log contains a ros2_control pipeline error.\n' >&2
  tail -n 120 "${server_log}" >&2
  exit 1
fi

printf 'PASS  VehicleCommand traversed controller_manager and GazeboSystem at force level.\n'
printf 'Gazebo log: %s\n' "${server_log}"
printf 'Controllers: %s\n' "${controller_log}"
printf 'Test result: %s\n' "${test_log}"
