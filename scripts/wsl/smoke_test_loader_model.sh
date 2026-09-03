#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
world_file="${project_root}/simulation/worlds/loader_kinematics.sdf"
xacro_file="${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro"
urdf_file="${runtime_root}/results/loader.nominal.urdf"
server_log="${runtime_root}/log/loader_model_smoke.log"
pose_log="${runtime_root}/results/loader_model_pose.txt"
joint_log="${runtime_root}/results/loader_model_joints.txt"

if [[ -f /etc/profile.d/loader-sim-wslg.sh ]]; then
  source /etc/profile.d/loader-sim-wslg.sh
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${runtime_root}/install/setup.bash"
set -u

mkdir -p "${runtime_root}/log" "${runtime_root}/results"
xacro "${xacro_file}" model_fidelity:=nominal >"${urdf_file}"

server_pid=''
cleanup() {
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

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
  -name nominal_loader \
  -z 0.20 2>&1)"
printf '%s\n' "${spawn_output}"
if ! grep -qi 'success' <<<"${spawn_output}"; then
  printf 'FAIL  Loader entity creation did not report success.\n' >&2
  exit 1
fi

sleep 6
model_list="$(gz model --list)"
if ! grep -q 'nominal_loader' <<<"${model_list}"; then
  printf 'FAIL  nominal_loader is absent from the running world.\n' >&2
  printf '%s\n' "${model_list}" >&2
  exit 1
fi

gz model -m nominal_loader -p >"${pose_log}"
gz model -m nominal_loader -j >"${joint_log}"

if grep -Eqi '(^|[^[:alpha:]])(nan|inf)([^[:alpha:]]|$)' "${pose_log}" "${joint_log}"; then
  printf 'FAIL  Non-finite model state detected.\n' >&2
  exit 1
fi

for joint in articulation_joint rear_axle_oscillation_joint lift_joint bucket_tilt_joint; do
  if ! grep -q "${joint}" "${joint_log}"; then
    printf 'FAIL  Gazebo model is missing joint: %s\n' "${joint}" >&2
    exit 1
  fi
done

printf 'PASS  Loader entity spawned, settled for 6 seconds, and retained finite state.\n'
printf 'Pose: %s\n' "${pose_log}"
printf 'Joints: %s\n' "${joint_log}"
