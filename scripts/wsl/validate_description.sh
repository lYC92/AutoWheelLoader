#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
xacro_file="${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro"
generated_urdf="${runtime_root}/results/loader.nominal.urdf"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${runtime_root}/install/setup.bash" ]]; then
  source "${runtime_root}/install/setup.bash"
fi
set -u

mkdir -p "$(dirname "${generated_urdf}")"
xacro "${xacro_file}" model_fidelity:=nominal >"${generated_urdf}"
check_urdf "${generated_urdf}"

for joint in \
  articulation_joint \
  rear_axle_oscillation_joint \
  front_left_wheel_joint \
  front_right_wheel_joint \
  rear_left_wheel_joint \
  rear_right_wheel_joint \
  lift_joint \
  bucket_tilt_joint; do
  if ! grep -q "joint name=\"${joint}\"" "${generated_urdf}"; then
    printf 'FAIL  Required joint missing: %s\n' "${joint}" >&2
    exit 1
  fi
done

printf 'PASS  Nominal loader URDF topology validated.\n'
printf 'Generated URDF: %s\n' "${generated_urdf}"
