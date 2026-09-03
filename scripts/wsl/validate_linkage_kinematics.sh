#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
output_dir="${runtime_root}/results/linkage"

mkdir -p "${output_dir}"

python3 "${project_root}/tools/kinematics/generate_linkage_table.py" \
  --config "${project_root}/ros_ws/src/loader_description/config/nominal_linkage.yaml" \
  --xacro "${project_root}/ros_ws/src/loader_description/urdf/loader.urdf.xacro" \
  --output "${output_dir}/nominal_linkage_table.csv" \
  --check

