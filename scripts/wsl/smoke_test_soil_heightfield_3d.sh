#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
output_dir="${HOME}/loader_sim_runtime/results/soil_heightfield_3d"

python3 "${project_root}/tools/soil_heightfield_3d/run_heightfield_smoke.py" \
  --config "${project_root}/simulation/config/materials/dry_sand_3d_nominal.yaml" \
  --output "${output_dir}"
