#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
build_dir="${runtime_root}/build/chrono-dem"
binary="${build_dir}/bin/demo_DEM_movingBoundary"
scenario="${project_root}/simulation/config/chrono/dem_smoke.json"

source /etc/profile.d/loader-sim-cuda.sh

if [[ ! -x "${binary}" ]]; then
  printf 'ERROR: Chrono DEM smoke binary is missing: %s\n' "${binary}" >&2
  exit 2
fi

run_dir="$(mktemp -d "${runtime_root}/results/chrono_dem_smoke.XXXXXX")"
log_file="${run_dir}/run.log"

cd "${run_dir}"
timeout 120s "${binary}" "${scenario}" 2>&1 | tee "${log_file}"

particle_file="${run_dir}/DEMO_OUTPUT/DEM/loader_sim_smoke/step000000.csv"
if [[ ! -s "${particle_file}" ]]; then
  printf 'ERROR: DEM did not create a non-empty particle output file.\n' >&2
  exit 1
fi

if ! grep -q 'rendering frame' "${log_file}"; then
  printf 'ERROR: DEM did not advance through a simulation frame.\n' >&2
  exit 1
fi

particle_rows="$(wc -l < "${particle_file}")"
printf 'PASS  Chrono DEM CUDA simulation completed.\n'
printf '      Particle output rows: %s\n' "${particle_rows}"
printf '      Evidence directory: %s\n' "${run_dir}"

