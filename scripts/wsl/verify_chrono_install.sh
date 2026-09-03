#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
source_dir="${project_root}/tools/chrono_dem_smoke"
build_dir="${runtime_root}/build/loader-chrono-dem-smoke"

source /etc/profile.d/loader-sim-cuda.sh
source "${project_root}/config/wsl/loader-sim-chrono.sh"

cmake \
  -S "${source_dir}" \
  -B "${build_dir}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=75 \
  -DChrono_DIR="${Chrono_DIR}"

cmake --build "${build_dir}" --parallel 2
"${build_dir}/loader_chrono_dem_smoke"

