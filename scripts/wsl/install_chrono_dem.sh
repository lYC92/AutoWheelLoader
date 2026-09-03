#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -eq 0 ]]; then
  printf '%s\n' 'ERROR: run this script as the normal WSL user, not root.' >&2
  exit 2
fi

runtime_root="${HOME}/loader_sim_runtime"
source_dir="${runtime_root}/src/chrono"
build_dir="${runtime_root}/build/chrono-dem"
install_dir="${runtime_root}/install/chrono"
build_jobs="${CHRONO_BUILD_JOBS:-2}"

source /etc/profile.d/loader-sim-cuda.sh

cmake \
  -S "${source_dir}" \
  -B "${build_dir}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_DEMOS=OFF \
  -DBUILD_TESTING=OFF \
  -DCH_ENABLE_MODULE_DEM=ON \
  -DCH_ENABLE_MODULE_VSG=OFF \
  -DCHRONO_GPU_BACKEND=CUDA \
  -DCHRONO_CUDA_ARCHITECTURES=75 \
  -DCMAKE_CUDA_ARCHITECTURES=75

cmake --build "${build_dir}" --target install --parallel "${build_jobs}"

config_file="$(find "${install_dir}" -type f -name ChronoConfig.cmake -print -quit)"
if [[ -z "${config_file}" ]]; then
  printf 'ERROR: ChronoConfig.cmake was not installed.\n' >&2
  exit 1
fi

printf 'PASS  Chrono core and DEM installed into %s\n' "${install_dir}"
printf '      CMake package: %s\n' "${config_file}"

