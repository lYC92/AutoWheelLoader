#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -eq 0 ]]; then
  printf '%s\n' 'ERROR: run this script as the normal WSL user, not root.' >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
source_dir="${runtime_root}/src/chrono"
build_dir="${runtime_root}/build/chrono-dem"
install_dir="${runtime_root}/install/chrono"
chrono_commit='583b8e6f48600699f2084154a31742261d28a7c7'
build_jobs="${CHRONO_BUILD_JOBS:-2}"

source /etc/profile.d/loader-sim-cuda.sh

for package in cmake ninja-build libeigen3-dev git; do
  if ! dpkg-query -W "${package}" >/dev/null 2>&1; then
    printf 'ERROR: required Ubuntu package is missing: %s\n' "${package}" >&2
    exit 2
  fi
done

mkdir -p "${runtime_root}/src" "${runtime_root}/build" "${runtime_root}/install"

if [[ ! -d "${source_dir}/.git" ]]; then
  git clone --filter=blob:none --no-checkout \
    https://github.com/projectchrono/chrono.git "${source_dir}"
fi

cd "${source_dir}"

if [[ -n "$(git status --porcelain)" ]]; then
  printf 'ERROR: refusing to change a dirty Chrono source tree: %s\n' "${source_dir}" >&2
  exit 2
fi

if ! git cat-file -e "${chrono_commit}^{commit}" 2>/dev/null; then
  git fetch --depth 1 origin "${chrono_commit}"
fi
git checkout --detach "${chrono_commit}"

printf 'Pinned Project Chrono commit: %s\n' "$(git rev-parse HEAD)"

cmake \
  -S "${source_dir}" \
  -B "${build_dir}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_DEMOS=ON \
  -DBUILD_TESTING=OFF \
  -DCH_ENABLE_MODULE_DEM=ON \
  -DCH_ENABLE_MODULE_VSG=OFF \
  -DCHRONO_GPU_BACKEND=CUDA \
  -DCHRONO_CUDA_ARCHITECTURES=75 \
  -DCMAKE_CUDA_ARCHITECTURES=75

cmake \
  --build "${build_dir}" \
  --target demo_DEM_movingBoundary \
  --parallel "${build_jobs}"

printf 'PASS  Chrono DEM official smoke target built in %s\n' "${build_dir}"
printf '      Source commit: %s\n' "${chrono_commit}"

