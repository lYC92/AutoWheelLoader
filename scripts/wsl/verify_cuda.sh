#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
source_file="${project_root}/tools/cuda/verify_cuda.cu"
binary_file="${runtime_root}/results/verify_cuda"

source /etc/profile.d/loader-sim-cuda.sh
mkdir -p "${runtime_root}/results"

nvcc \
  -std=c++17 \
  -O2 \
  -arch=sm_75 \
  "${source_file}" \
  -o "${binary_file}"

"${binary_file}"
