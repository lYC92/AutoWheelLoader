#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID} -ne 0 ]]; then
  printf "ERROR: run this script as root through 'wsl -u root'.\n" >&2
  exit 2
fi

source /etc/os-release
if [[ ${ID} != ubuntu || ${VERSION_CODENAME} != noble ]]; then
  printf 'ERROR: Ubuntu 24.04 (noble) is required.\n' >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
keyring_deb='/tmp/cuda-keyring_1.1-1_all.deb'

export DEBIAN_FRONTEND=noninteractive

printf '%s\n' '[1/4] Installing the official NVIDIA CUDA repository keyring'
curl -fL \
  -o "${keyring_deb}" \
  https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i "${keyring_deb}"

printf '%s\n' '[2/4] Installing CUDA Toolkit 13.2 without a Linux display driver'
apt-get update
apt-get install -y cuda-toolkit-13-2

printf '%s\n' '[3/4] Installing the project CUDA environment'
install -D -m 0644 \
  "${project_root}/config/wsl/loader-sim-cuda.sh" \
  /etc/profile.d/loader-sim-cuda.sh

printf '%s\n' '[4/4] Verifying nvcc and the WSL-provided driver'
source /etc/profile.d/loader-sim-cuda.sh
nvcc --version
nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv,noheader

printf '%s\n' 'CUDA Toolkit 13.2 deployment completed.'
