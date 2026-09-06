#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${LOADER_TEST_ROS_DOMAIN_ID:-84}"
export GZ_PARTITION="loader_localization_check_${$}"
export LOADER_HEADLESS=1
export LOADER_OPEN_FOXGLOVE=0
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
nice -n 10 bash "${script_dir}/run_loader_soil_demo.sh" perception auto kiss_icp localization
