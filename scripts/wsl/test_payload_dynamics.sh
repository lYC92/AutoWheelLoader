#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
source "${HOME}/loader_sim_runtime/install/setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/loader_sim_runtime/install/loader_soil/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
log="${HOME}/loader_sim_runtime/results/payload_probe.log"
gz sim -s -r --iterations 2000 simulation/worlds/payload_probe.sdf >"${log}" 2>&1
if ! grep '^PASS payload dynamics:' "${log}" || grep -Eq 'FAIL|\[Err\]|Segmentation fault|terminate called' "${log}"; then
  tail -n 40 "${log}"
  exit 1
fi
