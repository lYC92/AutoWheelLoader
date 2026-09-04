#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"
runtime_root="${HOME}/loader_sim_runtime"
result_dir="${runtime_root}/results/soil_slice"
server_log="${runtime_root}/log/soil_slice_collision_masks.log"
pose_log="${result_dir}/collision_mask_bucket_pose.txt"
summary_log="${result_dir}/soil_slice_smoke.txt"

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p "${result_dir}" "${runtime_root}/log"
python3 "${project_root}/tools/soil_slice/run_soil_slice_smoke.py" \
  --config "${project_root}/simulation/config/materials/dry_sand_nominal.yaml" \
  --output "${result_dir}/interaction_trace.csv" | tee "${summary_log}"

server_pid=''
cleanup() {
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

gz sim -s -r "${project_root}/simulation/worlds/soil_slice_collision_masks.sdf" \
  >"${server_log}" 2>&1 &
server_pid=$!

for _ in $(seq 1 30); do
  if gz model --list 2>/dev/null | grep -q 'soil_slice_test_bucket'; then
    break
  fi
  sleep 1
done
sleep 4
gz model -m soil_slice_test_bucket -p >"${pose_log}"

bucket_z="$(awk '
  /Pose \[ XYZ/ {
    getline;
    gsub(/\[/, "", $1);
    gsub(/\]/, "", $3);
    print $3;
    exit;
  }
' "${pose_log}")"
if [[ -z ${bucket_z} ]]; then
  printf 'FAIL  Could not parse collision-mask bucket pose.\n' >&2
  cat "${pose_log}" >&2
  exit 1
fi

awk -v z="${bucket_z}" 'BEGIN {exit !(z >= 0.20 && z <= 0.40)}' || {
  printf 'FAIL  Bucket stopped at z=%s; loose proxy may be producing native contact.\n' \
    "${bucket_z}" >&2
  exit 1
}

printf 'PASS  Collision masks: bucket crossed loose-material proxy and rested on rigid ground at z=%s m.\n' \
  "${bucket_z}"
printf 'Pose: %s\n' "${pose_log}"
