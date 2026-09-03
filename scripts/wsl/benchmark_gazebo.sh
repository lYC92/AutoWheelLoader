#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
world_path="${script_dir}/../../simulation/smoke/loader_smoke.sdf"
runtime_root="${HOME}/loader_sim_runtime"
log_dir="${runtime_root}/log"
result_dir="${runtime_root}/results"

mode="${1:-headless}"
case "${mode}" in
  headless)
    sim_args=(-s -r --headless-rendering)
    ;;
  gui)
    sim_args=(-r)
    ;;
  *)
    printf 'Usage: %s [headless|gui]\n' "$0" >&2
    exit 2
    ;;
esac

server_log="${log_dir}/gazebo_${mode}_benchmark.log"
stats_log="${result_dir}/gazebo_${mode}_stats.txt"
gpu_log="${result_dir}/gazebo_${mode}_gpu.csv"
camera_frequency_log="${result_dir}/gazebo_${mode}_camera_frequency.txt"
summary_file="${result_dir}/phase1_gazebo_${mode}_baseline.csv"

if [[ -f /etc/profile.d/loader-sim-wslg.sh ]]; then
  source /etc/profile.d/loader-sim-wslg.sh
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p "${log_dir}" "${result_dir}"
rm -f "${server_log}" "${stats_log}" "${gpu_log}" "${camera_frequency_log}" "${summary_file}"

server_pid=''
gpu_pid=''
camera_frequency_pid=''
cleanup() {
  if [[ -n ${camera_frequency_pid} ]]; then
    kill "${camera_frequency_pid}" >/dev/null 2>&1 || true
    wait "${camera_frequency_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${gpu_pid} ]]; then
    kill "${gpu_pid}" >/dev/null 2>&1 || true
    wait "${gpu_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n ${server_pid} ]]; then
    kill "${server_pid}" >/dev/null 2>&1 || true
    wait "${server_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

printf 'timestamp,memory_used_mib,utilization_gpu_pct\n' >"${gpu_log}"
gz sim "${sim_args[@]}" "${world_path}" >"${server_log}" 2>&1 &
server_pid=$!

(
  while kill -0 "${server_pid}" >/dev/null 2>&1; do
    printf '%s,' "$(date --iso-8601=seconds)" >>"${gpu_log}"
    nvidia-smi \
      --query-gpu=memory.used,utilization.gpu \
      --format=csv,noheader,nounits >>"${gpu_log}" 2>/dev/null || true
    sleep 1
  done
) &
gpu_pid=$!

stats_topic='/world/loader_smoke/stats'
camera_topic='/loader_sim/smoke/camera'
for _ in $(seq 1 30); do
  topics="$(gz topic -l 2>/dev/null || true)"
  if grep -q "^${stats_topic}$" <<<"${topics}" && grep -q "^${camera_topic}$" <<<"${topics}"; then
    break
  fi
  if ! kill -0 "${server_pid}" >/dev/null 2>&1; then
    printf 'FAIL  Gazebo exited before benchmark topics were ready.\n' >&2
    tail -n 100 "${server_log}" >&2
    exit 1
  fi
  sleep 1
done

if ! gz topic -l | grep -q "^${stats_topic}$"; then
  printf 'FAIL  Gazebo stats topic was not published.\n' >&2
  exit 1
fi

printf 'Sampling Gazebo statistics for 12 seconds...\n'
timeout 12s gz topic -f -t "${camera_topic}" >"${camera_frequency_log}" 2>&1 &
camera_frequency_pid=$!
gz topic -e -t "${stats_topic}" -d 12 >"${stats_log}"
set +e
wait "${camera_frequency_pid}"
camera_frequency_status=$?
set -e
camera_frequency_pid=''
if [[ ${camera_frequency_status} -ne 0 && ${camera_frequency_status} -ne 124 ]]; then
  printf 'WARN  Camera frequency sampler exited with status %s.\n' "${camera_frequency_status}" >&2
fi

rtf_summary="$(awk '
  /real_time_factor:/ {
    value=$2 + 0;
    count++;
    sum+=value;
    if (count == 1 || value < min) min=value;
    if (count == 1 || value > max) max=value;
  }
  END {
    if (count > 0) printf "%d,%.6f,%.6f,%.6f", count, sum/count, min, max;
  }
' "${stats_log}")"

if [[ -z ${rtf_summary} ]]; then
  printf 'FAIL  No real_time_factor samples were found.\n' >&2
  tail -n 80 "${stats_log}" >&2
  exit 1
fi

IFS=',' read -r rtf_samples rtf_avg rtf_min rtf_max <<<"${rtf_summary}"
gpu_mem_peak="$(awk -F, 'NR > 1 {gsub(/ /, "", $2); if ($2 + 0 > max) max=$2 + 0} END {print max + 0}' "${gpu_log}")"
gpu_util_peak="$(awk -F, 'NR > 1 {gsub(/ /, "", $3); if ($3 + 0 > max) max=$3 + 0} END {print max + 0}' "${gpu_log}")"
camera_hz="$(awk '/average rate:/ {value=$3} END {print value}' "${camera_frequency_log}")"
if [[ -z ${camera_hz} ]]; then
  camera_hz='unavailable'
fi

{
  printf 'metric,value\n'
  printf 'mode,%s\n' "${mode}"
  printf 'physics_step_hz,500\n'
  printf 'rtf_samples,%s\n' "${rtf_samples}"
  printf 'rtf_average,%s\n' "${rtf_avg}"
  printf 'rtf_minimum,%s\n' "${rtf_min}"
  printf 'rtf_maximum,%s\n' "${rtf_max}"
  printf 'gpu_memory_peak_mib,%s\n' "${gpu_mem_peak}"
  printf 'gpu_utilization_peak_pct,%s\n' "${gpu_util_peak}"
  printf 'camera_frequency_hz,%s\n' "${camera_hz}"
} | tee "${summary_file}"

awk -v rtf="${rtf_avg}" 'BEGIN {exit !(rtf >= 0.9)}' || {
  printf 'FAIL  Average real-time factor %s is below the 0.9 gate.\n' "${rtf_avg}" >&2
  exit 1
}

printf 'PASS  Gazebo 500 Hz %s baseline meets the RTF gate.\n' "${mode}"
printf 'Result: %s\n' "${summary_file}"
