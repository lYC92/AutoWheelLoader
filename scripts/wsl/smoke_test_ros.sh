#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  printf 'ROS 2 Jazzy is not installed.\n' >&2
  exit 1
fi

# ROS environment scripts may read unset shell variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u

runtime_root="${HOME}/loader_sim_runtime"
log_dir="${runtime_root}/log"
mkdir -p "${log_dir}"

talker_log="${log_dir}/ros_talker.log"
listener_log="${log_dir}/ros_listener.log"
rm -f "${talker_log}" "${listener_log}"

cleanup() {
  if [[ -n ${talker_pid:-} ]]; then
    kill "${talker_pid}" >/dev/null 2>&1 || true
    wait "${talker_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

timeout 15s ros2 run demo_nodes_cpp talker >"${talker_log}" 2>&1 &
talker_pid=$!

# Give DDS discovery time to establish the graph in a fresh WSL session.
sleep 2
set +e
timeout 10s ros2 run demo_nodes_py listener >"${listener_log}" 2>&1
listener_status=$?
set -e

if grep -q 'I heard' "${listener_log}"; then
  sample="$(grep -m1 'I heard' "${listener_log}")"
  printf 'PASS  ROS 2 DDS talker/listener: %s\n' "${sample}"
  exit 0
fi

printf 'FAIL  ROS 2 listener received no message (exit=%s).\n' "${listener_status}" >&2
printf '%s\n' '--- talker log ---' >&2
tail -n 20 "${talker_log}" >&2 || true
printf '%s\n' '--- listener log ---' >&2
tail -n 20 "${listener_log}" >&2 || true
exit 1
