#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
world_path="${script_dir}/../../simulation/smoke/loader_smoke.sdf"
log_path="${HOME}/loader_sim_runtime/log/gazebo_smoke.log"

if [[ -f /etc/profile.d/loader-sim-wslg.sh ]]; then
  source /etc/profile.d/loader-sim-wslg.sh
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$(dirname "${log_path}")"

server_pid=''
cleanup() {
  if [[ -n ${server_pid} ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "Starting Gazebo Harmonic headless rendering smoke test"
timeout 30s gz sim -s -r --headless-rendering "${world_path}" >"${log_path}" 2>&1 &
server_pid=$!

for _ in $(seq 1 20); do
  if gz topic -l 2>/dev/null | grep -q '^/loader_sim/smoke/camera$'; then
    break
  fi
  if ! kill -0 "${server_pid}" 2>/dev/null; then
    echo "ERROR: Gazebo exited before the camera topic became available." >&2
    tail -n 80 "${log_path}" >&2
    exit 1
  fi
  sleep 1
done

if ! gz topic -l | grep -q '^/loader_sim/smoke/camera$'; then
  echo "ERROR: camera topic was not published." >&2
  tail -n 80 "${log_path}" >&2
  exit 1
fi

topic_info="$(gz topic -i -t /loader_sim/smoke/camera 2>&1)"
if ! grep -q 'gz.msgs.Image' <<<"${topic_info}"; then
  echo "ERROR: camera topic does not expose gz.msgs.Image." >&2
  printf '%s\n' "${topic_info}" >&2
  exit 1
fi

echo "PASS: Gazebo server stayed alive and published the GPU camera topic."
echo "Log: ${log_path}"
