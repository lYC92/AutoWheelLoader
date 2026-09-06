#!/usr/bin/env bash
set -Eeuo pipefail
runtime_root="${HOME}/loader_sim_runtime/localization"
source_dir="${runtime_root}/src/kiss-icp"
revision=1ffa7d7512f10bfc8b1185095011fa31184019e3
mkdir -p "${runtime_root}/src"
if [[ ! -d ${source_dir}/.git ]]; then
  git clone https://github.com/PRBonn/kiss-icp.git "${source_dir}"
fi
if [[ -n $(git -C "${source_dir}" status --porcelain) ]]; then
  printf 'ERROR: localization source contains changes: %s\n' "${source_dir}" >&2
  exit 1
fi
git -C "${source_dir}" cat-file -e "${revision}^{commit}" 2>/dev/null || \
  git -C "${source_dir}" fetch origin "${revision}"
git -C "${source_dir}" checkout --detach "${revision}"
export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/jazzy/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "${runtime_root}/log" build \
  --base-paths "${source_dir}/ros" --build-base "${runtime_root}/build" \
  --install-base "${runtime_root}/install" --executor sequential \
  --event-handlers console_cohesion+
printf 'PASS  KISS-ICP baseline built at %s\n' "${revision}"
