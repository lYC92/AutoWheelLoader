#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run this script as root through 'wsl -u root'." >&2
  exit 2
fi

source /etc/os-release
if [[ ${ID} != "ubuntu" || ${VERSION_CODENAME} != "noble" ]]; then
  echo "ERROR: Ubuntu 24.04 (noble) is required; found ${PRETTY_NAME}." >&2
  exit 2
fi

target_user="${LOADER_SIM_USER:-}"
if [[ -z ${target_user} ]]; then
  target_user="$(getent passwd 1000 | cut -d: -f1)"
fi
if ! id "${target_user}" >/dev/null 2>&1; then
  echo "ERROR: target WSL user '${target_user}' does not exist." >&2
  exit 2
fi
target_home="$(getent passwd "${target_user}" | cut -d: -f6)"

export DEBIAN_FRONTEND=noninteractive
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

echo "[1/7] Installing Ubuntu prerequisites"
apt-get update
apt-get install -y \
  ca-certificates \
  curl \
  git \
  gnupg \
  locales \
  lsb-release \
  software-properties-common \
  wget

locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository -y universe

echo "[2/7] Installing the official ROS apt source package"
ros_apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' | head -n 1)"
if [[ -z ${ros_apt_source_version} ]]; then
  echo "ERROR: unable to resolve the latest ros-apt-source release." >&2
  exit 3
fi
ros_apt_source_deb="/tmp/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
curl -fL \
  -o "${ros_apt_source_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
dpkg -i "${ros_apt_source_deb}"

echo "[3/7] Installing ROS 2 Jazzy and Gazebo Harmonic"
apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-dev-tools

echo "[4/7] Installing native build and graphics diagnostics"
apt-get install -y \
  build-essential \
  ccache \
  cmake \
  libeigen3-dev \
  libopencv-dev \
  libpcl-dev \
  mesa-utils \
  ninja-build \
  python3-pip \
  python3-venv \
  vulkan-tools

echo "[5/7] Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
runuser -u "${target_user}" -- env HOME="${target_home}" LANG=en_US.UTF-8 rosdep update

echo "[6/7] Creating WSL-native build and result directories"
install -d -o "${target_user}" -g "${target_user}" \
  "${target_home}/loader_sim_runtime/build" \
  "${target_home}/loader_sim_runtime/install" \
  "${target_home}/loader_sim_runtime/log" \
  "${target_home}/loader_sim_runtime/cache" \
  "${target_home}/loader_sim_runtime/results"

install -D -m 0644 \
  "${project_root}/config/wsl/loader-sim-wslg.sh" \
  /etc/profile.d/loader-sim-wslg.sh

echo "[7/7] Verifying installed package roots"
test -f /opt/ros/jazzy/setup.bash
bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix ros_gz_sim && ros2 pkg prefix controller_manager && ros2 pkg prefix gz_ros2_control'

echo "ROS 2 Jazzy and Gazebo Harmonic deployment completed."
