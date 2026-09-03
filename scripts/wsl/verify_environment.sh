#!/usr/bin/env bash
set -uo pipefail

failures=0
warnings=0

if [[ -f /etc/profile.d/loader-sim-wslg.sh ]]; then
  source /etc/profile.d/loader-sim-wslg.sh
fi

pass() { printf 'PASS  %s\n' "$1"; }
warn() { printf 'WARN  %s\n' "$1"; warnings=$((warnings + 1)); }
fail() { printf 'FAIL  %s\n' "$1"; failures=$((failures + 1)); }

if [[ -r /etc/os-release ]]; then
  source /etc/os-release
  if [[ ${VERSION_CODENAME:-} == noble ]]; then
    pass "Ubuntu 24.04 (${PRETTY_NAME})"
  else
    fail "Ubuntu 24.04 required; found ${PRETTY_NAME:-unknown}"
  fi
else
  fail "cannot read /etc/os-release"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_line="$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null | head -n 1)"
  if [[ -n ${gpu_line} ]]; then pass "WSL GPU: ${gpu_line}"; else fail "nvidia-smi did not return a GPU"; fi
else
  fail "nvidia-smi is unavailable in WSL"
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS environment scripts may read optional variables that are unset.
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  pass "ROS 2 Jazzy setup found"
else
  fail "ROS 2 Jazzy setup missing"
fi

for package in ros_gz_sim controller_manager gz_ros2_control; do
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    pass "ROS package ${package}"
  else
    fail "ROS package ${package} missing"
  fi
done

for command_name in gz colcon cmake ninja gcc g++ python3 git glxinfo vulkaninfo; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    pass "command ${command_name}"
  else
    fail "command ${command_name} missing"
  fi
done

if command -v glxinfo >/dev/null 2>&1 && [[ -n ${DISPLAY:-} ]]; then
  renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^[[:space:]]*OpenGL renderer string:[[:space:]]*//p' | head -n 1)"
  version="$(glxinfo -B 2>/dev/null | sed -n 's/^[[:space:]]*OpenGL core profile version string:[[:space:]]*//p' | head -n 1)"
  if [[ -z ${renderer} ]]; then
    fail "OpenGL renderer could not be queried"
  elif grep -qi llvmpipe <<<"${renderer}"; then
    fail "software renderer detected: ${renderer}"
  else
    pass "OpenGL renderer: ${renderer}"
  fi
  if [[ -n ${version} ]]; then pass "OpenGL core: ${version}"; else warn "OpenGL core version unavailable"; fi
else
  warn "WSLg/OpenGL check skipped because DISPLAY or glxinfo is unavailable"
fi

if command -v vulkaninfo >/dev/null 2>&1; then
  vulkan_device="$(timeout 20s vulkaninfo --summary 2>/dev/null | sed -n 's/^[[:space:]]*deviceName[[:space:]]*=[[:space:]]*//p' | head -n 1)"
  if [[ -z ${vulkan_device} ]]; then
    warn "Vulkan device could not be queried"
  elif grep -qi llvmpipe <<<"${vulkan_device}"; then
    warn "Vulkan remains on software rendering: ${vulkan_device}; Gazebo OGRE2 uses the verified OpenGL path"
  else
    pass "Vulkan device: ${vulkan_device}"
  fi
fi

if command -v gz >/dev/null 2>&1; then
  gz_version="$(gz sim --versions 2>/dev/null | tr '\n' ' ' | sed 's/[[:space:]]\+/ /g')"
  if [[ -n ${gz_version} ]]; then pass "Gazebo Sim: ${gz_version}"; else warn "Gazebo version query returned no output"; fi
fi

printf '\nSummary: %d failure(s), %d warning(s)\n' "${failures}" "${warnings}"
exit "${failures}"
