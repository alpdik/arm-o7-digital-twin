#!/usr/bin/env bash

# Read-only environment inventory for a new teammate. It installs nothing and
# changes no system setting. Share the generated report before troubleshooting.
set -o pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
report_path="${1:-$project_dir/preflight_report.txt}"

{
  echo "ARM1.5 + O7 TEAM PREFLIGHT"
  echo "generated_at=$(date --iso-8601=seconds 2>/dev/null || date)"
  echo

  echo "===== OPERATING SYSTEM ====="
  uname -a
  if [[ -r /etc/os-release ]]; then
    grep -E '^(NAME|VERSION|ID|VERSION_ID|PRETTY_NAME)=' /etc/os-release
  fi
  if [[ -e /dev/dxg ]] && grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    echo "environment=WSL2_WSLg"
    if [[ -x /mnt/c/Windows/System32/wsl.exe ]]; then
      /mnt/c/Windows/System32/wsl.exe --version 2>/dev/null || true
    fi
  else
    echo "environment=native_or_virtualized_linux"
  fi
  echo

  echo "===== CPU AND MEMORY ====="
  echo "logical_cpu_count=$(nproc 2>/dev/null || echo unknown)"
  if command -v lscpu >/dev/null 2>&1; then
    lscpu | grep -E '^(Model name|CPU\(s\)|Thread|Core|Socket|Architecture):' || true
  fi
  free -h 2>/dev/null || true
  echo

  echo "===== DISK ====="
  df -h "$project_dir" 2>/dev/null || true
  echo

  echo "===== ROS 2 / GAZEBO / BUILD TOOLS ====="
  if [[ -r /opt/ros/jazzy/setup.bash ]]; then
    # ROS setup files are not guaranteed to be safe with nounset enabled.
    set +u
    source /opt/ros/jazzy/setup.bash
    echo "ros_jazzy_setup=present"
  else
    echo "ros_jazzy_setup=missing"
  fi
  echo "ROS_DISTRO=${ROS_DISTRO:-not_sourced}"
  for command_name in ros2 gz colcon xacro; do
    if command -v "$command_name" >/dev/null 2>&1; then
      echo "$command_name=$(command -v "$command_name")"
    else
      echo "$command_name=missing"
    fi
  done
  if command -v gz >/dev/null 2>&1; then
    gz sim --versions 2>/dev/null || true
  fi
  echo

  echo "===== GRAPHICS ====="
  env | grep -E '^(DISPLAY|WAYLAND_DISPLAY|LIBGL|MESA|GALLIUM|DRI|ARM_O7_GPU)' \
    || echo "graphics_environment_variables=none"
  if command -v glxinfo >/dev/null 2>&1; then
    glxinfo -B 2>&1 || true
  else
    echo "glxinfo=missing (package: mesa-utils)"
  fi
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null || true
  elif [[ -x /mnt/c/Windows/System32/nvidia-smi.exe ]]; then
    /mnt/c/Windows/System32/nvidia-smi.exe \
      --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null || true
  else
    echo "nvidia_smi=not_available_or_not_nvidia"
  fi
  echo

  echo "===== PROJECT CONTENT ====="
  echo "project_dir=$project_dir"
  test -f "$project_dir/src/arm_o7_description/urdf/arm_o7.urdf.xacro" \
    && echo "combined_xacro=present" || echo "combined_xacro=missing"
  test -f "$project_dir/src/arm_o7_moveit_config/config/arm_o7.srdf" \
    && echo "moveit_srdf=present" || echo "moveit_srdf=missing"
  test -d "$project_dir/src/arm_o7_description/meshes/o7" \
    && echo "o7_meshes=present" || echo "o7_meshes=missing"
  echo
  echo "PREFLIGHT_COMPLETE"
} 2>&1 | tee "$report_path"

echo
echo "Report saved to: $report_path"
