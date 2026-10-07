#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "This installer expects Ubuntu 24.04." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "Warning: validated target is Ubuntu 24.04; detected ${PRETTY_NAME:-unknown}." >&2
fi

sudo apt update
sudo apt install -y \
  mesa-utils \
  python3-colcon-common-extensions \
  python3-yaml \
  python3-rosdep \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  unzip

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "Dependencies installed. Next: bash scripts/build.sh"
