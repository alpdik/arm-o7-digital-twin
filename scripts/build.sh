#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
cd "$project_dir"

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  --skip-keys ament_python \
  -r -y
colcon build --symlink-install --event-handlers console_direct+
set +u
source install/setup.bash
set -u

python3 tests/validate_project.py
colcon test --event-handlers console_direct+
colcon test-result --verbose
