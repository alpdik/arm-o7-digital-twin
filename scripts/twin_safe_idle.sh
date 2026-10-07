#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

ros2 topic pub --once /twin/deadman std_msgs/msg/Bool '{data: false}'
ros2 service call /arm_o7_twin_arbiter/mode/safe_idle std_srvs/srv/Trigger '{}'
