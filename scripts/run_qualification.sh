#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

cycles="${1:-1}"
results_dir="$project_dir/results/digital_twin_qualification"

echo "Running local digital-twin qualification (${cycles} cycle(s))..."
echo "Gazebo and Terminal 2 must already be running and unpaused."

ros2 run arm_o7_twin qualification \
  --cycles "$cycles" \
  --output-dir "$results_dir"
