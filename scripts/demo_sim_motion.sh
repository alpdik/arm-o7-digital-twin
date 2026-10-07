#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

echo "Moving the SIMULATION arm to a conservative ready pose..."
ros2 action send_goal \
  /sim/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6], points: [{positions: [0.0, -0.45, 0.80, 0.0, -0.35, 0.0], time_from_start: {sec: 5}}]}}'

echo "Closing the SIMULATION hand..."
ros2 action send_goal \
  /sim/hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch], points: [{positions: [0.001, 0.001, 0.001, 1.15, 1.15, 1.15, 1.10], time_from_start: {sec: 5}}]}}'

echo "Holding the closed pose for two seconds..."
sleep 2

echo "Reopening the SIMULATION hand so MoveIt starts collision-free..."
ros2 action send_goal \
  /sim/hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch], points: [{positions: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001], time_from_start: {sec: 3}}]}}'

echo "Demo complete. This script never publishes to /real."
