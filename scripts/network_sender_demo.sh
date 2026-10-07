#!/usr/bin/env bash
set -eo pipefail

# Sender-only ROS 2 demo. It needs ROS 2 Jazzy and control_msgs, but it does
# not need Gazebo, RViz, meshes, colcon build, or this workspace's install tree.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash was not found." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$script_dir/network_env.sh"
set -u

sim_namespace="${ARM_O7_SIM_NAMESPACE:-/sim}"
arm_action="$sim_namespace/arm_controller/follow_joint_trajectory"
hand_action="$sim_namespace/hand_controller/follow_joint_trajectory"
arm_joints="[arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6]"
hand_joints="[thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch]"
test_dir="$(mktemp -d)"

cleanup() {
  rm -rf -- "$test_dir"
}
trap cleanup EXIT

echo "[sender] host=$(hostname) addresses=$(hostname -I 2>/dev/null || true)"
echo "[sender] waiting for remote simulation actions..."

available_actions="$(timeout 15 ros2 action list 2>/dev/null || true)"
for required_action in "$arm_action" "$hand_action"; do
  if ! grep -Fxq "$required_action" <<<"$available_actions"; then
    echo "ERROR: remote action is unavailable: $required_action" >&2
    echo "Check ROS_DOMAIN_ID, discovery, firewall and the receiver launch." >&2
    exit 1
  fi
done

run_pair() {
  local label="$1"
  local arm_positions="$2"
  local hand_positions="$3"
  local duration_sec="$4"
  local arm_log="$test_dir/${label}_arm.log"
  local hand_log="$test_dir/${label}_hand.log"
  local start_ns end_ns arm_pid hand_pid arm_rc hand_rc

  start_ns="$(date +%s%N)"
  echo "[$label] sending arm and hand goals (${duration_sec}s trajectory)..."

  ros2 action send_goal \
    "$arm_action" \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: $arm_joints, points: [{positions: $arm_positions, time_from_start: {sec: $duration_sec}}]}}" \
    >"$arm_log" 2>&1 &
  arm_pid=$!

  ros2 action send_goal \
    "$hand_action" \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: $hand_joints, points: [{positions: $hand_positions, time_from_start: {sec: $duration_sec}}]}}" \
    >"$hand_log" 2>&1 &
  hand_pid=$!

  arm_rc=0
  wait "$arm_pid" || arm_rc=$?
  hand_rc=0
  wait "$hand_pid" || hand_rc=$?
  end_ns="$(date +%s%N)"

  if (( arm_rc != 0 )) || ! grep -q "Goal finished with status: SUCCEEDED" "$arm_log"; then
    echo "[$label] ARM=FAIL"
    sed -n '1,200p' "$arm_log"
    return 1
  fi
  if (( hand_rc != 0 )) || ! grep -q "Goal finished with status: SUCCEEDED" "$hand_log"; then
    echo "[$label] HAND=FAIL"
    sed -n '1,200p' "$hand_log"
    return 1
  fi

  echo "[$label] ARM=SUCCEEDED HAND=SUCCEEDED elapsed_ms=$(((end_ns - start_ns) / 1000000))"
}

test_arm="[0.30, -0.45, 0.80, -0.25, -0.35, 0.25]"
ready_arm="[0.0, -0.45, 0.80, 0.0, -0.35, 0.0]"
four_finger_fist="[0.001, 0.001, 0.001, 1.15, 1.15, 1.15, 1.10]"
open_hand="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]"

run_pair "outbound" "$test_arm" "$four_finger_fist" 5
sleep 1
run_pair "return" "$ready_arm" "$open_hand" 5

echo "NETWORK_SIM_DEMO=PASS"
echo "This sender contacted only $sim_namespace actions and never /real."
