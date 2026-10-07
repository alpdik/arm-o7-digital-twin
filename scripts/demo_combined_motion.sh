#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

arm_action="/sim/arm_controller/follow_joint_trajectory"
hand_action="/sim/hand_controller/follow_joint_trajectory"
arm_joints="[arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5, arm_joint_6]"
hand_joints="[thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch]"
test_dir="$(mktemp -d)"

cleanup() {
  rm -rf -- "$test_dir"
}
trap cleanup EXIT

available_actions="$(ros2 action list)"
for required_action in "$arm_action" "$hand_action"; do
  if ! grep -Fxq "$required_action" <<<"$available_actions"; then
    echo "ERROR: required action is unavailable: $required_action" >&2
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
  local arm_pid hand_pid arm_rc hand_rc

  echo "[$label] Sending arm and hand goals concurrently (${duration_sec}s)..."

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

  echo "[$label] ARM=SUCCEEDED HAND=SUCCEEDED"
}

# Verified Gazebo gesture: the four fingers and their mimic followers close,
# while the thumb stays open.  thumb_cmc_roll is intentionally held at its
# open cushion until its isolated tracking fault is diagnosed.
four_finger_fist="[0.001, 0.001, 0.001, 1.15, 1.15, 1.15, 1.10]"
open_hand="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]"
test_arm="[0.30, -0.45, 0.80, -0.25, -0.35, 0.25]"
ready_arm="[0.0, -0.45, 0.80, 0.0, -0.35, 0.0]"

if ! run_pair "outbound" "$test_arm" "$four_finger_fist" 5; then
  echo "Outbound test failed; attempting a slow return to ready + open..." >&2
  if run_pair "recovery" "$ready_arm" "$open_hand" 5; then
    echo "RECOVERY=SUCCEEDED" >&2
  else
    echo "RECOVERY=FAILED; use MoveIt to return each group separately." >&2
  fi
  exit 1
fi
sleep 1
run_pair "return" "$ready_arm" "$open_hand" 5

echo "COMBINED_SIM_TEST=PASS"
echo "This script only uses /sim actions and never publishes to /real."
