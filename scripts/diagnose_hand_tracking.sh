#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

hand_action="/sim/hand_controller/follow_joint_trajectory"
hand_joints="[thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch]"
open_hand="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]"
diagnostic_dir="$(mktemp -d)"
failures=0

cleanup() {
  rm -rf -- "$diagnostic_dir"
}
trap cleanup EXIT

send_hand_goal() {
  local label="$1"
  local positions="$2"
  local duration_sec="$3"
  local log_file="$diagnostic_dir/${label}.log"

  if ros2 action send_goal \
      "$hand_action" \
      control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: $hand_joints, points: [{positions: $positions, time_from_start: {sec: $duration_sec}}]}}" \
      >"$log_file" 2>&1 \
      && grep -q "Goal finished with status: SUCCEEDED" "$log_file"; then
    echo "[$label] SUCCEEDED"
    return 0
  fi

  echo "[$label] FAILED"
  grep -E "error_code:|error_string:|Goal finished" "$log_file" || true
  return 1
}

test_joint() {
  local joint_name="$1"
  local positions="$2"

  if ! send_hand_goal "$joint_name" "$positions" 5; then
    failures=$((failures + 1))
  fi

  if ! send_hand_goal "${joint_name}_return" "$open_hand" 5; then
    echo "RECOVERY=FAILED after $joint_name" >&2
    exit 2
  fi
}

if ! send_hand_goal "initial_open" "$open_hand" 5; then
  echo "INITIAL_RECOVERY=FAILED" >&2
  exit 2
fi

test_joint "thumb_cmc_roll" \
  "[0.279, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]"
test_joint "thumb_cmc_yaw" \
  "[0.001, 0.434, 0.001, 0.001, 0.001, 0.001, 0.001]"
test_joint "thumb_cmc_pitch" \
  "[0.001, 0.001, 0.279, 0.001, 0.001, 0.001, 0.001]"
test_joint "index_mcp_pitch" \
  "[0.001, 0.001, 0.001, 0.713, 0.001, 0.001, 0.001]"
test_joint "middle_mcp_pitch" \
  "[0.001, 0.001, 0.001, 0.001, 0.713, 0.001, 0.001]"
test_joint "ring_mcp_pitch" \
  "[0.001, 0.001, 0.001, 0.001, 0.001, 0.713, 0.001]"
test_joint "pinky_mcp_pitch" \
  "[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.682]"

if (( failures == 0 )); then
  echo "HAND_JOINT_DIAGNOSTIC=PASS"
  exit 0
fi

echo "HAND_JOINT_DIAGNOSTIC=FAIL failures=$failures"
exit 1
