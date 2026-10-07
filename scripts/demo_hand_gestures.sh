#!/usr/bin/env bash
set -eo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$project_dir/install/setup.bash"
set -u

hand_action="/sim/hand_controller/follow_joint_trajectory"
hand_joints="[thumb_cmc_roll, thumb_cmc_yaw, thumb_cmc_pitch, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch]"
duration_sec="${ARM_O7_GESTURE_DURATION_SEC:-4}"
hold_sec="${ARM_O7_GESTURE_HOLD_SEC:-1}"
demo_dir="$(mktemp -d)"

cleanup() {
  rm -rf -- "$demo_dir"
}
trap cleanup EXIT

if [[ ! "$duration_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: ARM_O7_GESTURE_DURATION_SEC must be a positive integer." >&2
  exit 2
fi

if ! ros2 action list | grep -Fxq "$hand_action"; then
  echo "ERROR: hand action is unavailable: $hand_action" >&2
  echo "Start Terminal 1 and Terminal 2, then retry." >&2
  exit 1
fi

run_hand_goal() {
  local label="$1"
  local positions="$2"
  local log_path="$demo_dir/${label}.log"

  echo "[$label] Sending ${duration_sec}s simulation-only hand goal..."
  if ! ros2 action send_goal \
    "$hand_action" \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: $hand_joints, points: [{positions: $positions, time_from_start: {sec: $duration_sec}}]}}" \
    2>&1 | tee "$log_path"; then
    echo "[$label] FAIL: ros2 action command returned an error." >&2
    return 1
  fi

  if ! grep -q "Goal finished with status: SUCCEEDED" "$log_path"; then
    echo "[$label] FAIL: trajectory did not finish with SUCCEEDED." >&2
    return 1
  fi

  echo "[$label] PASS"
}

open_hand="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]"
middle_finger="[0.001, 0.001, 0.001, 1.15, 0.001, 1.15, 1.10]"
four_finger_fist="[0.001, 0.001, 0.001, 1.15, 1.15, 1.15, 1.10]"

run_hand_goal "open_start" "$open_hand"
sleep "$hold_sec"
run_hand_goal "middle_finger" "$middle_finger"
sleep "$hold_sec"
run_hand_goal "open_after_middle_finger" "$open_hand"
sleep "$hold_sec"
run_hand_goal "four_finger_fist" "$four_finger_fist"
sleep "$hold_sec"
run_hand_goal "open_finish" "$open_hand"

echo "HAND_GESTURE_DEMO=PASS"
echo "This script only uses $hand_action and never publishes to /real."
