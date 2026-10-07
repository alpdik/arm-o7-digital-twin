"""Runtime qualification for the local ARM1.5 and O7 digital twin."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Dict, Iterable, List
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = tuple(f"arm_joint_{index}" for index in range(1, 7))
HAND_JOINTS = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
)
REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "hand_controller",
    "thumb_coupling_controller",
}
REQUIRED_VALID_POSES = {
    ("arm", "home"),
    ("arm", "ready"),
    ("arm", "left_demo"),
    ("hand", "open"),
    ("hand", "pregrasp"),
    ("hand", "four_finger_fist"),
    ("hand", "middle_finger"),
}


@dataclass(frozen=True)
class NamedPose:
    """One complete SRDF group state."""

    group: str
    name: str
    joints: tuple[str, ...]
    positions: tuple[float, ...]


def load_named_poses() -> Dict[tuple[str, str], NamedPose]:
    """Load the installed SRDF as the single source of pose definitions."""

    package_share = Path(get_package_share_directory("arm_o7_moveit_config"))
    srdf_path = package_share / "config" / "arm_o7.srdf"
    root = ET.parse(srdf_path).getroot()
    poses: Dict[tuple[str, str], NamedPose] = {}
    for state in root.findall("group_state"):
        group = str(state.get("group"))
        name = str(state.get("name"))
        joints = tuple(str(item.get("name")) for item in state.findall("joint"))
        positions = tuple(float(item.get("value")) for item in state.findall("joint"))
        poses[(group, name)] = NamedPose(group, name, joints, positions)
    return poses


class QualificationNode(Node):
    """Measure interfaces, planning validity, execution, and tracking."""

    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("arm_o7_digital_twin_qualification")
        self.options = options
        self.latest_positions: Dict[str, float] = {}
        self.joint_sample_times: List[float] = []
        self.clock_samples: List[tuple[float, float]] = []

        self.create_subscription(
            JointState,
            options.joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Clock,
            "/clock",
            self._on_clock,
            qos_profile_sensor_data,
        )

        self.controller_client = self.create_client(
            ListControllers, options.controller_service
        )
        self.validity_client = self.create_client(
            GetStateValidity, options.validity_service
        )
        self.arm_action = ActionClient(
            self, FollowJointTrajectory, options.arm_action
        )
        self.hand_action = ActionClient(
            self, FollowJointTrajectory, options.hand_action
        )
        self.move_action = ActionClient(self, MoveGroup, options.move_action)

    def _on_joint_state(self, message: JointState) -> None:
        now = time.monotonic()
        self.joint_sample_times.append(now)
        if len(self.joint_sample_times) > 10000:
            self.joint_sample_times = self.joint_sample_times[-5000:]
        for name, position in zip(message.name, message.position):
            self.latest_positions[name] = float(position)

    def _on_clock(self, message: Clock) -> None:
        wall = time.monotonic()
        simulated = float(message.clock.sec) + float(message.clock.nanosec) / 1.0e9
        self.clock_samples.append((wall, simulated))
        if len(self.clock_samples) > 10000:
            self.clock_samples = self.clock_samples[-5000:]

    def wait_for_runtime(self, timeout_sec: float) -> List[str]:
        """Wait for every interface required by the qualification run."""

        failures = []
        if not self.controller_client.wait_for_service(timeout_sec=timeout_sec):
            failures.append(f"missing service {self.options.controller_service}")
        if not self.validity_client.wait_for_service(timeout_sec=timeout_sec):
            failures.append(f"missing service {self.options.validity_service}")
        if not self.arm_action.wait_for_server(timeout_sec=timeout_sec):
            failures.append(f"missing action {self.options.arm_action}")
        if not self.hand_action.wait_for_server(timeout_sec=timeout_sec):
            failures.append(f"missing action {self.options.hand_action}")
        if not self.move_action.wait_for_server(timeout_sec=timeout_sec):
            failures.append(f"missing action {self.options.move_action}")

        deadline = time.monotonic() + timeout_sec
        expected = set(ARM_JOINTS + HAND_JOINTS)
        while rclpy.ok() and time.monotonic() < deadline:
            if expected.issubset(self.latest_positions):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = sorted(expected - set(self.latest_positions))
        if missing:
            failures.append(f"joint states missing: {', '.join(missing)}")
        return failures

    def _future_result(self, future, timeout_sec: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        return future.result()

    def list_controllers(self) -> Dict[str, str]:
        """Return controller names and lifecycle states."""

        response = self._future_result(
            self.controller_client.call_async(ListControllers.Request()),
            self.options.interface_timeout,
        )
        if response is None:
            return {}
        return {item.name: item.state for item in response.controller}

    def measure_runtime(self, duration_sec: float) -> dict:
        """Measure joint-state rate and Gazebo simulated/wall time ratio."""

        self.joint_sample_times.clear()
        self.clock_samples.clear()
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        joint_rate = 0.0
        if len(self.joint_sample_times) >= 2:
            elapsed = self.joint_sample_times[-1] - self.joint_sample_times[0]
            if elapsed > 0.0:
                joint_rate = (len(self.joint_sample_times) - 1) / elapsed

        realtime_factor = 0.0
        if len(self.clock_samples) >= 2:
            wall_delta = self.clock_samples[-1][0] - self.clock_samples[0][0]
            sim_delta = self.clock_samples[-1][1] - self.clock_samples[0][1]
            if wall_delta > 0.0:
                realtime_factor = sim_delta / wall_delta

        return {
            "duration_sec": duration_sec,
            "joint_state_samples": len(self.joint_sample_times),
            "joint_state_rate_hz": joint_rate,
            "clock_samples": len(self.clock_samples),
            "realtime_factor": realtime_factor,
        }

    def check_pose_validity(self, pose: NamedPose) -> dict:
        """Ask MoveIt whether one named state is collision and bounds valid."""

        request = GetStateValidity.Request()
        request.group_name = pose.group
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(pose.joints)
        request.robot_state.joint_state.position = list(pose.positions)
        response = self._future_result(
            self.validity_client.call_async(request), self.options.interface_timeout
        )
        if response is None:
            return {
                "group": pose.group,
                "pose": pose.name,
                "valid": False,
                "contacts": [],
                "error": "state-validity service timeout",
            }
        contacts = []
        for contact in response.contacts:
            contacts.append(
                {
                    "body_1": contact.contact_body_1,
                    "body_2": contact.contact_body_2,
                }
            )
        return {
            "group": pose.group,
            "pose": pose.name,
            "valid": bool(response.valid),
            "contacts": contacts,
            "error": "",
        }

    def _tracking_error(self, pose: NamedPose) -> tuple[float, dict]:
        errors = {}
        for name, expected in zip(pose.joints, pose.positions):
            actual = self.latest_positions.get(name)
            errors[name] = None if actual is None else abs(actual - expected)
        finite = [value for value in errors.values() if value is not None]
        maximum = max(finite) if len(finite) == len(errors) else math.inf
        return maximum, errors

    def execute_direct(self, pose: NamedPose, duration_sec: int) -> dict:
        """Send a baseline action directly to one trajectory controller."""

        action = self.arm_action if pose.group == "arm" else self.hand_action
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(pose.joints)
        point = JointTrajectoryPoint()
        point.positions = list(pose.positions)
        point.time_from_start.sec = duration_sec
        goal.trajectory.points = [point]

        started = time.monotonic()
        goal_handle = self._future_result(
            action.send_goal_async(goal), self.options.interface_timeout
        )
        if goal_handle is None or not goal_handle.accepted:
            return self._execution_record(
                pose, "direct", False, -1, "goal rejected", started
            )
        wrapped = self._future_result(
            goal_handle.get_result_async(), duration_sec + self.options.execution_margin
        )
        if wrapped is None:
            return self._execution_record(
                pose, "direct", False, -1, "result timeout", started
            )
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == 0
        )
        return self._execution_record(
            pose,
            "direct",
            succeeded,
            int(wrapped.result.error_code),
            str(wrapped.result.error_string),
            started,
        )

    def execute_moveit(self, pose: NamedPose) -> dict:
        """Plan and execute a named pose through MoveIt move_group."""

        constraints = Constraints()
        constraints.name = pose.name
        for joint_name, position in zip(pose.joints, pose.positions):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = pose.group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.options.velocity_scale
        goal.request.max_acceleration_scaling_factor = self.options.acceleration_scale
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        started = time.monotonic()
        goal_handle = self._future_result(
            self.move_action.send_goal_async(goal), self.options.interface_timeout
        )
        if goal_handle is None or not goal_handle.accepted:
            return self._execution_record(
                pose, "moveit", False, -1, "MoveIt goal rejected", started
            )
        wrapped = self._future_result(
            goal_handle.get_result_async(), self.options.moveit_timeout
        )
        if wrapped is None:
            return self._execution_record(
                pose, "moveit", False, -1, "MoveIt result timeout", started
            )
        code = int(wrapped.result.error_code.val)
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and code == MoveItErrorCodes.SUCCESS
        )
        message = f"MoveIt error_code={code}"
        return self._execution_record(
            pose, "moveit", succeeded, code, message, started
        )

    def _execution_record(
        self,
        pose: NamedPose,
        source: str,
        succeeded: bool,
        result_code: int,
        message: str,
        started: float,
    ) -> dict:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        max_error, joint_errors = self._tracking_error(pose)
        tolerance = (
            self.options.arm_error_tolerance
            if pose.group == "arm"
            else self.options.hand_error_tolerance
        )
        tracking_ok = math.isfinite(max_error) and max_error <= tolerance
        return {
            "group": pose.group,
            "pose": pose.name,
            "source": source,
            "succeeded": bool(succeeded and tracking_ok),
            "action_succeeded": bool(succeeded),
            "result_code": result_code,
            "message": message,
            "elapsed_sec": time.monotonic() - started,
            "max_error_rad": max_error if math.isfinite(max_error) else None,
            "tolerance_rad": tolerance,
            "joint_errors_rad": joint_errors,
        }


def required_pose(
    poses: Dict[tuple[str, str], NamedPose], group: str, name: str
) -> NamedPose:
    """Return a required pose with a readable failure."""

    key = (group, name)
    if key not in poses:
        raise RuntimeError(f"required SRDF pose is missing: {group}/{name}")
    return poses[key]


def print_execution(cycle: int, record: dict) -> None:
    """Print one compact execution result."""

    status = "PASS" if record["succeeded"] else "FAIL"
    error = record["max_error_rad"]
    error_text = "missing" if error is None else f"{error:.4f}rad"
    print(
        f"[{status}] cycle={cycle} source={record['source']} "
        f"{record['group']}/{record['pose']} "
        f"error={error_text} "
        f"elapsed={record['elapsed_sec']:.2f}s"
    )


def write_csv(path: Path, executions: Iterable[dict]) -> None:
    """Write flat execution rows for later analysis."""

    fieldnames = [
        "cycle",
        "group",
        "pose",
        "source",
        "succeeded",
        "action_succeeded",
        "result_code",
        "message",
        "elapsed_sec",
        "max_error_rad",
        "tolerance_rad",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in executions:
            writer.writerow({key: record.get(key) for key in fieldnames})


def build_parser() -> argparse.ArgumentParser:
    """Create command-line options for local repeatable runs."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--output-dir", default="qualification_results")
    parser.add_argument("--interface-timeout", type=float, default=10.0)
    parser.add_argument("--execution-margin", type=float, default=8.0)
    parser.add_argument("--moveit-timeout", type=float, default=30.0)
    parser.add_argument("--measurement-duration", type=float, default=3.0)
    parser.add_argument("--min-joint-state-rate", type=float, default=20.0)
    parser.add_argument("--min-realtime-factor", type=float, default=0.80)
    parser.add_argument("--arm-error-tolerance", type=float, default=0.05)
    parser.add_argument("--hand-error-tolerance", type=float, default=0.08)
    parser.add_argument("--velocity-scale", type=float, default=0.60)
    parser.add_argument("--acceleration-scale", type=float, default=0.50)
    parser.add_argument("--joint-state-topic", default="/sim/joint_states")
    parser.add_argument(
        "--controller-service", default="/sim/controller_manager/list_controllers"
    )
    parser.add_argument("--validity-service", default="/sim/check_state_validity")
    parser.add_argument(
        "--arm-action", default="/sim/arm_controller/follow_joint_trajectory"
    )
    parser.add_argument(
        "--hand-action", default="/sim/hand_controller/follow_joint_trajectory"
    )
    parser.add_argument("--move-action", default="/sim/move_action")
    return parser


def validate_options(options: argparse.Namespace) -> None:
    """Reject nonsensical qualification settings before moving the robot."""

    if options.cycles < 1:
        raise ValueError("--cycles must be at least 1")
    bounded_scales = (options.velocity_scale, options.acceleration_scale)
    if any(not 0.0 < value <= 1.0 for value in bounded_scales):
        raise ValueError("velocity and acceleration scales must be in (0, 1]")


def main() -> None:
    """Run the qualification sequence and persist JSON/CSV evidence."""

    non_ros_args = remove_ros_args(args=sys.argv)
    options = build_parser().parse_args(non_ros_args[1:])
    validate_options(options)
    rclpy.init(args=sys.argv)
    node = QualificationNode(options)
    started_at = datetime.now(timezone.utc)
    report = {
        "schema_version": 1,
        "started_at_utc": started_at.isoformat(),
        "options": vars(options),
        "controllers": {},
        "runtime": {},
        "validity": [],
        "executions": [],
        "failures": [],
        "status": "FAIL",
    }

    try:
        failures = node.wait_for_runtime(options.interface_timeout)
        if failures:
            report["failures"].extend(failures)
            raise RuntimeError("; ".join(failures))

        controllers = node.list_controllers()
        report["controllers"] = controllers
        inactive = sorted(
            name for name in REQUIRED_CONTROLLERS if controllers.get(name) != "active"
        )
        if inactive:
            report["failures"].append(
                f"controllers not active: {', '.join(inactive)}"
            )

        report["runtime"] = node.measure_runtime(options.measurement_duration)
        if report["runtime"]["joint_state_rate_hz"] < options.min_joint_state_rate:
            report["failures"].append("joint-state publication rate is too low")
        if report["runtime"]["realtime_factor"] < options.min_realtime_factor:
            report["failures"].append("Gazebo real-time factor is too low")

        poses = load_named_poses()
        ready = required_pose(poses, "arm", "ready")
        open_hand = required_pose(poses, "hand", "open")

        for pose, duration in ((open_hand, 4), (ready, 5)):
            record = node.execute_direct(pose, duration)
            record["cycle"] = 0
            report["executions"].append(record)
            print_execution(0, record)
            if not record["succeeded"]:
                report["failures"].append(
                    f"baseline action failed: {pose.group}/{pose.name}"
                )

        validity_keys = sorted(REQUIRED_VALID_POSES | {("hand", "contact_closed")})
        validity_by_key = {}
        for key in validity_keys:
            pose = required_pose(poses, *key)
            result = node.check_pose_validity(pose)
            report["validity"].append(result)
            validity_by_key[key] = result["valid"]
            expectation = (
                "diagnostic" if key == ("hand", "contact_closed") else "required"
            )
            print(
                f"[VALIDITY] {pose.group}/{pose.name} "
                f"valid={result['valid']} expectation={expectation}"
            )
            if key in REQUIRED_VALID_POSES and not result["valid"]:
                report["failures"].append(
                    f"required pose is invalid: {pose.group}/{pose.name}"
                )

        sequence = [
            ("arm", "left_demo"),
            ("hand", "pregrasp"),
            ("hand", "open"),
            ("hand", "middle_finger"),
            ("hand", "open"),
            ("hand", "four_finger_fist"),
            ("hand", "open"),
            ("arm", "home"),
            ("arm", "ready"),
        ]
        for cycle in range(1, options.cycles + 1):
            for key in sequence:
                pose = required_pose(poses, *key)
                if not validity_by_key.get(key, False):
                    continue
                record = node.execute_moveit(pose)
                record["cycle"] = cycle
                report["executions"].append(record)
                print_execution(cycle, record)
                if not record["succeeded"]:
                    report["failures"].append(
                        f"MoveIt execution failed in cycle {cycle}: "
                        f"{pose.group}/{pose.name}"
                    )

        report["status"] = "PASS" if not report["failures"] else "FAIL"
    except Exception as error:  # Keep evidence even when a ROS interface fails.
        if str(error) not in report["failures"]:
            report["failures"].append(str(error))
        node.get_logger().error(f"qualification aborted: {error}")
    finally:
        finished_at = datetime.now(timezone.utc)
        report["finished_at_utc"] = finished_at.isoformat()
        report["elapsed_sec"] = (finished_at - started_at).total_seconds()
        output_root = Path(options.output_dir).expanduser().resolve()
        run_name = started_at.strftime("%Y%m%dT%H%M%SZ")
        run_dir = output_root / run_name
        run_dir.mkdir(parents=True, exist_ok=False)
        json_path = run_dir / "qualification.json"
        csv_path = run_dir / "executions.csv"
        json_path.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False),
            encoding="utf-8",
        )
        write_csv(csv_path, report["executions"])
        print(f"REPORT_JSON={json_path}")
        print(f"REPORT_CSV={csv_path}")
        print(f"DIGITAL_TWIN_QUALIFICATION={report['status']}")
        if report["failures"]:
            for failure in report["failures"]:
                print(f"FAILURE: {failure}")
        status = report["status"]
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if status == "PASS" else 1)


if __name__ == "__main__":
    main()
