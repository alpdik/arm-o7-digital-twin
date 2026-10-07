"""Read the 5DT Data Glove 5 Ultra over hidraw and publish /glove/finger_curls."""

import glob
import os
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_o7_glove.mapping import GLOVE_FINGERS, parse_5dt_report, raw_to_curl

HID_ID_5DT = "00005D70:00000010"
STALE_SEC = 0.3


def find_5dt_hidraw():
    for uevent in glob.glob("/sys/class/hidraw/hidraw*/device/uevent"):
        with open(uevent) as handle:
            if HID_ID_5DT in handle.read().upper():
                return "/dev/" + uevent.split("/")[4]
    return None


class GloveNode(Node):
    def __init__(self) -> None:
        super().__init__("glove_5dt")
        self.declare_parameter("device", "auto")
        # Sensor order in the HID report, as glove finger names.
        self.declare_parameter("sensor_fingers", list(GLOVE_FINGERS))
        # Raw counts for a relaxed open hand and a full fist, per sensor.
        self.declare_parameter("open_raw", [969, 2109, 1318, 1891, 1477])
        self.declare_parameter("closed_raw", [1962, 2869, 2611, 3023, 2629])

        self._fingers = list(self.get_parameter("sensor_fingers").value)
        self._open = list(self.get_parameter("open_raw").value)
        self._closed = list(self.get_parameter("closed_raw").value)
        if sorted(self._fingers) != sorted(GLOVE_FINGERS) or len(self._open) != 5 or len(self._closed) != 5:
            raise ValueError("sensor_fingers must name each finger once; open_raw/closed_raw need 5 values")

        # The teleop panel's calibration sets open_raw/closed_raw at runtime.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._pub = self.create_publisher(JointState, "/glove/finger_curls", 10)
        self._fd = None
        self._last_report = None
        self.create_timer(0.01, self._poll)
        self.create_timer(1.0, self._check)

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        updates = {}
        for parameter in parameters:
            if parameter.name in ("open_raw", "closed_raw"):
                values = list(parameter.value)
                if len(values) != 5:
                    return SetParametersResult(successful=False, reason=f"{parameter.name} needs 5 values")
                updates[parameter.name] = values
            elif parameter.name in ("device", "sensor_fingers"):
                return SetParametersResult(successful=False, reason=f"{parameter.name} is startup-only")
        if "open_raw" in updates:
            self._open = updates["open_raw"]
        if "closed_raw" in updates:
            self._closed = updates["closed_raw"]
        if updates:
            self.get_logger().info(f"calibration: open={self._open} closed={self._closed}")
        return SetParametersResult(successful=True)

    def _open_device(self) -> None:
        device = self.get_parameter("device").value
        if device == "auto":
            device = find_5dt_hidraw()
            if device is None:
                self.get_logger().error("5DT glove not found; is it plugged in?", throttle_duration_sec=5.0)
                return
        try:
            self._fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
            self.get_logger().info(f"Reading 5DT glove from {device}")
        except OSError as exc:
            self.get_logger().error(f"cannot open {device}: {exc}", throttle_duration_sec=5.0)

    def _poll(self) -> None:
        if self._fd is None:
            return
        report = None
        try:
            while True:
                report = os.read(self._fd, 64)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.get_logger().error(f"glove read failed ({exc}); reopening")
            os.close(self._fd)
            self._fd = None
            return
        if report is None:
            return
        try:
            raw = parse_5dt_report(report)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        self._last_report = time.monotonic()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._fingers
        message.position = [
            raw_to_curl(value, low, high)
            for value, low, high in zip(raw, self._open, self._closed)
        ]
        message.effort = [float(value) for value in raw]  # raw counts, for calibration
        self._pub.publish(message)

    def _check(self) -> None:
        if self._fd is None:
            self._open_device()
            return
        if self._last_report is None or time.monotonic() - self._last_report > STALE_SEC:
            self.get_logger().warning("no glove data", throttle_duration_sec=5.0)


def main() -> None:
    rclpy.init()
    node = GloveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
