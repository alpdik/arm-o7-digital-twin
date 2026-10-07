"""ROS-independent glove -> O7 model -> Ti5 hand conversions.

Data flow:
  5DT raw sensor counts --raw_to_curl--> curl 0..1 per finger
  curls --curls_to_hand_targets--> canonical O7 hand joint targets (rad)
  O7 hand joint positions (rad) --hand_to_ti5_angles--> Ti5 servo angles 0..90

All mappings are linear and provisional: they line up the ranges, not
measured kinematics. Verify on the real hand at low torque before trusting them.
"""

from typing import Dict, Mapping, Sequence

HAND_JOINTS = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
)

GLOVE_FINGERS = ("thumb", "index", "middle", "ring", "little")

# Model joint limits from arm_o7_twin.yaml (radians). Used only to scale O7
# positions onto the Ti5's 0..90 travel.
O7_UPPER_LIMITS = {
    "thumb_cmc_roll": 1.1339,
    "thumb_cmc_yaw": 1.9189,
    "thumb_cmc_pitch": 0.5146,
    "index_mcp_pitch": 1.3607,
    "middle_mcp_pitch": 1.3607,
    "ring_mcp_pitch": 1.3607,
    "pinky_mcp_pitch": 1.3607,
}

# Ti5 servo id (see ti5_protocol.FINGERS) -> O7 joint that drives it.
TI5_SERVO_SOURCES = {
    1: "pinky_mcp_pitch",   # little
    2: "ring_mcp_pitch",    # ring
    3: "middle_mcp_pitch",  # middle
    4: "index_mcp_pitch",   # index
    5: "thumb_cmc_pitch",   # thumb
    6: "thumb_cmc_yaw",     # thumb-base
}

TI5_OPEN = 0
TI5_CLOSED = 90

# The O7 "open" pose sits 0.001 rad above the zero lower limits so Gazebo's
# boundary noise never reads as out of bounds (same cushion as the SRDF).
OPEN_RAD = 0.001

# Fully curled targets. Fingers stop at the SRDF four_finger_fist pose and the
# thumb short of its limit, because the fully closed model self-collides.
DEFAULT_CLOSED_RAD = {
    "thumb_cmc_pitch": 0.45,
    "index_mcp_pitch": 1.15,
    "middle_mcp_pitch": 1.15,
    "ring_mcp_pitch": 1.15,
    "pinky_mcp_pitch": 1.10,
}

FINGER_TO_JOINT = {
    "thumb": "thumb_cmc_pitch",
    "index": "index_mcp_pitch",
    "middle": "middle_mcp_pitch",
    "ring": "ring_mcp_pitch",
    "little": "pinky_mcp_pitch",
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def raw_to_curl(raw: float, open_raw: float, closed_raw: float) -> float:
    """Normalise one sensor to 0 (open) .. 1 (curled).

    open_raw > closed_raw is allowed, for a sensor whose count falls as it bends.
    """
    span = closed_raw - open_raw
    if span == 0:
        return 0.0
    return clamp((raw - open_raw) / span, 0.0, 1.0)


def curls_to_hand_targets(
    curls: Mapping[str, float],
    closed_rad: Mapping[str, float] = DEFAULT_CLOSED_RAD,
    thumb_roll: float = OPEN_RAD,
    thumb_yaw: float = OPEN_RAD,
) -> Dict[str, float]:
    """Glove curls -> a complete O7 hand target.

    The 5DT has one thumb sensor, so it drives thumb_cmc_pitch only; the
    thumb's roll and yaw hold fixed values.
    """
    targets = {"thumb_cmc_roll": thumb_roll, "thumb_cmc_yaw": thumb_yaw}
    for finger, joint in FINGER_TO_JOINT.items():
        curl = clamp(float(curls[finger]), 0.0, 1.0)
        targets[joint] = OPEN_RAD + curl * (closed_rad[joint] - OPEN_RAD)
    return targets


def step_toward(
    targets: Mapping[str, float], current: Mapping[str, float], max_step: float
) -> Dict[str, float]:
    """Move each joint at most max_step from its current position.

    The arbiter rejects a command whose first point is more than its
    start-state tolerance from the measured state, so streamed commands must
    stay close to where the simulation actually is.
    """
    return {
        name: current[name] + clamp(target - current[name], -max_step, max_step)
        for name, target in targets.items()
    }


def hand_to_ti5_angles(positions: Mapping[str, float]) -> Dict[int, int]:
    """O7 hand joint positions (rad) -> Ti5 angles (0 open .. 90 closed)."""
    angles = {}
    for servo, joint in TI5_SERVO_SOURCES.items():
        fraction = clamp(positions[joint] / O7_UPPER_LIMITS[joint], 0.0, 1.0)
        angles[servo] = int(round(TI5_OPEN + fraction * (TI5_CLOSED - TI5_OPEN)))
    return angles


def ease_angles(
    current: Mapping[int, int], goal: Mapping[int, int], max_step: int
) -> Dict[int, int]:
    """Move each servo at most max_step angle units toward its goal."""
    return {
        servo: current[servo] + int(clamp(goal[servo] - current[servo], -max_step, max_step))
        for servo in current
    }


def parse_5dt_report(report: bytes) -> Sequence[int]:
    """5DT Data Glove 5 Ultra HID report -> five raw 12-bit sensor counts.

    Observed layout: five 6-byte groups, each a big-endian uint16 sent twice
    followed by two zero bytes, then status and the serial number in ASCII.
    """
    if len(report) < 30:
        raise ValueError(f"short report: {len(report)} bytes")
    return tuple((report[6 * i] << 8) | report[6 * i + 1] for i in range(5))


# A finger whose open and fist readings differ by less than this was not
# actually bent (or the sensor is dead); the full range is ~800-1300 counts.
MIN_CALIBRATION_SPAN = 150


def calibration_from_samples(
    open_samples: Mapping[str, Sequence[float]],
    closed_samples: Mapping[str, Sequence[float]],
    fingers: Sequence[str],
    min_span: float = MIN_CALIBRATION_SPAN,
):
    """Median raw counts per finger for the open hand and the fist.

    Returns (open_raw, closed_raw) as int lists in `fingers` order, or raises
    ValueError naming every finger that did not calibrate.
    """
    def median(values):
        ordered = sorted(values)
        return ordered[len(ordered) // 2]

    open_raw, closed_raw, problems = [], [], []
    for finger in fingers:
        opened = open_samples.get(finger) or []
        closed = closed_samples.get(finger) or []
        if not opened or not closed:
            problems.append(f"{finger}: no glove data")
            open_raw.append(0)
            closed_raw.append(0)
            continue
        low, high = int(median(opened)), int(median(closed))
        if abs(high - low) < min_span:
            problems.append(f"{finger} barely changed ({low} -> {high})")
        open_raw.append(low)
        closed_raw.append(high)
    if problems:
        raise ValueError("; ".join(problems))
    return open_raw, closed_raw
