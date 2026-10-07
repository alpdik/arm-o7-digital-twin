"""ROS-independent mapping, validation, interpolation, and state merging."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple


ARM_JOINTS: Tuple[str, ...] = tuple(f"arm_joint_{index}" for index in range(1, 7))
HAND_JOINTS: Tuple[str, ...] = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
)


class AdapterError(ValueError):
    """A deterministic adapter input or configuration failure."""


@dataclass(frozen=True)
class JointMap:
    canonical_name: str
    sdk_index: int
    scale: float
    offset: float
    minimum: float
    maximum: float

    def __post_init__(self) -> None:
        values = (self.scale, self.offset, self.minimum, self.maximum)
        if any(not math.isfinite(value) for value in values):
            raise AdapterError(f"{self.canonical_name}: mapping values must be finite")
        if self.scale == 0.0:
            raise AdapterError(f"{self.canonical_name}: scale cannot be zero")
        if self.minimum >= self.maximum:
            raise AdapterError(f"{self.canonical_name}: invalid canonical limits")


def validate_maps(maps: Sequence[JointMap]) -> Tuple[JointMap, ...]:
    maps = tuple(maps)
    if {item.canonical_name for item in maps} != set(HAND_JOINTS):
        raise AdapterError("mapping must contain each canonical hand joint exactly once")
    if {item.sdk_index for item in maps} != set(range(7)):
        raise AdapterError("SDK indices must be exactly 0 through 6")
    return maps


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def canonical_to_sdk(
    names: Sequence[str], positions: Sequence[float], maps: Sequence[JointMap]
) -> Tuple[float, ...]:
    maps = validate_maps(maps)
    if len(names) != len(positions):
        raise AdapterError("joint name and position lengths differ")
    if len(set(names)) != len(names):
        raise AdapterError("duplicate canonical joint name")
    if set(names) != set(HAND_JOINTS):
        raise AdapterError("command must contain all seven canonical hand joints")
    if not _finite(positions):
        raise AdapterError("command contains a non-finite position")

    by_name = dict(zip(names, (float(value) for value in positions)))
    result = [0.0] * 7
    for item in maps:
        value = by_name[item.canonical_name]
        if value < item.minimum or value > item.maximum:
            raise AdapterError(
                f"{item.canonical_name}={value:.6f} is outside "
                f"[{item.minimum:.6f}, {item.maximum:.6f}]"
            )
        result[item.sdk_index] = item.offset + item.scale * value
    return tuple(result)


def sdk_to_canonical(
    sdk_positions: Sequence[float], maps: Sequence[JointMap]
) -> Tuple[float, ...]:
    maps = validate_maps(maps)
    if len(sdk_positions) != 7:
        raise AdapterError("SDK O7 state must contain exactly seven positions")
    if not _finite(sdk_positions):
        raise AdapterError("SDK state contains a non-finite position")
    by_name = {
        item.canonical_name: (float(sdk_positions[item.sdk_index]) - item.offset)
        / item.scale
        for item in maps
    }
    return tuple(by_name[name] for name in HAND_JOINTS)


def validate_trajectory(
    names: Sequence[str],
    points: Sequence[Tuple[Sequence[float], float]],
    maps: Sequence[JointMap],
    max_duration_sec: float,
) -> Tuple[Tuple[Tuple[float, ...], float], ...]:
    if not points:
        raise AdapterError("trajectory has no points")
    if not math.isfinite(max_duration_sec) or max_duration_sec <= 0.0:
        raise AdapterError("max trajectory duration must be positive and finite")
    checked = []
    previous_time = -1.0
    for positions, time_sec in points:
        if not math.isfinite(time_sec) or time_sec < 0.0 or time_sec <= previous_time:
            raise AdapterError("trajectory times must be finite and strictly increasing")
        canonical_to_sdk(names, positions, maps)
        by_name = dict(zip(names, (float(value) for value in positions)))
        ordered = tuple(by_name[name] for name in HAND_JOINTS)
        checked.append((ordered, float(time_sec)))
        previous_time = time_sec
    if checked[-1][1] > max_duration_sec:
        raise AdapterError("trajectory exceeds the configured maximum duration")
    return tuple(checked)


def sample_trajectory(
    points: Sequence[Tuple[Sequence[float], float]],
    elapsed_sec: float,
    initial_positions: Optional[Sequence[float]] = None,
) -> Tuple[Tuple[float, ...], bool]:
    """Linearly sample a validated trajectory; return ``(positions, finished)``."""

    if not points:
        raise AdapterError("cannot sample an empty trajectory")
    if not math.isfinite(elapsed_sec):
        raise AdapterError("elapsed time must be finite")
    elapsed_sec = max(0.0, elapsed_sec)
    normalized = [(tuple(float(v) for v in positions), float(t)) for positions, t in points]
    if initial_positions is not None and normalized[0][1] > 0.0:
        initial = tuple(float(v) for v in initial_positions)
        if len(initial) != len(normalized[0][0]) or not _finite(initial):
            raise AdapterError("initial position vector is invalid")
        normalized.insert(0, (initial, 0.0))

    if elapsed_sec >= normalized[-1][1]:
        return normalized[-1][0], True
    if elapsed_sec <= normalized[0][1]:
        return normalized[0][0], False
    for (left, left_t), (right, right_t) in zip(normalized, normalized[1:]):
        if elapsed_sec <= right_t:
            ratio = (elapsed_sec - left_t) / (right_t - left_t)
            return tuple(a + ratio * (b - a) for a, b in zip(left, right)), False
    raise AssertionError("trajectory sampling fell through")


def merge_joint_states(
    arm: Mapping[str, float], hand: Mapping[str, float]
) -> Tuple[Tuple[str, ...], Tuple[float, ...]]:
    missing_arm = set(ARM_JOINTS).difference(arm)
    missing_hand = set(HAND_JOINTS).difference(hand)
    if missing_arm or missing_hand:
        raise AdapterError(
            f"incomplete state; missing arm={sorted(missing_arm)}, hand={sorted(missing_hand)}"
        )
    names = ARM_JOINTS + HAND_JOINTS
    positions = tuple(float((arm | hand)[name]) for name in names)
    if not _finite(positions):
        raise AdapterError("merged state contains a non-finite position")
    return names, positions
