"""ROS-independent safety and trajectory logic for the twin arbiter.

Keeping this module free of ROS imports makes the safety rules inexpensive to
unit-test on a development machine that does not have ROS 2 installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple


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
CANONICAL_JOINTS: Tuple[str, ...] = ARM_JOINTS + HAND_JOINTS


class Mode(str, Enum):
    """Command-routing modes.

    SHADOW mirrors validated real state only to simulation while requiring the
    real and simulated states to agree. TWIN_COMMAND is the only mode that
    publishes the same accepted canonical command to both sides.
    """

    SAFE_IDLE = "SAFE_IDLE"
    SIM_ONLY = "SIM_ONLY"
    SHADOW = "SHADOW"
    TWIN_COMMAND = "TWIN_COMMAND"
    REAL_ONLY = "REAL_ONLY"

    @classmethod
    def parse(cls, value: str) -> "Mode":
        normalized = value.strip().upper().replace("-", "_").replace(" ", "_")
        try:
            return cls(normalized)
        except ValueError as exc:
            expected = ", ".join(member.value for member in cls)
            raise ValueError(f"unknown mode {value!r}; expected one of: {expected}") from exc


@dataclass(frozen=True)
class JointLimit:
    minimum: float
    maximum: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.minimum) or not math.isfinite(self.maximum):
            raise ValueError("joint limits must be finite")
        if self.minimum >= self.maximum:
            raise ValueError("joint-limit minimum must be less than maximum")


@dataclass(frozen=True)
class TrajectoryPointData:
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...] = ()
    accelerations: Tuple[float, ...] = ()
    effort: Tuple[float, ...] = ()
    time_from_start_ns: int = 0


@dataclass(frozen=True)
class TrajectoryData:
    joint_names: Tuple[str, ...]
    points: Tuple[TrajectoryPointData, ...]
    frame_id: str = ""
    stamp_sec: int = 0
    stamp_nanosec: int = 0


@dataclass(frozen=True)
class JointStateSnapshot:
    positions: Mapping[str, float]
    received_monotonic: float


@dataclass(frozen=True)
class SafetyConfig:
    joint_limits: Mapping[str, JointLimit]
    heartbeat_timeout_sec: float = 0.30
    deadman_timeout_sec: float = 0.30
    joint_state_timeout_sec: float = 0.30
    start_state_tolerance_rad: float = 0.10
    tracking_error_tolerance_rad: float = 0.20
    require_all_command_joints: bool = False
    require_complete_controller_groups: bool = True

    def __post_init__(self) -> None:
        numeric_values = (
            self.heartbeat_timeout_sec,
            self.deadman_timeout_sec,
            self.joint_state_timeout_sec,
            self.start_state_tolerance_rad,
            self.tracking_error_tolerance_rad,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in numeric_values):
            raise ValueError("timeouts and tolerances must be finite and greater than zero")
        missing = set(CANONICAL_JOINTS).difference(self.joint_limits)
        if missing:
            raise ValueError(f"missing joint limits for: {', '.join(sorted(missing))}")


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    reason: str
    route_sim: bool = False
    route_real: bool = False
    tracking_error_rad: Optional[float] = None


class ValidationError(ValueError):
    """One or more deterministic trajectory-validation failures."""

    def __init__(self, issues: Sequence[str]):
        self.issues = tuple(issues)
        super().__init__("; ".join(self.issues))


def default_joint_limits() -> Dict[str, JointLimit]:
    """Return the position limits currently configured in the combined model."""

    bounds = {
        "arm_joint_1": (-3.14, 3.14),
        "arm_joint_2": (-1.57, 1.57),
        "arm_joint_3": (-1.30, 1.30),
        "arm_joint_4": (-3.14, 3.14),
        "arm_joint_5": (-1.30, 1.30),
        "arm_joint_6": (-3.14, 3.14),
        "thumb_cmc_roll": (0.0, 1.1339),
        "thumb_cmc_yaw": (0.0, 1.9189),
        "thumb_cmc_pitch": (0.0, 0.5146),
        "index_mcp_pitch": (0.0, 1.3607),
        "middle_mcp_pitch": (0.0, 1.3607),
        "ring_mcp_pitch": (0.0, 1.3607),
        "pinky_mcp_pitch": (0.0, 1.3607),
    }
    return {
        name: JointLimit(minimum, maximum)
        for name, (minimum, maximum) in bounds.items()
    }


def routes_for_mode(mode: Mode) -> Tuple[bool, bool]:
    """Return ``(route_sim, route_real)`` for a mode."""

    if mode is Mode.SAFE_IDLE:
        return False, False
    if mode in (Mode.SIM_ONLY, Mode.SHADOW):
        return True, False
    if mode is Mode.TWIN_COMMAND:
        return True, True
    if mode is Mode.REAL_ONLY:
        return False, True
    raise AssertionError(f"unhandled mode: {mode}")


def _finite_sequence(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate_trajectory(
    trajectory: TrajectoryData,
    config: SafetyConfig,
) -> None:
    """Validate names, vector sizes, values, timing, and position limits."""

    issues = []
    names = trajectory.joint_names
    name_count = len(names)

    if not names:
        issues.append("joint_names is empty")
    if any(not name or name.strip() != name for name in names):
        issues.append("joint_names contains an empty or whitespace-padded name")
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        issues.append(f"duplicate joint names: {', '.join(duplicates)}")
    unknown = sorted(set(names).difference(CANONICAL_JOINTS))
    if unknown:
        issues.append(f"unknown joint names: {', '.join(unknown)}")
    if config.require_all_command_joints:
        missing = sorted(set(CANONICAL_JOINTS).difference(names))
        if missing:
            issues.append(f"command omits required joints: {', '.join(missing)}")
    elif config.require_complete_controller_groups:
        for label, group in (("arm", ARM_JOINTS), ("hand", HAND_JOINTS)):
            present = set(names).intersection(group)
            missing = tuple(name for name in group if name not in names)
            if present and missing:
                issues.append(
                    f"partial {label} controller group; missing: {', '.join(missing)}"
                )
    if not trajectory.points:
        issues.append("trajectory has no points")

    previous_time = -1
    for point_index, point in enumerate(trajectory.points):
        prefix = f"point[{point_index}]"
        if len(point.positions) != name_count:
            issues.append(
                f"{prefix}.positions has {len(point.positions)} values; expected {name_count}"
            )
        for field_name in ("velocities", "accelerations", "effort"):
            field_values = getattr(point, field_name)
            if field_values and len(field_values) != name_count:
                issues.append(
                    f"{prefix}.{field_name} has {len(field_values)} values; "
                    f"expected 0 or {name_count}"
                )
        for field_name in ("positions", "velocities", "accelerations", "effort"):
            field_values = getattr(point, field_name)
            if not _finite_sequence(field_values):
                issues.append(f"{prefix}.{field_name} contains a non-finite value")
        if point.time_from_start_ns < 0:
            issues.append(f"{prefix}.time_from_start is negative")
        if point.time_from_start_ns <= previous_time:
            issues.append(f"{prefix}.time_from_start is not strictly increasing")
        previous_time = point.time_from_start_ns

        if len(point.positions) == name_count:
            for name, position in zip(names, point.positions):
                limit = config.joint_limits.get(name)
                if limit is None or not math.isfinite(float(position)):
                    continue
                if position < limit.minimum or position > limit.maximum:
                    issues.append(
                        f"{prefix}.{name}={position:.6g} is outside "
                        f"[{limit.minimum:.6g}, {limit.maximum:.6g}]"
                    )

    if issues:
        raise ValidationError(issues)


def make_joint_state_snapshot(
    names: Sequence[str],
    positions: Sequence[float],
    received_monotonic: float,
) -> JointStateSnapshot:
    """Validate and normalize an incoming JointState sample."""

    issues = []
    if len(names) != len(positions):
        issues.append(f"joint state has {len(names)} names and {len(positions)} positions")
    if len(set(names)) != len(names):
        issues.append("joint state contains duplicate names")
    if not _finite_sequence(positions):
        issues.append("joint state contains a non-finite position")
    if not math.isfinite(received_monotonic):
        issues.append("joint-state receive time is non-finite")
    if issues:
        raise ValidationError(issues)
    return JointStateSnapshot(dict(zip(names, positions)), received_monotonic)


def is_fresh(
    snapshot: Optional[JointStateSnapshot],
    now_monotonic: float,
    timeout_sec: float,
) -> bool:
    if snapshot is None:
        return False
    age = now_monotonic - snapshot.received_monotonic
    return math.isfinite(age) and 0.0 <= age <= timeout_sec


def start_state_error(
    trajectory: TrajectoryData,
    snapshot: JointStateSnapshot,
) -> Tuple[float, Tuple[str, ...]]:
    """Return max first-point error and state joints missing from the sample."""

    first = trajectory.points[0]
    missing = tuple(name for name in trajectory.joint_names if name not in snapshot.positions)
    errors = [
        abs(position - snapshot.positions[name])
        for name, position in zip(trajectory.joint_names, first.positions)
        if name in snapshot.positions
    ]
    return (max(errors, default=0.0), missing)


def tracking_error(
    sim_state: JointStateSnapshot,
    real_state: JointStateSnapshot,
    joint_names: Sequence[str],
) -> Tuple[float, Tuple[str, ...]]:
    """Return max sim-real error and joints missing from either state."""

    missing = tuple(
        name
        for name in joint_names
        if name not in sim_state.positions or name not in real_state.positions
    )
    errors = [
        abs(sim_state.positions[name] - real_state.positions[name])
        for name in joint_names
        if name in sim_state.positions and name in real_state.positions
    ]
    return (max(errors, default=0.0), missing)


def evaluate_command(
    *,
    mode: Mode,
    trajectory: TrajectoryData,
    config: SafetyConfig,
    now_monotonic: float,
    deadman_pressed: bool,
    deadman_monotonic: Optional[float],
    heartbeat_monotonic: Optional[float],
    sim_state: Optional[JointStateSnapshot],
    real_state: Optional[JointStateSnapshot],
    enable_real_output: bool,
) -> GateDecision:
    """Apply all fail-closed command interlocks in a deterministic order."""

    try:
        validate_trajectory(trajectory, config)
    except ValidationError as exc:
        return GateDecision(False, f"trajectory rejected: {exc}")

    route_sim, route_real = routes_for_mode(mode)
    if mode is Mode.SAFE_IDLE:
        return GateDecision(False, "SAFE_IDLE does not forward commands")
    if route_real and not enable_real_output:
        return GateDecision(False, "real output is disabled by configuration")
    if not deadman_pressed:
        return GateDecision(False, "deadman is not pressed")
    if deadman_monotonic is None:
        return GateDecision(False, "deadman state has not been received")
    deadman_age = now_monotonic - deadman_monotonic
    if (
        not math.isfinite(deadman_age)
        or deadman_age < 0.0
        or deadman_age > config.deadman_timeout_sec
    ):
        return GateDecision(False, "deadman state is stale")
    if heartbeat_monotonic is None:
        return GateDecision(False, "heartbeat has not been received")
    heartbeat_age = now_monotonic - heartbeat_monotonic
    if (
        not math.isfinite(heartbeat_age)
        or heartbeat_age < 0.0
        or heartbeat_age > config.heartbeat_timeout_sec
    ):
        return GateDecision(False, "heartbeat is stale")

    required_states = []
    if route_sim:
        required_states.append(("sim", sim_state))
    if route_real or mode is Mode.SHADOW:
        required_states.append(("real", real_state))
    for side, snapshot in required_states:
        if not is_fresh(snapshot, now_monotonic, config.joint_state_timeout_sec):
            return GateDecision(False, f"{side} joint state is missing or stale")
        assert snapshot is not None
        error, missing = start_state_error(trajectory, snapshot)
        if missing:
            return GateDecision(
                False,
                f"{side} joint state is missing commanded joints: {', '.join(missing)}",
            )
        if error > config.start_state_tolerance_rad:
            return GateDecision(
                False,
                f"{side} start-state mismatch {error:.4f} rad exceeds "
                f"{config.start_state_tolerance_rad:.4f} rad",
            )

    measured_tracking_error = None
    if mode in (Mode.SHADOW, Mode.TWIN_COMMAND):
        assert sim_state is not None and real_state is not None
        measured_tracking_error, missing = tracking_error(
            sim_state, real_state, trajectory.joint_names
        )
        if missing:
            return GateDecision(
                False,
                f"tracking state is missing commanded joints: {', '.join(missing)}",
            )
        if measured_tracking_error > config.tracking_error_tolerance_rad:
            return GateDecision(
                False,
                f"sim-real tracking error {measured_tracking_error:.4f} rad exceeds "
                f"{config.tracking_error_tolerance_rad:.4f} rad",
                tracking_error_rad=measured_tracking_error,
            )

    return GateDecision(
        True,
        "accepted",
        route_sim=route_sim,
        route_real=route_real,
        tracking_error_rad=measured_tracking_error,
    )


def split_trajectory(
    trajectory: TrajectoryData,
    selected_joint_names: Sequence[str],
) -> Optional[TrajectoryData]:
    """Select a controller-specific subset in the supplied canonical order."""

    source_indices = {name: index for index, name in enumerate(trajectory.joint_names)}
    output_names = tuple(name for name in selected_joint_names if name in source_indices)
    if not output_names:
        return None
    indices = tuple(source_indices[name] for name in output_names)

    def select(values: Tuple[float, ...]) -> Tuple[float, ...]:
        if not values:
            return ()
        return tuple(values[index] for index in indices)

    points = tuple(
        TrajectoryPointData(
            positions=select(point.positions),
            velocities=select(point.velocities),
            accelerations=select(point.accelerations),
            effort=select(point.effort),
            time_from_start_ns=point.time_from_start_ns,
        )
        for point in trajectory.points
    )
    return TrajectoryData(
        joint_names=output_names,
        points=points,
        frame_id=trajectory.frame_id,
        stamp_sec=trajectory.stamp_sec,
        stamp_nanosec=trajectory.stamp_nanosec,
    )


def make_shadow_trajectory(
    real_state: JointStateSnapshot,
    duration_sec: float,
    config: SafetyConfig,
    *,
    require_all_joints: bool = True,
) -> TrajectoryData:
    """Turn a validated real state into a short, position-only sim command.

    This function deliberately has no routing concept: the caller must publish
    its result only on simulation outputs. Requiring a complete combined state
    by default prevents a partial JointState publisher from silently freezing
    the rest of the shadow model.
    """

    issues = []
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        issues.append("shadow trajectory duration must be finite and greater than zero")
    available = tuple(name for name in CANONICAL_JOINTS if name in real_state.positions)
    missing = tuple(name for name in CANONICAL_JOINTS if name not in real_state.positions)
    if require_all_joints and missing:
        issues.append(f"real shadow state is missing joints: {', '.join(missing)}")
    if not available:
        issues.append("real shadow state contains no canonical joints")
    if issues:
        raise ValidationError(issues)

    duration_ns = int(round(duration_sec * 1_000_000_000.0))
    if duration_ns <= 0:
        raise ValidationError(("shadow trajectory duration rounds to zero",))
    trajectory = TrajectoryData(
        joint_names=available,
        points=(
            TrajectoryPointData(
                positions=tuple(real_state.positions[name] for name in available),
                time_from_start_ns=duration_ns,
            ),
        ),
    )
    validate_trajectory(trajectory, config)
    return trajectory


def shadow_publish_due(
    last_publish_monotonic: Optional[float],
    now_monotonic: float,
    maximum_rate_hz: float,
) -> bool:
    """Return whether a shadow sample is due under a monotonic rate limit."""

    if not math.isfinite(maximum_rate_hz) or maximum_rate_hz <= 0.0:
        raise ValueError("shadow maximum rate must be finite and greater than zero")
    if not math.isfinite(now_monotonic):
        return False
    if last_publish_monotonic is None:
        return True
    elapsed = now_monotonic - last_publish_monotonic
    return math.isfinite(elapsed) and elapsed >= (1.0 / maximum_rate_hz)
