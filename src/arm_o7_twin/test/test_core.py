import math
import unittest

from arm_o7_twin.core import (
    ARM_JOINTS,
    CANONICAL_JOINTS,
    HAND_JOINTS,
    JointLimit,
    Mode,
    SafetyConfig,
    TrajectoryData,
    TrajectoryPointData,
    ValidationError,
    default_joint_limits,
    evaluate_command,
    make_joint_state_snapshot,
    make_shadow_trajectory,
    shadow_publish_due,
    split_trajectory,
    validate_trajectory,
)


def point(positions, seconds=1.0, velocities=()):
    return TrajectoryPointData(
        positions=tuple(positions),
        velocities=tuple(velocities),
        time_from_start_ns=int(seconds * 1_000_000_000),
    )


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.config = SafetyConfig(default_joint_limits())
        self.trajectory = TrajectoryData(
            ARM_JOINTS,
            (
                point((0.05,) * len(ARM_JOINTS)),
                point((0.10,) * len(ARM_JOINTS), seconds=2.0),
            ),
        )
        self.sim = make_joint_state_snapshot(CANONICAL_JOINTS, [0.0] * 13, 10.0)
        self.real = make_joint_state_snapshot(CANONICAL_JOINTS, [0.0] * 13, 10.0)

    def evaluate(self, mode=Mode.SIM_ONLY, **changes):
        values = dict(
            mode=mode,
            trajectory=self.trajectory,
            config=self.config,
            now_monotonic=10.1,
            deadman_pressed=True,
            deadman_monotonic=10.0,
            heartbeat_monotonic=10.0,
            sim_state=self.sim,
            real_state=self.real,
            enable_real_output=False,
        )
        values.update(changes)
        return evaluate_command(**values)

    def test_mode_parser_is_strict_but_user_friendly(self):
        self.assertEqual(Mode.parse("twin-command"), Mode.TWIN_COMMAND)
        with self.assertRaises(ValueError):
            Mode.parse("dangerous_bypass")

    def test_valid_sim_command_is_accepted(self):
        decision = self.evaluate()
        self.assertTrue(decision.accepted)
        self.assertTrue(decision.route_sim)
        self.assertFalse(decision.route_real)

    def test_safe_idle_never_routes(self):
        decision = self.evaluate(mode=Mode.SAFE_IDLE)
        self.assertFalse(decision.accepted)
        self.assertFalse(decision.route_sim)
        self.assertFalse(decision.route_real)

    def test_real_modes_require_explicit_enable(self):
        for mode in (Mode.TWIN_COMMAND, Mode.REAL_ONLY):
            with self.subTest(mode=mode):
                decision = self.evaluate(mode=mode)
                self.assertFalse(decision.accepted)
                self.assertIn("disabled", decision.reason)

    def test_explicitly_enabled_real_routes_are_correct(self):
        twin = self.evaluate(mode=Mode.TWIN_COMMAND, enable_real_output=True)
        self.assertTrue(twin.accepted)
        self.assertTrue(twin.route_sim)
        self.assertTrue(twin.route_real)

        real = self.evaluate(mode=Mode.REAL_ONLY, enable_real_output=True)
        self.assertTrue(real.accepted)
        self.assertFalse(real.route_sim)
        self.assertTrue(real.route_real)

    def test_deadman_and_heartbeat_are_fail_closed(self):
        self.assertIn(
            "deadman", self.evaluate(deadman_pressed=False).reason
        )
        self.assertIn(
            "stale", self.evaluate(deadman_monotonic=9.0).reason
        )
        self.assertIn(
            "heartbeat", self.evaluate(heartbeat_monotonic=None).reason
        )
        self.assertIn(
            "stale", self.evaluate(heartbeat_monotonic=9.0).reason
        )

    def test_required_joint_state_must_be_fresh_and_complete(self):
        stale = make_joint_state_snapshot(
            CANONICAL_JOINTS, [0.0] * len(CANONICAL_JOINTS), 9.0
        )
        self.assertIn("stale", self.evaluate(sim_state=stale).reason)

        partial = make_joint_state_snapshot((ARM_JOINTS[0],), (0.0,), 10.0)
        decision = self.evaluate(sim_state=partial)
        self.assertFalse(decision.accepted)
        self.assertIn(ARM_JOINTS[1], decision.reason)

    def test_rejects_unknown_duplicate_nonfinite_and_out_of_limit(self):
        cases = [
            TrajectoryData(("unknown",), (point((0.0,)),)),
            TrajectoryData((ARM_JOINTS[0], ARM_JOINTS[0]), (point((0.0, 0.0)),)),
            TrajectoryData((ARM_JOINTS[0],), (point((math.nan,)),)),
            TrajectoryData((ARM_JOINTS[0],), (point((4.0,)),)),
        ]
        for trajectory in cases:
            with self.subTest(trajectory=trajectory):
                with self.assertRaises(ValidationError):
                    validate_trajectory(trajectory, self.config)

    def test_rejects_bad_vector_lengths_and_time_order(self):
        trajectory = TrajectoryData(
            (ARM_JOINTS[0], HAND_JOINTS[0]),
            (
                point((0.0,), seconds=1.0, velocities=(0.0,)),
                point((0.0, 0.0), seconds=1.0),
            ),
        )
        with self.assertRaises(ValidationError) as context:
            validate_trajectory(trajectory, self.config)
        self.assertGreaterEqual(len(context.exception.issues), 3)

    def test_start_state_mismatch_interlocks(self):
        decision = self.evaluate(
            trajectory=TrajectoryData(
                ARM_JOINTS,
                (point((1.0,) + (0.0,) * (len(ARM_JOINTS) - 1)),),
            )
        )
        self.assertFalse(decision.accepted)
        self.assertIn("start-state mismatch", decision.reason)

    def test_twin_tracking_error_interlocks(self):
        real = make_joint_state_snapshot(
            CANONICAL_JOINTS,
            [0.6] + [0.0] * 12,
            10.0,
        )
        config = SafetyConfig(
            default_joint_limits(),
            start_state_tolerance_rad=0.5,
            tracking_error_tolerance_rad=0.2,
        )
        decision = self.evaluate(
            mode=Mode.TWIN_COMMAND,
            enable_real_output=True,
            real_state=real,
            config=config,
            trajectory=TrajectoryData(
                ARM_JOINTS,
                (point((0.3,) + (0.0,) * (len(ARM_JOINTS) - 1)),),
            ),
        )
        self.assertFalse(decision.accepted)
        self.assertIn("tracking error", decision.reason)

    def test_split_preserves_time_and_optional_vectors(self):
        trajectory = TrajectoryData(
            (HAND_JOINTS[0], ARM_JOINTS[1], ARM_JOINTS[0]),
            (
                TrajectoryPointData(
                    positions=(3.0, 2.0, 1.0),
                    velocities=(0.3, 0.2, 0.1),
                    time_from_start_ns=123,
                ),
            ),
        )
        arm = split_trajectory(trajectory, ARM_JOINTS)
        hand = split_trajectory(trajectory, HAND_JOINTS)
        self.assertEqual(arm.joint_names, (ARM_JOINTS[0], ARM_JOINTS[1]))
        self.assertEqual(arm.points[0].positions, (1.0, 2.0))
        self.assertEqual(arm.points[0].velocities, (0.1, 0.2))
        self.assertEqual(arm.points[0].time_from_start_ns, 123)
        self.assertEqual(hand.points[0].positions, (3.0,))

    def test_custom_limits_are_enforced(self):
        limits = default_joint_limits()
        limits[ARM_JOINTS[0]] = JointLimit(-0.1, 0.1)
        config = SafetyConfig(limits)
        trajectory = TrajectoryData(
            ARM_JOINTS,
            (point((0.2,) + (0.0,) * (len(ARM_JOINTS) - 1)),),
        )
        with self.assertRaises(ValidationError):
            validate_trajectory(trajectory, config)

    def test_partial_controller_group_is_rejected_by_default(self):
        trajectory = TrajectoryData((HAND_JOINTS[0],), (point((0.1,)),))
        with self.assertRaises(ValidationError) as context:
            validate_trajectory(trajectory, self.config)
        self.assertIn("partial hand controller group", str(context.exception))

    def test_shadow_state_becomes_short_complete_trajectory(self):
        positions = [index / 100.0 for index in range(len(CANONICAL_JOINTS))]
        snapshot = make_joint_state_snapshot(CANONICAL_JOINTS, positions, 10.0)
        trajectory = make_shadow_trajectory(snapshot, 0.1, self.config)
        self.assertEqual(trajectory.joint_names, CANONICAL_JOINTS)
        self.assertEqual(trajectory.points[0].positions, tuple(positions))
        self.assertEqual(trajectory.points[0].time_from_start_ns, 100_000_000)

    def test_shadow_rejects_partial_nonfinite_and_out_of_limit_state(self):
        partial = make_joint_state_snapshot((ARM_JOINTS[0],), (0.0,), 10.0)
        with self.assertRaises(ValidationError):
            make_shadow_trajectory(partial, 0.1, self.config)

        with self.assertRaises(ValidationError):
            make_joint_state_snapshot(
                CANONICAL_JOINTS, [math.inf] * len(CANONICAL_JOINTS), 10.0
            )

        outside = make_joint_state_snapshot(
            CANONICAL_JOINTS,
            [4.0] + [0.0] * (len(CANONICAL_JOINTS) - 1),
            10.0,
        )
        with self.assertRaises(ValidationError):
            make_shadow_trajectory(outside, 0.1, self.config)

    def test_shadow_rate_limit_is_monotonic(self):
        self.assertTrue(shadow_publish_due(None, 10.0, 20.0))
        self.assertFalse(shadow_publish_due(10.0, 10.049, 20.0))
        self.assertTrue(shadow_publish_due(10.0, 10.05, 20.0))
        self.assertFalse(shadow_publish_due(10.0, 9.0, 20.0))

    def test_shadow_decision_routes_only_to_sim(self):
        decision = self.evaluate(mode=Mode.SHADOW)
        self.assertTrue(decision.accepted)
        self.assertTrue(decision.route_sim)
        self.assertFalse(decision.route_real)


if __name__ == "__main__":
    unittest.main()
