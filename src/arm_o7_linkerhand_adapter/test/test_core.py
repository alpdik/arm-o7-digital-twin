import math
import unittest

from arm_o7_linkerhand_adapter.core import (
    ARM_JOINTS,
    HAND_JOINTS,
    AdapterError,
    JointMap,
    canonical_to_sdk,
    merge_joint_states,
    sample_trajectory,
    sdk_to_canonical,
    validate_trajectory,
)


def maps():
    return tuple(
        JointMap(name, index, (-1.0 if index == 1 else 2.0), 0.1, 0.0, 2.0)
        for index, name in enumerate(HAND_JOINTS)
    )


class MappingTests(unittest.TestCase):
    def test_round_trip_reordered_vector(self):
        canonical = tuple(0.1 * (index + 1) for index in range(7))
        sdk = canonical_to_sdk(HAND_JOINTS, canonical, maps())
        recovered = sdk_to_canonical(sdk, maps())
        for actual, expected in zip(recovered, canonical):
            self.assertAlmostEqual(actual, expected)

    def test_names_can_arrive_in_any_order(self):
        names = tuple(reversed(HAND_JOINTS))
        positions = tuple(float(index) / 10.0 for index in range(7))
        sdk = canonical_to_sdk(names, positions, maps())
        by_name = dict(zip(names, positions))
        self.assertAlmostEqual(sdk[0], 0.1 + 2.0 * by_name[HAND_JOINTS[0]])

    def test_incomplete_command_rejected(self):
        with self.assertRaises(AdapterError):
            canonical_to_sdk(HAND_JOINTS[:-1], [0.0] * 6, maps())

    def test_nonfinite_and_limit_rejected(self):
        for bad in (math.nan, math.inf, 2.1):
            values = [0.0] * 7
            values[2] = bad
            with self.assertRaises(AdapterError):
                canonical_to_sdk(HAND_JOINTS, values, maps())


class TrajectoryTests(unittest.TestCase):
    def test_validation_and_interpolation(self):
        points = validate_trajectory(
            HAND_JOINTS,
            [([0.0] * 7, 0.0), ([1.0] * 7, 1.0)],
            maps(),
            2.0,
        )
        sampled, finished = sample_trajectory(points, 0.25)
        self.assertFalse(finished)
        self.assertEqual(sampled, (0.25,) * 7)

    def test_initial_state_used_before_delayed_first_point(self):
        sampled, finished = sample_trajectory(
            [((1.0,) * 7, 1.0)], 0.5, initial_positions=(0.0,) * 7
        )
        self.assertFalse(finished)
        self.assertEqual(sampled, (0.5,) * 7)

    def test_bad_timing_rejected(self):
        with self.assertRaises(AdapterError):
            validate_trajectory(
                HAND_JOINTS,
                [([0.0] * 7, 1.0), ([0.1] * 7, 1.0)],
                maps(),
                2.0,
            )

    def test_validation_normalizes_joint_order(self):
        names = tuple(reversed(HAND_JOINTS))
        values = tuple(float(index) / 10.0 for index in range(7))
        points = validate_trajectory(names, [(values, 0.0)], maps(), 2.0)
        by_name = dict(zip(names, values))
        self.assertEqual(points[0][0], tuple(by_name[name] for name in HAND_JOINTS))


class MuxTests(unittest.TestCase):
    def test_merge_has_canonical_order(self):
        arm = {name: float(index) for index, name in enumerate(ARM_JOINTS)}
        hand = {name: float(index + 6) for index, name in enumerate(HAND_JOINTS)}
        names, positions = merge_joint_states(arm, hand)
        self.assertEqual(names, ARM_JOINTS + HAND_JOINTS)
        self.assertEqual(positions, tuple(float(index) for index in range(13)))

    def test_incomplete_merge_rejected(self):
        with self.assertRaises(AdapterError):
            merge_joint_states({}, {})


if __name__ == "__main__":
    unittest.main()
