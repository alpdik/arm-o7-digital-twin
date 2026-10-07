import unittest

from arm_o7_glove.mapping import (
    DEFAULT_CLOSED_RAD,
    GLOVE_FINGERS,
    HAND_JOINTS,
    OPEN_RAD,
    calibration_from_samples,
    curls_to_hand_targets,
    ease_angles,
    hand_to_ti5_angles,
    parse_5dt_report,
    raw_to_curl,
    step_toward,
)

# A real report captured from the DG05UR glove on 2026-10-07.
REPORT = bytes.fromhex(
    "035c035c0000081a081a0000052c052c0000072607260000"
    "05b905b9000000020000444730355552303239353330353130303530353130303"
    "5ffff0000000000"
)


class RawToCurl(unittest.TestCase):
    def test_range_and_clamp(self):
        self.assertEqual(raw_to_curl(1000, 1000, 2000), 0.0)
        self.assertEqual(raw_to_curl(1500, 1000, 2000), 0.5)
        self.assertEqual(raw_to_curl(2500, 1000, 2000), 1.0)
        self.assertEqual(raw_to_curl(500, 1000, 2000), 0.0)

    def test_inverted_sensor(self):
        self.assertEqual(raw_to_curl(2000, 2000, 1000), 0.0)
        self.assertEqual(raw_to_curl(1000, 2000, 1000), 1.0)

    def test_degenerate_span(self):
        self.assertEqual(raw_to_curl(1234, 1000, 1000), 0.0)


class Report(unittest.TestCase):
    def test_parse_captured_report(self):
        self.assertEqual(parse_5dt_report(REPORT), (860, 2074, 1324, 1830, 1465))

    def test_short_report(self):
        with self.assertRaises(ValueError):
            parse_5dt_report(b"\x00" * 10)


class HandTargets(unittest.TestCase):
    def test_open_and_closed(self):
        opened = curls_to_hand_targets({f: 0.0 for f in GLOVE_FINGERS})
        closed = curls_to_hand_targets({f: 1.0 for f in GLOVE_FINGERS})
        self.assertEqual(set(opened), set(HAND_JOINTS))
        for joint in HAND_JOINTS:
            self.assertAlmostEqual(opened[joint], OPEN_RAD)
        for joint, value in DEFAULT_CLOSED_RAD.items():
            self.assertAlmostEqual(closed[joint], value)
        # One thumb sensor: roll and yaw do not follow it.
        self.assertAlmostEqual(closed["thumb_cmc_roll"], OPEN_RAD)
        self.assertAlmostEqual(closed["thumb_cmc_yaw"], OPEN_RAD)

    def test_curl_is_clamped(self):
        wild = curls_to_hand_targets({f: 5.0 for f in GLOVE_FINGERS})
        self.assertAlmostEqual(wild["index_mcp_pitch"], DEFAULT_CLOSED_RAD["index_mcp_pitch"])

    def test_step_toward_limits_each_joint(self):
        stepped = step_toward({"a": 1.0, "b": 0.05, "c": -1.0}, {"a": 0.0, "b": 0.0, "c": 0.0}, 0.08)
        self.assertAlmostEqual(stepped["a"], 0.08)
        self.assertAlmostEqual(stepped["b"], 0.05)
        self.assertAlmostEqual(stepped["c"], -0.08)


class Ti5(unittest.TestCase):
    def test_open_maps_to_zero(self):
        angles = hand_to_ti5_angles({j: OPEN_RAD for j in HAND_JOINTS})
        self.assertEqual(sorted(angles), [1, 2, 3, 4, 5, 6])
        self.assertTrue(all(a == 0 for a in angles.values()))

    def test_fist_and_clamp(self):
        positions = {j: OPEN_RAD for j in HAND_JOINTS}
        positions["index_mcp_pitch"] = 1.3607  # model upper limit
        positions["pinky_mcp_pitch"] = 9.0     # out of range
        positions["thumb_cmc_pitch"] = -1.0    # out of range
        angles = hand_to_ti5_angles(positions)
        self.assertEqual(angles[4], 90)
        self.assertEqual(angles[1], 90)
        self.assertEqual(angles[5], 0)

    def test_ease(self):
        eased = ease_angles({1: 0, 2: 50, 3: 10}, {1: 90, 2: 0, 3: 11}, 3)
        self.assertEqual(eased, {1: 3, 2: 47, 3: 11})


class Calibration(unittest.TestCase):
    FINGERS = ("thumb", "index")

    def test_medians_in_finger_order(self):
        opened = {"thumb": [1000, 1010, 990], "index": [2100, 2110, 2105]}
        closed = {"thumb": [1950, 1960, 1940], "index": [2850, 2870, 2860]}
        self.assertEqual(
            calibration_from_samples(opened, closed, self.FINGERS),
            ([1000, 2105], [1950, 2860]),
        )

    def test_inverted_sensor_is_fine(self):
        low, high = calibration_from_samples(
            {"thumb": [2000], "index": [2000]}, {"thumb": [1000], "index": [1000]}, self.FINGERS
        )
        self.assertEqual((low, high), ([2000, 2000], [1000, 1000]))

    def test_unbent_and_missing_fingers_are_named(self):
        with self.assertRaises(ValueError) as caught:
            calibration_from_samples({"thumb": [1000]}, {"thumb": [1050]}, self.FINGERS)
        message = str(caught.exception)
        self.assertIn("thumb barely changed", message)
        self.assertIn("index: no glove data", message)


if __name__ == "__main__":
    unittest.main()
