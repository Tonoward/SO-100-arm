"""Validates jaw_clearance.py -- the swept-jaw collision pre-filter.

Run with either:
    python3 -m unittest discover -s test
    colcon test --packages-select so_arm_100_kinematics

Deliberately plain unittest, matching test_chain.py/test_grasp.py's own
convention so this also runs inside Blender's bundled interpreter unmodified.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so_arm_100_kinematics.constants import (
    GRASP_OFFSET_M,
    JAW_RADIUS_M,
    STICK_COLLISION_RADIUS_M,
)
from so_arm_100_kinematics.jaw_clearance import (
    check_jaw_clearance,
    jaw_swept_capsule,
    segment_distance,
)


class TestSegmentDistance(unittest.TestCase):
    def test_parallel_segments(self):
        # Two unit-length segments, 1 m apart, running side by side.
        d = segment_distance((0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_perpendicular_skew_segments(self):
        # Classic skew case: one along X at z=0, one along Y at z=1,
        # both passing near the origin -- closest distance is the 1 m gap.
        d = segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 1), (0, 1, 1))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_intersecting_segments_zero_distance(self):
        d = segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
        self.assertAlmostEqual(d, 0.0, places=9)

    def test_degenerate_point_segments(self):
        # Both "segments" are single points (zero length).
        d = segment_distance((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0))
        self.assertAlmostEqual(d, 5.0, places=9)

    def test_point_to_segment(self):
        # A point 2 m off the middle of a segment running along X.
        d = segment_distance((0, 2, 0), (0, 2, 0), (-5, 0, 0), (5, 0, 0))
        self.assertAlmostEqual(d, 2.0, places=9)

    def test_closest_approach_beyond_endpoint_clamps(self):
        # Segment 2 sits entirely "past the end" of segment 1 -- the
        # clamped algorithm must use segment 1's endpoint, not its
        # infinite-line projection.
        d = segment_distance((0, 0, 0), (1, 0, 0), (3, 0, 1), (4, 0, 1))
        expected = (2.0 ** 2 + 1.0 ** 2) ** 0.5  # endpoint (1,0,0) to (3,0,1)
        self.assertAlmostEqual(d, expected, places=9)


class TestJawSweptCapsule(unittest.TestCase):
    def test_endpoints_are_grip_point_and_base(self):
        base = (0.0, -0.36, 0.0)
        tip = (0.0, -0.36, 0.110)
        grip, vertex = jaw_swept_capsule(base, tip)
        self.assertEqual(vertex, base)
        # Grip point sits GRASP_OFFSET_M above base, toward tip.
        self.assertAlmostEqual(grip[2], GRASP_OFFSET_M, places=9)
        self.assertAlmostEqual(grip[0], base[0], places=9)
        self.assertAlmostEqual(grip[1], base[1], places=9)


class TestCheckJawClearance(unittest.TestCase):
    BASE = (0.0, -0.36, 0.0)
    TIP = (0.0, -0.36, 0.110)

    def test_empty_scene_is_clear(self):
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, [])
        self.assertTrue(clear)
        self.assertIsNone(reason)
        self.assertIsNone(clearance_m)
        self.assertIsNone(index)

    def test_distant_stick_is_clear(self):
        far_away = [((0.30, -0.20, 0.0), (0.30, -0.20, 0.100))]
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, far_away)
        self.assertTrue(clear)
        self.assertIsNone(reason)
        self.assertEqual(index, 0)
        self.assertGreater(clearance_m, 0.0)

    def test_stick_crowding_the_joint_cone_is_blocked(self):
        # A stick running right alongside the jaw's own capsule, well
        # within JAW_RADIUS_M + STICK_COLLISION_RADIUS_M of it, but NOT
        # sharing this joint's vertex -- exactly the "some other stick
        # crowding into the same space" case Sec 8.2 warns about.
        offset = (JAW_RADIUS_M + STICK_COLLISION_RADIUS_M) * 0.3
        crowding = [((offset, -0.36, 0.0), (offset, -0.36, 0.110))]
        clear, reason, clearance_m, index = check_jaw_clearance(self.BASE, self.TIP, crowding)
        self.assertFalse(clear)
        self.assertIsNotNone(reason)
        self.assertEqual(index, 0)
        self.assertLess(clearance_m, 0.0)

    def test_tightest_of_several_sticks_is_reported(self):
        far_away = ((0.30, -0.20, 0.0), (0.30, -0.20, 0.100))
        offset = (JAW_RADIUS_M + STICK_COLLISION_RADIUS_M) * 0.3
        crowding = ((offset, -0.36, 0.0), (offset, -0.36, 0.110))
        clear, reason, clearance_m, index = check_jaw_clearance(
            self.BASE, self.TIP, [far_away, crowding])
        self.assertFalse(clear)
        self.assertEqual(index, 1)

    def test_caller_must_exclude_the_actual_joint_neighbour(self):
        # A stick sharing this exact joint's vertex (e.g. the neighbour
        # this new stick glues onto) reads as a severe "collision" if
        # naively included -- that is expected (it's the joint), not a
        # jaw-clearance violation. The module docstring says this is the
        # caller's responsibility, not something to guess here; this test
        # documents that boundary so it isn't "fixed" by accident later.
        joint_neighbour = [((0.0, -0.36, 0.0), (0.0, -0.46, 0.0))]
        clear, reason, clearance_m, index = check_jaw_clearance(
            self.BASE, self.TIP, joint_neighbour)
        self.assertFalse(clear)  # naive result: looks blocked
        self.assertEqual(index, 0)


if __name__ == "__main__":
    unittest.main()
