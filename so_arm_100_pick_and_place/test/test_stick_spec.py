"""Validates stick_spec.py -- Sec 11 Phase 3's parametric place path.

Run with:
    python3 -m unittest discover -s test
    colcon test --packages-select so_arm_100_pick_and_place

Deliberately plain unittest, matching so_arm_100_kinematics/test's own
convention.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import so_arm_100_kinematics as sak
from so_arm_100_pick_and_place.stick_spec import solve_stick_spec_joints


class TestSolveStickSpecJoints(unittest.TestCase):
    def test_recovers_the_tuned_place_pose(self):
        # pick_and_place.yaml's 'place' step: [0, 68, -17, -51, 90] deg.
        # Its FK grip point, offset down by GRASP_OFFSET_M along the
        # (vertical, tool_elevation~0) stick axis, gives a base/tip pair
        # that should round-trip back to the same tuned joint values --
        # the exact "Done when" Sec 11 Phase 3 asks for.
        place_deg = [0.0, 68.0, -17.0, -51.0, 90.0]
        place_rad = [math.radians(d) for d in place_deg]
        grip, _rot = sak.fk(place_rad)
        base = (grip[0], grip[1], grip[2] - sak.GRASP_OFFSET_M)
        tip = (grip[0], grip[1], grip[2] - sak.GRASP_OFFSET_M + 0.110)

        joints = solve_stick_spec_joints(base, tip)

        for solved_rad, expected_deg in zip(joints, place_deg):
            self.assertAlmostEqual(math.degrees(solved_rad), expected_deg, delta=0.5)

    def test_flip_retry_recovers_a_reversed_assignment(self):
        # Reconstructed from the exact regression target/axis in
        # so_arm_100_kinematics/test/test_grasp.py's own
        # test_wrist_roll_asymmetric_limit_makes_one_end_assignment_fail --
        # same known real case, not a freshly-invented one that might not
        # actually exercise the asymmetric Wrist_Roll limit (Sec 9.6).
        base = (-0.04716134554129445, -0.37227827310562134, 0.1132485403210376)
        tip = (0.06283865445870511, -0.37227827310562134, 0.11324855016892073)

        # Direct (un-flipped) solve fails -- confirms this really is that case.
        target = sak.grasp_target(base, tip)
        axis = tuple(t - b for t, b in zip(tip, base))
        self.assertIsNone(sak.solve_stick_orientation(target, axis))

        # solve_stick_spec_joints must still succeed, via the flip retry.
        joints = solve_stick_spec_joints(base, tip)
        self.assertEqual(len(joints), 5)

    def test_roll_deg_changes_only_wrist_roll(self):
        # A small roll_deg (20 deg, well inside Wrist_Roll's headroom for
        # this direction) should change ONLY Wrist_Roll, leaving the other
        # four joints untouched. Deliberately not a large roll_deg here --
        # see test_large_roll_deg_can_trigger_the_flip_retry below for why.
        base, tip = (0.0, -0.36, 0.0), (0.0, -0.36, 0.110)
        joints0 = solve_stick_spec_joints(base, tip, roll_deg=0.0)
        joints20 = solve_stick_spec_joints(base, tip, roll_deg=20.0)
        for i in range(4):
            self.assertAlmostEqual(joints0[i], joints20[i], places=6)
        self.assertAlmostEqual(math.degrees(joints20[4] - joints0[4]), 20.0, places=3)

    def test_large_roll_deg_can_trigger_the_flip_retry(self):
        # roll_deg=90 on this same direction pushes the direct (base->tip)
        # attempt's Wrist_Roll to 180 deg -- outside its +-2.75 rad limit --
        # so solve_stick_spec_joints must fall back to the tip->base
        # attempt: a genuinely different elevation/elbow branch (grasp_target
        # offsets from whichever end is passed as "base"), not just a
        # different Wrist_Roll. This is the flip-retry working as intended,
        # not a bug -- confirmed here so it isn't "fixed" by accident later
        # into assuming roll_deg never changes anything but Wrist_Roll.
        # (Shoulder_Rotation itself happens to stay put for this particular
        # direction -- it's a vertical stick, and flipping base/tip only
        # moves the grip point up/down, not sideways -- so the other three
        # joints are what actually show the flip.)
        base, tip = (0.0, -0.36, 0.0), (0.0, -0.36, 0.110)
        joints0 = solve_stick_spec_joints(base, tip, roll_deg=0.0)
        joints90 = solve_stick_spec_joints(base, tip, roll_deg=90.0)
        changed = any(abs(joints0[i] - joints90[i]) > math.radians(1.0) for i in (1, 2, 3))
        self.assertTrue(changed, "expected the flip retry to land on a different elevation/elbow branch")

    def test_real_build_file_stick_is_solvable(self):
        # s_004 from the user's real Blender-exported build file
        # (SO100BlenderTest.build.build.build.json) -- a real design's
        # placement, not a synthetic one.
        joints = solve_stick_spec_joints(
            (-0.047115, -0.372278, 0.0), (-0.050317, -0.372278, 0.109952))
        self.assertEqual(len(joints), 5)

    def test_unreachable_both_ways_raises(self):
        with self.assertRaises(sak.Unreachable):
            solve_stick_spec_joints((10.0, 10.0, 10.0), (10.0, 10.0, 10.11))


if __name__ == "__main__":
    unittest.main()
