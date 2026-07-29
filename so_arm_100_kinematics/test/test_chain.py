"""Validates so_arm_100_kinematics against real, hand-tuned hardware poses.

Run with either:
    python3 -m unittest discover -s test
    colcon test --packages-select so_arm_100_kinematics

Deliberately plain unittest (no pytest import required at collection time)
so this also runs inside Blender's bundled interpreter unmodified.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so_arm_100_kinematics.chain import Unreachable, fk, ik, tool_elevation_rad
from so_arm_100_kinematics.envelope import is_reachable, sweep_envelope

# The five joint-space waypoints tuned by hand in RViz against the real
# arm, verbatim from so_arm_100_pick_and_place/config/pick_and_place.yaml
# (degrees, CHAIN order). These are ground truth: this module must agree
# with them, not the other way around.
TUNED_POSES_DEG = {
    "home": (0.0, -99.98, 85.94, 71.62, 90.0),
    "pregrasp": (-92.0, 30.0, -4.0, -26.0, 90.0),
    "lower": (-93.0, 54.0, -7.0, -46.0, 90.0),
    "place": (0.0, 68.0, -17.0, -51.0, 90.0),
    "retreat": (0.0, 29.0, 63.0, -92.0, 90.0),
}


def _deg2rad(pose_deg):
    return tuple(math.radians(v) for v in pose_deg)


class TestForwardKinematics(unittest.TestCase):
    def test_lower_pose_matches_measured_stick_position(self):
        # ROS2_IMPLEMENTATION_PLAN.md Sec 9.3: the 'lower' pose's FK must
        # land within a few mm of stick.pose = [0.379, -0.026, 0.064] --
        # this is the number that validated the whole model against
        # hardware in the first place.
        pos, _rot = fk(_deg2rad(TUNED_POSES_DEG["lower"]))
        expected = (0.379, -0.026, 0.064)
        for got, want in zip(pos, expected):
            self.assertAlmostEqual(got, want, delta=0.006)

    def test_wrist_roll_does_not_move_the_tcp(self):
        base = list(_deg2rad(TUNED_POSES_DEG["place"]))
        pos_a, _ = fk(base)
        base[4] += math.radians(37.0)
        pos_b, _ = fk(base)
        for a, b in zip(pos_a, pos_b):
            self.assertAlmostEqual(a, b, places=9)

    def test_tool_elevation_identity_matches_tuned_poses(self):
        # place: 68 - 17 - 51 = 0 deg elevation.
        # home:  -99.98 + 85.94 + 71.62 = 57.58 -> elevation -57.58 deg.
        cases = {"place": 0.0, "home": -57.58}
        for name, expected_deg in cases.items():
            q = _deg2rad(TUNED_POSES_DEG[name])
            self.assertAlmostEqual(math.degrees(tool_elevation_rad(q)), expected_deg, places=1)


class TestInverseKinematics(unittest.TestCase):
    def test_round_trip_on_every_tuned_pose(self):
        # Strongest test: feed each tuned pose's own FK output back into ik()
        # and check the reconstructed joints reproduce the same TCP exactly.
        for name, pose_deg in TUNED_POSES_DEG.items():
            q = _deg2rad(pose_deg)
            target_pos, _rot = fk(q)
            elevation = tool_elevation_rad(q)
            for elbow_up in (True, False):
                try:
                    solved = ik(target_pos, elevation, elbow_up=elbow_up)
                except Unreachable:
                    continue
                solved_pos, _ = fk(solved)
                for got, want in zip(solved_pos, target_pos):
                    self.assertAlmostEqual(
                        got, want, delta=1e-5,
                        msg=f"{name} (elbow_up={elbow_up}) round-trip position mismatch",
                    )
                self.assertAlmostEqual(
                    tool_elevation_rad(solved), elevation, delta=1e-5,
                    msg=f"{name} (elbow_up={elbow_up}) round-trip elevation mismatch",
                )

    def test_recovers_the_actual_tuned_joint_angles(self):
        # Not just "some" solution -- the SAME solution the hardware was
        # tuned to, on the branch that matches (elbow_up picked per case).
        expected_branch = {
            "home": True, "pregrasp": True, "lower": True,
            "place": True, "retreat": True,
        }
        for name, pose_deg in TUNED_POSES_DEG.items():
            q_expected = _deg2rad(pose_deg)
            target_pos, _rot = fk(q_expected)
            elevation = tool_elevation_rad(q_expected)
            solved = ik(target_pos, elevation, elbow_up=expected_branch[name])
            for i, (got, want) in enumerate(zip(solved, q_expected)):
                self.assertAlmostEqual(
                    math.degrees(got), math.degrees(want), delta=0.5,
                    msg=f"{name} joint[{i}] mismatch",
                )

    def test_random_grid_round_trip(self):
        # Broader net: sample a grid of realistic vertical-stick targets and
        # confirm every ik() success round-trips through fk() exactly.
        count_checked = 0
        for x_mm in range(-100, 101, 25):
            for y_mm in range(-420, -290, 25):
                for z_mm in range(10, 180, 25):
                    target = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
                    try:
                        solved = ik(target, tool_elevation_target_rad=0.0, elbow_up=True)
                    except Unreachable:
                        continue
                    solved_pos, _ = fk(solved)
                    for got, want in zip(solved_pos, target):
                        self.assertAlmostEqual(got, want, delta=1e-5)
                    count_checked += 1
        self.assertGreater(count_checked, 20, "grid found suspiciously few reachable points")

    def test_unreachable_target_raises(self):
        with self.assertRaises(Unreachable):
            ik((5.0, 0.0, 0.0), tool_elevation_target_rad=0.0)

    def test_stick_roll_only_changes_wrist_roll(self):
        target = (0.0, -0.38, 0.10)
        q0 = ik(target, tool_elevation_target_rad=0.0, stick_roll_rad=0.0)
        q1 = ik(target, tool_elevation_target_rad=0.0, stick_roll_rad=math.radians(30.0))
        for i in range(4):
            self.assertAlmostEqual(q0[i], q1[i], places=9)
        self.assertAlmostEqual(q1[4] - q0[4], math.radians(30.0), places=9)


class TestEnvelope(unittest.TestCase):
    def test_lower_pose_target_is_reachable(self):
        ok, reason = is_reachable((0.379, -0.026, 0.064), tool_elevation_target_rad=0.0)
        self.assertTrue(ok, reason)

    def test_far_target_is_unreachable(self):
        ok, _reason = is_reachable((1.0, 0.0, 0.1), tool_elevation_target_rad=0.0)
        self.assertFalse(ok)

    def test_sweep_envelope_matches_known_ballpark(self):
        # Sanity check, not a re-derivation: at table height (z=0) with a
        # horizontal tool (vertical stick), the envelope should be a
        # plausible ring a few tens of cm out -- roughly matching the
        # hand-swept numbers in ROS2_IMPLEMENTATION_PLAN.md Sec 9.4 (that
        # sweep used a slightly different grasp-offset convention, so this
        # only checks the same ballpark, not exact agreement).
        result = sweep_envelope([0.0], tool_elevation_target_rad=0.0)
        min_r, max_r = result[0.0]
        self.assertGreater(min_r, 0.20)
        self.assertLess(min_r, 0.35)
        self.assertGreater(max_r, 0.35)
        self.assertLess(max_r, 0.50)


if __name__ == "__main__":
    unittest.main()
