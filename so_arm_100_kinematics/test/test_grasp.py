"""Validates grasp.py -- the grasp-orientation transform.

Run with either:
    python3 -m unittest discover -s test
    colcon test --packages-select so_arm_100_kinematics

Deliberately plain unittest (no pytest import required at collection time)
so this also runs inside Blender's bundled interpreter unmodified, matching
test_chain.py's own convention.

Most cases here are not synthetic: they are the exact targets/directions
that surfaced real bugs while this module was built (as ``core/validate.py``
inside the Blender addon, before being promoted here) -- an ill-conditioned
elevation branch, a too-tight anchor tolerance, and an unhandled asymmetric
Wrist_Roll limit. Reusing the exact numbers means a regression here is a
regression in the same case that broke a real user's design, not a
freshly-invented one that might not actually exercise the bug.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so_arm_100_kinematics.chain import Unreachable, fk, ik, tool_axis
from so_arm_100_kinematics.constants import CHAIN, GRASP_OFFSET_M
from so_arm_100_kinematics.grasp import (
    ANCHOR_TOLERANCE_DEG,
    azimuth_frame,
    grasp_target,
    solve_stick_orientation,
    solve_stick_placement,
    stick_axis,
)

SHOULDER = CHAIN[0][1]


def _achieved_error_deg(grip_dir, achieved):
    cos_err = max(-1.0, min(1.0, sum(a * b for a, b in zip(grip_dir, achieved))))
    return math.degrees(math.acos(cos_err))


def _normalize(vector):
    length = math.sqrt(sum(c * c for c in vector))
    return tuple(c / length for c in vector)


class TestGraspTarget(unittest.TestCase):
    def test_offset_along_the_stick_axis_toward_the_tip(self):
        base, tip = (0.0, -0.36, 0.0), (0.0, -0.36, 0.110)
        target = grasp_target(base, tip)
        self.assertAlmostEqual(target[0], 0.0, places=9)
        self.assertAlmostEqual(target[1], -0.36, places=9)
        self.assertAlmostEqual(target[2], GRASP_OFFSET_M, places=9)

    def test_matches_how_grasp_offset_m_was_itself_derived(self):
        # constants.py: GRASP_OFFSET_M = (tuned 'lower' pose TCP height) -
        # (feeder stick's base height), for a VERTICAL stick. This formula
        # must reproduce that relationship exactly, as its own special case.
        base_z = 0.014
        target = grasp_target((0.0, 0.0, base_z), (0.0, 0.0, base_z + 0.100))
        self.assertAlmostEqual(target[2], base_z + GRASP_OFFSET_M, places=9)

    def test_works_for_a_tilted_stick_too(self):
        base, tip = (0.0, -0.36, 0.0), (0.06, -0.42, 0.08)
        target = grasp_target(base, tip)
        length = math.dist(base, tip)
        distance_from_base = math.dist(base, target)
        self.assertAlmostEqual(distance_from_base, GRASP_OFFSET_M, places=9)
        # target must lie exactly on the base-tip line
        axis = tuple((t - b) / length for b, t in zip(base, tip))
        expected = tuple(b + a * GRASP_OFFSET_M for b, a in zip(base, axis))
        for got, want in zip(target, expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_zero_length_stick_is_rejected(self):
        with self.assertRaises(ValueError):
            grasp_target((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


class TestAzimuthFrame(unittest.TestCase):
    def test_on_axis_target_returns_none(self):
        self.assertIsNone(azimuth_frame(SHOULDER))

    def test_frame_is_orthonormal(self):
        r_hat, z_hat, tan_hat = azimuth_frame((0.30, -0.20, 0.10))
        for vector in (r_hat, z_hat, tan_hat):
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in vector)), 1.0,
                                   places=9)
        self.assertAlmostEqual(sum(a * b for a, b in zip(r_hat, tan_hat)), 0.0,
                               places=9)
        self.assertAlmostEqual(sum(a * b for a, b in zip(r_hat, z_hat)), 0.0,
                               places=9)


class TestStickAxis(unittest.TestCase):
    """Finding 1: this points grip -> base, confirmed against the five
    hand-tuned hardware poses (all known-vertical sticks)."""

    # Same waypoints as test_chain.py's TUNED_POSES_DEG. Only pregrasp,
    # lower, place and retreat actually hold a stick vertically in the
    # gripper -- "home" is the idle/folded rest stance the arm returns to
    # with an EMPTY gripper, so its stick_axis is meaningless and excluded.
    TUNED_POSES_DEG = {
        "pregrasp": (-92.0, 30.0, -4.0, -26.0, 90.0),
        "lower": (-93.0, 54.0, -7.0, -46.0, 90.0),
        "place": (0.0, 68.0, -17.0, -51.0, 90.0),
        "retreat": (0.0, 29.0, 63.0, -92.0, 90.0),
    }

    def test_tuned_poses_give_a_vertical_stick_axis(self):
        for name, pose_deg in self.TUNED_POSES_DEG.items():
            q = tuple(math.radians(v) for v in pose_deg)
            axis = stick_axis(q)
            self.assertAlmostEqual(abs(axis[2]), 1.0, delta=0.02, msg=name)

    def test_it_points_toward_negative_z_not_positive(self):
        # The sign that matters: grip-to-base is DOWNWARD (the base sits
        # below the grip point), not upward.
        q = tuple(math.radians(v) for v in self.TUNED_POSES_DEG["place"])
        axis = stick_axis(q)
        self.assertLess(axis[2], 0.0)

    def test_always_perpendicular_to_tool_axis(self):
        for pose_deg in self.TUNED_POSES_DEG.values():
            q = tuple(math.radians(v) for v in pose_deg)
            axis, tool = stick_axis(q), tool_axis(q)
            dot = sum(a * b for a, b in zip(axis, tool))
            self.assertAlmostEqual(dot, 0.0, places=9)


class TestSolveOrientationIsSelfVerifying(unittest.TestCase):
    """The module's central claim: every accepted candidate is verified by
    round-tripping through fk(), not just derived."""

    def test_a_vertical_stick_solves_essentially_exactly(self):
        # The 'lower' tuned pose's own TCP position -- hardware-measured
        # (test_chain.py's test_lower_pose_matches_measured_stick_position),
        # so this is a real, confirmed-reachable point, not a guess.
        target = (0.379, -0.026, 0.064)
        solved = solve_stick_orientation(target, (0.0, 0.0, 1.0))
        self.assertIsNotNone(solved)
        _elevation, _roll, _elbow_up, error_deg = solved
        self.assertLess(error_deg, 0.01)

    def test_solutions_reproduce_the_target_direction_on_replay(self):
        cases = [
            ((0.0, -0.36, 0.0), (0.0, -0.36, 0.110)),
            ((0.30, -0.05, 0.0), (0.30, 0.03, 0.0)),
            ((0.25, -0.30, 0.05), (0.31, -0.36, 0.13)),
        ]
        for base, tip in cases:
            target = grasp_target(base, tip)
            axis = tuple(t - b for t, b in zip(tip, base))
            solved = solve_stick_orientation(target, axis)
            if solved is None:
                continue  # some are legitimately unreachable
            elevation, roll, elbow_up, _error_deg = solved
            q = ik(target, elevation, stick_roll_rad=roll, elbow_up=elbow_up)
            _pos, rot = fk(q)
            achieved = (rot[0][2], rot[1][2], rot[2][2])
            grip_dir = _normalize(tuple(-c for c in axis))
            self.assertLess(_achieved_error_deg(grip_dir, achieved), 0.1,
                            msg="base=%s tip=%s" % (base, tip))


class TestRealWorldCases(unittest.TestCase):
    """Exact targets/directions that surfaced real bugs while this module
    was built inside the Blender addon, before promotion here. Each
    docstring names which finding it regression-tests."""

    def test_the_disputed_out_of_plane_tilt_is_reachable(self):
        # Finding: Sec 9.4's table originally called this categorically
        # unreachable. It is not -- this is the exact case that disproved
        # it: ik() at elevation=0, roll=-45deg succeeds directly.
        q = ik((0.30, -0.0452, 0.10), tool_elevation_target_rad=0.0,
              stick_roll_rad=math.radians(-45.0))
        _pos, rot = fk(q)
        grip_dir = (rot[0][2], rot[1][2], rot[2][2])
        base = (0.30, -0.0452, 0.10)
        target = tuple(b + GRASP_OFFSET_M * g for b, g in zip(base, grip_dir))
        stick_base = tuple(t + GRASP_OFFSET_M * g for t, g in zip(target, grip_dir))
        stick_tip = tuple(t - GRASP_OFFSET_M * g for t, g in zip(target, grip_dir))
        solved = solve_stick_orientation(
            target, tuple(t - b for t, b in zip(stick_tip, stick_base)))
        self.assertIsNotNone(solved)

    def test_a_071_degree_off_tangential_stick_still_solves(self):
        # Finding 2 (ill-conditioning): a real inverted-U's top stick, after
        # mesh-expansion solving, was 0.71deg off exactly tangential --
        # ordinary geometric noise, not a design intent. The naive
        # exact-root formula swung ~90deg onto an unreachable branch; the
        # natural elevation=0 pose (a cardinal anchor) reaches it fine.
        target = (0.004025880499069927, -0.37, 0.1132008160463194)
        axis = (-0.9999999996560391, 0.0, 2.6228260948088245e-05)
        solved = solve_stick_orientation(target, axis)
        self.assertIsNotNone(solved, "the near-tangential case must still solve")
        elevation, _roll, _elbow_up, _error_deg = solved
        self.assertLess(min(abs(elevation), abs(elevation - 2 * math.pi)),
                        math.radians(5.0))

    def test_a_2077_degree_case_needs_the_full_anchor_tolerance(self):
        # Finding: a second real mesh needed slightly more slack than the
        # first (2.077 > an earlier, too-tight 2.0 deg tolerance). The
        # EXACT mathematical root for this tilt is genuinely unreachable
        # (confirmed directly below) -- accepting the near-tangential
        # anchor's small residual is correct, not a bug to chase further.
        target = (0.011862862055960405, -0.37227827310562134, 0.11324854488687432)
        axis = (-0.9999999000904604, 0.0, 0.0004470112631756867)

        solved = solve_stick_orientation(target, axis)
        self.assertIsNotNone(solved)
        elevation, _roll, _elbow_up, error_deg = solved
        self.assertLess(min(abs(elevation), abs(elevation - 2 * math.pi)),
                        math.radians(5.0))
        self.assertGreater(error_deg, 2.0)  # genuinely needed the wider tolerance
        self.assertLess(error_deg, ANCHOR_TOLERANCE_DEG)

        # The exact root really is unreachable here -- confirms accepting
        # the anchor is correct, not a search bug papering over one.
        grip_dir = _normalize(tuple(-c for c in axis))
        r_hat, _z, _tan = azimuth_frame(target)
        a_r = sum(g * r for g, r in zip(grip_dir, r_hat))
        from so_arm_100_kinematics.grasp import _wrap_angle
        exact_root = math.atan2(-a_r, grip_dir[2])
        for e in (_wrap_angle(exact_root), _wrap_angle(exact_root + math.pi)):
            with self.assertRaises(Unreachable):
                ik(target, e, stick_roll_rad=0.0, elbow_up=True)

    def test_wrist_roll_asymmetric_limit_makes_one_end_assignment_fail(self):
        # Finding: Wrist_Roll's limit is asymmetric (~-157..+68deg, since
        # the +90deg zero-roll convention eats most of the positive
        # headroom). This exact direction is unreachable as base->tip...
        target = (0.0038386544587053484, -0.37227827310562134, 0.11324854488687432)
        axis = (0.999999999999996, 0.0, 8.95262103240344e-08)
        self.assertIsNone(solve_stick_orientation(target, axis))
        # ...but the OPPOSITE end assignment (negate the axis -- exactly
        # what a caller's "flip which end is base" toggle does) is fine.
        self.assertIsNotNone(solve_stick_orientation(
            target, tuple(-c for c in axis)))


class TestSolveStickPlacement(unittest.TestCase):
    """The PlaceStick-facing convenience: returns joints or raises."""

    def test_returns_five_joint_angles(self):
        joints = solve_stick_placement((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))
        self.assertEqual(len(joints), 5)

    def test_the_returned_joints_reproduce_the_target_on_replay(self):
        base, tip = (0.0, -0.36, 0.0), (0.0, -0.36, 0.110)
        joints = solve_stick_placement(base, tip)
        achieved_target = fk(joints)[0]
        expected_target = grasp_target(base, tip)
        for got, want in zip(achieved_target, expected_target):
            self.assertAlmostEqual(got, want, places=5)

    def test_raises_unreachable_rather_than_a_wrong_answer(self):
        far = (5.0, 0.0, 0.0)
        with self.assertRaises(Unreachable):
            solve_stick_placement(far, (far[0], far[1], far[2] + 0.1))


class TestBroadRandomSweep(unittest.TestCase):
    """Every accepted candidate, across many fully random 3D directions
    (not restricted to any plane), verified against fk(). This is what
    originally disproved the "out of plane is unreachable" claim -- kept at
    a size that runs quickly under both plain unittest and colcon test."""

    def test_every_accepted_candidate_verifies(self):
        import random

        random.seed(20260730)
        tested = solved = 0
        worst_error = 0.0
        for _ in range(500):
            azimuth = random.uniform(-1.8, 1.8)
            radius = random.uniform(0.28, 0.42)
            base = (SHOULDER[0] + radius * math.cos(azimuth),
                   SHOULDER[1] + radius * math.sin(azimuth),
                   random.uniform(0.0, 0.18))
            direction = _normalize((random.uniform(-1, 1), random.uniform(-1, 1),
                                    random.uniform(-1, 1)))
            tip = tuple(b + 0.100 * d for b, d in zip(base, direction))
            tested += 1

            target = grasp_target(base, tip)
            solved_result = solve_stick_orientation(
                target, tuple(t - b for t, b in zip(tip, base)))
            if solved_result is None:
                continue
            solved += 1
            elevation, roll, elbow_up, _error_deg = solved_result
            q = ik(target, elevation, stick_roll_rad=roll, elbow_up=elbow_up)
            achieved = stick_axis(q)
            grip_dir = _normalize(tuple(-d for d in direction))
            error_deg = _achieved_error_deg(grip_dir, achieved)
            worst_error = max(worst_error, error_deg)
            self.assertLessEqual(error_deg, ANCHOR_TOLERANCE_DEG,
                                 "candidate accepted but does not verify")

        self.assertGreater(solved, 50, "suspiciously few reachable placements found")
        self.assertLess(worst_error, ANCHOR_TOLERANCE_DEG)


if __name__ == "__main__":
    unittest.main()
