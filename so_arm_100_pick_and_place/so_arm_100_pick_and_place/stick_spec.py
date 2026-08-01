"""Turns a stick's physical base/tip/roll into joint angles this arm can
reach -- for use as a MoveIt joint-space goal
(ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 3 / Sec 7.2 steps 1-2).

Pure Python -- no rclpy -- so it's testable standalone, same style as
so_arm_100_kinematics itself. Deliberately thin: the actual exhaustive,
self-verifying orientation search lives in
``so_arm_100_kinematics.grasp.solve_stick_orientation`` and is never
reimplemented here -- see that package's README on why a second
implementation would let this side and the Blender addon silently disagree
about where a stick goes.
"""
import math

import so_arm_100_kinematics as sak


def solve_stick_spec_joints(base_xyz_m, tip_xyz_m, roll_deg=0.0):
    """5 joint angles (radians, ``sak.JOINT_NAMES`` order) placing a stick
    with these physical ends.

    Two things this adds on top of ``sak.solve_stick_placement()``:

    1. **Retries with base/tip flipped** if the first assignment raises
       ``Unreachable``. ``Wrist_Roll``'s joint limit is asymmetric in
       ``stick_roll`` terms (``ROS2_IMPLEMENTATION_PLAN.md`` Sec 9.6), so a
       direction that fails one way routinely succeeds with the ends
       swapped -- this was flagged in ``STATUS.md`` as "not yet wired into
       any caller"; this function is that caller.
    2. **Applies ``roll_deg`` on top of whichever orientation solves.**
       ``solve_stick_orientation`` only solves for the stick's AXIS
       direction (a vector) -- the twist about that axis is a second,
       independent degree of freedom for the SQUARE stock
       (``ROS2_IMPLEMENTATION_PLAN.md`` Sec 8.3's ``StickSpec.roll_deg``),
       which the shared kinematics package does not take as an input at
       all. ``ik()`` is re-run (the returned tuple is never mutated
       directly) with ``stick_roll_rad = solved_roll + radians(roll_deg)``,
       so the ``Wrist_Roll`` joint-limit check re-validates the adjusted
       value rather than silently accepting an out-of-range twist.

    Raises ``sak.Unreachable`` (combining both attempts' reasons) if
    neither end assignment, with the requested roll applied, is reachable.
    """
    roll_offset_rad = math.radians(roll_deg)
    reasons = []
    for attempt_base, attempt_tip in ((base_xyz_m, tip_xyz_m), (tip_xyz_m, base_xyz_m)):
        target = sak.grasp_target(attempt_base, attempt_tip)
        axis = tuple(t - b for t, b in zip(attempt_tip, attempt_base))
        solved = sak.solve_stick_orientation(target, axis)
        if solved is None:
            reasons.append(f"base={attempt_base} tip={attempt_tip}: no orientation verifies")
            continue
        elevation, roll, elbow_up, _error_deg = solved
        try:
            return sak.ik(target, elevation, stick_roll_rad=roll + roll_offset_rad, elbow_up=elbow_up)
        except sak.Unreachable as exc:
            reasons.append(f"base={attempt_base} tip={attempt_tip}: {exc}")
            continue
    raise sak.Unreachable(
        "no placement of base=%r tip=%r (roll_deg=%r) is reachable either way round: %s"
        % (base_xyz_m, tip_xyz_m, roll_deg, "; ".join(reasons))
    )
