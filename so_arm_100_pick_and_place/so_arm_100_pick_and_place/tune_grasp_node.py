#!/usr/bin/env python3
"""Interactive tool for empirically tuning `grasp_verification` (Sec 4 D4
of ROS2_IMPLEMENTATION_PLAN.md): what gripper closed position and what
`gap_threshold` reliably tell "holding the stick" apart from "closed on
nothing", on the REAL gripper against the REAL stick.

Moves the arm through home -> pregrasp -> lower (gated by the usual
plan -> preview -> confirm prompt, same as pick_and_place_node.py), so the
jaws end up open around the real stick in the feeder. Then repeatedly:
closes the gripper to whatever position you type, reads back the ACTUAL
settled position, and reports the grasp-verification gap plus a PASS/FAIL
verdict at the currently-configured `grasp_verification.gap_threshold` AND
a few other candidate thresholds, so you can see in one grasp how sensitive
the choice of threshold actually is -- not just whether today's default
happens to work.

This matters now specifically because `PickStick` (the task server, Sec 11
Phase 4) has no attach-anyway override for a failed check, unlike the
interactive demo -- an untuned threshold is a hard failure there, not a
prompt.

⚠ **The gripper open/close loop executes immediately, with no per-attempt
confirm prompt** -- that is the point of a fast iteration tool, and gripper
motion alone is low-risk. The arm moves to get INTO position
(home/pregrasp/lower, and the final retreat) ARE gated, same as always.
Keep the stick loaded, watch the arm, and be ready to Ctrl+C.

Requires the real hardware launch:
    ros2 launch so_arm_100_moveit_config pickandplace_demo.launch.py
Then, in another terminal (use `ros2 run`, not `ros2 launch`, for reliable
stdin -- see motion.MotionController.prompt) -- **with `--params-file`,
explicitly**, since `ros2 run` never auto-loads a yaml the way a launch
file does, and this node isn't named `pick_and_place_node` so
`ros2 launch`'s own auto-load wouldn't have matched it either way (Sec 4
D9 -- found this node silently ran on its own code-fallback stick pose/size,
13cm off, until this was fixed 2026-08-02):
    ros2 run so_arm_100_pick_and_place tune_grasp --ros-args --params-file \\
        $(ros2 pkg prefix so_arm_100_pick_and_place)/share/so_arm_100_pick_and_place/config/pick_and_place.yaml

Once you've picked good values, write them back into
config/pick_and_place.yaml's `grasp.gripper_grasp_position_deg` and
`grasp_verification.gap_threshold`.
"""
import math

import rclpy
from rclpy.node import Node

from so_arm_100_pick_and_place import pose_utils, scene, sequences
from so_arm_100_pick_and_place.motion import MotionController


def main():
    rclpy.init()
    node = Node("tune_grasp_node")

    base_frame = node.declare_parameter("base_frame", "base_link").value
    # Matches config/pick_and_place.yaml's real feeder-hole measurement --
    # kept in sync 2026-08-02 (Sec 4 D9); fallback safety net only, not the
    # intended source of truth -- pass --params-file (see module docstring).
    # base_xyz_m is the stick's PHYSICAL BASE (bottom, at the feeder hole),
    # not a box center -- see pose_utils.feeder_stick_pose.
    stick_base_xyz_m = tuple(node.declare_parameter("stick.base_xyz_m", [0.379, -0.026, 0.0103]).value)
    stick_section_m = list(node.declare_parameter("stick.section_m", [0.0060, 0.0060]).value)
    stick_length_m = node.declare_parameter("stick.default_length_m", 0.11).value
    stick_size = (stick_section_m[0], stick_section_m[1], stick_length_m)
    stick_pos, stick_quat = pose_utils.feeder_stick_pose(stick_base_xyz_m, stick_length_m)

    steps_cfg = sequences.declare_all_steps(node)

    default_open_deg = node.declare_parameter("grasp.gripper_open_position_deg", 30.0).value
    # -9.0 deg / 0.0157 rad re-tuned 2026-08-02 on the real gripper/stock via
    # this same tool -- see config/pick_and_place.yaml's own comment.
    default_grasp_deg = node.declare_parameter("grasp.gripper_grasp_position_deg", -9.0).value
    configured_gap_threshold_deg = math.degrees(
        node.declare_parameter("grasp_verification.gap_threshold", 0.0157).value)
    velocity_scaling = node.declare_parameter("velocity_scaling", 0.4).value
    acceleration_scaling = node.declare_parameter("acceleration_scaling", 0.4).value
    planning_time = node.declare_parameter("planning_time", 5.0).value

    open_position_rad = math.radians(default_open_deg)

    motion = MotionController(
        node,
        base_frame=base_frame,
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        planning_time=planning_time,
        interactive=True,  # the approach moves ARE gated -- see module docstring.
        gripper_open_position=open_position_rad,
        gripper_closed_position=math.radians(default_grasp_deg),
    )
    logger = motion.logger

    logger.info("Adding stick collision object to the planning scene.")
    scene.add_stick_at_feeder(motion.arm, stick_size, stick_pos, stick_quat, base_frame)

    if not motion.run_step(motion.arm, "home position", lambda: motion.plan_arm_step(steps_cfg["home"])):
        return motion.shutdown(1)
    if not motion.run_step(motion.gripper, "open gripper", lambda: motion.plan_gripper(open_position_rad)):
        return motion.shutdown(1)
    if not motion.run_step(motion.arm, "move to pregrasp", lambda: motion.plan_arm_step(steps_cfg["pregrasp"])):
        return motion.shutdown(1)
    if not motion.run_step(motion.arm, "lower to stick", lambda: motion.plan_arm_step(steps_cfg["lower"])):
        return motion.shutdown(1)

    candidate_thresholds_deg = sorted({configured_gap_threshold_deg, 3.0, 5.0, 8.0, 11.0, 15.0})

    logger.info(
        "Ready -- jaws are open around the stick. Type a gripper CLOSED position "
        f"in degrees (configured default is {default_grasp_deg:.1f} deg) and press "
        "Enter to test it: the gripper closes immediately (no confirm prompt -- "
        "see the module docstring), reopens, and reports the gap. Type 'q' to "
        "stop, open the gripper, and retreat."
    )

    while True:
        try:
            raw = input("closed position (deg), or q: ").strip()
        except EOFError:
            break
        if raw.lower() in ("q", "quit"):
            break
        try:
            commanded_deg = float(raw)
        except ValueError:
            print("Enter a number (degrees) or 'q'.")
            continue

        if not _close_gripper(motion, math.radians(commanded_deg), stick_size, stick_pos, stick_quat, base_frame):
            logger.error("Planning/execution failed for that position -- outside joint limits, or a real collision.")
            continue

        state = motion.gripper.joint_state
        if state is None or "Gripper" not in state.name:
            logger.warn("No gripper joint state available yet -- try again.")
            continue
        actual_deg = math.degrees(state.position[state.name.index("Gripper")])
        gap_deg = abs(actual_deg - commanded_deg)

        logger.info(f"commanded={commanded_deg:.1f} deg  actual={actual_deg:.1f} deg  gap={gap_deg:.2f} deg")
        for threshold_deg in candidate_thresholds_deg:
            verdict = "HOLDING (verified)" if gap_deg >= threshold_deg else "EMPTY (would fail)"
            marker = "  <- currently configured" if abs(threshold_deg - configured_gap_threshold_deg) < 1e-6 else ""
            logger.info(f"    threshold {threshold_deg:5.1f} deg -> {verdict}{marker}")

        # Reopen so every attempt starts from the same, known, visible state.
        if not _move_gripper(motion, open_position_rad):
            logger.warn("Could not reopen the gripper automatically -- check it by eye before the next attempt.")

    logger.info("Opening gripper and retreating.")
    if not _move_gripper(motion, open_position_rad):
        logger.warn("Could not reopen the gripper automatically -- check it by eye before retreating.")
    motion.run_step(motion.arm, "retreat", lambda: motion.plan_arm_step(steps_cfg["retreat"]))
    motion.shutdown(0)


def _move_gripper(motion, position_rad) -> bool:
    trajectory = motion.gripper.plan(joint_positions=[position_rad])
    if trajectory is None:
        return False
    motion.gripper.execute(trajectory)
    return motion.gripper.wait_until_executed()


def _close_gripper(motion, position_rad, stick_size, stick_pos, stick_quat, base_frame) -> bool:
    """Same remove -> plan -> re-add dance as sequences.run_pick_sequence's
    plan_grasp_gripper() -- the stick collision box sits exactly where the
    jaws close (Sec 4 D9's fix made this accurate), so planning a close
    through it without removing it first is rejected as a collision, not a
    joint-limits issue. Only closing needs this; opening away from the
    stick never collides with it regardless of the box's pose."""
    scene.remove_stick(motion.arm)
    trajectory = motion.gripper.plan(joint_positions=[position_rad])
    scene.add_stick_at_feeder(motion.arm, stick_size, stick_pos, stick_quat, base_frame)
    if trajectory is None:
        return False
    motion.gripper.execute(trajectory)
    return motion.gripper.wait_until_executed()


if __name__ == "__main__":
    main()
