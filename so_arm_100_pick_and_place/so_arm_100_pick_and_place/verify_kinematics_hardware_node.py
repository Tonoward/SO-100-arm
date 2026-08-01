#!/usr/bin/env python3
"""Moves the REAL arm to a so_arm_100_kinematics-computed joint target,
gated by the same plan -> RViz ghost preview -> Enter-to-execute workflow as
pick_and_place_node.py (now shared via motion.MotionController -- Sec 11
Phase 2 -- rather than a second, drifting inline copy of the same pattern).

This is the one thing verify_kinematics_node.py cannot do: that script only
ever calls MoveIt's compute_fk (a pure kinematics query -- no motion, safe to
run with hardware physically connected). This script commands the real
servos, so you must be present and ready to hit q=abort.

Requires the REAL hardware launch (not moveit.launch.py, which brings up
move_group only):
    ros2 launch so_arm_100_moveit_config pickandplace_demo.launch.py

Then, in another terminal -- use `ros2 run`, not `ros2 launch`, or the
Enter-key prompts may not reach this process:
    ros2 run so_arm_100_pick_and_place verify_kinematics_hardware

With no arguments this reproduces the tuned 'lower' pose's target AND
approach angle ([0.379, -0.026, 0.064] m, -1.0 deg tool elevation) -- the one
point in the workspace with strong existing hardware ground truth, verified
repeatedly over the course of this project. So the FIRST thing this proves is
"does ik() send the arm to the same physical configuration the hand-tuned
pose does" -- confirm that before trusting a brand-new point.

To test a different point:
    ros2 run so_arm_100_pick_and_place verify_kinematics_hardware \\
        --ros-args -p target_xyz_m:="[0.30, -0.35, 0.10]" -p tool_elevation_deg:=0.0

Safety: always plans first, always publishes the RViz ghost preview, and
always waits for an explicit Enter (or q to abort) before commanding real
motion -- never auto-executes. Keep velocity/acceleration scaling low, keep
the gripper empty, and keep it clear of the feeder stick until you trust a
result.
"""
import math

import rclpy
from rclpy.node import Node

import so_arm_100_kinematics as sak
from so_arm_100_pick_and_place.motion import MotionController

ARM_JOINT_NAMES = list(sak.JOINT_NAMES)

# The EXACT FK position of the tuned 'lower' pose (joint_positions
# [-93, 54, -7, -46, 90] deg from pick_and_place.yaml) -- the one target with
# strong existing hardware ground truth, verified repeatedly on this
# hardware. Deliberately NOT pick_and_place.yaml's stick.pose value
# ([0.379, -0.026, 0.064]) -- that's a hand-rounded description of the same
# point, off by ~4-5mm from the true FK position, and at this arm's
# extension that few mm shifts the IK solution by several degrees. Using the
# exact value makes ik() reproduce the tuned angles to <0.1 deg (confirmed
# offline before this script was ever run against real hardware) -- see
# so_arm_100_kinematics/test/test_chain.py's
# test_recovers_the_actual_tuned_joint_angles for the same check.
DEFAULT_TARGET_XYZ_M = (0.38348449719309957, -0.025102606457800616, 0.06478080524067709)
DEFAULT_TOOL_ELEVATION_DEG = -1.0


def main():
    rclpy.init()
    node = Node("verify_kinematics_hardware_node")

    target_xyz_m = list(node.declare_parameter("target_xyz_m", list(DEFAULT_TARGET_XYZ_M)).value)
    tool_elevation_deg = node.declare_parameter("tool_elevation_deg", DEFAULT_TOOL_ELEVATION_DEG).value
    stick_roll_deg = node.declare_parameter("stick_roll_deg", 0.0).value
    velocity_scaling = node.declare_parameter("velocity_scaling", 0.2).value
    acceleration_scaling = node.declare_parameter("acceleration_scaling", 0.2).value
    planning_time = node.declare_parameter("planning_time", 5.0).value
    interactive = node.declare_parameter("interactive", True).value

    logger = node.get_logger()

    motion = MotionController(
        node,
        base_frame="base_link",
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        planning_time=planning_time,
        interactive=interactive,
    )

    try:
        joints = sak.ik(
            tuple(target_xyz_m),
            tool_elevation_target_rad=math.radians(tool_elevation_deg),
            stick_roll_rad=math.radians(stick_roll_deg),
        )
    except sak.Unreachable as exc:
        logger.error(f"so_arm_100_kinematics reports this target is unreachable: {exc}")
        return motion.shutdown(1)

    logger.info(
        f"Target {target_xyz_m} m, elevation {tool_elevation_deg} deg, roll {stick_roll_deg} deg "
        f"-> joints (deg) = {[round(math.degrees(q), 2) for q in joints]}"
    )

    def plan_target():
        return motion.arm.plan(joint_positions=list(joints), joint_names=ARM_JOINT_NAMES)

    ok = motion.run_step(motion.arm, "move to target", plan_target, allow_skip=False)
    if ok:
        logger.info(
            "Execution OK -- now physically compare the real arm's position "
            f"against the intended target {target_xyz_m} m."
        )
    motion.shutdown(0 if ok else 1)


if __name__ == "__main__":
    main()
