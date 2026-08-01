#!/usr/bin/env python3
"""Hardcoded pick-and-place sequence (no perception): home, grasp a stick at
a fixed pose, lift it clear of its mounting hole, and move it to a fixed
place pose. Every arm step's waypoint AND planning mode is fully
config-driven from config/pick_and_place.yaml (`steps.<name>.mode`: "joint"
for a directly-verified joint-space target, or "cartesian_relative" /
"cartesian_absolute" for a straight-line Cartesian path when the path shape
matters, not just the destination) -- retuning or switching a step's mode is
a pure YAML edit, no code changes. Joint angles (joint_positions, pose's
roll/pitch/yaw, and the gripper open/grasp positions) are given in the yaml
as DEGREES -- matching how you'd read them off RViz's Joints tab -- and
converted to radians here before being sent to MoveIt. See that file for the
tuning workflow.

This module is now a thin `main()`: parameter declaration + wiring, over
`motion.MotionController` (the MoveIt2 plan/preview/gate/execute machinery),
`scene` (collision-object lifecycle), and `sequences.run_full_demo` (the
composed step sequence itself) -- ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 2.

Interactive mode (default, `interactive:=true`): every step is planned
first and published to `display_planned_path`, so RViz shows the ghost
robot animating through the planned motion -- the same preview you'd see
after dragging the interactive marker and hitting "Plan", except nothing
moves yet. A terminal prompt then asks whether to execute it, skip it, or
abort the whole run; only Enter actually triggers execution. Run via
`ros2 run`, not `ros2 launch`, if the Enter-key prompts don't seem to reach
this process -- launch's stdin passthrough can be unreliable with multiple
nodes.

Grasp verification (`grasp_verification.enabled`, default true): since
there's no force/tactile sensing, "did we actually grab the stick" is
inherently a guess. When enabled, the gripper's actual settled position
after closing is compared against the commanded closed position -- stopping
well short means it hit resistance (probably holding something); reaching
the commanded value means it likely closed on nothing. Set to false to skip
the check and always attach (pure assumption).
"""
import math

import rclpy
from rclpy.node import Node

from so_arm_100_pick_and_place import sequences
from so_arm_100_pick_and_place.motion import MotionController
from so_arm_100_pick_and_place.pose_utils import vec6_to_pos_quat


def main():
    rclpy.init()
    node = Node("pick_and_place_node")

    base_frame = node.declare_parameter("base_frame", "base_link").value
    stick_size = list(node.declare_parameter("stick.size", [0.0065, 0.0065, 0.1]).value)
    stick_pose_v = list(node.declare_parameter(
        "stick.pose", [0.25, 0.0, 0.05, 0.0, 0.0, 0.0]).value)
    stick_pos, stick_quat = vec6_to_pos_quat(stick_pose_v)

    steps_cfg = sequences.declare_all_steps(node)

    gripper_open_position = math.radians(node.declare_parameter("grasp.gripper_open_position_deg", 30.0).value)
    gripper_grasp_position = math.radians(node.declare_parameter("grasp.gripper_grasp_position_deg", -10.0).value)
    grasp_verification_enabled = node.declare_parameter("grasp_verification.enabled", True).value
    grasp_gap_threshold = node.declare_parameter("grasp_verification.gap_threshold", 0.1).value
    velocity_scaling = node.declare_parameter("velocity_scaling", 0.2).value
    acceleration_scaling = node.declare_parameter("acceleration_scaling", 0.2).value
    planning_time = node.declare_parameter("planning_time", 5.0).value
    interactive = node.declare_parameter("interactive", True).value

    motion = MotionController(
        node,
        base_frame=base_frame,
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        planning_time=planning_time,
        interactive=interactive,
        gripper_open_position=gripper_open_position,
        gripper_closed_position=gripper_grasp_position,
    )

    sequences.run_full_demo(
        motion,
        steps_cfg,
        stick_cfg={"size": stick_size, "position": stick_pos, "quat_xyzw": stick_quat},
        grasp_cfg={
            "open_position": gripper_open_position,
            "grasp_position": gripper_grasp_position,
            "verification_enabled": grasp_verification_enabled,
            "gap_threshold": grasp_gap_threshold,
        },
    )


if __name__ == "__main__":
    main()
