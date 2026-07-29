#!/usr/bin/env python3
"""Cross-checks so_arm_100_kinematics against MoveIt's own FK -- the one
thing the package's offline unit tests cannot prove, since those only check
the module against numbers hand-copied from the URDF file. This script
checks against whatever robot_description move_group actually has loaded
right now, which is the real risk: a transcription typo, or the URDF
changing (e.g. Phase 0's Mount_Platform origin fix) without constants.py
being updated to match.

Two automatic numeric checks, plus an optional RViz walkthrough:

1. FK cross-check: sample random joint configurations within limits, compare
   so_arm_100_kinematics.fk() against MoveIt's compute_fk() (KDL, reading the
   live robot_description) for the same joint values.
2. IK cross-check: sample random targets across the chosen build volume,
   solve with so_arm_100_kinematics.ik(), then confirm MoveIt's OWN compute_fk
   agrees the solution reaches the target -- this validates ik() against an
   independent FK path, not just this package's own fk().
3. --preview: publish each sampled IK solution to display_planned_path so you
   can watch the ghost robot move to each candidate stick placement in RViz,
   one at a time, gated by Enter -- the same mechanism
   pick_and_place_node.py uses, just for a single static pose per stick.

Run with MoveIt already up (e.g. `ros2 launch so_arm_100_moveit_config
pickandplace_demo.launch.py`), then:
    ros2 run so_arm_100_pick_and_place verify_kinematics
    ros2 run so_arm_100_pick_and_place verify_kinematics --preview
"""
import math
import random
import sys
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2

import so_arm_100_kinematics as sak

ARM_JOINT_NAMES = list(sak.JOINT_NAMES)
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "End_Effector"
ROBOT_MODEL_ID = "so_arm_100"

FK_SAMPLE_COUNT = 40
IK_SAMPLE_COUNT = 40
FK_TOLERANCE_M = 0.001  # both paths read the same URDF -- should be near-exact
IK_TOLERANCE_M = 0.001

# Margin pulled in from the hard joint limits: this check is about matching
# MoveIt's model, not about probing exact mechanical edge cases (chain.py's
# own tests already cover the 'home' pose sitting on a limit).
LIMIT_MARGIN_RAD = math.radians(3.0)


def _random_joint_sample():
    q = []
    for _name, _xyz, _rpy, _axis, lo, hi in sak.chain.CHAIN:
        lo_m, hi_m = lo + LIMIT_MARGIN_RAD, hi - LIMIT_MARGIN_RAD
        q.append(random.uniform(lo_m, hi_m))
    return q


def _random_build_volume_target():
    lo = sak.BUILD_VOLUME_MIN_M
    hi = sak.BUILD_VOLUME_MAX_M
    return (
        random.uniform(lo[0], hi[0]),
        random.uniform(lo[1], hi[1]),
        random.uniform(lo[2], hi[2]),
    )


def main():
    preview = "--preview" in sys.argv

    rclpy.init()
    node = Node("verify_kinematics_node")
    logger = node.get_logger()
    callback_group = ReentrantCallbackGroup()

    arm = MoveIt2(
        node=node,
        joint_names=ARM_JOINT_NAMES,
        base_link_name=BASE_LINK_NAME,
        end_effector_name=END_EFFECTOR_NAME,
        group_name="arm",
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    def shutdown(code):
        rclpy.shutdown()
        executor_thread.join()
        sys.exit(code)

    # --- 1. FK cross-check ---------------------------------------------------
    logger.info(f"FK cross-check: {FK_SAMPLE_COUNT} random joint samples vs. MoveIt's compute_fk...")
    fk_errors = []
    fk_failures = 0
    for i in range(FK_SAMPLE_COUNT):
        q = _random_joint_sample()
        ours_pos, _rot = sak.fk(q)
        pose_stamped = arm.compute_fk(joint_state=q)
        if pose_stamped is None:
            fk_failures += 1
            logger.warn(f"  [{i}] MoveIt compute_fk returned None for {[round(math.degrees(v),1) for v in q]}")
            continue
        p = pose_stamped.pose.position
        err = math.dist(ours_pos, (p.x, p.y, p.z))
        fk_errors.append(err)
        if err > FK_TOLERANCE_M:
            logger.warn(f"  [{i}] FK MISMATCH: ours={ours_pos} moveit=({p.x:.4f},{p.y:.4f},{p.z:.4f}) err={err*1000:.2f}mm")

    if fk_errors:
        logger.info(
            f"FK cross-check: {len(fk_errors)} compared, max err={max(fk_errors)*1000:.3f}mm, "
            f"mean err={sum(fk_errors)/len(fk_errors)*1000:.3f}mm, {fk_failures} MoveIt failures"
        )
    fk_ok = fk_errors and max(fk_errors) <= FK_TOLERANCE_M and fk_failures == 0

    # --- 2. IK cross-check ----------------------------------------------------
    logger.info(f"IK cross-check: {IK_SAMPLE_COUNT} random targets across the build volume...")
    ik_errors = []
    ik_unreachable = 0
    ik_moveit_disagreed = 0
    solved_examples = []
    for i in range(IK_SAMPLE_COUNT):
        target = _random_build_volume_target()
        try:
            q = sak.ik(target, tool_elevation_target_rad=0.0)
        except sak.Unreachable:
            ik_unreachable += 1
            continue
        pose_stamped = arm.compute_fk(joint_state=list(q))
        if pose_stamped is None:
            ik_moveit_disagreed += 1
            logger.warn(f"  [{i}] MoveIt compute_fk returned None for our ik() solution")
            continue
        p = pose_stamped.pose.position
        err = math.dist(target, (p.x, p.y, p.z))
        ik_errors.append(err)
        solved_examples.append((target, q))
        if err > IK_TOLERANCE_M:
            logger.warn(f"  [{i}] IK MISMATCH: target={target} moveit_fk=({p.x:.4f},{p.y:.4f},{p.z:.4f}) err={err*1000:.2f}mm")

    if ik_errors:
        logger.info(
            f"IK cross-check: {len(ik_errors)} solved+confirmed, max err={max(ik_errors)*1000:.3f}mm, "
            f"mean err={sum(ik_errors)/len(ik_errors)*1000:.3f}mm, "
            f"{ik_unreachable} unreachable (expected -- build volume isn't 100% covered), "
            f"{ik_moveit_disagreed} MoveIt disagreements"
        )
    ik_ok = ik_errors and max(ik_errors) <= IK_TOLERANCE_M and ik_moveit_disagreed == 0

    logger.info(f"RESULT: FK cross-check {'PASS' if fk_ok else 'FAIL'}, IK cross-check {'PASS' if ik_ok else 'FAIL'}")

    # --- 3. optional RViz walkthrough -----------------------------------------
    if preview and solved_examples:
        preview_pub = node.create_publisher(DisplayTrajectory, "display_planned_path", 1)
        node.create_rate(1.0).sleep()  # let the publisher match up before the first message
        logger.info(f"Preview: {len(solved_examples)} solved stick placements. Press Enter to step through, q to stop.")
        for i, (target, q) in enumerate(solved_examples):
            point = JointTrajectoryPoint(positions=list(q), time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg())
            traj = JointTrajectory(joint_names=ARM_JOINT_NAMES, points=[point])
            display = DisplayTrajectory(model_id=ROBOT_MODEL_ID)
            display.trajectory.append(RobotTrajectory(joint_trajectory=traj))
            current_state = arm.joint_state
            if current_state is not None:
                display.trajectory_start.joint_state = current_state
            preview_pub.publish(display)
            logger.info(f"[{i+1}/{len(solved_examples)}] target={tuple(round(v,3) for v in target)} -- check RViz. Enter to continue, q to stop.")
            answer = input().strip().lower()
            if answer in ("q", "quit"):
                break

    shutdown(0 if (fk_ok and ik_ok) else 1)


if __name__ == "__main__":
    main()
