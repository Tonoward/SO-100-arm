#!/usr/bin/env python3
"""Hardcoded pick-and-place sequence (no perception, no closed loop): grasp
a stick at a fixed pose, lift it clear of its mounting hole, and move it to
a fixed place pose. All waypoints come from config/pick_and_place.yaml --
see that file for the tuning workflow.

Built on pymoveit2 (ros-humble-pymoveit2), since no compiled Python MoveIt
bindings (moveit_commander / moveit_py) are available on this ROS2 Humble
install.

Interactive mode (default, `interactive:=true`): each step is planned first
and published to `display_planned_path` so RViz shows it as a preview --
nothing moves yet. The terminal then prompts before executing, so you can
check the preview in RViz and tune pick_and_place.yaml without the arm
actually moving on a bad plan. Run via `ros2 run`, not `ros2 launch`, if the
Enter-key prompts don't seem to reach this process -- launch's stdin
passthrough can be unreliable with multiple nodes.
"""
from threading import Thread
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

from pymoveit2 import MoveIt2, MoveIt2Gripper

from so_arm_100_pick_and_place.pose_utils import compose, vec6_to_pos_quat

ARM_JOINT_NAMES = [
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
]
GRIPPER_JOINT_NAMES = ["Gripper"]
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "End_Effector"
ROBOT_MODEL_ID = "so_arm_100"


def main():
    rclpy.init()
    node = Node("pick_and_place_node")

    base_frame = node.declare_parameter("base_frame", "base_link").value
    stick_size = list(node.declare_parameter("stick.size", [0.0065, 0.0065, 0.1]).value)
    stick_pose_v = list(node.declare_parameter(
        "stick.pose", [0.25, 0.0, 0.05, 0.0, 0.0, 0.0]).value)
    grasp_offset_v = list(node.declare_parameter(
        "grasp.offset", [0.0, 0.0, 0.03, 0.0, 1.5708, 0.0]).value)
    pregrasp_lift = node.declare_parameter("grasp.pregrasp_lift", 0.05).value
    lift_height = node.declare_parameter("grasp.lift_height", 0.05).value
    gripper_open_position = node.declare_parameter("grasp.gripper_open_position", 0.7854).value
    gripper_grasp_position = node.declare_parameter("grasp.gripper_grasp_position", 0.2).value
    place_pose_v = list(node.declare_parameter(
        "place.pose", [0.0, 0.25, 0.15, 0.0, 1.5708, 0.0]).value)
    velocity_scaling = node.declare_parameter("velocity_scaling", 0.2).value
    acceleration_scaling = node.declare_parameter("acceleration_scaling", 0.2).value
    planning_time = node.declare_parameter("planning_time", 5.0).value
    interactive = node.declare_parameter("interactive", True).value

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
    gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=GRIPPER_JOINT_NAMES,
        open_gripper_joint_positions=[gripper_open_position],
        closed_gripper_joint_positions=[gripper_grasp_position],
        gripper_group_name="gripper",
        callback_group=callback_group,
    )
    arm.max_velocity = velocity_scaling
    arm.max_acceleration = acceleration_scaling
    arm.allowed_planning_time = planning_time
    gripper.allowed_planning_time = planning_time

    preview_pub = node.create_publisher(DisplayTrajectory, "display_planned_path", 1)

    # MultiThreadedExecutor + ReentrantCallbackGroup: pymoveit2's plan()/
    # wait_until_executed() block the calling thread via rclpy.spin_once()
    # internally, so the node must already be spinning on another thread or
    # those calls would deadlock waiting on their own callbacks.
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    def shutdown(code):
        rclpy.shutdown()
        executor_thread.join()
        exit(code)

    def publish_preview(moveit_interface, trajectory: JointTrajectory):
        display = DisplayTrajectory()
        display.model_id = ROBOT_MODEL_ID
        display.trajectory.append(RobotTrajectory(joint_trajectory=trajectory))
        current_state = moveit_interface.joint_state
        if current_state is not None:
            display.trajectory_start.joint_state = current_state
        preview_pub.publish(display)

    def confirm(step_name: str) -> str:
        """Returns 'execute', 'skip', or 'abort'."""
        if not interactive:
            return "execute"
        # Logged (not just passed to input()'s prompt arg) so the message is
        # guaranteed to appear even under `ros2 launch`, whose line-buffered,
        # per-process output prefixing never flushes an unterminated prompt
        # string. input() itself is still only reliable under `ros2 run` --
        # `ros2 launch` does not consistently forward the terminal's stdin to
        # a launched node.
        logger.info(
            f"[{step_name}] Plan ready -- check RViz. "
            "Type Enter/e to execute, s to skip, or q to abort, then press Enter."
        )
        while True:
            answer = input().strip().lower()
            if answer in ("", "e", "execute"):
                return "execute"
            if answer in ("s", "skip"):
                return "skip"
            if answer in ("q", "quit", "abort"):
                return "abort"
            print("Please enter Enter/e to execute, s to skip, or q to abort.")

    def run_step(moveit_interface, step_name: str, plan_fn: Callable[[], Optional[JointTrajectory]]) -> bool:
        logger.info(f"Planning step '{step_name}'...")
        trajectory = plan_fn()
        if trajectory is None:
            logger.error(f"Planning failed at step '{step_name}'")
            return False

        publish_preview(moveit_interface, trajectory)
        decision = confirm(step_name)
        if decision == "abort":
            logger.warn("Aborted by user.")
            shutdown(1)
        if decision == "skip":
            logger.warn(f"Step '{step_name}' skipped by user.")
            return True

        moveit_interface.execute(trajectory)
        ok = moveit_interface.wait_until_executed()
        if ok:
            logger.info(f"Step '{step_name}' OK")
        else:
            logger.error(f"Execution failed at step '{step_name}'")
        return ok

    def plan_arm_pose(position, quat, cartesian: bool = False):
        return arm.plan(
            position=position, quat_xyzw=quat, frame_id=base_frame,
            cartesian=cartesian, max_step=0.01, cartesian_fraction_threshold=0.95,
        )

    def plan_gripper(position: float):
        return gripper.plan(joint_positions=[position])

    stick_pos, stick_quat = vec6_to_pos_quat(stick_pose_v)
    grasp_offset_pos, grasp_offset_quat = vec6_to_pos_quat(grasp_offset_v)
    grasp_pos, grasp_quat = compose(stick_pos, stick_quat, grasp_offset_pos, grasp_offset_quat)
    place_pos, place_quat = vec6_to_pos_quat(place_pose_v)

    pregrasp_pos = (grasp_pos[0], grasp_pos[1], grasp_pos[2] + pregrasp_lift)
    unlock_pos = (grasp_pos[0], grasp_pos[1], grasp_pos[2] + lift_height)
    retreat_pos = (place_pos[0], place_pos[1], place_pos[2] + lift_height)

    logger.info("Adding stick collision object to the planning scene.")
    arm.add_collision_box(
        id="stick", size=stick_size, position=stick_pos, quat_xyzw=stick_quat,
        frame_id=base_frame,
    )

    if not run_step(gripper, "open gripper (initial)", lambda: plan_gripper(gripper_open_position)):
        return shutdown(1)
    if not run_step(arm, "move to pregrasp", lambda: plan_arm_pose(pregrasp_pos, grasp_quat)):
        return shutdown(1)
    if not run_step(arm, "descend to grasp", lambda: plan_arm_pose(grasp_pos, grasp_quat, cartesian=True)):
        return shutdown(1)
    if not run_step(gripper, "close gripper on stick", lambda: plan_gripper(gripper_grasp_position)):
        return shutdown(1)

    logger.info("Attaching stick to the end effector.")
    arm.attach_collision_object(
        id="stick", link_name="Fixed_Gripper",
        touch_links=["Fixed_Gripper", "Moving_Jaw", "End_Effector"],
    )

    if not run_step(arm, "lift to unlock", lambda: plan_arm_pose(unlock_pos, grasp_quat, cartesian=True)):
        return shutdown(1)
    if not run_step(arm, "move to place", lambda: plan_arm_pose(place_pos, place_quat)):
        return shutdown(1)
    if not run_step(gripper, "open gripper at place", lambda: plan_gripper(gripper_open_position)):
        return shutdown(1)

    logger.info("Detaching stick.")
    arm.detach_collision_object(id="stick")
    arm.add_collision_box(
        id="stick", size=stick_size, position=place_pos, quat_xyzw=place_quat,
        frame_id=base_frame,
    )

    run_step(arm, "retreat", lambda: plan_arm_pose(retreat_pos, place_quat))

    logger.info("Pick-and-place sequence complete.")
    shutdown(0)


if __name__ == "__main__":
    main()
