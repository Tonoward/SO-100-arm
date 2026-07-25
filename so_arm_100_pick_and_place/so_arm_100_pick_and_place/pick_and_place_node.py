#!/usr/bin/env python3
"""Hardcoded pick-and-place sequence (no perception, no closed loop): home,
grasp a stick at a fixed pose, lift it clear of its mounting hole, and move
it to a fixed place pose. All waypoints come from config/pick_and_place.yaml
-- see that file for the tuning workflow.

Built on pymoveit2 (ros-humble-pymoveit2), since no compiled Python MoveIt
bindings (moveit_commander / moveit_py) are available on this ROS2 Humble
install.

Interactive mode (default, `interactive:=true`): every step is planned
first and published to `display_planned_path`, so RViz shows the ghost
robot animating through the planned motion -- the same preview you'd see
after dragging the interactive marker and hitting "Plan", except nothing
moves yet. A terminal prompt then asks whether to execute it, skip it, or
abort the whole run; only Enter actually triggers execution. Run via
`ros2 run`, not `ros2 launch`, if the Enter-key prompts don't seem to reach
this process -- launch's stdin passthrough can be unreliable with multiple
nodes.
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
    home_joint_positions = list(node.declare_parameter(
        "home.joint_positions", [0.0, -1.745, 1.5, 1.25, 1.5708]).value)
    stick_size = list(node.declare_parameter("stick.size", [0.0065, 0.0065, 0.1]).value)
    stick_pose_v = list(node.declare_parameter(
        "stick.pose", [0.25, 0.0, 0.05, 0.0, 0.0, 0.0]).value)
    pregrasp_joint_positions = list(node.declare_parameter(
        "grasp.pregrasp_joint_positions", home_joint_positions).value)
    grasp_offset_v = list(node.declare_parameter(
        "grasp.offset", [0.0, 0.0, 0.03, 0.0, 1.5708, 0.0]).value)
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

    def prompt(message: str) -> str:
        """Returns 'proceed', 'skip', or 'abort'."""
        if not interactive:
            return "proceed"
        # Logged (not just passed to input()'s prompt arg) so the message is
        # guaranteed to appear even under `ros2 launch`, whose line-buffered,
        # per-process output prefixing never flushes an unterminated prompt
        # string. input() itself is still only reliable under `ros2 run` --
        # `ros2 launch` does not consistently forward the terminal's stdin to
        # a launched node.
        logger.info(f"{message} Type Enter/e to proceed, s to skip, or q to abort, then press Enter.")
        while True:
            answer = input().strip().lower()
            if answer in ("", "e", "execute", "proceed"):
                return "proceed"
            if answer in ("s", "skip"):
                return "skip"
            if answer in ("q", "quit", "abort"):
                return "abort"
            print("Please enter Enter/e to proceed, s to skip, or q to abort.")

    def run_step(moveit_interface, step_name: str, plan_fn: Callable[[], Optional[JointTrajectory]]) -> bool:
        logger.info(f"Planning step '{step_name}'...")
        trajectory = plan_fn()
        if trajectory is None:
            logger.error(f"Planning failed at step '{step_name}'")
            return False

        publish_preview(moveit_interface, trajectory)
        decision = prompt(f"[{step_name}] Plan ready -- check the ghost movement preview in RViz.")
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

    def plan_arm_joints(positions):
        return arm.plan(joint_positions=list(positions), joint_names=ARM_JOINT_NAMES)

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

    unlock_pos = (grasp_pos[0], grasp_pos[1], grasp_pos[2] + lift_height)
    retreat_pos = (place_pos[0], place_pos[1], place_pos[2] + lift_height)

    logger.info("Adding stick collision object to the planning scene.")
    arm.add_collision_box(
        id="stick", size=stick_size, position=stick_pos, quat_xyzw=stick_quat,
        frame_id=base_frame,
    )

    # 1. Start position is wherever the arm already is -- no action needed.
    # 2-3. Home.
    if not run_step(arm, "home position", lambda: plan_arm_joints(home_joint_positions)):
        return shutdown(1)
    # 4-5. Open gripper.
    if not run_step(gripper, "open gripper", lambda: plan_gripper(gripper_open_position)):
        return shutdown(1)
    # 6-7. Pregrasp -- joint-space target verified directly in RViz, not
    # derived from stick.pose, so it can't fail on Cartesian/IK reachability.
    if not run_step(arm, "move to pregrasp", lambda: plan_arm_joints(pregrasp_joint_positions)):
        return shutdown(1)
    # 8-9. Lower to the stick, gripper still open.
    if not run_step(arm, "lower to stick", lambda: plan_arm_pose(grasp_pos, grasp_quat, cartesian=True)):
        return shutdown(1)
    # 10-11. Grasp: close the gripper on the stick.
    if not run_step(gripper, "grasp stick (close gripper)", lambda: plan_gripper(gripper_grasp_position)):
        return shutdown(1)

    logger.info("Attaching stick to the end effector.")
    arm.attach_collision_object(
        id="stick", link_name="Fixed_Gripper",
        touch_links=["Fixed_Gripper", "Moving_Jaw", "End_Effector"],
    )

    # 12-13. Lift the stick clear of the hole.
    if not run_step(arm, "lift stick", lambda: plan_arm_pose(unlock_pos, grasp_quat, cartesian=True)):
        return shutdown(1)
    # 14-15. Move to the place location.
    if not run_step(arm, "place stick", lambda: plan_arm_pose(place_pos, place_quat)):
        return shutdown(1)

    # Not in the original numbered list, but needed to actually release the
    # stick rather than leave it stuck in the gripper -- flag if this isn't
    # what's wanted.
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
