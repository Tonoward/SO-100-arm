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

Grasp verification (`grasp_verification.enabled`, default true): since
there's no force/tactile sensing, "did we actually grab the stick" is
inherently a guess. When enabled, the gripper's actual settled position
after closing is compared against the commanded closed position -- stopping
well short means it hit resistance (probably holding something); reaching
the commanded value means it likely closed on nothing. Set to false to skip
the check and always attach (pure assumption).
"""
import math
from threading import Thread
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

from pymoveit2 import MoveIt2, MoveIt2Gripper

from so_arm_100_pick_and_place.pose_utils import rpy_to_quat, vec6_to_pos_quat

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

    def declare_arm_step(name: str, default_mode: str, **defaults):
        """Every arm step is declared with the same fields regardless of
        which `mode` it actually uses -- unused fields just keep harmless
        defaults. `mode` selects how plan_arm_step() interprets them:
          - "joint": joint_positions (5 DEGREES in the yaml, converted to
            radians here) -- OMPL joint-space goal, robust but the
            Cartesian path in between is unconstrained.
          - "cartesian_relative": translation [dx, dy, dz] (meters, world
            frame) added to the arm's actual CURRENT pose (via FK), with
            orientation held fixed. Straight-line Cartesian path -- use
            this when the path shape matters (e.g. withdrawing from a
            snug hole), not just the destination.
          - "cartesian_absolute": pose [x, y, z, roll, pitch, yaw] (meters
            for xyz, DEGREES for rpy in the yaml, base_frame) --
            straight-line Cartesian path to a fixed target. Can fail on
            IK/reachability the way a joint target can't; prefer "joint"
            unless the path shape matters.

        Defaults passed in are in the same units as the yaml (degrees for
        joint_positions / pose's rpy) -- kept that way so the in-code
        fallback values read the same as what you'd type into the yaml.
        """
        mode = node.declare_parameter(f"steps.{name}.mode", default_mode).value
        joint_positions_deg = list(node.declare_parameter(
            f"steps.{name}.joint_positions", defaults.get("joint_positions", [0.0] * 5)).value)
        translation = list(node.declare_parameter(
            f"steps.{name}.translation", defaults.get("translation", [0.0, 0.0, 0.0])).value)
        pose_deg = list(node.declare_parameter(
            f"steps.{name}.pose", defaults.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])).value)
        joint_positions = [math.radians(v) for v in joint_positions_deg]
        pose = pose_deg[:3] + [math.radians(v) for v in pose_deg[3:]]
        return {"mode": mode, "joint_positions": joint_positions, "translation": translation, "pose": pose}

    steps_cfg = {
        "home": declare_arm_step("home", "joint", joint_positions=[0.0, -99.98, 85.94, 71.62, 90.0]),
        "pregrasp": declare_arm_step(
            "pregrasp", "joint", joint_positions=[-92.0, 30.0, -4.0, -26.0, 90.0]),
        "lower": declare_arm_step(
            "lower", "joint", joint_positions=[-91.0, 54.0, -9.0, -45.0, 90.0]),
        "lift": declare_arm_step("lift", "cartesian_relative", translation=[0.0, 0.0, 0.03]),
        "place": declare_arm_step(
            "place", "joint", joint_positions=[0.0, 68.0, -17.0, -51.0, 90.0]),
        "retreat": declare_arm_step(
            "retreat", "joint", joint_positions=[0.0, 29.0, 63.0, -92.0, 90.0]),
    }

    gripper_open_position = math.radians(node.declare_parameter("grasp.gripper_open_position_deg", 30.0).value)
    gripper_grasp_position = math.radians(node.declare_parameter("grasp.gripper_grasp_position_deg", -10.0).value)
    grasp_verification_enabled = node.declare_parameter("grasp_verification.enabled", True).value
    grasp_gap_threshold = node.declare_parameter("grasp_verification.gap_threshold", 0.1).value
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

    def get_current_ee_pose():
        pose_stamped = arm.compute_fk()
        if pose_stamped is None:
            return None
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def plan_arm_step(step_cfg):
        mode = step_cfg["mode"]
        if mode == "joint":
            return arm.plan(joint_positions=step_cfg["joint_positions"], joint_names=ARM_JOINT_NAMES)

        if mode == "cartesian_relative":
            current_pose = get_current_ee_pose()
            if current_pose is None:
                logger.error("Could not compute the current end-effector pose for a cartesian_relative step.")
                return None
            current_pos, current_quat = current_pose
            dx, dy, dz = step_cfg["translation"]
            target_pos = (current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz)
            return arm.plan(
                position=target_pos, quat_xyzw=current_quat, frame_id=base_frame,
                cartesian=True, max_step=0.005, cartesian_fraction_threshold=0.95,
            )

        if mode == "cartesian_absolute":
            x, y, z, roll, pitch, yaw = step_cfg["pose"]
            return arm.plan(
                position=(x, y, z), quat_xyzw=rpy_to_quat(roll, pitch, yaw), frame_id=base_frame,
                cartesian=True, max_step=0.005, cartesian_fraction_threshold=0.95,
            )

        raise ValueError(f"Unknown step mode: '{mode}' (expected joint, cartesian_relative, or cartesian_absolute)")

    def plan_gripper(position: float):
        return gripper.plan(joint_positions=[position])

    def plan_grasp_gripper():
        # Closing the gripper on the stick necessarily means the jaws end up
        # colliding with it -- that's the point of a grasp, but by default
        # the planner treats any collision as invalid and refuses to find a
        # path into it.
        #
        # This used to call MoveIt2.allow_collisions() to modify the Allowed
        # Collision Matrix instead, but that goes through a synchronous
        # Client.call() deep inside pymoveit2 (the only place in this script
        # that does, everywhere else uses the manual spin_once-loop pattern)
        # and was observed to hang indefinitely after several prior
        # plan/execute cycles -- move_group's own logs confirmed the request
        # never even arrived, and the identical call succeeded instantly in
        # an isolated repro, so the difference is specifically about this
        # long-lived process's accumulated state, not the ACM logic itself.
        # Removing the object during the close-plan sidesteps the collision
        # question entirely and only uses add/remove_collision_box, a plain
        # topic publish -- the same reliable, already-proven mechanism used
        # for the initial "Adding stick collision object" step below.
        arm.remove_collision_object(id="stick")
        trajectory = gripper.plan(joint_positions=[gripper_grasp_position])
        arm.add_collision_box(
            id="stick", size=stick_size, position=stick_pos, quat_xyzw=stick_quat, frame_id=base_frame,
        )
        return trajectory

    def attach_stick():
        logger.info("Attaching stick to the end effector.")
        arm.attach_collision_object(
            id="stick", link_name="Fixed_Gripper",
            touch_links=["Fixed_Gripper", "Moving_Jaw", "End_Effector"],
        )

    def check_grasp_success() -> bool:
        """No force/tactile sensing exists, so this is a heuristic, not
        proof: compare the gripper's actual settled position against the
        commanded closed position. Stopping well short of it means the
        jaws hit resistance -- decent evidence of a real grasp."""
        if not grasp_verification_enabled:
            return True
        state = gripper.joint_state
        if state is None or "Gripper" not in state.name:
            logger.warn("No gripper joint state available for grasp verification -- assuming success.")
            return True
        actual = state.position[state.name.index("Gripper")]
        gap = abs(actual - gripper_grasp_position)
        if gap < grasp_gap_threshold:
            logger.warn(
                f"Grasp verification: gripper reached within {gap:.3f} rad of the commanded "
                f"closed position ({gripper_grasp_position:.3f} rad) -- likely closed on nothing."
            )
            return False
        logger.info(
            f"Grasp verification: gripper stopped {gap:.3f} rad short of the commanded closed "
            "position -- likely holding the stick."
        )
        return True

    stick_pos, stick_quat = vec6_to_pos_quat(stick_pose_v)

    logger.info("Adding stick collision object to the planning scene.")
    arm.add_collision_box(
        id="stick", size=stick_size, position=stick_pos, quat_xyzw=stick_quat,
        frame_id=base_frame,
    )

    # 1. Start position is wherever the arm already is -- no action needed.
    # 2-3. Home.
    if not run_step(arm, "home position", lambda: plan_arm_step(steps_cfg["home"])):
        return shutdown(1)
    # 4-5. Open gripper.
    if not run_step(gripper, "open gripper", lambda: plan_gripper(gripper_open_position)):
        return shutdown(1)
    # 6-7. Pregrasp.
    if not run_step(arm, "move to pregrasp", lambda: plan_arm_step(steps_cfg["pregrasp"])):
        return shutdown(1)
    # 8-9. Lower to the stick, gripper still open.
    if not run_step(arm, "lower to stick", lambda: plan_arm_step(steps_cfg["lower"])):
        return shutdown(1)
    # 10-11. Grasp: close the gripper on the stick.
    if not run_step(gripper, "grasp stick (close gripper)", plan_grasp_gripper):
        return shutdown(1)

    if check_grasp_success():
        attach_stick()
    else:
        decision = prompt(
            "Grasp verification failed -- attach anyway (assume success), "
            "skip attaching (continue without it), or abort?"
        )
        if decision == "abort":
            logger.warn("Aborted by user.")
            return shutdown(1)
        if decision == "proceed":
            logger.info("Attaching stick anyway (forced by user despite failed check).")
            attach_stick()
        else:
            logger.warn("Continuing without attaching the stick -- it will not visually follow the gripper.")

    # 12-13. Lift the stick clear of the hole. steps.lift.mode defaults to
    # cartesian_relative (straight line in task space) rather than joint,
    # since the stick sits in a 5mm-deep hole and needs to pull straight
    # out -- a joint-space goal only guarantees the destination, not the
    # path shape in between, and can jam it against the hole's sides.
    if not run_step(arm, "lift stick", lambda: plan_arm_step(steps_cfg["lift"])):
        return shutdown(1)
    # 14-15. Move to the place location.
    if not run_step(arm, "place stick", lambda: plan_arm_step(steps_cfg["place"])):
        return shutdown(1)

    # Not in the original numbered list, but needed to actually release the
    # stick rather than leave it stuck in the gripper -- flag if this isn't
    # what's wanted.
    if not run_step(gripper, "open gripper at place", lambda: plan_gripper(gripper_open_position)):
        return shutdown(1)

    logger.info("Detaching stick.")
    arm.detach_collision_object(id="stick")
    current_pose = get_current_ee_pose()
    if current_pose is not None:
        pos, quat = current_pose
        arm.add_collision_box(id="stick", size=stick_size, position=pos, quat_xyzw=quat, frame_id=base_frame)
    else:
        logger.warn("Could not compute the current end-effector pose -- stick collision box not re-added.")

    run_step(arm, "retreat", lambda: plan_arm_step(steps_cfg["retreat"]))

    logger.info("Pick-and-place sequence complete.")
    shutdown(0)


if __name__ == "__main__":
    main()
