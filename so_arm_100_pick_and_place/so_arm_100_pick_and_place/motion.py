"""MoveIt2 wrapper: the one plan -> preview -> gated-prompt -> execute
implementation in this package.

Extracted from pick_and_place_node.py (ROS2_IMPLEMENTATION_PLAN.md Sec 11
Phase 2) so verify_kinematics_hardware_node.py -- which used to keep its own
inline copy of this exact pattern -- and the future task server share one
implementation instead of drifting apart.

Built on pymoveit2 (ros-humble-pymoveit2), since no compiled Python MoveIt
bindings (moveit_commander / moveit_py) are available on this ROS2 Humble
install.
"""
from threading import Thread
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

from pymoveit2 import MoveIt2, MoveIt2Gripper

import so_arm_100_kinematics as sak
from so_arm_100_pick_and_place.pose_utils import rpy_to_quat
from so_arm_100_pick_and_place.stick_spec import solve_stick_spec_joints

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


class MotionController:
    """Owns the MoveIt2/MoveIt2Gripper interfaces, the executor thread they
    need (pymoveit2's plan()/wait_until_executed() block the calling thread
    via internal rclpy.spin_once() calls, so the node must already be
    spinning on another thread or they'd deadlock waiting on their own
    callbacks), and the plan -> preview -> gated-prompt -> execute pattern
    every step goes through.

    `gripper_open_position`/`gripper_closed_position` are optional: pass
    neither when a caller only ever moves the arm (see
    verify_kinematics_hardware_node.py) and `self.gripper` stays `None`.
    """

    def __init__(self, node: Node, *, base_frame: str, velocity_scaling: float,
                 acceleration_scaling: float, planning_time: float,
                 interactive: bool, gripper_open_position: Optional[float] = None,
                 gripper_closed_position: Optional[float] = None):
        self.node = node
        self.logger = node.get_logger()
        self.base_frame = base_frame
        self.interactive = interactive

        callback_group = ReentrantCallbackGroup()
        self.arm = MoveIt2(
            node=node,
            joint_names=ARM_JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name="arm",
            callback_group=callback_group,
        )
        self.arm.max_velocity = velocity_scaling
        self.arm.max_acceleration = acceleration_scaling
        self.arm.allowed_planning_time = planning_time

        self.gripper = None
        if gripper_open_position is not None and gripper_closed_position is not None:
            self.gripper = MoveIt2Gripper(
                node=node,
                gripper_joint_names=GRIPPER_JOINT_NAMES,
                open_gripper_joint_positions=[gripper_open_position],
                closed_gripper_joint_positions=[gripper_closed_position],
                gripper_group_name="gripper",
                callback_group=callback_group,
            )
            self.gripper.allowed_planning_time = planning_time

        self.preview_pub = node.create_publisher(DisplayTrajectory, "display_planned_path", 1)

        self._executor = rclpy.executors.MultiThreadedExecutor(4)
        self._executor.add_node(node)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()
        node.create_rate(1.0).sleep()

    def shutdown(self, code: int):
        rclpy.shutdown()
        self._executor_thread.join()
        exit(code)

    def get_current_ee_pose(self):
        pose_stamped = self.arm.compute_fk()
        if pose_stamped is None:
            return None
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def plan_arm_step(self, step_cfg):
        """`step_cfg["mode"]` selects how this is interpreted:
          - "joint": step_cfg["joint_positions"] (5 radians) -- OMPL
            joint-space goal. Robust, but the Cartesian path in between is
            unconstrained.
          - "cartesian_relative": step_cfg["translation"] ([dx,dy,dz]
            meters, world frame) added to the arm's ACTUAL current pose (via
            FK), orientation held fixed. Straight-line path -- use when the
            path shape matters (e.g. withdrawing from a snug hole).
          - "cartesian_absolute": step_cfg["pose"] ([x,y,z,roll,pitch,yaw],
            meters/radians, base_frame) -- straight-line path to a fixed
            target. Can fail on IK/reachability the way a joint target
            can't; prefer "joint" unless the path shape specifically
            matters.
          - "stick_spec": step_cfg["base_xyz_m"]/["tip_xyz_m"] (meters,
            base_frame) + step_cfg["roll_deg"] -- a stick's physical ends,
            solved into a joint target via
            stick_spec.solve_stick_spec_joints() (Sec 11 Phase 3) instead of
            a hand-tuned constant, then planned as a joint goal (Phase 3's
            own instruction: prefer "joint" mode -- "by far the most
            reliable here", per finding #11).
        """
        mode = step_cfg["mode"]
        if mode == "joint":
            return self.arm.plan(joint_positions=step_cfg["joint_positions"], joint_names=ARM_JOINT_NAMES)

        if mode == "stick_spec":
            try:
                joints = solve_stick_spec_joints(
                    step_cfg["base_xyz_m"], step_cfg["tip_xyz_m"], step_cfg.get("roll_deg", 0.0))
            except sak.Unreachable as exc:
                self.logger.error(f"Stick-spec placement is unreachable either way round: {exc}")
                return None
            return self.arm.plan(joint_positions=list(joints), joint_names=ARM_JOINT_NAMES)

        if mode == "cartesian_relative":
            current_pose = self.get_current_ee_pose()
            if current_pose is None:
                self.logger.error(
                    "Could not compute the current end-effector pose for a cartesian_relative step.")
                return None
            current_pos, current_quat = current_pose
            dx, dy, dz = step_cfg["translation"]
            target_pos = (current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz)
            return self.arm.plan(
                position=target_pos, quat_xyzw=current_quat, frame_id=self.base_frame,
                cartesian=True, max_step=0.005, cartesian_fraction_threshold=0.95,
            )

        if mode == "cartesian_absolute":
            x, y, z, roll, pitch, yaw = step_cfg["pose"]
            return self.arm.plan(
                position=(x, y, z), quat_xyzw=rpy_to_quat(roll, pitch, yaw), frame_id=self.base_frame,
                cartesian=True, max_step=0.005, cartesian_fraction_threshold=0.95,
            )

        raise ValueError(
            f"Unknown step mode: '{mode}' "
            "(expected joint, cartesian_relative, cartesian_absolute, or stick_spec)")

    def plan_gripper(self, position: float):
        return self.gripper.plan(joint_positions=[position])

    def publish_preview(self, moveit_interface, trajectory: JointTrajectory):
        display = DisplayTrajectory(model_id=ROBOT_MODEL_ID)
        display.trajectory.append(RobotTrajectory(joint_trajectory=trajectory))
        current_state = moveit_interface.joint_state
        if current_state is not None:
            display.trajectory_start.joint_state = current_state
        self.preview_pub.publish(display)

    def prompt(self, message: str, allow_skip: bool = True) -> str:
        """Returns 'proceed', 'skip' (only ever if allow_skip), or 'abort'."""
        if not self.interactive:
            return "proceed"
        # Logged (not just passed to input()'s prompt arg) so the message is
        # guaranteed to appear even under `ros2 launch`, whose line-buffered,
        # per-process output prefixing never flushes an unterminated prompt
        # string. input() itself is still only reliable under `ros2 run` --
        # `ros2 launch` does not consistently forward the terminal's stdin to
        # a launched node.
        options = "Enter/e to proceed, s to skip, or q to abort" if allow_skip else "Enter/e to execute, q to abort"
        self.logger.info(f"{message} Type {options}, then press Enter.")
        while True:
            answer = input().strip().lower()
            if answer in ("", "e", "execute", "proceed"):
                return "proceed"
            if allow_skip and answer in ("s", "skip"):
                return "skip"
            if answer in ("q", "quit", "abort"):
                return "abort"
            print(f"Please enter {options}.")

    def run_step(self, moveit_interface, step_name: str,
                 plan_fn: Callable[[], Optional[JointTrajectory]], allow_skip: bool = True) -> bool:
        self.logger.info(f"Planning step '{step_name}'...")
        trajectory = plan_fn()
        if trajectory is None:
            self.logger.error(f"Planning failed at step '{step_name}'")
            return False

        self.publish_preview(moveit_interface, trajectory)
        decision = self.prompt(
            f"[{step_name}] Plan ready -- check the ghost movement preview in RViz.", allow_skip=allow_skip)
        if decision == "abort":
            self.logger.warn("Aborted by user.")
            self.shutdown(1)
        if decision == "skip":
            self.logger.warn(f"Step '{step_name}' skipped by user.")
            return True

        moveit_interface.execute(trajectory)
        ok = moveit_interface.wait_until_executed()
        if ok:
            self.logger.info(f"Step '{step_name}' OK")
        else:
            self.logger.error(f"Execution failed at step '{step_name}'")
        return ok
