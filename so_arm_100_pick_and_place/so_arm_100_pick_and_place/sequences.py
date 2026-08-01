"""The composed pick-and-place sequence: today's fixed sequence of named
steps, driven by a MotionController (motion.py) and the collision-object
lifecycle (scene.py).

Extracted from pick_and_place_node.py's previously-monolithic main()
(ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 2) -- no behaviour change, same
interactive demo.
"""
import math

from so_arm_100_pick_and_place import scene


def declare_arm_step(node, name: str, default_mode: str, **defaults):
    """Every arm step is declared with the same fields regardless of which
    `mode` it actually uses -- unused fields just keep harmless defaults.
    See motion.MotionController.plan_arm_step for what each mode consumes.

    Defaults passed in are in the same units as the yaml (degrees for
    joint_positions / pose's rpy / roll_deg) -- kept that way so the
    in-code fallback values read the same as what you'd type into the
    yaml.
    """
    mode = node.declare_parameter(f"steps.{name}.mode", default_mode).value
    joint_positions_deg = list(node.declare_parameter(
        f"steps.{name}.joint_positions", defaults.get("joint_positions", [0.0] * 5)).value)
    translation = list(node.declare_parameter(
        f"steps.{name}.translation", defaults.get("translation", [0.0, 0.0, 0.0])).value)
    pose_deg = list(node.declare_parameter(
        f"steps.{name}.pose", defaults.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])).value)
    base_xyz_m = tuple(node.declare_parameter(
        f"steps.{name}.base_xyz_m", defaults.get("base_xyz_m", [0.0, 0.0, 0.0])).value)
    tip_xyz_m = tuple(node.declare_parameter(
        f"steps.{name}.tip_xyz_m", defaults.get("tip_xyz_m", [0.0, 0.0, 0.0])).value)
    roll_deg = node.declare_parameter(
        f"steps.{name}.roll_deg", defaults.get("roll_deg", 0.0)).value
    joint_positions = [math.radians(v) for v in joint_positions_deg]
    pose = pose_deg[:3] + [math.radians(v) for v in pose_deg[3:]]
    return {
        "mode": mode, "joint_positions": joint_positions, "translation": translation, "pose": pose,
        "base_xyz_m": base_xyz_m, "tip_xyz_m": tip_xyz_m, "roll_deg": roll_deg,
    }


def declare_all_steps(node):
    return {
        "home": declare_arm_step(node, "home", "joint", joint_positions=[0.0, -99.98, 85.94, 71.62, 90.0]),
        "pregrasp": declare_arm_step(
            node, "pregrasp", "joint", joint_positions=[-92.0, 30.0, -4.0, -26.0, 90.0]),
        "lower": declare_arm_step(
            node, "lower", "joint", joint_positions=[-91.0, 54.0, -9.0, -45.0, 90.0]),
        "lift": declare_arm_step(node, "lift", "cartesian_relative", translation=[0.0, 0.0, 0.03]),
        "place": declare_arm_step(
            node, "place", "joint", joint_positions=[0.0, 68.0, -17.0, -51.0, 90.0]),
        "retreat": declare_arm_step(
            node, "retreat", "joint", joint_positions=[0.0, 29.0, 63.0, -92.0, 90.0]),
    }


def check_grasp_success(gripper, logger, grasp_position: float, gap_threshold: float, enabled: bool) -> bool:
    """No force/tactile sensing exists, so this is a heuristic, not proof:
    compare the gripper's actual settled position against the commanded
    closed position. Stopping well short of it means the jaws hit
    resistance -- decent evidence of a real grasp."""
    if not enabled:
        return True
    state = gripper.joint_state
    if state is None or "Gripper" not in state.name:
        logger.warn("No gripper joint state available for grasp verification -- assuming success.")
        return True
    actual = state.position[state.name.index("Gripper")]
    gap = abs(actual - grasp_position)
    if gap < gap_threshold:
        logger.warn(
            f"Grasp verification: gripper reached within {gap:.3f} rad of the commanded "
            f"closed position ({grasp_position:.3f} rad) -- likely closed on nothing."
        )
        return False
    logger.info(
        f"Grasp verification: gripper stopped {gap:.3f} rad short of the commanded closed "
        "position -- likely holding the stick."
    )
    return True


def run_full_demo(motion, steps_cfg, stick_cfg, grasp_cfg):
    """stick_cfg: {"size", "position", "quat_xyzw"} for the fed stick's
    collision box. grasp_cfg: {"open_position", "grasp_position",
    "verification_enabled", "gap_threshold"} (radians)."""
    logger = motion.logger
    arm = motion.arm
    gripper = motion.gripper

    def plan_grasp_gripper():
        # Closing the gripper on the stick necessarily means the jaws end
        # up colliding with it -- that's the point of a grasp, but by
        # default the planner treats any collision as invalid and refuses
        # to find a path into it.
        #
        # This used to call MoveIt2.allow_collisions() to modify the
        # Allowed Collision Matrix instead, but that goes through a
        # synchronous Client.call() deep inside pymoveit2 (the only place
        # that did, everywhere else uses the manual spin_once-loop pattern)
        # and was observed to hang indefinitely after several prior
        # plan/execute cycles -- move_group's own logs confirmed the
        # request never even arrived, and the identical call succeeded
        # instantly in an isolated repro, so the difference is specifically
        # about this long-lived process's accumulated state, not the ACM
        # logic itself. Removing the object during the close-plan
        # sidesteps the collision question entirely and only uses
        # add/remove_collision_box, a plain topic publish -- the same
        # reliable mechanism used for the initial "Adding stick collision
        # object" step below.
        scene.remove_stick(arm)
        trajectory = gripper.plan(joint_positions=[grasp_cfg["grasp_position"]])
        scene.add_stick_at_feeder(arm, stick_cfg["size"], stick_cfg["position"], stick_cfg["quat_xyzw"],
                                  motion.base_frame)
        return trajectory

    logger.info("Adding stick collision object to the planning scene.")
    scene.add_stick_at_feeder(arm, stick_cfg["size"], stick_cfg["position"], stick_cfg["quat_xyzw"],
                              motion.base_frame)

    # 1. Start position is wherever the arm already is -- no action needed.
    # 2-3. Home.
    if not motion.run_step(arm, "home position", lambda: motion.plan_arm_step(steps_cfg["home"])):
        return motion.shutdown(1)
    # 4-5. Open gripper.
    if not motion.run_step(gripper, "open gripper", lambda: motion.plan_gripper(grasp_cfg["open_position"])):
        return motion.shutdown(1)
    # 6-7. Pregrasp.
    if not motion.run_step(arm, "move to pregrasp", lambda: motion.plan_arm_step(steps_cfg["pregrasp"])):
        return motion.shutdown(1)
    # 8-9. Lower to the stick, gripper still open.
    if not motion.run_step(arm, "lower to stick", lambda: motion.plan_arm_step(steps_cfg["lower"])):
        return motion.shutdown(1)
    # 10-11. Grasp: close the gripper on the stick.
    if not motion.run_step(gripper, "grasp stick (close gripper)", plan_grasp_gripper):
        return motion.shutdown(1)

    if check_grasp_success(gripper, logger, grasp_cfg["grasp_position"], grasp_cfg["gap_threshold"],
                            grasp_cfg["verification_enabled"]):
        scene.attach_stick(arm, logger)
    else:
        decision = motion.prompt(
            "Grasp verification failed -- attach anyway (assume success), "
            "skip attaching (continue without it), or abort?"
        )
        if decision == "abort":
            logger.warn("Aborted by user.")
            return motion.shutdown(1)
        if decision == "proceed":
            logger.info("Attaching stick anyway (forced by user despite failed check).")
            scene.attach_stick(arm, logger)
        else:
            logger.warn("Continuing without attaching the stick -- it will not visually follow the gripper.")

    # 12-13. Lift the stick clear of the hole. steps.lift.mode defaults to
    # cartesian_relative (straight line in task space) rather than joint,
    # since the stick sits in a 5mm-deep hole and needs to pull straight
    # out -- a joint-space goal only guarantees the destination, not the
    # path shape in between, and can jam it against the hole's sides.
    if not motion.run_step(arm, "lift stick", lambda: motion.plan_arm_step(steps_cfg["lift"])):
        return motion.shutdown(1)
    # 14-15. Move to the place location.
    if not motion.run_step(arm, "place stick", lambda: motion.plan_arm_step(steps_cfg["place"])):
        return motion.shutdown(1)

    # Not in the original numbered list, but needed to actually release the
    # stick rather than leave it stuck in the gripper -- flag if this isn't
    # what's wanted.
    if not motion.run_step(gripper, "open gripper at place", lambda: motion.plan_gripper(grasp_cfg["open_position"])):
        return motion.shutdown(1)

    scene.detach_stick(arm, logger)
    current_pose = motion.get_current_ee_pose()
    if current_pose is not None:
        pos, quat = current_pose
        scene.re_add_stick(arm, stick_cfg["size"], pos, quat, motion.base_frame)
    else:
        logger.warn("Could not compute the current end-effector pose -- stick collision box not re-added.")

    motion.run_step(arm, "retreat", lambda: motion.plan_arm_step(steps_cfg["retreat"]))

    logger.info("Pick-and-place sequence complete.")
    motion.shutdown(0)
