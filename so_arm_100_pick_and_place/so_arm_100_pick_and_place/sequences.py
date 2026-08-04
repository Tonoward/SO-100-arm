"""The composed pick-and-place sequences, driven by a MotionController
(motion.py) and the collision-object lifecycle (scene.py).

Extracted from pick_and_place_node.py's previously-monolithic main()
(ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 2) -- no behaviour change, same
interactive demo.

`run_pick_sequence`/`run_place_sequence`/`run_release_sequence` (Sec 11
Phase 4) are the same proven steps split into three independently callable
pieces, one per task-server action (Sec 7). Unlike `run_full_demo`, none of
them ever calls `motion.shutdown()` -- a failed step in the task server
must leave the process alive and transition to ERROR (Sec 6), never exit.
They return `(success: bool, message: str)` and accept an optional
`feedback_cb(step_name, step_index, step_count)`, called before each step,
which the task server wires to `goal_handle.publish_feedback()`.
"""
import math

from so_arm_100_pick_and_place import pose_utils, scene


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
            # Matches config/pick_and_place.yaml's tuned value -- kept in
            # sync 2026-08-02 after finding stick_task_server/tune_grasp
            # were silently falling back to these code defaults instead of
            # the yaml (Sec 4 D9).
            node, "lower", "joint", joint_positions=[-92.5, 52.0, -6.0, -45.0, 90.0]),
        "lift": declare_arm_step(node, "lift", "cartesian_relative", translation=[0.0, 0.0, 0.05]),
        "place": declare_arm_step(
            node, "place", "joint", joint_positions=[0.0, 68.0, -17.0, -51.0, 90.0]),
        "retreat": declare_arm_step(
            node, "retreat", "joint", joint_positions=[0.0, -84.0, 70.0, -78.0, 90.0]),
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


def _tick(feedback_cb, step_name, index, count):
    if feedback_cb is not None:
        feedback_cb(step_name, index, count)


def run_pick_sequence(motion, steps_cfg, stick_cfg, grasp_cfg, length_m=None, feedback_cb=None):
    """Sec 7.1's proven pick steps (home -> open -> pregrasp -> lower ->
    grasp+verify+attach -> lift), ending wherever `lift` leaves the arm.
    Returns (success, message); never calls motion.shutdown().

    `stick_cfg`: {"base_xyz_m", "section_m", "default_length_m"}.
    `base_xyz_m` is the fed stick's PHYSICAL BASE (bottom, at the feeder
    hole) -- fixed regardless of stick length, same convention as
    StickSpec.base. The collision box's actual pose is computed from this
    base plus the stick's LENGTH (pose_utils.feeder_stick_pose), so a
    longer or shorter stick still sits base-down in the same real spot.
    `length_m`, if given (the task server passes PickStick.length), is the
    length actually used; otherwise falls back to `stick_cfg["default_length_m"]`
    (the standalone demo/tune_grasp, which have no per-goal length).

    ⚠ Deliberately does not add Sec 7.1 step 8's distinct `standby_holding`
    joint pose -- that constant has never been tuned or hardware-tested, so
    inventing one now would be an untested new behaviour, not a reuse of a
    proven one. HOLDING is therefore "wherever `lift` left the arm" for
    this phase, not a dedicated standby position.

    ⚠ On a failed grasp verification: when `motion.interactive` (the
    standalone demo), this offers the exact same attach-anyway/skip/abort
    prompt as before the Phase 2 refactor. When not interactive (the task
    server, always), there is no one to answer that prompt --
    `motion.prompt` would silently auto-answer "proceed" and force-attach
    an empty gripper, misreporting HOLDING. So in that case this is a hard
    failure instead: the caller sees it and can decide via a future
    retry/reset action, never a blocking terminal prompt nobody can answer.
    """
    logger = motion.logger
    arm = motion.arm
    gripper = motion.gripper
    step_count = 6

    length = length_m if length_m else stick_cfg["default_length_m"]
    size = (stick_cfg["section_m"][0], stick_cfg["section_m"][1], length)
    feeder_pos, feeder_quat = pose_utils.feeder_stick_pose(stick_cfg["base_xyz_m"], length)

    def plan_grasp_gripper():
        # See the module's own history: MoveIt2.allow_collisions() hangs in
        # this long-lived process; remove/re-add (a plain topic publish) is
        # the proven-reliable way to let the jaws close on the stick.
        scene.remove_stick(arm)
        trajectory = gripper.plan(joint_positions=[grasp_cfg["grasp_position"]])
        # Re-add at the arm's ACTUAL current EE pose (FK), not the static
        # feeder-pose config constant -- attach_collision_object() attaches
        # whatever pose "stick" has in the scene AT THAT MOMENT (Sec 4 D8),
        # so any gap between the config constant and where the arm truly is
        # becomes a fixed, silently-wrong offset that gets amplified across
        # the much larger pick->place joint move. Same technique already
        # proven correct for `register_placed_stick` below.
        current_pose = motion.get_current_ee_pose()
        if current_pose is not None:
            pos, quat = current_pose
        else:
            pos, quat = feeder_pos, feeder_quat
        scene.add_stick_at_feeder(arm, size, pos, quat, motion.base_frame)
        return trajectory

    logger.info("Adding stick collision object to the planning scene.")
    scene.add_stick_at_feeder(arm, size, feeder_pos, feeder_quat, motion.base_frame)

    _tick(feedback_cb, "home position", 0, step_count)
    if not motion.run_step(arm, "home position", lambda: motion.plan_arm_step(steps_cfg["home"])):
        return False, "planning/execution failed at 'home position'"

    _tick(feedback_cb, "open gripper", 1, step_count)
    if not motion.run_step(gripper, "open gripper", lambda: motion.plan_gripper(grasp_cfg["open_position"])):
        return False, "planning/execution failed at 'open gripper'"

    _tick(feedback_cb, "move to pregrasp", 2, step_count)
    if not motion.run_step(arm, "move to pregrasp", lambda: motion.plan_arm_step(steps_cfg["pregrasp"])):
        return False, "planning/execution failed at 'move to pregrasp'"

    _tick(feedback_cb, "lower to stick", 3, step_count)
    if not motion.run_step(arm, "lower to stick", lambda: motion.plan_arm_step(steps_cfg["lower"])):
        return False, "planning/execution failed at 'lower to stick'"

    _tick(feedback_cb, "grasp stick (close gripper)", 4, step_count)
    if not motion.run_step(gripper, "grasp stick (close gripper)", plan_grasp_gripper):
        # plan_grasp_gripper() re-adds "stick" at the gripper's current pose
        # regardless of whether planning/execution succeeded (needed so the
        # box is there for the verification-success attach below), which
        # means a failure here leaves it sitting exactly where the robot
        # now is -- a collision object overlapping the CURRENT state, which
        # then blocks planning ANY next move (confirmed on hardware: this is
        # what forced manually removing it before the arm could even be
        # driven back to home). Remove it so recovery isn't blocked.
        scene.remove_stick(arm)
        return False, "planning/execution failed at 'grasp stick (close gripper)'"

    if check_grasp_success(gripper, logger, grasp_cfg["grasp_position"], grasp_cfg["gap_threshold"],
                           grasp_cfg["verification_enabled"]):
        scene.attach_stick(arm, logger)
    elif motion.interactive:
        # Exactly today's demo behaviour, unchanged by the Phase 4 split.
        decision = motion.prompt(
            "Grasp verification failed -- attach anyway (assume success), "
            "skip attaching (continue without it), or abort?"
        )
        if decision == "abort":
            scene.remove_stick(arm)  # same reasoning as above
            return False, "aborted by user after failed grasp verification"
        if decision == "proceed":
            logger.info("Attaching stick anyway (forced by user despite failed check).")
            scene.attach_stick(arm, logger)
        else:
            logger.warn("Continuing without attaching the stick -- it will not visually follow the gripper.")
    else:
        # No one to answer the prompt above -- motion.prompt() would
        # silently auto-answer "proceed" and force-attach. Hard-fail instead,
        # and remove "stick" (same reasoning as the motion-failure branch
        # above) so a manual recovery move isn't blocked by a stale
        # collision object sitting on top of the current robot state.
        scene.remove_stick(arm)
        return False, "grasp verification failed -- gripper likely closed on nothing"

    _tick(feedback_cb, "lift stick", 5, step_count)
    if not motion.run_step(arm, "lift stick", lambda: motion.plan_arm_step(steps_cfg["lift"])):
        return False, "planning/execution failed at 'lift stick'"

    return True, "holding stick"


def run_place_sequence(motion, stick_spec, feedback_cb=None):
    """Sec 7.2: solve the stick's base/tip/roll via motion.plan_arm_step's
    `stick_spec` mode (Sec 11 Phase 3 -- includes the base/tip flip-retry
    and roll_deg handling, never reimplemented here) and plan it as a joint
    goal. Ends AT_PLACE, gripper still closed -- do NOT open it here.

    `stick_spec`: {"base_xyz_m", "tip_xyz_m", "roll_deg"}.

    ⚠ Deliberately does not implement Sec 7.2 steps 3-4's distinct
    approach-from-above + straight-down-insert cartesian path -- that is a
    real refinement with its own clearance constant to tune and test on
    hardware, deferred to a follow-up. This reuses Phase 3's single
    joint-move `place`, already hardware-proven.
    """
    step_cfg = {
        "mode": "stick_spec",
        "base_xyz_m": stick_spec["base_xyz_m"],
        "tip_xyz_m": stick_spec["tip_xyz_m"],
        "roll_deg": stick_spec.get("roll_deg", 0.0),
    }
    _tick(feedback_cb, "place stick", 0, 1)
    if not motion.run_step(motion.arm, "place stick", lambda: motion.plan_arm_step(step_cfg)):
        return False, "planning/execution failed at 'place stick' (see log for the unreachable reason, if any)"
    return True, "at place, gripper still closed"


def run_release_sequence(motion, stick_id, size, frame_id, grasp_cfg, retreat_step_cfg, feedback_cb=None):
    """Sec 7.3: open the gripper, detach the held stick, register it as a
    PERMANENT collision object `placed_<stick_id>` at its current pose
    (Sec 10 -- the sculpture is an obstacle field from here on), then
    retreat. Ends IDLE.

    `size`: the collision-box size to register the placed stick under --
    this phase's PickStick/ReleaseStick actions don't yet round-trip a full
    StickSpec's `section_m` between Pick and Release (Phase 5's build-file
    loader will), so callers pass a configured default (Sec 11 Phase 4
    scope note).

    ⚠ Deliberately reuses today's single `retreat` joint step for both
    Sec 7.3 step 4 (a distinct straight-up cartesian `withdraw`) and step 5
    (`standby`) -- the same "don't invent an untested step" principle as
    `run_pick_sequence`.
    """
    logger = motion.logger
    arm = motion.arm
    gripper = motion.gripper
    step_count = 3

    _tick(feedback_cb, "open gripper at place", 0, step_count)
    if not motion.run_step(gripper, "open gripper at place",
                           lambda: motion.plan_gripper(grasp_cfg["open_position"])):
        return False, "planning/execution failed at 'open gripper at place'"

    scene.detach_stick(arm, logger)
    # detach_collision_object() (Sec 4 D8) returns "stick" to the world at
    # its attach-time-derived pose rather than deleting it -- leaving it
    # behind as a stray obstacle wherever that pose lands, which has caused
    # a real planning failure at 'retreat' (confirmed on hardware
    # 2026-08-02). `placed_<stick_id>` below is the authoritative record of
    # this stick from here on, so the transient proxy has no further job.
    scene.remove_stick(arm)
    current_pose = motion.get_current_ee_pose()
    if current_pose is None:
        return False, "could not compute the current end-effector pose -- placed stick not registered"
    pos, quat = current_pose
    _tick(feedback_cb, "register placed stick", 1, step_count)
    scene.register_placed_stick(arm, stick_id, size, pos, quat, frame_id)

    _tick(feedback_cb, "retreat", 2, step_count)
    if not motion.run_step(arm, "retreat", lambda: motion.plan_arm_step(retreat_step_cfg)):
        return False, "planning/execution failed at 'retreat'"

    return True, "released, idle"


def run_full_demo(motion, steps_cfg, stick_cfg, grasp_cfg):
    """The standalone interactive demo (pick_and_place_node.py): the three
    sequences above run back to back with no pause between place and
    release, and a failure at any step calls motion.shutdown(1) -- exactly
    today's behaviour (ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 2's
    zero-behaviour-change guarantee), now expressed as a thin wrapper
    instead of one monolithic function.

    Note `steps_cfg["place"]` is planned directly via `motion.plan_arm_step`
    here, not through `run_place_sequence` -- the demo's `place` step can be
    ANY of the four step modes (Sec 11 Phase 3 added `stick_spec` as one
    more option alongside `joint`/`cartesian_relative`/`cartesian_absolute`,
    switchable by a pure yaml edit), while `run_place_sequence` is
    specifically the task server's StickSpec-goal-driven path (Sec 7.2)."""
    ok, message = run_pick_sequence(motion, steps_cfg, stick_cfg, grasp_cfg)
    if not ok:
        motion.logger.error(message)
        return motion.shutdown(1)

    if not motion.run_step(motion.arm, "place stick", lambda: motion.plan_arm_step(steps_cfg["place"])):
        motion.logger.error("planning/execution failed at 'place stick'")
        return motion.shutdown(1)

    placed_size = (stick_cfg["section_m"][0], stick_cfg["section_m"][1], stick_cfg["default_length_m"])
    ok, message = run_release_sequence(
        motion, stick_id="stick", size=placed_size, frame_id=motion.base_frame,
        grasp_cfg=grasp_cfg, retreat_step_cfg=steps_cfg["retreat"])
    if not ok:
        motion.logger.error(message)
        return motion.shutdown(1)

    motion.logger.info("Pick-and-place sequence complete.")
    motion.shutdown(0)
