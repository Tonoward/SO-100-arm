#!/usr/bin/env python3
"""The build execution loop (ROS2_IMPLEMENTATION_PLAN.md Sec 11 Phase 5,
BRIDGE_PROTOCOL.md Sec A.4/A.5): load a Blender-exported build file and run
it stick by stick against `stick_task_server`, with the human gates the
protocol calls for.

This is an **ActionClient** of `stick_task_server` -- a separate process,
not a change to the task server itself. `stick_task_server` stays the same
single source of truth for the state machine; this node just drives it.

⚠ **Simpler than `stick_task_server_node.py` on purpose**: this node never
calls `pymoveit2`'s blocking `plan()`/`execute()`/`wait_until_executed()`
(the specific calls that trigger finding #12's executor-detachment wedge),
so it does NOT need that node's two-`Node`/two-`Executor` split for THAT
reason. It still uses two `Node`s, but for an unrelated, ordinary reason:
one (`_motion_node`) is handed to a `MotionController` purely so its
`.arm.add_collision_box()` is available for rebuilding the scene on resume
(Sec A.5) -- `add_collision_box` is a plain topic publish (finding #7), not
a blocking call, so this carries none of the wedge risk either; it's just
reusing `MotionController`'s already-proven node/publisher/spin-up pattern
instead of hand-rolling a thinner one. The other (`_action_node`) holds the
four `ActionClient`s and is spun on demand via
`rclpy.spin_until_future_complete()` per goal -- an ordinary, long-proven
rclpy pattern for a top-level script that was never at risk from finding
#12 in the first place (that bug is specifically about a callback nested
inside an Executor's OWN dispatch, and nothing here runs as such a
callback).

Run alongside the same hardware bringup as the other pick-and-place nodes,
plus `stick_task_server` itself:
    ros2 launch so_arm_100_moveit_config pickandplace_demo.launch.py
    ros2 launch so_arm_100_pick_and_place stick_task_server.launch.py
Then, in a third terminal (`ros2 run`, not `ros2 launch`, for reliable
stdin -- finding #10):
    ros2 run so_arm_100_pick_and_place build_runner --ros-args \\
        -p build_file:=/path/to/your.build.json

Resumes automatically from `<build_file>.status.json` if one exists next to
the build file (Sec A.3/A.5) -- that's the intended checkpoint feature, not
a stale cache, and it survives closing every terminal since it's just a
file on disk. Pass `-p fresh_start:=true` to ignore it and start over.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from so_arm_100_stick_msgs.action import PickStick, PlaceStick, ReleaseStick, Reset
from so_arm_100_stick_msgs.msg import StickSpec

from so_arm_100_pick_and_place import build_file, pose_utils, scene
from so_arm_100_pick_and_place.motion import MotionController


class ActionCallFailed(Exception):
    """A goal was rejected or finished unsuccessfully. `needs_reset` is True
    only when the goal was ACCEPTED and then failed (a real ERROR
    transition on the server) -- a flat rejection never changes the
    server's state, so Reset would itself be rejected too."""

    def __init__(self, message, needs_reset):
        super().__init__(message)
        self.message = message
        self.needs_reset = needs_reset


def _feedback_logger(logger):
    def _cb(feedback_msg):
        fb = feedback_msg.feedback
        logger.info(f"    step: {fb.step} ({fb.step_index + 1}/{fb.step_count})")
    return _cb


def call_action(node, logger, client, action_name, goal, with_feedback=True):
    """Send one goal, block for the result, raise ActionCallFailed on
    anything but a clean success. Standard synchronous ActionClient usage
    -- see the module docstring for why this is safe here."""
    if not client.wait_for_server(timeout_sec=10.0):
        raise ActionCallFailed(f"{action_name}: action server not available", needs_reset=False)

    feedback_cb = _feedback_logger(logger) if with_feedback else None
    send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        raise ActionCallFailed(f"{action_name}: goal rejected by stick_task_server", needs_reset=False)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    if not result.success:
        raise ActionCallFailed(f"{action_name}: {result.message}", needs_reset=True)
    return result


def prompt(logger, message: str):
    """Same logger.info + input() gate as everywhere else in this package
    (finding #10) -- logged, not passed to input()'s own prompt argument,
    so it's guaranteed to flush under `ros2 run`."""
    logger.info(f"{message} Press Enter to continue.")
    input()


def prompt_failure_recovery(logger) -> str:
    """Returns 'retry', 'skip', or 'abort'."""
    logger.warn("Options: [r]etry, [s]kip this stick, [a]bort the build.")
    while True:
        answer = input("retry/skip/abort: ").strip().lower()
        if answer in ("r", "retry"):
            return "retry"
        if answer in ("s", "skip"):
            return "skip"
        if answer in ("a", "abort"):
            return "abort"
        print("Please enter r, s, or a.")


def prompt_stick_physically_placed(logger) -> bool:
    """Asked before finalizing a skip/abort: a failure can happen AFTER the
    stick is already physically glued in place (e.g. ReleaseStick failing
    at its 'open gripper at place' step, before run_release_sequence's own
    register_placed_stick call -- see sequences.py) -- ground truth the
    operator can see and this process cannot. Answering yes overrides the
    skip/abort decision's status: the stick is registered as a permanent
    collision object and marked placed instead of skipped/failed, so later
    sticks in this build plan around it correctly."""
    logger.warn(
        "Before finalizing: check the sculpture -- did this stick actually end up "
        "glued in its final place, despite the reported failure?")
    while True:
        answer = input("stick physically placed? [y/n]: ").strip().lower()
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please enter y or n.")


def reset_task_server(node, logger, reset_client):
    logger.warn(
        "A step failed. Visually check the robot -- is the gripper actually empty right now?")
    while True:
        answer = input("gripper empty? [y/n]: ").strip().lower()
        if answer in ("y", "yes"):
            assume_empty = True
            break
        if answer in ("n", "no"):
            assume_empty = False
            break
        print("Please enter y or n.")
    reset_client_result = call_action(
        node, logger, reset_client, "Reset", Reset.Goal(assume_gripper_empty=assume_empty),
        with_feedback=False)
    logger.info(reset_client_result.message)


def stick_spec_msg(stick, section_m):
    return StickSpec(
        id=stick["id"],
        base=[float(v) for v in stick["base"]],
        tip=[float(v) for v in stick["tip"]],
        roll_deg=float(stick.get("roll_deg", 0.0)),
        length_m=float(stick["length_m"]),
        section_m=[float(v) for v in section_m],
    )


def main():
    rclpy.init()
    action_node = Node("build_runner")
    motion_node = Node("build_runner_motion")

    build_file_path = action_node.declare_parameter("build_file", "").value
    if not build_file_path:
        action_node.get_logger().error(
            "Required parameter 'build_file' not set -- pass --ros-args -p build_file:=/path/to/x.build.json")
        rclpy.shutdown()
        return
    feeder_source = action_node.declare_parameter("feeder_source", "feeder_0").value
    base_frame = action_node.declare_parameter("base_frame", "base_link").value
    fresh_start = action_node.declare_parameter("fresh_start", False).value

    logger = action_node.get_logger()

    with open(build_file_path, "r", encoding="utf-8") as handle:
        try:
            document = build_file.load_build_file(handle.read())
        except build_file.ProtocolError as exc:
            logger.error(f"Refusing to run: {exc}")
            rclpy.shutdown()
            return
    sticks_by_id = {stick["id"]: stick for stick in document["sticks"]}
    ordered_ids = [stick["id"] for stick in document["sticks"]]
    section_m = document["stock"]["section_m"]
    logger.info(f"Loaded build file: {len(ordered_ids)} sticks, kinematics_version {document['kinematics_version']}.")

    status_file_path = build_file.status_path_for(build_file_path)
    statuses = {}
    current_index = 0
    if fresh_start:
        # The status sidecar is the checkpoint (Sec A.3) -- it's a plain
        # file next to the build file, so it outlives this process and gets
        # picked back up by the NEXT build_runner run even after every
        # terminal involved was closed and everything rebuilt/resourced.
        # That's the resume feature working as designed, not a stale cache
        # -- but sometimes you really do want to redo everything, so this
        # flag skips loading it. write_status() below still overwrites the
        # file on disk with this run's own progress, so the old contents
        # are gone after the first stick finishes either way.
        logger.info("fresh_start:=true -- ignoring any existing status sidecar, starting from the beginning.")
    else:
        try:
            with open(status_file_path, "r", encoding="utf-8") as handle:
                current_index, statuses = build_file.load_status_file(handle.read())
            logger.info(f"Resuming from status sidecar: {status_file_path}")
        except FileNotFoundError:
            logger.info("No status sidecar found -- starting a fresh build.")

    # Motion-only node, purely for scene.register_placed_stick() below --
    # see the module docstring for why this doesn't need the two-node wedge
    # workaround (add_collision_box is a plain publish, not a blocking call).
    motion = MotionController(
        motion_node, base_frame=base_frame, velocity_scaling=0.4, acceleration_scaling=0.4,
        planning_time=5.0, interactive=False,
    )

    # The planning scene lives in move_group, not in this process -- killing
    # and restarting build_runner/stick_task_server does NOT clear whatever
    # a previous crashed/aborted run left behind (confirmed: a user report
    # of a stale collision object surviving a restart, only cleared by also
    # closing RViz/move_group). Clear deterministically by id before
    # rebuilding from the sidecar below, rather than relying on a fresh
    # move_group process: drop the transient feeder "stick" (in case a
    # previous run left it attached or sitting in the world -- Sec 4 D8's
    # stray-object failure mode) and any placed_<id> for a stick this
    # sidecar does not currently call placed (e.g. one that was placed in
    # an older sidecar but is now failed/skipped/absent).
    scene.detach_stick(motion.arm, logger)
    scene.remove_stick(motion.arm)
    for stick_id in ordered_ids:
        entry = statuses.get(stick_id)
        if entry is None or entry.get("status") != build_file.STATUS_PLACED:
            motion.arm.remove_collision_object(id=f"placed_{stick_id}")

    # Sec A.5: rebuild the scene from every already-placed stick before
    # touching anything, purely from the build file's own base/tip geometry
    # (there's no live FK for a stick placed in a previous run).
    placed_count = 0
    for stick_id, entry in statuses.items():
        if entry.get("status") != build_file.STATUS_PLACED:
            continue
        stick = sticks_by_id.get(stick_id)
        if stick is None:
            continue
        pos, quat = pose_utils.axis_aligned_box_pose(tuple(stick["base"]), tuple(stick["tip"]))
        size = (section_m[0], section_m[1], stick["length_m"])
        scene.register_placed_stick(motion.arm, stick_id, size, pos, quat, base_frame)
        placed_count += 1
    if placed_count:
        logger.info(f"Resume: re-registered {placed_count} already-placed stick(s) in the planning scene.")

    pick_client = ActionClient(action_node, PickStick, "pick_stick")
    place_client = ActionClient(action_node, PlaceStick, "place_stick")
    release_client = ActionClient(action_node, ReleaseStick, "release_stick")
    reset_client = ActionClient(action_node, Reset, "reset")

    placed, failed, skipped = [], [], []

    def write_status():
        doc = build_file.status_document(build_file_path, statuses, current_index=current_index)
        build_file.write_status_file(status_file_path, doc)

    # next_stick_to_load is used ONCE here to find the resume point (Sec
    # A.5) -- it deliberately does not skip past a failed/skipped stick, so
    # calling it again after an in-session "skip" would just return the
    # SAME stick forever. Once we know where to start, plain array-position
    # iteration drives the rest of this run; a skip/placed outcome both
    # advance the `for` loop naturally, and "retry" only loops the INNER
    # while below, never touching this one.
    start_id = build_file.next_stick_to_load(ordered_ids, statuses)
    if start_id is None:
        logger.info("Every stick in this build file is already placed -- nothing to do.")
        _print_summary(logger, placed, failed, skipped)
        motion.shutdown(0)
        return
    start_index = ordered_ids.index(start_id)

    for current_index in range(start_index, len(ordered_ids)):
        stick_id = ordered_ids[current_index]
        stick = sticks_by_id[stick_id]

        # Array-position iteration (not a repeated next_stick_to_load call --
        # see the comment above) walks every stick from start_index onward
        # unconditionally, so a stick already `placed` from an EARLIER
        # session's sidecar (re-registered as a permanent placed_<id> box by
        # the resume block above) must be skipped here explicitly, or this
        # loop re-attempts PickStick/PlaceStick on it and drives a second
        # physical stick straight at the collision box already occupying its
        # target pose -- a real, valid planning failure, not a stale-scene
        # one. Confirmed on hardware 2026-08-04: s_005 was already `placed`
        # from a prior session, got re-picked and re-placed here anyway, and
        # PlaceStick failed because its own target pose was already occupied
        # by its own already-registered collision box.
        if statuses.get(stick_id, {}).get("status") == build_file.STATUS_PLACED:
            logger.info(f"'{stick_id}' is already placed (from a previous session) -- skipping.")
            continue

        length_mm = stick["length_m"] * 1000.0

        outcome = None  # 'placed', 'failed', or 'skipped'
        while outcome is None:
            prompt(logger, f"Load stick '{stick_id}', length {length_mm:.1f} mm, into the feeder ({feeder_source}).")
            try:
                call_action(
                    action_node, logger, pick_client, "PickStick",
                    PickStick.Goal(stick_id=stick_id, length=stick["length_m"], source=feeder_source))

                prompt(logger, f"Ready to place '{stick_id}'.")
                call_action(
                    action_node, logger, place_client, "PlaceStick",
                    PlaceStick.Goal(target=stick_spec_msg(stick, section_m), approach_from_above=False))

                prompt(logger, f"Glue '{stick_id}' now.")
                call_action(
                    action_node, logger, release_client, "ReleaseStick", ReleaseStick.Goal(stick_id=stick_id))

                statuses[stick_id] = {"status": build_file.STATUS_PLACED, "at": "", "reason": ""}
                outcome = "placed"
                placed.append(stick_id)
            except ActionCallFailed as exc:
                logger.error(str(exc))
                if exc.needs_reset:
                    reset_task_server(action_node, logger, reset_client)
                decision = prompt_failure_recovery(logger)
                if decision == "retry":
                    continue

                # Ground truth check before finalizing -- see
                # prompt_stick_physically_placed's docstring: a failure can
                # land after the stick is already glued in place, and
                # skip/abort must not silently drop it from the scene.
                if prompt_stick_physically_placed(logger):
                    pos, quat = pose_utils.axis_aligned_box_pose(
                        tuple(float(v) for v in stick["base"]), tuple(float(v) for v in stick["tip"]))
                    size = (section_m[0], section_m[1], stick["length_m"])
                    scene.register_placed_stick(motion.arm, stick_id, size, pos, quat, base_frame)
                    statuses[stick_id] = {"status": build_file.STATUS_PLACED, "at": "", "reason": ""}
                    outcome = "placed"
                    placed.append(stick_id)
                    logger.info(f"Registered '{stick_id}' as placed in the scene despite the reported failure.")
                elif decision == "skip":
                    statuses[stick_id] = {"status": build_file.STATUS_SKIPPED, "at": "", "reason": str(exc)}
                    outcome = "skipped"
                    skipped.append(stick_id)
                else:
                    statuses[stick_id] = {"status": build_file.STATUS_FAILED, "at": "", "reason": str(exc)}
                    outcome = "failed"
                    failed.append(stick_id)

                if decision == "abort":
                    write_status()
                    logger.error("Build aborted by user.")
                    _print_summary(logger, placed, failed, skipped)
                    motion.shutdown(1)

        write_status()  # A.3: rewritten after every finalized stick.

    _print_summary(logger, placed, failed, skipped)
    motion.shutdown(0)


def _print_summary(logger, placed, failed, skipped):
    logger.info(
        f"Build finished. placed={len(placed)} failed={len(failed)} skipped={len(skipped)}")
    if failed:
        logger.warn(f"Failed sticks: {', '.join(failed)}")
    if skipped:
        logger.warn(f"Skipped sticks: {', '.join(skipped)}")


if __name__ == "__main__":
    main()
