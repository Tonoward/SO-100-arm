#!/usr/bin/env python3
"""The task server (ROS2_IMPLEMENTATION_PLAN.md Sec 5.1/6/7/11 Phase 4):
PickStick/PlaceStick/ReleaseStick as ROS actions, gated by a state machine
instead of a terminal prompt.

Sec 6's own rule: "the existing per-step interactive gate is retained but
demoted -- off inside the task server, whose gates are the action
boundaries." So `motion.MotionController` is built with `interactive=False`
here, always -- the *actions themselves* are the human gate: a client calls
PickStick, waits for the result, decides whether to call PlaceStick next.

State machine (Sec 6):
    IDLE --PickStick--> PICKING --> HOLDING --PlaceStick--> PLACING --> AT_PLACE --ReleaseStick--> IDLE
A goal is rejected (result.success=False, action aborted) if the state
doesn't accept it -- see _ACCEPTS below. On a failed sequence, the state
becomes ERROR and stays there until a `Reset` goal (Sec 11 Phase 5) is
called -- the only way back to IDLE. Reset's `assume_gripper_empty` flag
(mirrors BRIDGE_PROTOCOL.md Sec 5.11's socket `reset` command) decides
whether the "stick" collision object is cleaned up on the way back to IDLE
or left alone for planning to stay collision-aware around it. `Abort`
(cancelling an *in-flight* goal, real ROS2 action-cancellation semantics)
is a different, still out-of-scope problem -- Reset only recovers from a
goal that has already terminated into ERROR.

`TaskState` is published on a latched topic (`/stick_task/state`,
durability TRANSIENT_LOCAL) after every transition, so a late-joining
client sees the current state without polling.

⚠ **Two separate `Node`s, on purpose** (`_motion_node` / `_action_node`),
each with its own executor. Found on real hardware: an `ActionServer`
whose `execute_callback` runs `pymoveit2` calls (which internally call the
free function `rclpy.spin_once(node, ...)` in a loop -- see
`motion.MotionController`/`pymoveit2.moveit2.MoveIt2.wait_until_executed`)
wedges permanently after the FIRST goal terminates: every later goal on
ANY action on that node then hangs forever, no exception logged. This is a
known upstream rclpy issue (ros2/ros2#1609) -- calling
`spin_once`/`spin_until_future_complete` from *within* a callback already
being dispatched by the executor that owns the node detaches the node from
that executor's wait-set. The fix confirmed working here: give the
ActionServers their own `Node` (`_action_node`, spun by its own
`_action_executor`), completely separate from the `Node` pymoveit2 drives
(`_motion_node`, spun by `MotionController`'s own executor) -- so whatever
happens to `_motion_node`'s executor association, `_action_executor` never
shares it and keeps receiving new goals fine. Do not collapse these back
into one `Node`.

Run alongside the same hardware/MoveIt bringup as pick_and_place_node.py:
    ros2 launch so_arm_100_moveit_config pickandplace_demo.launch.py
    ros2 launch so_arm_100_pick_and_place stick_task_server.launch.py
Then drive it with `ros2 action send_goal`, e.g.:
    ros2 action send_goal /pick_stick so_arm_100_stick_msgs/action/PickStick \\
        "{stick_id: s_004, length: 0.11, source: feeder_0}"

⚠ **Must be launched with `config/pick_and_place.yaml`** (the launch file
above does this) -- `ros2 run so_arm_100_pick_and_place stick_task_server`
directly, with no `--params-file`, silently runs on this file's own
code-fallback defaults instead (found 2026-08-02: `stick.pose` was off by
13cm this way -- Sec 4 D9). The yaml is scoped `/**:` (wildcard node name)
specifically so it applies here even though this node isn't named
`pick_and_place_node`.
"""
import math
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from so_arm_100_stick_msgs.action import PickStick, PlaceStick, ReleaseStick, Reset
from so_arm_100_stick_msgs.msg import TaskState

from so_arm_100_pick_and_place import scene, sequences
from so_arm_100_pick_and_place.motion import MotionController

# Which states each action accepts (Sec 6's table). Anything else is a
# rejection, not a silent no-op or a queued retry.
_ACCEPTS = {
    "PickStick": "IDLE", "PlaceStick": "HOLDING", "ReleaseStick": "AT_PLACE", "Reset": "ERROR",
}

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def main():
    rclpy.init()
    # Two Nodes on purpose -- see the module docstring's "Two separate
    # Node's" note. action_node declares all the params (it's the "public"
    # node clients/launch files interact with); motion_node exists only to
    # be handed to MotionController.
    motion_node = Node("stick_task_server_motion")
    action_node = Node("stick_task_server")

    base_frame = action_node.declare_parameter("base_frame", "base_link").value
    # Matches config/pick_and_place.yaml's real feeder-hole measurement --
    # kept in sync 2026-08-02 after finding this node never actually
    # received the yaml (Sec 4 D9); these are now only a fallback safety
    # net, not the intended source of truth.
    # base_xyz_m is the stick's PHYSICAL BASE (bottom, at the feeder hole),
    # not a box center -- see pose_utils.feeder_stick_pose. A real
    # PickStick goal's `length` overrides default_length_m per-goal (see
    # _execute_pick), so different-length sticks still land base-down at
    # this same fixed spot instead of needing this constant retuned per size.
    stick_base_xyz_m = tuple(action_node.declare_parameter("stick.base_xyz_m", [0.379, -0.026, 0.0103]).value)
    stick_section_m = list(action_node.declare_parameter("stick.section_m", [0.0060, 0.0060]).value)
    stick_default_length_m = action_node.declare_parameter("stick.default_length_m", 0.11).value

    steps_cfg = sequences.declare_all_steps(action_node)

    gripper_open_position = math.radians(
        action_node.declare_parameter("grasp.gripper_open_position_deg", 30.0).value)
    # -9.0 deg / 0.0157 rad re-tuned 2026-08-02 on the real gripper/stock
    # via tune_grasp_node.py -- see config/pick_and_place.yaml's own comment.
    gripper_grasp_position = math.radians(
        action_node.declare_parameter("grasp.gripper_grasp_position_deg", -9.0).value)
    grasp_verification_enabled = action_node.declare_parameter("grasp_verification.enabled", True).value
    grasp_gap_threshold = action_node.declare_parameter("grasp_verification.gap_threshold", 0.0157).value
    velocity_scaling = action_node.declare_parameter("velocity_scaling", 0.4).value
    acceleration_scaling = action_node.declare_parameter("acceleration_scaling", 0.4).value
    planning_time = action_node.declare_parameter("planning_time", 5.0).value

    motion = MotionController(
        motion_node,
        base_frame=base_frame,
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        planning_time=planning_time,
        interactive=False,  # Sec 6: action boundaries are the gate now, not a terminal prompt.
        gripper_open_position=gripper_open_position,
        gripper_closed_position=gripper_grasp_position,
    )

    stick_cfg = {
        "base_xyz_m": stick_base_xyz_m,
        "section_m": stick_section_m,
        "default_length_m": stick_default_length_m,
    }
    grasp_cfg = {
        "open_position": gripper_open_position,
        "grasp_position": gripper_grasp_position,
        "verification_enabled": grasp_verification_enabled,
        "gap_threshold": grasp_gap_threshold,
    }

    server = StickTaskServer(action_node, motion, steps_cfg, stick_cfg, grasp_cfg)  # noqa: F841
    action_node.get_logger().info("stick_task_server ready. State: IDLE.")

    # action_node's own executor -- deliberately NOT motion._executor (see
    # the module docstring). Spun on this thread; motion_node keeps
    # spinning independently on MotionController's own background thread.
    action_executor = MultiThreadedExecutor(4)
    action_executor.add_node(action_node)
    try:
        action_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motion.shutdown(0)


class StickTaskServer:
    def __init__(self, node, motion, steps_cfg, stick_cfg, grasp_cfg):
        """`node` is the ACTION node (`main()`'s `action_node`) -- a
        different `Node` than the one `motion` (a MotionController) drives.
        See the module docstring for why that separation matters."""
        self._node = node
        self._logger = node.get_logger()
        self._motion = motion
        self._steps_cfg = steps_cfg
        self._stick_cfg = stick_cfg
        self._grasp_cfg = grasp_cfg

        self._lock = threading.Lock()
        self._state = "IDLE"
        self._current_stick_id = ""
        self._current_stick_length_m = 0.0

        self._state_pub = node.create_publisher(TaskState, "/stick_task/state", _LATCHED_QOS)
        self._publish_state(message="ready")

        # This node's OWN callback group, deliberately not motion's (they
        # must not be shared -- see the module docstring). Reentrant so a
        # quick rejection isn't blocked behind a long-running execution.
        cbg = ReentrantCallbackGroup()
        self._pick_server = ActionServer(
            node, PickStick, "pick_stick", self._execute_pick, callback_group=cbg)
        self._place_server = ActionServer(
            node, PlaceStick, "place_stick", self._execute_place, callback_group=cbg)
        self._release_server = ActionServer(
            node, ReleaseStick, "release_stick", self._execute_release, callback_group=cbg)
        self._reset_server = ActionServer(
            node, Reset, "reset", self._execute_reset, callback_group=cbg)

    def _publish_state(self, message="", current_stick_id=None, current_stick_length_m=None):
        if current_stick_id is not None:
            self._current_stick_id = current_stick_id
        if current_stick_length_m is not None:
            self._current_stick_length_m = current_stick_length_m
        msg = TaskState(
            state=self._state, current_stick_id=self._current_stick_id,
            current_stick_length_m=self._current_stick_length_m, message=message)
        self._state_pub.publish(msg)
        self._logger.info(f"[{self._state}] {message}")

    def _reject(self, action_name, goal_handle, result_type):
        state = self._state
        message = f"rejected: state is {state}, {action_name} needs {_ACCEPTS[action_name]}"
        self._logger.warn(message)
        goal_handle.abort()
        return result_type(success=False, message=message)

    def _measure_gripper_gap(self):
        # Mirrors sequences.check_grasp_success's own computation.
        state = self._motion.gripper.joint_state
        if state is None or "Gripper" not in state.name:
            return 0.0
        actual = state.position[state.name.index("Gripper")]
        return abs(actual - self._grasp_cfg["grasp_position"])

    def _execute_pick(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            if self._state != "IDLE":
                return self._reject("PickStick", goal_handle, PickStick.Result)
            self._state = "PICKING"
        self._publish_state(
            message="picking", current_stick_id=goal.stick_id, current_stick_length_m=goal.length)

        def feedback_cb(step_name, index, count):
            goal_handle.publish_feedback(PickStick.Feedback(step=step_name, step_index=index, step_count=count))

        ok, message = sequences.run_pick_sequence(
            self._motion, self._steps_cfg, self._stick_cfg, self._grasp_cfg,
            length_m=goal.length, feedback_cb=feedback_cb)

        with self._lock:
            self._state = "HOLDING" if ok else "ERROR"
        self._publish_state(message=message)
        # Always the real measured gap, even on failure -- a hardcoded 0.0
        # here (the previous behaviour) threw away exactly the number
        # needed to tell "genuinely closed on nothing" apart from "a real
        # grasp that missed the threshold by a hair".
        gripper_gap = self._measure_gripper_gap()
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return PickStick.Result(success=ok, message=message, gripper_gap=gripper_gap)

    def _execute_place(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            if self._state != "HOLDING":
                return self._reject("PlaceStick", goal_handle, PlaceStick.Result)
            self._state = "PLACING"
        self._publish_state(message="placing")

        if goal.approach_from_above:
            self._logger.warn(
                "approach_from_above requested but not implemented in this phase -- "
                "using the single joint-move place (Phase 3), same as the standalone demo.")

        target = goal.target
        stick_spec = {
            "base_xyz_m": tuple(target.base), "tip_xyz_m": tuple(target.tip), "roll_deg": target.roll_deg,
        }

        def feedback_cb(step_name, index, count):
            goal_handle.publish_feedback(PlaceStick.Feedback(step=step_name, step_index=index, step_count=count))

        ok, message = sequences.run_place_sequence(self._motion, stick_spec, feedback_cb=feedback_cb)

        with self._lock:
            self._state = "AT_PLACE" if ok else "ERROR"
        self._publish_state(message=message)
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return PlaceStick.Result(success=ok, message=message)

    def _execute_release(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            if self._state != "AT_PLACE":
                return self._reject("ReleaseStick", goal_handle, ReleaseStick.Result)
        self._publish_state(message="releasing")  # Sec 6 has no distinct RELEASING state

        def feedback_cb(step_name, index, count):
            goal_handle.publish_feedback(ReleaseStick.Feedback(step=step_name, step_index=index, step_count=count))

        # The placed stick's actual length, remembered from PickStick's goal
        # (ReleaseStick's own goal only carries stick_id) -- so a real,
        # possibly non-default-length stick gets registered at its true
        # size, not a static config guess.
        section_m = self._stick_cfg["section_m"]
        placed_size = (section_m[0], section_m[1], self._current_stick_length_m)
        ok, message = sequences.run_release_sequence(
            self._motion, stick_id=goal.stick_id, size=placed_size,
            frame_id=self._motion.base_frame, grasp_cfg=self._grasp_cfg,
            retreat_step_cfg=self._steps_cfg["retreat"], feedback_cb=feedback_cb)

        with self._lock:
            self._state = "IDLE" if ok else "ERROR"
        self._publish_state(
            message=message,
            current_stick_id="" if ok else None,
            current_stick_length_m=0.0 if ok else None,
        )
        if ok:
            goal_handle.succeed()
            self._logger.info(f"Released '{goal.stick_id}'. Ready for the next PickStick.")
        else:
            goal_handle.abort()
        return ReleaseStick.Result(success=ok, message=message)

    def _execute_reset(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            if self._state != "ERROR":
                return self._reject("Reset", goal_handle, Reset.Result)
        if goal.assume_gripper_empty:
            # Confirmed empty by the human -- safe to clean up any lingering
            # "stick" collision object (attached or world) left behind by
            # whatever failed. Same D8/D12 cleanup pattern, now reachable
            # from a manual recovery too, not just the normal pick/release
            # success paths.
            scene.detach_stick(self._motion.arm, self._logger)
            scene.remove_stick(self._motion.arm)
        # else: leave the collision scene untouched -- if "stick" is still
        # attached, subsequent planning stays collision-aware around it.
        with self._lock:
            self._state = "IDLE"
        self._publish_state(message="reset", current_stick_id="", current_stick_length_m=0.0)
        goal_handle.succeed()
        return Reset.Result(success=True, message="reset to IDLE")


if __name__ == "__main__":
    main()
