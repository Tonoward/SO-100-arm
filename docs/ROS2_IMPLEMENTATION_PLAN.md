# ROS2 side — status & implementation plan (stick-assembly project)

**Last updated:** 2026-07-28
**Audience:** a developer or AI agent picking this project up cold.
**Companion documents:**
- [`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) — the Blender client.
- [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) — **the data contracts between the two**: the build-file format and the optional live protocol. Both sides must agree with that file; it is the single source of truth. Do not restate the formats here or in the Blender doc — link to it.

---

## 0. How to use this document

Read §1–§4 to understand where the project is *today*, §5–§10 for the target
design, §11 for the ordered work plan, and §13 for questions that are still
open with the user (do not invent answers to those — ask).

**Before changing anything, read §3 ("Do not regress").** Several of the
settings in this repo look arbitrary and are not: they are the outcome of
long hardware debugging sessions. Each one has a comment in-file explaining
why. Preserve those comments.

### ⚠ Terminology: "TCP" means two different things

An unfortunate acronym collision, so be careful when reading or searching:

| Term | Meaning | Status |
|---|---|---|
| **TCP** (default, and the only one used in §7–§9) | **Tool Center Point** — the `End_Effector` link, the point on the gripper whose position the arm actually controls. What `fk()` returns and `ik()` takes. | ✅ Core concept |
| **TCP** (networking) | Transmission Control Protocol — a network socket | ❌ **Not used.** Appears only in `BRIDGE_PROTOCOL.md` Part B and the Option A comparison, both kept as a record of a rejected design. |

Unless the sentence is explicitly about sockets, ports, or `BRIDGE_PROTOCOL.md`
Part B, **TCP means Tool Center Point.**

---

## 1. Project goal

The robot is a build tool for a physical sculpture made of small wooden
sticks.

1. A mesh is designed in Blender. It decomposes into **sticks** — straight
   segments of 6.45 mm square section, of *varying lengths* (80–150 mm).
2. Sticks are always picked from **one fixed feeder location** (a hole in the
   mount platform). The robot therefore never needs to *find* a stick — the
   pick pose is constant and already tuned. The human reloads the feeder.
3. For each stick, in a repeating cycle:
   - **PICK** — the robot runs the whole pregrasp → lower → close → verify →
     lift sequence *without stopping*, and ends holding the stick at a safe
     standby pose. It stops there.
   - *(human presses a button / sends a command)*
   - **PLACE** — the robot carries the stick to its designed pose in the
     sculpture, **still gripping it**, and holds still. The human applies
     glue by hand.
   - *(human presses a button / sends a command)*
   - **RELEASE** — the robot opens the gripper, retreats to standby, and
     stops until commanded for the next stick.
4. Future upgrade: a **rotating table** in front of the robot so the workpiece
   can be re-indexed instead of demanding a bigger arm workspace. Out of
   scope for now, but the frame layout in §8 is designed so it can be added
   without a redesign.

The critical change from today's demo: **the place pose is different for every
stick and comes from outside the ROS2 process**, so it can no longer be a
hardcoded joint-angle list.

### 1.1 Confirmed parameters (answered by the user, 2026-07-27)

| # | Decision |
|---|---|
| Q1 | Sticks are **all different lengths**, cut in advance, in the range **80–150 mm** (raised from 50 mm by the N3 decision below). Lengths are decided by the Blender mesh. **The system must tell the human which stick to load next.** |
| Q2 | **One** feeder hole, reloaded by the human every cycle. Today's tuned `pregrasp`/`lower` constants therefore remain valid forever. |
| Q3 | The grip is at a **fixed distance from the stick's bottom** (it sits in a hole, so the bottom is always datum). ⇒ `grasp_offset_from_base` is a single constant; variable lengths cost nothing. Measured value ≈ **51 mm** (§8.2). |
| Q4 | Placed sticks **may be tilted**. Before a build starts, ROS2 must check **every** placement and report which ones are impossible. The rotating table will come later; the system must work without it first. |
| Q5 | Sticks glue to a **base plate** *and* to **each other, vertex to vertex** (Blender-mesh topology). |
| Q6 | Requested build volume: **300 (W) × 200 (D) × 300 (H) mm**, centred at **X = 0, Y = −350 mm** in `base_link`. ⚠ **See §9.5 — this box is only ~85 % reachable.** |
| Q7 | Human loading of the feeder is mandatory for now; full automation is possible later. |
| Q8 | **Blender 5.2.0 LTS**, on this Linux machine *and* on a separate Windows machine. |
| QB1 | Authoring is a **wireframe mesh** (each edge = one stick). Stock is **6.45 mm** square. Cutting to arbitrary lengths is fine, but a **fixed-stock-length mode** is also wanted for easier testing. |
| QB2 | Both: arbitrary cut lengths *and* a snap-to-stock-length mode. |
| QB3 | Yes — mirror the robot in the Blender viewport. |
| QB4 | Build state must live in **both** the `.blend` and a compact **JSON** file. |
| QB5 | ✅ **Decided: Option C — "slicer + shared kinematics"** (§5.2). Blender exports a build file; ROS2 executes it. No live connection. The closed-form kinematics module is pure Python with zero ROS imports and is **vendored verbatim into the addon**, so Blender validates and previews offline, including on Windows. |

### 1.2 Decisions taken 2026-07-27 / 07-28

| # | Decision |
|---|---|
| **N1** | ✅ **Option C** (see QB5 above). Part A of [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) — the build file — is the primary integration path. Part B (live socket) is **not** being implemented now. |
| **N2** | ✅ **Build volume = 240 (W) × 160 (D) × 200 (H) mm, centred at Y = −370 mm.** i.e. X ∈ [−120, +120], Y ∈ [−290, −450], Z ∈ [0, 200] mm in `base_link`. **98.0 %** reachable for vertical sticks (§9.5). |
| **N3** | ✅ **Superseded 2026-07-28 — do NOT lower the grip height.** The `lower` step keeps its tuned value and the grip stays ~51 mm above the stick's base. Instead the **stick length range is raised to 80–150 mm**, so even the shortest stick extends ~29 mm above the grip point. Simpler, no retuning, no risk to the feeder-hole clearance — and it leaves the §9.4/§9.5 envelope numbers valid as computed. |
| **N5** | ✅ **Blank surface — no sockets, no markings.** Sticks glue directly onto a plain base plate. A fiducial-marked (QR or similar) base shape that the robot can locate is a **future** feature, not in scope. ⇒ the first layer is glued freehand onto flat stock, so first-layer accuracy rests entirely on the arm's own positioning. |
| **N6** | ✅ **Target < 60 s of hold per glue joint**, with **servo temperature monitoring** added. See §12.1 — the temperature is already being read every control cycle and discarded, so exposing it is nearly free. |
| **N4** | ✅ **Joint allowance = 3.25 mm per shared end**, and — importantly — **the design mesh is expanded rather than the sticks shortened** (§8.4). Note this is essentially half the 6.45 mm section (6.45/2 = 3.225), i.e. each stick stops at the *face* of the joint rather than at its centreline — the geometrically correct value for a 90° joint. Glue is applied with a glue gun, so gaps and blobs are acceptable. See §8.4 for the shallow-angle caveat. |

---

## 2. System snapshot (what exists today)

### 2.1 Environment

| Item | Value |
|---|---|
| ROS distro | Humble (Ubuntu 22.04) |
| ROS Python | 3.10 (`/opt/ros/humble/lib/python3.10`) |
| Workspace | `~/ros2_ws`, sources under `~/ros2_ws/src/SO-100-arm` |
| Motion planning | MoveIt2 2.5.9, OMPL (RRTConnect) |
| Python MoveIt binding | `pymoveit2` 4.2.0 (`ros-humble-pymoveit2`). `moveit_commander` / `moveit_py` are **not available** on this install — do not plan around them. |
| IK plugin | `kdl_kinematics_plugin` (`so_arm_100_moveit_config/config/kinematics.yaml`). TRAC-IK, pick_ik, bio_ik and IKFast are **not installed**. See §9 — this matters. |
| Servos | Feetech STS3215 over a single serial bus, 1 Mbaud, custom `so_arm_100_hardware` ros2_control system |
| Blender | **not installed on this machine yet** |

### 2.2 Packages

| Package | Role |
|---|---|
| `so_arm_100_description` | URDF/xacro, meshes, `Mount_Platform` |
| `so_arm_100_hardware` | ros2_control `SystemInterface` for the STS3215 bus; calibration, gravity compensation, integral trim |
| `so_arm_100_moveit_config` | SRDF, kinematics, controller configs, RViz, launch files |
| `so_arm_100_bringup` | hardware / gz / rviz launch |
| `so_arm_100_pick_and_place` | **the current demo** — a single linear script |
| `so_arm_100` | metapackage |

### 2.3 Kinematic chain

5 arm joints + 1 gripper joint:

```
base_link → Shoulder_Rotation (yaw, about base Z)
          → Shoulder_Pitch    (pitch)
          → Elbow             (pitch)
          → Wrist_Pitch       (pitch)
          → Wrist_Roll        (roll, about the tool axis)
          → End_Effector      (TCP link used by MoveIt)
            Gripper joint drives Moving_Jaw against Fixed_Gripper
```

MoveIt groups (SRDF): `arm` (the 5), `gripper` (1), `arm_gripper` (both).
Named states include `home`, `init`, `open`, `close`.

### 2.4 What works today

`so_arm_100_pick_and_place/so_arm_100_pick_and_place/pick_and_place_node.py`
is a **linear, single-shot, interactive script**. It:

- builds `MoveIt2` + `MoveIt2Gripper` on a `MultiThreadedExecutor(4)` with a
  `ReentrantCallbackGroup` (required — pymoveit2's blocking calls spin
  internally and deadlock on a single-threaded executor);
- adds a `stick` collision box to the planning scene;
- runs: `home → open gripper → pregrasp → lower → grasp → (verify) →
  attach → lift → place → open gripper → detach/re-add → retreat`;
- for each step: **plans first**, publishes the plan to
  `display_planned_path` so RViz animates a ghost preview, then waits at a
  terminal prompt (`Enter` = execute, `s` = skip, `q` = abort);
- reads every waypoint and mode from
  `so_arm_100_pick_and_place/config/pick_and_place.yaml`, with **joint angles
  in degrees**, converted to radians in the node;
- supports three per-step modes: `joint`, `cartesian_relative`,
  `cartesian_absolute`;
- has an optional grasp-verification heuristic (commanded vs. settled gripper
  position) that can be toggled off entirely.

**This is confirmed working end-to-end on real hardware**, including the
Cartesian straight-up `lift` out of the feeder hole.

---

## 3. Do not regress — hard-won settings and their reasons

Every item below cost real debugging time. If a change appears to require
undoing one of these, re-read the in-file comment first.

| # | Finding | Where it lives |
|---|---|---|
| 1 | **Feetech servo EEPROM angle limits.** 5 of 6 servos shipped with non-zero `MinAngleLimit`/`MaxAngleLimit` (EEPROM addr 9/11), silently clamping motion far inside the URDF limits. Fixed by writing `0`/`4095` via `unLockEprom → writeWord → LockEprom`. This is **independent of** `calibration.yaml`, URDF limits and MoveIt. | Documented in `README.md` §"Servo-level EEPROM angle limits" |
| 2 | **`computeCartesianPath` ignores velocity/acceleration scaling on Humble** (scaling was only added in Iron). Cartesian plans are always timestamped at *full* speed while the hardware moves at its own slower pace → two separate watchdogs fire spuriously. | `moveit_controllers.yaml` header comment |
| 3 | Watchdog A: move_group's execution monitor. Set `allowed_execution_duration_scaling: 3.0`, `allowed_goal_duration_margin: 15.0`. Symptom when too tight: `waitForExecution timed out` / `TIMED_OUT`. | `so_arm_100_moveit_config/config/moveit_controllers.yaml` |
| 4 | Watchdog B: the **controller's own** `constraints.goal_time`, a *separate* lower-level check. Set to `15.0` for `arm_controller` **and** `gripper_controller`, in **both** `ros2_controllers.yaml` and `hardware_controllers.yaml`. Symptom when too tight: `GOAL_TOLERANCE_VIOLATED: Aborted due to goal_time_tolerance exceeding by N seconds` *even though the arm physically arrived*. | both controller yamls |
| 5 | Per-joint `trajectory:` (path) tolerance is deliberately `0.0` (= unconstrained). Gravity compensation intentionally offsets the commanded position from the interpolated path; only the end-of-motion `goal:` tolerance is meaningful. | both controller yamls |
| 6 | `allowed_start_tolerance: 0.15` (not MoveIt's 0.01) — gravity compensation means the settled position is always slightly offset from the commanded one. | `moveit_controllers.yaml` |
| 7 | **`MoveIt2.allow_collisions()` hangs** in this long-lived process. It is the only pymoveit2 path that uses a synchronous `Client.call()`; move_group never even receives the request, and the identical call succeeds instantly in an isolated script. **Do not reintroduce it.** The working pattern for "let the jaws collide with the stick" is `remove_collision_object()` → plan → `add_collision_box()` (plain topic publishes). | `pick_and_place_node.plan_grasp_gripper()` |
| 8 | The gripper is a **`JointTrajectoryController`**, not a gripper action controller — `parallel_gripper_action_controller` is not installed in this image. Do not switch it back. | `ros2_controllers.yaml`, `moveit_controllers.yaml` |
| 9 | `arm_effort_controller` / `gripper_effort_controller` are deliberately **not** listed in `moveit_simple_controller_manager.controller_names` — they claim the same joints' effort interfaces and win the race, leaving the position controllers inactive. | `moveit_controllers.yaml` comment |
| 10 | Interactive prompts must be printed via `logger.info()` (a complete, flushed line), not `input()`'s prompt argument, and the node should be started with `ros2 run` — `ros2 launch` does not reliably forward stdin. | `pick_and_place_node.prompt()` |
| 11 | Joint-space (`joint` mode) planning is by far the most reliable path on this arm. Cartesian modes are used only where the *path shape* matters (pulling straight out of the feeder hole). | `pick_and_place.yaml` header |

**Debugging technique that worked repeatedly:** to read a live node's ROS log
without asking the user to copy-paste, find it via
`ls -l /proc/<pid>/fd | grep log`. `py-spy` needs `ptrace_scope=0` or sudo on
this machine, so it is usually unavailable.

---

## 4. Known open issues / technical debt

| # | Issue | Impact on this project |
|---|---|---|
| D1 | **`mount_platform_joint` origin is still `xyz="0 0 0" rpy="0 0 0"`** — a placeholder. The visual platform therefore does not correspond to reality. | **Blocking for Phase 0.** Everything Blender sends is in `base_link`; if the platform/table frame is wrong, the whole sculpture is offset. |
| D2 | `README.md` §"Pick-and-place demo" is **stale** — it documents `grasp.offset`, `grasp.pregrasp_lift`, `grasp.lift_height`, `place.pose`, none of which exist any more. The current config is the `steps.<name>.mode` scheme. | Misleads any new reader. Fix during Phase 1. |
| D3 | `README.md` does not yet document findings #2/#3/#4 in §3 (the Cartesian-scaling / `goal_time` timing chain). | Same. |
| D4 | `grasp_verification.gap_threshold: 0.1` rad is untuned against a real 6 mm stick. | Verification may false-positive or false-negative; low priority while a human watches every cycle. |
| D5 | KDL is the only IK plugin available and is a poor fit for a 5-DOF chain. | See §9 — drives a real architectural decision. |
| D6 | The demo assumes exactly one stick and one place pose; there is no notion of a build plan, of placed sticks as obstacles, or of resuming an interrupted build. | The entire Phase 3–5 work. |
| D7 | No e-stop / "hold still, human's hands are in the workspace" state. | Safety — see §12. |

---

## 5. Target architecture

**There is no live connection between Blender and ROS2** (Option C, §5.2).
The two sides are coupled only by a file, and by a kinematics module that is
byte-identical on both sides.

```
  ┌─────────────────────────────────────┐
  │  Blender addon (so100_builder)      │      Blender's own Python.
  │    design mesh → expand → order     │      Runs offline, Linux or
  │    → validate → export              │      Windows, no ROS present.
  │    ├── kinematics/  ◄──────────────────┐   VENDORED VERBATIM
  └──────────────┬──────────────────────┘  │   (pure Python, no ROS,
                 │ writes                  │    no numpy)
                 ▼                         │
      ┌──────────────────────────┐         │
      │  <name>.build.json       │         │   the ONLY interface:
      │  <name>.status.json      │         │   BRIDGE_PROTOCOL.md Part A
      └──────────────┬───────────┘         │
                 │ reads / writes status   │
                 ▼                         │
  ┌──────────────────────────────────┐     │
  │ so_arm_100_pick_and_place        │     │
  │   stick_task_server_node         │     │
  │     ├── motion.py  (MoveIt2)     │     │
  │     ├── sequences.py             │     │
  │     └── scene.py   (collision)   │     │
  └──────────────┬───────────────────┘     │
                 │ uses                    │
  ┌──────────────▼───────────────────┐     │
  │ so_arm_100_kinematics  ──────────────────┘  ✅ BUILT & TESTED
  │   fk / ik / envelope  (no ROS)   │
  └──────────────┬───────────────────┘
                 │ MoveIt
                 ▼
   move_group ─► arm_controller / gripper_controller
                 ▼
        so_arm_100_hardware (STS3215)
```

The build file moves by whatever means is convenient — shared folder, USB
stick, git, scp. Nothing in the design cares.

### 5.1 New / changed packages

| Package | Type | Contents |
|---|---|---|
| `so_arm_100_stick_msgs` | **new**, `ament_cmake` | `action/PickStick.action`, `action/PlaceStick.action`, `action/ReleaseStick.action`, `srv/ValidatePlacements.srv`, `srv/SetBuildPlan.srv`, `srv/GoToNamed.srv`, `msg/StickSpec.msg`, `msg/TaskState.msg` |
| `so_arm_100_kinematics` | ✅ **BUILT & TESTED**, `ament_python` | Closed-form 5-DOF FK/IK + reachability (§9). `chain.py`, `constants.py`, `envelope.py`. **Pure Python, `math` is its only import** — verified to run under `env -i` with no ROS sourced, which is the condition it faces inside Blender. 11 unit tests pass, including recovering all five hand-tuned hardware poses from their own FK output. |
| `so_arm_100_pick_and_place` | **refactor** | split the monolith into `motion.py`, `sequences.py`, `scene.py`; keep `pick_and_place_node.py` working as-is; add `stick_task_server_node.py` |
| `so_arm_100_blender_bridge` | ❌ **not being built** | Made unnecessary by the Option C decision (§5.2). Blender exports a build file instead of holding a socket. |

### 5.2 ✅ Architecture: "slicer + shared kinematics" (Option C)

**Decided 2026-07-27.** Blender is a slicer: it exports a build file, ROS2
loads and executes it. **There is no live connection**, so
`so_arm_100_blender_bridge` is *not* being built and Part B of the protocol
doc is dormant.

The one design rule that makes this work: **`so_arm_100_kinematics` is pure
Python with zero ROS imports**, so the exact same module is vendored into the
Blender addon and runs inside Blender's own interpreter. Blender therefore
validates reachability, previews the robot, and draws the reachable envelope
**offline, instantly, on Windows, with no ROS installed** — while ROS2
re-validates on load using the identical code, so the two can never drift.

Consequences for this document: §6/§7 (the task server's state machine and
sequences) still apply, but they are driven by the build-file execution loop
in [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) §A.4 rather than by ROS actions
from a socket. Exposing them as ROS actions as well remains worthwhile for
`ros2 action send_goal` testing.

Full reasoning for the three options considered:
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) §2.

### 5.3 Why a live link, if used at all, cannot be rclpy inside Blender

Blender ships its **own** Python interpreter (3.11 for Blender 4.x). `rclpy`
and every ROS2 C extension on this machine are built for **CPython 3.10**.
They cannot be imported into Blender's interpreter — this is an ABI
incompatibility, not a `PYTHONPATH` problem, and attempts to force it will
either fail to import or crash Blender.

Therefore the addon must speak a transport, not a Python API. See
[`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) for the chosen protocol and the
rejected alternatives (rosbridge_suite, ROS 1-style, shared files).

---

## 6. Interaction model — the task server's state machine

The three human-gated stops in §1 become three ROS actions. The task server
holds exactly one state and rejects commands that don't fit it, so a
mis-sequenced Blender click can never produce a surprise motion.

```
                 ┌──────────────────────────────────────────┐
                 ▼                                          │
            [ IDLE ] ──PickStick──► [ PICKING ] ──► [ HOLDING ]
             ▲   ▲                      │                   │
             │   └──────(failed)◄───────┘             PlaceStick
             │                                              ▼
             │                                        [ PLACING ]
             │                                              │
             │                                              ▼
             └──ReleaseStick◄── [ AT_PLACE / GLUING ] ◄─────┘
```

| State | Meaning | Accepts |
|---|---|---|
| `IDLE` | At standby, gripper empty | `PickStick`, `GoToNamed`, `ValidatePlacements` |
| `PICKING` | Running the pick sequence, no stops | `Abort` |
| `HOLDING` | At standby, holding a stick, motionless | `PlaceStick`, `Abort`, `ReleaseStick` (drop/recover) |
| `PLACING` | Carrying the stick to its target | `Abort` |
| `AT_PLACE` | **Motionless at the target, still gripping.** Human is gluing. | `ReleaseStick`, `Abort` |
| `ERROR` | A step failed; requires explicit operator recovery | `GoToNamed`, `Abort`, `Reset` |

Rules:
- **`AT_PLACE` never times out.** Glue can take a minute; the arm must simply
  hold. See §12 for the servo-load caveat.
- Every action publishes `TaskState` on a latched topic (`/stick_task/state`)
  so the bridge can push events to Blender without polling.
- The existing per-step interactive gate is **retained but demoted**: it stays
  available as `interactive:=true` on the standalone demo node for tuning, and
  is *off* inside the task server (whose gates are the action boundaries).

---

## 7. Sequence definitions

Each of the three actions is a fixed list of steps, reusing today's proven
step machinery.

### 7.1 `PickStick`
Goal: `string stick_id`, `float64 length`, `string source` (default
`"feeder_0"`).

1. `open_gripper` (joint, `grasp.gripper_open_position_deg`)
2. `pregrasp` (joint — the tuned constant, from config)
3. `lower` (joint — the tuned constant)
4. `close_gripper` = `remove_collision_object("stick") → plan → execute →
   add_collision_box(...)` (finding #7)
5. `verify_grasp` (heuristic; on failure → `ERROR`, do **not** silently
   continue; result carries the measured gap so Blender can show it)
6. `attach_collision_object` to `Fixed_Gripper`
7. `lift` (**cartesian_relative**, straight +Z — finding #11 / the hole)
8. `standby_holding` (joint — a new tuned constant, high and clear of the
   build area)

Result: `bool success`, `string message`, `float64 gripper_gap`.
Feedback: `string step`, `uint8 step_index`, `uint8 step_count`.

### 7.2 `PlaceStick`
Goal: `StickSpec target` (see §8.3), `bool approach_from_above`.

1. Compute the TCP goal pose from the stick spec + grasp offset (§8.2).
2. Solve IK (§9) → joint target. **If no solution, abort before moving** with
   a machine-readable reason so Blender can mark the stick red.
3. `approach` — joint move to a pose `place.approach_clearance` (default
   0.05 m) above the final pose.
4. `insert` — **cartesian_relative** straight down. Mirror image of the pick
   `lift`; same reasoning about path shape.
5. Stop. Enter `AT_PLACE`. Do **not** open the gripper.

### 7.3 `ReleaseStick`
1. `open_gripper`
2. `detach_collision_object`
3. **`add_collision_box` for the placed stick at its final pose, with a
   permanent id `placed_<stick_id>`** — the sculpture grows, and every placed
   stick is an obstacle for subsequent motions (§10).
4. `withdraw` — **cartesian_relative** straight up, out from between the
   already-placed sticks.
5. `standby` (joint) → `IDLE`.

---

## 8. Geometry & frames contract

### 8.1 Frames

| Frame | Definition | Status |
|---|---|---|
| `base_link` | Robot base. **The one and only frame used on the wire** — everything Blender sends is already expressed here (see the protocol doc). | exists |
| `Mount_Platform` | The static plate the arm and the feeder hole are mounted on. | **origin is a placeholder — D1** |
| `build_table` | **New.** The origin of the sculpture: where Blender's world origin maps to. A fixed TF child of `base_link` today; becomes a *revolute* child when the rotating table arrives, with zero further changes required upstream. | to create |
| `feeder_0` | The stick source hole. Today implicit in the tuned `pregrasp`/`lower` joint constants. | to formalise (optional) |

Introducing `build_table` now, even as a fixed transform, is the single
cheapest piece of future-proofing in this plan.

### 8.2 Grasp offset — the piece that makes variable-length sticks work

Define, in the gripper frame:

- `grasp_axis` — the direction of the stick's long axis expressed in the
  `End_Effector` frame, fixed by how the jaws close on it.
- `grasp_offset_from_base` — the distance from the stick's **base end** (the
  end that touches the table or another stick) to the point where the jaws
  grip.

If the feeder hole has a fixed depth and the `lower` pose is a fixed joint
constant, then **the jaws always grip a fixed distance above the stick's base
end, regardless of the stick's total length** — the extra length simply
protrudes further above the jaws. In that case `grasp_offset_from_base` is a
single constant and variable-length sticks cost nothing.

✅ **Confirmed by the user (Q3)** — the hole makes the bottom the datum, so
this is the good case. Variable stick lengths cost nothing in the place maths.

**Measured value.** At the tuned `lower` pose the TCP sits at z = 64.8 mm; the
100 mm stick's bottom end is at z ≈ 14 mm. So
**`grasp_offset_from_base` ≈ 51 mm**, to be confirmed with a ruler in Phase 0.

⚠ **Two consequences of a 51 mm grip height:**

1. **Short sticks.** ✅ **Resolved (N3):** the minimum stick length is raised
   to **80 mm** rather than lowering the grip. The shortest stick then extends
   ~29 mm above the grip point.

   The limit applies to the **physical stick length** (`length_m` in the build
   file) — the thing that is cut and gripped — not to the expanded edge length
   in the mesh (§8.4). Two distinct limits, do not conflate:

   | Limit | Value | Nature |
   |---|---|---|
   | `min_stick_length` | **80 mm** (default) | The user's chosen stock threshold. A config field — lower it if short sticks turn out to be wanted. |
   | Hard physical floor | ~**66 mm** | `grasp_offset_from_base` (51 mm) + jaw height margin. Below this the jaws close at or above the stick's tip. Never configurable below this. |
2. **Gripper clearance at joints (new validation rule).** The jaws sit only
   ~51 mm above the stick's base — i.e. right where the glue joint and the
   neighbouring already-placed sticks are. The jaws are ~20 mm across. So a
   placement is only *physically* achievable if the cone around the target
   vertex, out to ~60 mm, is clear of already-placed sticks. This must be
   checked alongside IK; a design can be perfectly reachable and still
   unbuildable because the gripper cannot fit into the joint. Model the jaws
   as a simple box swept along the approach and collision-check it.

TCP goal for a place:
```
T_target_tcp = T_stick_base_target ∘ translate(grasp_offset_from_base along stick axis)
                                   ∘ inverse(T_grasp_in_tool)
```
Implement this once in `so_arm_100_kinematics`, unit-test it, and never
duplicate it.

### 8.3 `StickSpec` — the canonical stick description

Used by the action goals *and* by the wire protocol (identical fields):

| Field | Type | Meaning |
|---|---|---|
| `id` | string | Stable identifier from Blender |
| `base` | `float64[3]` | m, in `base_link` — the end that seats against the table/another stick |
| `tip` | `float64[3]` | m, in `base_link` — the free end |
| `roll_deg` | `float64` | rotation about the stick's own axis (matters: the stock is **square**, not round) |
| `length_m` | `float64` | ‖tip − base‖; cross-checked against the nominal stock length |
| `section_m` | `float64[2]` | cross-section, default `[0.006, 0.006]` |

`base`+`tip`+`roll` is far more natural for a stick lattice than a pose, and
it makes "which end goes down" explicit — which is exactly the information
needed to compute the grasp offset above.

---

### 8.4 Joint allowance — 3.25 mm per shared end

Wireframe edges meet at a mathematical point; 6.45 mm square sticks cannot.
Every stick therefore stops **3.25 mm short** of each vertex it shares with
another stick, leaving a gap for a glue blob. An end that seats on the base
plate is **not** shared and gets no gap.

⚠ **The design is grown, not the sticks shortened.** Rather than cutting every
stick to fit the mesh, the addon keeps the stick lengths as drawn and moves
the mesh vertices apart:

```
required_edge_length = stick_length + 3.25 mm × (number of shared ends)
```

Full treatment, including the constraint solve for looped structures, is in
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) §5.2.1–§5.2.3. **What matters
on the ROS2 side:**

- The build file carries the **physical stick endpoints** — already inset from
  the ideal vertices. ROS2 does no gap arithmetic; it places what it is given.
- `length_m` is the real stick length, which is what the operator cuts and
  loads. There is no separate "cut length" any more.
- ⚠ **The built structure is larger than the mesh the user drew**, by ~3.25 mm
  per joint along any path. Validation therefore runs on the expanded
  geometry, and a design that fit the build volume before expansion may not
  after. Blender validates the expanded mesh; ROS2 re-validates whatever the
  file contains, so this needs no special handling here — but do not be
  surprised when the numbers do not match the user's design dimensions.

**Why 3.25 mm is the right default.** For two sticks of width `w` meeting at
angle θ, the interpenetration extends roughly `w / (2·tan(θ/2))` along each
axis. At θ = 90° that is `6.45 / 2 = 3.225 mm` — so 3.25 mm makes each stick
stop exactly at the neighbour's face for a perpendicular joint.

⚠ **Shallow angles need more.** At θ = 45° the required allowance is
`6.45 / (2·tan 22.5°) ≈ 7.8 mm`, more than double. A fixed 3.25 mm under-cuts
those joints and the sticks will physically clash. The user is gluing with a
glue gun and has accepted gaps and blobs, so a **fixed** allowance is the
default — but the addon should compute the angle-based value per vertex and
**warn** when it exceeds the fixed one. Cheap to implement, and it catches a
class of design that would otherwise simply not fit together.

## 9. Kinematics — solved analytically, with numbers

**This was the biggest technical risk in the project. It is now largely
resolved:** the chain has a clean closed form, it has been derived from the
URDF and **validated against the working hardware poses**, and the reachable
envelope has been computed. Everything in §9.2–§9.5 is measured, not assumed.

### 9.1 Why KDL will fight you

`kdl_kinematics_plugin` solves for a **full 6-DOF pose** using an iterative
Newton method. This arm has **5 DOF**. A 5-DOF chain cannot reach an
arbitrary 6-DOF pose, so for most requested poses KDL simply iterates to its
timeout and returns "no solution" — even when a perfectly good placement
exists. Historical "that pose is unreachable" arguments in this project trace
partly to this, not only to true workspace limits.

Mitigations, in increasing order of quality:

1. `position_only_ik: true` in `kinematics.yaml` — makes KDL solve position
   only, ignoring orientation entirely. Fast to try, but throws away the
   orientation control that matters for placing a stick upright.
2. Install `trac_ik_kinematics_plugin` — better convergence, still fights the
   dimensionality mismatch.
3. ✅ **Write a closed-form solver for this specific chain** (recommended).

### 9.2 Why closed-form is easy here and is the right call

The chain is the classic *base yaw + planar 3R + tool roll*:

- `Shoulder_Rotation` is the only joint that rotates about the base Z. It is
  therefore **fully determined** by the target's `atan2(y, x)` (plus the
  180°-flipped branch).
- `Shoulder_Pitch`, `Elbow`, `Wrist_Pitch` form a planar 3R arm inside that
  vertical plane → standard 2-link closed form once the tool pitch angle is
  chosen, with the tool pitch as the single free parameter to sweep.
- `Wrist_Roll` sets the rotation about the tool axis.

Consequences to build into the design:

- Solutions are **exhaustive and instant** — sweep the free parameter and you
  have every solution, with no timeouts, no seeds, no randomness.
- **Reachability maps come free.** Sample the build volume on a grid, mark
  each cell reachable/not, cache it. This is what Blender needs to show the
  user a "you may only design inside here" volume (see the Blender doc).
- Feed the resulting **joint angles** to MoveIt as a `joint` goal — the mode
  that finding #11 says is by far the most reliable on this arm. The whole
  brittle "pose goal → planner IK → maybe" path disappears.

### 9.3 The closed form (derived and validated — implement exactly this)

Link parameters read straight out of
`so_arm_100_description/urdf/so_arm_100_5dof_arm.urdf.xacro`:

| Segment | Length |
|---|---|
| `Shoulder_Rotation` axis → `Shoulder_Pitch` axis | 30.6 mm radial, 119.0 mm up |
| `Shoulder_Pitch` → `Elbow` (upper arm, `L1`) | **116.0 mm** |
| `Elbow` → `Wrist_Pitch` (forearm, `L2`) | **135.0 mm** |
| `Wrist_Pitch` → TCP (`End_Effector`, `L3`) | **150.1 mm**, along the tool axis |
| `Shoulder_Rotation` axis offset from `base_link` origin | (0, −45.2 mm) |

Three facts that make this easy, all verified numerically:

1. **The chain after `Shoulder_Rotation` is planar.** Every subsequent joint
   origin has `x = 0` in its parent frame and every pitch axis is `(1,0,0)`,
   so the whole arm lies in one vertical plane through the rotation axis.
   ⇒ `Shoulder_Rotation` is fully determined by the target's azimuth.
2. **Tool elevation = −(`Shoulder_Pitch` + `Elbow` + `Wrist_Pitch`)**, exactly.
   Checked against all five tuned poses in `pick_and_place.yaml`:
   `place` = 68 − 17 − 51 = 0 → tool elevation 0.0°; `home` = −99.98 + 85.94 +
   71.62 = 57.58 → −57.6°. No exceptions.
3. **The stick is held perpendicular to the tool axis.** At the tuned `lower`
   pose the tool axis is horizontal and the stick in the hole is vertical.
   `Wrist_Roll` then spins the stick around the tool axis.

**FK validation:** the analytic model puts the `lower` pose's TCP at
`[0.3835, −0.0251, 0.0648]`; `stick.pose` in the yaml — tuned by hand against
the real hardware — is `[0.379, −0.026, 0.064]`. **Agreement within 5 mm.**
The model is trustworthy; build on it.

**IK recipe:**
```
azimuth   -> Shoulder_Rotation        (planar chain; one solution + a flipped branch)
tool elev -> Shoulder_Pitch + Elbow + Wrist_Pitch = -elevation   (one constraint)
wrist centre = TCP - L3 * tool_axis   -> standard 2-link (L1, L2) closed form
Wrist_Roll -> rotates the stick about the tool axis to the desired stick roll
```

### 9.4 What orientations are placeable, and the reachable envelope

Because the stick is perpendicular to the tool axis (fact 3) and the tool must
lie in the arm's vertical plane (fact 1):

| Desired stick orientation | Required tool axis | Feasible? |
|---|---|---|
| **Vertical** | horizontal, pointing radially outward | ✅ the common case; largest envelope |
| **Horizontal, tangential** (perpendicular to the arm's reach direction) | horizontal, radial | ✅ same poses, different `Wrist_Roll` |
| **Horizontal, radial** (pointing away from the robot) | vertical (tool pointing down) | ✅ but costs ~150 mm of reach |
| **Tilted in the arm's plane** (leaning toward/away from the robot) | tilted correspondingly | ✅ |
| **Tilted out of the arm's plane** (leaning sideways) | would need independent tool yaw | ❌ **not achievable with 5 DOF** |

The last row is a genuine 5-DOF limitation — no solver fixes it. It is exactly
what the future **rotating table** buys back, since rotating the workpiece
changes which plane a given stick lies in.

**Measured envelope for vertical sticks** (tool horizontal; `base z` is the
height of the stick's bottom end, = TCP z − 51 mm):

| stick base z | min TCP radius | max TCP radius |
|---:|---:|---:|
| 0 mm | **311 mm** | 469 mm |
| 50 mm | 268 mm | 476 mm |
| 100 mm | 258 mm | 475 mm |
| 150 mm | 220 mm | 464 mm |
| 200 mm | 154 mm | 442 mm |
| 250 mm | 102 mm | 403 mm |
| 300 mm | 142 mm | 331 mm |

Absolute maximum TCP reach is **477 mm** (fully extended, horizontal tool).
Note the **minimum** radius: at table height nothing closer than ~**311 mm**
can be placed with a vertical stick — the arm cannot fold in that tightly.
This surprises people; the near edge of the build area is as much of a
constraint as the far edge.

### 9.5 Sizing the build volume (why the requested one was changed)

The volume from Q6 (300 × 200 × 300 mm centred at Y = −350 mm) spans radii
250–474 mm and heights 0–300 mm. Sampled on a 25 mm grid against the envelope
above, **only ~85 % of it is reachable for vertical sticks.** The misses are
the near-bottom region (radius < 311 mm at table height) and the far-top
corners (radius > ~450 mm above 200 mm height).

Alternatives, scored the same way:

| Volume (W × D × H) | Centre Y | Vertical-stick reachability |
|---|---|---|
| 300 × 200 × 300 mm *(as requested)* | −350 mm | 85.2 % |
| 240 × 160 × 200 mm | −370 mm | 98.0 % |
| 260 × 140 × 220 mm | −380 mm | 96.9 % |
| **300 × 120 × 200 mm** | **−380 mm** | **99.2 %** ✅ |

✅ **Chosen (N2): 240 (W) × 160 (D) × 200 (H) mm centred at Y = −370 mm** —
98.0 % reachable. Keeps more depth than the 99 % option at the cost of 60 mm
of width. In `base_link`: X ∈ [−0.12, +0.12], Y ∈ [−0.29, −0.45],
Z ∈ [0, 0.20] m. This box is the reference volume drawn in Blender and the
bounds written into every build file.

Caveats on these numbers: they cover **vertical sticks only**, and they ignore
**self-collision, the mount platform, the table surface, and collisions with
already-placed sticks**, all of which shrink the usable set further. Treat
them as an upper bound, and let the Phase 1 reachability map (which includes
collision checking) be the authority.

---

## 10. Planning-scene management

As the sculpture grows, it becomes an obstacle field.

- Every released stick is re-added as a persistent collision box
  `placed_<stick_id>` (§7.3 step 3). Never let them vanish — the robot must
  plan around what it has already built.
- Represent each stick as a **box** (`add_collision_box` is a plain topic
  publish and is the proven-reliable mechanism). Boxes are conservative for a
  square stick — good.
- Consider a small inflation (1–2 mm) on placed sticks so plans don't graze
  fresh glue.
- Add the `build_table` surface as a static collision box at startup.
- On restart, the scene must be **rebuilt from the build plan's `placed`
  states** — hence build-plan persistence (§11, Phase 5).
- Keep `stick` (the one in the feeder) as a separate, always-present object;
  it is removed/re-added only during the grasp close (finding #7).

### 10.1 Validation must be order-aware

The user's Q4 request — "before starting, tell me which placements are
impossible" — cannot be answered by checking each stick independently against
an empty workspace. **Reachability shrinks as the sculpture grows.** A stick
in the middle of the lattice may be trivially reachable on day one and
completely walled in by the time its turn comes.

So `ValidatePlacements` must **simulate the whole build in order**:

```
scene = {table, base_plate}
for stick in build_order:
    check IK reachable (§9.3)
    check jaw clearance at the target vertex (§8.2)
    check approach path collision-free against `scene`
    record the verdict, then add the stick to `scene` and continue
```

A consequence worth stating plainly: **a design's buildability depends on the
build order**, so validation and ordering are the same problem. The ordering
algorithm lives on the Blender side (see
[`BLENDER_ADDON_PLAN.md`](BLENDER_ADDON_PLAN.md) §6) and ROS2 re-validates the
order it is given rather than inventing its own — one authority, no drift.

### 10.2 Loop closure — a physical risk to warn about

A wireframe mesh contains closed loops (any triangle, any quad face). The
**last** stick of a loop must fit between two vertices that are already glued
in place, so it absorbs all accumulated positioning and cutting error. With a
6.45 mm stick and hand-applied glue, that error will not be small.

Mitigations to offer the user: cut loop-closing sticks ~1 mm short and let
glue fill the gap; order the build so loop closures happen while at least one
end is still adjustable; or flag loop-closing sticks in the UI as high-risk so
they can be hand-fitted. This is a *design* warning, not something the
software can fix.

---

## 11. Phase plan

Each phase is independently testable and leaves the system working.

### Phase 0 — Fix the geometry foundation *(blocking; no code)*
- [ ] Measure and set `mount_platform_joint`'s `origin xyz/rpy` in
      `so_arm_100_description/urdf/so_arm_100_5dof_arm.urdf.xacro` (D1).
- [ ] Add the `build_table` frame (§8.1) as a fixed joint / static TF, and a
      collision box for the table surface.
- [ ] Physically measure and record: feeder hole position & depth, stock
      cross-section (6.45 mm), `grasp_offset_from_base` (predicted ≈51 mm).
- [ ] ✅ **N3: no retuning needed.** The `lower` step keeps its tuned value;
      the minimum stick length was raised to 80 mm instead. Just **confirm the
      51 mm grip offset with a ruler** — it is currently derived from the
      config, not measured — and update §8.2/§9.4 only if it differs
      materially.
- [ ] Measure the jaw envelope (width, depth, how far they protrude past the
      TCP) for the clearance check in §8.2 consequence 2.
- [ ] Re-verify the existing tuned `pregrasp`/`lower` joint constants still
      look correct in RViz *after* the platform origin changes.
- **Done when:** the RViz model visually matches reality, and the stick
  collision box lands on the real stick.

### Phase 1 — Kinematics & reachability map — 🟡 PARTLY DONE

Promoted ahead of the refactor because it answers "what can actually be
built", which the user needs **before** designing a sculpture (§9.5).

- [x] ✅ **`so_arm_100_kinematics` built** — closed-form FK + IK exactly as in
      §9.3. **Pure Python, `math` is the only import**, verified to run under
      `env -i` with no ROS sourced (the condition it faces in Blender).
- [x] ✅ **Unit tests pass (11/11)**, in `test/test_chain.py`: the `lower`
      pose's FK lands within 6 mm of `stick.pose`; `ik()` recovers **all five**
      tuned poses' original joint angles from their own FK output (within
      0.5°, not merely "some valid solution"); a 343-point grid round-trips;
      `Wrist_Roll` is proven not to move the TCP.
- [x] ✅ **Cross-checked against MoveIt's own KDL solver** reading the live
      `robot_description` — an independent implementation, which is what
      catches a transcription error or a URDF that has moved on without
      `constants.py`. Max error **0.000 mm** (FK) / **0.002 mm** (IK) over 80
      random samples. Tool: `ros2 run so_arm_100_pick_and_place
      verify_kinematics` (needs only `moveit.launch.py`, i.e. `move_group`
      alone — no controllers, no serial port, safe with hardware plugged in).
- [x] ✅ **Validated on real hardware.** The arm was commanded to
      `ik()`-computed targets and physically reached them, confirmed against
      a collision box placed at the intended coordinates. Tool:
      `ros2 run so_arm_100_pick_and_place verify_kinematics_hardware`
      (plan → RViz ghost preview → Enter-gated execute; never auto-moves).
      ⚠ Re-run this after Phase 0 changes `mount_platform_joint`'s origin.
- [ ] Grasp-offset transform (§8.2) and the jaw-clearance test (§8.2) —
      `GRASP_OFFSET_M` is in `constants.py` but the transform helper and the
      swept-jaw collision test are not written yet.
- [ ] Reachability map generator: `envelope.sweep_envelope()` exists and gives
      min/max radius per height, but does **not** yet model self-collision,
      the mount platform, the table, or placed sticks. Add those, plus a
      cached output file and a CLI to regenerate it.
- [ ] Re-confirm the build volume against the collision-aware map (§9.5's
      numbers are IK-only and are an upper bound).
- **Done when:** the collision-aware map exists and the tuned `place` pose is
  marked reachable by it.

⚠ **Two caveats recorded in the code, not to be forgotten:**
- `GRASP_OFFSET_M = 0.051` is *derived*, not measured — Phase 0's ruler check.
- The `stick_roll_rad = 0 ⇒ Wrist_Roll = π/2` mapping is a **software
  convention** chosen to match the tuned poses, not something the URDF
  dictates. Confirm the sign on hardware before trusting it for a real build.

### Phase 2 — Refactor without behaviour change
- [ ] Split `pick_and_place_node.py` into `motion.py` (MoveIt2 wrapper, step
      primitives, preview publisher, prompt), `scene.py` (collision objects),
      `sequences.py` (the composed step lists).
- [ ] `pick_and_place_node.py` becomes a thin `main()` over those — **the
      interactive demo must behave exactly as it does today.**
- [ ] Fix D2/D3: bring `README.md`'s pick-and-place section in line with the
      current config schema and document the timing findings (§3 #2–#4).
- **Done when:** a full interactive run on hardware is indistinguishable from
  today's.

### Phase 3 — Parametric place
- [ ] Drive the demo's `place` step from a `StickSpec` (§8.3) through the
      Phase 1 IK instead of hand-tuned joint angles.
- [ ] Feed the resulting joint angles to MoveIt as a **`joint` goal** — the
      mode finding #11 says is by far the most reliable here. The brittle
      "pose goal → planner IK → maybe" path disappears entirely.
- **Done when:** placing via a `StickSpec` lands in the same spot as today's
  hand-tuned `place` pose, verified on hardware.

### Phase 4 — Task server
- [ ] New `so_arm_100_stick_msgs` with the actions/services in §5.1.
- [ ] `stick_task_server_node.py`: the state machine in §6, the three
      sequences in §7, `TaskState` publishing, planning-scene bookkeeping
      (§10).
- [ ] `ValidatePlacements` — batch check returning per-stick reachable/reason.
      ⚠ **Must be order-aware** (§10.1): validate by simulating the build in
      order, since a placement that is reachable in an empty scene can become
      unreachable once its neighbours exist.
- [ ] **Tell the human which stick to load** (Q1): every `PickStick` result
      and `TaskState` carries the next stick's id and length, and the node
      logs it prominently.
- **Done when:** an entire pick→place→release cycle runs from three
  `ros2 action send_goal` invocations, with no Blender involved.

### Phase 5 — Build execution & robustness
- [ ] Load a build file (see [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) Part A)
      and execute it stick by stick, with the human gates.
- [ ] Persist per-stick status; resume after restart by rebuilding the
      planning scene from placed sticks (§10).
- [ ] Failure handling: a stick that fails IK or execution is marked and
      skipped, not fatal to the build.
- [ ] Retry / re-pick after a failed grasp verification.
- [ ] *(Only if the live-bridge option is chosen — §5.2)* implement
      `so_arm_100_blender_bridge` per the protocol doc, including its
      `--mock` mode.

### Phase 6 — Rotating table *(future)*
- [ ] Make `build_table` a revolute joint (either a 6th controlled servo, or
      a manually-indexed table whose angle is published as a TF/parameter).
- [ ] Extend IK: the table angle becomes an extra free parameter — for each
      stick, search over table angles for the one that makes the placement
      feasible and comfortable, then command the table first.
- [ ] Protocol: add `table_angle_deg` to the place command. Because everything
      on the wire is already `base_link`-relative and the transform chain
      already goes through `build_table`, this is additive.

---

## 12. Safety & human-in-the-loop

- **The human's hands enter the workspace during `AT_PLACE`.** The state
  machine must refuse every motion command in that state except `Abort` and
  `ReleaseStick`, and `ReleaseStick`'s first motion (opening the gripper) is
  small and predictable.
- Provide an `Abort` that cancels the active action *and* stops the
  controller, reachable from Blender and from the terminal.
- **Servo thermal load while holding** — see §12.1. Do not silently disable
  torque as a workaround.
- Keep `velocity_scaling`/`acceleration_scaling` at 0.2 until the build cycle
  is proven; they are per-run config, not code.

### 12.1 Servo temperature monitoring (N6)

The arm holds `AT_PLACE` statically — extended, loaded, for up to a minute per
joint, dozens of times per build. That is the worst thermal case for the
STS3215s, and torque droop from a hot servo would show up as placement drift
long before anything fails outright.

**The data is already there and free.** `so_arm_100_hardware`'s read loop
already calls `st3215_.ReadTemper(-1)` every cycle
(`src/so_arm_100_interface.cpp:624`), from the same cached `FeedBack()` read
as position/velocity/effort — so it costs **no extra bus traffic**. It is
currently only printed at `RCLCPP_DEBUG` and then discarded.

Proposed work (small, and worth doing before the first long build session):

- [ ] Expose temperature as a per-joint **state interface** (e.g.
      `<joint>/temperature`) on the ros2_control system, alongside voltage and
      current, which are read in the same place and equally discarded.
- [ ] Publish a `diagnostic_msgs/DiagnosticArray` with WARN/ERROR thresholds
      (STS3215 datasheet limit is ~70 °C; warn well below).
- [ ] Have the task server **surface temperature in `AT_PLACE`** and warn the
      operator if a servo is climbing — the operator controls the glue time,
      so a warning is directly actionable.
- [ ] Log peak temperature per build so the real duty cycle is measurable
      rather than guessed.

Mitigations if it does become a problem: a holding pose closer to the base
(less gravity torque), a mechanical rest/jig supporting the stick during
gluing, or simply pausing between sticks.

---


## 13. Open questions

**All of Q1–Q8, QB1–QB5 and N1–N6 are answered** — see §1.1 and §1.2. Nothing
is blocked on the user.

New questions raised by the N4 mesh-expansion model (§8.4,
`BLENDER_ADDON_PLAN.md` §5.2.1), to settle once there is something to look at
rather than by asking up front:

| # | Question | Blocks |
|---|---|---|
| **N7** | **Uniform vs. per-edge growth.** Adding 3.25 mm at *every* end (free ones too, so free ends overhang harmlessly) makes every edge grow by exactly 6.5 mm — a far better-conditioned constraint solve on looped structures. Offer as a toggle and see which the user prefers in practice. | Nothing; implement both. |
| **N8** | **Residual tolerance.** How much per-edge length error is acceptable after the relaxation solve on a looped design, before the addon calls it unbuildable? Suggest 0.5 mm and revisit against a real print. | Only the warning threshold. |
| **N9** | **First-layer accuracy (from N5).** With no sockets or markings, the first layer's position depends entirely on the arm's own accuracy and the operator's glue hand. Worth a dedicated calibration/test before a tall build — an error at layer 1 propagates through everything above it. | Nothing yet; a test to schedule. |

✅ **The §9.4 envelope table and §9.5 percentages stand as computed.** N3's
final answer (raise the minimum stick length rather than lower the grip) keeps
`grasp_offset_from_base` at ≈51 mm, so nothing needs recomputing. The only
follow-up is Phase 0's ruler check of that 51 mm; re-run the envelope
calculation only if the measurement differs materially.
