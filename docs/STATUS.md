# Shared status & open-issue log

**Part of the shared docs triad** (alongside `BLENDER_ADDON_PLAN.md`,
`ROS2_IMPLEMENTATION_PLAN.md`, `BRIDGE_PROTOCOL.md`): this file must stay
**byte-identical** between the two workspaces —

- Blender addon: `RobotArm_UbuntuAddon/docs/STATUS.md`
- ROS2 workspace: `ros2_ws/src/SO-100-arm/docs/STATUS.md`

— reconciled by hand, the same way the other three already are. There is no
automated sync between the two repos on purpose (see `BLENDER_ADDON_PLAN.md`
§2: Option C, the slicer/printer split, is a deliberate scope boundary, not
an oversight).

**What this file is for, and what it is not:**

- It is the first thing either side's agent should read at the start of a
  session, to get "what's the current state, what's open" without reading
  the entire phase plan in both design docs.
- It is **not** a duplicate of the design docs' own inline corrections.
  `BLENDER_ADDON_PLAN.md` and `ROS2_IMPLEMENTATION_PLAN.md` already record
  found-and-fixed findings inline, in the context where they matter (e.g.
  §9.4's out-of-plane tilt correction) — that is the right home for anything
  tied to a specific derivation or spec section. This file is for **current,
  cross-cutting, or in-flight** state: what's open right now, which side
  owns it, and a pointer into the design docs for the detail.
- **Append, don't rewrite history.** Move an entry to "Resolved" with a date
  and a one-line pointer to where the real writeup lives, rather than
  deleting it.

---

## Current state (as of 2026-08-03)

**Shared kinematics — `so_arm_100_kinematics` v1.2.0**, now identical (incl.
`jaw_clearance.py`) across all three copies: the ROS2 package itself, the
Blender workspace's `so_arm_100_kinematics/` staging copy, and (as of
2026-08-03) `so100_builder/kinematics/` — the copy the addon's own Blender
interpreter actually imports, previously stuck at 1.1.0 without
`jaw_clearance.py` at all. Includes the grasp-orientation transform
(`grasp.py`, since 1.1.0) — see `ROS2_IMPLEMENTATION_PLAN.md` §9.6 — and, as
of 1.2.0, the swept-jaw clearance pre-filter (`jaw_clearance.py`) — see
§8.2 consequence 2. Re-vendored per the package README's own rules (copy
verbatim, re-point test imports, add
`test_kinematics_jaw_clearance_vendored.py`); all 42 vendored tests pass in
a plain `env -i` Python (no numpy, simulating Blender's bundled
interpreter). Practical effect: a build file exported from Blender now
correctly stamps `kinematics_version: "1.2.0"` (`so100_builder/ops/build.py`
reads it dynamically from the vendored copy) instead of a stale `"1.1.0"` —
matters because ROS2's `build_file.py` loader (Phase 5) hard-refuses to
execute on any `kinematics_version` mismatch.

**Blender addon (`so100_builder`)** — Phases A–E software-complete.
Phase D's hardware run (build a real design on the real robot, confirm
status round-trips) is the one remaining "done when" item and cannot be
exercised from the Blender side alone. See `BLENDER_ADDON_PLAN.md` §11.

**ROS2 workspace (`SO-100-arm`)** — Phase 0 (geometry foundation: measure
`mount_platform_joint` origin, jaw envelope, confirm the 51 mm grip offset
with a ruler) is **blocking and physical** — no amount of agent work
substitutes for someone measuring the real hardware. Phase 1 (kinematics &
reachability map) is partly done: FK/IK, the grasp-orientation transform,
and the jaw-clearance test are built, tested (42/42), and (FK/IK only)
hardware-validated; a collision-aware reachability map generator is not
written yet, and the jaw-clearance check has no caller wired up yet (it's a
primitive Phase 4's task server will call per-placement). **Phases 2
(refactor) and 3 (parametric place) are ✅ DONE, hardware-confirmed
2026-08-01** (`pick_and_place_node.py` split into
`motion.py`/`scene.py`/`sequences.py`; new `stick_spec.py` drives the
`place` step from a real stick's base/tip via `so_arm_100_kinematics`, with
the base/tip flip-retry now wired in) — verified by unit tests (48/48), a
MoveIt-only dry run, and on the real robot: the tuned pose reproduced via
`stick_spec`, then `s_004` from the user's own Blender-exported build file
(`SO100BlenderTest.build.build.build.json`) placed at a computed, not
hand-tuned, location — the first time this system has done that. **Phase 4 (task
server) is ✅ DONE, hardware-confirmed 2026-08-02** — new
`so_arm_100_stick_msgs` interface package, `stick_task_server_node.py`'s
state machine, all three actions, a full `PickStick`→`PlaceStick`→
`ReleaseStick` cycle run end-to-end on real hardware via plain
`ros2 action send_goal` (no Blender, no interactive prompts) with `s_004`.
Getting there took several rounds of real hardware debugging the same
day — the task-server wedge (rclpy bug, two-`Node` fix), a params-file
node-name-scoping bug that left `stick_task_server`/`tune_grasp` silently
running on stale fallback config, a misplaced feeder-stick collision
object, an arm/gripper interface mixup in `ReleaseStick`, and two rounds of
`grasp_verification.gap_threshold` re-tuning once real hardware variance
turned out wider than the first calibration pass suggested — see Resolved
below and `ROS2_IMPLEMENTATION_PLAN.md` §3 findings #12-15 and §4 D8-D12
for the full trail. One deliberately-deferred gap remains (D13: grasp
height doesn't adapt to stick length yet) and one narrower, still-
unexplained dry-run-only symptom from the wedge fix (see Open items).
**Phase 5 (build execution) is built and dry-run-verified; several real
hardware attempts made 2026-08-03/04, none completed a full clean build
yet** — new `Reset` action (the only way back from `ERROR` to `IDLE`,
discovered during planning to be a hard prerequisite, not optional, for
this phase's own "not fatal to the build" requirement), a `build_file.py`
loader compatible with the Blender addon's own build-file/status-sidecar
format (17 unit tests), and a new `build_runner` node that drives
`stick_task_server` through an entire build stick by stick with the human
gates, persists status after each stick, and resumes correctly after a
restart. The real hardware attempts surfaced two genuine bugs (a
physically-placed-but-unregistered stick after a release failure, and
planning-scene debris surviving a `build_runner` restart) — both diagnosed
and fixed, not yet re-verified on hardware; see Open/Resolved below and
`ROS2_IMPLEMENTATION_PLAN.md` §3 findings #17/#18. `ValidatePlacements` and
Phase 6 (rotating table) have not been started. See
`ROS2_IMPLEMENTATION_PLAN.md` §11 for the full phase plan and each phase's
"done when."

---

## Open items

Format: `[owning side]` short description — pointer.

- `[ROS2, physical]` **Full `PickStick`→`PlaceStick`→`ReleaseStick` cycle
  confirmed working end-to-end on hardware 2026-08-02** (`s_004` picked,
  placed at its computed location, released — all three actions
  `SUCCEEDED` via plain `ros2 action send_goal`, no Blender, no interactive
  prompts). Phase 4's own hardware milestone.
- `[ROS2, physical]` `PickStick`'s grip height along the stick
  (`steps.lower.joint_positions`) is a fixed constant tuned for one stick
  length — doesn't adapt to `PickStick.length`, so other lengths would be
  gripped off-center. Deliberately deferred (user priority is a full
  Blender-build-driven build first) — `ROS2_IMPLEMENTATION_PLAN.md` §4 D13.
- `[ROS2, physical]` One remaining piece of the task-server wedge (see
  Resolved below for the part that's fixed): in the no-controller dry-run
  harness, a `PickStick`'s `home position` planning step itself hung
  indefinitely after the two-node fix (server stayed responsive to other
  goals throughout — this is a different, more localized symptom than the
  fixed total-wedge). Not yet understood; needs a real hardware retest —
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 4.
- `[ROS2]` Reachability map generator is IK-only (`envelope.sweep_envelope()`);
  does not yet model self-collision, the mount platform, the table, or
  already-placed sticks — `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 1.
- `[ROS2, physical]` `GRASP_OFFSET_M = 0.051` is derived, not measured.
  Needs a ruler check against the real feeder — `ROS2_IMPLEMENTATION_PLAN.md`
  §11 Phase 0.
- `[ROS2, physical]` `stick_roll_rad = 0 ⇒ Wrist_Roll = π/2` sign convention
  is untested on hardware — every hardware run so far used `stick_roll = 0`.
  Confirm with a non-zero roll before trusting it for a real build (the
  stock is square, so a wrong sign is visible) — `ROS2_IMPLEMENTATION_PLAN.md`
  §11 Phase 1 caveats.
- `[Blender + ROS2]` Phase D's hardware run itself: export a real design,
  build it, confirm the addon's Build panel reflects the real outcome after
  reopening. Needs both sides working together, not schedulable from either
  workspace alone.
- `[ROS2 + Blender]` `jaw_clearance.check_jaw_clearance()` is built and
  tested (both sides, incl. the now-vendored Blender copy) but has no
  caller on **either** side yet — nothing in this workspace invokes it
  during a real placement, and the addon's `core/validate.py` doesn't call
  it either. Needs wiring into Phase 4's task server (and into
  `ValidatePlacements`, §10.1) on this side once it exists; the Blender
  side's own equivalent wiring is that workspace's call.
- `[ROS2, physical]` **Phase 5's own hardware milestone — several real
  attempts made 2026-08-03/04, none completed a full clean build.** One run
  had a `ReleaseStick` failure right after a successful placement; skipping
  that stick left its collision box missing even though it was physically
  glued in place, and a later placement in the same run also failed. A
  second run had a clean pick, then a `PlaceStick` planning failure
  (`INVALID_MOTION_PLAN`) on a stick that should have been reachable, plus a
  restart not clearing a leftover collision object (only closing
  RViz/`move_group` did). A third run's `INVALID_MOTION_PLAN` was root-
  caused via an RViz screenshot: the resume loop had re-attempted a stick
  already `placed` from an earlier session, driving a second physical stick
  straight at that stick's own already-registered collision box. A fourth
  run (fresh sidecar deleted, clean start) got further — pick, place, and
  glue all succeeded for the first stick — but `ReleaseStick` then failed at
  its own `retreat` step with no other collision objects present besides
  that stick's own about-to-be-registered box. All four bugs diagnosed and
  fixed (see Resolved below, `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #17/#18/#19/#20). Not yet re-verified against hardware. Separately, one
  `PlaceStick` run had a visually odd trajectory (dipped toward the floor
  before lifting into place) despite succeeding — not yet root-caused,
  plausibly just OMPL's normal joint-space path variance (finding #11's
  tradeoff); see `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5's note.

## Resolved

- `[ROS2]` Two usability gaps raised directly from the Phase 5 hardware
  attempts above, both addressed 2026-08-04: (1) resuming from the status
  sidecar after closing every terminal and rebuilding looked like unwanted
  caching rather than the intended checkpoint feature — added
  `build_runner`'s `fresh_start` param (`-p fresh_start:=true`) as an
  explicit way to discard the sidecar and start over; the resume behavior
  itself needed no fix, it was working as designed. (2) a planning failure
  only logged a bare MoveIt error code, leaving the operator to dig through
  RViz/`get_planning_scene` to find out why — added
  `motion.py`'s `explain_collision()`, which calls MoveIt's
  `check_state_validity` service on the failed goal configuration and logs
  which two collision bodies are actually touching (e.g. `'stick' vs
  'placed_s_005'`) directly in the terminal; best-effort, never raises.
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5.
- `[ROS2]` Four real bugs found and fixed from actual Phase 5 hardware
  attempts 2026-08-03/04 (see the Open item above for what surfaced them):
  (1) `register_placed_stick` only runs partway through
  `run_release_sequence`, so a `ReleaseStick` failure at its first step
  could leave a physically-glued stick with no collision box and no way to
  recover it — fixed by adding `prompt_stick_physically_placed()` to
  `build_runner`, asked before finalizing any skip/abort; answering yes
  registers the stick's collision box from the build file's own geometry
  and marks it `placed` regardless of which action failed. (2) MoveIt's
  planning scene lives in `move_group`, not in `build_runner`/
  `stick_task_server`, so restarting those two processes doesn't clear
  collision debris a previous crashed run left behind — fixed by having
  `build_runner`'s `main()` deterministically clear the transient `stick`
  id and any `placed_<id>` not currently marked `placed` in the sidecar, by
  id, before rebuilding the scene on resume. (3) The array-position resume
  loop re-attempted `PickStick`/`PlaceStick` on a stick already `placed`
  from an earlier session, driving a second physical stick straight at that
  stick's own already-registered collision box — root-caused via an RViz
  screenshot, fixed by an explicit already-`placed` guard at the top of
  each loop iteration. (4) `run_release_sequence` registered the just-placed
  stick's permanent collision box BEFORE attempting `retreat`, at a pose
  necessarily co-located with the gripper's pre-retreat position — the
  identical "obstacle sitting on the arm's own current pose" bug already
  fixed once for the `"stick"` transient object, resurfacing under the
  permanent box's name — fixed by reordering to retreat first, register
  second (same pose data, applied later; also now registers even if
  retreat itself still fails, since the physical release already happened
  by that point). `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #17/#18/#19/#20. Not yet re-verified against hardware.
- `[Blender]` `so_arm_100_kinematics` re-vendored into
  `so100_builder/kinematics/` 2026-08-03 — was stuck at 1.1.0 without
  `jaw_clearance.py` at all, now matches 1.2.0 exactly (copied verbatim per
  the package README's own rules; `chain.py`/`envelope.py`/`grasp.py` were
  byte-identical already, only `__init__.py`/`constants.py` differed, plus
  the missing module). Added the missing
  `test_kinematics_jaw_clearance_vendored.py`; all 42 vendored tests pass
  in a plain `env -i` Python. A build file exported from Blender now
  correctly stamps `kinematics_version: "1.2.0"` instead of a stale
  `"1.1.0"`. See `~/MagnaRecta/RobotArm_UbuntuAddon/KINEMATICS_VENDOR_UPDATE.md`
  for the full writeup.
- `[ROS2]` Phase 5 (build execution) built and dry-run-verified 2026-08-03:
  new `Reset` action (`so_arm_100_stick_msgs`) as the only way back from
  `ERROR` to `IDLE` — a hard prerequisite discovered during planning, not
  originally requested, since without it a single failure anywhere in a
  multi-stick build would permanently reject every subsequent action, not
  just fail that one stick; `assume_gripper_empty` flag mirrors
  `BRIDGE_PROTOCOL.md` §5.11's own already-designed socket `reset` command.
  New `build_file.py`, a pure-Python build-file/status-sidecar loader
  ported to stay a compatible counterpart to the Blender addon's own
  `so100_builder/io/build_file.py` (same sidecar filename derivation, same
  pending-omission/unknown-status-degrades rules), plus the
  `kinematics_version`/`frame` checks the protocol assigns specifically to
  this side (hard refuse on mismatch, not a warning). New `build_runner`
  node drives `stick_task_server` through an entire build stick by stick
  with the human gates, resumes correctly from a status sidecar (rebuilding
  the scene from already-placed sticks' own base/tip geometry via a new
  `pose_utils.axis_aligned_box_pose()`), and offers retry/skip/abort on any
  failure. Along the way, found and fixed an unrelated bug:
  `stick_task_server.launch.py`'s `name=` parameter was globally remapping
  BOTH of that node's internal `Node`s onto the same displayed name.
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5, §3 finding #16.
- `[ROS2, physical]` A failed `PickStick` grasp left "stick" sitting in the
  world scene exactly at the robot's current pose (re-added unconditionally
  by `plan_grasp_gripper()` so a success case can attach it, never removed
  again on failure) — blocked planning ANY next move, including a manual
  RViz-driven return to home, until the object was removed by hand.
  Confirmed on hardware 2026-08-02. Fixed: `run_pick_sequence` now removes
  "stick" on all three grasp-failure exits. Also: `PickStick`'s result was
  reporting `gripper_gap: 0.0` on every failure instead of the real
  measured gap — fixed so failures are diagnosable going forward.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D12.
- `[ROS2]` Motion speed raised for both the programmatic pipeline and RViz's
  own manual-plan defaults, now that the basic sequence is hardware-proven:
  `pick_and_place.yaml`'s `velocity_scaling`/`acceleration_scaling` 0.2 to
  0.4 (affects `pick_and_place_node`/`stick_task_server`/`tune_grasp`, all
  three read this yaml); RViz's MotionPlanning panel defaults
  (`moveit.rviz`/`moveit.xtra.rviz`) 0.1 to 0.5.
- `[ROS2, physical]` The first-pass grasp tuning below (`-12.0°`/`0.0611`
  rad) stopped being valid the moment D9 was fixed: `stick_task_server`
  started using the yaml's real `lower` pose instead of a stale code
  default, which changed the real grasp geometry. `PickStick` then reliably
  stalled at -12° (hard stop ~-7°, 15s `goal_time_tolerance` watchdog,
  `ABORTED`, stick never lifted) — confirmed on hardware. Re-tuned under
  the now-correct pose with the same empty-vs-holding `tune_grasp`
  methodology, at smaller commanded magnitudes (no usable signal below
  ~-8°, since the jaws haven't reached the stick yet there): new values
  `gripper_grasp_position_deg: -9.0`, `gap_threshold: 0.0244` rad (~1.4°) —
  empty ~0.4° gap, holding ~2.1° gap. Set consistently in `pick_and_place.yaml`
  and all three nodes' code fallback defaults, same as before.
  **Grasp tuning is pose-dependent, not just gripper-dependent** — re-run
  `tune_grasp` (both empty and holding) any time `steps.lower` or the stick
  geometry changes. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2, physical]` The `-9.0°`/`0.0244` rad (~1.4°) values above were
  themselves too tight — from only 2 calibration samples per condition.
  Real `PickStick` runs then produced holding gaps as low as 1.09° (real,
  visually-confirmed grasps rejected as "closed on nothing") alongside
  empty gaps consistently under 0.4°. Re-tuned to `gap_threshold: 0.0157`
  rad (~0.9°) from the fuller picture (holding: 1.09/1.35/2.14/2.23°;
  empty: 0.21/0.30/0.39°) — same day, same D4 entry. `gripper_grasp_position_deg`
  stayed at `-9.0`. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2, physical]` `tune_grasp` couldn't close past -10° at all — even
  -10° itself failed ("Planning failed! Error code: FAILURE"), despite -12°
  being already hardware-proven via `PickStick`. Two things, found in order:
  (1) the Gripper joint's URDF `<limit>` was `-0.1792` rad (~-10.27°) — the
  actual enforced bound for real hardware (`so_arm_100_moveit_config`'s own
  `ros2_control.xacro` has no min/max clamp at all) — widened to `-0.24` rad
  (~-13.75°) in `so_arm_100_5dof_arm.urdf.xacro`. That alone didn't fix it:
  (2) the REAL cause was `tune_grasp_node.py`'s close-test never removing
  the "stick" collision object before planning a close, unlike
  `run_pick_sequence`'s proven remove/plan/re-add pattern — masked until
  today by the D9 pose bug (the box used to sit somewhere harmless), now
  that D9 is fixed the box sits exactly where the jaws close and blocked
  every close as a collision. Fixed with a new `_close_gripper()` mirroring
  the proven pattern. Separately: LeRobot's calibration JSONs
  (`so_leader`/`so_follower`) are not read by this ROS2 workspace at all —
  editing them does nothing here. `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #13/#14/#15.
- `[ROS2]` `run_release_sequence`'s `open gripper at place` step passed the
  ARM's MoveIt2 interface to `motion.run_step()` instead of the gripper's,
  while still planning a gripper-only trajectory (`motion.plan_gripper()`)
  — so it executed and waited on that trajectory through the wrong
  interface. Confirmed on hardware 2026-08-02: `PickStick`/`PlaceStick`
  succeeded, `ReleaseStick` hung on this exact step for an extended period
  before aborting with "planning/execution failed at 'open gripper at
  place'". An isolated copy/paste slip (the equivalent step in
  `run_pick_sequence` already passed the gripper correctly) — fixed by
  passing `gripper`. `ROS2_IMPLEMENTATION_PLAN.md` §4 D10.
- `[ROS2]` Feature: the fed stick's config was a box-center pose + a fixed
  length, so a different-length stick would land off-position without
  retuning the pose constant. Replaced with `stick.base_xyz_m` (the
  stick's physical base at the feeder hole, same convention as
  `StickSpec.base`) + `stick.section_m` + `stick.default_length_m`;
  `pose_utils.feeder_stick_pose()` computes the box center from base +
  length, so any length lands base-down at the same spot. `stick_task_server`
  now threads `PickStick.length` through per-goal (previously ignored) and
  remembers it for sizing `placed_<stick_id>` at `ReleaseStick`.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D11.
- `[ROS2]` `config/pick_and_place.yaml` was scoped to node name
  `pick_and_place_node:`, which a ROS2 params-file only applies to a node
  with that exact name. `stick_task_server` and `tune_grasp_node` are
  different node names, so **neither ever received this yaml's tuned
  values** (regardless of whether `--params-file` was passed) — both
  silently ran on their own code-fallback defaults, including a `stick.pose`
  ~13cm off the real feeder location. This, not the attach/detach drift
  below, is the real explanation for the "misplaced stick, visible from the
  very first action" report 2026-08-02. Fixed: yaml rescoped to `/**:`
  (wildcard node-name match), new `stick_task_server.launch.py` wires
  `--params-file` automatically, `tune_grasp_node.py`'s docstring spells out
  the explicit flag it still needs (must stay `ros2 run` for stdin). Code
  fallback defaults synced to the yaml's real values too, as a safety net.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D9.
- `[ROS2, physical]` `attach_collision_object()` attached the fed stick's
  collision box at whatever pose it currently had in the world scene, and
  `plan_grasp_gripper()` re-added that box at the *static feeder-pose
  constant* before attaching, not the gripper's actual pose. Flagged
  2026-08-02 as cosmetic-only; a second hardware run the same day showed it
  is not: `detach_collision_object()` returns "stick" to the world (rather
  than deleting it) at the pose implied by that wrong offset carried
  through the whole pick→place move, landing it between the feeder and the
  base and blocking `ReleaseStick`'s `retreat` plan. Fixed 2026-08-02:
  `run_pick_sequence` now re-adds "stick" at `motion.get_current_ee_pose()`
  (the same FK technique already proven for `register_placed_stick`)
  instead of the static constant, and `run_release_sequence` now calls
  `scene.remove_stick()` right after detaching, since `placed_<stick_id>`
  is the authoritative record from `ReleaseStick` onward. `ROS2_IMPLEMENTATION_PLAN.md`
  §4 D8.
- `[ROS2 doc]` §9.6 named the per-stick base/tip override `so100_flip`; the
  real field is `flip` (`BLENDER_ADDON_PLAN.md` §9.2, `core/sticks.py`).
  Fixed 2026-07-31.
- `[shared]` Grasp-orientation transform gap — was addon-only, now
  `so_arm_100_kinematics.grasp` v1.1.0, both sides vendor it identically.
  Fixed 2026-07-31 — full writeup in `BLENDER_ADDON_PLAN.md`'s Phase D
  section and `so100_builder/README.md`.
- `[BRIDGE_PROTOCOL.md]` §A.2's worked example had `s_002.shared_ends` wrong
  in one iteration (reasoned from temporary build-order support state
  instead of final topology, which are independent properties). Fixed
  2026-07-31.
- `[ROS2]` Jaw-clearance test (swept-jaw collision check) written —
  `so_arm_100_kinematics.jaw_clearance` v1.2.0. Fixed 2026-07-31 — full
  writeup in `ROS2_IMPLEMENTATION_PLAN.md` §8.2 consequence 2 and §11
  Phase 1. Synced into the Blender workspace's `so_arm_100_kinematics/`
  staging copy (tests re-run and passing there too); still needs
  re-vendoring into `so100_builder/kinematics/` (see the open item above)
  and still has no caller wired up on this side either.
- `[ROS2]` `stick_task_server_node` wedged permanently (every action goal
  on the node hung) after the first goal that involved real motion —
  confirmed on real hardware 2026-08-02, not just the dry-run harness.
  Root cause identified precisely: `rclpy.spin_once()`, which `pymoveit2`
  calls internally, detaches a `Node` from its `Executor` when called from
  within a callback that same `Executor` is dispatching — a known upstream
  bug ([ros2/ros2#1609](https://github.com/ros2/ros2/issues/1609)). Fixed
  2026-08-02 by splitting into two `Node`s (`stick_task_server_motion` for
  `pymoveit2`, `stick_task_server` for the `ActionServer`s), each with its
  own `Executor` — confirmed fixed in the dry-run harness (the server now
  stays responsive to new goals through an in-flight failure). See
  `ROS2_IMPLEMENTATION_PLAN.md` §3 finding #12 and §11 Phase 4. One
  narrower, not-yet-understood symptom remains — see Open items.
- `[ROS2]` `grasp_verification.gap_threshold` was untuned (the old default,
  0.1 rad/~5.7°, was higher than any gap a genuine grasp could produce —
  every real grasp would false-negative). Fixed 2026-08-02: built
  `tune_grasp` (`tune_grasp_node.py`), a new interactive tool that closes
  the real gripper to a commanded position, reads back the actual settled
  position, and reports the verification verdict at several candidate
  thresholds at once. Measured empty-vs-holding on the real gripper/stock;
  new values `gripper_grasp_position_deg: -12.0`,
  `gap_threshold: 0.0611` rad (~3.5°) set consistently in
  `pick_and_place.yaml` and as the code fallback default in all three nodes
  that read it. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2]` Base/tip flip-retry on an unreachable placement — was flagged as
  routine/expected (`Wrist_Roll`'s asymmetric limit, §9.6) but unwired on
  this side, while the Blender side already did it automatically
  (`WARN_AUTO_FLIPPED`). Fixed 2026-08-01: `stick_spec.solve_stick_spec_joints()`
  tries base→tip then tip→base before raising `Unreachable` — see
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 3. Not yet reachable from a real
  `PlaceStick` action (Phase 4 doesn't exist yet), but the retry logic
  itself is built, tested, and available to any caller of `stick_spec.py`.
- `[ROS2 doc]` D2/D3 — root `README.md`'s pick-and-place section documented
  the pre-`steps.<name>.mode` config scheme and omitted the §3 #2-4 timing
  watchdog findings. Fixed 2026-08-01, alongside the Phase 2/3 work that
  made the old table even more stale (added the `stick_spec` step mode).
- `[ROS2, physical]` Phases 2 (refactor) and 3 (parametric place) were
  software-verified only (unit tests, MoveIt-only dry run) pending a real
  hardware run. Confirmed on hardware 2026-08-01: the interactive demo
  behaves as before, and `s_004` from the real build file was placed at a
  computed location — `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 2/3.
