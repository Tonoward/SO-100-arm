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

## Current state (as of 2026-08-01)

**Shared kinematics — `so_arm_100_kinematics` v1.2.0**, now identical (incl.
`jaw_clearance.py`, tests re-run and passing, 42/42) in the ROS2 package
itself and in the Blender workspace's own `so_arm_100_kinematics/` staging
copy. Includes the grasp-orientation transform (`grasp.py`, since 1.1.0) —
see `ROS2_IMPLEMENTATION_PLAN.md` §9.6 — and, as of 1.2.0, the swept-jaw
clearance pre-filter (`jaw_clearance.py`) — see §8.2 consequence 2. **⚠ Not
yet re-vendored into `so100_builder/kinematics/`** (the copy the addon's own
Blender interpreter actually imports) — that step, and adding
`test_kinematics_jaw_clearance_vendored.py` alongside the other vendored
test copies, is still the Blender-side agent's own job per the package
README's vendoring rules.

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
(refactor) and 3 (parametric place) are software-done as of 2026-08-01**
(`pick_and_place_node.py` split into `motion.py`/`scene.py`/`sequences.py`;
new `stick_spec.py` drives the `place` step from a real stick's base/tip
via `so_arm_100_kinematics`, with the base/tip flip-retry now wired in) —
verified by unit tests (48/48), a MoveIt-only dry run (no hardware,
`move_group` + a throwaway fake `/joint_states` publisher), and a MoveIt
cross-check placing a real stick from the user's own Blender-exported build
file (`SO100BlenderTest.build.build.build.json`'s `s_004`). **Not yet
confirmed on real hardware** — see `ROS2_IMPLEMENTATION_PLAN.md` §11
Phase 2/3's own "Done when". Phases 4–6 (task server, build execution,
rotating table) have not been started. See `ROS2_IMPLEMENTATION_PLAN.md`
§11 for the full phase plan and each phase's "done when."

---

## Open items

Format: `[owning side]` short description — pointer.

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
- `[Blender]` `so_arm_100_kinematics` v1.2.0 (`jaw_clearance.py`) is synced
  into the Blender workspace's own `so_arm_100_kinematics/` staging copy,
  but **not yet re-vendored into `so100_builder/kinematics/`** — the copy
  the addon's Blender interpreter actually imports. README §"Vendoring into
  the Blender addon" has the exact steps (copy verbatim, re-point the test
  imports, add `test_kinematics_jaw_clearance_vendored.py`).
- `[ROS2]` `jaw_clearance.check_jaw_clearance()` is built and tested but has
  no caller yet — nothing in this workspace invokes it during a real
  placement. Needs wiring into Phase 4's task server (and into
  `ValidatePlacements`, §10.1) once it exists.
- `[ROS2, physical]` Phases 2 (refactor) and 3 (parametric place) are
  software-verified only (unit tests, MoveIt-only dry run) — neither has
  been confirmed on real hardware yet. `ROS2_IMPLEMENTATION_PLAN.md` §11's
  own "Done when" for both phases requires it. Suggested order: reproduce
  the tuned `place` pose via `stick_spec` mode first (confirms no
  regression), then place a real stick from the build file (the actual
  Phase 3 milestone).

## Resolved

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
