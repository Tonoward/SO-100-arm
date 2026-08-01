# so_arm_100_kinematics

Closed-form forward and inverse kinematics for the SO-100 5-DOF arm.

**Pure Python. Stdlib `math` is its only import. Zero ROS dependencies.**
That is a hard design constraint, not an accident — this package is
**vendored verbatim into the Blender addon** so the addon can validate
designs offline, on Windows, with no ROS installed. See
`docs/BLENDER_ADDON_PLAN.md` §2 in the parent repo.

Current version: **1.2.0** (see `VERSION` and `__init__.__version__`).

---

## Quick use

```python
import math
import so_arm_100_kinematics as sak

# Forward: joint angles (radians, 5 values) -> TCP position + rotation matrix
pos, rot = sak.fk([0.0, 0.94, -0.12, -0.80, 1.57])

# Inverse: TCP position -> joint angles. tool_elevation 0 == horizontal tool
# == the orientation that holds a stick VERTICAL.
try:
    joints = sak.ik((0.30, -0.35, 0.10), tool_elevation_target_rad=0.0)
except sak.Unreachable as exc:
    print("no solution:", exc)

# Cheap boolean form, tries both elbow branches:
ok, reason = sak.is_reachable((0.30, -0.35, 0.10), 0.0)

# Grasp orientation: turn a stick's own base/tip into IK-consumable
# (tool_elevation_rad, stick_roll_rad), or straight to joint angles.
joints = sak.solve_stick_placement((0.0, -0.36, 0.0), (0.0, -0.36, 0.110))

# Jaw clearance: will the gripper clip already-placed sticks reaching this
# joint? Exclude the stick(s) THIS one glues onto -- see jaw_clearance.py's
# docstring for why.
clear, reason, clearance_m, _idx = sak.check_jaw_clearance(
    (0.0, -0.36, 0.0), (0.0, -0.36, 0.110), placed_sticks=[])
```

**"TCP" here means Tool Center Point** (the `End_Effector` link) — never the
network protocol. See the terminology note in
`docs/ROS2_IMPLEMENTATION_PLAN.md` §0.

## Conventions that are NOT obvious

| | |
|---|---|
| Joint order | `Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll` (`sak.JOINT_NAMES`) |
| Angles | **radians** everywhere in this API. The yaml configs and RViz use degrees; convert at the boundary. |
| Frame | `base_link`, metres, right-handed Z-up (same handedness as Blender) |
| `tool_elevation_target_rad` | Angle of the tool axis above/below horizontal. **0 = horizontal tool = vertical stick.** Exactly `-(q1+q2+q3)`. |
| Stick orientation | The stick is held **perpendicular** to the tool axis. This is why a vertical stick needs a horizontal tool. |
| `stick_roll_rad` | Rotation about the stick's own axis. Maps to `Wrist_Roll = π/2 + stick_roll_rad`. ⚠ **The π/2 offset is a software convention** chosen to match the hand-tuned poses, not something the URDF dictates — see caveats below. |

## Validation status

Validated at three independent levels:

1. **Offline unit tests** (`test/test_chain.py` + `test/test_grasp.py` +
   `test/test_jaw_clearance.py`, 42 tests) — `ik()` recovers all five
   hand-tuned hardware poses' original joint angles from their own FK
   output, to <0.1°. Not "some valid solution" — the *same* one.
   `test_grasp.py` additionally locks in the grasp-orientation transform's
   exact real-world regression cases (an ill-conditioned elevation branch, a
   too-tight anchor tolerance, and an unhandled asymmetric `Wrist_Roll`
   limit — see `grasp.py`'s own docstring). `test_jaw_clearance.py` checks
   the segment-distance math against known geometric cases (parallel, skew,
   intersecting, degenerate) and the swept-jaw capsule model itself.
   ```bash
   python3 -m unittest discover -s test
   ```
2. **Cross-checked against MoveIt's own KDL solver**, reading the live
   `robot_description` — a completely independent implementation. Max
   position error **0.000 mm** (FK) and **0.002 mm** (IK) over 80 random
   samples. This is what catches a transcription error against the URDF.
   ```bash
   ros2 launch so_arm_100_moveit_config moveit.launch.py rviz:=false
   ros2 run so_arm_100_pick_and_place verify_kinematics
   ```
3. **Real hardware** — the arm was commanded to `ik()`-computed targets and
   physically reached them, confirmed against a collision box placed at the
   intended coordinates.
   ```bash
   ros2 launch so_arm_100_moveit_config pickandplace_demo.launch.py
   ros2 run so_arm_100_pick_and_place verify_kinematics_hardware
   ```

## ⚠ Known caveats — do not treat these as settled

- **`GRASP_OFFSET_M = 0.051` is derived, not measured.** It comes from the
  tuned `lower` pose's FK height minus the feeder stick's estimated base
  height. Confirm with a ruler (parent repo's Phase 0). Everything about
  stick placement depends on it.
- **The `stick_roll_rad = 0 → Wrist_Roll = π/2` mapping is unverified on
  hardware.** Every test so far used roll = 0, so the *sign* of a non-zero
  roll has never been checked physically. Confirm before trusting it for a
  real build — the stock is square, so a wrong roll sign is visible.
- **`envelope.sweep_envelope()` is IK-only.** It does not model
  self-collision, the mount platform, the table, or already-placed sticks.
  It is a fast upper-bound pre-filter, never a final verdict.
- **`JAW_RADIUS_M` (used by `jaw_clearance.check_jaw_clearance()`) is an
  estimate, not measured.** Same status as `GRASP_OFFSET_M` — confirm real
  jaw dimensions in Phase 0. The capsule-vs-capsule model is also a
  deliberate simplification of the real (roughly box-shaped) jaws and
  square-section sticks — conservative in most orientations, but not a
  substitute for MoveIt's own collision checking. See `jaw_clearance.py`'s
  own docstring for what it does and does not decide (notably: it does
  **not** know which already-placed stick is this joint's own neighbour —
  the caller must exclude that one before calling).

---

## Vendoring into the Blender addon

Copy the inner module directory, not the ROS package wrapper:

```
so_arm_100_kinematics/so_arm_100_kinematics/*.py   ->   so100_builder/kinematics/
so_arm_100_kinematics/VERSION                      ->   so100_builder/kinematics/VERSION
```

`package.xml`, `setup.py`, `setup.cfg` and `resource/` are ROS packaging
only — do **not** copy them into the addon.

**Rules:**

1. **Copy verbatim. Never fork.** If you find a bug, fix it *here* and
   re-vendor. A Blender-side edit means the two sides silently disagree
   about where sticks go, which is the exact failure the shared-module
   design exists to prevent.
2. **Relative imports (`from .chain import ...`) survive the rename** — the
   directory can be called `kinematics/` in the addon and still work,
   because nothing imports the package by its own name internally.
3. **Bump `__version__` *and* `VERSION` together** on any change that alters
   computed results. Build files record `kinematics_version`; a mismatch at
   load time should be a loud error, not a warning.
4. **Re-run the tests against the vendored copy.** `test/test_chain.py`,
   `test/test_grasp.py` and `test/test_jaw_clearance.py` import
   `so_arm_100_kinematics.*`; after vendoring, change that to the addon's
   package path (see `so100_builder/tests/test_kinematics_vendored.py` and
   `test_kinematics_grasp_vendored.py`, which do exactly this — add a
   `test_kinematics_jaw_clearance_vendored.py` alongside them). If the
   tests don't pass in Blender's interpreter, the copy is wrong.
5. **Keep it numpy-free.** Blender bundles no numpy guarantee. Plain `math`
   only.
