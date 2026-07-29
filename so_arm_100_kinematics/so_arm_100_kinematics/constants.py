"""Geometry and limits for the SO-100 5-DOF arm.

Every number here is read off
``so_arm_100_description/urdf/so_arm_100_5dof_arm.urdf.xacro`` -- this file
has no other source of truth. If the URDF changes, update here and re-run
``test/test_chain.py``, which cross-checks these values against the five
joint-space poses tuned by hand in
``so_arm_100_pick_and_place/config/pick_and_place.yaml``.

Pure Python, stdlib only, zero ROS imports -- this module (and the rest of
this package) is meant to be vendored verbatim into the Blender addon and
must run unmodified in Blender's bundled interpreter (no numpy guarantee).
"""

# Each entry: (name, origin_xyz_m, origin_rpy_rad, axis, limit_lower_rad, limit_upper_rad)
# origin_xyz/origin_rpy are the URDF <joint><origin> values (child frame
# relative to parent, translate-then-rotate); axis is the URDF <joint><axis>.
# All entries after Shoulder_Rotation have origin_xyz.x == 0 -- this is what
# makes the Shoulder_Pitch/Elbow/Wrist_Pitch sub-chain exactly planar (see
# chain.py's module docstring for the consequence this has for IK).
CHAIN = (
    ("Shoulder_Rotation", (0.0, -0.0452, 0.0165), (1.5708, 0.0, 0.0), (0.0, -1.0, 0.0), -1.96, 1.96),
    ("Shoulder_Pitch", (0.0, 0.1025, 0.0306), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -1.745, 1.745),
    ("Elbow", (0.0, 0.11257, 0.028), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -1.5, 1.5),
    ("Wrist_Pitch", (0.0, 0.0052, 0.1349), (-1.57079, 0.0, 0.0), (1.0, 0.0, 0.0), -1.658, 1.658),
    ("Wrist_Roll", (0.0, -0.0601, 0.0), (0.0, 1.57079, 0.0), (0.0, 1.0, 0.0), -2.75, 2.75),
)

# End_Effector_Joint: fixed joint, child of Fixed_Gripper (Wrist_Roll's
# child link), origin xyz="0.0 -0.09 0.0" rpy="0 0 0".
EE_OFFSET = (0.0, -0.09, 0.0)

JOINT_NAMES = tuple(entry[0] for entry in CHAIN)

# --- Grasp / stick geometry -------------------------------------------------
# Distance from the stick's BASE end (bottom, in the feeder hole) to the
# point the jaws close on, along the stick's own axis. Derived from the
# tuned 'lower' pose's FK (z=0.0648 m) vs. the feeder stick's estimated base
# height (z~0.014 m) in pick_and_place.yaml's stick.pose -- NOT yet
# confirmed with a ruler. See ROS2_IMPLEMENTATION_PLAN.md N3/Phase 0.
GRASP_OFFSET_M = 0.051

STICK_SECTION_M = 0.00645  # square stock, both dimensions
STICK_LENGTH_RANGE_M = (0.080, 0.150)  # ROS2_IMPLEMENTATION_PLAN.md N3
JOINT_ALLOWANCE_M = 0.00325  # ROS2_IMPLEMENTATION_PLAN.md N4 / §8.4

# --- Build volume (base_link frame) -----------------------------------------
# ROS2_IMPLEMENTATION_PLAN.md N2: 240 x 160 x 200 mm centred at Y=-370mm.
BUILD_VOLUME_MIN_M = (-0.12, -0.45, 0.0)
BUILD_VOLUME_MAX_M = (0.12, -0.29, 0.20)

# --- Roll convention ---------------------------------------------------------
# All five tuned example poses in pick_and_place.yaml use Wrist_Roll = 90 deg
# for a vertical stick grasped/placed with no extra twist. That is therefore
# defined as the roll_rad=0 reference: Wrist_Roll = pi/2 + roll_rad. This is
# a SOFTWARE CONVENTION, not something derived from the URDF -- confirm the
# sign feels right on hardware before trusting it for a real build.
WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD = 1.5707963267948966  # pi/2
