"""Closed-form FK/IK for the SO-100 5-DOF arm.

Pure Python, stdlib only (``math``), zero ROS imports -- this package is
vendored verbatim into the Blender addon and must run unmodified in
Blender's bundled interpreter. See README.md before copying it anywhere.
"""

# Bump on ANY change to chain.py, constants.py, grasp.py or jaw_clearance.py
# that alters computed results. Written into every build file as
# `kinematics_version` so a Blender-side copy that has drifted from the
# robot-side original is caught loudly at load time rather than silently
# producing wrong placements. Must match the VERSION file at this package's
# root.
#
# 1.2.0 (2026-07-31): added jaw_clearance.py -- the swept-jaw collision
# pre-filter (ROS2_IMPLEMENTATION_PLAN.md Sec 8.2 consequence 2 / Sec 11
# Phase 1). New public surface only; chain.py/constants.py's existing
# fields/grasp.py are untouched, so every pre-1.2.0 computed result is
# identical.
# 1.1.0 (2026-07-30): added grasp.py -- the grasp-orientation transform
# (ROS2_IMPLEMENTATION_PLAN.md Sec 8.2's "transform helper", needed by
# PlaceStick Sec 7.2 steps 1-2). chain.py/constants.py/envelope.py are
# untouched; every existing computed result is identical to 1.0.0.
__version__ = "1.2.0"

from .chain import Unreachable, fk, ik, tool_axis, tool_elevation_rad
from .constants import (
    BUILD_VOLUME_MAX_M,
    BUILD_VOLUME_MIN_M,
    GRASP_OFFSET_M,
    JAW_RADIUS_M,
    JOINT_ALLOWANCE_M,
    JOINT_NAMES,
    STICK_COLLISION_RADIUS_M,
    STICK_LENGTH_RANGE_M,
    STICK_SECTION_M,
)
from .envelope import is_reachable, sweep_envelope
from .grasp import (
    ANCHOR_TOLERANCE_DEG,
    CARDINAL_ANCHORS_RAD,
    VERIFY_TOLERANCE_DEG,
    azimuth_frame,
    grasp_target,
    solve_stick_orientation,
    solve_stick_placement,
    stick_axis,
)
from .jaw_clearance import check_jaw_clearance, jaw_swept_capsule, segment_distance

__all__ = [
    "__version__",
    "fk",
    "ik",
    "Unreachable",
    "tool_axis",
    "tool_elevation_rad",
    "is_reachable",
    "sweep_envelope",
    "JOINT_NAMES",
    "GRASP_OFFSET_M",
    "STICK_SECTION_M",
    "STICK_LENGTH_RANGE_M",
    "JOINT_ALLOWANCE_M",
    "BUILD_VOLUME_MIN_M",
    "BUILD_VOLUME_MAX_M",
    "grasp_target",
    "azimuth_frame",
    "stick_axis",
    "solve_stick_orientation",
    "solve_stick_placement",
    "VERIFY_TOLERANCE_DEG",
    "ANCHOR_TOLERANCE_DEG",
    "CARDINAL_ANCHORS_RAD",
    "check_jaw_clearance",
    "jaw_swept_capsule",
    "segment_distance",
    "JAW_RADIUS_M",
    "STICK_COLLISION_RADIUS_M",
]
