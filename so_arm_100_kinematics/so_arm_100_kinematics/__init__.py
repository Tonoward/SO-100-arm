"""Closed-form FK/IK for the SO-100 5-DOF arm.

Pure Python, stdlib only (``math``), zero ROS imports -- this package is
vendored verbatim into the Blender addon and must run unmodified in
Blender's bundled interpreter. See README.md before copying it anywhere.
"""

# Bump on ANY change to chain.py or constants.py that alters computed
# results. Written into every build file as `kinematics_version` so a
# Blender-side copy that has drifted from the robot-side original is caught
# loudly at load time rather than silently producing wrong placements.
# Must match the VERSION file at this package's root.
__version__ = "1.0.0"

from .chain import Unreachable, fk, ik, tool_axis, tool_elevation_rad
from .constants import (
    BUILD_VOLUME_MAX_M,
    BUILD_VOLUME_MIN_M,
    GRASP_OFFSET_M,
    JOINT_ALLOWANCE_M,
    JOINT_NAMES,
    STICK_LENGTH_RANGE_M,
    STICK_SECTION_M,
)
from .envelope import is_reachable, sweep_envelope

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
]
