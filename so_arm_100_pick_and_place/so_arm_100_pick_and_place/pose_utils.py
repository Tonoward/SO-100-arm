"""Minimal RPY/quaternion math for the stick's collision-box pose, without
pulling in tf_transformations/scipy as a dependency."""
import math
from typing import Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def vec6_to_pos_quat(v) -> Tuple[Vec3, Quat]:
    """v is [x, y, z, roll, pitch, yaw]."""
    return (v[0], v[1], v[2]), rpy_to_quat(v[3], v[4], v[5])
