"""Minimal RPY/quaternion math for composing the grasp offset onto the
stick's pose, without pulling in tf_transformations/scipy as a dependency."""
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


def quat_mult(q1: Quat, q2: Quat) -> Quat:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    x, y, z, w = q
    vx, vy, vz = v
    uv = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0])
    return (
        vx + 2.0 * (w * uv[0] + uuv[0]),
        vy + 2.0 * (w * uv[1] + uuv[1]),
        vz + 2.0 * (w * uv[2] + uuv[2]),
    )


def compose(parent_pos: Vec3, parent_quat: Quat, child_pos: Vec3, child_quat: Quat) -> Tuple[Vec3, Quat]:
    """world_T_child = world_T_parent * parent_T_child."""
    rotated = rotate_vector(parent_quat, child_pos)
    result_pos = (
        parent_pos[0] + rotated[0],
        parent_pos[1] + rotated[1],
        parent_pos[2] + rotated[2],
    )
    return result_pos, quat_mult(parent_quat, child_quat)


def vec6_to_pos_quat(v) -> Tuple[Vec3, Quat]:
    """v is [x, y, z, roll, pitch, yaw]."""
    return (v[0], v[1], v[2]), rpy_to_quat(v[3], v[4], v[5])
