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


def feeder_stick_pose(base_xyz_m: Vec3, length_m: float) -> Tuple[Vec3, Quat]:
    """The feeder stick's collision-box CENTER pose, from its physical BASE
    (bottom, where it meets the feeder hole -- same convention as
    StickSpec.base) and its length, so the box's bottom stays pinned at the
    real feeder hole no matter what length the current stick is. Always
    vertical (identity orientation) -- matches the physical feeder, which
    doesn't tilt."""
    x, y, z = base_xyz_m
    return (x, y, z + length_m / 2.0), (0.0, 0.0, 0.0, 1.0)


def axis_aligned_box_pose(base_xyz_m: Vec3, tip_xyz_m: Vec3) -> Tuple[Vec3, Quat]:
    """A placed stick's collision-box pose from its physical BASE/TIP
    (Sec 11 Phase 5's resume path -- Sec A.5: rebuild the scene from the
    build file's own geometry, there's no live FK for a stick placed in a
    PREVIOUS run). Center at the midpoint; orientation is the shortest
    rotation taking the box's local Z axis onto the base->tip direction, so
    an `add_collision_box` sized [section_x, section_y, length] sits flush
    along the stick regardless of which way it points. `roll_deg` is
    deliberately not applied -- Sec 10 already treats these boxes as a
    conservative approximation of a square stick, and every other caller of
    `register_placed_stick` (the live pick/place/release path) uses the
    gripper's actual FK orientation instead, which doesn't carry roll_deg
    explicitly either.
    """
    dx = tip_xyz_m[0] - base_xyz_m[0]
    dy = tip_xyz_m[1] - base_xyz_m[1]
    dz = tip_xyz_m[2] - base_xyz_m[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-9:
        raise ValueError("axis_aligned_box_pose: base and tip coincide, no direction to align to")
    bx, by, bz = dx / length, dy / length, dz / length  # unit base->tip direction

    center = (
        (base_xyz_m[0] + tip_xyz_m[0]) / 2.0,
        (base_xyz_m[1] + tip_xyz_m[1]) / 2.0,
        (base_xyz_m[2] + tip_xyz_m[2]) / 2.0,
    )

    # Shortest-arc quaternion rotating world +Z onto (bx, by, bz).
    dot = bz  # (0, 0, 1) . (bx, by, bz)
    if dot > 1.0 - 1e-9:
        quat = (0.0, 0.0, 0.0, 1.0)
    elif dot < -1.0 + 1e-9:
        # 180 degrees -- +Z and the target are exactly opposite, so
        # cross((0,0,1), target) is zero and the general formula below is
        # undefined. A 180-degree rotation about world +X always takes
        # +Z to -Z, and +X is perpendicular to +Z by construction, so it's
        # a valid (if arbitrary about the stick's own axis) choice here.
        quat = (1.0, 0.0, 0.0, 0.0)
    else:
        # cross((0, 0, 1), (bx, by, bz)) = (-by, bx, 0)
        cx, cy, cz = -by, bx, 0.0
        s = math.sqrt((1.0 + dot) * 2.0)
        inv_s = 1.0 / s
        quat = (cx * inv_s, cy * inv_s, cz * inv_s, s * 0.5)

    return center, quat
