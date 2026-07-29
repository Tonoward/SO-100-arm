"""Forward and inverse kinematics for the SO-100 5-DOF arm.

Pure Python (stdlib ``math`` only, no numpy) so this module runs unmodified
in Blender's bundled interpreter -- see the package docstring / README for
why that constraint exists.

Geometry, derived from the URDF
--------------------------------
The chain is ``Shoulder_Rotation`` (base yaw) -> ``Shoulder_Pitch`` ->
``Elbow`` -> ``Wrist_Pitch`` -> ``Wrist_Roll`` -> tool. Every joint origin
*after* Shoulder_Rotation has zero local-X component, and Shoulder_Pitch,
Elbow and Wrist_Pitch all rotate about their local X axis. A rotation about
X never introduces an X component into a vector that has none, so the
**entire Shoulder_Pitch/Elbow/Wrist_Pitch sub-chain is exactly planar**: it
lives in the single vertical plane fixed by ``Shoulder_Rotation``'s azimuth.

``Wrist_Roll`` breaks out of that plane on purpose -- it rotates about the
*tool's own axis* (after a fixed 90 deg reorientation), which is exactly
"spin the stick without moving it": working through the algebra, the
combined ``Wrist_Roll`` translate + ``End_Effector`` offset is a vector
aligned with the roll axis itself, so rotating about that axis leaves the
TCP *position* completely unchanged. Wrist_Roll only ever changes stick
roll, never TCP position -- confirmed by both the derivation and the
round-trip tests in ``test/test_chain.py``.

This lets the whole 5-DOF problem split cleanly:
  1. ``Shoulder_Rotation`` (q0): solved directly from the target's azimuth.
  2. ``Shoulder_Pitch``, ``Elbow`` (q1, q2): a standard 2-link planar solve,
     with the two links' non-radial offset vectors folded into a constant
     phase shift (see ``_ik_planar`` below).
  3. ``Wrist_Pitch`` (q3): whatever is left over to hit the requested tool
     angle, once q1+q2 are fixed.
  4. ``Wrist_Roll`` (q4): set directly from the desired stick roll (a
     software convention -- see constants.WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD).

No iteration, no seeds, no timeouts: every solution (subject to the elbow
up/down branch) is closed form.
"""

import math

from .constants import CHAIN, EE_OFFSET, WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD

# --- minimal 3D/2D vector + 3x3 matrix helpers (no numpy) -------------------


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_mat(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rpy_matrix(r, p, y):
    return _mat_mat(_rot_z(y), _mat_mat(_rot_y(p), _rot_x(r)))


def _axis_angle_matrix(axis, angle):
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    ux, uy, uz = axis[0] / n, axis[1] / n, axis[2] / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return (
        (c + ux * ux * C, ux * uy * C - uz * s, ux * uz * C + uy * s),
        (uy * ux * C + uz * s, c + uy * uy * C, uy * uz * C - ux * s),
        (uz * ux * C - uy * s, uz * uy * C + ux * s, c + uz * uz * C),
    )


def _rotate_2d(v, theta):
    """Rotate a (y, z) pair by theta -- matches Rx(theta) acting on (0, y, z)."""
    c, s = math.cos(theta), math.sin(theta)
    y, z = v
    return (y * c - z * s, y * s + z * c)


# --- forward kinematics ------------------------------------------------------


def fk(joint_angles_rad):
    """Full FK. Returns (position_xyz_m, rotation_matrix) of the tool
    (End_Effector) frame in base_link. rotation_matrix is a 3x3 tuple of
    tuples; columns are the frame's X/Y/Z axes expressed in base_link."""
    pos = (0.0, 0.0, 0.0)
    rot = _IDENTITY
    for (_name, xyz, rpy, axis, _lo, _hi), q in zip(CHAIN, joint_angles_rad):
        pos = _v_add(pos, _mat_vec(rot, xyz))
        rot = _mat_mat(rot, _rpy_matrix(*rpy))
        rot = _mat_mat(rot, _axis_angle_matrix(axis, q))
    pos = _v_add(pos, _mat_vec(rot, EE_OFFSET))
    return pos, rot


def tool_axis(joint_angles_rad):
    """Unit vector the tool points along, in base_link. Matches the
    gripper's own -Y axis (the direction validated against pick_and_place's
    tuned poses)."""
    _pos, rot = fk(joint_angles_rad)
    return _mat_vec(rot, (0.0, -1.0, 0.0))


def tool_elevation_rad(joint_angles_rad):
    """Angle of the tool axis above/below horizontal. 0 => horizontal tool
    (the orientation that holds a stick vertical). Exactly
    -(q1+q2+q3) for this chain -- provided directly rather than recomputed
    from the rotation matrix, since it is what the IK below both consumes
    and guarantees."""
    return -(joint_angles_rad[1] + joint_angles_rad[2] + joint_angles_rad[3])


# --- geometry used by the IK -------------------------------------------------

_SHOULDER_AXIS_POINT = CHAIN[0][1]  # Shoulder_Rotation's own origin xyz, x==0
_A = (CHAIN[1][1][1], CHAIN[1][1][2])  # Shoulder_Pitch offset, (y, z)
_B = (CHAIN[2][1][1], CHAIN[2][1][2])  # Elbow offset, (y, z)
_C = (CHAIN[3][1][1], CHAIN[3][1][2])  # Wrist_Pitch offset, (y, z)
# Combined Wrist_Roll translate + End_Effector offset -- both pure-Y vectors
# in the frame Wrist_Roll's rotation is about, so they add directly (see
# module docstring: Wrist_Roll's own rotation does not move this vector).
_D = (CHAIN[4][1][1] + EE_OFFSET[1], 0.0)

_L1 = math.hypot(*_B)
_L2 = math.hypot(*_C)
_PHI_B = math.atan2(_B[1], _B[0])
_PHI_C = math.atan2(_C[1], _C[0])
_DELTA = _PHI_C - _PHI_B


class Unreachable(Exception):
    """Raised by ik() when no joint solution exists for the target."""


def ik(target_xyz_m, tool_elevation_target_rad=0.0, stick_roll_rad=0.0, elbow_up=True):
    """Closed-form IK. target_xyz_m is the desired TCP position in
    base_link. tool_elevation_target_rad=0 places a vertical stick (the
    common case); see tool_elevation_rad() for the convention.

    Returns the 5 joint angles (radians, CHAIN order) on success, or raises
    Unreachable with a specific reason (never returns a silently-wrong
    answer).
    """
    dx = target_xyz_m[0] - _SHOULDER_AXIS_POINT[0]
    dy = target_xyz_m[1] - _SHOULDER_AXIS_POINT[1]
    dz = target_xyz_m[2] - _SHOULDER_AXIS_POINT[2]

    rho = math.hypot(dx, dy)
    if rho < 1e-9:
        raise Unreachable("target lies on the Shoulder_Rotation axis -- azimuth undefined")
    phi = math.atan2(dy, dx)
    q0 = math.atan2(-math.cos(phi), -math.sin(phi))

    # Entire remaining sub-chain is planar in (VY, VZ) = (height, reach)
    # relative to the shoulder axis point, per the module docstring.
    v_target = (dz, rho)

    alpha3 = -tool_elevation_target_rad - math.pi / 2.0
    d_rotated = _rotate_2d(_D, alpha3)
    wrist_target = (v_target[0] - d_rotated[0], v_target[1] - d_rotated[1])

    g = (wrist_target[0] - _A[0], wrist_target[1] - _A[1])
    r = math.hypot(*g)

    cos_delta_angle = (r * r - _L1 * _L1 - _L2 * _L2) / (2.0 * _L1 * _L2)
    if cos_delta_angle < -1.0 - 1e-9 or cos_delta_angle > 1.0 + 1e-9:
        raise Unreachable(
            f"target {target_xyz_m} at reach {r:.4f} m is outside the "
            f"Shoulder_Pitch/Elbow/Wrist_Pitch sub-chain's envelope "
            f"({abs(_L1 - _L2):.4f}..{_L1 + _L2:.4f} m)"
        )
    cos_delta_angle = max(-1.0, min(1.0, cos_delta_angle))
    delta_angle = math.acos(cos_delta_angle)
    if not elbow_up:
        delta_angle = -delta_angle

    beta1 = math.atan2(g[1], g[0]) - math.atan2(
        _L2 * math.sin(delta_angle), _L1 + _L2 * math.cos(delta_angle)
    )

    q1 = beta1 - _PHI_B
    q2 = delta_angle - _DELTA
    alpha2 = q1 + q2
    # alpha3 = alpha2 - pi/2 + q3 (Wrist_Pitch's own rpy contributes a fixed
    # -pi/2 before its variable rotation -- see CHAIN's Wrist_Pitch entry).
    q3 = alpha3 - alpha2 + math.pi / 2.0
    q4 = WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD + stick_roll_rad

    joints = (q0, q1, q2, q3, q4)
    # Epsilon set to ~0.06 deg, not float-noise-tight: the 'home' pose sits
    # essentially exactly on Shoulder_Pitch's mechanical limit, and the
    # round-trip through a 2-decimal-degree yaml value accumulates noise at
    # that scale. Still far tighter than anything mechanically meaningful.
    limit_eps = 1e-3
    for (name, _xyz, _rpy, _axis, lo, hi), q in zip(CHAIN, joints):
        if not (lo - limit_eps <= q <= hi + limit_eps):
            raise Unreachable(f"{name} solution {math.degrees(q):.2f} deg exceeds limits")
    return joints
