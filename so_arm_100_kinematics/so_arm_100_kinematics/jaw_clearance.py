"""Jaw-clearance check: can the gripper actually reach into this joint, not
just does the IK have a solution.

``ROS2_IMPLEMENTATION_PLAN.md`` Sec 8.2 consequence 2: the jaws close only
``GRASP_OFFSET_M`` (~51 mm) above a stick's base end -- right where the glue
joint and any already-placed neighbour sticks are. A placement can be
perfectly reachable by :func:`grasp.solve_stick_placement` and still be
physically unbuildable because the jaws cannot fit into the joint. This
module is the "simple box swept along the approach" the doc calls for,
simplified to a capsule (segment + radius) for cheap, dependency-free 3D
distance math -- pure Python, stdlib ``math`` only, like the rest of this
package.

## The model

* **The jaws** are modelled as a single capsule spanning the grip point
  (:func:`grasp.grasp_target`) down to the stick's own base vertex, radius
  ``JAW_RADIUS_M``. This directly encodes Sec 8.2's own numbers: a ~51 mm
  span plus a ~10 mm radius reaches almost exactly the "~60 mm cone around
  the target vertex" the doc describes -- not a coincidence, this *is* that
  cone, made checkable.
* **Each already-placed stick** is modelled as a capsule from its own base
  to its own tip, radius ``STICK_COLLISION_RADIUS_M`` (the square stock's
  circumscribed radius, so a corner-on approach is still caught, plus a
  small inflation -- Sec 10's "consider 1-2 mm inflation" note).
* Clearance is checked with the standard closest-distance-between-two-
  segments computation, minus both radii.

## What this module does NOT decide

* **Which already-placed sticks are the *expected* neighbours at this exact
  joint.** A stick being glued onto an existing joint is *supposed* to sit
  within a few mm of that joint (Sec 8.4's 3.25 mm/end allowance) -- that is
  not a clearance violation, it is the point. Distinguishing "the neighbour
  this joint is glued to" from "some other stick crowding into the same
  space" is a topology question the caller already has the answer to (the
  build order's own shared-ends/parent bookkeeping -- see
  ``BRIDGE_PROTOCOL.md`` Sec A.2). **Pass only genuinely-other, already-
  placed structure in ``placed_sticks`` -- exclude this joint's own
  neighbour(s) before calling, do not rely on this module to guess which
  ones those are.**
* **Self-collision, the mount platform, the table, or MoveIt's own path
  planning.** Like ``envelope.is_reachable``, this is a fast, conservative
  pre-filter -- never a final verdict. MoveIt has the last word regardless
  of what this module says.
* **Neither ``JAW_RADIUS_M`` nor the capsule-vs-box simplification is
  measured or validated against real jaw geometry** -- see the caveat on
  ``JAW_RADIUS_M`` in ``constants.py``. Treat a "clear" result as
  provisional until Phase 0 confirms the real jaw dimensions.
"""

import math

from .constants import GRASP_OFFSET_M, JAW_RADIUS_M, STICK_COLLISION_RADIUS_M
from .grasp import grasp_target


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def segment_distance(p1, q1, p2, q2):
    """Closest distance between segment ``p1``-``q1`` and segment
    ``p2``-``q2`` in 3D. Handles degenerate (zero-length, i.e. point)
    segments and parallel/skew segments alike -- the standard clamped
    closest-point algorithm (Ericson, *Real-Time Collision Detection*
    Sec 5.1.9), reimplemented here to keep this package dependency-free.
    """
    d1 = _v_sub(q1, p1)
    d2 = _v_sub(q2, p2)
    r = _v_sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    eps = 1e-12

    if a <= eps and e <= eps:
        s, t = 0.0, 0.0
    elif a <= eps:
        s = 0.0
        t = _clamp(f / e, 0.0, 1.0)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t = 0.0
            s = _clamp(-c / a, 0.0, 1.0)
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = _clamp((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = _clamp((b - c) / a, 0.0, 1.0)

    c1 = _v_add(p1, _v_scale(d1, s))
    c2 = _v_add(p2, _v_scale(d2, t))
    return math.sqrt(_dot(_v_sub(c1, c2), _v_sub(c1, c2)))


def jaw_swept_capsule(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M):
    """The ``(grip_xyz_m, vertex_xyz_m)`` capsule endpoints for placing a
    stick with these physical ends -- the segment the jaws occupy, from the
    grip point down to the base vertex being glued. Radius is
    ``JAW_RADIUS_M``, applied by :func:`check_jaw_clearance`, not here."""
    return grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m), tuple(base_xyz_m)


def check_jaw_clearance(base_xyz_m, tip_xyz_m, placed_sticks,
                         grasp_offset_m=GRASP_OFFSET_M,
                         jaw_radius_m=JAW_RADIUS_M,
                         stick_collision_radius_m=STICK_COLLISION_RADIUS_M):
    """Can the jaws reach this placement's joint without clipping
    already-placed structure?

    ``placed_sticks`` -- an iterable of ``(base_xyz_m, tip_xyz_m)`` pairs for
    already-placed sticks, in ``base_link``. **Exclude the stick(s) this
    joint is actually gluing onto** -- see the module docstring's "What this
    does NOT decide".

    Returns ``(clear: bool, reason: str or None, min_clearance_m: float or
    None, tightest_index: int or None)``. ``min_clearance_m`` is the
    smallest capsule-to-capsule clearance found (negative if overlapping),
    against ``placed_sticks[tightest_index]`` -- reported even when
    ``clear`` is True, so a caller can flag a placement that is clear but
    tight. ``(True, None, None, None)`` if ``placed_sticks`` is empty.
    """
    jaw_grip, jaw_vertex = jaw_swept_capsule(base_xyz_m, tip_xyz_m, grasp_offset_m)

    tightest_index = None
    tightest_clearance = None
    for index, (p_base, p_tip) in enumerate(placed_sticks):
        dist = segment_distance(jaw_grip, jaw_vertex, p_base, p_tip)
        clearance = dist - jaw_radius_m - stick_collision_radius_m
        if tightest_clearance is None or clearance < tightest_clearance:
            tightest_clearance = clearance
            tightest_index = index

    if tightest_index is None:
        return True, None, None, None

    if tightest_clearance < 0.0:
        reason = (
            "jaw capsule for base=%r tip=%r overlaps placed_sticks[%d] by "
            "%.1f mm" % (base_xyz_m, tip_xyz_m, tightest_index, -tightest_clearance * 1000.0)
        )
        return False, reason, tightest_clearance, tightest_index

    return True, None, tightest_clearance, tightest_index
