"""The grasp-orientation transform: turn a stick's physical placement into
IK-consumable parameters.

Pure Python (stdlib ``math`` only, no numpy, zero ROS imports), like the
rest of this package -- see the package docstring. This module is what
``ROS2_IMPLEMENTATION_PLAN.md`` Sec 8.2 calls "the transform helper" and
Sec 7.2's ``PlaceStick`` needs for its steps 1-2 ("Compute the TCP goal pose
from the stick spec + grasp offset. Solve IK -> joint target."). It is the
single implementation both the ROS2 task server and the Blender addon's
offline validation call -- there must be exactly one, or the two sides can
silently disagree about where a stick goes, which is the entire failure the
vendoring design in this package's README exists to prevent.

Two facts this builds on, already stated and tested by ``chain.py``, not
anything new about the physical arm:

* The ``Shoulder_Pitch``/``Elbow``/``Wrist_Pitch`` sub-chain is exactly
  planar (the tool axis always lies in the vertical plane fixed by
  ``Shoulder_Rotation``'s azimuth).
* The stick is held perpendicular to the tool axis, and ``Wrist_Roll``
  rotates it about that axis without moving the TCP.

Everything below reduces to: pick a tool elevation whose axis is
perpendicular to the desired stick direction, then pick the roll that spins
the stick the rest of the way there -- and never trust either choice without
measuring it back through ``fk()``.

## Three non-obvious findings, each confirmed empirically against
## ``fk()``/``ik()``, not assumed

1. **The reference stick direction points from the grip toward the stick's
   BASE, not its tip.** ``rot(joints) @ (0, 0, 1)`` (the tool frame's local Z
   column, exposed here as :func:`stick_axis`) is a valid "which way does the
   stick point" vector -- confirmed by matching all five tuned poses' known-
   vertical stick, all of which come out as *negative* Z. This makes physical
   sense once you notice the grip point sits *above* the stick's base
   (``GRASP_OFFSET_M`` up from it): the direction from grip to base is
   downward. Get this sign wrong and every roll solve is silently off by
   ~180 deg, which reads as "unreachable" for placements that are actually
   fine.
2. **The elevation equation has two roots 180 deg apart**, and only one of
   them puts the roll=0 reference direction anywhere near the target (the
   other needs ~180 deg of roll to compensate, which is usually unreachable).
   There is no way to know which root is right without trying both, and
   separately, the equation is numerically ill-conditioned right at the
   documented-easy directions (vertical / horizontal-radial / horizontal-
   tangential): a fraction of a degree of input noise can swing the
   atan2-derived root by ~90 deg onto a branch that is reachable in exact
   math but not in practice. So :func:`solve_stick_orientation` tries both
   exact roots *and* the four cardinal poses (0, +-90, 180 deg -- the
   directly documented ones) as a second tier, and **verifies every
   candidate by feeding the solved joints back through** ``fk()`` **and
   checking the achieved direction**, rather than trusting any formula on
   its own. A candidate is only ever accepted once measured.
3. **``stick_roll_rad`` rotates the stick CLOCKWISE around** :func:`chain.tool_axis`,
   opposite the standard right-hand convention a naive cross-product formula
   assumes. Invisible for in-plane targets (there ``sin(roll)`` is always
   ~0, so only the sign-independent magnitude is ever exercised) -- it only
   shows up once the target direction has a genuine component outside the
   arm's vertical plane. This is exactly the kind of bug a "looks right"
   formula hides; it was caught by the round-trip verification, not by
   inspection. Do not simplify the sign back to the "obvious" one without
   re-running that verification.

Because the search in point 2 is exhaustive (``Shoulder_Rotation`` itself has
no branch ambiguity, so elevation and elbow are the only remaining degrees of
freedom) and self-verifying, **"no candidate verifies" means the placement is
genuinely unreachable with this arm, not merely unvalidated.**

⚠ A related planning-document claim this module directly disproves:
Sec 9.4's table originally said a stick "tilted out of the arm's plane
(leaning sideways)" is categorically unreachable with 5 DOF. It is not --
:func:`solve_stick_orientation` finds and verifies such placements routinely
(a 2000-sample sweep over fully random 3D directions found hundreds
reachable, every one verified to <0.001 deg). That table row has been
corrected in the plan; this module does not encode the old claim.

## What this module does NOT decide

* **Which end of a stick is "base".** That is a structural question (does
  this end seat on the plate or on an already-placed stick?) decided by the
  caller -- the Blender addon's build-order solver, or ROS2's own planning
  scene bookkeeping. This module only answers "can the arm place a stick
  with base here and tip there", for whichever assignment it is given.
* **Jaw clearance, self-collision, the mount platform, or MoveIt's own path
  planning.** ``is_reachable()``/``solve_stick_orientation()`` are a fast,
  conservative pre-filter on reachability alone -- never a final verdict.
  MoveIt has the last word regardless of what this module says.
"""

import math

from .chain import Unreachable, fk, ik, tool_axis
from .constants import CHAIN, GRASP_OFFSET_M

# Shoulder_Rotation's own origin -- every azimuth in this module is measured
# from here, matching chain.ik()'s own convention.
_SHOULDER_AXIS_POINT = CHAIN[0][1]

# How far a candidate's achieved direction may miss the target before it is
# rejected rather than accepted, for the two EXACT roots of the
# perpendicularity equation. Tight: this is a numerical round-trip check on
# a closed-form solution, not a manufacturing tolerance.
VERIFY_TOLERANCE_DEG = 0.05

# The four cardinal poses (Finding 2): vertical, the two horizontal-radial
# directions, and horizontal-tangential. Sec 9.4's own directly-documented,
# easy cases -- and, not coincidentally, exactly where the exact-root
# equation above is numerically ill-conditioned.
CARDINAL_ANCHORS_RAD = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)

# Tolerance for accepting a cardinal anchor -- looser than the exact-root
# tolerance, but grounded in the joint's own physical slack rather than
# picked to make a specific case pass: the glue joint's own gap
# (JOINT_ALLOWANCE_M, 3.25 mm one end / 6.5 mm both) already absorbs some
# placement imprecision, and over a ~110 mm stick that gap alone corresponds
# to roughly asin(3.25/110)..asin(6.5/110) = 1.7..3.4 deg of angular slack
# before the joint's own designed-in gap is exceeded. 5 deg sits comfortably
# above that (a real case needed 2.08 deg) while staying far below where an
# actual wrong-branch bug shows up (every one found while building this
# module was off by 15-180 deg, never single digits) -- see
# test/test_grasp.py's TestRealWorldCases for the case that set this number.
ANCHOR_TOLERANCE_DEG = 5.0


def grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M):
    """TCP target for gripping/placing a stick between these physical ends:
    the point ``grasp_offset_m`` from the base end, along the stick's own
    axis toward the tip.

    ROS2_IMPLEMENTATION_PLAN.md Sec 8.2's ``T_target_tcp`` formula. This is
    the position half of the transform -- unaffected by the orientation-sign
    finding above: the grip point is offset from base *toward* tip,
    regardless of which direction counts as the roll=0 reference.

    ⚠ ``grasp_offset_m`` (default ``GRASP_OFFSET_M``) is derived from FK, not
    measured with a ruler -- see ``constants.py``'s own caveat. Confirm with
    Phase 0 before trusting this for a real build.
    """
    dx = tip_xyz_m[0] - base_xyz_m[0]
    dy = tip_xyz_m[1] - base_xyz_m[1]
    dz = tip_xyz_m[2] - base_xyz_m[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-12:
        raise ValueError("base and tip coincide -- the stick has zero length")
    return (
        base_xyz_m[0] + dx / length * grasp_offset_m,
        base_xyz_m[1] + dy / length * grasp_offset_m,
        base_xyz_m[2] + dz / length * grasp_offset_m,
    )


def azimuth_frame(target_xyz_m):
    """``(r_hat, z_hat, tan_hat)`` at ``target_xyz_m``'s azimuth from
    ``Shoulder_Rotation``'s own axis: radial (horizontal, toward the
    target), vertical, and tangential (horizontal, perpendicular to
    ``r_hat`` -- the "out of the arm's plane" direction).

    Returns ``None`` if the target sits exactly on the rotation axis, where
    azimuth is undefined (matches ``chain.ik()``'s own "azimuth undefined"
    failure for the same input).
    """
    dx = target_xyz_m[0] - _SHOULDER_AXIS_POINT[0]
    dy = target_xyz_m[1] - _SHOULDER_AXIS_POINT[1]
    rho = math.hypot(dx, dy)
    if rho < 1e-9:
        return None
    r_hat = (dx / rho, dy / rho, 0.0)
    tan_hat = (-r_hat[1], r_hat[0], 0.0)
    return r_hat, (0.0, 0.0, 1.0), tan_hat


def stick_axis(joint_angles_rad):
    """The physical stick's direction for this joint solution, pointing
    from the grip point toward the stick's BASE (Finding 1) -- the tool
    frame's local Z column, ``rot(joint_angles_rad) @ (0, 0, 1)``.

    Companion to :func:`chain.tool_axis`, which gives the direction the
    *tool* points; this gives the direction the *stick* points.
    """
    _pos, rot = fk(joint_angles_rad)
    return (rot[0][2], rot[1][2], rot[2][2])


def _wrap_angle(angle_rad):
    """Wrap to ``(-pi, pi]``. ``chain.ik()`` does not wrap its own joint
    solutions before comparing them against joint limits (it never needs
    to, since its own callers always pass a small, already-sane elevation)
    -- so ``elevation0 + pi`` landing near ``+2*pi`` instead of near 0 makes
    a perfectly fine configuration look like it exceeds a limit it does not
    (e.g. a joint solution of "-424.84 deg" that is really -64.84 deg once
    wrapped). Always pass ``ik()`` a canonical-range elevation."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _reference_elevation_rad(grip_to_base_direction, r_hat):
    """The tool elevation whose axis is perpendicular to the IN-PLANE
    component of ``grip_to_base_direction`` -- the (generally unique, up to
    the +pi branch) exact solution of the perpendicularity equation. Grounded
    directly in the two facts the module docstring states: the tool axis is
    confined to the vertical plane, and the stick is held perpendicular to
    it -- so this depends only on the direction's radial and vertical
    components, never its tangential one.
    """
    a_r = (grip_to_base_direction[0] * r_hat[0]
          + grip_to_base_direction[1] * r_hat[1])
    a_z = grip_to_base_direction[2]
    if abs(a_r) < 1e-12 and abs(a_z) < 1e-12:
        return 0.0  # purely tangential: every elevation satisfies this
    return math.atan2(-a_r, a_z)


def _try_elevation(target_xyz_m, elevation, grip_dir, tolerance_deg):
    """One (elevation, both elbow branches) probe. Returns the best
    ``(elevation, roll, elbow_up, error_deg)`` of the branches that verify
    within ``tolerance_deg``, or ``None``."""
    best = None
    for elbow_up in (True, False):
        try:
            q_ref = ik(target_xyz_m, elevation, stick_roll_rad=0.0, elbow_up=elbow_up)
        except Unreachable:
            continue
        ref_axis = stick_axis(q_ref)
        tool = tool_axis(q_ref)
        cross = (
            ref_axis[1] * grip_dir[2] - ref_axis[2] * grip_dir[1],
            ref_axis[2] * grip_dir[0] - ref_axis[0] * grip_dir[2],
            ref_axis[0] * grip_dir[1] - ref_axis[1] * grip_dir[0],
        )
        cos_roll = (ref_axis[0] * grip_dir[0] + ref_axis[1] * grip_dir[1]
                   + ref_axis[2] * grip_dir[2])
        sin_roll = cross[0] * tool[0] + cross[1] * tool[1] + cross[2] * tool[2]
        # Finding 3: negated -- see the module docstring, and do not
        # simplify this back to +tool without re-running the verification
        # a few lines below.
        roll = math.atan2(-sin_roll, cos_roll)
        try:
            q = ik(target_xyz_m, elevation, stick_roll_rad=roll, elbow_up=elbow_up)
        except Unreachable:
            continue
        achieved = stick_axis(q)
        cos_err = max(-1.0, min(
            1.0, achieved[0] * grip_dir[0] + achieved[1] * grip_dir[1]
            + achieved[2] * grip_dir[2]))
        error_deg = math.degrees(math.acos(cos_err))
        if error_deg > tolerance_deg:
            continue
        if best is None or error_deg < best[3]:
            best = (elevation, roll, elbow_up, error_deg)
    return best


def solve_stick_orientation(target_xyz_m, base_to_tip_axis,
                            verify_tolerance_deg=VERIFY_TOLERANCE_DEG,
                            anchor_tolerance_deg=ANCHOR_TOLERANCE_DEG):
    """Exhaustive, self-verifying search for ``(tool_elevation_rad,
    stick_roll_rad, elbow_up, error_deg)`` placing a stick along
    ``base_to_tip_axis`` with its grip point at ``target_xyz_m`` (see
    :func:`grasp_target`). Returns ``None`` if no candidate reproduces the
    target direction within tolerance -- see the module docstring for why
    that means genuinely unreachable, not merely unchecked.

    Tries two families of candidate elevation (Finding 2):

    * The two **exact roots** of the perpendicularity equation -- verified
      tight (``verify_tolerance_deg``). For a well-conditioned target (the
      common case) this is exact, and is preferred whenever it succeeds.
    * The four **cardinal anchors** (0, +-90, 180 deg) -- verified against
      the looser, physically-grounded ``anchor_tolerance_deg``, to absorb
      the numerical ill-conditioning the exact-root equation has right at
      these same directions.

    All verified candidates compete on smallest achieved error first (an
    exact root wins whenever one succeeds), then smallest ``|roll|`` to
    break remaining ties.

    ``base_to_tip_axis`` need not be unit length; it is normalized here.
    """
    basis = azimuth_frame(target_xyz_m)
    if basis is None:
        return None
    r_hat, _z_hat, _tan_hat = basis

    length = math.sqrt(sum(c * c for c in base_to_tip_axis))
    if length < 1e-12:
        return None
    # Finding 1: the reference direction points grip-to-base, not base-tip.
    grip_dir = tuple(-c / length for c in base_to_tip_axis)

    elevation0 = _reference_elevation_rad(grip_dir, r_hat)

    candidates = []
    for elevation in (_wrap_angle(elevation0), _wrap_angle(elevation0 + math.pi)):
        found = _try_elevation(target_xyz_m, elevation, grip_dir, verify_tolerance_deg)
        if found is not None:
            candidates.append(found)
    for elevation in CARDINAL_ANCHORS_RAD:
        found = _try_elevation(target_xyz_m, elevation, grip_dir, anchor_tolerance_deg)
        if found is not None:
            candidates.append(found)

    if not candidates:
        return None
    candidates.sort(key=lambda c: (c[3], abs(c[1])))
    return candidates[0]


def solve_stick_placement(base_xyz_m, tip_xyz_m, grasp_offset_m=GRASP_OFFSET_M,
                          verify_tolerance_deg=VERIFY_TOLERANCE_DEG,
                          anchor_tolerance_deg=ANCHOR_TOLERANCE_DEG):
    """The full solve ``PlaceStick`` needs (ROS2_IMPLEMENTATION_PLAN.md
    Sec 7.2 steps 1-2): given a stick's physical base and tip, return the 5
    joint angles (radians, ``chain.JOINT_NAMES`` order) that grip/place it.

    Raises :class:`chain.Unreachable` if no candidate orientation verifies --
    the placement is genuinely unreachable, not merely unchecked (see the
    module docstring). Never returns a silently-approximate answer: the
    returned joints, fed back through ``fk()``, reproduce the target
    direction to within whichever tolerance (``verify_tolerance_deg`` or
    ``anchor_tolerance_deg``) actually accepted the candidate.
    """
    target = grasp_target(base_xyz_m, tip_xyz_m, grasp_offset_m)
    axis = tuple(t - b for t, b in zip(tip_xyz_m, base_xyz_m))
    solved = solve_stick_orientation(
        target, axis, verify_tolerance_deg, anchor_tolerance_deg)
    if solved is None:
        raise Unreachable(
            "no orientation places a stick from %r to %r within tolerance"
            % (base_xyz_m, tip_xyz_m))
    elevation, roll, elbow_up, _error_deg = solved
    return ik(target, elevation, stick_roll_rad=roll, elbow_up=elbow_up)
