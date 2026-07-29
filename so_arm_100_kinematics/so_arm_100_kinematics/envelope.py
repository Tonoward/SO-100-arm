"""Reachability queries built on chain.ik().

This only checks the closed-form IK's own success/failure (target within
reach, tool-elevation-consistent, within joint limits). It does NOT model
self-collision, the mount platform, the table, or already-placed sticks --
those are checked separately once a real planning scene exists (see
ROS2_IMPLEMENTATION_PLAN.md Sec 10 and BLENDER_ADDON_PLAN.md Sec 7). Treat
`is_reachable` as a fast, conservative pre-filter, not a final verdict.
"""

from .chain import Unreachable, ik


def is_reachable(target_xyz_m, tool_elevation_target_rad=0.0, stick_roll_rad=0.0):
    """Returns (reachable: bool, reason: str or None)."""
    for elbow_up in (True, False):
        try:
            ik(target_xyz_m, tool_elevation_target_rad, stick_roll_rad, elbow_up=elbow_up)
            return True, None
        except Unreachable as exc:
            last_reason = str(exc)
    return False, last_reason


def sweep_envelope(z_values_m, tool_elevation_target_rad=0.0, radius_step_m=0.005,
                    max_radius_m=0.6):
    """For each height in z_values_m, find the min/max horizontal radius (in
    base_link, from the base's own vertical axis) reachable with the given
    tool elevation. Mirrors the ad-hoc sweep used to size the build volume
    in ROS2_IMPLEMENTATION_PLAN.md Sec 9.4/9.5 -- use this instead of
    re-deriving it by hand.

    Returns {z_m: (min_radius_m, max_radius_m) or None if unreachable at
    every radius}.
    """
    result = {}
    for z in z_values_m:
        reachable_radii = []
        r = 0.0
        while r <= max_radius_m:
            ok, _reason = is_reachable((r, 0.0, z), tool_elevation_target_rad)
            if ok:
                reachable_radii.append(r)
            r += radius_step_m
        if reachable_radii:
            result[z] = (min(reachable_radii), max(reachable_radii))
        else:
            result[z] = None
    return result
