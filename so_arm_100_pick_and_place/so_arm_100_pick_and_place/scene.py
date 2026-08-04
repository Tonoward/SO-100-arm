"""Collision-object lifecycle: the transient fed stick (`STICK_ID`,
Sec 11 Phase 2) plus, as of Phase 4, permanent per-stick boxes for the
sculpture as it's built (`register_placed_stick`, Sec 10).

Deliberately kept to plain functions over an `arm` (a
`motion.MotionController.arm`, i.e. a `pymoveit2.MoveIt2`) rather than
hardcoding "the stick" as singular state -- this is exactly why
`register_placed_stick` was additive here, not a rewrite.
"""

STICK_ID = "stick"
GRIPPER_LINK_NAME = "Fixed_Gripper"
GRIPPER_TOUCH_LINKS = ["Fixed_Gripper", "Moving_Jaw", "End_Effector"]


def add_stick_at_feeder(arm, size, position, quat_xyzw, frame_id):
    arm.add_collision_box(id=STICK_ID, size=size, position=position, quat_xyzw=quat_xyzw, frame_id=frame_id)


def remove_stick(arm):
    # Removing the object (a plain topic publish) sidesteps a real hang
    # found with MoveIt2.allow_collisions()'s synchronous Client.call() --
    # see the grasp step in sequences.py for the full story.
    arm.remove_collision_object(id=STICK_ID)


def register_placed_stick(arm, stick_id, size, position, quat_xyzw, frame_id):
    """Add a PERMANENT collision box for a stick that has just been
    released (Sec 10 / Sec 11 Phase 4's ReleaseStick): the sculpture is an
    obstacle field the arm must plan around from here on, so this is never
    removed. `id` is `placed_<stick_id>`, distinct from the transient
    feeder-stick `STICK_ID` -- each call adds one more permanent box, it
    never overwrites a previous one."""
    arm.add_collision_box(
        id=f"placed_{stick_id}", size=size, position=position, quat_xyzw=quat_xyzw, frame_id=frame_id)


def attach_stick(arm, logger=None):
    if logger is not None:
        logger.info("Attaching stick to the end effector.")
    arm.attach_collision_object(id=STICK_ID, link_name=GRIPPER_LINK_NAME, touch_links=GRIPPER_TOUCH_LINKS)


def detach_stick(arm, logger=None):
    if logger is not None:
        logger.info("Detaching stick.")
    arm.detach_collision_object(id=STICK_ID)
