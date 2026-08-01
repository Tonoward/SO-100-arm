"""Collision-object lifecycle for the fed stick -- the one persistent scene
object so_arm_100_pick_and_place manages today.

Extracted from pick_and_place_node.py (ROS2_IMPLEMENTATION_PLAN.md Sec 11
Phase 2), no behaviour change. Deliberately kept to plain functions over an
`arm` (a `motion.MotionController.arm`, i.e. a `pymoveit2.MoveIt2`) rather
than hardcoding "the stick" as singular state, so Sec 10's future extension
-- one persistent collision box per PLACED stick, not just the one still in
the feeder -- is additive later, not a rewrite.
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


def re_add_stick(arm, size, position, quat_xyzw, frame_id):
    add_stick_at_feeder(arm, size, position, quat_xyzw, frame_id)


def attach_stick(arm, logger=None):
    if logger is not None:
        logger.info("Attaching stick to the end effector.")
    arm.attach_collision_object(id=STICK_ID, link_name=GRIPPER_LINK_NAME, touch_links=GRIPPER_TOUCH_LINKS)


def detach_stick(arm, logger=None):
    if logger is not None:
        logger.info("Detaching stick.")
    arm.detach_collision_object(id=STICK_ID)
