from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Kept separate from demo.launch.py so the pick-and-place project (the
    # Mount_Platform + stick collision object, the scripted grasp/transport/
    # place sequence, eventually a perception node) can be layered onto this
    # file without risking the generic MoveIt demo used for everything else.
    # Currently identical to demo.launch.py: same real-hardware controllers,
    # same RViz. Add pick-and-place-specific nodes (e.g. a node that seeds the
    # planning scene with the stick's collision object at startup) below.
    moveit_config = MoveItConfigsBuilder("so_arm_100", package_name="so_arm_100_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
