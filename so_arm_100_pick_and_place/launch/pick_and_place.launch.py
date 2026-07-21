import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('so_arm_100_pick_and_place'),
        'config',
        'pick_and_place.yaml',
    )

    interactive_arg = DeclareLaunchArgument(
        'interactive',
        default_value='true',
        description=(
            'Preview each planned step in RViz and prompt before executing '
            'it. Set to false for an unattended run.'
        ),
    )

    # Run this alongside so_arm_100_moveit_config's pickandplace_demo.launch.py
    # (which must already be up: move_group + controllers + RViz). This node
    # only talks to MoveIt -- it does not bring up the robot itself.
    #
    # If interactive mode's Enter-key prompts don't seem to reach this
    # process, run it with `ros2 run so_arm_100_pick_and_place
    # pick_and_place_node --ros-args --params-file <config_file>` instead --
    # `ros2 launch`'s stdin passthrough can be unreliable once more than one
    # node is in the launch description.
    pick_and_place_node = Node(
        package='so_arm_100_pick_and_place',
        executable='pick_and_place_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file, {'interactive': LaunchConfiguration('interactive')}],
    )

    return LaunchDescription([interactive_arg, pick_and_place_node])
