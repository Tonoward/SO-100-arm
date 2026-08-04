import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('so_arm_100_pick_and_place'),
        'config',
        'pick_and_place.yaml',
    )

    # Run this alongside so_arm_100_moveit_config's pickandplace_demo.launch.py
    # (which must already be up: move_group + controllers + RViz). This node
    # only talks to MoveIt -- it does not bring up the robot itself.
    #
    # config/pick_and_place.yaml is scoped `/**:` (wildcard node name)
    # specifically so it applies to this node's actual name
    # (`stick_task_server`), not just `pick_and_place_node` -- see Sec 4 D9
    # in ROS2_IMPLEMENTATION_PLAN.md for why that distinction matters (a
    # `ros2 run` with no --params-file, or a params-file scoped to the
    # wrong node name, silently falls back to code defaults instead).
    #
    # No `interactive` launch arg here, unlike pick_and_place.launch.py --
    # the task server is always non-interactive (Sec 6: action boundaries
    # are the gate, not a terminal prompt).
    #
    # Deliberately no `name=` here (found 2026-08-02 testing Phase 5): it
    # applies a GLOBAL `-r __node:=<name>` remap to the whole process, not
    # just to whichever rclpy Node happens to match -- since
    # stick_task_server_node.py's own main() creates TWO Nodes in one
    # process (`stick_task_server_motion` for pymoveit2,
    # `stick_task_server` for the ActionServers -- see that module's own
    # docstring for why), setting name= here collapsed BOTH onto the same
    # displayed name ("Publisher already registered for provided node
    # name"). The script already names both nodes itself; nothing here
    # needs to override that, matching pick_and_place.launch.py's own Node
    # action, which has never set name= either.
    stick_task_server_node = Node(
        package='so_arm_100_pick_and_place',
        executable='stick_task_server',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )

    return LaunchDescription([stick_task_server_node])
