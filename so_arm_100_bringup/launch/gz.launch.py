import os
from pathlib import Path
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description(context, *args, **kwargs):
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    so_arm_100_moveit_config_path = os.path.join(
        get_package_share_directory('so_arm_100_moveit_config')
    )

    dof = LaunchConfiguration('dof').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    use_topic_hardware_interface = LaunchConfiguration('use_topic_hardware_interface').perform(context)

    xacro_file = os.path.join(
        so_arm_100_description_path,
        'urdf',
        f'so_arm_100_{dof}dof.urdf.xacro',
    )

    ros2_controllers_file = os.path.join(
        so_arm_100_moveit_config_path,
        'config',
        f'controllers_{dof}dof.yaml',
    )

    doc = xacro.process_file(xacro_file, mappings={
        'prefix': prefix,
        'use_sim': 'true',
        'use_topic_hardware_interface': use_topic_hardware_interface,
        'ros2_controllers_file': ros2_controllers_file,
    })

    robot_desc = doc.toprettyxml(indent='  ')

    controller_path = os.path.join(
        so_arm_100_moveit_config_path,
        'config',
        f'controllers_{dof}dof.yaml'
    )
    
    # Convert package:// to model:// for Gazebo
    replace_str = f'package://so_arm_100_description/models/so_arm_100_{dof}dof/meshes'
    with_str = f'model://so_arm_100_{dof}dof/meshes'
    gazebo_robot_desc = robot_desc.replace(replace_str, with_str)

    return {
        'robot_description': ParameterValue(robot_desc, value_type=str),
        'gazebo_description': ParameterValue(gazebo_robot_desc, value_type=str),
        'controller_path': controller_path
    }

def generate_launch_description():
    # Launch Arguments
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    so_arm_100_bringup_path = os.path.join(
        get_package_share_directory('so_arm_100_bringup')
    )

    so_arm_100_moveit_config_path = os.path.join(
        get_package_share_directory('so_arm_100_moveit_config')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(so_arm_100_description_path, 'models'),
            ':' + str(Path(so_arm_100_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'dof',
            default_value='5',
            description='DOF configuration - either 5 or 7'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of joint and link names'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'use_topic_hardware_interface',
            default_value='false',
            description='Use topic based hardware interface instead of gz_ros_control'
        ),
    ])

    def launch_setup(context, *args, **kwargs):
        descriptions = get_robot_description(context)

        robot_desc = descriptions['robot_description']

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc}
            ]
        )

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]),
            launch_arguments=[
                ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
            ],
        )

        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=[
                '-string',
                descriptions['gazebo_description'].value,
                '-name',
                'so_arm_100',
                '-allow_renaming',
                'true',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0',
                '-use_sim',
                'true',
            ],
            output='screen'
        )

        # TODO: use dof launch argument consistently

        # ros2_control configuration (since not using gz_ros2_control)
        ros2_control_file = os.path.join(
            so_arm_100_moveit_config_path,
            'config',
            'controllers_5dof.yaml',
        )

        # controller manager (if not using gz_ros2_control)
        ros2_control_node = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                ros2_control_file,
                {"use_sim_time": True},
            ],
            remappings=[
                ("/robot_description", "/robot_description"),
            ],
        )

        # Controller spawner nodes
        joint_state_broadcaster_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
            output='screen'
        )

        arm_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_controller'],
            output='screen'
        )

        gripper_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'gripper_controller'],
            output='screen'
        )

        # ros_gz_bridge when using gz_ros2_control
        gz_ros2_control_bridge = Node(
            condition=UnlessCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            parameters=[{
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
        )

        # TODO: rename config file
        bridge_config_file = os.path.join(
            so_arm_100_bringup_path,
            'config',
            'ros_gz_bridge.yaml',
        )

        # ros_gz_bridge and relay when using topic based control
        topic_based_control_bridge = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_config_file}],
            output="screen",
        )

        command_relay = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="so_arm_100_bringup",
            executable="so_arm_100_5dof_cmd_relay",
            parameters=[],
            output="screen",
        )

        nodes = [
            robot_state_publisher,
            gazebo,
            spawn_robot,

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[ros2_control_node]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[gripper_controller_spawner]
                )
            ),
            gz_ros2_control_bridge,
            topic_based_control_bridge,
            command_relay,
        ]
        return nodes

    return LaunchDescription([
        arguments,
        gazebo_resource_path,
        OpaqueFunction(function=launch_setup)
    ])
