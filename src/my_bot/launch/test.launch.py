import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_bot = FindPackageShare('my_bot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description = Command([
        'xacro ', PathJoinSubstitution([pkg_my_bot, 'description', 'robot.urdf.xacro'])
    ])

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Motor + Encoder bridge
    motor_bridge = Node(
        package='my_bot',
        executable='motor_encoder.py',
        name='motor_encoder_bridge',
        output='screen'
    )

    # ------------------ Joint State Publisher ------------------
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp_node,
        motor_bridge,
        joint_state_publisher
    ])
