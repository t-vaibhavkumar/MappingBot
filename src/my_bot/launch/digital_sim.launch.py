import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    pkg_my_bot = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Robot State Publisher (URDF) ----
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','robot.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ---- Controller config file ----
    controller_config = os.path.join(pkg_my_bot, 'config', 'diff_drive_controller.yaml')

     # ---- Start controller manager ----
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_my_bot, 'description', 'ros2_control.yaml'),
            controller_config
        ],
        output='screen'
    )

    # ---- Load individual controllers ----
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    # # ---- Your Python bridge node (Arduino communication) ----
    # robot_bridge = Node(
    #     package='my_bot',
    #     executable='motor_encoder_lidar.py',  # make sure this is executable
    #     name='robot_bridge',
    #     output='screen'
    # )
        # ---- Your Python bridge node (Arduino communication) ----
    robot_bridge_motor_encoder = Node(
        package='my_bot',
        executable='motor_encoder.py',  # make sure this is executable
        name='robot_bridge',
        output='screen'
    )


    # ---- Optional: RViz node ----
    rviz_config_file = os.path.join(pkg_my_bot, 'config', 'rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        rsp,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        # robot_bridge,
        rviz_node,
        robot_bridge_motor_encoder 
    ])
