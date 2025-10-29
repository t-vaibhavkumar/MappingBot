import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    pkg_my_bot = get_package_share_directory(package_name)

    # ---- Robot State Publisher (URDF) ----
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','robot.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # rsp = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': open(os.path.join(pkg_my_bot, 'description', 'robot.urdf.xacro')).read(),
    #         'use_sim_time': False
    #     }]
    # )

    # ---- Your Python bridge node (Arduino communication) ----
    robot_bridge = Node(
        package='my_bot',
        executable='motor_encoder_lidar.py',  # make sure this is executable
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
        rsp,
        robot_bridge,
        rviz_node
    ])
