from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/vaibhav/maps/my_room_map.yaml'
            }]
        ),

        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_map_topic': True,
                'scan_topic': '/scan',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link'
            },
            '/home/vaibhav/iot_ws/src/my_bot/nav2_ultrasonic/config/nav2_params.yaml']
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/vaibhav/iot_ws/src/my_bot/nav2_ultrasonic/config/nav2_params.yaml']
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/vaibhav/iot_ws/src/my_bot/nav2_ultrasonic/config/nav2_params.yaml']
        ),

        # Behavior Server (used instead of nav2_recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=['/home/vaibhav/iot_ws/src/my_bot/nav2_ultrasonic/config/nav2_params.yaml']
        ),

        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=['/home/vaibhav/iot_ws/src/my_bot/nav2_ultrasonic/config/nav2_params.yaml']
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
