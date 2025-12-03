import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'gps_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'gps_config.rviz')

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='magnetometer_node',
            name='magnetometer_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='visualizer_node.py',
            name='visualizer_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='gps_path_node.py',
            name='gps_path_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
