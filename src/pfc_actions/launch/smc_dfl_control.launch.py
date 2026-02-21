import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    pfc_params = os.path.join(pfc_actions_dir, 'config', 'params.yaml')

    return LaunchDescription([
        # Nodo SMC+DFL
        Node(
            package='pfc_actions',
            executable='smc_dfl_action',
            name='smc_dfl_action_server',
            parameters=[pfc_params],
            output='screen'
        ),
        # Nodo Health centralizado
        Node(
            package='pfc_actions',
            executable='health_node',
            name='health_node',
            parameters=[pfc_params],
            output='screen'
        )
    ])