import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    # Usamos el archivo de parámetros específico para simulación
    sim_params = os.path.join(pfc_actions_dir, 'config', 'sim_params.yaml')

    # Remapeos comunes para que el código hable con el TurtleBot3
    # Mapeamos '/odom' (TB3) a '/odometry/filtered' (lo que espera tu código)
    common_remappings = [
        ('/odometry/filtered', '/odom'),
        ('/cmd_vel', '/cmd_vel')
    ]

    return LaunchDescription([
        # 1. Servidor de Acción SMC+DFL
        Node(
            package='pfc_actions',
            executable='smc_dfl_action',
            name='smc_dfl_action_server',
            parameters=[sim_params],
            remappings=common_remappings,
            output='screen'
        ),

        # 2. Vigilante de Salud (Modo Bypass Simulación)
        Node(
            package='pfc_actions',
            executable='health_node',
            name='health_node',
            parameters=[sim_params],
            remappings=common_remappings,
            output='screen'
        ),

        # 3. Director: Behavior Tree Manager
        Node(
            package='pfc_actions',
            executable='bt_manager',
            name='bt_manager_node',
            parameters=[sim_params],
            output='screen'
        )
    ])