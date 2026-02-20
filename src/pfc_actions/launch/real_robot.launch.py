import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obtener las rutas de los paquetes
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    control_pkg_dir = get_package_share_directory('control_pkg')
    
    # 2. Definir las rutas de los archivos de configuración
    params_file = os.path.join(pfc_actions_dir, 'config', 'params.yaml')
    # Aquí es donde incluimos tu launch de drivers existente
    bringup_launch_path = os.path.join(control_pkg_dir, 'launch', 'pfc_bringup.launch.py')

    # 3. Configuraciones
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 4. Incluir el launch de drivers (Sensores + Motores)
    # Esto ejecuta todo lo que ya tienes configurado en control_pkg
    pfc_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Nodo de Health Status (Supervisión)
    health_node = Node(
        package='pfc_actions',
        executable='health_node',
        name='health_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. Nodo de Polar Action Server
    # Este nodo usará las ganancias (k_rho, k_alpha, etc.) definidas en tu params.yaml
    polar_action = Node(
        package='pfc_actions',
        executable='polar_action',
        name='polar_action_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        pfc_bringup,
        health_node,
        polar_action
    ])