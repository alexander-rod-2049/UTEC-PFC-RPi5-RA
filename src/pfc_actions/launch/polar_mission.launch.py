import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Directorios de los paquetes
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    control_pkg_dir = get_package_share_directory('control_pkg')
    
    # Rutas de archivos de parámetros
    ekf_params = os.path.join(pfc_actions_dir, 'config', 'ekf_real.yaml')
    pfc_params = os.path.join(pfc_actions_dir, 'config', 'params.yaml')

    return LaunchDescription([
        LogInfo(msg="Iniciando Navegación Polar con EKF..."),

        # 1. Bringup de Hardware (Motores RoboClaw y Sensores)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg_dir, 'launch', 'pfc_bringup.launch.py')
            )
        ),

        # 2. Transformada Estática (Base_link -> IMU)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0.04', '0.02', '0.082', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
        ),

        # 3. Localización - Filtro de Kalman (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen'
        ),

        # 4. Servidor de Acción Polar
        Node(
            package='pfc_actions',
            executable='polar_action_server',
            name='polar_action_server',
            parameters=[pfc_params],
            output='screen'
        ),

        # 5. Vigilante de Salud (Health Node)
        Node(
            package='pfc_actions',
            executable='health_node',
            name='health_node',
            parameters=[pfc_params],
            output='screen'
        ),

        # 6. Mission Manager Polar (Lanzado con un delay de 5s)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='control_pkg',
                    executable='mission_manager_polar',
                    name='mission_manager_polar',
                    output='screen'
                )
            ]
        )
    ])