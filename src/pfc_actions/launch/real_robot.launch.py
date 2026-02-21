# Ubicación: ~/PFCII/pfc2_ws/src/pfc_actions/launch/real_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    control_pkg_dir = get_package_share_directory('control_pkg')
    ekf_params = os.path.join(pfc_actions_dir, 'config', 'ekf_real.yaml')

    return LaunchDescription([
        LogInfo(msg="Iniciando Hardware y Localización (EKF)..."),
        # 1. Sensores y Motores
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg_dir, 'launch', 'pfc_bringup.launch.py')
            )
        ),
        # 2. TF Estática IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0.04', '0.02', '0.082', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
        ),
        # 3. Filtro de Kalman (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params]
        )
        # 4. Vigilante de Salud (Health)
        #Node(package='pfc_actions', executable='health_node', name='health_node')
    ])