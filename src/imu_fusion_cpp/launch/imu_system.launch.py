import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'imu_fusion_cpp'
    config = os.path.join(get_package_share_directory(pkg), 'config', 'slip_fusion.yaml')

    return LaunchDescription([
        # Driver Hardware
        Node(package=pkg, executable='multi_imu_driver', name='imu_driver'),
        
        # Procesadores
        Node(package=pkg, executable='lateral_imu_processor', name='proc_left', parameters=[{'side': 'left'}]),
        Node(package=pkg, executable='lateral_imu_processor', name='proc_right', parameters=[{'side': 'right'}]),
        
        # Estimador Slip
        Node(package=pkg, executable='slip_estimator_node', name='slip_est', parameters=[{'track_width': 0.55}]),

        # Fusión
        Node(package='robot_localization', executable='ekf_node', name='ekf_node', parameters=[config], remappings=[('odometry/filtered', '/odometry/filtered')]),
        
        # Transformadas (Tus datos)
# 4. TRANSFORMADAS ESTÁTICAS (Sintaxis Nueva - Sin Warnings)
        
        # Central: (-0.03, 0.01, 0.30)
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['--x', '-0.03', '--y', '0.01', '--z', '0.30', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link_central']),
             
        # Izquierda: (0.12, 0.25, -0.02)
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['--x', '0.12', '--y', '0.25', '--z', '-0.02', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link_left']),

        # Derecha: (0.12, -0.25, -0.02)
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['--x', '0.12', '--y', '-0.25', '--z', '-0.02', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link_right']),

        # Magnetómetro GPS: (-0.03, 0.01, 0.50)
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['--x', '-0.03', '--y', '0.01', '--z', '0.50', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link_mag']),
    ])