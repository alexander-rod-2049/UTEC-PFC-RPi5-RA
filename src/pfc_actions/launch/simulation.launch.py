import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationEquals

def generate_launch_description():
    pfc_actions_dir = get_package_share_directory('pfc_actions')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    params_file = os.path.join(pfc_actions_dir, 'config', 'params.yaml')

    controller_arg = LaunchConfiguration('controller')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Gazebo con TurtleBot 3
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Health Node
    health_node = Node(
        package='pfc_actions', executable='health_node', name='health_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # 3. Controlador Polar (Solo si controller:=polar)
    polar_node = Node(
        package='pfc_actions', executable='polar_action', name='polar_action_server',
        condition=LaunchConfigurationEquals('controller', 'polar'),
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # 4. Controlador SMC+DFL (Solo si controller:=smc)
    smc_node = Node(
        package='pfc_actions', executable='smc_dfl_action', name='smc_dfl_action_server',
        condition=LaunchConfigurationEquals('controller', 'smc'),
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='polar', description='polar o smc'),
        gz_sim,
        health_node,
        polar_node,
        smc_node
    ])