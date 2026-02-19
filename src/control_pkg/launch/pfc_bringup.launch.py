from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Locate the RoboClaw launch file included in the driver package
    roboclaw_launch_path = PathJoinSubstitution([
        FindPackageShare('ros2_roboclaw_driver'),
        'launch',
        'ros2_roboclaw_driver.launch.py'
    ])

    # 2. Include the RoboClaw Launch
    roboclaw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboclaw_launch_path)
    )

    # 3. Define your IMU Node
    # (Based on: ros2 run imu_serial imu_BNO08X_serial_node)
    imu_node = Node(
        package='imu_serial',
        executable='imu_BNO08X_serial_node',
        name='imu_node',
        output='screen',
        emulate_tty=True
    )

    imu_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        # Argumentos: x, y, z, yaw, pitch, roll, frame_padre, frame_hijo
        arguments=['0.04', '0.02', '0.082', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )
    
    mtf02p_node = Node(
        package='mtf02p_odometry',
        executable='mtf02p_odometry_node',
        name='mtf02p_node',
        #output='screen',
        emulate_tty=True
    )

    gps_node = Node(
        package='gps_pkg',
        executable='gps_node',
        name='gps_node',
        #output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        LogInfo(msg="Starting PFCII Robot Drivers..."),
        roboclaw_launch,
        imu_node,
        imu_static_tf_node,
        mtf02p_node,
        gps_node
    ])