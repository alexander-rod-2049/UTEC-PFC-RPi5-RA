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

    return LaunchDescription([
        LogInfo(msg="Starting PFCII Robot Drivers..."),
        roboclaw_launch,
        imu_node
    ])