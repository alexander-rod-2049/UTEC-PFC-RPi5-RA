import time
import rclpy
import serial
import json
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

# from geometry_msgs.msg import TransformStamped  <-- Not needed anymore
# from tf2_ros import TransformBroadcaster      <-- Not needed anymore

class ImuBNO08XPublisher(Node):

    def __init__(self):
        super().__init__('imu_bno08x_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_topic', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        timer_period = 0.02  # 50Hz (Approx)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- TF BROADCASTER REMOVED FOR SENSOR FUSION ---
        # self.tf_broadcaster = TransformBroadcaster(self) 
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1) # Lowered timeout for better loop performance
            time.sleep(2)
            self.get_logger().info("Serial connection established on /dev/ttyACM0")
        except serial.SerialException:
             self.get_logger().error("FATAL: Could not open /dev/ttyACM0. Is the sensor plugged in?")
             exit(1)

    def timer_callback(self):
        msg = Imu()
        time_now = self.get_clock().now().to_msg()
        try:
            # Read line from serial
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline()
                if raw_data:
                    decoded_data = raw_data.decode('utf-8').strip()
                    if decoded_data.startswith('{') and decoded_data.endswith('}'):
                        imu_data = json.loads(decoded_data)

                        # Orientation (Quaternion)
                        msg.orientation.w = float(imu_data.get('qw', 1.0))
                        msg.orientation.x = float(imu_data.get('qx', 0.0))
                        msg.orientation.y = float(imu_data.get('qy', 0.0))
                        msg.orientation.z = float(imu_data.get('qz', 0.0))
                        
                        # Angular Velocity
                        msg.angular_velocity.x = float(imu_data.get('gx', 0.0))
                        msg.angular_velocity.y = float(imu_data.get('gy', 0.0))
                        msg.angular_velocity.z = float(imu_data.get('gz', 0.0))
                        
                        # Linear Acceleration
                        msg.linear_acceleration.x = float(imu_data.get('ax', 0.0))
                        msg.linear_acceleration.y = float(imu_data.get('ay', 0.0))
                        msg.linear_acceleration.z = float(imu_data.get('az', 0.0))

                        msg.header.stamp = time_now
                        msg.header.frame_id = 'imu_link'
                        
                        # Covariance is vital for EKF later!
                        # 0.01 is standard for "trust this sensor"
                        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                                    0.0, 0.01, 0.0,
                                                    0.0, 0.0, 0.01]
                        
                        # Magnetometer Data
                        mag_msg = MagneticField()
                        mag_msg.header.stamp = time_now
                        mag_msg.header.frame_id = "imu_link"
                        mag_msg.magnetic_field.x = imu_data.get('mx', 0.0) * 1e-6 
                        mag_msg.magnetic_field.y = imu_data.get('my', 0.0) * 1e-6
                        mag_msg.magnetic_field.z = imu_data.get('mz', 0.0) * 1e-6

                        # --- TF BROADCASTING DISABLED ---
                        # We do NOT want to publish 'odom' -> 'base_link' here.
                        # The EKF (robot_localization) or your wheel odometry will do that.
                        
                        self.mag_pub.publish(mag_msg)
                        self.publisher_.publish(msg)
                        
        except (serial.SerialException, json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f"Sensor read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuBNO08XPublisher()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()