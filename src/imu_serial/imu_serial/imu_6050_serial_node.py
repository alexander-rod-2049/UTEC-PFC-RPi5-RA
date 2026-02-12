import time
import rclpy
import serial
import json
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu


from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Imu6050Publisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu_topic', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ser = serial.Serial('/dev/ttyACM0',115200,timeout=0.01)
        time.sleep(2)

    def timer_callback(self):
        msg = Imu()
        time_now = self.get_clock().now().to_msg()
        try:
            
            raw_data = self.ser.readline()
            if raw_data:
                decoded_data = raw_data.decode('utf-8').strip()
                if decoded_data.startswith('{') and decoded_data.endswith('}'):
                    imu_data = json.loads(decoded_data)

                    # Assuming imu_data contains the necessary fields
                    roll = imu_data['r']*(np.pi/180)  # Se recibe en grados y se transforma a radianes.
                    pitch = imu_data['p']*(np.pi/180) # Se recibe en grados y se transforma a radianes.
                    yaw = 0.0*(np.pi/180) # Yaw is not provided by MPU6050. Al menos no por ahora.
                    # De RPY a cuaterniones.
                    msg.orientation.x = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
                    msg.orientation.y = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
                    msg.orientation.z = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
                    msg.orientation.w = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)

                    msg.angular_velocity.x = imu_data['gx']
                    msg.angular_velocity.y = imu_data['gy']
                    msg.angular_velocity.z = imu_data['gz']

                    msg.linear_acceleration.x = imu_data['ax']
                    msg.linear_acceleration.y = imu_data['ay']
                    msg.linear_acceleration.z = imu_data['az']

                    msg.header.stamp = time_now
                    msg.header.frame_id = 'imu_link'
                    msg.orientation_covariance[0] = -1
                    t = TransformStamped()

                    t.header.stamp = time_now
                    t.header.frame_id = 'base_link'
                    t.child_frame_id = 'imu_link'

                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0

                    t.transform.rotation = msg.orientation

                    self.tf_broadcaster.sendTransform(t)

                    self.publisher_.publish(msg)
        except serial.SerialException:
            self.get_logger().error("Error: No se pudo abrir el puerto serial '/dev/ttyACM0'")
            return
        except json.JSONDecodeError:
        # Code to handle the specific exception
            self.get_logger().error("Error: Cannot decode JSON data from serial port")
        #finally:
            #if ser and ser.isOpen():
                #ser.close()
                #print("Serial port closed")
        

def main(args=None):
    rclpy.init(args=args)

    imu6050_publisher = Imu6050Publisher()

    rclpy.spin(imu6050_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu6050_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()