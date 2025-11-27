#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from .mpu6050_driver import DualMPU6050

class IMUReaderNode(Node):
    def __init__(self):
        super().__init__('imu_reader_node')
        
        # Publishers para cada IMU
        self.pub_imu1 = self.create_publisher(Imu, '/imu/raw_1', 10)
        self.pub_imu2 = self.create_publisher(Imu, '/imu/raw_2', 10)
        
        # Inicializar driver de IMUs
        self.imu_driver = DualMPU6050()
        
        # Timer para lectura periódica (50 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info("📡 Nodo lector IMU iniciado")
        self.get_logger().info("📍 Publicando en /imu/raw_1 y /imu/raw_2")
    
    def create_imu_message(self, sensor_data, frame_id):
        """Crear mensaje Imu ROS2 a partir de datos del sensor"""
        imu_msg = Imu()
        
        # Header con timestamp
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        
        # Aceleración lineal (en m/s²)
        imu_msg.linear_acceleration.x = sensor_data['accel'][0]
        imu_msg.linear_acceleration.y = sensor_data['accel'][1]
        imu_msg.linear_acceleration.z = sensor_data['accel'][2]
        
        # Velocidad angular (en rad/s)
        imu_msg.angular_velocity.x = sensor_data['gyro'][0] * 0.0174533  # °/s a rad/s
        imu_msg.angular_velocity.y = sensor_data['gyro'][1] * 0.0174533
        imu_msg.angular_velocity.z = sensor_data['gyro'][2] * 0.0174533
        
        # Covarianzas
        imu_msg.orientation_covariance[0] = -1  # Orientación no disponible
        
        return imu_msg
    
    def timer_callback(self):
        """Callback del timer para lectura periódica"""
        try:
            # Leer datos de ambos IMUs
            imu1_data, imu2_data = self.imu_driver.read_both_imus()
            
            # Publicar mensajes
            imu1_msg = self.create_imu_message(imu1_data, "imu_1")
            imu2_msg = self.create_imu_message(imu2_data, "imu_2")
            
            self.pub_imu1.publish(imu1_msg)
            self.pub_imu2.publish(imu2_msg)
            
            # Log cada 1 segundo para no saturar
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                self.get_logger().info(f"📊 IMU1 - Accel: [{imu1_data['accel'][0]:.3f}, {imu1_data['accel'][1]:.3f}, {imu1_data['accel'][2]:.3f}] Gyro: [{imu1_data['gyro'][0]:.3f}, {imu1_data['gyro'][1]:.3f}, {imu1_data['gyro'][2]:.3f}]")
                self.get_logger().info(f"📊 IMU2 - Accel: [{imu2_data['accel'][0]:.3f}, {imu2_data['accel'][1]:.3f}, {imu2_data['accel'][2]:.3f}] Gyro: [{imu2_data['gyro'][0]:.3f}, {imu2_data['gyro'][1]:.3f}, {imu2_data['gyro'][2]:.3f}]")
                self.get_logger().info("---")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error en lectura IMU: {e}")
    
    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self.imu_driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = IMUReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
