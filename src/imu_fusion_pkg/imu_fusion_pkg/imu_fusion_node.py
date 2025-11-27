#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import statistics

class IMUFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')
        
        # Subscriptores para los datos crudos de ambos IMUs
        self.sub_imu1 = self.create_subscription(
            Imu,
            '/imu/raw_1',
            self.imu1_callback,
            10
        )
        
        self.sub_imu2 = self.create_subscription(
            Imu,
            '/imu/raw_2', 
            self.imu2_callback,
            10
        )
        
        # Publisher para datos fusionados
        self.pub_fused = self.create_publisher(Imu, '/imu/fused', 10)
        
        # Buffers para almacenar los últimos datos
        self.imu1_data = None
        self.imu2_data = None
        
        # Timer para procesamiento periódico (50 Hz = 0.02s)
        self.timer = self.create_timer(0.01, self.fusion_callback)
        
        # Contador para display (mostrar cada 1 segundo)
        self.display_counter = 0
        
        self.get_logger().info("🔄 Nodo de fusión IMU iniciado")
        self.get_logger().info("📍 Suscrito a /imu/raw_1 y /imu/raw_2")
        self.get_logger().info("📡 Publicando en /imu/fused")
        self.get_logger().info("⏰ Frecuencia: 50 Hz | Display cada 1s")
    
    def imu1_callback(self, msg):
        """Callback para datos del IMU 1"""
        self.imu1_data = msg
    
    def imu2_callback(self, msg):
        """Callback para datos del IMU 2"""  
        self.imu2_data = msg
    
    def fusion_callback(self):
        """Callback para fusionar datos y publicar"""
        if self.imu1_data is None or self.imu2_data is None:
            return  # Esperar hasta tener datos de ambos sensores
        
        try:
            # Crear mensaje fusionado
            fused_msg = Imu()
            
            # Copiar header del mensaje más reciente
            fused_msg.header.stamp = self.get_clock().now().to_msg()
            fused_msg.header.frame_id = "imu_fused"
            
            # FUSIÓN: Promedio de aceleración lineal
            fused_msg.linear_acceleration.x = statistics.mean([
                self.imu1_data.linear_acceleration.x,
                self.imu2_data.linear_acceleration.x
            ])
            fused_msg.linear_acceleration.y = statistics.mean([
                self.imu1_data.linear_acceleration.y, 
                self.imu2_data.linear_acceleration.y
            ])
            fused_msg.linear_acceleration.z = statistics.mean([
                self.imu1_data.linear_acceleration.z,
                self.imu2_data.linear_acceleration.z
            ])
            
            # FUSIÓN: Promedio de velocidad angular
            fused_msg.angular_velocity.x = statistics.mean([
                self.imu1_data.angular_velocity.x,
                self.imu2_data.angular_velocity.x
            ])
            fused_msg.angular_velocity.y = statistics.mean([
                self.imu1_data.angular_velocity.y,
                self.imu2_data.angular_velocity.y
            ])
            fused_msg.angular_velocity.z = statistics.mean([
                self.imu1_data.angular_velocity.z, 
                self.imu2_data.angular_velocity.z
            ])
            
            # Copiar covarianzas (usar las del IMU1 como referencia)
            fused_msg.orientation_covariance = self.imu1_data.orientation_covariance
            fused_msg.linear_acceleration_covariance = self.imu1_data.linear_acceleration_covariance
            fused_msg.angular_velocity_covariance = self.imu1_data.angular_velocity_covariance
            
            # Publicar mensaje fusionado
            self.pub_fused.publish(fused_msg)
            
            # Mostrar datos cada 1 segundo (cada 50 ejecuciones = 50 Hz)
            self.display_counter += 1
            if self.display_counter >= 50:
                accel_x = fused_msg.linear_acceleration.x / 9.81  # Convertir a g
                accel_y = fused_msg.linear_acceleration.y / 9.81
                accel_z = fused_msg.linear_acceleration.z / 9.81
                
                gyro_x = fused_msg.angular_velocity.x
                gyro_y = fused_msg.angular_velocity.y
                gyro_z = fused_msg.angular_velocity.z
                
                self.get_logger().info(f"🎯 IMU FUSIONADO - Accel: [{accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}] g")
                self.get_logger().info(f"🎯 IMU FUSIONADO - Gyro: [{gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f}] rad/s")
                self.get_logger().info("---")
                self.display_counter = 0
            
        except Exception as e:
            self.get_logger().error(f"❌ Error en fusión IMU: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = IMUFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
