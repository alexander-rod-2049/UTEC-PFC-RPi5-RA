import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import \
    MPU9050_ADDRESS_68, GFS_2000, AFS_16G, AK8963_BIT_16, AK8963_MODE_C100HZ
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_publisher_ = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50 Hz

        try:
            # The address of the MPU-9250 can be 0x68 or 0x69. 0x68 is the default.
            self.mpu = MPU9250(
                address_mpu_master=MPU9050_ADDRESS_68,
                bus=1, # Raspberry Pi typically uses I2C bus 1
                gfs=GFS_2000,
                afs=AFS_16G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ
            )
            self.mpu.configure() # Apply the settings to the registers.
            self.get_logger().info('IMU MPU-9250 inicializada exitosamente.')
        except Exception as e:
            self.get_logger().error(f'No se pudo inicializar la IMU: {e}')
            rclpy.shutdown()
            return

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Leer datos del sensor
        accel = self.mpu.readAccelerometerMaster() # Returns in g's
        gyro = self.mpu.readGyroscopeMaster() # Returns in deg/s
        mag = self.mpu.readMagnetometerMaster() # Returns in uT (micro Teslas)

        # --- Publicar Mensaje Imu ---
        # Rellenar mensaje - Aceleración Lineal (sin convertir, en g's)
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1
        
        # Rellenar mensaje - Velocidad Angular (sin convertir, en deg/s)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance[0] = 0.1
        imu_msg.angular_velocity_covariance[4] = 0.1
        imu_msg.angular_velocity_covariance[8] = 0.1

        # Orientación no es proporcionada directamente por este sensor.
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_publisher_.publish(imu_msg)

        # --- Publicar Mensaje MagneticField ---
        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = 'imu_link'
        
        # La librería devuelve microTeslas (uT). El mensaje de ROS espera Teslas (T).
        # Convertimos de uT a T
        mag_msg.magnetic_field.x = mag[0] * 1e-6
        mag_msg.magnetic_field.y = mag[1] * 1e-6
        mag_msg.magnetic_field.z = mag[2] * 1e-6
        mag_msg.magnetic_field_covariance[0] = 0.1
        mag_msg.magnetic_field_covariance[4] = 0.1
        mag_msg.magnetic_field_covariance[8] = 0.1
        self.mag_publisher_.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    if rclpy.ok():
        try:
            rclpy.spin(imu_node)
        except KeyboardInterrupt:
            pass
        finally:
            imu_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
