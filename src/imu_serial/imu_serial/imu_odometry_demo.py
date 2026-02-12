import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class ImuOdomDemo(Node):
    def __init__(self):
        super().__init__('imu_odom_demo')
        
        # Suscribirse al tópico que ya creaste
        self.subscription = self.create_subscription(
            Imu,
            '/imu_topic',
            self.listener_callback,
            10)
        
        # Variables para integración
        self.last_time = time.time()
        self.velocity_x = 0.0  # Velocidad lineal acumulada
        self.position_x = 0.0  # Posición acumulada (solo por curiosidad)
        
        # Filtro de zona muerta (Deadzone) para eliminar ruido mínimo
        self.accel_threshold = 0.05 

    def listener_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 1. VELOCIDAD ANGULAR (GIROSCOPIO) - ESTO ES ORO PURO
        # El sensor ya te da rad/s. No hay que integrar nada.
        angular_vel_z = msg.angular_velocity.z 

        # 2. VELOCIDAD LINEAL (ACELERÓMETRO) - AQUÍ ESTÁ EL RETO
        ax = msg.linear_acceleration.x

        # Filtro simple: Si la aceleración es muy bajita, asumimos 0 para no integrar ruido
        if abs(ax) < self.accel_threshold:
            ax = 0.0
        
        # Integración: V = V_inicial + (a * dt)
        self.velocity_x += ax * dt
        
        # (Opcional) Fricción virtual para que la velocidad baje a 0 si no hay aceleración
        # Sin esto, si empujas el robot y lo paras, la velocidad se quedaría "pegada" en el último valor.
        self.velocity_x *= 0.98 

        # Imprimir resultados
        print(f"--- MONITOREO IMU ---")
        print(f"Giro Z (Real): {angular_vel_z:.4f} rad/s")
        print(f"Accel X (Raw): {ax:.4f} m/s2")
        print(f"Velocidad X (Estimada): {self.velocity_x:.4f} m/s")
        print(f"---------------------")

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()