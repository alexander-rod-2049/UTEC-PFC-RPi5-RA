import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SimpleGoTo(Node):
    def __init__(self):
        super().__init__('simple_goto_node')
        
        # Publicador a los motores y suscriptor al EKF
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # --- LA META ---
        self.target_x = 1.2  # Ir 1 metro hacia adelante
        self.target_y = 0.5  # Sin moverse a los lados
        
        # Estado actual del robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # --- PARAMETROS DE CONTROL (Ajustables) ---
        self.k_linear = 0.6       # Qué tan fuerte acelera al avanzar
        self.k_angular = 0.5      # Qué tan fuerte gira
        self.max_v = 0.2          # Límite de velocidad lineal (m/s)
        self.max_w = 0.4          # Límite de velocidad angular (rad/s)
        
        self.dist_tolerance = 0.1 # Se detiene a 10 cm de la meta
        self.yaw_tolerance = 0.1  # Tolerancia de +- 5 grados para apuntar
        
        self.goal_reached = False

    def get_yaw_from_quaternion(self, q):
        # Conversión manual de Cuaternión a Ángulo Euler (Z)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        if self.goal_reached:
            return

        # 1. Leer posición actual del EKF
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        # 2. Calcular errores matemáticos
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        
        # Normalizar el error del ángulo entre -PI y PI
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        cmd = Twist()

        # 3. Lógica de Control (Máquina de estados simple)
        if distance < self.dist_tolerance:
            self.get_logger().info('🏁 ¡META ALCANZADA! Apagando motores.')
            self.cmd_pub.publish(Twist()) # Mandar ceros
            self.goal_reached = True
            return

        # AUMENTAMOS LA TOLERANCIA a 0.25 rad (~14 grados) para que no sea tan estricto
        self.yaw_tolerance = 0.25 

        # Fase 1: Si estoy mirando muy lejos de la meta, solo giro
        if abs(yaw_error) > self.yaw_tolerance:
            cmd.angular.z = self.k_angular * yaw_error  # <-- SIN EL MENOS
            cmd.angular.z = max(min(cmd.angular.z, self.max_w), -self.max_w)
            
            # Fuerza mínima para vencer la fricción estática de la oruga
            if cmd.angular.z > 0 and cmd.angular.z < 0.15: cmd.angular.z = 0.15
            if cmd.angular.z < 0 and cmd.angular.z > -0.15: cmd.angular.z = -0.15
            
            self.get_logger().info(f'🔄 Apuntando... Error: {yaw_error:.2f} | Comando Z: {cmd.angular.z:.2f}')
            
        # Fase 2: Ya estoy apuntando a la meta, ahora avanzo
        else:
            cmd.linear.x = self.k_linear * distance
            cmd.linear.x = max(min(cmd.linear.x, self.max_v), 0.08) 
            
            # Corregimos el rumbo suavemente mientras avanza
            cmd.angular.z = self.k_angular * yaw_error * 0.8  # <-- SIN EL MENOS
            self.get_logger().info(f'⬆️ Avanzando {distance:.2f}m | Corrigiendo {yaw_error:.2f}rad')

        # 4. Enviar comando a las orugas
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist()) # Freno de emergencia si haces Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()