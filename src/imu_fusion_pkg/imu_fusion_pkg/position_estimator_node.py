
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from collections import deque

class BidirectionalStopEstimator(Node):
    def __init__(self):
        super().__init__('bidirectional_stop_estimator')
        
        # Subscriptor IMU
        self.sub_fused = self.create_subscription(
            Imu,
            '/imu/fused',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.pub_position = self.create_publisher(Vector3, '/estimation/position', 10)
        self.pub_velocity = self.create_publisher(Vector3, '/estimation/velocity', 10)
        self.pub_acceleration = self.create_publisher(Vector3, '/estimation/acceleration', 10)
        self.pub_orientation = self.create_publisher(Quaternion, '/estimation/orientation', 10)
        self.pub_rpy = self.create_publisher(Vector3, '/estimation/rpy', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/estimation/pose', 10)
        
        # PARÁMETROS MEJORADOS
        self.ACCEL_THRESHOLD_XY = 0.08
        self.ACCEL_THRESHOLD_Z = 0.1
        self.GRAVITY = 9.81
        
        # Estado del sistema
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.linear_accel = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        
        # DETECCIÓN DE MOVIMIENTO BIDIRECCIONAL
        self.movement_detection_window = deque(maxlen=5)
        self.consecutive_still_frames = 0
        self.still_frames_required = 4  # Más frames para evitar falsas paradas
        self.is_currently_moving = False
        
        # Historial para detectar cambios
        self.accel_history = deque(maxlen=10)
        self.velocity_history = deque(maxlen=8)
        
        # Tiempo
        self.last_time = self.get_clock().now()
        self.update_count = 0
        
        self.get_logger().info("🔄 Estimador Bidireccional con Parada Mejorada")
        self.get_logger().info("🎯 Detecta avance/retroceso y para correctamente")

    def imu_callback(self, msg):
        """Callback con detección bidireccional mejorada"""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            self.update_count += 1
            
            if dt <= 0 or dt > 0.1:
                return
            
            # Extraer datos IMU
            accel_body = np.array([
                msg.linear_acceleration.y,
                msg.linear_acceleration.x, 
                msg.linear_acceleration.z
            ])
            
            gyro_body = np.array([
                msg.angular_velocity.y,
                msg.angular_velocity.x,
                msg.angular_velocity.z
            ])
            
            # 1. ACTUALIZAR ORIENTACIÓN
            self.update_orientation(gyro_body, accel_body, dt)
            
            # 2. ESTIMAR ACELERACIÓN LINEAL (BIDIRECCIONAL)
            accel_global = self.get_bidirectional_acceleration(accel_body)
            self.linear_accel = accel_global
            self.accel_history.append(accel_global)
            
            # 3. DETECTAR MOVIMIENTO CLARO (AVANCE O RETROCESO)
            clear_movement_detected = self.detect_clear_movement(accel_global, self.velocity)
            
            # 4. ACTUALIZAR ESTADO DE MOVIMIENTO
            self.update_movement_state(clear_movement_detected)
            
            # 5. EJECUTAR LÓGICA PRINCIPAL
            if self.is_currently_moving:
                self.integrate_movement(accel_global, dt)
            else:
                self.apply_hard_stop()
            
            # 6. PUBLICAR ESTIMACIONES
            self.publish_all_estimations()
            
            # 7. MOSTRAR ESTADO
            if self.update_count % 20 == 0:
                self.display_bidirectional_status()
                
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

    def get_bidirectional_acceleration(self, accel_body):
        """Aceleración que detecta tanto avance como retroceso"""
        rot = R.from_quat([self.orientation[1], self.orientation[2], 
                          self.orientation[3], self.orientation[0]])
        accel_global = rot.apply(accel_body)
        accel_global[2] -= self.GRAVITY
        
        # FILTRADO BIDIRECCIONAL - mantener signo para retroceso
        filtered_accel = np.array([0.0, 0.0, 0.0])
        
        for i in range(3):
            threshold = self.ACCEL_THRESHOLD_XY if i < 2 else self.ACCEL_THRESHOLD_Z
            
            # Mantener el signo original para detectar dirección
            if abs(accel_global[i]) > threshold:
                filtered_accel[i] = accel_global[i]  # Mantener signo negativo para retroceso
        
        return filtered_accel

    def detect_clear_movement(self, accel_global, velocity):
        """Detectar movimiento claro en cualquier dirección"""
        accel_magnitude = np.linalg.norm(accel_global)
        velocity_magnitude = np.linalg.norm(velocity)
        
        # MOVIMIENTO CLARO: Aceleración significativa O velocidad considerable
        has_significant_accel = accel_magnitude > 0.15  # Umbral más bajo para mejor sensibilidad
        has_considerable_velocity = velocity_magnitude > 0.05
        
        # También considerar aceleración sostenida
        if len(self.accel_history) == self.accel_history.maxlen:
            avg_accel = np.mean([np.linalg.norm(a) for a in self.accel_history])
            has_sustained_accel = avg_accel > 0.08
        else:
            has_sustained_accel = False
        
        movement_detected = (has_significant_accel or 
                           has_considerable_velocity or 
                           has_sustained_accel)
        
        self.movement_detection_window.append(movement_detected)
        
        # Requerir confirmación múltiple
        movement_confidence = sum(self.movement_detection_window) / len(self.movement_detection_window)
        return movement_confidence > 0.6  # 60% de confianza

    def update_movement_state(self, movement_detected):
        """Actualizar estado de movimiento con histeresis"""
        if movement_detected:
            self.consecutive_still_frames = 0
            self.is_currently_moving = True
        else:
            self.consecutive_still_frames += 1
            
            # Solo cambiar a "parado" después de múltiples frames quietos
            if self.consecutive_still_frames >= self.still_frames_required:
                self.is_currently_moving = False

    def integrate_movement(self, accel_global, dt):
        """Integrar movimiento en cualquier dirección"""
        # Integrar velocidad (mantiene signo para retroceso)
        self.velocity += accel_global * dt
        
        # Integrar posición
        self.position += self.velocity * dt
        
        # Decay suave solo durante movimiento
        self.velocity *= 0.97
        
        # Guardar historial de velocidad
        self.velocity_history.append(self.velocity.copy())

    def apply_hard_stop(self):
        """Aplicar parada dura cuando no hay movimiento"""
        # 1. Reset inmediato de velocidad
        self.velocity = np.array([0.0, 0.0, 0.0])
        
        # 2. Corrección suave de posición si hay drift
        position_magnitude = np.linalg.norm(self.position)
        if position_magnitude > 0.01:  # 1 cm de drift
            correction_factor = 0.99
            self.position *= correction_factor
        
        # 3. Limpiar historiales
        if len(self.velocity_history) > 0:
            self.velocity_history.clear()

    def update_orientation(self, gyro, accel, dt):
        """Actualizar orientación"""
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0:
            accel_normalized = accel / accel_norm
            roll_accel = math.atan2(accel_normalized[1], accel_normalized[2])
            pitch_accel = math.atan2(-accel_normalized[0], 
                                   math.sqrt(accel_normalized[1]**2 + accel_normalized[2]**2))
        else:
            roll_accel, pitch_accel = 0, 0
        
        current_rot = R.from_quat([self.orientation[1], self.orientation[2], 
                                 self.orientation[3], self.orientation[0]])
        roll_current, pitch_current, yaw_current = current_rot.as_euler('xyz')
        
        alpha = 0.98
        roll_fused = alpha * (roll_current + gyro[1] * dt) + (1 - alpha) * roll_accel
        pitch_fused = alpha * (pitch_current + gyro[0] * dt) + (1 - alpha) * pitch_accel
        yaw_fused = yaw_current + gyro[2] * dt
        
        fused_rot = R.from_euler('xyz', [roll_fused, pitch_fused, yaw_fused])
        new_orientation = fused_rot.as_quat()
        self.orientation = np.array([new_orientation[3], new_orientation[0], 
                                   new_orientation[1], new_orientation[2]])

    def quaternion_to_euler(self, q):
        """Convertir cuaternión a ángulos Euler"""
        w, x, y, z = q
        roll = math.atan2(2*(w*x + y*z), 1-2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
        return roll, pitch, yaw

    def publish_all_estimations(self):
        """Publicar todas las estimaciones"""
        # Posición
        pos_msg = Vector3()
        pos_msg.x = self.position[0]
        pos_msg.y = self.position[1]
        pos_msg.z = self.position[2]
        self.pub_position.publish(pos_msg)
        
        # Velocidad
        vel_msg = Vector3()
        vel_msg.x = self.velocity[0]
        vel_msg.y = self.velocity[1]
        vel_msg.z = self.velocity[2]
        self.pub_velocity.publish(vel_msg)
        
        # Aceleración
        accel_msg = Vector3()
        accel_msg.x = self.linear_accel[0]
        accel_msg.y = self.linear_accel[1]
        accel_msg.z = self.linear_accel[2]
        self.pub_acceleration.publish(accel_msg)
        
        # Orientación
        orient_msg = Quaternion()
        orient_msg.w = self.orientation[0]
        orient_msg.x = self.orientation[1]
        orient_msg.y = self.orientation[2]
        orient_msg.z = self.orientation[3]
        self.pub_orientation.publish(orient_msg)
        
        # Roll, Pitch, Yaw
        rpy_msg = Vector3()
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation)
        rpy_msg.x = roll
        rpy_msg.y = pitch
        rpy_msg.z = yaw
        self.pub_rpy.publish(rpy_msg)
        
        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        pose_msg.pose.orientation = orient_msg
        self.pub_pose.publish(pose_msg)

    def display_bidirectional_status(self):
        """Mostrar estado bidireccional"""
        pos_cm = self.position * 100
        vel_cms = self.velocity * 100
        accel_mag = np.linalg.norm(self.linear_accel)
        
        status = "🚀 MOVIÉNDOSE" if self.is_currently_moving else "🛑 PARADO"
        direction = ""
        
        # Detectar dirección
        if self.is_currently_moving:
            if self.velocity[0] > 0.01:
                direction = " → AVANZANDO"
            elif self.velocity[0] < -0.01:
                direction = " ← RETROCEDIENDO"
            elif self.velocity[1] > 0.01:
                direction = " ↑ IZQUIERDA"
            elif self.velocity[1] < -0.01:
                direction = " ↓ DERECHA"
        
        self.get_logger().info(f"{status}{direction}")
        self.get_logger().info(f"📍 POS: X={pos_cm[0]:.1f}cm, Y={pos_cm[1]:.1f}cm")
        self.get_logger().info(f"🎯 VEL: {np.linalg.norm(vel_cms):.1f}cm/s, ACC: {accel_mag:.3f}m/s²")
        
        if not self.is_currently_moving and np.linalg.norm(vel_cms) > 0.5:
            self.get_logger().warning("⚠️  Parando...")
        
        self.get_logger().info("---")

def main(args=None):
    rclpy.init(args=args)
    node = BidirectionalStopEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
