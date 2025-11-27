import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.signal import butter, lfilter
import math

def quaternion_to_euler(q):
    """Convierte un cuaternión a ángulos de Euler (roll, pitch, yaw)."""
    (w, x, y, z) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def euler_to_quaternion(roll, pitch, yaw):
    """Convierte ángulos de Euler a un cuaternión."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])

class EKF:
    def __init__(self, dt, g, initial_state, initial_P_diag, Q_diag, R_diag):
        self.dt = dt
        self.g = g
        self.x = initial_state  # [qw, qx, qy, qz, bgx, bgy, bgz, bax, bay, baz]
        self.P = np.diag(initial_P_diag)
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        
        # Estado de 10 elementos: 4 para cuaternión, 3 para sesgo de giróscopo, 3 para sesgo de acelerómetro
        self.F = np.eye(10) 
        self.H = np.zeros((3, 10))

    def predict(self, gyro_meas):
        # Extraer estados
        q = self.x[:4]
        bg = self.x[4:7]
        
        # Giróscopo corregido por el sesgo
        gyro_corrected = gyro_meas - bg
        wx, wy, wz = gyro_corrected
        
        # Matriz de transición del cuaternión
        Omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        
        # Propagación del estado del cuaternión
        q_dot = 0.5 * Omega @ q
        q = q + q_dot * self.dt
        q = q / np.linalg.norm(q) # Normalizar cuaternión
        self.x[:4] = q
        
        # Jacobiano F
        # d(q_dot)/d(q)
        F_qq = np.eye(4) + 0.5 * Omega * self.dt
        # d(q_dot)/d(bg)
        F_qbg = 0.5 * self.dt * np.array([
            [q[1], q[2], q[3]],
            [-q[0], q[3], -q[2]],
            [-q[3], -q[0], q[1]],
            [q[2], -q[1], -q[0]]
        ])
        
        self.F[0:4, 0:4] = F_qq
        self.F[0:4, 4:7] = F_qbg
        
        # Propagación de la covarianza
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, acc_meas):
        q = self.x[:4]
        ba = self.x[7:10]
        
        # Medición predicha (aceleración debida a la gravedad en el marco del sensor)
        g_sensor_frame = np.array([
            2 * (q[1]*q[3] - q[0]*q[2]),
            2 * (q[2]*q[3] + q[0]*q[1]),
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        ]) * self.g
        z_pred = g_sensor_frame + ba
        
        # Jacobiano H
        qw, qx, qy, qz = q
        H_q = 2 * self.g * np.array([
            [-qy, qz, -qw, qx],
            [qx, qw, qz, qy],
            [qw, -qx, -qy, qz]
        ])
        H_ba = np.eye(3)
        
        self.H[:, 0:4] = H_q
        self.H[:, 7:10] = H_ba
        
        # Ganancia de Kalman
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Actualización del estado
        y = acc_meas - z_pred # Residuo
        self.x = self.x + K @ y
        self.x[:4] = self.x[:4] / np.linalg.norm(self.x[:4]) # Normalizar cuaternión
        
        # Actualización de la covarianza
        self.P = (np.eye(10) - K @ self.H) @ self.P

class ImuFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        
        # Parámetros proporcionados por el usuario
        self.dt = 0.02
        self.g = 9.81
        initial_state = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        initial_P_diag = np.array([0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01])
        Q_diag = np.array([0.0001, 0.0001, 0.0001, 0.0001, 1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5])
        R_diag = np.array([0.1, 0.1, 0.1])

        # Inicializar EKF
        self.ekf = EKF(self.dt, self.g, initial_state, initial_P_diag, Q_diag, R_diag)

        # Parámetros del filtro Butterworth
        fs = 50.0  # Frecuencia de muestreo
        cutoff_freq = 2.0  # Frecuencia de corte
        order = 4
        normal_cutoff = cutoff_freq / (0.5 * fs)
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi_roll = np.zeros(max(len(self.a), len(self.b)) - 1)
        self.zi_pitch = np.zeros(max(len(self.a), len(self.b)) - 1)
        self.zi_yaw = np.zeros(max(len(self.a), len(self.b)) - 1)

        # ROS
        self.subscription = self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        self.publisher = self.create_publisher(Imu, 'imu/data_filtered', 10)
        self.last_time = None
        
        self.get_logger().info("Nodo de filtrado de IMU iniciado.")

    def imu_callback(self, msg):
        # Recordar que el nodo de datos crudos no convierte a rad/s ni m/s^2
        # El EKF los necesita en esas unidades, así que los convertimos aquí.
        gyro_meas = np.array([
            msg.angular_velocity.x * (math.pi / 180.0),
            msg.angular_velocity.y * (math.pi / 180.0),
            msg.angular_velocity.z * (math.pi / 180.0)
        ])
        acc_meas = np.array([
            msg.linear_acceleration.x * self.g,
            msg.linear_acceleration.y * self.g,
            msg.linear_acceleration.z * self.g
        ])

        # Actualizar dt
        current_time = self.get_clock().now()
        if self.last_time is not None:
            self.dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Pasos del EKF
        self.ekf.predict(gyro_meas)
        self.ekf.update(acc_meas)
        
        # Extraer cuaternión del EKF
        q_ekf = self.ekf.x[:4]

        # Aplicar filtro Butterworth a los ángulos de Euler
        roll, pitch, yaw = quaternion_to_euler(q_ekf)
        
        roll_filt, self.zi_roll = lfilter(self.b, self.a, [roll], zi=self.zi_roll)
        pitch_filt, self.zi_pitch = lfilter(self.b, self.a, [pitch], zi=self.zi_pitch)
        yaw_filt, self.zi_yaw = lfilter(self.b, self.a, [yaw], zi=self.zi_yaw)
        
        q_filtered = euler_to_quaternion(roll_filt[0], pitch_filt[0], yaw_filt[0])
        
        # Publicar mensaje filtrado
        filtered_msg = Imu()
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.header.frame_id = 'imu_link'
        
        filtered_msg.orientation.w = q_filtered[0]
        filtered_msg.orientation.x = q_filtered[1]
        filtered_msg.orientation.y = q_filtered[2]
        filtered_msg.orientation.z = q_filtered[3]
        
        # Se puede añadir covarianza de la orientación desde self.ekf.P si es necesario
        filtered_msg.orientation_covariance[0] = self.ekf.P[0,0]
        filtered_msg.orientation_covariance[4] = self.ekf.P[1,1]
        filtered_msg.orientation_covariance[8] = self.ekf.P[2,2]

        # Rellenar los otros campos con los datos originales (sin filtrar)
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_filter_node = ImuFilterNode()
    rclpy.spin(imu_filter_node)
    imu_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
