#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, MagneticField
import matplotlib.pyplot as plt
import os
import math
import time
import csv
import numpy as np

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # Suscripciones
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(MagneticField, 'gps/mag', self.mag_callback, 10)
        
        # Datos
        self.lats = []
        self.lons = []
        self.alts = []
        self.gps_timestamps = []
        
        self.mag_x = []
        self.mag_y = []
        self.mag_z = []
        self.headings = []
        self.mag_timestamps = []
        
        # Configurar directorio de salida
        self.output_dir = os.path.expanduser('~/PFCII/pfc2_ws/data/gps_data')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        self.get_logger().info(f'📸 Visualizador iniciado. Guardando en: {self.output_dir}')
        
        # Timer para guardar gráficos cada 30 segundos
        self.create_timer(30.0, self.save_data)

    def gps_callback(self, msg):
        # Guardar incluso si no es válido para debug
        self.lats.append(msg.latitude)
        self.lons.append(msg.longitude)
        self.alts.append(msg.altitude)
        self.gps_timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def mag_callback(self, msg):
        self.mag_x.append(msg.magnetic_field.x)
        self.mag_y.append(msg.magnetic_field.y)
        self.mag_z.append(msg.magnetic_field.z)
        self.mag_timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
        # Calcular heading
        heading = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        # Declinación Lima -1.5 deg (aprox)
        heading += math.radians(-1.5)
        if heading < 0: heading += 2*math.pi
        if heading > 2*math.pi: heading -= 2*math.pi
        self.headings.append(math.degrees(heading))

    def save_data(self):
        timestamp = int(time.time())
        saved_any = False
        
        # 1. Guardar CSV GPS
        if self.lats:
            csv_path = os.path.join(self.output_dir, f'gps_log_{timestamp}.csv')
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Lat', 'Lon', 'Alt'])
                for t, lat, lon, alt in zip(self.gps_timestamps, self.lats, self.lons, self.alts):
                    writer.writerow([t, lat, lon, alt])
            self.get_logger().info(f'💾 GPS CSV guardado: {csv_path}')

        # 2. Guardar CSV Magnetometro
        if self.mag_x:
            csv_path = os.path.join(self.output_dir, f'mag_log_{timestamp}.csv')
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'MagX', 'MagY', 'MagZ', 'Heading_Deg'])
                for t, x, y, z, h in zip(self.mag_timestamps, self.mag_x, self.mag_y, self.mag_z, self.headings):
                    writer.writerow([t, x, y, z, h])
            self.get_logger().info(f'💾 Mag CSV guardado: {csv_path}')

        # 3. Plots
        if self.lats:
            # Scatter
            plt.figure(figsize=(10, 8))
            plt.scatter(self.lons, self.lats, c='blue', alpha=0.5, label='GPS Raw')
            plt.title(f'GPS Scatter Plot (n={len(self.lats)})')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')
            plt.grid(True)
            plt.legend()
            plt.savefig(os.path.join(self.output_dir, f'gps_scatter_{timestamp}.png'))
            plt.close()

        if self.headings:
            # Heading vs Time
            plt.figure(figsize=(10, 6))
            times = [t - self.mag_timestamps[0] for t in self.mag_timestamps]
            plt.plot(times, self.headings, 'r-')
            plt.title('Orientation (Heading) vs Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Heading (Degrees)')
            plt.grid(True)
            plt.savefig(os.path.join(self.output_dir, f'heading_time_{timestamp}.png'))
            plt.close()
            
            # Polar Plot (Orientation)
            plt.figure(figsize=(6, 6))
            ax = plt.subplot(111, projection='polar')
            rads = [math.radians(h) for h in self.headings]
            ax.scatter(rads, [1]*len(rads), c='red', alpha=0.5)
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            plt.title("Heading Distribution")
            plt.savefig(os.path.join(self.output_dir, f'heading_polar_{timestamp}.png'))
            plt.close()

            # Angular Velocity (deg/s)
            if len(self.headings) > 1:
                dt = np.diff(self.mag_timestamps)
                # Handle angle wrap around for diff
                headings_rad = np.radians(self.headings)
                d_heading = np.diff(headings_rad)
                # Wrap
                d_heading = (d_heading + np.pi) % (2 * np.pi) - np.pi
                
                # Avoid div by zero
                dt[dt == 0] = 0.001
                ang_vel = np.degrees(d_heading / dt)
                
                plt.figure(figsize=(10, 4))
                plt.plot(times[1:], ang_vel, 'g-')
                plt.title('Angular Velocity (estimated from Mag)')
                plt.xlabel('Time (s)')
                plt.ylabel('Ang Vel (deg/s)')
                plt.grid(True)
                plt.ylim(-200, 200) # Limit y axis as noise can be huge
                plt.savefig(os.path.join(self.output_dir, f'angular_velocity_{timestamp}.png'))
                plt.close()
            
            saved_any = True

        if saved_any:
            self.get_logger().info('💾 Gráficos actualizados.')

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data() # Guardar al salir
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
