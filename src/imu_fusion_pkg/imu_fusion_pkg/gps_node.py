import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
            self.get_logger().info('GPS Puerto serial /dev/ttyUSB0 abierto exitosamente.')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')
            rclpy.shutdown()
            return

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore')
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.types.talker.GGA) and msg.latitude != 0.0:
                        nav_msg = NavSatFix()
                        nav_msg.header.stamp = self.get_clock().now().to_msg()
                        nav_msg.header.frame_id = 'gps_link'
                        nav_msg.status.status = nav_msg.status.STATUS_FIX
                        nav_msg.status.service = nav_msg.status.SERVICE_GPS
                        
                        nav_msg.latitude = msg.latitude
                        nav_msg.longitude = msg.longitude
                        nav_msg.altitude = msg.altitude
                        
                        nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                        nav_msg.position_covariance[0] = 1.0
                        nav_msg.position_covariance[4] = 1.0
                        nav_msg.position_covariance[8] = 4.0
                        
                        self.publisher_.publish(nav_msg)
                        self.get_logger().info(f'Publicando GPS Fix: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.1f}')

                except pynmea2.ParseError:
                    # self.get_logger().warn(f'Error parseando NMEA: {e} | Linea: "{line.strip()}"')
                    pass

        except Exception as e:
            self.get_logger().error(f'Error leyendo el puerto serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    gps_node = GpsNode()
    if rclpy.ok():
        try:
            rclpy.spin(gps_node)
        except KeyboardInterrupt:
            pass
        finally:
            gps_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
