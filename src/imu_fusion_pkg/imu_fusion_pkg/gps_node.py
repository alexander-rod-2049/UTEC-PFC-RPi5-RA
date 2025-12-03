import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2
import glob
import time
from collections import deque

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Buffer para filtro de media móvil
        self.buffer_size = 20  # Ajustable: mayor número = más suavizado pero más retardo
        self.lat_buffer = deque(maxlen=self.buffer_size)
        self.lon_buffer = deque(maxlen=self.buffer_size)
        self.alt_buffer = deque(maxlen=self.buffer_size)
        
        # Conectar automáticamente al GPS
        self.serial_port = self.find_and_connect_gps()
        if self.serial_port:
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.get_logger().info('🚀 Nodo GPS iniciado correctamente')
        else:
            self.get_logger().error('❌ No se pudo conectar a ningún GPS')
            self.serial_port = None

    def find_and_connect_gps(self):
        """Encuentra y conecta automáticamente al GPS en cualquier puerto"""
        possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        self.get_logger().info(f'🔍 Buscando GPS en puertos: {possible_ports}')
        
        for port in possible_ports:
            try:
                self.get_logger().info(f'🔄 Probando puerto: {port}')
                ser = serial.Serial(port, baudrate=9600, timeout=1)
                
                # Leer varias líneas para verificar si es un GPS
                for _ in range(10):
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line and ('GNGGA' in line or 'GPGGA' in line or 'GNRMC' in line):
                            self.get_logger().info(f'✅ GPS encontrado en: {port}')
                            self.get_logger().info(f'📡 Primer mensaje: {line}')
                            return ser
                    time.sleep(0.1)  # Pequeña pausa entre lecturas
                
                ser.close()
                self.get_logger().info(f'❌ {port} no parece ser un GPS')
                
            except serial.SerialException as e:
                if "Device or resource busy" in str(e):
                    self.get_logger().info(f'⏸️  {port} está ocupado, probando siguiente...')
                else:
                    self.get_logger().info(f'❌ No se pudo abrir {port}: {e}')
                continue
            except Exception as e:
                self.get_logger().info(f'❌ Error con {port}: {e}')
                continue
        
        return None

    def timer_callback(self):
        if self.serial_port is None:
            # Intentar reconectar cada 5 segundos
            self.serial_port = self.find_and_connect_gps()
            return
            
        try:
            # Leer toda la data disponible
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    self.get_logger().info(f'📨 RAW: {line}')
                    
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        try:
                            msg = pynmea2.parse(line)
                            self.get_logger().info(f'✅ NMEA parsed: {msg}')
                            
                            # Verificar si tiene fix
                            if hasattr(msg, 'gps_qual') and msg.gps_qual > 0:
                                nav_msg = NavSatFix()
                                nav_msg.header.stamp = self.get_clock().now().to_msg()
                                nav_msg.header.frame_id = 'gps'
                                nav_msg.status.status = 1  # FIX
                                nav_msg.status.service = 1  # GPS
                                
                                # Añadir al buffer de filtrado
                                self.lat_buffer.append(msg.latitude)
                                self.lon_buffer.append(msg.longitude)
                                self.alt_buffer.append(msg.altitude)
                                
                                # Calcular promedios
                                avg_lat = sum(self.lat_buffer) / len(self.lat_buffer)
                                avg_lon = sum(self.lon_buffer) / len(self.lon_buffer)
                                avg_alt = sum(self.alt_buffer) / len(self.alt_buffer)
                                
                                nav_msg.latitude = avg_lat
                                nav_msg.longitude = avg_lon
                                nav_msg.altitude = avg_alt
                                
                                self.publisher_.publish(nav_msg)
                                self.get_logger().info(f'🎯 FILTERED: Lat={avg_lat:.6f}, Lon={avg_lon:.6f}, Alt={avg_alt:.1f}m (Sats: {len(self.lat_buffer)})')
                            else:
                                # Mostrar información de satélites
                                sat_info = ""
                                if hasattr(msg, 'num_sats'):
                                    sat_info = f" - Satélites: {msg.num_sats}"
                                self.get_logger().warn(f'⚠️  GPS sin fix satelital{sat_info}')
                                
                        except pynmea2.ParseError as e:
                            self.get_logger().warn(f'❌ Parse error en línea: {line}')
                        except Exception as e:
                            self.get_logger().error(f'� Error procesando: {e}')
                    
                    # También mostrar mensajes de satélites para diagnóstico
                    elif line.startswith('$GPGSV') or line.startswith('$GLGSV'):
                        parts = line.split(',')
                        if len(parts) > 3 and parts[3].isdigit():
                            satellites = int(parts[3])
                            if satellites > 0:
                                self.get_logger().info(f'�🛰️  Satélites visibles: {satellites}')

        except Exception as e:
            self.get_logger().error(f'💥 Error lectura: {e}')
            # Intentar reconectar si hay error de comunicación
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None


def main(args=None):
    rclpy.init(args=args)
    gps_node = GpsNode()
    
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        gps_node.get_logger().info('👋 Apagando nodo GPS...')
    except Exception as e:
        gps_node.get_logger().error(f'💥 Error en el nodo: {e}')
    finally:
        # Cerrar el puerto serial si está abierto
        if gps_node.serial_port:
            try:
                gps_node.serial_port.close()
                gps_node.get_logger().info('🔌 Puerto serial cerrado')
            except:
                pass
        
        # Destruir el nodo
        gps_node.destroy_node()
        
        # Shutdown seguro
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
