#!/usr/bin/env python3
import smbus2
import time

class DualMPU6050:
    def __init__(self, bus_number=1):
        self.bus = smbus2.SMBus(bus_number)
        
        # Direcciones I2C de los dos sensores
        self.MPU6050_ADDR_1 = 0x68  # AD0 a GND
        self.MPU6050_ADDR_2 = 0x69  # AD0 a 3.3V
        
        # Registros MPU6050
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        
        # Offsets de calibración para cada sensor
        # Giroscopio
        self.gyro_offset_1 = [0.0, 0.0, 0.0]
        self.gyro_offset_2 = [0.0, 0.0, 0.0]
        # Acelerómetro  
        self.accel_offset_1 = [0.0, 0.0, 0.0]
        self.accel_offset_2 = [0.0, 0.0, 0.0]
        # Escala de gravedad (debería ser 1.0 para Z en reposo)
        self.accel_scale_1 = [1.0, 1.0, 1.0]
        self.accel_scale_2 = [1.0, 1.0, 1.0]
        
        # Configurar ambos sensores
        self._setup_sensors()
        
        # Calibrar automáticamente al iniciar
        self.calibrate_sensors()
        
    def _setup_sensors(self):
        """Configurar ambos sensores MPU6050"""
        try:
            # Despertar ambos sensores
            self.bus.write_byte_data(self.MPU6050_ADDR_1, self.PWR_MGMT_1, 0)
            self.bus.write_byte_data(self.MPU6050_ADDR_2, self.PWR_MGMT_1, 0)
            time.sleep(0.1)
            print("✅ Ambos MPU6050 configurados correctamente")
            return True
        except Exception as e:
            print(f"❌ Error configurando sensores: {e}")
            return False
    
    def _read_raw_data(self, address, reg):
        """Leer datos raw del sensor"""
        try:
            high = self.bus.read_byte_data(address, reg)
            low = self.bus.read_byte_data(address, reg + 1)
            value = (high << 8) | low
            if value > 32768:
                value = value - 65536
            return value
        except Exception as e:
            print(f"❌ Error leyendo sensor {hex(address)}: {e}")
            return 0
    
    def calibrate_sensors(self, samples=300):
        """Calibrar automáticamente ambos sensores (acelerómetro + giroscopio)"""
        print("🔧 CALIBRACIÓN AUTOMÁTICA INICIADA")
        print("📝 Coloca los sensores en una superficie PLANA y QUIETA")
        print(f"📊 Tomando {samples} muestras...")
        time.sleep(2)  # Dar tiempo para colocar los sensores
        
        # Inicializar acumuladores
        gyro_sum_1 = [0.0, 0.0, 0.0]
        gyro_sum_2 = [0.0, 0.0, 0.0]
        accel_sum_1 = [0.0, 0.0, 0.0]
        accel_sum_2 = [0.0, 0.0, 0.0]
        
        for i in range(samples):
            # Leer datos raw de ambos sensores
            # IMU1
            accel_x1 = self._read_raw_data(self.MPU6050_ADDR_1, self.ACCEL_XOUT_H) / 16384.0
            accel_y1 = self._read_raw_data(self.MPU6050_ADDR_1, self.ACCEL_XOUT_H + 2) / 16384.0
            accel_z1 = self._read_raw_data(self.MPU6050_ADDR_1, self.ACCEL_XOUT_H + 4) / 16384.0
            gyro_x1 = self._read_raw_data(self.MPU6050_ADDR_1, self.GYRO_XOUT_H) / 131.0
            gyro_y1 = self._read_raw_data(self.MPU6050_ADDR_1, self.GYRO_XOUT_H + 2) / 131.0
            gyro_z1 = self._read_raw_data(self.MPU6050_ADDR_1, self.GYRO_XOUT_H + 4) / 131.0
            
            # IMU2
            accel_x2 = self._read_raw_data(self.MPU6050_ADDR_2, self.ACCEL_XOUT_H) / 16384.0
            accel_y2 = self._read_raw_data(self.MPU6050_ADDR_2, self.ACCEL_XOUT_H + 2) / 16384.0
            accel_z2 = self._read_raw_data(self.MPU6050_ADDR_2, self.ACCEL_XOUT_H + 4) / 16384.0
            gyro_x2 = self._read_raw_data(self.MPU6050_ADDR_2, self.GYRO_XOUT_H) / 131.0
            gyro_y2 = self._read_raw_data(self.MPU6050_ADDR_2, self.GYRO_XOUT_H + 2) / 131.0
            gyro_z2 = self._read_raw_data(self.MPU6050_ADDR_2, self.GYRO_XOUT_H + 4) / 131.0
            
            # Acumular para promediar
            accel_sum_1[0] += accel_x1
            accel_sum_1[1] += accel_y1
            accel_sum_1[2] += accel_z1
            gyro_sum_1[0] += gyro_x1
            gyro_sum_1[1] += gyro_y1
            gyro_sum_1[2] += gyro_z1
            
            accel_sum_2[0] += accel_x2
            accel_sum_2[1] += accel_y2
            accel_sum_2[2] += accel_z2
            gyro_sum_2[0] += gyro_x2
            gyro_sum_2[1] += gyro_y2
            gyro_sum_2[2] += gyro_z2
            
            if i % 50 == 0:
                print(f"🔄 Calibrando... {i}/{samples}")
            time.sleep(0.01)
        
        # Calcular offsets y escalas
        # Giroscopio: el promedio debería ser 0 en reposo
        self.gyro_offset_1 = [gyro_sum_1[0] / samples, gyro_sum_1[1] / samples, gyro_sum_1[2] / samples]
        self.gyro_offset_2 = [gyro_sum_2[0] / samples, gyro_sum_2[1] / samples, gyro_sum_2[2] / samples]
        
        # Acelerómetro: 
        # - X e Y deberían promediar 0 (nivelado)
        # - Z debería promediar 1.0g (gravedad)
        accel_avg_1 = [accel_sum_1[0] / samples, accel_sum_1[1] / samples, accel_sum_1[2] / samples]
        accel_avg_2 = [accel_sum_2[0] / samples, accel_sum_2[1] / samples, accel_sum_2[2] / samples]
        
        # Offset: corregir el bias
        self.accel_offset_1 = [accel_avg_1[0], accel_avg_1[1], accel_avg_1[2] - 1.0]  # Z debería ser 1.0g
        self.accel_offset_2 = [accel_avg_2[0], accel_avg_2[1], accel_avg_2[2] - 1.0]  # Z debería ser 1.0g
        
        print("✅ CALIBRACIÓN COMPLETADA:")
        print(f"   IMU1 - Gyro Offset: [{self.gyro_offset_1[0]:.3f}, {self.gyro_offset_1[1]:.3f}, {self.gyro_offset_1[2]:.3f}] °/s")
        print(f"   IMU1 - Accel Offset: [{self.accel_offset_1[0]:.3f}, {self.accel_offset_1[1]:.3f}, {self.accel_offset_1[2]:.3f}] g")
        print(f"   IMU2 - Gyro Offset: [{self.gyro_offset_2[0]:.3f}, {self.gyro_offset_2[1]:.3f}, {self.gyro_offset_2[2]:.3f}] °/s")
        print(f"   IMU2 - Accel Offset: [{self.accel_offset_2[0]:.3f}, {self.accel_offset_2[1]:.3f}, {self.accel_offset_2[2]:.3f}] g")
    
    def read_sensor_data(self, address, sensor_name):
        """Leer todos los datos de un sensor específico (CALIBRADOS)"""
        # Leer datos raw
        accel_x_raw = self._read_raw_data(address, self.ACCEL_XOUT_H) / 16384.0
        accel_y_raw = self._read_raw_data(address, self.ACCEL_XOUT_H + 2) / 16384.0
        accel_z_raw = self._read_raw_data(address, self.ACCEL_XOUT_H + 4) / 16384.0
        gyro_x_raw = self._read_raw_data(address, self.GYRO_XOUT_H) / 131.0
        gyro_y_raw = self._read_raw_data(address, self.GYRO_XOUT_H + 2) / 131.0
        gyro_z_raw = self._read_raw_data(address, self.GYRO_XOUT_H + 4) / 131.0
        
        # Aplicar calibración
        if address == self.MPU6050_ADDR_1:
            # Acelerómetro calibrado
            accel_x = accel_x_raw - self.accel_offset_1[0]
            accel_y = accel_y_raw - self.accel_offset_1[1]
            accel_z = accel_z_raw - self.accel_offset_1[2]
            # Giroscopio calibrado
            gyro_x = gyro_x_raw - self.gyro_offset_1[0]
            gyro_y = gyro_y_raw - self.gyro_offset_1[1]
            gyro_z = gyro_z_raw - self.gyro_offset_1[2]
        else:  # IMU2
            # Acelerómetro calibrado
            accel_x = accel_x_raw - self.accel_offset_2[0]
            accel_y = accel_y_raw - self.accel_offset_2[1]
            accel_z = accel_z_raw - self.accel_offset_2[2]
            # Giroscopio calibrado
            gyro_x = gyro_x_raw - self.gyro_offset_2[0]
            gyro_y = gyro_y_raw - self.gyro_offset_2[1]
            gyro_z = gyro_z_raw - self.gyro_offset_2[2]
        
        return {
            'sensor_name': sensor_name,
            'address': address,
            'accel': [accel_x, accel_y, accel_z],
            'gyro': [gyro_x, gyro_y, gyro_z]
        }
    
    def read_both_imus(self):
        """Leer datos de ambos IMUs (calibrados)"""
        sensor1_data = self.read_sensor_data(self.MPU6050_ADDR_1, "IMU_1")
        sensor2_data = self.read_sensor_data(self.MPU6050_ADDR_2, "IMU_2")
        
        return sensor1_data, sensor2_data
    
    def close(self):
        """Cerrar conexión I2C"""
        self.bus.close()
