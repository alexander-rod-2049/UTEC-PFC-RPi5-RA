# Estado de los sensores (True = OK, False = Fallo/Timeout)
bool imu_ok
bool motors_ok
bool odom_ok
bool optical_flow_ok

# Datos de energía
float32 battery_voltage

# Mensaje de diagnóstico humano
string status_message