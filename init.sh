#!/bin/bash
# init.sh - Inicializador completo del workspace ROS 2

echo "=== Inicializando workspace PFCII ==="

# Verificar y sourcear ROS 2 si es necesario
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 no está sourceado, sourceando jazzy..."
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS 2 $ROS_DISTRO sourceado"
else
    echo "✓ ROS 2 $ROS_DISTRO ya está sourceado"
fi

# Sourcear el workspace
echo "Sourceando el workspace PFCII..."
source ~/PFCII/pfc2_ws/install/setup.bash
echo "✓ Workspace sourceado"

# Verificación completa
echo ""
echo "=== VERIFICACIÓN ==="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"

# Verificar el workspace de diferentes formas
echo "Workspace verificado mediante:"
echo "1. COLCON_PREFIX_PATH: $COLCON_PREFIX_PATH" | grep pfc2_ws
echo "2. AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH" | grep pfc2_ws

# Verificar paquetes disponibles
echo "3. Paquetes en el workspace:"
colcon list --names-only 2>/dev/null || echo " - (Usa 'colcon list' para ver los paquetes)"

#echo "Es posible que se deba correr: sudo rosdep init y luego rosdep update."
#echo "Sólo si sale el error: ERROR: your rosdep installation has not been initialized yet."

echo "Comandos a usar:"
echo "ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py"
echo "ros2 topic list"
echo "ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
echo "ls -l /dev/ttyACM*"
#echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.15"

echo ""
echo "Inicialización completada. Workspace listo para usar."

