cmake_minimum_required(VERSION 3.8)
project(pfc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar las interfaces (mensajes)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemStatus.msg"
)

ament_package()