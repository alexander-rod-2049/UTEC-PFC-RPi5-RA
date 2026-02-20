cmake_minimum_required(VERSION 3.8)
project(pfc_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pfc_msgs REQUIRED)
find_package(ros2_roboclaw_driver REQUIRED)

# 2. Declarar el ejecutable
add_executable(goto_action src/goto_action.cpp)


# 3. Vincular TODO usando la función de ROS
# Esto se encarga de los headers y las librerías automáticamente sin pelearse con CMake
ament_target_dependencies(goto_action
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  pfc_msgs
)

add_executable(polar_action src/polar_action.cpp)

ament_target_dependencies(polar_action
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  pfc_msgs
)

add_executable(health_node src/health_node.cpp)
ament_target_dependencies(health_node
  rclcpp
  nav_msgs
  sensor_msgs
  pfc_msgs
  ros2_roboclaw_driver
)

add_executable(smc_dfl_action src/smc_dfl_action.cpp)
ament_target_dependencies(smc_dfl_action
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs tf2 tf2_geometry_msgs pfc_msgs
)

# 4. Instalación
install(TARGETS
  goto_action
  polar_action
  health_node
  smc_dfl_action
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()