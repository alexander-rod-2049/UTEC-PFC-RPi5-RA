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
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

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

# 6. Compilar el ejecutable del BT Manager
add_executable(bt_manager src/bt_manager.cpp)
ament_target_dependencies(bt_manager
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  ament_index_cpp
  geometry_msgs
)
# Vinculamos con el plugin para asegurar visibilidad
#target_link_libraries(bt_manager follow_smc_path_plugin)

# 2. Compilar el Plugin del Action Node (SMC+DFL)
add_library(follow_smc_path_plugin SHARED
  src/plugins/follow_smc_path_node.cpp
)
target_include_directories(follow_smc_path_plugin PRIVATE include)
ament_target_dependencies(follow_smc_path_plugin
  behaviortree_cpp
  behaviortree_ros2
  nav2_msgs
  geometry_msgs
)

# 4. Instalación
install(TARGETS
  goto_action
  polar_action
  health_node
  smc_dfl_action
  bt_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 3. Instalar el XML del Behavior Tree
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# 4. Instalar la librería del plugin
install(TARGETS
  follow_smc_path_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 5. Exportar dependencias para el manager
ament_export_libraries(follow_smc_path_plugin)

ament_package()