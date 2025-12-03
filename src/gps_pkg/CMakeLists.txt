cmake_minimum_required(VERSION 3.8)
project(gps_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gps_node src/gps_node.cpp)
ament_target_dependencies(gps_node rclcpp sensor_msgs)

add_executable(magnetometer_node src/magnetometer_node.cpp)
ament_target_dependencies(magnetometer_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  gps_node
  magnetometer_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/visualizer_node.py
  scripts/gps_path_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
