cmake_minimum_required(VERSION 3.8)
project(imu_fusion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Driver
add_executable(multi_imu_driver src/multi_imu_driver.cpp)
ament_target_dependencies(multi_imu_driver rclcpp sensor_msgs)

# Procesador
add_executable(lateral_imu_processor src/lateral_imu_processor.cpp)
ament_target_dependencies(lateral_imu_processor rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Estimador
add_executable(slip_estimator_node src/slip_estimator_node.cpp)
ament_target_dependencies(slip_estimator_node rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS multi_imu_driver lateral_imu_processor slip_estimator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()