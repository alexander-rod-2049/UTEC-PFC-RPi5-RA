cmake_minimum_required(VERSION 3.8)
project(mtf02p_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(mtf02p_odometry_node src/mtf02p_odometry_node.cpp)

ament_target_dependencies(mtf02p_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS mtf02p_odometry_node DESTINATION lib/${PROJECT_NAME})

ament_package()