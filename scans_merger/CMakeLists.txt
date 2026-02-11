cmake_minimum_required(VERSION 3.8)
project(scans_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(scans_merger src/scans_merger.cpp)

ament_target_dependencies(scans_merger
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  tf2_ros
  tf2
  Eigen3
)

install(TARGETS scans_merger DESTINATION lib/${PROJECT_NAME})

ament_package()
