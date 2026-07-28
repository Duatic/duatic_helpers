cmake_minimum_required(VERSION 3.22)
project(duatic_geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(duatic_geometry REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/StateStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Header-only conversion library: Factory<T> converts between duatic_geometry
# kinematic types and their ROS 2 message counterparts (this package's own
# State/StateStamped, plus the standard geometry_msgs Pose/Twist/Accel).
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_factory INTERFACE)

target_compile_features(${PROJECT_NAME}_factory INTERFACE cxx_std_20)

target_include_directories(${PROJECT_NAME}_factory INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_factory INTERFACE
  "${cpp_typesupport_target}"
  duatic_geometry::duatic_geometry
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  rclcpp::rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_factory
  EXPORT export_${PROJECT_NAME}_factory
)

ament_export_targets(export_${PROJECT_NAME}_factory HAS_LIBRARY_TARGET)
ament_export_dependencies(duatic_geometry geometry_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematic_variable
    test/test_kinematic_variable.cpp
  )
  target_link_libraries(test_kinematic_variable
    ${PROJECT_NAME}_factory
  )
endif()

ament_package()
