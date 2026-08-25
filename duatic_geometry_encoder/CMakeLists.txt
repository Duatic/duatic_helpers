cmake_minimum_required(VERSION 3.22)
project(duatic_geometry_encoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(duatic_geometry REQUIRED)
find_package(duatic_geometry_msgs REQUIRED)
find_package(duatic_data_encoding REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only: FactoryEncoder<T> specializations mapping duatic_geometry's
# KinematicVariable/KinematicState types to their ROS 2 message counterparts
# (geometry_msgs' Pose/Twist/Accel, and this package's own State/StateStamped).
add_library(${PROJECT_NAME} INTERFACE)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  duatic_geometry_msgs::duatic_geometry_msgs
  duatic_geometry::duatic_geometry
  duatic_data_encoding::duatic_data_encoding
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  rclcpp::rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs duatic_geometry duatic_geometry_msgs duatic_data_encoding rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # only the tests construct TimedData/StampedData directly (to build round-trip
  # fixtures), the library itself only needs the Timed/Stamped traits via
  # duatic_data_encoding -- so this stays a test-only dependency.
  find_package(duatic_data_annotation REQUIRED)

  ament_add_gtest(test_kinematic_variable
    test/test_kinematic_variable.cpp
  )
  target_link_libraries(test_kinematic_variable
    ${PROJECT_NAME}
    duatic_data_annotation::duatic_data_annotation
  )

  ament_add_gtest(test_kinematic_state
    test/test_kinematic_state.cpp
  )
  target_link_libraries(test_kinematic_state
    ${PROJECT_NAME}
    duatic_data_annotation::duatic_data_annotation
  )
endif()

ament_package()
