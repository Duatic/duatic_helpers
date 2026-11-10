cmake_minimum_required(VERSION 3.22)
project(duatic_rviz_highlight_joint_limits)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(duatic_rviz_highlight REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

# Parameters of the highlight
generate_parameter_library(
  joint_limits_highlight_parameters
  config/highlight_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/joint_limits_highlight.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  angles::angles
  duatic_rviz_highlight::duatic_rviz_highlight
  joint_limits_highlight_parameters
  pluginlib::pluginlib
  rclcpp::rclcpp
  urdf::urdf
  ${sensor_msgs_TARGETS}
)

# Makes the highlight known to duatic_rviz_highlight (e.g. `highlight.enabled:="[joint_limits]"`)
pluginlib_export_plugin_description_file(duatic_rviz_highlight plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME} joint_limits_highlight_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits ${PROJECT_NAME})

  ament_add_gtest(test_moving_parts test/test_moving_parts.cpp)
  target_link_libraries(test_moving_parts ${PROJECT_NAME})

  ament_add_gtest(test_highlight test/test_highlight.cpp)
  target_link_libraries(test_highlight ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(angles duatic_rviz_highlight generate_parameter_library pluginlib rclcpp sensor_msgs urdf)
ament_package()
