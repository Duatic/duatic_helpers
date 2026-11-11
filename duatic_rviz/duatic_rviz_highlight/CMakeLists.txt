cmake_minimum_required(VERSION 3.22)
project(duatic_rviz_highlight)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(duatic_rviz_highlight_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

# Parameters of the RViz node (`highlight.enabled`)
generate_parameter_library(
  highlighted_robot_model_display_parameters
  config/display_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/highlighted_robot_model_display.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${duatic_rviz_highlight_msgs_TARGETS}
  highlighted_robot_model_display_parameters
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost)

# The RobotModel display with highlights, loaded by RViz
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME} highlighted_robot_model_display_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(duatic_rviz_highlight_msgs generate_parameter_library pluginlib rclcpp rviz_common rviz_default_plugins)
ament_package()
