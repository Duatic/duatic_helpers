cmake_minimum_required(VERSION 3.22)
project(duatic_trajectory)

find_package(ament_cmake REQUIRED)
find_package(duatic_geometry REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/trajectory.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  duatic_geometry::duatic_geometry
  rclcpp::rclcpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(duatic_geometry rclcpp)

ament_package()
