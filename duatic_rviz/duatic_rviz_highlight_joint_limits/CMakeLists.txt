cmake_minimum_required(VERSION 3.22)
project(duatic_rviz_highlight_joint_limits)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(duatic_rviz_highlight REQUIRED)
find_package(urdf REQUIRED)

# Header-only for now: the joint limit and moving parts helpers
add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  angles::angles
  duatic_rviz_highlight::duatic_rviz_highlight
  urdf::urdf
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits ${PROJECT_NAME})

  ament_add_gtest(test_moving_parts test/test_moving_parts.cpp)
  target_link_libraries(test_moving_parts ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(angles duatic_rviz_highlight urdf)
ament_package()
