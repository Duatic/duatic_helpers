cmake_minimum_required(VERSION 3.22)
project(duatic_geometry)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only: the kinematic_variable_3d* stack is all templates over Eigen.
add_library(${PROJECT_NAME} INTERFACE)

target_compile_features(${PROJECT_NAME} INTERFACE c_std_99 cxx_std_20)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
  rclcpp::rclcpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(Eigen3 rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematic_variable_3d_eigen
    test/test_kinematic_variable_3d_eigen.cpp
  )
  target_link_libraries(test_kinematic_variable_3d_eigen
    ${PROJECT_NAME}
  )
endif()

ament_package()
