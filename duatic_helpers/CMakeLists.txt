cmake_minimum_required(VERSION 3.8)

project(duatic_helpers)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(duatic_helpers)



include(cmake/duatic_package.cmake)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  duatic_add_pytest_test(test_rewritten_yaml test/test_rewritten_yaml.py
  )
  duatic_add_pytest_test(test_launch_config_as_bool test/test_launch_config_as_bool.py
  )
endif()



## And the c++ part
find_package(rclcpp REQUIRED)
find_package(duatic_helper_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_state_provider ./src/joint_state_provider_node.cpp)
target_compile_features(joint_state_provider PRIVATE c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(joint_state_provider PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Support ros distros older than lyrical
if(COMMAND ament_target_dependencies)
    ament_target_dependencies(joint_state_provider
      rclcpp
      duatic_helper_msgs
      sensor_msgs
    )
else()
    target_link_libraries(joint_state_provider
      rclcpp::rclcpp
      duatic_helper_msgs::duatic_helper_msgs
      sensor_msgs::sensor_msgs
    )
endif()


install(TARGETS joint_state_provider
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "duatic_helpers-extras.cmake"
)
