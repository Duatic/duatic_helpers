cmake_minimum_required(VERSION 3.22)

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

ament_package(
  CONFIG_EXTRAS "duatic_helpers-extras.cmake"
)
