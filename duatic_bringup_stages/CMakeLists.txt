cmake_minimum_required(VERSION 3.22)
project(duatic_bringup_stages)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/stage_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_pytest_test(test_stages test/test_stages.py TIMEOUT 120)
  # Runs a controller manager, isolated so that it cannot reach other ROS nodes
  ament_add_ros_isolated_pytest_test(test_stage_gate test/test_stage_gate.py TIMEOUT 120)
endif()

ament_package()
