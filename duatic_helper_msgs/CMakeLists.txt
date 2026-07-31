cmake_minimum_required(VERSION 3.22)
project(duatic_helper_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetJointStates.srv"
  # Scene manipulation for duatic_gazebo's sim_scene node. Here rather than in
  # duatic_gazebo so that callers depend on the interfaces alone — the simulated clamp
  # driver needs AttachModel to have a load welded to it, and an end-effector package
  # has no business pulling in a whole simulation to get two service definitions.
  "srv/AttachModel.srv"
  "srv/DetachModel.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
