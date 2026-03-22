cmake_minimum_required(VERSION 3.8)
project(duatic_bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(duatic_skills_msgs REQUIRED)

add_executable(bt_executor_node
  src/bt_executor_node.cpp
)
target_include_directories(bt_executor_node PRIVATE
  include
)
ament_target_dependencies(bt_executor_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  duatic_skills_msgs
)

install(TARGETS bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
