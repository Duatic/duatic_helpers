cmake_minimum_required(VERSION 3.22)
project(duatic_rviz_highlight_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HighlightInfo.msg"
  "srv/ListHighlights.srv"
)

ament_package()
