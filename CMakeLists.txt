cmake_minimum_required(VERSION 3.8)
project(duatic_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  config launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/param_loader_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
