# Copyright 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Standard duatic project setup
#
# @public
#
macro(duatic_package)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
      set(CMAKE_CXX_STANDARD 17)
    else()
      message(FATAL_ERROR "cxx_std_17 could not be found.")
    endif()
  endif()

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
  endif()

  option(COVERAGE_ENABLED "Enable code coverage" FALSE)
  if(COVERAGE_ENABLED)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

  # Defaults for Microsoft C++ compiler
  if(MSVC)
    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    add_compile_definitions(
      _USE_MATH_DEFINES
    )
  endif()
endmacro()

# The ros_isolated functions only exist in Kilted and newer releases
option(USE_ISOLATED_TESTS "Enable ros_isolated_test" OFF)

function(duatic_add_test target)
  if(COMMAND ament_add_ros_isolated_test AND USE_ISOLATED_TESTS)
    ament_add_ros_isolated_test(${target} ${ARGN})
  else()
    ament_add_test(${target} ${ARGN})
  endif()
endfunction()

function(duatic_add_gtest target)
  if(COMMAND ament_add_ros_isolated_gtest AND USE_ISOLATED_TESTS)
    ament_add_ros_isolated_gtest(${target} ${ARGN})
  else()
    ament_add_gtest(${target} ${ARGN})
  endif()
endfunction()

function(duatic_add_pytest_test target)
  if(COMMAND ament_add_ros_isolated_pytest_test AND USE_ISOLATED_TESTS)
    ament_add_ros_isolated_pytest_test(${target} ${ARGN})
  else()
    ament_add_pytest_test(${target} ${ARGN})
  endif()
endfunction()

function(_duatic_add_description_test testname)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${testname}.xunit.xml")
  duatic_add_test(${testname}
    COMMAND "${Python3_EXECUTABLE}" -u -m duatic_helpers.test_descriptions --package ${PROJECT_NAME} ${ARGN}
      "--junit-xml=${result_file}" "--junit-prefix=${PROJECT_NAME}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_pytest/${testname}.txt"
    RESULT_FILE "${result_file}"
    TIMEOUT 60
  )
  set_tests_properties(${testname} PROPERTIES LABELS "pytest")
endfunction()

# Test URDF files (one test per file)
function(duatic_add_urdf_tests)
  if(NOT ARGN)
    message(FATAL_ERROR "duatic_add_urdf_tests: no URDF files given")
  endif()
  foreach(urdf IN LISTS ARGN)
    # Add one test per file, to be more granular
    get_filename_component(urdf_stem ${urdf} NAME_WE)
    _duatic_add_description_test(urdf_${PROJECT_NAME}_${urdf_stem} --files ${urdf})
  endforeach()
endfunction()

# Test an SRDF file (against a URDF file)
function(duatic_add_srdf_test srdf urdf)
  if(NOT srdf OR NOT urdf OR ARGN)
    message(FATAL_ERROR "duatic_add_srdf_test: expected exactly 2 arguments (srdf urdf), "
      "got: '${srdf}' '${urdf}' ${ARGN}")
  endif()
  get_filename_component(srdf_stem ${srdf} NAME_WE)
  get_filename_component(urdf_stem ${urdf} NAME_WE)
  _duatic_add_description_test(srdf_${PROJECT_NAME}_${srdf_stem}_${urdf_stem} --files ${srdf} ${urdf})
endfunction()

function(duatic_add_gmock target)
  if(COMMAND ament_add_ros_isolated_gmock AND USE_ISOLATED_TESTS)
    ament_add_ros_isolated_gmock(${target} ${ARGN})
  else()
    ament_add_gmock(${target} ${ARGN})
  endif()
endfunction()
