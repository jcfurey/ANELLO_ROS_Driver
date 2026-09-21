# Copyright (c) 2023 ANELLO Photonics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.14)
project(anello_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(anello_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Capture the configured source identity in the running node, rather than
# guessing its revision from the export client's checkout.
find_package(Git QUIET)
set(ANELLO_BUILD_REVISION "unknown")
if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE ANELLO_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(ANELLO_GIT_HEAD MATCHES "^[0-9a-f]+$")
    set(ANELLO_BUILD_REVISION "${ANELLO_GIT_HEAD}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      OUTPUT_VARIABLE ANELLO_GIT_CHANGES ERROR_QUIET)
    if(ANELLO_GIT_CHANGES)
      string(APPEND ANELLO_BUILD_REVISION "-dirty")
    endif()
  endif()
endif()
file(GLOB_RECURSE ANELLO_BUILD_INPUTS CONFIGURE_DEPENDS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "src/*.cpp" "src/*.h")
list(APPEND ANELLO_BUILD_INPUTS CMakeLists.txt package.xml cmake/build_info.h.in)
list(SORT ANELLO_BUILD_INPUTS)
set(ANELLO_BUILD_HASHES "")
foreach(input IN LISTS ANELLO_BUILD_INPUTS)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/${input}")
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${input}" input_hash)
  string(APPEND ANELLO_BUILD_HASHES "${input}:${input_hash}\n")
endforeach()
string(SHA256 ANELLO_BUILD_SOURCE_SHA256 "${ANELLO_BUILD_HASHES}")
configure_file(cmake/build_info.h.in
  generated/anello_ros_driver/build_info.h @ONLY)

set(DRIVER_SOURCES
  src/anello_ros_driver/main_anello_ros_driver.cpp
  src/anello_ros_driver/bit_tools.cpp
  src/anello_ros_driver/comm/serial_interface.cpp
  src/anello_ros_driver/comm/ethernet_interface.cpp
  src/anello_ros_driver/comm/anello_config_port.cpp
  src/anello_ros_driver/comm/anello_data_port.cpp
  src/anello_ros_driver/messaging/protocol_decoder.cpp
  src/anello_ros_driver/messaging/health_message.cpp
  src/anello_ros_driver/messaging/message_publisher.cpp
  src/anello_ros_driver/messaging/rtcm_decoder.cpp
)

# Shared library for component loading
add_library(anello_ros_driver_component SHARED ${DRIVER_SOURCES})
target_include_directories(anello_ros_driver_component PRIVATE src
  "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(anello_ros_driver_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${anello_interfaces_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nmea_msgs_TARGETS}
  ${rtcm_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  diagnostic_updater::diagnostic_updater
)
# Register for component loading without generating an executable (the
# plural macro); the standalone executable below provides its own main().
rclcpp_components_register_nodes(anello_ros_driver_component
  "anello::AnelloRosDriver"
)

# Standalone executable with a custom main() that spins under a
# MultiThreadedExecutor (the generated component executable would use a
# SingleThreadedExecutor, which cannot run the config callback group in
# parallel with the data poll). The node stays loadable as a component too.
add_executable(anello_ros_driver_node
  src/anello_ros_driver/standalone_main.cpp
)
target_link_libraries(anello_ros_driver_node
  anello_ros_driver_component
  rclcpp::rclcpp
)
target_include_directories(anello_ros_driver_node PRIVATE src)

install(TARGETS anello_ros_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS anello_ros_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/anello_config.py
  DESTINATION lib/${PROJECT_NAME} RENAME anello_config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_LANGUAGE c++)
  set(ament_cmake_uncrustify_LANGUAGE C++)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ros_regressions test/test_ros_regressions.py TIMEOUT 120)
  ament_add_pytest_test(test_config_export test/test_config_export.py TIMEOUT 30)
  ament_add_pytest_test(test_config_export_ros test/test_config_export_ros.py TIMEOUT 120)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_anello_decoding
    test/test_protocol_regressions.cpp
    test/test_bit_tools.cpp
    test/test_ascii_decoder.cpp
    test/test_rtcm_decoder.cpp
    test/test_health_message.cpp
    test/test_clock_translator.cpp
    test/test_serial_reconnect.cpp
  )
  target_link_libraries(test_anello_decoding anello_ros_driver_component)
  target_include_directories(test_anello_decoding PRIVATE src)
endif()

ament_package()

if(BUILD_TESTING AND TEST cppcheck)
  # Perform the registered check even for versions ament skips by default.
  # CTest retains the check's 300-second timeout.
  set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT
    "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
endif()
