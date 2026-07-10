cmake_minimum_required(VERSION 3.14)
project(anello_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

set(DRIVER_SOURCES
  src/anello_ros_driver/main_anello_ros_driver.cpp
  src/anello_ros_driver/bit_tools.cpp
  src/anello_ros_driver/comm/serial_interface.cpp
  src/anello_ros_driver/comm/ethernet_interface.cpp
  src/anello_ros_driver/comm/anello_config_port.cpp
  src/anello_ros_driver/comm/anello_data_port.cpp
  src/anello_ros_driver/messaging/ascii_decoder.cpp
  src/anello_ros_driver/messaging/health_message.cpp
  src/anello_ros_driver/messaging/message_publisher.cpp
  src/anello_ros_driver/messaging/rtcm_decoder.cpp
)

# Shared library for component loading
add_library(anello_ros_driver_component SHARED ${DRIVER_SOURCES})
target_link_libraries(anello_ros_driver_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${anello_interfaces_TARGETS}
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

install(TARGETS anello_ros_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS anello_ros_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the pure style linters: the codebase predates them and a bulk
  # reformat would obscure history. Functional linters still run.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_anello_decoding
    test/test_bit_tools.cpp
    test/test_ascii_decoder.cpp
    test/test_rtcm_decoder.cpp
    test/test_health_message.cpp
    test/test_clock_translator.cpp
    test/test_serial_reconnect.cpp
  )
  target_link_libraries(test_anello_decoding anello_ros_driver_component)
endif()

ament_package()
