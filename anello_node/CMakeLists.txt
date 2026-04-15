cmake_minimum_required(VERSION 3.8)
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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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
  src/anello_ros_driver/messaging/ntrip_buffer.cpp
  src/anello_ros_driver/messaging/rtcm_decoder.cpp
)

set(DRIVER_DEPS
  rclcpp
  rclcpp_components
  anello_interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
  nmea_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
)

# Shared library for component loading
add_library(anello_ros_driver_component SHARED ${DRIVER_SOURCES})
ament_target_dependencies(anello_ros_driver_component ${DRIVER_DEPS})
rclcpp_components_register_node(anello_ros_driver_component
  PLUGIN "anello::AnelloRosDriver"
  EXECUTABLE anello_ros_driver_node
)

install(TARGETS anello_ros_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
