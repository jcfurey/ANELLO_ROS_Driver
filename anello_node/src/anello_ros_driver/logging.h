#ifndef ANELLO_LOGGING_H
#define ANELLO_LOGGING_H
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#define DEBUG_PRINT(...) RCLCPP_DEBUG(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define ERROR_PRINT(...) RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define WARNING_PRINT(...) RCLCPP_WARN(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)

#endif
