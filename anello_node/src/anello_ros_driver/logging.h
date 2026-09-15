// Copyright (c) 2023 ANELLO Photonics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ANELLO_ROS_DRIVER__LOGGING_H_
#define ANELLO_ROS_DRIVER__LOGGING_H_
#include "rclcpp/logging.hpp"

#include "rclcpp/logger.hpp"
#define DEBUG_PRINT(...) RCLCPP_DEBUG(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define ERROR_PRINT(...) RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define WARNING_PRINT(...) RCLCPP_WARN(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)

#endif  // ANELLO_ROS_DRIVER__LOGGING_H_
