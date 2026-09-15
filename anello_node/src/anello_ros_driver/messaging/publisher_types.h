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

#ifndef ANELLO_ROS_DRIVER__MESSAGING__PUBLISHER_TYPES_H_
#define ANELLO_ROS_DRIVER__MESSAGING__PUBLISHER_TYPES_H_
#include "anello_interfaces/msg/apcov.hpp"
#include "anello_interfaces/msg/apgps.hpp"
#include "anello_interfaces/msg/aphdg.hpp"
#include "anello_interfaces/msg/aphealth.hpp"
#include "anello_interfaces/msg/apim1.hpp"
#include "anello_interfaces/msg/apimu.hpp"
#include "anello_interfaces/msg/apins.hpp"
#include "anello_interfaces/msg/apodo.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
// Publisher type aliases
using imu_pub_t = rclcpp::Publisher<anello_interfaces::msg::APIMU>::SharedPtr;
using im1_pub_t = rclcpp::Publisher<anello_interfaces::msg::APIM1>::SharedPtr;
using ins_pub_t = rclcpp::Publisher<anello_interfaces::msg::APINS>::SharedPtr;
using gps_pub_t = rclcpp::Publisher<anello_interfaces::msg::APGPS>::SharedPtr;
using hdg_pub_t = rclcpp::Publisher<anello_interfaces::msg::APHDG>::SharedPtr;
using health_pub_t = rclcpp::Publisher<anello_interfaces::msg::APHEALTH>::SharedPtr;
using gga_pub_t = rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr;
using apcov_pub_t = rclcpp::Publisher<anello_interfaces::msg::APCOV>::SharedPtr;
using ros_imu_pub_t = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;
using navfix_pub_t = rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr;

#endif  // ANELLO_ROS_DRIVER__MESSAGING__PUBLISHER_TYPES_H_
