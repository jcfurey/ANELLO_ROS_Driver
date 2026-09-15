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

/********************************************************************************
 * File Name:   message_publisher.h
 * Description: Header for message_publisher.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__MESSAGING__MESSAGE_PUBLISHER_H_
#define ANELLO_ROS_DRIVER__MESSAGING__MESSAGE_PUBLISHER_H_

#include <string>

#include "anello_ros_driver/messaging/health_message.h"
#include "anello_ros_driver/messaging/publisher_types.h"
#include "rclcpp/rclcpp.hpp"
void publish_imu(
  double *imu, const imu_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_im1(
  double *im1, const im1_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_ins(
  double *ins, const ins_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_gps(
  double *gps, const gps_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_gp2(
  double *gp2, const gps_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_hdr(
  double *hdg, const hdg_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);
void publish_gga(
  double *gps, const gga_pub_t & pub, rclcpp::Time time,
  const std::string & frame_id, int leap_seconds = 18);
void publish_health(const health_message *health_msg, const health_pub_t & pub, rclcpp::Time stamp);
void publish_cov(
  double *cov, const apcov_pub_t & pub, rclcpp::Time stamp,
  const std::string & frame_id);

#endif  // ANELLO_ROS_DRIVER__MESSAGING__MESSAGE_PUBLISHER_H_
