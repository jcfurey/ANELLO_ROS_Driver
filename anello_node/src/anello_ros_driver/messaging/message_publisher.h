/********************************************************************************
 * File Name:   message_publisher.h
 * Description: Header for message_publisher.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef MESSAGE_PUBLISHER_H
#define MESSAGE_PUBLISHER_H

#include "../main_anello_ros_driver.h"
#include "health_message.h"
#include "rclcpp/rclcpp.hpp"

void publish_imu(double *imu, imu_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_im1(double *im1, im1_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_ins(double *ins, ins_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gps(double *gps, gps_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gp2(double *gp2, gps_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_hdr(double *hdg, hdg_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gga(double *gps, gga_pub_t pub, rclcpp::Time time, const std::string &frame_id);
void publish_health(const health_message *health_msg, health_pub_t pub, rclcpp::Time stamp);
void publish_cov(double *cov, apcov_pub_t pub, rclcpp::Time stamp, const std::string &frame_id);

#endif
