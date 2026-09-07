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

#include "publisher_types.h"
#include "health_message.h"
#include "rclcpp/rclcpp.hpp"

void publish_imu(double *imu, const imu_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_im1(double *im1, const im1_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_ins(double *ins, const ins_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gps(double *gps, const gps_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gp2(double *gp2, const gps_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_hdr(double *hdg, const hdg_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gga(double *gps, const gga_pub_t &pub, rclcpp::Time time, const std::string &frame_id, int leap_seconds=18);
void publish_health(const health_message *health_msg, const health_pub_t &pub, rclcpp::Time stamp);
void publish_cov(double *cov, const apcov_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);

#endif
