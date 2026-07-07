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

void publish_imu(const double *imu, const imu_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_im1(const double *im1, const im1_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_ins(const double *ins, const ins_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_gps(const double *gps, const gps_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
void publish_hdr(const double *hdg, const hdg_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);
/* Format a decoded APGPS value array as a "$GNGGA,...*CK\r\n" NMEA
 * sentence (the payload published by publish_gga). */
std::string build_gga_sentence(const double *gps);
void publish_gga(const double *gps, const gga_pub_t &pub, rclcpp::Time time, const std::string &frame_id);
void publish_health(const health_message *health_msg, const health_pub_t &pub, rclcpp::Time stamp);
void publish_cov(const double *cov, const apcov_pub_t &pub, rclcpp::Time stamp, const std::string &frame_id);

#endif
