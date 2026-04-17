/********************************************************************************
 * File Name:   message_publisher.cpp
 * Description: Contains functions for publishing messages to ROS topics.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#include "message_publisher.h"
#include "../main_anello_ros_driver.h"
#include "health_message.h"
#include "../bit_tools.h"

#include <cstring>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "anello_interfaces/msg/apimu.hpp"
#include "anello_interfaces/msg/apim1.hpp"
#include "anello_interfaces/msg/apins.hpp"
#include "anello_interfaces/msg/apgps.hpp"
#include "anello_interfaces/msg/aphdg.hpp"
#include "anello_interfaces/msg/aphealth.hpp"
#include "anello_interfaces/msg/apcov.hpp"

#include <nmea_msgs/msg/sentence.hpp>
#include <mavros_msgs/msg/rtcm.hpp>
#include <std_msgs/msg/header.hpp>

void publish_gga(double *gps, gga_pub_t pub, rclcpp::Time time, const std::string &frame_id)
{
    std_msgs::msg::Header msg_header;
    nmea_msgs::msg::Sentence gga_message;

    msg_header.frame_id = frame_id;
    msg_header.stamp = time;

    std::ostringstream gngga_message;
    gngga_message << "$GNGGA,";

    double gps_seconds = gps[1] * 1e-9;
    // GPS-UTC offset: 18 leap seconds as of 2017
    time_t utc_time_s = static_cast<time_t>(gps_seconds - 18.0);
    struct tm *utc_info = std::gmtime(&utc_time_s);

    int utc_hours = utc_info->tm_hour;
    int utc_minutes = utc_info->tm_min;
    int utc_seconds_int = utc_info->tm_sec;
    int utc_milliseconds = static_cast<int>(floor((gps_seconds - floor(gps_seconds)) * 1e3));

    gngga_message << std::setw(2) << std::setfill('0') << utc_hours
                  << std::setw(2) << std::setfill('0') << utc_minutes
                  << std::setw(2) << std::setfill('0') << utc_seconds_int
                  << "."
                  << std::setw(3) << std::setfill('0') << utc_milliseconds << ",";

    double lat_float = gps[2];
    char lat_hemisphere = (lat_float >= 0) ? 'N' : 'S';
    int lat_deg = static_cast<int>(floor(fabs(lat_float)));
    double lat_min = 60.0 * (fabs(lat_float) - lat_deg);
    int lat_min_int = static_cast<int>(floor(lat_min));
    int lat_min_dec = static_cast<int>(floor((lat_min - lat_min_int) * 1e7));

    gngga_message << std::setw(2) << std::setfill('0') << lat_deg
                  << std::setw(2) << std::setfill('0') << lat_min_int
                  << "."
                  << std::setw(7) << std::setfill('0') << lat_min_dec
                  << ",";
    gngga_message << lat_hemisphere << ",";

    double lon_float = gps[3];
    char lon_hemisphere = (lon_float >= 0) ? 'E' : 'W';
    int lon_deg = static_cast<int>(floor(fabs(lon_float)));
    double lon_min = 60.0 * (fabs(lon_float) - lon_deg);
    int lon_min_int = static_cast<int>(floor(lon_min));
    int lon_min_dec = static_cast<int>(floor((lon_min - lon_min_int) * 1e7));

    gngga_message << std::setw(3) << std::setfill('0') << lon_deg
                  << std::setw(2) << std::setfill('0') << lon_min_int
                  << "."
                  << std::setw(7) << std::setfill('0') << lon_min_dec
                  << ",";
    gngga_message << lon_hemisphere << ",";

    int value_map[3] = {1, 5, 4};
    int rtk_fix_quality = static_cast<int>(gps[15]);
    if (rtk_fix_quality >= 0 && rtk_fix_quality < 3)
        gngga_message << value_map[rtk_fix_quality] << ",";
    else
        gngga_message << "0,";

    gngga_message << static_cast<int>(gps[12]) << ",";
    gngga_message << std::setw(4) << std::setfill('0') << gps[10] << ",";
    gngga_message << gps[5] << ",M,";
    gngga_message << ",M,";
    gngga_message << ",";
    gngga_message << "";

    std::string ck = compute_checksum(gngga_message.str().c_str() + 1, gngga_message.str().length() - 1);
    gngga_message << "*" << ck << "\r\n";

    gga_message.header = msg_header;
    gga_message.sentence = gngga_message.str();

    pub->publish(gga_message);
}

void publish_gps(double *gps, gps_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APGPS msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = gps[0];
    msg.gps_time = gps[1];
    msg.lat = gps[2];
    msg.lon = gps[3];
    msg.alt_ellipsoid = gps[4];
    msg.alt_msl = gps[5];
    msg.speed = gps[6];
    msg.heading = gps[7];
    msg.hacc = gps[8];
    msg.vacc = gps[9];
    msg.pdop = gps[10];
    msg.fix_type = static_cast<uint8_t>(gps[11]);
    msg.sat_num = static_cast<uint8_t>(gps[12]);
    msg.speed_accuracy = gps[13];
    msg.heading_accuracy = gps[14];
    msg.rtk_fix_status = static_cast<uint8_t>(gps[15]);

    pub->publish(msg);
}

void publish_gp2(double *gp2, gps_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APGPS msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = gp2[0];
    msg.gps_time = gp2[1];
    msg.lat = gp2[2];
    msg.lon = gp2[3];
    msg.alt_ellipsoid = gp2[4];
    msg.alt_msl = gp2[5];
    msg.speed = gp2[6];
    msg.heading = gp2[7];
    msg.hacc = gp2[8];
    msg.vacc = gp2[9];
    msg.pdop = gp2[10];
    msg.fix_type = static_cast<uint8_t>(gp2[11]);
    msg.sat_num = static_cast<uint8_t>(gp2[12]);
    msg.speed_accuracy = gp2[13];
    msg.heading_accuracy = gp2[14];
    msg.rtk_fix_status = static_cast<uint8_t>(gp2[15]);

    pub->publish(msg);
}

void publish_hdr(double *hdr, hdg_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    uint16_t status = static_cast<uint16_t>(hdr[9]);

    anello_interfaces::msg::APHDG msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = hdr[0];
    msg.gps_time = hdr[1];
    msg.rel_pos_n = hdr[2];
    msg.rel_pos_e = hdr[3];
    msg.rel_pos_d = hdr[4];
    msg.rel_pos_length = hdr[5];
    msg.rel_pos_heading = hdr[6];
    msg.rel_pos_length_accuracy = hdr[7];
    msg.rel_pos_heading_accuracy = hdr[8];
    msg.status_flags = status;
    msg.gnss_fix_ok = (status & (1 << 0)) > 0;
    msg.diff_soln = (status & (1 << 1)) > 0;
    msg.rel_pos_valid = (status & (1 << 2)) > 0;
    msg.carrier_solution = (status & (3 << 3)) >> 3;
    msg.is_moving = (status & (1 << 5)) > 0;
    msg.ref_pos_miss = (status & (1 << 6)) > 0;
    msg.ref_obs_miss = (status & (1 << 7)) > 0;
    msg.rel_pos_heading_valid = (status & (1 << 8)) > 0;
    msg.rel_pos_normalized = (status & (1 << 9)) > 0;

    pub->publish(msg);
}

void publish_imu(double *imu, imu_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APIMU msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = imu[0];
    msg.ax = imu[1];
    msg.ay = imu[2];
    msg.az = imu[3];
    msg.wx = imu[4];
    msg.wy = imu[5];
    msg.wz = imu[6];
    msg.wz_fog = imu[7];
    msg.odometer_speed = imu[8];
    msg.odometer_time = imu[9];
    msg.temp = imu[10];

    pub->publish(msg);
}

void publish_im1(double *im1, im1_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APIM1 msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = im1[0];
    msg.ax = im1[1];
    msg.ay = im1[2];
    msg.az = im1[3];
    msg.wx = im1[4];
    msg.wy = im1[5];
    msg.wz = im1[6];
    msg.wz_fog = im1[7];
    msg.temp = im1[8];
    msg.t_sync = im1[9];

    pub->publish(msg);
}

void publish_ins(double *ins, ins_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APINS msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = ins[0];
    msg.gps_time = ins[1];
    msg.ins_status = static_cast<uint8_t>(ins[2]);
    msg.lat = ins[3];
    msg.lon = ins[4];
    msg.alt_ellipsoid = ins[5];
    msg.vn = ins[6];
    msg.ve = ins[7];
    msg.vd = ins[8];
    msg.roll = ins[9];
    msg.pitch = ins[10];
    msg.heading = ins[11];
    msg.zupt = static_cast<uint8_t>(ins[12]);

    pub->publish(msg);
}

void publish_cov(double *cov, apcov_pub_t pub, rclcpp::Time stamp, const std::string &frame_id)
{
    anello_interfaces::msg::APCOV msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.mcu_time = cov[0];
    msg.cov_lat_lat = cov[1];
    msg.cov_lon_lon = cov[2];
    msg.cov_alt_alt = cov[3];
    msg.cov_lat_lon = cov[4];
    msg.cov_lat_alt = cov[5];
    msg.cov_lon_alt = cov[6];
    msg.cov_vn_vn = cov[7];
    msg.cov_ve_ve = cov[8];
    msg.cov_vd_vd = cov[9];
    msg.cov_vn_ve = cov[10];
    msg.cov_vn_vd = cov[11];
    msg.cov_ve_vd = cov[12];
    msg.cov_roll_roll = cov[13];
    msg.cov_pitch_pitch = cov[14];
    msg.cov_heading_heading = cov[15];
    msg.cov_roll_pitch = cov[16];
    msg.cov_roll_heading = cov[17];
    msg.cov_pitch_heading = cov[18];

    pub->publish(msg);
}

void publish_health(const health_message *health_msg, health_pub_t pub, rclcpp::Time stamp)
{
    anello_interfaces::msg::APHEALTH msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "anello";

    msg.position_acc_flag = health_msg->get_position_status();
    msg.heading_health_flag = health_msg->get_heading_status();
    msg.gyro_health_flag = health_msg->get_gyro_status();

    pub->publish(msg);
}
