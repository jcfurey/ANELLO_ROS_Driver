/********************************************************************************
 * File Name:   main_anello_ros_driver.h
 * Description: Header file for the ANELLO ROS2 driver node.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef MAIN_ANELLO_ROS_DRIVER_H
#define MAIN_ANELLO_ROS_DRIVER_H

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "anello_interfaces/msg/apimu.hpp"
#include "anello_interfaces/msg/apim1.hpp"
#include "anello_interfaces/msg/apins.hpp"
#include "anello_interfaces/msg/apgps.hpp"
#include "anello_interfaces/msg/aphdg.hpp"
#include "anello_interfaces/msg/aphealth.hpp"
#include "anello_interfaces/msg/apcov.hpp"
#include "anello_interfaces/msg/apodo.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "bit_tools.h"
#include "comm/serial_interface.h"
#include "comm/ethernet_interface.h"
#include "comm/anello_config_port.h"
#include "comm/anello_data_port.h"

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#define DEBUG_PRINT(...) RCLCPP_DEBUG(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define ERROR_PRINT(...) RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)
#define WARNING_PRINT(...) RCLCPP_WARN(rclcpp::get_logger("anello_ros_driver"), __VA_ARGS__)

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

// Message buffer structure
struct a1buff_t
{
    uint8_t buf[MAX_BUF_LEN] = {};
    int nseg = 0;
    int nbyte = 0;
    int nlen = 0;
    int type = 0;
    int subtype = 0;
    int crc = 0;
    int loc[MAXFIELD] = {};
};

// RTCM binary message structures
#pragma pack(push, 1)

struct rtcm_apim1_t
{
    uint64_t MCU_Time;
    uint64_t Sync_Time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t Temp_C;
};

struct rtcm_apimu_t
{
    uint64_t MCU_Time;
    uint64_t Sync_Time;
    uint64_t ODO_time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t ODO;
    int16_t Temp_C;
};

struct rtcm_old_apimu_t
{
    uint64_t MCU_Time;
    uint64_t ODO_time;
    int32_t AX, AY, AZ;
    int32_t WX, WY, WZ;
    int32_t OG_WZ;
    int16_t ODO;
    int16_t Temp_C;
};

struct rtcm_apgps_t
{
    uint64_t Time;
    uint64_t GPS_Time;
    int32_t Latitude;
    int32_t Longitude;
    int32_t Alt_ellipsoid;
    int32_t Alt_msl;
    int32_t Speed;
    int32_t Heading;
    uint32_t Hor_Acc;
    uint32_t Ver_Acc;
    uint32_t Hdg_Acc;
    uint32_t Spd_Acc;
    uint16_t PDOP;
    uint8_t FixType;
    uint8_t SatNum;
    uint8_t RTK_Status;
    uint8_t Antenna_ID;
};

struct rtcm_aphdr_t
{
    uint64_t MCU_Time;
    uint64_t GPS_Time;
    int32_t relPosN;
    int32_t relPosE;
    int32_t resPosD;
    int32_t relPosLength;
    int32_t relPosHeading;
    uint32_t relPosLength_Accuracy;
    uint32_t relPosHeading_Accuracy;
    uint16_t statusFlags;
};

struct rtcm_apins_t
{
    uint64_t Time;
    uint64_t GPS_Time;
    int32_t Latitude;
    int32_t Longitude;
    int32_t Alt_ellipsoid;
    int32_t Vn, Ve, Vd;
    int32_t Roll, Pitch, Heading_Yaw;
    uint8_t ZUPT;
    uint8_t Status;
};

struct rtcm_apcov_t
{
    uint64_t Time;
    float covLatLat;
    float covLonLon;
    float covAltAlt;
    float covLatLon;
    float covLatAlt;
    float covLonAlt;
    float covVnVn;
    float covVeVe;
    float covVdVd;
    float covVnVe;
    float covVnVd;
    float covVeVd;
    float covRollRoll;
    float covPitchPitch;
    float covYawYaw;
    float covRollPitch;
    float covRollYaw;
    float covPitchYaw;
};

#pragma pack(pop)

namespace anello
{
// Factory for the standalone executable (see standalone_main.cpp): builds
// the driver node so main() can spin it under a MultiThreadedExecutor.
rclcpp::Node::SharedPtr make_anello_driver(const rclcpp::NodeOptions &options);
}  // namespace anello

#endif
