#ifndef ANELLO_PUBLISHER_TYPES_H
#define ANELLO_PUBLISHER_TYPES_H
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

#endif
