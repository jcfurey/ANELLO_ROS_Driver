#ifndef MAIN_ANELLO_ROS_DRIVER_H
#define MAIN_ANELLO_ROS_DRIVER_H
#include "rclcpp/node.hpp"
namespace anello {
rclcpp::Node::SharedPtr make_anello_driver(const rclcpp::NodeOptions &options);
}
#endif
