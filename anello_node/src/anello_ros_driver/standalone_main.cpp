/********************************************************************************
 * File Name:   standalone_main.cpp
 * Description: Entry point for the standalone anello_ros_driver_node
 *              executable. Spins the driver under a MultiThreadedExecutor so
 *              the config-port callback group (send_cmd service, APODO input)
 *              cannot stall the data poll / publish path.
 *
 * License:     MIT License
 ********************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include "main_anello_ros_driver.h"

int main(int argc, char **argv)
{
    try {
        rclcpp::init(argc, argv);
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = anello::make_anello_driver(rclcpp::NodeOptions());
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), "%s",error.what());
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
