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
 * File Name:   standalone_main.cpp
 * Description: Entry point for the standalone anello_ros_driver_node
 *              executable. Spins the driver under a MultiThreadedExecutor so
 *              the config-port callback group (send_cmd service, APODO input)
 *              cannot stall the data poll / publish path.
 *
 * License:     MIT License
 ********************************************************************************/

#include "anello_ros_driver/main_anello_ros_driver.h"
#include "rclcpp/rclcpp.hpp"
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
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), "%s", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
}
