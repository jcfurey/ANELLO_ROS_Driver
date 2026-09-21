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

#ifndef ANELLO_ROS_DRIVER__COMM__ANELLO_CONFIG_PORT_H_
#define ANELLO_ROS_DRIVER__COMM__ANELLO_CONFIG_PORT_H_
#include <atomic>
#include <chrono>
#include <string>

#include "anello_ros_driver/comm/ethernet_interface.h"
#include "anello_ros_driver/comm/serial_interface.h"
class anello_config_port {
public:
  explicit anello_config_port(const interface_config_t *config, std::string directory = PORT_DIR);
  void init();
  void poll();  // run in the config callback group
  size_t get_data(char *buf, size_t size) {return get_data(buf, size, 0);}
  size_t get_data(char *buf, size_t size, int timeout_ms);
  bool write_data(const char *buf, size_t size);
  bool connected() const {return confirmed_;}
  uint64_t truncated_datagrams() const {return ethernet_.truncated_datagrams();}
  std::string get_portname() const
  {
    return config_.type == ETH ? ethernet_.get_remote_ip() : uart_.get_portname();
  }

private:
  interface_config_t config_;
  serial_interface uart_;
  ethernet_interface ethernet_;
  std::string directory_, probe_response_;
  size_t scan_index_ = 0;
  bool probing_ = false;
  std::atomic<bool> confirmed_{false};
  std::chrono::steady_clock::time_point deadline_{};
};
#endif  // ANELLO_ROS_DRIVER__COMM__ANELLO_CONFIG_PORT_H_
