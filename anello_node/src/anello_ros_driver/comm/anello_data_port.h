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
 * File Name:   anello_data_port.h
 * Description: header file for the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        The anello_data_port class is used to read/write data to the anello GNSS INS, EVK, and IMU+.
 *              This class is meant to abstract the UART/ethernet communication with the anello devices.
 *
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__COMM__ANELLO_DATA_PORT_H_
#define ANELLO_ROS_DRIVER__COMM__ANELLO_DATA_PORT_H_

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

#include "anello_ros_driver/comm/ethernet_interface.h"
#include "anello_ros_driver/comm/serial_interface.h"
class anello_data_port
{
private:
  std::chrono::steady_clock::time_point last_ok_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_retry_{};
  std::chrono::steady_clock::time_point next_open_{};
  std::atomic<uint64_t> confirmed_generation_{0};
  bool decode_success = false;
  bool auto_detect = false;
  bool had_loss = false;  // a confirmed stream was lost (log recovery)
  bool reopen_warned = false;  // one open-failure warning per loss episode
  std::vector<std::string> port_names;
  std::string port_dir;
  uint32_t port_index = 0;
  int fail_count = 0;

  interface_config_t config;
  serial_interface uart_port;
  ethernet_interface ethernet_port;

  void init_ethernet();
  void init_uart();

    /* Refresh port_names from port_dir (sorted for a deterministic scan
     * order). Re-run on every scan step: a power-cycled unit usually
     * re-enumerates under a different /dev name. */
  void enumerate_ports();

  void port_parse_fail_uart();
  void port_parse_fail_ethernet();
  void port_confirm_uart();
  void port_confirm_ethernet();

  size_t get_data_uart(char *buf, size_t buf_len, int timeout_ms);
  size_t get_data_ethernet(char *buf, size_t buf_len);

  bool write_data_uart(const char *buf, size_t buf_len);
  bool write_data_ethernet(const char *buf, size_t buf_len);

public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     * port_directory overrides where AUTO mode scans for serial ports
     * (tests point it at a directory of pty symlinks).
     */
  explicit anello_data_port(
    const interface_config_t *config,
    std::string port_directory = PORT_DIR);
  ~anello_data_port();

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     */
  void init();
  size_t get_data(char *buf, size_t buf_len);
    /* timeout_ms bounds UART poll(); ethernet reads are non-blocking.
     * Empty drain polls advance recovery only after sustained silence. */
  size_t get_data(char *buf, size_t buf_len, int timeout_ms);
  bool write_data(const char *buf, size_t buf_len);

  void port_parse_fail();
  void port_confirm();
  uint64_t generation() const {return uart_port.generation();}
  uint64_t truncated_datagrams() const {return ethernet_port.truncated_datagrams();}

  const std::string get_portname() const
  {
    if (this->config.type == ETH) {
      return this->ethernet_port.get_remote_ip();  // Assuming get_remote_ip() exists
    } else {
      return this->uart_port.get_portname();
    }
  }
};

#endif  // ANELLO_ROS_DRIVER__COMM__ANELLO_DATA_PORT_H_
