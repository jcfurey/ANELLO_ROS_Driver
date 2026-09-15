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
 * File Name:   base_interface.h
 * Description: Contains the base implementation used for all ANELLO device interfaces.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        All interfaces to the ANELLO devices should inherit from this class.
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__COMM__BASE_INTERFACE_H_
#define ANELLO_ROS_DRIVER__COMM__BASE_INTERFACE_H_

#include <cstdint>
#include <string>
enum interface_type_t
{
  UART,
  ETH
};

struct interface_config_t
{
  interface_type_t type = UART;
  std::string data_port_name;
  std::string config_port_name;
  std::string remote_ip;
  int local_data_port = 1111;
  int local_config_port = 2222;
  int local_odometer_port = 3333;
  uint32_t baud_rate = 230400;
};

class base_interface
{
public:
  virtual ~base_interface() = default;

  virtual size_t get_data(char *buf, size_t buf_len) {(void)buf; (void)buf_len; return 0;}
  virtual bool write_data(const char *buf, size_t buf_len) = 0;
};

#endif  // ANELLO_ROS_DRIVER__COMM__BASE_INTERFACE_H_
