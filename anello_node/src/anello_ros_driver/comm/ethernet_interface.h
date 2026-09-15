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
 * File Name:   ethernet_interface.h
 * Description: header file for the ethernet_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        This class is used to read and write data to the ANELLO unit over ethernet.
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__COMM__ETHERNET_INTERFACE_H_
#define ANELLO_ROS_DRIVER__COMM__ETHERNET_INTERFACE_H_

#include <netinet/in.h>

#include <string>

#include "anello_ros_driver/comm/base_interface.h"
class ethernet_interface : public base_interface
{
protected:
  std::string remote_ip_address;
  int local_port;
  int remote_port;

  uint64_t truncated_ = 0;
  int sockfd;
  struct sockaddr_in servaddr, cliaddr;

public:
  ethernet_interface(const std::string & remote_ip_address, int remote_port, int local_port);
  ~ethernet_interface() override;

  ethernet_interface(const ethernet_interface &) = delete;
  ethernet_interface & operator=(const ethernet_interface &) = delete;
  uint64_t truncated_datagrams() const {return truncated_;}
  void init();

  size_t get_data(char *buf, size_t buf_len) override;
  size_t get_data(char *buf, size_t buf_len, int timeout_ms);
  bool write_data(const char *buf, size_t buf_len) override;
  const std::string & get_remote_ip() const {return remote_ip_address;}
};

#endif  // ANELLO_ROS_DRIVER__COMM__ETHERNET_INTERFACE_H_
