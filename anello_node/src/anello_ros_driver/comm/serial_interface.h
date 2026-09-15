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

#ifndef ANELLO_ROS_DRIVER__COMM__SERIAL_INTERFACE_H_
#define ANELLO_ROS_DRIVER__COMM__SERIAL_INTERFACE_H_
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "anello_ros_driver/comm/base_interface.h"
#define PORT_DIR "/dev/"
#define PORT_PREFIX "ttyUSB"
#define MAX_PORT_PARSE_FAIL 5
#define MAX_CONFIRMED_PORT_FAIL 200
class serial_interface : public base_interface {
public:
  serial_interface() = default;
  ~serial_interface() override;
  serial_interface(const serial_interface &) = delete;
  serial_interface & operator=(const serial_interface &) = delete;
  void init(const std::string & name, uint32_t baud);
  size_t get_data(char *buf, size_t size) override {return get_data(buf, size, 0);}
  size_t get_data(char *buf, size_t size, int timeout_ms);
  bool write_data(const char *buf, size_t size) override;
  bool write_data(const char *buf, size_t size, uint64_t expected_generation);
  std::string get_portname() const;
  bool get_port_enabled() const;
  uint64_t generation() const {return generation_;}
  void close_port();

private:
  int duplicate_fd() const;
  void close_generation(uint64_t generation);
  mutable std::mutex mutex_;
  std::timed_mutex write_mutex_;
  int usb_fd = -1;
  std::string portname;
  std::atomic<uint64_t> generation_{0};
};
#endif  // ANELLO_ROS_DRIVER__COMM__SERIAL_INTERFACE_H_
