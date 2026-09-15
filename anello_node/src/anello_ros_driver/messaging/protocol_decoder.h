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

#ifndef ANELLO_ROS_DRIVER__MESSAGING__PROTOCOL_DECODER_H_
#define ANELLO_ROS_DRIVER__MESSAGING__PROTOCOL_DECODER_H_

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "anello_ros_driver/protocol_types.h"
namespace anello
{
enum class MessageKind { imu, im1, gps, gps2, heading, ins, covariance, ahrs };
// The legacy array layout is confined to the protocol/publication boundary.
struct DecodedPacket
{
  MessageKind kind;
  std::array<double, MAXFIELD> values{};
};
class StreamDecoder {
public:
  using Callback = std::function<void(const DecodedPacket &)>;
  void feed(uint8_t byte, const Callback & callback);
  void reset() {buffer_.clear();}
  bool pending() const {return !buffer_.empty();}
  uint64_t checksum_failures = 0;
  uint64_t parse_failures = 0;

private:
  std::string buffer_;
};
bool decode_ascii_frame(const std::string & frame, DecodedPacket & packet);
bool decode_binary_frame(const a1buff_t & frame, DecodedPacket & packet);
bool validate_packet(const DecodedPacket & packet);
bool ins_position_valid(const double *ins);
bool ins_heading_valid(const double *ins);
}  // namespace anello
#endif  // ANELLO_ROS_DRIVER__MESSAGING__PROTOCOL_DECODER_H_
