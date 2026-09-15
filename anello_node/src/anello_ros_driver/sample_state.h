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

#ifndef ANELLO_ROS_DRIVER__SAMPLE_STATE_H_
#define ANELLO_ROS_DRIVER__SAMPLE_STATE_H_
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "anello_ros_driver/rate_monitor.h"
namespace anello
{
using SteadyClock = std::chrono::steady_clock;
struct SampleTime
{
  bool valid = false;
  double device_ms = 0;
  SteadyClock::time_point arrival{};
  void set(double ms, SteadyClock::time_point now = SteadyClock::now())
  {
    valid = true; device_ms = ms; arrival = now;
  }
  double age(SteadyClock::time_point now = SteadyClock::now()) const
  {
    return valid ? std::chrono::duration<double>(now - arrival).count() : INFINITY;
  }
  bool matches(double ms, double max_age, SteadyClock::time_point now = SteadyClock::now()) const
  {
    return valid && std::abs(ms - device_ms) * 1e-3 <= max_age && age(now) <= max_age;
  }
};
// Ordering and freshness belong to a logical stream, not the whole multiplexed
// connection. Counters survive clock epochs; timing/rate windows do not.
struct StreamContinuity
{
  SampleTime last;
  RateMonitor rate;
  uint64_t accepted = 0, duplicates = 0, out_of_order = 0, stamp_rejections = 0, gaps = 0;
  double last_interval_s = 0.0, max_interval_s = 0.0;
  bool newer(double ms)
  {
    if (!last.valid || ms > last.device_ms) {return true;}
    if (ms == last.device_ms) {++duplicates;} else {++out_of_order;}
    return false;
  }
  void accept(double ms, double gap_threshold_s, SteadyClock::time_point now = SteadyClock::now())
  {
    if (last.valid) {
      last_interval_s = (ms - last.device_ms) * 1e-3;
      max_interval_s = std::max(max_interval_s, last_interval_s);
      if (last_interval_s > gap_threshold_s || last.age(now) > gap_threshold_s) {++gaps;}
    }
    last.set(ms, now);
    ++accepted;
    rate.add_ok(1, now);
  }
  void reset_epoch() {last = {}; rate = {}; last_interval_s = 0.0;}
};
// A ROS clock step and a device reboot both invalidate the mapping and
// caches. Compare ROS time to steady time so ordinary transport gaps do
// not look like clock jumps. The device high-water mark tolerates latency
// differences between message types.
class ClockDiscontinuity {
public:
  enum Reason : uint8_t
  {
    NONE = 0, DEVICE_TIME = 1, ROS_CLOCK = 2, CLOCK_MODE = 4
  };
  bool update(double device_ms, int64_t ros_ns, int64_t steady_ns, bool simulated)
  {
    reasons_ = NONE;
    if (valid_) {
      const long double ros_delta = static_cast<long double>(ros_ns) - ros_ns_;
      const long double steady_delta = static_cast<long double>(steady_ns) - steady_ns_;
      if (device_ms + 1000 < device_high_ms_) {reasons_ |= DEVICE_TIME;}
      if (simulated != simulated_) {reasons_ |= CLOCK_MODE;}
      if (ros_delta < 0 || (!simulated && std::abs(ros_delta - steady_delta) > 5e8) ||
        (simulated && ros_delta > 0 && ros_delta - steady_delta > 5e8)) {reasons_ |= ROS_CLOCK;}
    }
    const bool reset = reasons_ != NONE;
    device_high_ms_ = (!valid_ || reset) ? device_ms : std::max(device_high_ms_, device_ms);
    valid_ = true; ros_ns_ = ros_ns; steady_ns_ = steady_ns; simulated_ = simulated;
    return reset;
  }
  uint8_t reasons() const {return reasons_;}
  void reset() {valid_ = false; reasons_ = NONE;}

private:
  uint8_t reasons_ = NONE;
  bool valid_ = false, simulated_ = false;
  double device_high_ms_ = 0;
  int64_t ros_ns_ = 0, steady_ns_ = 0;
};
}  // namespace anello
#endif  // ANELLO_ROS_DRIVER__SAMPLE_STATE_H_
