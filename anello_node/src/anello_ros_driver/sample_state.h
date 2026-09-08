#ifndef ANELLO_SAMPLE_STATE_H
#define ANELLO_SAMPLE_STATE_H
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace anello {
using SteadyClock=std::chrono::steady_clock;
struct SampleTime {
    bool valid=false;
    double device_ms=0;
    SteadyClock::time_point arrival{};
    void set(double ms, SteadyClock::time_point now=SteadyClock::now()) {
        valid=true; device_ms=ms; arrival=now;
    }
    double age(SteadyClock::time_point now=SteadyClock::now()) const {
        return valid?std::chrono::duration<double>(now-arrival).count():INFINITY;
    }
    bool matches(double ms, double max_age, SteadyClock::time_point now=SteadyClock::now()) const {
        return valid && std::abs(ms-device_ms)*1e-3<=max_age && age(now)<=max_age;
    }
};
// A ROS clock step and a device reboot both invalidate the mapping and
// caches. Compare ROS time to steady time so ordinary transport gaps do
// not look like clock jumps. The device high-water mark tolerates latency
// differences between message types.
class ClockDiscontinuity {
public:
    bool update(double device_ms, int64_t ros_ns, int64_t steady_ns, bool simulated) {
        bool reset=false;
        if (valid_) {
            const long double ros_delta=static_cast<long double>(ros_ns)-ros_ns_;
            const long double steady_delta=static_cast<long double>(steady_ns)-steady_ns_;
            reset=device_ms+1000<device_high_ms_ || simulated!=simulated_ ||
                ros_delta<0 || (!simulated && std::abs(ros_delta-steady_delta)>5e8) ||
                (simulated && ros_delta>0 && ros_delta-steady_delta>5e8);
        }
        device_high_ms_=(!valid_ || reset)?device_ms:std::max(device_high_ms_,device_ms);
        valid_=true; ros_ns_=ros_ns; steady_ns_=steady_ns; simulated_=simulated;
        return reset;
    }
    void reset() { valid_=false; }
private:
    bool valid_=false, simulated_=false;
    double device_high_ms_=0;
    int64_t ros_ns_=0, steady_ns_=0;
};
}  // namespace anello
#endif
