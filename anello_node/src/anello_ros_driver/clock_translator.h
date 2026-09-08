/********************************************************************************
 * File Name:   clock_translator.h
 * Description: One-way device->host clock translation for message stamping.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef CLOCK_TRANSLATOR_H
#define CLOCK_TRANSLATOR_H
#include <cstdint>
#include <cmath>
#include <optional>

namespace anello
{

/* One-way device->host clock translator (Olson, IROS 2010 "A passive
 * solution to the sensor synchronization problem"): transport latency
 * only ever adds to arrival time, so the true clock offset is bounded
 * by the minimum observed (arrival - device) delta. Tracking that
 * minimum and letting it creep upward by a bounded drift rate gives
 * stamps whose dt follows the device clock with reduced arrival jitter.
 * Accuracy still depends on transport latency and oscillator behavior. */
class ClockTranslator
{
public:
    void update_ns(double device_s, int64_t arrival_ns)
    {
        if (!std::isfinite(device_s) || device_s<0) return;
        if (have_last_ && device_s+kResetThreshold<last_device_s_) reset();
        if (!epoch_valid_) { epoch_ns_=arrival_ns; epoch_valid_=true; }
        // Subtract after widening: two valid signed timestamps can have
        // a difference outside the int64 range after a clock step.
        update(device_s, static_cast<double>(
            (static_cast<long double>(arrival_ns)-epoch_ns_)*1e-9L));
    }
    std::optional<int64_t> translate_ns(double device_s) const {
        if (!epoch_valid_ || !std::isfinite(device_s) || device_s<0) return std::nullopt;
        const long double relative=std::round(static_cast<long double>(translate(device_s))*1e9L);
        const long double stamp=static_cast<long double>(epoch_ns_)+relative;
        // Exclusive upper bound also works where long double == double:
        // converting INT64_MAX to that type rounds it up to 2^63.
        constexpr long double limit=9223372036854775808.0L;
        if (!std::isfinite(stamp) || stamp < -limit || stamp >= limit) return std::nullopt;
        return static_cast<int64_t>(stamp);
    }
    void update(double device_s, double arrival_s)
    {
        if (!std::isfinite(device_s) || device_s<0 || !std::isfinite(arrival_s)) return;
        const double offset = arrival_s - device_s;
        if (!std::isfinite(offset)) return;
        if (have_last_ && device_s + kResetThreshold < last_device_s_)
        {
            reset();  // device time jumped far backwards: unit rebooted
        }
        if (have_last_)
        {
            // Absorb relative oscillator drift (bounded at 200 ppm).
            // Only forward steps advance the reference: GNSS-derived
            // messages (APGPS/APHDG) carry MCU times slightly behind the
            // concurrently streaming IMU messages, and those small
            // backwards steps must not rewind or reset the tracker.
            if (device_s > last_device_s_)
            {
                min_offset_ += kDriftBound * (device_s - last_device_s_);
                last_device_s_ = device_s;
            }
        }
        else
        {
            last_device_s_ = device_s;
            have_last_ = true;
        }
        if (n_samples_ == 0 || offset < min_offset_)
        {
            min_offset_ = offset;
        }
        if (n_samples_ < kWarmupSamples)
        {
            n_samples_++;
        }
    }
    bool ready() const { return n_samples_ >= kWarmupSamples; }
    double translate(double device_s) const { return device_s + min_offset_; }
    void reset()
    {
        epoch_valid_ = false;
        n_samples_ = 0;
        have_last_ = false;
        min_offset_ = 0.0;
    }

    static constexpr double kDriftBound = 200e-6;
    static constexpr int kWarmupSamples = 100;
    /* Backwards step large enough to mean a device reboot rather than
     * cross-stream reordering (GNSS PVT latency is well under 1 s). */
    static constexpr double kResetThreshold = 1.0;

private:
    int64_t epoch_ns_ = 0;
    bool epoch_valid_ = false;
    double min_offset_ = 0.0;
    double last_device_s_ = 0.0;
    bool have_last_ = false;
    int n_samples_ = 0;
};

}  // namespace anello

#endif
