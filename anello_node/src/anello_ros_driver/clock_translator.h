/********************************************************************************
 * File Name:   clock_translator.h
 * Description: One-way device->host clock translation for message stamping.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef CLOCK_TRANSLATOR_H
#define CLOCK_TRANSLATOR_H

namespace anello
{

/* One-way device->host clock translator (Olson, IROS 2010 "A passive
 * solution to the sensor synchronization problem"): transport latency
 * only ever adds to arrival time, so the true clock offset is bounded
 * by the minimum observed (arrival - device) delta. Tracking that
 * minimum and letting it creep upward by a bounded drift rate gives
 * stamps whose dt follows the device clock, free of serial/OS jitter,
 * and provably never worse than arrival stamping. */
class ClockTranslator
{
public:
    void update(double device_s, double arrival_s)
    {
        if (have_last_ && device_s + 1e-6 < last_device_s_)
        {
            reset();  // device time went backwards: unit rebooted
        }
        if (have_last_ && n_samples_ > 0)
        {
            // Absorb relative oscillator drift (bounded at 200 ppm)
            min_offset_ += kDriftBound * (device_s - last_device_s_);
        }
        const double offset = arrival_s - device_s;
        if (n_samples_ == 0 || offset < min_offset_)
        {
            min_offset_ = offset;
        }
        last_device_s_ = device_s;
        have_last_ = true;
        if (n_samples_ < kWarmupSamples)
        {
            n_samples_++;
        }
    }
    bool ready() const { return n_samples_ >= kWarmupSamples; }
    double translate(double device_s) const { return device_s + min_offset_; }
    void reset()
    {
        n_samples_ = 0;
        have_last_ = false;
        min_offset_ = 0.0;
    }

    static constexpr double kDriftBound = 200e-6;
    static constexpr int kWarmupSamples = 100;

private:
    double min_offset_ = 0.0;
    double last_device_s_ = 0.0;
    bool have_last_ = false;
    int n_samples_ = 0;
};

}  // namespace anello

#endif
