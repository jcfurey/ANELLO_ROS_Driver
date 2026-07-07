/********************************************************************************
 * File Name:   rate_monitor.h
 * Description: Sliding-window message/error rate counter for diagnostics.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef RATE_MONITOR_H
#define RATE_MONITOR_H

#include <algorithm>
#include <chrono>
#include <cstdint>

/* Sliding-window message/error rates for diagnostics: lifetime totals
 * plus a trailing window (kWindowSeconds), so a device that stops
 * streaming (or a link that degrades) is visible on /diagnostics
 * instead of showing the last known health flags as "nominal".
 * Implemented as a fixed ring of 10 x 500 ms count buckets covering the
 * 5 s window. Templated on the clock so tests can inject a fake one.
 * Single-threaded access only. */
template <typename Clock = std::chrono::steady_clock>
struct RateMonitor
{
    static constexpr double kWindowSeconds = 5.0;

    uint64_t total_ok = 0;
    uint64_t total_checksum_fail = 0;
    uint64_t total_parse_fail = 0;

    void add_ok()            { total_ok++;            bump_ok(); }
    void add_checksum_fail() { total_checksum_fail++; bump_err(); }
    void add_parse_fail()    { total_parse_fail++;    bump_err(); }

    double rate_hz()
    {
        advance();
        uint64_t ok = 0;
        for (const Bucket &b : buckets_)
            ok += b.ok;
        return static_cast<double>(ok) / kWindowSeconds;
    }

    double error_percent()
    {
        advance();
        uint64_t ok = 0, err = 0;
        for (const Bucket &b : buckets_) {
            ok += b.ok;
            err += b.err;
        }
        const uint64_t total = ok + err;
        return total > 0
            ? 100.0 * static_cast<double>(err) / static_cast<double>(total)
            : 0.0;
    }

private:
    struct Bucket
    {
        uint32_t ok = 0;
        uint32_t err = 0;
    };

    static constexpr int kNumBuckets = 10;
    static constexpr std::chrono::milliseconds kBucketWidth{500};

    Bucket buckets_[kNumBuckets] = {};
    int64_t cur_epoch_ = -1;  // now() in bucket widths at the last update

    // Move the ring to the current bucket epoch, zeroing every bucket
    // in (cur_epoch_, min(epoch, cur_epoch_ + kNumBuckets)] — after a
    // gap longer than the window that is the whole ring.
    void advance()
    {
        const int64_t epoch = Clock::now().time_since_epoch() / kBucketWidth;
        if (epoch > cur_epoch_) {
            const int64_t last =
                std::min(epoch, cur_epoch_ + static_cast<int64_t>(kNumBuckets));
            for (int64_t e = cur_epoch_ + 1; e <= last; ++e)
                buckets_[e % kNumBuckets] = Bucket{};
            cur_epoch_ = epoch;
        }
    }

    void bump_ok()
    {
        advance();
        buckets_[cur_epoch_ % kNumBuckets].ok++;
    }

    void bump_err()
    {
        advance();
        buckets_[cur_epoch_ % kNumBuckets].err++;
    }
};

#endif
