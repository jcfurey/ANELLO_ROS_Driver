// Tests for the bucket-ring sliding-window rate monitor, driven by an
// injectable fake steady clock so bucket boundaries are deterministic.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "../src/anello_ros_driver/rate_monitor.h"

namespace
{
// steady_clock-compatible clock whose now() the test sets explicitly.
struct FakeClock
{
    using duration = std::chrono::steady_clock::duration;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<FakeClock, duration>;
    static constexpr bool is_steady = true;

    static inline time_point current{};
    static time_point now() { return current; }
    static void set_ms(int64_t ms)
    {
        current = time_point(std::chrono::milliseconds(ms));
    }
    static void advance_ms(int64_t ms)
    {
        current += std::chrono::milliseconds(ms);
    }
};
}  // namespace

TEST(RateMonitor, SteadyStreamMeasuresRate)
{
    FakeClock::set_ms(1000000);  // bucket-aligned (multiple of 500 ms)
    RateMonitor<FakeClock> m;

    // 100 Hz for exactly the 5 s window: samples at t = 0..4990 ms fill
    // all 10 buckets and none has expired at the last sample time, so
    // the reading is exact. (One 500 ms bucket = +/-10 Hz quantization
    // when the query time is not aligned like this.)
    for (int i = 0; i < 500; ++i) {
        if (i > 0)
            FakeClock::advance_ms(10);
        m.add_ok();
    }
    EXPECT_NEAR(m.rate_hz(), 100.0, 1e-9);

    // One bucket width later the oldest 50-sample bucket falls out of
    // the ring: the rate drops by exactly one bucket's worth.
    FakeClock::advance_ms(500);
    EXPECT_NEAR(m.rate_hz(), 90.0, 1e-9);
}

TEST(RateMonitor, SilenceDropsRateToZero)
{
    FakeClock::set_ms(2000000);
    RateMonitor<FakeClock> m;
    for (int i = 0; i < 100; ++i) {
        m.add_ok();
        FakeClock::advance_ms(10);
    }
    EXPECT_GT(m.rate_hz(), 0.0);

    // A device that stops streaming must read 0 Hz once the window has
    // fully elapsed — showing the last known rate as "nominal" is the
    // exact failure mode this monitor exists to expose.
    FakeClock::advance_ms(5500);
    EXPECT_DOUBLE_EQ(m.rate_hz(), 0.0);
}

TEST(RateMonitor, ErrorPercentWithinWindow)
{
    FakeClock::set_ms(3000000);
    RateMonitor<FakeClock> m;
    // 8 ok + 2 errors (one of each error kind: both must count) inside
    // the window -> 20% exactly.
    for (int i = 0; i < 8; ++i) {
        m.add_ok();
        FakeClock::advance_ms(10);
    }
    m.add_checksum_fail();
    FakeClock::advance_ms(10);
    m.add_parse_fail();
    EXPECT_DOUBLE_EQ(m.error_percent(), 20.0);
}

TEST(RateMonitor, EmptyWindowErrorPercentIsZero)
{
    // No traffic must read 0% errors (not 0/0 -> NaN) so /diagnostics
    // stays parseable before the first message arrives.
    FakeClock::set_ms(4000000);
    RateMonitor<FakeClock> m;
    EXPECT_DOUBLE_EQ(m.error_percent(), 0.0);
    EXPECT_DOUBLE_EQ(m.rate_hz(), 0.0);
}

TEST(RateMonitor, LifetimeTotalsAreNeverTrimmed)
{
    FakeClock::set_ms(5000000);
    RateMonitor<FakeClock> m;
    for (int i = 0; i < 42; ++i)
        m.add_ok();
    m.add_checksum_fail();
    m.add_parse_fail();

    // Far past the window: the ring empties but the lifetime totals
    // must survive — they are the "since startup" counters on
    // /diagnostics and may only ever grow.
    FakeClock::advance_ms(60000);
    EXPECT_DOUBLE_EQ(m.rate_hz(), 0.0);
    EXPECT_EQ(m.total_ok, 42u);
    EXPECT_EQ(m.total_checksum_fail, 1u);
    EXPECT_EQ(m.total_parse_fail, 1u);

    m.add_ok();
    EXPECT_EQ(m.total_ok, 43u);
}
