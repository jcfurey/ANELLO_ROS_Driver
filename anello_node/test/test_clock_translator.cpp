// Tests for the Olson minimum-offset device->host clock translator.

#include <gtest/gtest.h>

#include "../src/anello_ros_driver/clock_translator.h"

using anello::ClockTranslator;

TEST(ClockTranslator, NotReadyUntilWarmedUp)
{
    ClockTranslator ct;
    for (int i = 0; i < ClockTranslator::kWarmupSamples - 1; ++i) {
        ct.update(i * 0.01, 100.0 + i * 0.01);
        EXPECT_FALSE(ct.ready());
    }
    ct.update(1.0, 101.0);
    EXPECT_TRUE(ct.ready());
}

TEST(ClockTranslator, TracksMinimumOffsetThroughJitter)
{
    ClockTranslator ct;
    // True offset 100 s; transport latency jitters 5..50 ms but one
    // packet arrives with only 5 ms latency — the translator must lock
    // to (close to) the minimum.
    for (int i = 0; i < 200; ++i) {
        const double device = i * 0.01;
        const double latency = (i == 50) ? 0.005 : 0.005 + 0.045 * ((i * 7) % 10) / 10.0;
        ct.update(device, 100.0 + device + latency);
    }
    ASSERT_TRUE(ct.ready());
    // translate(device) - (100 + device) == captured min latency, which
    // can only have crept up by drift_bound * elapsed (2 s * 200ppm)
    const double residual = ct.translate(2.0) - 102.0;
    EXPECT_GE(residual, 0.0049);
    EXPECT_LE(residual, 0.005 + 2.0 * ClockTranslator::kDriftBound + 1e-9);
}

TEST(ClockTranslator, TranslatedTimesPreserveDeviceDeltas)
{
    ClockTranslator ct;
    for (int i = 0; i < 150; ++i) {
        const double device = i * 0.01;
        ct.update(device, 50.0 + device + 0.002 + 0.001 * (i % 3));
    }
    ASSERT_TRUE(ct.ready());
    // dt between translated stamps equals dt between device times,
    // jitter-free — the entire point of the translator.
    const double dt = ct.translate(1.50) - ct.translate(1.49);
    EXPECT_NEAR(dt, 0.01, 1e-12);
}

TEST(ClockTranslator, ResetsWhenDeviceTimeGoesBackwards)
{
    ClockTranslator ct;
    for (int i = 0; i < 150; ++i) {
        ct.update(i * 0.01, 100.0 + i * 0.01 + 0.002);
    }
    ASSERT_TRUE(ct.ready());

    // Unit reboots: device time restarts near zero
    ct.update(0.0, 103.002);
    EXPECT_FALSE(ct.ready());

    // After re-warm-up the translator follows the new timeline
    for (int i = 1; i <= ClockTranslator::kWarmupSamples; ++i) {
        ct.update(i * 0.01, 103.0 + i * 0.01 + 0.002);
    }
    EXPECT_TRUE(ct.ready());
    EXPECT_NEAR(ct.translate(1.0), 104.002, 1e-3);
}

TEST(ClockTranslator, ToleratesCrossStreamBackwardsSteps)
{
    ClockTranslator ct;
    // Mixed-stream feed: a 100 Hz IMU stream interleaved with 4 Hz GNSS
    // messages whose MCU times lag the IMU stream by ~100 ms (PVT
    // computation latency). These small backwards steps must not reset
    // the warm-up — only a reboot-scale jump may.
    for (int i = 0; i < 300; ++i) {
        const double device = i * 0.01;
        ct.update(device, 100.0 + device + 0.002);
        if (i > 10 && i % 25 == 0) {
            const double gnss_device = device - 0.1;  // lags the IMU stream
            ct.update(gnss_device, 100.0 + device + 0.003);
        }
    }
    ASSERT_TRUE(ct.ready());
    // The lagged messages must also not corrupt the learned offset.
    const double residual = ct.translate(3.0) - 103.0;
    EXPECT_GE(residual, 0.0);
    EXPECT_LE(residual, 0.002 + 3.0 * ClockTranslator::kDriftBound + 1e-9);

    // A reboot-scale backwards jump still resets.
    ct.update(0.0, 103.1);
    EXPECT_FALSE(ct.ready());
}

TEST(ClockTranslator, DriftCreepStaysBounded)
{
    ClockTranslator ct;
    // Constant 10 ms latency for 100 s of device time: the offset may
    // only creep up by kDriftBound per second of device time.
    for (int i = 0; i < 10000; ++i) {
        ct.update(i * 0.01, 200.0 + i * 0.01 + 0.010);
    }
    const double residual = ct.translate(100.0) - 300.0;
    EXPECT_GE(residual, 0.0);
    EXPECT_LE(residual, 0.010 + 100.0 * ClockTranslator::kDriftBound + 1e-9);
}
