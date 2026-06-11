// Health monitor behavior tests with synthetic sensor streams:
// stuck-channel detection, MEMS-vs-FOG discrepancy gating, the FOG
// saturation guard, FOG-disabled handling, GPS heading speed gating,
// and APHDG validity-flag gating.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "../src/anello_ros_driver/messaging/health_message.h"

namespace
{
constexpr int kFill = 3 * IMU_MOVING_AVERAGE_SIZE;  // enough to fill buffers

// imu_msg layout: [0]=mcu_time, [6]=wz (MEMS), [7]=wz_fog
void feed_imu(health_message &h, int n, double wz_mean, double wz_noise,
              double fog_mean, double fog_noise)
{
    double msg[16] = {};
    for (int i = 0; i < n; ++i) {
        const double s = (i % 2 == 0) ? 1.0 : -1.0;  // +/- alternation
        msg[0] = 10.0 * i;
        msg[6] = wz_mean + s * wz_noise;
        msg[7] = fog_mean + s * fog_noise;
        h.add_imu_message(msg);
    }
}

// gps_msg layout: [6]=speed, [7]=heading, [8]=hacc, [14]=heading_acc,
// [15]=rtk
void feed_gps(health_message &h, double speed, double heading,
              double heading_acc)
{
    double msg[16] = {};
    msg[6] = speed;
    msg[7] = heading;
    msg[8] = 0.5;
    msg[14] = heading_acc;
    h.add_gps_message(msg);
}

// ins_msg layout: [2]=status, [11]=heading
void feed_ins(health_message &h, double heading, double status = 2.0)
{
    double msg[16] = {};
    msg[2] = status;
    msg[11] = heading;
    h.add_ins_message(msg);
}

// hdg_msg layout: [5]=baseline, [6]=heading, [8]=heading_acc, [9]=flags
void feed_hdg(health_message &h, double heading, uint16_t flags)
{
    double msg[16] = {};
    msg[5] = 1.0;
    msg[6] = heading;
    msg[8] = 0.1;
    msg[9] = static_cast<double>(flags);
    h.add_hdg_message(msg);
}

constexpr uint16_t kHdgValidFlags = (1 << 0) | (1 << 2) | (1 << 8);
}  // namespace

TEST(GyroHealth, HealthyNoisyChannelsAreGood)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);
    EXPECT_EQ(h.get_gyro_status(), GYRO_GOOD);
}

TEST(GyroHealth, StuckMemsChannelIsBad)
{
    health_message h;
    // MEMS frozen at a constant; FOG healthy
    feed_imu(h, kFill, 0.05, 0.0, 0.0, 0.01);
    EXPECT_EQ(h.get_gyro_status(), GYRO_BAD);
}

TEST(GyroHealth, StuckFogChannelIsBad)
{
    health_message h;
    // FOG frozen at a nonzero constant (zero would mean disabled)
    feed_imu(h, kFill, 0.0, 0.05, 0.02, 0.0);
    EXPECT_EQ(h.get_gyro_status(), GYRO_BAD);
}

TEST(GyroHealth, DisabledFogIsNotAFault)
{
    health_message h;
    // APCFG fog off: OG_WZ exactly 0 forever
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.0);
    EXPECT_EQ(h.get_gyro_status(), GYRO_GOOD);
}

TEST(GyroHealth, BiasDiscrepancyIsBad)
{
    health_message h;
    // Both channels alive but means differ by 0.5 deg/s (gate is 0.25)
    feed_imu(h, kFill, 0.5, 0.05, 0.0, 0.01);
    EXPECT_EQ(h.get_gyro_status(), GYRO_BAD);
}

TEST(GyroHealth, SmallDiscrepancyWithinGateIsGood)
{
    health_message h;
    feed_imu(h, kFill, 0.1, 0.05, 0.0, 0.01);
    EXPECT_EQ(h.get_gyro_status(), GYRO_GOOD);
}

TEST(GyroHealth, FogSaturationIsNotAFault)
{
    health_message h;
    // Yaw rate beyond the FOG's 200 deg/s range: MEMS tracks 250,
    // FOG rails at 200 — large divergence, but expected.
    feed_imu(h, kFill, 250.0, 0.05, 200.0, 0.01);
    EXPECT_EQ(h.get_gyro_status(), GYRO_GOOD);
}

TEST(HeadingHealth, MismatchAtSpeedTripsAfterStreak)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);  // sane gyro context

    for (int i = 0; i < 5; ++i) {
        feed_gps(h, 5.0, 90.0, 0.5);   // moving at 5 m/s, accurate heading
        feed_ins(h, 0.0);              // INS disagrees by 90 deg
    }
    EXPECT_EQ(h.get_heading_status(), HEADING_UNSTABLE);
}

TEST(HeadingHealth, SlowSpeedComparisonsAreGated)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);

    // Same 90-deg disagreement, but below the 2 m/s speed gate:
    // course-over-ground is meaningless, so no streak may accumulate.
    for (int i = 0; i < 10; ++i) {
        feed_gps(h, 0.5, 90.0, 0.5);
        feed_ins(h, 0.0);
    }
    EXPECT_EQ(h.get_heading_status(), HEADING_STABLE);
}

TEST(HeadingHealth, AgreementAtSpeedStaysStable)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);

    for (int i = 0; i < 10; ++i) {
        feed_gps(h, 5.0, 90.0, 0.5);
        feed_ins(h, 89.0);  // within the 3 deg threshold
    }
    EXPECT_EQ(h.get_heading_status(), HEADING_STABLE);
}

TEST(HeadingHealth, DualAntennaMismatchWithValidFlagsTrips)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);

    for (int i = 0; i < 5; ++i) {
        feed_hdg(h, 90.0, kHdgValidFlags);
        feed_ins(h, 0.0);
    }
    EXPECT_EQ(h.get_heading_status(), HEADING_UNSTABLE);
}

TEST(HeadingHealth, InvalidHdgFlagsAreIgnored)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);

    // Heading-valid bit (8) missing: epochs must not count
    const uint16_t no_heading_valid = (1 << 0) | (1 << 2);
    for (int i = 0; i < 10; ++i) {
        feed_hdg(h, 90.0, no_heading_valid);
        feed_ins(h, 0.0);
    }
    EXPECT_EQ(h.get_heading_status(), HEADING_STABLE);
}

TEST(HeadingHealth, UninitializedInsResetsStreak)
{
    health_message h;
    feed_imu(h, kFill, 0.0, 0.05, 0.0, 0.01);

    for (int i = 0; i < 3; ++i) {
        feed_gps(h, 5.0, 90.0, 0.5);
        feed_ins(h, 0.0);
    }
    // INS drops to attitude-only before the streak reaches 4
    feed_gps(h, 5.0, 90.0, 0.5);
    feed_ins(h, 0.0, /*status=*/1.0);
    EXPECT_EQ(h.get_heading_status(), HEADING_STABLE);
}

TEST(PositionHealth, RtkFixedIsCmLevel)
{
    health_message h;
    double msg[16] = {};
    msg[6] = 5.0;
    msg[8] = 0.02;   // hacc
    msg[15] = 2.0;   // RTK fixed
    h.add_gps_message(msg);
    EXPECT_EQ(h.get_position_status(), CM_LEVEL_ACCURACY);

    msg[15] = 0.0;   // SPP with sub-meter accuracy
    msg[8] = 0.5;
    h.add_gps_message(msg);
    EXPECT_EQ(h.get_position_status(), SUB_METER_LEVEL_ACCURACY);

    msg[8] = 5.0;    // poor accuracy
    h.add_gps_message(msg);
    EXPECT_EQ(h.get_position_status(), GPS_ACC_POOR);
}
