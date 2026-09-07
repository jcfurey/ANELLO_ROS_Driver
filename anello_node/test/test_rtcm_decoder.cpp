// Synthetic RTCM (message 4058) decoding tests: packed binary payloads
// built from the scale factors in the ANELLO Developer Manual.

#include <gtest/gtest.h>

#include <cstring>

#include "../src/anello_ros_driver/protocol_types.h"
#include "../src/anello_ros_driver/messaging/rtcm_decoder.h"

namespace
{
// Place a packed payload at buf+5 like the frame parser does
// (3-byte envelope header + 12-bit message number + 4-bit subtype).
template <typename T>
a1buff_t make_a1buff(const T &payload)
{
    a1buff_t a1{};
    memcpy(a1.buf + 5, &payload, sizeof(T));
    a1.nlen = static_cast<int>(sizeof(T)) + 5;
    a1.type = 4058;
    return a1;
}
}  // namespace

TEST(DecodeRtcmImu, CurrentFormatScaleFactors)
{
    rtcm_apimu_t p{};
    p.MCU_Time = 1000000000ULL;   // 1e9 ns -> 1000 ms
    p.Sync_Time = 500000000ULL;   // -> 500 ms
    p.ODO_time = 2000000000ULL;   // -> 2 s
    p.AX = 143165577;             // 1.0 g  (15g/2^31 scale)
    p.AY = -143165577 / 2;        // -0.5 g
    p.AZ = 143165577;
    p.WX = 4772186;               // 1.0 deg/s (450/2^31 scale)
    p.WY = -4772186 * 2;
    p.WZ = 4772186 / 2;
    p.OG_WZ = 4772186;
    p.ODO = 150;                  // 1.5 m/s at 0.01 scale
    p.Temp_C = 2500;              // 25.00 C

    a1buff_t a1 = make_a1buff(p);
    ASSERT_GE(a1.nlen, 61);  // selects the current (Sync_Time) layout

    double out[32] = {};
    decode_rtcm_imu_msg(out, a1);

    EXPECT_DOUBLE_EQ(out[0], 1000.0);        // ms
    EXPECT_NEAR(out[1], 1.0, 1e-8);          // ax g
    EXPECT_NEAR(out[2], -0.5, 1e-8);
    EXPECT_NEAR(out[4], 1.0, 1e-8);          // wx deg/s
    EXPECT_NEAR(out[5], -2.0, 1e-8);
    EXPECT_NEAR(out[6], 0.5, 1e-8);
    EXPECT_NEAR(out[7], 1.0, 1e-8);          // og_wz
    EXPECT_DOUBLE_EQ(out[8], 1.5);           // odo m/s
    EXPECT_DOUBLE_EQ(out[9], 2.0);           // odo time s
    EXPECT_DOUBLE_EQ(out[10], 25.0);         // temp C
    EXPECT_DOUBLE_EQ(out[11], 500.0);        // t_sync ms
}

TEST(DecodeRtcmImu, LegacyFormatWithoutSyncTime)
{
    rtcm_old_apimu_t p{};
    p.MCU_Time = 1000000000ULL;
    p.ODO_time = 2000000000ULL;
    p.AX = 143165577;
    p.WZ = 4772186;
    p.OG_WZ = 4772186;
    p.ODO = 150;
    p.Temp_C = 2500;

    a1buff_t a1 = make_a1buff(p);
    ASSERT_LT(a1.nlen, 61);  // selects the legacy layout

    double out[32] = {};
    decode_rtcm_imu_msg(out, a1);

    EXPECT_DOUBLE_EQ(out[0], 1000.0);
    EXPECT_NEAR(out[1], 1.0, 1e-8);
    EXPECT_NEAR(out[6], 1.0, 1e-8);
    EXPECT_DOUBLE_EQ(out[10], 25.0);
}

TEST(DecodeRtcmGps, ScalesAndAccuracyFieldOrder)
{
    rtcm_apgps_t p{};
    p.Time = 1000000000ULL;
    p.GPS_Time = 1343773580500184320ULL;
    p.Latitude = 374000000;        // 37.4 deg at 1e-7
    p.Longitude = -1219000000;     // -121.9 deg
    p.Alt_ellipsoid = -27965;      // -27.965 m at 0.001
    p.Alt_msl = 1924;
    p.Speed = 1500;                // 1.5 m/s
    p.Heading = 90000;             // 90.0 deg at 1e-3
    p.Hor_Acc = 110;               // 0.110 m
    p.Ver_Acc = 380;               // 0.380 m
    p.Hdg_Acc = 12345678;          // 123.45678 deg at 1e-5
    p.Spd_Acc = 82;                // 0.082 m/s at 1e-3
    p.PDOP = 97;                   // 0.97
    p.FixType = 3;
    p.SatNum = 29;
    p.RTK_Status = 2;
    p.Antenna_ID = 0;

    a1buff_t a1 = make_a1buff(p);
    double out[32] = {};
    int ant = decode_rtcm_gps_msg(out, a1);

    EXPECT_EQ(ant, GPS1);
    EXPECT_NEAR(out[2], 37.4, 1e-9);
    EXPECT_NEAR(out[3], -121.9, 1e-9);
    EXPECT_DOUBLE_EQ(out[4], -27.965);
    EXPECT_DOUBLE_EQ(out[6], 1.5);
    EXPECT_DOUBLE_EQ(out[7], 90.0);
    EXPECT_DOUBLE_EQ(out[8], 0.110);
    EXPECT_DOUBLE_EQ(out[9], 0.380);
    // Binary places Hdg_Acc before Spd_Acc; decoded order must match
    // the ASCII convention (speed acc at [13], heading acc at [14]).
    EXPECT_DOUBLE_EQ(out[13], 0.082);
    EXPECT_NEAR(out[14], 123.45678, 1e-9);
    EXPECT_DOUBLE_EQ(out[11], 3.0);
    EXPECT_DOUBLE_EQ(out[15], 2.0);

    p.Antenna_ID = 1;
    a1 = make_a1buff(p);
    EXPECT_EQ(decode_rtcm_gps_msg(out, a1), GPS2);
}

TEST(DecodeRtcmIns, ZuptBeforeStatusLayout)
{
    rtcm_apins_t p{};
    p.Time = 2000000000ULL;        // 2000 ms
    p.GPS_Time = 1343773580502990592ULL;
    p.Latitude = 374000000;
    p.Longitude = -1219000000;
    p.Alt_ellipsoid = 10000;       // 10 m
    p.Vn = 1000;                   // 1.0 m/s
    p.Ve = -2000;
    p.Vd = 500;
    p.Roll = -16623;               // -0.16623 deg at 1e-5
    p.Pitch = 177318;
    p.Heading_Yaw = 25075;
    p.ZUPT = 1;                    // binary layout: ZUPT then Status
    p.Status = 4;

    a1buff_t a1 = make_a1buff(p);
    double out[32] = {};
    decode_rtcm_ins_msg(out, a1);

    EXPECT_DOUBLE_EQ(out[0], 2000.0);
    EXPECT_DOUBLE_EQ(out[2], 4.0);           // status
    EXPECT_NEAR(out[3], 37.4, 1e-9);
    EXPECT_DOUBLE_EQ(out[6], 1.0);           // vn
    EXPECT_DOUBLE_EQ(out[7], -2.0);          // ve
    EXPECT_NEAR(out[9], -0.16623, 1e-9);     // roll
    EXPECT_DOUBLE_EQ(out[12], 1.0);          // zupt
}

TEST(DecodeRtcmHdg, BaselineAccuracyTenthMillimeterScale)
{
    rtcm_aphdr_t p{};
    p.MCU_Time = 1000000000ULL;
    p.GPS_Time = 1362269876750000128ULL;
    p.relPosN = 213;                // 2.13 m at 0.01
    p.relPosE = 160;
    p.resPosD = 323;
    p.relPosLength = 419;           // 4.19 m
    p.relPosHeading = 3692845;      // 36.92845 deg at 1e-5
    p.relPosLength_Accuracy = 2796; // 0.2796 m at 0.1 mm
    p.relPosHeading_Accuracy = 400156;  // 4.00156 deg at 1e-5
    p.statusFlags = 0x0137;

    a1buff_t a1 = make_a1buff(p);
    double out[32] = {};
    decode_rtcm_hdg_msg(out, a1);

    EXPECT_DOUBLE_EQ(out[2], 2.13);
    EXPECT_DOUBLE_EQ(out[5], 4.19);
    EXPECT_NEAR(out[6], 36.92845, 1e-9);
    EXPECT_NEAR(out[7], 0.2796, 1e-9);
    EXPECT_NEAR(out[8], 4.00156, 1e-9);
    EXPECT_DOUBLE_EQ(out[9], static_cast<double>(0x0137));
}

TEST(DecodeRtcmCov, FloatFieldsPassThrough)
{
    rtcm_apcov_t p{};
    p.Time = 1000000000ULL;
    p.covLatLat = 0.01f;
    p.covLonLon = 0.02f;
    p.covAltAlt = 0.03f;
    p.covVnVn = 0.07f;
    p.covYawYaw = 0.15f;
    p.covPitchYaw = 0.18f;

    a1buff_t a1 = make_a1buff(p);
    double out[32] = {};
    decode_rtcm_cov_msg(out, a1);

    EXPECT_DOUBLE_EQ(out[0], 1000.0);
    EXPECT_NEAR(out[1], 0.01, 1e-7);
    EXPECT_NEAR(out[2], 0.02, 1e-7);
    EXPECT_NEAR(out[3], 0.03, 1e-7);
    EXPECT_NEAR(out[7], 0.07, 1e-7);
    EXPECT_NEAR(out[15], 0.15, 1e-7);
    EXPECT_NEAR(out[18], 0.18, 1e-7);
}
