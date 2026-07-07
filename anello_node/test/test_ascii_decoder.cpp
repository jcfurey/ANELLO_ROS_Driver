// Synthetic ASCII message decoding tests, including the firmware
// variants (APIMU/APIM1 with and without the T_Sync field added in
// firmware v1.0.39).

#include <gtest/gtest.h>

#include <cstring>

#include "../src/anello_ros_driver/bit_tools.h"
#include "../src/anello_ros_driver/messaging/ascii_decoder.h"

namespace
{
// Parse a writable copy of the sentence and return the field count.
int fields_from(const char *sentence, char *buf, size_t buf_len, char **val)
{
    snprintf(buf, buf_len, "%s", sentence);
    return parse_fields(buf, val);
}
}  // namespace

TEST(DecodeAsciiGps, ManualExampleSentence)
{
    // Example APGPS capture from the ANELLO driver sources
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,"
        "-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,"
        "180.0000,0*65\r",
        buf, sizeof(buf), val);
    ASSERT_GE(n, 17);

    double out[32] = {};
    decode_ascii_gps(val, out);

    EXPECT_DOUBLE_EQ(out[0], 318213.135);       // MCU time ms
    EXPECT_DOUBLE_EQ(out[2], 37.3988755);       // lat
    EXPECT_DOUBLE_EQ(out[3], -121.9791327);     // lon
    EXPECT_DOUBLE_EQ(out[4], -27.9650);         // alt ellipsoid
    EXPECT_DOUBLE_EQ(out[5], 1.9240);           // alt msl
    EXPECT_DOUBLE_EQ(out[6], 0.0110);           // speed
    EXPECT_DOUBLE_EQ(out[7], 0.0);              // heading
    EXPECT_DOUBLE_EQ(out[8], 0.2380);           // horizontal accuracy
    EXPECT_DOUBLE_EQ(out[9], 0.3820);           // vertical accuracy
    EXPECT_DOUBLE_EQ(out[10], 0.9700);          // pdop
    EXPECT_DOUBLE_EQ(out[11], 3.0);             // fix type (3D)
    EXPECT_DOUBLE_EQ(out[12], 29.0);            // sats
    // Accuracy pair order matters: publish_gps maps [13] to
    // speed_accuracy and [14] to heading_accuracy — swapping them
    // corrupts both APGPS fields silently.
    EXPECT_DOUBLE_EQ(out[13], 0.0820);          // speed accuracy
    EXPECT_DOUBLE_EQ(out[14], 180.0);           // heading accuracy
    EXPECT_DOUBLE_EQ(out[15], 0.0);             // rtk status
}

TEST(DecodeAsciiImu, ModernFirmwareWithTSync)
{
    // Time, T_Sync, AX..AZ, WX..WZ, OG_WZ, ODO, ODO_Time, Temp
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APIMU,1000.5,500.25,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,1.50,999.0,47.05*00\r",
        buf, sizeof(buf), val);
    ASSERT_EQ(n, 15);  // name + 12 data + checksum + remainder

    double out[32] = {};
    decode_ascii_imu(val, n, out);

    EXPECT_DOUBLE_EQ(out[0], 1000.5);    // MCU time ms
    EXPECT_DOUBLE_EQ(out[11], 500.25);   // T_Sync ms
    EXPECT_DOUBLE_EQ(out[1], 0.0344);    // ax (g)
    EXPECT_DOUBLE_EQ(out[3], 1.0077);    // az (g)
    EXPECT_DOUBLE_EQ(out[6], -0.0038);   // wz (deg/s)
    EXPECT_DOUBLE_EQ(out[7], 0.0105);    // og_wz (deg/s)
    EXPECT_DOUBLE_EQ(out[8], 1.50);      // odometer speed (m/s)
    EXPECT_DOUBLE_EQ(out[9], 0.999);     // odometer time ms -> s
    EXPECT_DOUBLE_EQ(out[10], 47.05);    // temp (C)
}

TEST(DecodeAsciiImu, LegacyFirmwareWithoutTSync)
{
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APIMU,1000.5,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,1.50,999.0,47.05*00\r",
        buf, sizeof(buf), val);
    ASSERT_EQ(n, 14);

    double out[32] = {};
    decode_ascii_imu(val, n, out);

    EXPECT_DOUBLE_EQ(out[0], 1000.5);
    EXPECT_DOUBLE_EQ(out[11], 0.0);     // no T_Sync on old firmware
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // fields must not shift
    EXPECT_DOUBLE_EQ(out[10], 47.05);
}

TEST(DecodeAsciiIm1, ModernFirmwareWithTSync)
{
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APIM1,1000.5,500.25,0.0344,-0.0128,1.0077,-0.0817,0.0013,"
        "-0.0038,0.0105,47.05*00\r",
        buf, sizeof(buf), val);
    ASSERT_EQ(n, 13);

    double out[32] = {};
    decode_ascii_im1(val, n, out);

    EXPECT_DOUBLE_EQ(out[0], 1000.5);
    EXPECT_DOUBLE_EQ(out[9], 500.25);   // T_Sync
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // ax
    EXPECT_DOUBLE_EQ(out[7], 0.0105);   // og_wz
    EXPECT_DOUBLE_EQ(out[8], 47.05);    // temp
}

TEST(DecodeAsciiIm1, LegacyFirmwareWithoutTSync)
{
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APIM1,1000.5,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,47.05*00\r",
        buf, sizeof(buf), val);
    ASSERT_EQ(n, 12);

    double out[32] = {};
    decode_ascii_im1(val, n, out);

    EXPECT_DOUBLE_EQ(out[9], 0.0);      // T_Sync absent -> 0
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // ax must not shift
    EXPECT_DOUBLE_EQ(out[8], 47.05);    // temp
}

TEST(DecodeAsciiIns, ManualExampleSentence)
{
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APINS,318215,1343773580502990592,1,37.398875500000,"
        "-121.979132700000,-27.965002059937,0.1,-0.2,0.3,"
        "-0.166232,1.773182,0.250746,1*74\r",
        buf, sizeof(buf), val);
    ASSERT_GE(n, 14);

    double out[32] = {};
    decode_ascii_ins(val, out);

    EXPECT_DOUBLE_EQ(out[0], 318215.0);          // MCU time
    EXPECT_DOUBLE_EQ(out[2], 1.0);               // status
    EXPECT_DOUBLE_EQ(out[3], 37.3988755);        // lat deg
    EXPECT_DOUBLE_EQ(out[6], 0.1);               // vn
    EXPECT_DOUBLE_EQ(out[8], 0.3);               // vd
    EXPECT_DOUBLE_EQ(out[9], -0.166232);         // roll
    EXPECT_DOUBLE_EQ(out[11], 0.250746);         // heading
    EXPECT_DOUBLE_EQ(out[12], 1.0);              // zupt
}

TEST(DecodeAsciiHdg, FieldOrderAndFlags)
{
    char buf[256];
    char *val[MAXFIELD];
    int n = fields_from(
        "#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,"
        "36.92845,0.2796,4.00156,303*59\r",
        buf, sizeof(buf), val);
    ASSERT_GE(n, 12);

    double out[32] = {};
    decode_ascii_hdr(val, out);

    EXPECT_DOUBLE_EQ(out[2], 2.13);     // relPosN
    EXPECT_DOUBLE_EQ(out[3], 1.60);     // relPosE
    EXPECT_DOUBLE_EQ(out[4], 3.23);     // relPosD
    EXPECT_DOUBLE_EQ(out[5], 4.19);     // baseline length
    EXPECT_DOUBLE_EQ(out[6], 36.92845); // heading
    // Accuracy order: [7] is baseline-length accuracy, [8] heading
    // accuracy — publish_hdr and the health monitor index them by
    // position, so a swap would go unnoticed downstream.
    EXPECT_DOUBLE_EQ(out[7], 0.2796);   // baseline length accuracy
    EXPECT_DOUBLE_EQ(out[8], 4.00156);  // heading accuracy
    EXPECT_DOUBLE_EQ(out[9], 303.0);    // flags
}

TEST(DecodeAsciiCov, AllNineteenValues)
{
    char buf[512];
    char *val[MAXFIELD];
    // mcu_time then 18 covariance values 0.01 .. 0.18
    int n = fields_from(
        "#APCOV,1234.5,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,"
        "0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18*00\r",
        buf, sizeof(buf), val);
    ASSERT_GE(n, 20);

    double out[32] = {};
    decode_ascii_cov(val, out);

    EXPECT_DOUBLE_EQ(out[0], 1234.5);
    for (int i = 1; i <= 18; ++i)
        EXPECT_DOUBLE_EQ(out[i], i * 0.01) << "index " << i;
}
