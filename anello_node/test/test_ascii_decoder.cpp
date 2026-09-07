// Synthetic ASCII message decoding tests, including the firmware
// variants (APIMU/APIM1 with and without the T_Sync field added in
// firmware v1.0.39).

#include <gtest/gtest.h>

#include <cstring>

#include "../src/anello_ros_driver/messaging/protocol_decoder.h"
namespace {
anello::DecodedPacket decode_sentence(const std::string &sample) {
    const auto body=sample.substr(1,sample.find('*')-1);
    const auto frame="#"+body+"*"+compute_checksum(body.data(),body.size());
    anello::DecodedPacket packet{};
    EXPECT_TRUE(anello::decode_ascii_frame(frame,packet));
    return packet;
}
}

TEST(DecodeAsciiGps, ManualExampleSentence)
{
    // Example APGPS capture from the ANELLO driver sources
    const auto packet = decode_sentence(
        "#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,"
        "-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,"
        "180.0000,0*65\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[0], 318213.135);       // MCU time ms
    EXPECT_DOUBLE_EQ(out[2], 37.3988755);       // lat
    EXPECT_DOUBLE_EQ(out[3], -121.9791327);     // lon
    EXPECT_DOUBLE_EQ(out[4], -27.9650);         // alt ellipsoid
    EXPECT_DOUBLE_EQ(out[5], 1.9240);           // alt msl
    EXPECT_DOUBLE_EQ(out[6], 0.0110);           // speed
    EXPECT_DOUBLE_EQ(out[10], 0.9700);          // pdop
    EXPECT_DOUBLE_EQ(out[11], 3.0);             // fix type (3D)
    EXPECT_DOUBLE_EQ(out[12], 29.0);            // sats
    EXPECT_DOUBLE_EQ(out[15], 0.0);             // rtk status
}

TEST(DecodeAsciiImu, ModernFirmwareWithTSync)
{
    // Time, T_Sync, AX..AZ, WX..WZ, OG_WZ, ODO, ODO_Time, Temp
    const auto packet = decode_sentence(
        "#APIMU,1000.5,500.25,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,1.50,999.0,47.05*00\r");

    const auto &out=packet.values;

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
    const auto packet = decode_sentence(
        "#APIMU,1000.5,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,1.50,999.0,47.05*00\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[0], 1000.5);
    EXPECT_DOUBLE_EQ(out[11], 0.0);     // no T_Sync on old firmware
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // fields must not shift
    EXPECT_DOUBLE_EQ(out[10], 47.05);
}

TEST(DecodeAsciiIm1, ModernFirmwareWithTSync)
{
    const auto packet = decode_sentence(
        "#APIM1,1000.5,500.25,0.0344,-0.0128,1.0077,-0.0817,0.0013,"
        "-0.0038,0.0105,47.05*00\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[0], 1000.5);
    EXPECT_DOUBLE_EQ(out[9], 500.25);   // T_Sync
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // ax
    EXPECT_DOUBLE_EQ(out[7], 0.0105);   // og_wz
    EXPECT_DOUBLE_EQ(out[8], 47.05);    // temp
}

TEST(DecodeAsciiIm1, LegacyFirmwareWithoutTSync)
{
    const auto packet = decode_sentence(
        "#APIM1,1000.5,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
        "0.0105,47.05*00\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[9], 0.0);      // T_Sync absent -> 0
    EXPECT_DOUBLE_EQ(out[1], 0.0344);   // ax must not shift
    EXPECT_DOUBLE_EQ(out[8], 47.05);    // temp
}

TEST(DecodeAsciiIns, ManualExampleSentence)
{
    const auto packet = decode_sentence(
        "#APINS,318215,1343773580502990592,1,37.398875500000,"
        "-121.979132700000,-27.965002059937,0.1,-0.2,0.3,"
        "-0.166232,1.773182,0.250746,1*74\r");

    const auto &out=packet.values;

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
    const auto packet = decode_sentence(
        "#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,"
        "36.92845,0.2796,4.00156,303*59\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[2], 2.13);     // relPosN
    EXPECT_DOUBLE_EQ(out[5], 4.19);     // baseline length
    EXPECT_DOUBLE_EQ(out[6], 36.92845); // heading
    EXPECT_DOUBLE_EQ(out[9], 303.0);    // flags
}

TEST(DecodeAsciiCov, AllNineteenValues)
{
    // Distinct values in a physically valid positive-semidefinite covariance.
    const auto packet = decode_sentence(
        "#APCOV,1234.5,1,2,3,0.04,0.05,0.06,7,8,9,"
        "0.10,0.11,0.12,13,14,15,0.16,0.17,0.18*00\r");

    const auto &out=packet.values;

    EXPECT_DOUBLE_EQ(out[0], 1234.5);
    for (int i = 1; i <= 18; ++i)
        EXPECT_DOUBLE_EQ(out[i], (i%6>=1 && i%6<=3)?i:i*0.01) << "index " << i;
}
