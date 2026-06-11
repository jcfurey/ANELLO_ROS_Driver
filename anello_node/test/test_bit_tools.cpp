// Synthetic-data tests for the ASCII checksum, field parser, and bit
// utilities, using the worked examples published in the ANELLO
// Developer Manual as gold vectors.

#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "../src/anello_ros_driver/bit_tools.h"

TEST(ComputeChecksum, ManualGoldVectors)
{
    // Every pair below appears verbatim in the ANELLO Developer Manual.
    const struct { const char *body; const char *ck; } vectors[] = {
        {"APPNG", "48"},
        {"APPNG,0", "54"},
        {"APRST,0", "58"},
        {"APODO,-,24", "7E"},
        {"APODO,-24", "52"},
        {"APODO,-,-24", "53"},
        {"APVEH,R,bsl", "65"},
    };
    for (const auto &v : vectors) {
        EXPECT_EQ(compute_checksum(v.body, static_cast<int>(strlen(v.body))),
                  std::string(v.ck))
            << "body: " << v.body;
    }
}

TEST(Checksum, AcceptsValidFrameRejectsCorrupt)
{
    char good[] = "#APPNG*48\r";
    EXPECT_NE(checksum(reinterpret_cast<unsigned char *>(good),
                       static_cast<int>(strlen(good))), 0);

    char bad[] = "#APPNG*49\r";
    EXPECT_EQ(checksum(reinterpret_cast<unsigned char *>(bad),
                       static_cast<int>(strlen(bad))), 0);

    // Payload corruption with the original checksum must fail
    char tampered[] = "#APPNF*48\r";
    EXPECT_EQ(checksum(reinterpret_cast<unsigned char *>(tampered),
                       static_cast<int>(strlen(tampered))), 0);
}

TEST(ParseFields, SplitsNameFieldsChecksum)
{
    char buf[] = "#APGPS,1.0,2.0,3.0*5A\r";
    char *val[MAXFIELD];
    int n = parse_fields(buf, val);

    // name + 3 data + checksum + trailing remainder
    ASSERT_GE(n, 5);
    EXPECT_STREQ(val[0], "#APGPS");
    EXPECT_STREQ(val[1], "1.0");
    EXPECT_STREQ(val[2], "2.0");
    EXPECT_STREQ(val[3], "3.0");
    EXPECT_STREQ(val[4], "5A");
}

TEST(ParseFields, EmptyFieldsPreserved)
{
    // APINS from the manual can carry empty velocity fields
    char buf[] = "#APINS,1,2,3,,,6*00\r";
    char *val[MAXFIELD];
    int n = parse_fields(buf, val);

    ASSERT_GE(n, 7);
    EXPECT_STREQ(val[4], "");
    EXPECT_STREQ(val[5], "");
    EXPECT_STREQ(val[6], "6");
}

TEST(BitUtils, GetSetRoundTrip)
{
    unsigned char buf[8] = {};
    setbitu(buf, 14, 10, 0x2A5u);
    EXPECT_EQ(getbitu(buf, 14, 10), 0x2A5u);

    setbitu(buf, 24, 12, 4058u);  // ANELLO RTCM message number
    EXPECT_EQ(getbitu(buf, 24, 12), 4058u);
}

TEST(BitUtils, GetBitsSignExtends)
{
    unsigned char buf[4] = {};
    setbitu(buf, 0, 8, 0xFFu);
    EXPECT_EQ(getbits(buf, 0, 8), -1);
    setbitu(buf, 8, 8, 0x7Fu);
    EXPECT_EQ(getbits(buf, 8, 8), 127);
}

TEST(Crc24q, MatchesFrameRoundTrip)
{
    // CRC of the empty message is 0 (init value), and any single-bit
    // corruption must change the CRC.
    unsigned char frame[16] = {0xD3, 0x00, 0x0A, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    EXPECT_EQ(crc24q(frame, 0), 0u);
    unsigned int crc = crc24q(frame, 13);
    EXPECT_NE(crc, 0u);

    frame[5] ^= 0x01;
    EXPECT_NE(crc24q(frame, 13), crc);
}
