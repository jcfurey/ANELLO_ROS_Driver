// Tests for the $GNGGA sentence builder, fed with arrays in the
// decode_ascii_gps output layout ([1]=GPS ns, [2]=lat, [3]=lon,
// [5]=MSL alt, [10]=PDOP, [11]=fix type, [12]=sats, [15]=RTK status).

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/anello_ros_driver/bit_tools.h"
#include "../src/anello_ros_driver/messaging/ascii_decoder.h"
#include "../src/anello_ros_driver/messaging/message_publisher.h"

namespace
{
// Split the sentence body (everything before '*') on commas, keeping
// empty fields.
std::vector<std::string> split_body(const std::string &sentence)
{
    const size_t star = sentence.rfind('*');
    const std::string body =
        (star == std::string::npos) ? sentence : sentence.substr(0, star);
    std::vector<std::string> out;
    std::string cur;
    for (char c : body) {
        if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(cur);
    return out;
}

// Independent recomputation of the NMEA XOR checksum over the body
// between '$' and '*'.
std::string recompute_checksum(const std::string &sentence)
{
    const size_t star = sentence.rfind('*');
    unsigned char ck = 0;
    for (size_t i = 1; i < star; ++i)
        ck ^= static_cast<unsigned char>(sentence[i]);
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", ck);
    return std::string(hex);
}
}  // namespace

TEST(BuildGgaSentence, RealisticFixFormatsAllFields)
{
    // Decode the known-good APGPS capture first so this test stays
    // coupled to decode_ascii_gps's actual output layout.
    char buf[256];
    char *val[MAXFIELD];
    snprintf(buf, sizeof(buf),
             "#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,"
             "-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,"
             "180.0000,0*65\r");
    ASSERT_GE(parse_fields(buf, val), 17);
    double gps[32] = {};
    decode_ascii_gps(val, gps);

    const std::string s = build_gga_sentence(gps);

    // Framing: talker/type prefix, CRLF terminator, valid checksum.
    ASSERT_GE(s.size(), 9u);
    EXPECT_EQ(s.substr(0, 7), "$GNGGA,");
    EXPECT_EQ(s.substr(s.size() - 2), "\r\n");
    const size_t star = s.rfind('*');
    ASSERT_NE(star, std::string::npos);
    EXPECT_EQ(s.substr(star + 1, 2), recompute_checksum(s));

    const std::vector<std::string> f = split_body(s);
    ASSERT_GE(f.size(), 11u);
    // GPS ns 1343773580500184320 minus the 18 s GPS-UTC leap offset is
    // 22:26:02.500 UTC.
    EXPECT_EQ(f[1], "222602.500");
    // Latitude as DDMM.MMMMMMM + hemisphere; round-trip the packed
    // degrees+minutes value instead of pinning the truncated digits.
    ASSERT_EQ(f[2].size(), 12u);
    EXPECT_EQ(f[3], "N");
    const double lat = std::stod(f[2].substr(0, 2)) +
                       std::stod(f[2].substr(2)) / 60.0;
    EXPECT_NEAR(lat, 37.3988755, 1e-8);
    // Longitude as DDDMM.MMMMMMM + hemisphere.
    ASSERT_EQ(f[4].size(), 13u);
    EXPECT_EQ(f[5], "W");
    const double lon = std::stod(f[4].substr(0, 3)) +
                       std::stod(f[4].substr(3)) / 60.0;
    EXPECT_NEAR(lon, 121.9791327, 1e-8);

    EXPECT_EQ(f[6], "1");     // 3D fix, no RTK -> plain GPS quality
    EXPECT_EQ(f[7], "29");    // satellites
    EXPECT_EQ(f[8], "0.97");  // PDOP, fixed 2 decimals
    EXPECT_EQ(f[9], "1.9");   // MSL altitude, fixed 1 decimal
    EXPECT_EQ(f[10], "M");

    EXPECT_LE(s.size(), 82u);  // NMEA 0183 limit, incl. CRLF
}

TEST(BuildGgaSentence, SouthWestHemispheresUsePositiveFields)
{
    // NMEA encodes sign via the hemisphere letter: negative lat/lon
    // must flip to S/W with positive magnitude fields, never emit a
    // minus sign the NTRIP caster would reject.
    double gps[32] = {};
    gps[1] = 45314.5e9;  // (45314.5 - 18) s -> 12:34:56.500 UTC
    gps[2] = -33.5;      // 33 deg 30.0 min South
    gps[3] = -70.75;     // 70 deg 45.0 min West
    gps[5] = 10.0;
    gps[10] = 1.5;
    gps[11] = 3.0;
    gps[12] = 7.0;
    gps[15] = 0.0;

    const std::string s = build_gga_sentence(gps);
    const std::vector<std::string> f = split_body(s);
    ASSERT_GE(f.size(), 6u);
    EXPECT_EQ(f[1], "123456.500");
    EXPECT_EQ(f[2], "3330.0000000");
    EXPECT_EQ(f[3], "S");
    EXPECT_EQ(f[4], "07045.0000000");
    EXPECT_EQ(f[5], "W");
    EXPECT_EQ(s.substr(s.rfind('*') + 1, 2), recompute_checksum(s));
}

TEST(BuildGgaSentence, FixQualityMapping)
{
    // Per implementation: APGPS FixType is {0 none, 2 2D, 3 3D,
    // 5 time-only} and only 2D/3D carry a position; RTK status then
    // maps {0 -> 1 (GPS), 1 -> 5 (RTK float), 2 -> 4 (RTK fixed)},
    // anything else -> 0. A wrong quality digit makes NTRIP casters
    // and loggers misjudge the fix.
    const struct {
        double fix_type;
        double rtk;
        const char *quality;
    } cases[] = {
        {0.0, 0.0, "0"},  // no fix
        {5.0, 2.0, "0"},  // time-only: no position, RTK flag irrelevant
        {3.0, 0.0, "1"},  // plain 3D fix
        {3.0, 1.0, "5"},  // RTK float
        {3.0, 2.0, "4"},  // RTK fixed
        {2.0, 2.0, "4"},  // 2D fix still carries a position
        {3.0, 7.0, "0"},  // out-of-range RTK value must not index the map
    };
    for (const auto &c : cases) {
        double gps[32] = {};
        gps[1] = 45314.5e9;
        gps[2] = 37.0;
        gps[3] = -122.0;
        gps[11] = c.fix_type;
        gps[12] = 10.0;
        gps[15] = c.rtk;
        const std::vector<std::string> f = split_body(build_gga_sentence(gps));
        ASSERT_GE(f.size(), 7u);
        EXPECT_EQ(f[6], c.quality)
            << "fix_type=" << c.fix_type << " rtk=" << c.rtk;
    }
}

TEST(BuildGgaSentence, WorstCaseMagnitudesStayWithinNmeaLimit)
{
    // NMEA 0183 caps a sentence at 82 characters including CRLF; the
    // NTRIP client's Python NMEAParser rejects anything longer, which
    // would silently stop RTK corrections. Stress every field width.
    double gps[32] = {};
    gps[1] = 45314.5e9;
    gps[2] = -89.9999999;   // widest latitude minutes
    gps[3] = -179.9999999;  // widest longitude degrees+minutes
    gps[5] = -999.9;        // negative MSL altitude
    gps[10] = 99.99;        // widest PDOP
    gps[11] = 3.0;
    gps[12] = 99.0;         // 2-digit satellite count
    gps[15] = 2.0;

    const std::string s = build_gga_sentence(gps);
    EXPECT_LE(s.size(), 82u);
    EXPECT_EQ(s.substr(s.size() - 2), "\r\n");
    EXPECT_EQ(s.substr(s.rfind('*') + 1, 2), recompute_checksum(s));
}
