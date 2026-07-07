// Byte-level tests for the ASCII/RTCM frame state machine
// (input_a1_data): terminator handling, the single-recursion header
// resync fix, RTCM framing/CRC, and the buffer-overflow guard.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "../src/anello_ros_driver/bit_tools.h"
#include "../src/anello_ros_driver/messaging/frame_parser.h"

namespace
{
// Real APIMU capture from the ANELLO driver sources (checksum 0x55).
const char kImuFrame[] =
    "#APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,"
    "0.01051,0.0000,318214.548,47.0547*55\r\n";

// Feed every byte of a stream, recording how many frames completed
// (ret==1), the stream index of the last completion, and the buffered
// length at that moment (later bytes keep mutating a1->nbyte).
int feed_ascii(a1buff_t &a1, const std::string &stream,
               size_t *complete_at, int *nbyte_at_complete)
{
    int completions = 0;
    for (size_t i = 0; i < stream.size(); ++i) {
        if (input_a1_data(&a1, static_cast<uint8_t>(stream[i])) == 1) {
            completions++;
            if (complete_at != nullptr)
                *complete_at = i;
            if (nbyte_at_complete != nullptr)
                *nbyte_at_complete = a1.nbyte;
        }
    }
    return completions;
}

// Build a synthetic ANELLO RTCM frame: 0xD3 preamble, 10-bit payload
// length, 12-bit message number 4058, 4-bit subtype, filler payload,
// and a trailing crc24q over header+payload — the same layout
// test_rtcm_decoder.cpp assumes at buf+5.
std::vector<uint8_t> make_rtcm_frame(unsigned int subtype, int payload_len = 10)
{
    std::vector<uint8_t> f(3 + payload_len + 3, 0);
    setbitu(f.data(), 0, 8, 0xD3);
    setbitu(f.data(), 14, 10, static_cast<unsigned int>(payload_len));
    setbitu(f.data(), 24, 12, 4058u);
    setbitu(f.data(), 36, 4, subtype);
    for (int i = 5; i < 3 + payload_len; ++i)
        f[i] = static_cast<uint8_t>(0xA0 + i);  // arbitrary filler
    const unsigned int crc = crc24q(f.data(), 3 + payload_len);
    setbitu(f.data(), (3 + payload_len) * 8, 24, crc);
    return f;
}
}  // namespace

TEST(FrameParser, AsciiFrameCompletesExactlyOnTerminator)
{
    a1buff_t a1{};
    const std::string stream(kImuFrame);

    size_t complete_at = 0;
    int nbyte = 0;
    EXPECT_EQ(feed_ascii(a1, stream, &complete_at, &nbyte), 1);
    // The device terminates with "\r\n": the frame must be ready on the
    // '\r' (first terminator byte), so the caller never waits on the
    // '\n' of a message that is already whole.
    EXPECT_EQ(complete_at, stream.size() - 2);
    // The buffered frame ("...*55\r") must pass the ASCII XOR checksum
    // exactly as handed to the decode path.
    EXPECT_NE(checksum(a1.buf, nbyte), 0);
}

TEST(FrameParser, ResyncAfterFalseHeaderPrefix)
{
    // Regression for the single-recursion header-resync fix: a '#' that
    // aborts a false "#A" header must itself open the next frame, or
    // streams like "#A#APIMU,..." silently drop the valid message.
    a1buff_t a1{};
    const std::string stream = std::string("#A") + kImuFrame;

    size_t complete_at = 0;
    int nbyte = 0;
    EXPECT_EQ(feed_ascii(a1, stream, &complete_at, &nbyte), 1);
    EXPECT_EQ(complete_at, stream.size() - 2);
    EXPECT_NE(checksum(a1.buf, nbyte), 0);
}

TEST(FrameParser, RtcmFrameCompletesWithCleanCrc)
{
    a1buff_t a1{};
    const std::vector<uint8_t> frame = make_rtcm_frame(/*subtype=*/1);

    int ret = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
        ret = input_a1_data(&a1, frame[i]);
        if (i + 1 < frame.size()) {
            ASSERT_EQ(ret, 0) << "frame reported ready early at byte " << i;
        }
    }
    // Binary frames complete only once the length-declared payload plus
    // the 3 CRC bytes have arrived.
    EXPECT_EQ(ret, 5);
    EXPECT_EQ(a1.crc, 0);
    EXPECT_EQ(a1.type, 4058);
    EXPECT_EQ(a1.subtype, 1);
}

TEST(FrameParser, RtcmPayloadCorruptionSetsCrcFlag)
{
    a1buff_t a1{};
    std::vector<uint8_t> frame = make_rtcm_frame(/*subtype=*/1);
    frame[7] ^= 0x01;  // corrupt one filler payload byte, not the type

    int ret = 0;
    for (uint8_t b : frame)
        ret = input_a1_data(&a1, b);
    // The frame still completes (framing is length-based), but crc==1
    // must mark it so the caller can drop it instead of decoding trash.
    EXPECT_EQ(ret, 5);
    EXPECT_EQ(a1.crc, 1);
    EXPECT_EQ(a1.type, 4058);  // type is parsed before the CRC verdict
}

TEST(FrameParser, OversizedFrameResetsWithoutOverflow)
{
    a1buff_t a1{};
    EXPECT_EQ(input_a1_data(&a1, '#'), 0);
    EXPECT_EQ(input_a1_data(&a1, 'A'), 0);
    EXPECT_EQ(input_a1_data(&a1, 'P'), 0);

    // A runaway "frame" with no terminator: the parser must reset its
    // state instead of writing past the buffer, and must never report
    // a completed frame from the garbage.
    int completions = 0;
    for (int i = 0; i < MAX_BUF_LEN + 50; ++i)
        completions += (input_a1_data(&a1, 'X') != 0) ? 1 : 0;
    EXPECT_EQ(completions, 0);
    // The reset happens one byte early so buf[MAX_BUF_LEN-1] stays an
    // untouched NUL — a full buffer must never reach "%s" logging or
    // parse_fields() unterminated.
    EXPECT_EQ(a1.buf[MAX_BUF_LEN - 1], 0);

    // The stream recovers: the next valid frame still decodes.
    size_t complete_at = 0;
    int nbyte = 0;
    const std::string stream(kImuFrame);
    EXPECT_EQ(feed_ascii(a1, stream, &complete_at, &nbyte), 1);
    EXPECT_EQ(complete_at, stream.size() - 2);
    EXPECT_NE(checksum(a1.buf, nbyte), 0);
}

TEST(FrameParser, NewlineOnlyTerminatorAlsoCompletesFrame)
{
    // The implementation accepts either '\r' or '\n' after "*CK" (the
    // terminator test is data=='\r' || data=='\n'), so a stream whose
    // CR was stripped by a transport still frames. Assert that
    // implemented behavior.
    std::string stream(kImuFrame);
    stream = stream.substr(0, stream.size() - 2) + "\n";

    a1buff_t a1{};
    size_t complete_at = 0;
    int nbyte = 0;
    EXPECT_EQ(feed_ascii(a1, stream, &complete_at, &nbyte), 1);
    EXPECT_EQ(complete_at, stream.size() - 1);
    EXPECT_NE(checksum(a1.buf, nbyte), 0);
}
