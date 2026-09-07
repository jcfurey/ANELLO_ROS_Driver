#ifndef ANELLO_PROTOCOL_DECODER_H
#define ANELLO_PROTOCOL_DECODER_H

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include "../protocol_types.h"

namespace anello {
enum class MessageKind { imu, im1, gps, gps2, heading, ins, covariance, ahrs };
// The legacy array layout is confined to the protocol/publication boundary.
struct DecodedPacket {
    MessageKind kind;
    std::array<double, MAXFIELD> values{};
};
class StreamDecoder {
public:
    using Callback = std::function<void(const DecodedPacket &)>;
    void feed(uint8_t byte, const Callback &callback);
    void reset() { buffer_.clear(); }
    bool pending() const { return !buffer_.empty(); }
    uint64_t checksum_failures = 0;
    uint64_t parse_failures = 0;
private:
    std::string buffer_;
};
bool decode_ascii_frame(const std::string &frame, DecodedPacket &packet);
bool decode_binary_frame(const a1buff_t &frame, DecodedPacket &packet);
bool validate_packet(const DecodedPacket &packet);
bool ins_position_valid(const double *ins);
}  // namespace anello
#endif
