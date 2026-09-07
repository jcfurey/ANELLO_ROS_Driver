#ifndef ANELLO_DEVICE_INPUT_H
#define ANELLO_DEVICE_INPUT_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include "bit_tools.h"

namespace anello {
// A ROS RTCM message may bundle complete frames. Validate the entire bundle
// before forwarding any bytes; never forward a partial frame or ASCII tail.
inline bool valid_rtcm_input(const uint8_t *bytes, size_t size, size_t *frame_count=nullptr) {
    if (frame_count) *frame_count=0;
    if (!bytes || size<8 || size>4096) return false;
    size_t count=0;
    for (size_t offset=0;offset<size;) {
        if (size-offset<8 || bytes[offset]!=0xd3 || (bytes[offset+1]&0xfc)) return false;
        const size_t payload=((bytes[offset+1]&3u)<<8)|bytes[offset+2];
        const size_t length=payload+6;
        if (payload<2 || length>size-offset) return false;
        const auto type=(bytes[offset+3]<<4)|(bytes[offset+4]>>4);
        // ANELLO output telemetry is not a GNSS correction stream.
        if (type==0 || type==4058) return false;
        const auto crc=crc24q(bytes+offset,static_cast<int>(length-3));
        const auto tail=offset+length-3;
        if (crc!=((uint32_t{bytes[tail]}<<16)|(uint32_t{bytes[tail+1]}<<8)|bytes[tail+2])) return false;
        offset+=length;
        ++count;
    }
    if (frame_count) *frame_count=count;
    return true;
}

inline bool valid_command_body(const std::string &body) {
    return body.size()>=5 && body.size()<=128 && body.rfind("AP",0)==0 &&
        std::all_of(body.begin(),body.end(),[](unsigned char c) {
            return c>=0x20 && c<=0x7e && c!='#' && c!='*';
        });
}
inline bool read_only_command(const std::string &body) {
    if (!valid_command_body(body)) return false;
    const auto queries={"APPNG","APVER","APSER","APSTA","APPID","APFSN","APFHW"};
    if (std::any_of(queries.begin(),queries.end(),[&body](const char *name){return body==name;})) return true;
    if (body.rfind("APECH,",0)==0) return true;
    if (body.rfind("APCFG,",0)!=0 && body.rfind("APVEH,",0)!=0) return false;
    if (body.size()<9 || (body[6]!='r' && body[6]!='R') || body[7]!=',') return false;
    size_t start=8;
    while (start<body.size()) {
        const auto end=body.find(',',start);
        const auto key=body.substr(start,end==std::string::npos?end:end-start);
        if (key.empty() || key.size()>32 || !std::all_of(key.begin(),key.end(),[](unsigned char c) {
            return (c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='_';
        })) return false;
        if (end==std::string::npos) return true;
        start=end+1;
    }
    return false;
}

// Admission budget, with a bounded burst and no queue or delayed replay.
// Access each instance from one mutually-exclusive callback group.
class TrafficBudget {
public:
    using Clock=std::chrono::steady_clock;
    explicit TrafficBudget(double rate=1, double burst=1):rate_(rate),burst_(burst),tokens_(burst) {}
    bool take(double amount, Clock::time_point now=Clock::now()) {
        if (!std::isfinite(amount) || amount<0 || amount>burst_) return false;
        if (initialized_) tokens_=std::min(burst_,tokens_+std::max(0.0,std::chrono::duration<double>(now-last_).count())*rate_);
        last_=now; initialized_=true;
        if (amount>tokens_) return false;
        tokens_-=amount; return true;
    }
private:
    double rate_,burst_,tokens_;
    bool initialized_=false;
    Clock::time_point last_{};
};
}  // namespace anello
#endif
