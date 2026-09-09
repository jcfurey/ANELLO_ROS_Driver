#include "protocol_decoder.h"
#include "rtcm_decoder.h"
#include "../bit_tools.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

namespace anello {
namespace {
bool integer(double value, double lo, double hi) {
    return std::isfinite(value) && value >= lo && value <= hi && std::floor(value) == value;
}
bool one_of(double value, std::initializer_list<int> allowed) {
    return std::any_of(allowed.begin(), allowed.end(),
        [value](int v) { return value == v; });
}
bool latitude(double v) { return std::isfinite(v) && std::abs(v) <= 90; }
bool longitude(double v) { return std::isfinite(v) && std::abs(v) <= 180; }
bool psd(const double *v) {
    // Packed symmetric matrix: three diagonals followed by xy,xz,yz.
    if (v[0] < 0 || v[1] < 0 || v[2] < 0 || !std::isfinite(v[0]+v[1]+v[2])) return false;
    // Check correlations per axis pair; a large third-axis variance must not
    // hide an indefinite block on the two quieter axes.
    constexpr double eps=1e-6;  // allow binary float rounding
    double correlations[3]{};
    int pair=0;
    for (int i=0;i<3;++i) for (int j=i+1;j<3;++j,++pair) {
        const double scale=std::sqrt(v[i])*std::sqrt(v[j]);
        if (scale==0) {
            if (v[3+pair]!=0) return false;
        } else {
            correlations[pair]=v[3+pair]/scale;
            if (!std::isfinite(correlations[pair]) || std::abs(correlations[pair])>1+eps)
                return false;
        }
    }
    const double d=correlations[0], e=correlations[1], f=correlations[2];
    return 1+2*d*e*f-d*d-e*e-f*f >= -eps;
}
bool ascii_checksum(const std::string &frame) {
    return frame.size() <= 1200 && frame.find_first_of("\r\n") == std::string::npos &&
        checksum(reinterpret_cast<const unsigned char *>(frame.data()), frame.size());
}
}  // namespace

bool ins_position_valid(const double *v) {
    return one_of(v[2], {1,2,3,4,9,10}) && latitude(v[3]) && longitude(v[4]) &&
        std::isfinite(v[5]) && v[5]>=-1e6 && v[5]<=1e8;
}

bool validate_packet(const DecodedPacket &p) {
    const auto &v=p.values;
    // Conversion to int64 nanoseconds must be bounded even after translation.
    if (!std::isfinite(v[0]) || v[0]<0 || v[0]>9e12) return false;
    size_t count=0;
    switch (p.kind) {
    case MessageKind::imu: count=12; break;
    case MessageKind::im1: count=10; break;
    case MessageKind::gps: case MessageKind::gps2: count=16; break;
    case MessageKind::heading: count=10; break;
    case MessageKind::ins: count=13; break;
    case MessageKind::covariance: count=19; break;
    case MessageKind::ahrs: count=6; break;
    }
    for (size_t i=1;i<count;++i) {
        // Missing INS position (attitude-only) and velocity are unavailable,
        // never silently converted to zero. Literal NaN is rejected by ASCII parsing.
        if (p.kind==MessageKind::ins && i>=3 && i<=8 && std::isnan(v[i])) {
            if (i>=6 || one_of(v[2],{0,8})) continue;
        }
        if (!std::isfinite(v[i])) return false;
    }
    switch (p.kind) {
    case MessageKind::gps: case MessageKind::gps2:
        return v[1]>=0 && v[1]<=1.84e19 && latitude(v[2]) && longitude(v[3]) && v[6]>=0 &&
            v[4]>=-1e6 && v[4]<=1e8 && std::isfinite(v[8]*v[8]+v[9]*v[9]) &&
            v[8]>=0 && v[9]>=0 && v[10]>=0 && v[13]>=0 && v[14]>=0 &&
            one_of(v[11],{0,2,3,5}) && integer(v[12],0,255) && one_of(v[15],{0,1,2});
    case MessageKind::ins:
        return v[1]>=0 && v[1]<=1.84e19 && one_of(v[2],{0,1,2,3,4,8,9,10}) &&
            (one_of(v[2],{0,8}) || ins_position_valid(v.data())) &&
            (std::isnan(v[6]) || std::abs(v[6])<=1e6) &&
            (std::isnan(v[7]) || std::abs(v[7])<=1e6) && (std::isnan(v[8]) || std::abs(v[8])<=1e6) &&
            std::abs(v[9])<=180 && std::abs(v[10])<=90 && std::abs(v[11])<=360 &&
            one_of(v[12],{0,1});
    case MessageKind::heading:
        return v[1]>=0 && v[1]<=1.84e19 && v[5]>=0 && v[7]>=0 && v[8]>=0 && integer(v[9],0,65535);
    case MessageKind::covariance: return psd(v.data()+1) && psd(v.data()+7) && psd(v.data()+13);
    case MessageKind::imu: return std::all_of(v.begin()+1,v.begin()+8,[](double x){return std::abs(x)<=1e6;}) && v[9]>=0 && v[11]>=0;
    case MessageKind::im1: return std::all_of(v.begin()+1,v.begin()+8,[](double x){return std::abs(x)<=1e6;}) && v[9]>=0;
    case MessageKind::ahrs:
        return v[1]>=0 && std::abs(v[2])<=180 && std::abs(v[3])<=90 &&
            std::abs(v[4])<=360 && one_of(v[5],{0,1});
    }
    return false;
}

bool decode_ascii_frame(const std::string &frame, DecodedPacket &p) {
    if (!ascii_checksum(frame)) return false;
    const auto star=frame.find('*');
    std::vector<std::string> fields;
    size_t start=1;
    for (size_t i=1;i<=star;++i) {
        if (i==star || frame[i]==',') {
            fields.emplace_back(frame.substr(start,i-start)); start=i+1;
        }
    }
    const auto &name=fields.front();
    const size_t n=fields.size()-1;
    if (name=="APIMU" && (n==11 || n==12)) p.kind=MessageKind::imu;
    else if (name=="APIM1" && (n==9 || n==10)) p.kind=MessageKind::im1;
    else if (name=="APGPS" && n==16) p.kind=MessageKind::gps;
    else if (name=="APGP2" && n==16) p.kind=MessageKind::gps2;
    else if (name=="APHDG" && n==10) p.kind=MessageKind::heading;
    else if (name=="APINS" && n==13) p.kind=MessageKind::ins;
    else if (name=="APCOV" && n==19) p.kind=MessageKind::covariance;
    else if (name=="APAHRS" && n==6) p.kind=MessageKind::ahrs;
    else return false;
    std::array<double,MAXFIELD> parsed{};
    for (size_t i=1;i<fields.size();++i) {
        if (fields[i].empty() && p.kind==MessageKind::ins && i>=4 && i<=9) {
            parsed[i-1]=std::numeric_limits<double>::quiet_NaN(); continue;
        }
        // Parse a complete decimal field independently of process locale.
        if (fields[i].empty() || fields[i].find_first_not_of("+-0123456789.eE")!=std::string::npos)
            return false;
        const char *begin=fields[i].data(), *end=begin+fields[i].size();
        if (*begin=='+') {
            ++begin;
            if (begin==end || *begin=='-' || *begin=='+') return false;
        }
        const auto conversion=std::from_chars(begin,end,parsed[i-1],std::chars_format::general);
        if (conversion.ec!=std::errc() || conversion.ptr!=end || !std::isfinite(parsed[i-1])) return false;
    }
    p.values=parsed;
    if (p.kind==MessageKind::imu || p.kind==MessageKind::im1) {
        bool imu=p.kind==MessageKind::imu;
        bool sync=n==(imu?12u:10u);
        p.values[0]=parsed[0];
        for (size_t i=1;i<=(imu?10u:8u);++i) p.values[i]=parsed[i+(sync?1:0)];
        p.values[imu?11:9]=sync?parsed[1]:0;
        if (imu) p.values[9]*=1e-3;
    }
    if (p.kind==MessageKind::ins) p.values[13]=p.values[1]*1e-9;
    return validate_packet(p);
}

bool decode_binary_frame(const a1buff_t &f, DecodedPacket &p) {
    if (f.type!=4058 || f.crc) return false;
    auto exact=[&f](size_t n) { return f.nlen==static_cast<int>(n)+5; };
    auto *v=p.values.data(); p.values.fill(0);
    switch (f.subtype) {
    case 1:
        if (!exact(sizeof(rtcm_old_apimu_t)) && !exact(sizeof(rtcm_apimu_t))) return false;
        p.kind=MessageKind::imu; decode_rtcm_imu_msg(v,f); break;
    case 2: {
        if (!exact(sizeof(rtcm_apgps_t))) return false;
        rtcm_apgps_t gps{}; std::memcpy(&gps,f.buf+5,sizeof(gps));
        if (gps.Antenna_ID>1) return false;
        p.kind=decode_rtcm_gps_msg(v,f)==GPS1?MessageKind::gps:MessageKind::gps2; break;
    }
    case 3:
        if (!exact(sizeof(rtcm_aphdr_t))) return false;
        p.kind=MessageKind::heading; decode_rtcm_hdg_msg(v,f); break;
    case 4:
        if (!exact(sizeof(rtcm_apins_t))) return false;
        p.kind=MessageKind::ins; decode_rtcm_ins_msg(v,f); v[13]=v[1]*1e-9; break;
    case 6:
        if (!exact(sizeof(rtcm_apim1_t))) return false;
        p.kind=MessageKind::im1; decode_rtcm_im1_msg(v,f); break;
    case 8: {
        if (!exact(sizeof(rtcm_apahrs_t))) return false;
        rtcm_apahrs_t a{}; std::memcpy(&a,f.buf+5,sizeof(a));
        p.kind=MessageKind::ahrs;
        v[0]=a.Time*1e-6; v[1]=a.SyncTime; v[2]=a.Roll*1e-5;
        v[3]=a.Pitch*1e-5; v[4]=a.Yaw*1e-5; v[5]=a.ZUPT; break;
    }
    case 10:
        if (!exact(sizeof(rtcm_apcov_t))) return false;
        p.kind=MessageKind::covariance; decode_rtcm_cov_msg(v,f); break;
    default: return false;
    }
    return validate_packet(p);
}

void StreamDecoder::feed(uint8_t byte, const Callback &callback) {
    if (!buffer_.empty() && buffer_[0]=='#' && (byte=='#' || byte==0xd3)) {
        ++parse_failures; buffer_.clear();
    }
    buffer_.push_back(static_cast<char>(byte));
    while (!buffer_.empty()) {
        if (buffer_[0]=='#') {
            auto end=buffer_.find_first_of("\r\n");
            if (end==std::string::npos) {
                if (buffer_.size()<MAX_BUF_LEN) return;
                ++parse_failures; buffer_.erase(0,1); continue;
            }
            auto frame=buffer_.substr(0,end); buffer_.erase(0,end+1);
            DecodedPacket packet{};
            if (!ascii_checksum(frame)) ++checksum_failures;
            else if (!decode_ascii_frame(frame,packet)) ++parse_failures;
            else callback(packet);
        } else if (static_cast<uint8_t>(buffer_[0])==0xd3) {
            if (buffer_.size()<3) return;
            if ((static_cast<uint8_t>(buffer_[1])&0xfc)!=0) {
                ++parse_failures; buffer_.erase(0,1); continue;
            }
            size_t len=((static_cast<uint8_t>(buffer_[1])&3)<<8) |
                static_cast<uint8_t>(buffer_[2]);
            if (len<2) { ++parse_failures; buffer_.erase(0,1); continue; }
            if (buffer_.size()<len+6) return;
            a1buff_t f{}; f.nlen=static_cast<int>(len)+3;
            std::memcpy(f.buf,buffer_.data(),len+6);
            auto crc=getbitu(f.buf,static_cast<int>((len+3)*8),24);
            if (crc24q(f.buf,static_cast<int>(len+3))!=crc) {
                ++checksum_failures; buffer_.erase(0,1); continue;
            }
            f.type=getbitu(f.buf,24,12); f.subtype=getbitu(f.buf,36,4);
            buffer_.erase(0,len+6);
            DecodedPacket packet{};
            if (decode_binary_frame(f,packet)) callback(packet);
            else ++parse_failures;
        } else buffer_.erase(0,1);
    }
}
}  // namespace anello
