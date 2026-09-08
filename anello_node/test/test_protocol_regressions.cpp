#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <random>
#include "../src/anello_ros_driver/messaging/protocol_decoder.h"
#include "../src/anello_ros_driver/navigation_math.h"
#include "../src/anello_ros_driver/sample_state.h"
#include "../src/anello_ros_driver/clock_translator.h"
#include "../src/anello_ros_driver/device_input.h"
#include "../src/anello_ros_driver/rate_monitor.h"
using namespace anello;
namespace {
std::string ascii(const std::string &body) {
    return "#"+body+"*"+compute_checksum(body.data(),body.size());
}
std::vector<DecodedPacket> decode(const std::string &bytes, StreamDecoder &parser) {
    std::vector<DecodedPacket> out;
    for (unsigned char c:bytes) parser.feed(c,[&](const auto &p){out.push_back(p);});
    return out;
}
template<typename T> std::string binary(int subtype, const T &payload, int extra=0) {
    std::string bytes(sizeof(T)+8+extra,'\0');
    auto *data=reinterpret_cast<unsigned char *>(bytes.data());
    data[0]=0xd3;
    setbitu(data,14,10,sizeof(T)+2+extra); setbitu(data,24,12,4058); setbitu(data,36,4,subtype);
    std::memcpy(data+5,&payload,sizeof(T));
    setbitu(data,(bytes.size()-3)*8,24,crc24q(data,bytes.size()-3));
    return bytes;
}
const std::string imu="APIMU,1000,0,0,0,-1,1,2,3,4,0,999,20";
std::string correction(size_t payload=19, uint16_t type=1005) {
    std::string frame(payload+6,'\0');
    auto *p=reinterpret_cast<unsigned char *>(frame.data());
    p[0]=0xd3; setbitu(p,14,10,payload); setbitu(p,24,12,type);
    setbitu(p,(frame.size()-3)*8,24,crc24q(p,frame.size()-3));
    return frame;
}
}

TEST(DeviceInput, RtcmValidatesWholeBundlesBeforeAnyTransmission) {
    const auto valid=[](const std::string &s) {
        return valid_rtcm_input(reinterpret_cast<const uint8_t *>(s.data()),s.size());
    };
    const auto frame=correction();
    EXPECT_TRUE(valid(frame)); EXPECT_TRUE(valid(frame+correction(21,1006)));
    size_t count=0;
    const auto bundle=frame+frame;
    ASSERT_TRUE(valid_rtcm_input(reinterpret_cast<const uint8_t *>(bundle.data()),bundle.size(),&count));
    EXPECT_EQ(count,2u);
    for (size_t length=0;length<frame.size();++length) EXPECT_FALSE(valid(frame.substr(0,length)));
    auto corrupt=frame; corrupt.back()^=1;
    EXPECT_FALSE(valid(frame+corrupt)); EXPECT_FALSE(valid(frame+"#APRST,0*58\r\n"));
    corrupt=frame; corrupt[1]|=0xfc; EXPECT_FALSE(valid(corrupt));
    EXPECT_FALSE(valid(correction(1))); EXPECT_FALSE(valid(correction(19,4058)));
    EXPECT_FALSE(valid(correction(19,0))); EXPECT_FALSE(valid(std::string(4097,'x')));
    EXPECT_FALSE(valid_rtcm_input(nullptr,8));
}
TEST(DeviceInput, QueriesCannotSmuggleConfigurationOrResetCommands) {
    for (const auto body:{"APPNG","APVER","APVEH,R,bsl","APCFG,r,odr,mfm","APECH,hello"})
        EXPECT_TRUE(read_only_command(body))<<body;
    for (const auto body:{"AP","APVER,0","APCFG,W,odr,100","APVEH,w,bsl,1","APRST,0", "APUNKNOWN",
                         "APCFG,R,","APCFG,R,bsl,","APCFG,R,,bsl","APCFG,R,bsl;APRST", "APPNG\r\nAPRST,0"})
        EXPECT_FALSE(read_only_command(body))<<body;
    EXPECT_FALSE(valid_command_body(std::string("APECH,")+std::string(123,'x')));
    EXPECT_FALSE(valid_command_body(std::string("APECH,\0x",8)));
}
TEST(DeviceInput, TrafficBudgetBoundsBurstsWithoutQueuing) {
    TrafficBudget bytes(100,50);
    auto now=TrafficBudget::Clock::time_point{};
    EXPECT_TRUE(bytes.take(50,now)); EXPECT_FALSE(bytes.take(1,now));
    EXPECT_FALSE(bytes.take(51,now+std::chrono::seconds(1)));
    EXPECT_TRUE(bytes.take(50,now+std::chrono::seconds(1)));
    EXPECT_FALSE(bytes.take(1,now+std::chrono::seconds(1)));
    TrafficBudget commands(2,1);
    EXPECT_TRUE(commands.take(1,now));
    EXPECT_FALSE(commands.take(1,now+std::chrono::milliseconds(499)));
    EXPECT_TRUE(commands.take(1,now+std::chrono::milliseconds(501)));
}
TEST(DeviceInput, ArbitraryInputLengthsAreBounded) {
    std::mt19937 random(0xdec0de);
    std::vector<uint8_t> bytes;
    for (size_t length=0;length<=4097;++length) {
        bytes.push_back(static_cast<uint8_t>(random()));
        (void)valid_rtcm_input(bytes.data(),length);
    }
}
TEST(FullProtocol, ValidModernLegacyAndFragmented) {
    StreamDecoder parser;
    const auto line=ascii(imu)+"\r\n";
    EXPECT_TRUE(decode(line.substr(0,7),parser).empty());
    auto out=decode(line.substr(7),parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::imu);
    EXPECT_DOUBLE_EQ(out[0].values[3],-1); EXPECT_DOUBLE_EQ(out[0].values[9],0.999);
    out=decode(ascii("APIMU,1000,0,0,-1,1,2,3,4,0,999,20")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_DOUBLE_EQ(out[0].values[3],-1);
}
TEST(FullProtocol, RejectsMalformedNumbersAndUnsupportedLayouts) {
    for (const auto &bad:{"nan","inf","-inf","1e309","12junk","", " 1","0x2"}) {
        auto body=imu; body.replace(body.find("1000"),4,bad);
        StreamDecoder parser; EXPECT_TRUE(decode(ascii(body)+"\r\n",parser).empty())<<bad;
    }
    for (const auto &body:{"APIMU,1000,0,0,-1,1,2,3,4,0", "APIMU,1000,0,0,0,-1,1,2,3,4,0,999,20,42",
                          "XXAPIMU,1000,0,0,0,-1,1,2,3,4,0,999,20"}) {
        StreamDecoder parser; EXPECT_TRUE(decode(ascii(body)+"\r\n",parser).empty());
    }
    StreamDecoder parser;
    EXPECT_TRUE(decode(ascii("APIMU,1000,0,nan,0,-1,1,2,3,4,0,999,20")+"\r\n",parser).empty());
}
TEST(FullProtocol, RejectsRepeatedNumericSigns) {
    for (const auto field:{"+-1","++1","--1","-+1","1e+-1"}) {
        StreamDecoder parser;
        const auto body=std::string("APIMU,1000,0,0,0,")+field+",1,2,3,4,0,999,20";
        EXPECT_TRUE(decode(ascii(body)+"\r\n",parser).empty())<<field;
    }
    StreamDecoder parser;
    EXPECT_EQ(decode(ascii("APIMU,+1000,0,0,0,-1,1e+0,2,3,4,0,999,20")+"\r\n",parser).size(),1u);
}
TEST(FullProtocol, StatusAndOptionalVelocity) {
    for (const auto &status:{"255","1.5","-1"}) {
        StreamDecoder parser;
        EXPECT_TRUE(decode(ascii(std::string("APINS,1000,0,")+status+",37,-122,0,0,0,0,0,0,0,0")+"\r\n",parser).empty());
    }
    StreamDecoder parser;
    auto out=decode(ascii("APINS,1000,0,1,37,-122,0,,,,0,0,0,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_TRUE(std::isnan(out[0].values[6]));
    EXPECT_TRUE(ins_position_valid(out[0].values.data()));
    out=decode(ascii("APINS,1000,0,8,,,,,,,0,0,0,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_FALSE(ins_position_valid(out[0].values.data()));
    EXPECT_TRUE(decode(ascii("APINS,1000,0,1,91,-122,0,0,0,0,0,0,0,0")+"\r\n",parser).empty());
}
TEST(FullProtocol, ResyncAndErrorAccounting) {
    StreamDecoder parser;
    auto bad=ascii(imu); bad.back()=bad.back()=='0'?'1':'0';
    auto out=decode("junk#APbroken"+bad+"\r\n"+ascii(imu)+"\r\n",parser);
    EXPECT_EQ(out.size(),1u); EXPECT_EQ(parser.checksum_failures,1u); EXPECT_EQ(parser.parse_failures,1u);
    std::string reserved="\xd3\xfc\x01";
    out=decode(reserved+ascii(imu)+"\r\n",parser);
    EXPECT_EQ(out.size(),1u); EXPECT_EQ(parser.parse_failures,2u);
    EXPECT_EQ(decode(std::string(5000,'x')+ascii(imu)+"\r\n",parser).size(),1u);
}
TEST(FullProtocol, BinaryLengthCrcAndCovariance) {
    rtcm_apins_t ins{}; ins.Status=4; ins.Latitude=370000000; ins.Longitude=-1220000000;
    StreamDecoder parser;
    EXPECT_EQ(decode(binary(4,ins),parser).size(),1u);
    EXPECT_TRUE(decode(binary(4,ins,1),parser).empty());
    auto corrupt=binary(4,ins); corrupt.back()^=1;
    EXPECT_TRUE(decode(corrupt,parser).empty());
    EXPECT_EQ(parser.checksum_failures,1u);
    rtcm_apcov_t cov{}; cov.covLatLat=std::nanf("");
    EXPECT_TRUE(decode(binary(10,cov),parser).empty());
    cov.covLatLat=1; cov.covLonLon=1; cov.covLatLon=2;
    EXPECT_TRUE(decode(binary(10,cov),parser).empty());
    cov.covLatLon=0.25f;
    EXPECT_EQ(decode(binary(10,cov),parser).size(),1u);
}
TEST(FullProtocol, AhrsAndIm1) {
    StreamDecoder parser;
    auto out=decode(ascii("APAHRS,1000,1000000000,1,2,3,0")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::ahrs);
    rtcm_apahrs_t a{1000000000,1000000000,100000,200000,300000,0};
    out=decode(binary(8,a),parser); ASSERT_EQ(out.size(),1u);
    EXPECT_DOUBLE_EQ(out[0].values[2],1);
    out=decode(ascii("APIM1,1000,0,0,0,-1,1,2,3,4,20")+"\r\n",parser);
    ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].kind,MessageKind::im1);
}
TEST(FullProtocol, DeterministicNoiseAndBoundedRecovery) {
    std::mt19937 generator(42);
    StreamDecoder parser;
    for (int i=0;i<100000;++i) parser.feed(generator()%256,[](const auto &p){EXPECT_TRUE(validate_packet(p));});
    parser.reset();
    EXPECT_EQ(decode(ascii(imu)+"\r\n",parser).size(),1u);
}
TEST(NavigationMath, EquatorHeightAndAntimeridian) {
    LocalCartesian local; local.set_origin(0,0,10);
    auto p=local.position(0,0,15); EXPECT_NEAR(p.z(),5,1e-8); EXPECT_NEAR(p.length(),5,1e-8);
    local.set_origin(0,179.999,0); p=local.position(0,-179.999,0);
    EXPECT_NEAR(p.x(),222.63898154,1e-5); EXPECT_NEAR(p.y(),0,1e-8);
}
TEST(NavigationMath, CurvatureAndCurrentAxes) {
    LocalCartesian local; local.set_origin(37,-122,0);
    const double lon=-122+1000/89011.67172648299;
    auto p=local.position(37,lon,0);
    EXPECT_NEAR(p.x(),999.999994,0.001); EXPECT_NEAR(p.y(),0.059002,1e-5); EXPECT_NEAR(p.z(),-0.078298,1e-5);
    auto rotation=local.rotation(37,lon);
    EXPECT_NEAR(rotation.determinant(),1,1e-12);
    EXPECT_GT(std::abs(rotation[2][0]),1e-5);
}
TEST(NavigationMath, EulerCovarianceUsesAttitudeJacobian) {
    tf2::Matrix3x3 cov(1,0,0,0,2,0,0,0,3);
    auto out=euler_covariance_to_fixed(cov,0,3.14159265358979323846/2);
    EXPECT_NEAR(out[0][0],2,1e-12); EXPECT_NEAR(out[1][1],1,1e-12);
    out=euler_covariance_to_fixed(cov,0.5,0);
    EXPECT_NEAR(out[0][2],-std::cos(0.5)*std::sin(0.5),1e-12);
}
TEST(SampleState, RequiresBothAcquisitionAndArrivalFreshness) {
    SampleTime sample; const auto now=SteadyClock::now();
    EXPECT_FALSE(sample.matches(1000,0.1,now)); sample.set(1000,now);
    EXPECT_TRUE(sample.matches(1050,0.1,now)); EXPECT_FALSE(sample.matches(1200,0.1,now));
    EXPECT_FALSE(sample.matches(1000,0.1,now+std::chrono::seconds(1)));
}
TEST(SampleState, HostDeviceAndSimulationClockResets) {
    ClockDiscontinuity clock;
    EXPECT_FALSE(clock.update(5000,10000000000LL,0,false));
    EXPECT_FALSE(clock.update(5010,10010000000LL,10000000,false));
    EXPECT_TRUE(clock.update(5020,110020000000LL,20000000,false));
    EXPECT_TRUE(clock.update(5030,9000000000LL,30000000,false));
    EXPECT_TRUE(clock.update(1,9010000000LL,40000000,false));
    EXPECT_TRUE(clock.update(11,9020000000LL,50000000,true));
    EXPECT_FALSE(clock.update(21,9020000000LL,60000000,true));
    EXPECT_FALSE(clock.update(1021,9020000000LL,1060000000LL,true));
}
TEST(SampleState, IntegerHostEpochPreservesNanoseconds) {
    ClockTranslator clock;
    constexpr int64_t epoch=1788800000000000017LL;
    for (int i=0;i<100;++i) clock.update_ns(i*0.01,epoch+i*10000000LL);
    EXPECT_TRUE(clock.ready()); EXPECT_EQ(clock.translate_ns(0.99),epoch+990000000LL);
}
TEST(SampleState, ExtremeClockDifferencesDoNotOverflow) {
    const auto minimum=std::numeric_limits<int64_t>::min();
    const auto maximum=std::numeric_limits<int64_t>::max();
    ClockDiscontinuity changes;
    EXPECT_FALSE(changes.update(0,minimum,minimum,false));
    // The forward ROS and steady steps agree, even across the signed range.
    EXPECT_FALSE(changes.update(1,maximum,maximum,false));
    EXPECT_TRUE(changes.update(2,minimum,maximum,false));
    ClockTranslator clock;
    clock.update_ns(0,minimum);
    clock.update_ns(1,maximum);
    EXPECT_EQ(clock.translate_ns(1),minimum+1000200000LL);
}
TEST(SampleState, UnrepresentableTranslationsAreUnavailable) {
    ClockTranslator clock;
    clock.update_ns(0,std::numeric_limits<int64_t>::max());
    EXPECT_EQ(clock.translate_ns(0),std::numeric_limits<int64_t>::max());
    EXPECT_FALSE(clock.translate_ns(1));
    EXPECT_FALSE(clock.translate_ns(INFINITY));
    EXPECT_FALSE(clock.translate_ns(NAN));
    clock.reset();
    EXPECT_FALSE(clock.translate_ns(0));
}
TEST(SampleState, NonfiniteUpdatesCannotPoisonClockWarmup) {
    ClockTranslator clock;
    for (int i=0;i<100;++i) clock.update_ns(i*0.01,10000000000LL+i*10000000LL);
    ASSERT_TRUE(clock.ready());
    clock.update_ns(NAN,0);
    clock.update_ns(INFINITY,0);
    clock.update_ns(-INFINITY,0);
    clock.update(1,INFINITY);
    clock.update(1,-INFINITY);
    EXPECT_TRUE(clock.ready());
    EXPECT_EQ(clock.translate_ns(1),11000000000LL);
}
TEST(RateMonitor, LargeBatchesRetainCountsAndExpire) {
    RateMonitor monitor;
    const auto now=RateMonitor::Clock::time_point{};
    monitor.add_ok(1000000,now);
    monitor.add_parse_fail(2000000,now);
    monitor.add_checksum_fail(1000000,now);
    EXPECT_DOUBLE_EQ(monitor.rate_hz(now),200000);
    EXPECT_DOUBLE_EQ(monitor.error_percent(now),75);
    const auto expired=now+std::chrono::seconds(5);
    EXPECT_DOUBLE_EQ(monitor.rate_hz(expired),0);
    EXPECT_DOUBLE_EQ(monitor.error_percent(expired),0);
    EXPECT_EQ(monitor.total_ok,1000000u);
    EXPECT_EQ(monitor.total_parse_fail,2000000u);
    EXPECT_EQ(monitor.total_checksum_fail,1000000u);
}
TEST(RateMonitor, WindowRollsAcrossManyBucketReuses) {
    RateMonitor monitor;
    const auto start=RateMonitor::Clock::time_point{};
    for (int i=0;i<500;++i) {
        auto now=start+std::chrono::milliseconds(100*i);
        monitor.add_ok(1,now);
        monitor.add_checksum_fail(1,now);
        EXPECT_DOUBLE_EQ(monitor.rate_hz(now),std::min(i+1,50)/5.0);
        EXPECT_DOUBLE_EQ(monitor.error_percent(now),50);
    }
    auto later=start+std::chrono::seconds(70);
    monitor.add_ok(10,later);
    EXPECT_DOUBLE_EQ(monitor.rate_hz(later),2);
    EXPECT_DOUBLE_EQ(monitor.error_percent(later),0);
}
TEST(RateMonitor, ClockRewindCannotResurrectOldBuckets) {
    RateMonitor monitor;
    const auto start=RateMonitor::Clock::time_point{};
    monitor.add_ok(10,start+std::chrono::seconds(100));
    EXPECT_DOUBLE_EQ(monitor.rate_hz(start),0);
    monitor.add_parse_fail(10,start);
    EXPECT_DOUBLE_EQ(monitor.error_percent(start),100);
    EXPECT_DOUBLE_EQ(monitor.rate_hz(start+std::chrono::seconds(100)),0);
    EXPECT_EQ(monitor.total_ok,10u);
}
